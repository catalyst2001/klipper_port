// Reactor - C++ port of Klipper's klippy/reactor.py
// Uses minicoro for stackful coroutines (replaces Python's greenlets).
// See reactor.h for design notes.

#define MINICORO_IMPL
#include "third_party/minicoro.h"

#include "reactor.h"

#include <algorithm>
#include <cassert>

// ===== ReactorTimer ========================================================

ReactorTimer::ReactorTimer(Callback cb, double waketime)
    : m_callback(std::move(cb))
    , m_waketime(waketime)
{
}

ReactorTimer::~ReactorTimer()
{
    if (m_coro) {
        // Safe to destroy suspended or dead coroutines
        if (mco_status(m_coro) != MCO_RUNNING)
            mco_destroy(m_coro);
        m_coro = nullptr;
    }
}

// ===== ReactorCompletion ===================================================

ReactorCompletion::ReactorCompletion(Reactor& reactor)
    : m_reactor(reactor)
{
}

bool ReactorCompletion::test() const
{
    return m_completed;
}

void ReactorCompletion::complete(double result)
{
    m_result    = result;
    m_completed = true;
    // Wake coroutines paused in wait()
    for (auto& timer : m_waiting) {
        m_reactor.updateTimer(timer, REACTOR_NOW);
    }
    // Wake cross-thread waiters
    m_cv.notify_all();
}

double ReactorCompletion::wait(double waketime, double waketimeResult)
{
    if (m_completed)
        return m_result;

    auto timer = m_reactor.currentTimer();
    if (timer) {
        // Inside a reactor coroutine — use pause
        m_waiting.push_back(timer);
        m_reactor.pause(waketime);
        m_waiting.erase(
            std::remove(m_waiting.begin(), m_waiting.end(), timer),
            m_waiting.end());
        return m_completed ? m_result : waketimeResult;
    }

    // Cross-thread wait using condition variable
    std::unique_lock<std::mutex> lk(m_cvMutex);
    if (waketime >= REACTOR_NEVER) {
        m_cv.wait(lk, [this]() { return m_completed; });
    } else {
        double dt = waketime - m_reactor.monotonic();
        if (dt > 0.0) {
            m_cv.wait_for(lk, std::chrono::duration<double>(dt),
                          [this]() { return m_completed; });
        }
    }
    return m_completed ? m_result : waketimeResult;
}

// ===== ReactorMutex ========================================================

ReactorMutex::ReactorMutex(Reactor& reactor, bool isLocked)
    : m_reactor(reactor)
    , m_isLocked(isLocked)
{
}

void ReactorMutex::lock()
{
    if (!m_isLocked) {
        m_isLocked = true;
        return;
    }
    auto timer = m_reactor.currentTimer();
    assert(timer);
    m_queue.push_back(timer);
    while (true) {
        m_reactor.pause(REACTOR_NEVER);
        if (m_nextPending && m_queue.front() == timer) {
            m_nextPending = false;
            m_queue.pop_front();
            return;
        }
    }
}

void ReactorMutex::unlock()
{
    if (m_queue.empty()) {
        m_isLocked = false;
        return;
    }
    m_nextPending = true;
    m_reactor.updateTimer(m_queue.front(), REACTOR_NOW);
}

// ===== Reactor =============================================================

Reactor::Reactor()
    : m_epoch(Clock::now())
{
}

Reactor::~Reactor()
{
    end();
    finalize();
}

// ---- Monotonic time -------------------------------------------------------

double Reactor::monotonic() const
{
    return std::chrono::duration<double>(Clock::now() - m_epoch).count();
}

// ---- Coroutine entry point ------------------------------------------------
// Each timer callback runs in its own coroutine.
// The entry function invokes the callback and stores the return value.

void Reactor::coroEntry(mco_coro* co)
{
    ReactorTimer* timer = (ReactorTimer*)mco_get_user_data(co);
    timer->m_coroReturnWake = timer->m_callback(timer->m_coroEventtime);
    // Coroutine exits → state becomes MCO_DEAD
}

// ---- Timers ---------------------------------------------------------------

ReactorTimerPtr Reactor::registerTimer(ReactorTimer::Callback callback,
                                       double waketime)
{
    auto timer = std::make_shared<ReactorTimer>(std::move(callback), waketime);
    m_timers.push_back(timer);
    m_nextTimer = std::min(m_nextTimer, waketime);
    return timer;
}

void Reactor::updateTimer(ReactorTimerPtr timer, double waketime)
{
    if (timer->m_timerIsRunning)
        return;  // Can't update while callback is executing
    timer->m_waketime = waketime;
    m_nextTimer = std::min(m_nextTimer, waketime);
    wakeDispatch();
}

void Reactor::unregisterTimer(ReactorTimerPtr timer)
{
    timer->m_waketime = NEVER;
    m_timers.erase(
        std::remove(m_timers.begin(), m_timers.end(), timer),
        m_timers.end());
    // Destroy coroutine if not currently running
    if (timer->m_coro && mco_status(timer->m_coro) != MCO_RUNNING) {
        mco_destroy(timer->m_coro);
        timer->m_coro = nullptr;
    }
}

// ---- Callbacks ------------------------------------------------------------

ReactorCompletionPtr Reactor::registerCallback(
    std::function<double(double)> callback,
    double waketime)
{
    auto comp = std::make_shared<ReactorCompletion>(*this);
    auto cb   = std::move(callback);
    auto compCopy = comp;

    registerTimer([cb, compCopy, this](double eventtime) -> double {
        double result = cb(eventtime);
        compCopy->complete(result);
        return REACTOR_NEVER;
    }, waketime);

    return comp;
}

void Reactor::registerAsyncCallback(
    std::function<double(double)> callback,
    double waketime)
{
    auto cb = std::move(callback);
    auto wt = waketime;
    {
        std::lock_guard<std::mutex> lk(m_asyncMutex);
        m_asyncQueue.push_back({[this, cb, wt]() {
            registerCallback(cb, wt);
        }});
    }
    wakeDispatch();
}

void Reactor::asyncComplete(ReactorCompletionPtr completion, double result)
{
    {
        std::lock_guard<std::mutex> lk(m_asyncMutex);
        m_asyncQueue.push_back({[completion, result]() {
            completion->complete(result);
        }});
    }
    wakeDispatch();
}

// ---- Completions / Mutexes ------------------------------------------------

ReactorCompletionPtr Reactor::completion()
{
    return std::make_shared<ReactorCompletion>(*this);
}

std::shared_ptr<ReactorMutex> Reactor::mutex(bool isLocked)
{
    return std::make_shared<ReactorMutex>(*this, isLocked);
}

// ---- Pause ----------------------------------------------------------------

double Reactor::sysPause(double waketime)
{
    double dt = waketime - monotonic();
    if (dt > 0.0)
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    return monotonic();
}

double Reactor::pause(double waketime)
{
    mco_coro* co = mco_running();
    if (!co) {
        // Not in a coroutine — fall back to system sleep
        return sysPause(waketime);
    }

    // Set this timer's waketime so dispatch loop knows when to resume us
    if (m_currentTimer)
        m_currentTimer->m_waketime = waketime;

    // Yield back to dispatch loop (returns from mco_resume in checkTimers)
    mco_yield(co);

    // Resumed — return the eventtime set by checkTimers before resume
    ReactorTimer* timer = (ReactorTimer*)mco_get_user_data(co);
    return timer->m_coroEventtime;
}

// ---- Main loop ------------------------------------------------------------

double Reactor::checkTimers(double eventtime, bool busy)
{
    if (eventtime < m_nextTimer) {
        if (busy)
            return 0.0;
        return std::min(1.0, std::max(0.001, m_nextTimer - eventtime));
    }

    m_nextTimer = NEVER;

    // Copy timer list — callbacks may register/unregister timers
    auto timers = m_timers;

    for (auto& t : timers) {
        double waketime = t->m_waketime;
        if (eventtime >= waketime) {
            m_currentTimer = t;

            // Set waketime to NEVER before firing (like Python)
            t->m_waketime = NEVER;

            if (t->m_coro && mco_status(t->m_coro) == MCO_SUSPENDED) {
                // Resume paused coroutine
                t->m_coroEventtime = eventtime;
                t->m_timerIsRunning = true;
                mco_resume(t->m_coro);
                t->m_timerIsRunning = false;
            } else {
                // New invocation — create coroutine for this callback
                if (t->m_coro) {
                    mco_destroy(t->m_coro);
                    t->m_coro = nullptr;
                }

                mco_desc desc = mco_desc_init(coroEntry, 256 * 1024);
                desc.user_data = t.get();
                t->m_coroEventtime  = eventtime;
                t->m_coroReturnWake = REACTOR_NEVER;

                mco_result res = mco_create(&t->m_coro, &desc);
                assert(res == MCO_SUCCESS);

                t->m_timerIsRunning = true;
                mco_resume(t->m_coro);
                t->m_timerIsRunning = false;
            }

            // Check how the coroutine exited
            if (t->m_coro && mco_status(t->m_coro) == MCO_DEAD) {
                // Callback returned — set next waketime
                t->m_waketime = t->m_coroReturnWake;
                mco_destroy(t->m_coro);
                t->m_coro = nullptr;
            }
            // If SUSPENDED: callback called pause(), waketime already set

            m_currentTimer = nullptr;
        }

        m_nextTimer = std::min(m_nextTimer, t->m_waketime);
    }

    return 0.0;
}

void Reactor::drainAsyncQueue()
{
    std::deque<AsyncEntry> batch;
    {
        std::lock_guard<std::mutex> lk(m_asyncMutex);
        batch.swap(m_asyncQueue);
    }
    for (auto& entry : batch) {
        entry.func();
    }
}

void Reactor::wakeDispatch()
{
    m_dispatchCv.notify_one();
}

void Reactor::run()
{
    m_dispatchThread = std::this_thread::get_id();
    m_process.store(true, std::memory_order_release);

    bool busy = true;
    double eventtime = monotonic();

    while (m_process.load(std::memory_order_acquire)) {
        drainAsyncQueue();
        double timeout = checkTimers(eventtime, busy);
        busy = false;

        if (m_process.load(std::memory_order_acquire)) {
            if (timeout <= 0.0) {
                busy = true;
            } else {
                std::unique_lock<std::mutex> lk(m_dispatchMutex);
                if (timeout < NEVER) {
                    m_dispatchCv.wait_for(
                        lk, std::chrono::duration<double>(timeout));
                } else {
                    // No timers active — poll with reasonable interval
                    m_dispatchCv.wait_for(lk, std::chrono::milliseconds(100));
                }
            }
        }

        eventtime = monotonic();
    }
}

void Reactor::end()
{
    m_process.store(false, std::memory_order_release);
    wakeDispatch();
}

void Reactor::finalize()
{
    m_currentTimer = nullptr;
    for (auto& t : m_timers) {
        if (t->m_coro) {
            if (mco_status(t->m_coro) != MCO_RUNNING)
                mco_destroy(t->m_coro);
            t->m_coro = nullptr;
        }
    }
    m_timers.clear();
}
