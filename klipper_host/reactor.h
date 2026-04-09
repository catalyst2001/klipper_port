// Reactor - event loop with timers, async callbacks, and completions
// C++ port of Klipper's klippy/reactor.py
//
// Design notes:
//   Python Klipper uses greenlets for cooperative multitasking within the
//   reactor thread.  This C++ port uses minicoro (stackful coroutines) to
//   provide the same pause/resume semantics.  Each timer callback runs in
//   its own coroutine.  pause() yields back to the dispatch loop via
//   mco_yield(); the dispatch loop resumes the coroutine when its waketime
//   arrives via mco_resume().
//
//   Key API equivalences:
//     Python                          C++
//     reactor.register_timer(cb,wt)   reactor.registerTimer(cb, wt)
//     reactor.update_timer(t,wt)      reactor.updateTimer(t, wt)
//     reactor.unregister_timer(t)     reactor.unregisterTimer(t)
//     reactor.pause(waketime)         reactor.pause(waketime) [yields coro]
//     reactor.register_callback(cb)   reactor.registerCallback(cb)
//     reactor.register_async_callback reactor.registerAsyncCallback(cb, wt)
//     reactor.async_complete(c,r)     reactor.asyncComplete(c, r)
//     reactor.completion()            reactor.completion()
//     completion.wait()               completion->wait()
//     completion.complete(result)     completion->complete(result)
//     reactor.mutex()                 reactor.mutex()
//     reactor.monotonic()             reactor.monotonic()
//     reactor.run() / reactor.end()   reactor.run() / reactor.end()

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// Forward-declare minicoro coroutine type (definition in minicoro.h)
struct mco_coro;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

constexpr double REACTOR_NOW   = 0.0;
constexpr double REACTOR_NEVER = 9999999999999999.0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

class Reactor;
class ReactorCompletion;
class ReactorMutex;

using ReactorCompletionPtr = std::shared_ptr<ReactorCompletion>;

// ---------------------------------------------------------------------------
// ReactorTimer
// ---------------------------------------------------------------------------
// Equivalent to Python's ReactorTimer.
// callback(eventtime) → next waketime.  Return REACTOR_NEVER to stop.
// The timer's callback runs inside a minicoro coroutine, so it may call
// reactor.pause() to suspend mid-execution.

class ReactorTimer {
public:
    using Callback = std::function<double(double eventtime)>;

    ReactorTimer(Callback cb, double waketime);
    ~ReactorTimer();

    double getWaketime() const { return m_waketime; }
    bool   isRunning() const { return m_timerIsRunning; }

private:
    friend class Reactor;
    friend class ReactorCompletion;
    friend class ReactorMutex;

    Callback  m_callback;
    double    m_waketime;
    bool      m_timerIsRunning = false;

    // Coroutine executing this timer's callback
    mco_coro* m_coro            = nullptr;
    double    m_coroEventtime   = 0.0;           // eventtime for callback
    double    m_coroReturnWake  = REACTOR_NEVER;  // waketime returned by callback
};

using ReactorTimerPtr = std::shared_ptr<ReactorTimer>;

// ---------------------------------------------------------------------------
// ReactorCompletion
// ---------------------------------------------------------------------------
// Equivalent to Python's ReactorCompletion.
// wait() pauses the calling coroutine.  complete() wakes all waiters.
// Cross-thread wait uses a condition variable fallback.

class ReactorCompletion {
public:
    explicit ReactorCompletion(Reactor& reactor);

    bool   test() const;
    void   complete(double result);
    double wait(double waketime = REACTOR_NEVER, double waketimeResult = 0.0);
    double getResult() const { return m_result; }

private:
    Reactor&                     m_reactor;
    double                       m_result    = 0.0;
    bool                         m_completed = false;
    std::vector<ReactorTimerPtr> m_waiting;       // timers paused in wait()
    // Cross-thread fallback
    std::mutex                   m_cvMutex;
    std::condition_variable      m_cv;
};

// ---------------------------------------------------------------------------
// ReactorMutex
// ---------------------------------------------------------------------------
// Equivalent to Python's ReactorMutex.
// lock()/unlock() pause the calling coroutine when contended.

class ReactorMutex {
public:
    explicit ReactorMutex(Reactor& reactor, bool isLocked = false);

    void lock();
    void unlock();
    bool test() const { return m_isLocked; }

    struct Guard {
        ReactorMutex& mtx;
        Guard(ReactorMutex& m) : mtx(m) { mtx.lock(); }
        ~Guard() { mtx.unlock(); }
    };
    Guard guard() { return Guard{*this}; }

private:
    Reactor&                     m_reactor;
    bool                         m_isLocked;
    bool                         m_nextPending = false;
    std::deque<ReactorTimerPtr>  m_queue;
};

// ---------------------------------------------------------------------------
// Reactor
// ---------------------------------------------------------------------------
// Main event loop.  Manages timers, async callbacks, completions.
//
// Threading model:
//   - run() executes the dispatch loop on the calling thread.
//   - Timer callbacks run in per-timer coroutines on the reactor thread.
//   - pause() yields the current coroutine back to the dispatch loop.
//   - registerAsyncCallback() / asyncComplete() are thread-safe.
//   - Callbacks fire sequentially (no parallelism).

class Reactor {
public:
    static constexpr double NOW   = REACTOR_NOW;
    static constexpr double NEVER = REACTOR_NEVER;

    Reactor();
    ~Reactor();

    // ---- Monotonic time ---------------------------------------------------
    double monotonic() const;

    // ---- Timers -----------------------------------------------------------
    ReactorTimerPtr registerTimer(ReactorTimer::Callback callback,
                                  double waketime = NEVER);
    void updateTimer(ReactorTimerPtr timer, double waketime);
    void unregisterTimer(ReactorTimerPtr timer);

    // ---- Callbacks --------------------------------------------------------
    ReactorCompletionPtr registerCallback(
        std::function<double(double)> callback,
        double waketime = NOW);

    void registerAsyncCallback(
        std::function<double(double)> callback,
        double waketime = NOW);

    void asyncComplete(ReactorCompletionPtr completion, double result);

    // ---- Completions / Mutexes --------------------------------------------
    ReactorCompletionPtr completion();
    std::shared_ptr<ReactorMutex> mutex(bool isLocked = false);

    // ---- Pause ------------------------------------------------------------
    // Yields the current coroutine until waketime.  Returns actual eventtime.
    // If not in a coroutine (reactor not running), falls back to system sleep.
    double pause(double waketime);

    // ---- Main loop --------------------------------------------------------
    void run();
    void end();
    void finalize();
    bool isRunning() const { return m_process.load(std::memory_order_acquire); }

    // Timer currently being dispatched (used by Completion/Mutex)
    ReactorTimerPtr currentTimer() const { return m_currentTimer; }

private:
    double checkTimers(double eventtime, bool busy);
    void   drainAsyncQueue();
    void   wakeDispatch();
    double sysPause(double waketime);

    static void coroEntry(mco_coro* co);

    // ---- Timers -----------------------------------------------------------
    std::vector<ReactorTimerPtr> m_timers;
    double                       m_nextTimer = NEVER;
    ReactorTimerPtr              m_currentTimer;

    // ---- Async queue (thread-safe injection) ------------------------------
    struct AsyncEntry { std::function<void()> func; };
    std::mutex              m_asyncMutex;
    std::deque<AsyncEntry>  m_asyncQueue;

    // ---- Dispatch loop ----------------------------------------------------
    std::atomic<bool>       m_process{false};
    std::thread::id         m_dispatchThread;
    std::mutex              m_dispatchMutex;
    std::condition_variable m_dispatchCv;

    // ---- Time base --------------------------------------------------------
    using Clock = std::chrono::steady_clock;
    Clock::time_point       m_epoch;
};
