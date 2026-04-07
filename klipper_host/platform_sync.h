#pragma once

// Platform-abstracted synchronization primitives.
// Win32 implementation (SRWLock + CONDITION_VARIABLE).
// POSIX implementation (pthread_mutex + pthread_cond) in #else branch.

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#else
#include <pthread.h>
#include <time.h>
#include <errno.h>
#endif

#include <cstdint>

// Non-copyable, non-movable base
class NonCopyable {
protected:
    NonCopyable() = default;
    ~NonCopyable() = default;
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
};

// ---- Mutex (non-recursive) ----
// Win32: SRWLOCK (slim reader/writer lock, exclusive mode)
// POSIX: pthread_mutex_t (PTHREAD_MUTEX_NORMAL)

class PlatformMutex : NonCopyable {
public:
#ifdef _WIN32
    PlatformMutex()  { InitializeSRWLock(&m_lock); }
    ~PlatformMutex() { /* SRWLock has no destroy */ }
    void lock()      { AcquireSRWLockExclusive(&m_lock); }
    void unlock()    { ReleaseSRWLockExclusive(&m_lock); }

    // Raw handle for CondVar (Win32 needs the SRWLOCK*, not a generic type)
    SRWLOCK* nativeHandle() { return &m_lock; }

private:
    SRWLOCK m_lock;
#else
    PlatformMutex()  { pthread_mutex_init(&m_lock, nullptr); }
    ~PlatformMutex() { pthread_mutex_destroy(&m_lock); }
    void lock()      { pthread_mutex_lock(&m_lock); }
    void unlock()    { pthread_mutex_unlock(&m_lock); }

    pthread_mutex_t* nativeHandle() { return &m_lock; }

private:
    pthread_mutex_t m_lock;
#endif
};

// RAII lock guard for PlatformMutex
class PlatformLockGuard : NonCopyable {
public:
    explicit PlatformLockGuard(PlatformMutex& m) : m_mutex(m) { m_mutex.lock(); }
    ~PlatformLockGuard() { m_mutex.unlock(); }
private:
    PlatformMutex& m_mutex;
};

// ---- Condition Variable ----
// Win32: CONDITION_VARIABLE + SRWLock
// POSIX: pthread_cond_t + pthread_mutex_t

class PlatformCondVar : NonCopyable {
public:
#ifdef _WIN32
    PlatformCondVar()  { InitializeConditionVariable(&m_cond); }
    ~PlatformCondVar() { /* CONDITION_VARIABLE has no destroy */ }

    // Wait indefinitely. Mutex must be locked by caller.
    void wait(PlatformMutex& mtx) {
        SleepConditionVariableSRW(&m_cond, mtx.nativeHandle(), INFINITE, 0);
    }

    // Wait with timeout in milliseconds. Returns true if signaled, false on timeout.
    bool waitFor(PlatformMutex& mtx, uint32_t timeoutMs) {
        return SleepConditionVariableSRW(&m_cond, mtx.nativeHandle(),
                                         timeoutMs, 0) != 0;
    }

    void notifyOne() { WakeConditionVariable(&m_cond); }
    void notifyAll() { WakeAllConditionVariable(&m_cond); }

private:
    CONDITION_VARIABLE m_cond;
#else
    PlatformCondVar()  { pthread_cond_init(&m_cond, nullptr); }
    ~PlatformCondVar() { pthread_cond_destroy(&m_cond); }

    void wait(PlatformMutex& mtx) {
        pthread_cond_wait(&m_cond, mtx.nativeHandle());
    }

    bool waitFor(PlatformMutex& mtx, uint32_t timeoutMs) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += timeoutMs / 1000;
        ts.tv_nsec += (timeoutMs % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += 1;
            ts.tv_nsec -= 1000000000L;
        }
        return pthread_cond_timedwait(&m_cond, mtx.nativeHandle(), &ts) == 0;
    }

    void notifyOne() { pthread_cond_signal(&m_cond); }
    void notifyAll() { pthread_cond_broadcast(&m_cond); }

private:
    pthread_cond_t m_cond;
#endif
};

// ---- Thread wrapper ----
// Owns a single joinable thread. Start with a callable, join on destroy.

#ifdef _WIN32
#include <process.h>

class PlatformThread : NonCopyable {
public:
    using ThreadFunc = unsigned (__stdcall *)(void*);

    PlatformThread() = default;
    ~PlatformThread() { join(); }

    // Start thread. func receives arg. Returns true on success.
    bool start(ThreadFunc func, void* arg) {
        if (m_handle) return false;
        m_handle = reinterpret_cast<HANDLE>(
            _beginthreadex(nullptr, 0, func, arg, 0, nullptr));
        return m_handle != nullptr;
    }

    void join() {
        if (m_handle) {
            WaitForSingleObject(m_handle, INFINITE);
            CloseHandle(m_handle);
            m_handle = nullptr;
        }
    }

    bool isRunning() const { return m_handle != nullptr; }

private:
    HANDLE m_handle = nullptr;
};

#else

class PlatformThread : NonCopyable {
public:
    using ThreadFunc = void* (*)(void*);

    PlatformThread() = default;
    ~PlatformThread() { join(); }

    bool start(ThreadFunc func, void* arg) {
        if (m_started) return false;
        m_started = (pthread_create(&m_thread, nullptr, func, arg) == 0);
        return m_started;
    }

    void join() {
        if (m_started) {
            pthread_join(m_thread, nullptr);
            m_started = false;
        }
    }

    bool isRunning() const { return m_started; }

private:
    pthread_t m_thread{};
    bool m_started = false;
};

#endif

// ---- MonotonicClock ----
// High-resolution monotonic clock returning seconds as double.

class MonotonicClock {
public:
#ifdef _WIN32
    static double now() {
        LARGE_INTEGER cnt;
        QueryPerformanceCounter(&cnt);
        return static_cast<double>(cnt.QuadPart) * invFreq();
    }
private:
    static double invFreq() {
        static double inv = [] {
            LARGE_INTEGER freq;
            QueryPerformanceFrequency(&freq);
            return 1.0 / static_cast<double>(freq.QuadPart);
        }();
        return inv;
    }
#else
    static double now() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + ts.tv_nsec * 1e-9;
    }
#endif
};
