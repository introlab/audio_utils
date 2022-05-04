#ifndef UTILS_SEMAPHORE_H
#define UTILS_SEMAPHORE_H

#include <condition_variable>
#include <mutex>

class Semaphore
{
    int m_count;
    std::mutex m_mutex;
    std::condition_variable m_conditionVariable;

public:
    Semaphore(int count);

    void release();
    void acquire();
    bool tryAcquire();
    template<class Rep, class Period>
    bool tryAcquireFor(const std::chrono::duration<Rep, Period>& duration);
};

inline void Semaphore::release()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_count++;
    m_conditionVariable.notify_one();
}

inline void Semaphore::acquire()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_conditionVariable.wait(lock, [this]() { return m_count > 0; });
    m_count--;
}

inline bool Semaphore::tryAcquire()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_count > 0)
    {
        m_count--;
        return true;
    }
    else
    {
        return false;
    }
}

template<class Rep, class Period>
inline bool Semaphore::tryAcquireFor(const std::chrono::duration<Rep, Period>& duration)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_conditionVariable.wait_for(lock, duration, [this]() { return m_count > 0; }))
    {
        m_count--;
        return true;
    }
    else
    {
        return false;
    }
}

#endif
