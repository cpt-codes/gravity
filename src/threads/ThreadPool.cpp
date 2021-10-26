#include "gravity/threads/ThreadPool.h"

namespace gravity::threads
{
    ThreadPool::ThreadPool(unsigned int const threads)
    {
        if (threads == 0u)
        {
            throw std::invalid_argument("ThreadPool: cannot instantiate zero threads.");
        }

        try
        {
            for (auto i = 0u; i < threads; ++i)
            {
                threads_.emplace_back(&ThreadPool::Worker, this);
            }
        }
        catch(...) // regardless of exception we must stop the threads
        {
            JoinThreads();
            throw;
        }
    }

    unsigned int ThreadPool::HardwareConcurrency()
    {
        // hardware concurrency can be zero if it is not computable. Therefore,
        // we must check and protect against overflow of the unsigned int.

        return std::max(std::thread::hardware_concurrency(), 2u) - 1u;
    }

    void ThreadPool::Submit(const std::shared_ptr<ITask>& task)
    {
        queue_.Push(task);
    }

    ThreadPool::~ThreadPool()
    {
        JoinThreads();
    }

    void ThreadPool::Worker()
    {
        std::shared_ptr<ITask> task;

        while (!queue_.Closed())
        {
            if (queue_.Pop(task, true))
            {
                active_++;
                task->Execute();
                task = nullptr;
                active_--;
            }
        }
    }

    void ThreadPool::JoinThreads()
    {
        queue_.Closed() = true;

        for (auto& thread : threads_)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }
}
