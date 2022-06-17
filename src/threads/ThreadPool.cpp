#include "gravity/threads/ThreadPool.h"

namespace gravity::threads
{
    ThreadPool::ThreadPool(unsigned int const threads)
    {
        if (threads == 0U)
        {
            throw std::invalid_argument("ThreadPool: cannot instantiate zero threads.");
        }

        try
        {
            for (auto i = 0U; i < threads; ++i)
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

        return std::max(std::thread::hardware_concurrency(), 2U) - 1U;
    }

    void ThreadPool::Submit(std::shared_ptr<ITask> const& task)
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
                task->Execute();
                task = nullptr;
            }
        }
    }

    void ThreadPool::JoinThreads()
    {
        queue_.Closed(true);

        for (auto& thread : threads_)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

    void ThreadPool::CheckForErrors(futures_t const& futures)
    {
        except::ErrorList errors;

        for (auto& future : futures)
        {
            if (!future.valid())
            {
                errors << "Invalid future";
                continue;
            }

            try
            {
                future.get();
            }
            catch (std::exception const& e)
            {
                errors << e.what();
            }
            catch (...)
            {
                errors << "Non-standard exception caught";
            }
        }

        if (!errors.Empty())
        {
            throw std::runtime_error(errors.Message());
        }
    }
}
