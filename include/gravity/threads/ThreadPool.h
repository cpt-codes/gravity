#ifndef GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_
#define GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_

#include <algorithm>
#include <atomic>
#include <future>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "gravity/except/ErrorList.h"
#include "gravity/threads/ITask.h"
#include "gravity/threads/Task.h"
#include "gravity/threads/TaskQueue.h"

namespace gravity::threads
{
    // ThreadPool Keeps a set of threads waiting to execute incoming Tasks.
    class ThreadPool
    {
    public:
        // Construct a ThreadPool with @param threads given
        explicit ThreadPool(unsigned int threads = HardwareConcurrency());

        // Number of physical and logical CPUs available. Guaranteed to be >= 1.
        static unsigned int HardwareConcurrency();

        // Total number of threads in the pool
        unsigned int ThreadCount() { return threads_.size(); }

        // Number of threads working on tasks
        unsigned int ActiveThreads() { return active_; }

        // Number of tasks waiting to be executed
        unsigned int TasksQueued() { return queue_.Size(); }

        // Submit an ITask to be run in the thread pool
        void Submit(std::shared_ptr<ITask> const& task);

        // Submit a task to be run in the thread pool and return a future
        template <typename Func, typename... Args> requires std::invocable<Func, Args...>
        auto Submit(Func&& func, Args&&... args);

        // Parallel for-each. Applies the given invocable object func to the result of each de-
        // referenced iterator in the range [being, end] in task_count threads. Returns a vector
        // of shared futures or blocks until all tasks are complete.
        template<std::random_access_iterator Iter, std::copy_constructible Func, bool Block = true>
            requires std::invocable<Func, std::iter_value_t<Iter>>
        auto ForEach(Iter begin, Iter end, Func func, unsigned int task_count = 0);

        ~ThreadPool();

        ThreadPool(ThreadPool const&) = delete;
        ThreadPool(ThreadPool&&) noexcept = delete;

        ThreadPool& operator=(ThreadPool const&) = delete;
        ThreadPool& operator=(ThreadPool&&) noexcept = delete;

    private:
        std::vector<std::thread> threads_;
        std::atomic_uint active_; // number of threads executing tasks
        TaskQueue queue_; // tasks submitted to the thread pool

        using futures_t = std::vector<std::shared_future<void>>;

        // Runs in each thread. Acquires a task from the queue and executes it.
        void Worker();

        void JoinThreads();

        static void CheckForErrors(futures_t const& futures);
    };

    template <typename Func, typename... Args> requires std::invocable<Func, Args...>
    auto ThreadPool::Submit(Func&& func, Args&&... args)
    {
        using task_t = Task<Func, Args...>;

        auto task = std::make_shared<task_t>(
            std::forward<Func>(func), std::forward<Args>(args)...);

        Submit(task);

        return task->Future();
    }

    template<std::random_access_iterator Iter, std::copy_constructible Func, bool Block>
        requires std::invocable<Func, std::iter_value_t<Iter>>
    auto ThreadPool::ForEach(Iter begin, Iter end, Func func, unsigned int task_count)
    {
        if (begin == end)
        {
            return futures_t();
        }

        if (task_count == 0)
        {
            task_count = ThreadCount();
        }

        futures_t futures(task_count);

        auto item_count = static_cast<unsigned int>(end - begin);
        auto per_task = item_count / task_count;
        auto tasks_remaining = task_count % item_count;

        end = begin; // now use end to temporarily store the end of each block

        for (unsigned int i = 0; i < task_count; i++)
        {
            if (tasks_remaining > 0)
            {
                end += per_task + 1; // keep adding on the remainders...
                tasks_remaining--;
            }
            else
            {
                end += per_task; // ...until we have submitted them all
            }

            auto task = [begin, end, func]() mutable -> void
            {
                for (; begin != end; ++begin)
                {
                    func(*begin);
                }
            };

            futures[i] = Submit(std::move(task));
            begin = end; // now use begin to store the beginning of the next block
        }

        if constexpr(Block)
        {
            CheckForErrors(futures);
        }
        else
        {
            return futures;
        }
    }
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_
