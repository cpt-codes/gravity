#ifndef GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_
#define GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_

#include <algorithm>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

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

        // Submit a task to be run in the thread pool
        void Submit(std::shared_ptr<ITask> const& task);

        // Submit a task to be run in the thread pool and return a future
        template <typename Func, typename... Args>
        auto Submit(Func&& func, Args&&... args);

        ~ThreadPool();

        ThreadPool(ThreadPool const&) = delete;
        ThreadPool(ThreadPool&&) noexcept = delete;

        ThreadPool& operator=(ThreadPool const&) = delete;
        ThreadPool& operator=(ThreadPool&&) noexcept = delete;

    private:
        std::vector<std::thread> threads_;
        std::atomic_uint active_; // number of threads executing tasks
        TaskQueue queue_; // tasks submitted to the thread pool

        // Runs in each thread. Acquires a task from the queue and executes it.
        void Worker();

        void JoinThreads();
    };

    template <typename Func, typename... Args>
    auto ThreadPool::Submit(Func&& func, Args&&... args)
    {
        using task_type = Task<Func, Args...>;

        auto task = std::make_shared<task_type>(
            std::forward<Func>(func), std::forward<Args>(args)...);

        Submit(task);

        return task->Future();
    }
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_
