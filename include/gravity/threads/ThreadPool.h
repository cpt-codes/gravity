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
    /// ThreadPool Keeps a set of threads waiting to execute incoming Tasks.
    class ThreadPool
    {
    public:
        /// Construct a ThreadPool with @p threads given
        explicit ThreadPool(unsigned int threads = HardwareConcurrency());

        /// Number of physical and logical CPUs available. Guaranteed to be >= 1.
        static unsigned int HardwareConcurrency();

        /// Total number of threads in the pool
        [[nodiscard]]
        unsigned int ThreadCount() const { return threads_.size(); }

        /// Number of tasks waiting to be executed
        [[nodiscard]]
        unsigned int TasksQueued() const { return queue_.Size(); }

        /// Submit an ITask to be run in the thread pool
        void Submit(std::shared_ptr<ITask> const& task);

        /// Submit a task to be run in the thread pool and return a future
        template <typename Func, typename... Args> requires std::invocable<Func, Args...>
        auto Submit(Func&& func, Args&&... args);

        /// @brief
        ///     Parallel for-each.
        /// @details
        ///     Applies the given invocable @p func to the result of each
        ///     iterator value type in the range @p begin to @p end in
        ///     @p task_count tasks.
        /// @return
        ///     Returns immediately if @c Block is @c false, otherwise returns
        ///     once all tasks are complete.
        template<std::random_access_iterator Iter, std::copy_constructible Func, bool Block = true>
            requires std::invocable<Func, std::iter_value_t<Iter>>
        void ForEach(Iter begin, Iter end, Func func, unsigned int task_count = 0);

        ~ThreadPool();

        ThreadPool(ThreadPool const&) = delete;
        ThreadPool(ThreadPool&&) noexcept = delete;

        ThreadPool& operator=(ThreadPool const&) = delete;
        ThreadPool& operator=(ThreadPool&&) noexcept = delete;

    private:
        std::vector<std::thread> threads_; ///< Threads in the pool
        TaskQueue queue_; ///< Tasks submitted to the thread pool

        using futures_t = std::vector<std::shared_future<void>>;

        /// Simple loop that acquires a task from the queue and executes it.
        /// If there are no tasks, it waits on the queue.
        void Worker();

        /// Closes the queue, then joins all threads in the pool.
        void JoinThreads();

        /// Catches any exceptions thrown by the futures, wraps them into an
        /// ErrorList, then throws the message created in a single exception.
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
    void ThreadPool::ForEach(Iter begin, Iter end, Func func, unsigned int task_count)
    {
        if (begin == end)
        {
            return;
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

        for (auto& future : futures)
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

            future = Submit(std::move(task));

            begin = end; // now use begin to store the beginning of the next block
        }

        if constexpr(Block) // Check the futures if we're blocking
        {
            CheckForErrors(futures);
        }
    }
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_
