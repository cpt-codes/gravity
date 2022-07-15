#ifndef GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_
#define GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_

#include <algorithm>
#include <future>
#include <iterator>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <thread>
#include <vector>

#include "gravity/except/AsyncError.h"
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

        ThreadPool(ThreadPool const&) = delete;
        ThreadPool(ThreadPool&&) noexcept = delete;

        ThreadPool& operator=(ThreadPool const&) = delete;
        ThreadPool& operator=(ThreadPool&&) noexcept = delete;

        ~ThreadPool();

        /// Number of physical and logical CPUs available. Guaranteed to be >= 1.
        static unsigned HardwareConcurrency();

        /// Total number of threads in the pool
        [[nodiscard]]
        unsigned ThreadCount() const { return threads_.size(); }

        /// Number of tasks waiting to be executed
        [[nodiscard, maybe_unused]]
        unsigned TasksQueued() const { return queue_.Size(); }

        /// Submit a task to be run in the thread pool and return a future
        template <typename Func, typename... Args> requires std::invocable<Func, Args...>
        auto Submit(Func&& func, Args&&... args)
        {
            using task_t = Task<Func, Args...>;

            auto task = std::make_shared<task_t>(
                std::forward<Func>(func), std::forward<Args>(args)...);

            queue_.Push(task);

            return task->Future();
        }

        /// @brief
        ///     Applies the unary invocable @p func to each de-referenced
        ///     iterator in the @p range in @p task_count threads. Blocks
        ///     until all tasks are complete.
        /// @throws
        ///     gravity::except::async_exception if an exception is thrown in
        ///     any of the threads executing the given unary invokable.
        template<std::ranges::random_access_range Range, typename Func>
            requires std::indirectly_unary_invocable<Func, std::ranges::iterator_t<Range>>
        [[maybe_unused]]
        void ForEach(Range&& range, Func&& func, unsigned task_count = 0)
        {
            WaitOnResults(
                ParallelForEach(std::ranges::begin(range),
                                std::ranges::end(range),
                                std::forward<Func>(func),
                                task_count));
        }

        /// @brief
        ///     Applies the unary invocable @p func to each de-referenced
        ///     iterator in the @p range in @p task_count threads. Returns
        ///     immediately while the tasks are run asynchronously.
        template<std::ranges::random_access_range Range, typename Func>
            requires std::indirectly_unary_invocable<Func, std::ranges::iterator_t<Range>>
        [[maybe_unused]]
        void ForEachAsync(Range&& range, Func&& func, unsigned task_count = 0)
        {
            ParallelForEach(std::ranges::begin(range),
                            std::ranges::end(range),
                            std::forward<Func>(func),
                            task_count);
        }

        /// @brief
        ///     Applies the unary invocable @p func to each de-referenced
        ///     iterator in the range @p begin to @p end in @p task_count
        ///     threads. Blocks until all tasks are complete.
        /// @throws
        ///     gravity::except::async_exception if an exception is thrown in
        ///     any of the threads executing the given unary invokable.
        template<std::random_access_iterator Iter, typename Func>
            requires std::indirectly_unary_invocable<Func, Iter>
        [[maybe_unused]]
        void ForEach(Iter begin, Iter end, Func&& func, unsigned task_count = 0)
        {
            WaitOnResults(
                ParallelForEach(begin, end, std::forward<Func>(func), task_count));
        }

        /// @brief
        ///     Applies the unary invocable @p func to each de-referenced
        ///     iterator in the range @p begin to @p end in @p task_count
        ///     threads. Returns immediately while the tasks are run
        ///     asynchronously.
        template<std::random_access_iterator Iter, typename Func>
            requires std::indirectly_unary_invocable<Func, Iter>
        [[maybe_unused]]
        void ForEachAsync(Iter begin, Iter end, Func&& func, unsigned task_count = 0)
        {
            ParallelForEach(begin, end, std::forward<Func>(func), task_count);
        }

    private:
        using futures_t = std::vector<std::shared_future<void>>;

        /// @brief
        ///     Parallel for-each.
        /// @details
        ///     Applies the invocable @p func to each de-referenced iterator
        ///     in the range @p begin to @p end in @p task_count threads.
        /// @return
        ///     Returns a future for each task submitted to the thread pool.
        template<std::random_access_iterator Iter, typename Func>
            requires std::indirectly_unary_invocable<Func, Iter>
        futures_t ParallelForEach(Iter begin, Iter end, Func func, unsigned task_count)
        {
            if (begin == end)
            {
                return {};
            }

            if (task_count == 0)
            {
                task_count = ThreadCount();
            }

            futures_t futures(task_count);

            auto item_count = static_cast<unsigned>(end - begin);
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

                future = Submit([begin, end, func]() mutable -> void
                {
                    for (; begin != end; ++begin)
                    {
                        func(*begin);
                    }
                });

                begin = end; // now use begin to store the beginning of the next block
            }

            return futures;
        }

        /// Simple loop that acquires a task from the queue and executes it.
        /// If there are no tasks, it waits on the queue.
        void Worker();

        /// Closes the queue, then joins all threads in the pool.
        void JoinThreads();

        /// Waits on the futures results. Exceptions are caught and thrown
        static void WaitOnResults(futures_t const& futures);

        std::vector<std::thread> threads_; ///< Threads in the pool
        TaskQueue queue_; ///< Tasks submitted to the thread pool
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_THREADPOOL_H_
