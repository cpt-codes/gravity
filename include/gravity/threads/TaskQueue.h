#ifndef GRAVITY_INCLUDE_GRAVITY_THREAD_TASKQUEUE_H_
#define GRAVITY_INCLUDE_GRAVITY_THREAD_TASKQUEUE_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include "gravity/threads/ITask.h"

namespace gravity::threads
{
    /// @brief
    ///     Thread-safe queue class of Task instances.
    /// @details
    ///     Wraps around std::queue for thread safety. Intended to be used as
    ///     a means of submitting tasks in a thread-safe manner.
    class TaskQueue
    {
    public:
        TaskQueue() = default;

        /// Push the @p task onto the queue.
        void Push(std::shared_ptr<ITask> task);

        /// Number of tasks in the queue.
        unsigned int Size() const;

        /// Return @c true if the queue is empty, @c false otherwise.
        [[maybe_unused]]
        bool Empty() const;

        /// Clear all items from the queue.
        [[maybe_unused]]
        void Clear();

        /// @brief
        ///     Pop a @p task off the queue.
        /// @details
        ///     If block is @c true, Pop blocks until a value is available or
        ///     the instance is destructed, otherwise Pop will try to retrieve
        ///     a value immediately.
        /// @return
        ///     Returns @c true if a value was successfully written to @p task,
        ///     @c false otherwise. Returns immediately if the queue is closed.
        bool Pop(std::shared_ptr<ITask>& task, bool block = true);

        /// Returns @c true if the queue is accepting and returning tasks,
        /// @c false otherwise.
        bool Closed() const;

        /// Set closed status to @p closed.
        void Closed(bool closed);

        ~TaskQueue();

        TaskQueue(TaskQueue const& other) = delete;
        TaskQueue(TaskQueue&& other) noexcept = delete;

        TaskQueue& operator=(TaskQueue const& other) = delete;
        TaskQueue& operator=(TaskQueue&& other) noexcept = delete;

    private:
        mutable std::mutex mutex_; ///< Synchronizes use of the std::queue
        std::queue<std::shared_ptr<ITask>> queue_; ///< Queue of tasks
        std::condition_variable changed_; ///< Communicates that the state of the queue has changed
        bool closed_{}; ///< True if the queue is accepting and returning tasks, false otherwise
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_TASKQUEUE_H_
