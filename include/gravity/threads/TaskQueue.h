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
    // Thread-safe queue class of Task instances. Wraps around std::queue for thread
    // safety. Intended to be used as a means of submitting tasks in a thread-safe
    // manner.
    class TaskQueue
    {
    public:
        TaskQueue() = default;

        // Push a new value onto the queue.
        void Push(std::shared_ptr<ITask> task);

        // Number of tasks in the queue.
        unsigned int Size() const;

        // Whether the queue is empty or not.
        bool Empty() const;

        //  Clear all items from the queue.
        void Clear();

        // Get the first value in the TaskQueue. If block is true, Pop blocks until a
        // value is available or the instance is destructed, otherwise Pop will try to
        // retrieve a value immediately. Also returns immediately if the queue's indicator
        // has been set to false. Returns true if a value was successfully written
        // to the task parameter, false otherwise.
        bool Pop(std::shared_ptr<ITask>& task, bool block = true);

        // Whether the queue is accepting and returning tasks or not.
        bool Closed() const;
        void Closed(bool closed);

        // Destructor ensures no threads are waiting.
        ~TaskQueue();

        TaskQueue(TaskQueue const& other) = delete;
        TaskQueue(TaskQueue&& other) noexcept = delete;

        TaskQueue& operator=(TaskQueue const& other) = delete;
        TaskQueue& operator=(TaskQueue&& other) noexcept = delete;

    private:
        mutable std::mutex mutex_; // used to synchronize use of the TaskQueue
        std::queue<std::shared_ptr<ITask>> queue_;
        std::condition_variable changed_; // used to wait for and communicate the state of the queue across threads
        bool closed_{}; // whether the queue is accepting and returning tasks or not
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_TASKQUEUE_H_
