#include "gravity/threads/TaskQueue.h"

namespace gravity::threads
{
    void TaskQueue::Push(std::shared_ptr<ITask> task)
    {
        std::lock_guard const lock(mutex_);

        if (closed_)
        {
            return;
        }

        queue_.push(std::move(task));
        changed_.notify_one(); // notify one waiting thread a task available
    }

    unsigned int TaskQueue::Size() const
    {
        std::lock_guard const lock(mutex_);
        return queue_.size();
    }

    bool TaskQueue::Empty() const
    {
        std::lock_guard const lock(mutex_);
        return queue_.empty();
    }

    void TaskQueue::Clear()
    {
        {
            std::lock_guard const lock(mutex_);

            while (!queue_.empty())
            {
                queue_.pop();
            }
        }

        changed_.notify_all(); // notify all waiting threads the queue is now empty
    }

    bool TaskQueue::Pop(std::shared_ptr<ITask>& task, bool const block)
    {
        std::unique_lock lock(mutex_);

        if (block)
        {
            // The predicate protects against spurious wake-ups
            changed_.wait(lock, [this]()
            {
                return !queue_.empty() || closed_;
            });

            if (closed_)
            {
                return false;
            }
        }
        else if (queue_.empty() || closed_)
        {
            return false;
        }

        task = std::move(queue_.front());

        queue_.pop();

        return true;
    }

    // Using the mutex, instead of an atomic, ensures that the state of the queue
    // being open or closed is properly synchronized with pushing and popping items
    // to and from the queue.

    bool TaskQueue::Closed() const
    {
        std::lock_guard const lock(mutex_);
        return closed_;
    }

    void TaskQueue::Closed(bool closed)
    {
        {
            std::lock_guard const lock(mutex_);
            closed_ = closed;
        }

        changed_.notify_all(); // notify all waiting threads the state of closed has changed
    }

    TaskQueue::~TaskQueue()
    {
        // Stop blocking any threads before destructing
        Closed(true);
    }
}
