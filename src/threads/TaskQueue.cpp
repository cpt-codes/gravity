#include "gravity/threads/TaskQueue.h"

namespace gravity::threads
{
    TaskQueue::TaskQueue()
        : task_cond_(std::make_shared<std::condition_variable>()),
          closed_(false, task_cond_)
    {

    }

    void TaskQueue::Push(std::shared_ptr<ITask> task)
    {
        std::lock_guard const lock(mutex_);

        if (closed_)
        {
            return;
        }

        queue_.push(std::move(task));
        task_cond_->notify_one(); // Notify a blocked thread a task available
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
        std::lock_guard const lock(mutex_);

        while(!queue_.empty())
        {
            queue_.pop();
        }

        task_cond_->notify_all(); // notify all threads the queue is empty
    }

    bool TaskQueue::Pop(std::shared_ptr<ITask>& task, bool const block)
    {
        std::unique_lock lock(mutex_);

        if (block)
        {
            // the predicate protects against spurious wake-ups and is guarded by the lock.
            // if timeout is reached the
            task_cond_->wait(lock, [this]()
            {
                return !queue_.empty() || closed_;
            });

            // don't return a task if the queue has been closed
            if (closed_)
            {
                return false;
            }
        }
        else if (queue_.empty())
        {
            return false;
        }

        task = std::move(queue_.front());

        queue_.pop();

        return true;
    }

    TaskQueue::~TaskQueue()
    {
        // Stop blocking any threads before destructing
        closed_ = true;
    }
}
