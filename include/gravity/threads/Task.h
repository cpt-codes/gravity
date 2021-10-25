#ifndef GRAVITY_INCLUDE_GRAVITY_THREAD_TASK_H_
#define GRAVITY_INCLUDE_GRAVITY_THREAD_TASK_H_

#include <functional>
#include <future>
#include <type_traits>
#include <utility>

#include "gravity/threads/ITask.h"

namespace gravity::threads
{
    // Thread safe Task template for ambiguous functions.
    template<typename Func, typename... Args>
    class Task final : public ITask
    {
    public:
        using return_t = std::invoke_result_t<std::decay_t<Func>, std::decay_t<Args>...>;

        explicit Task(Func&& func, Args&&... args)
            : task_(std::bind_front(std::forward<Func>(func), std::forward<Args>(args)...)),
            future_(task_.get_future())
        {

        }

        void Execute() override { task_(); }

        std::shared_future<return_t> const& Future() const { return future_; }

        // Move only due to std::packaged_task

        Task(Task const& other) = delete;
        Task(Task&& other) noexcept = default;

        Task& operator=(Task const& other) = delete;
        Task& operator=(Task&& other) noexcept = default;

        ~Task() override = default;

    private:
        std::packaged_task<return_t()> task_;
        std::shared_future<return_t> future_;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_TASK_H_
