#ifndef GRAVITY_INCLUDE_GRAVITY_THREAD_TASK_H_
#define GRAVITY_INCLUDE_GRAVITY_THREAD_TASK_H_

#include <functional>
#include <future>
#include <type_traits>
#include <utility>

#include "gravity/threads/ITask.h"

namespace gravity::threads
{
    // Task template for ambiguous functions
    template<typename Func, typename... Args>
    class Task final : public ITask
    {
    public:
        using return_t = std::invoke_result_t<std::decay_t<Func>, std::decay_t<Args>...>;

        explicit Task(Func&& func, Args&&... args)
            : task_(std::bind_front(std::forward<Func>(func), std::forward<Args>(args)...)),
            future_(task_.get_future()) {}

        void Execute() override { task_(); }

        std::shared_future<return_t> const& Future() const { return future_; }

    private:
        std::packaged_task<return_t()> task_;
        std::shared_future<return_t> future_;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_TASK_H_
