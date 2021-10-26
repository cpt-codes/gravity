#ifndef GRAVITY_INCLUDE_GRAVITY_THREADS_INDICATOR_H_
#define GRAVITY_INCLUDE_GRAVITY_THREADS_INDICATOR_H_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <utility>

namespace gravity::threads
{
    // Atomic status indicator to share between threads.
    template<typename T>
    class Indicator
    {
    public:
        explicit Indicator(T status, std::shared_ptr<std::condition_variable> condition)
            : state_(std::move(status)), condition_(std::move(condition))
        {
            if (!condition_)
            {
                throw std::invalid_argument("Condition variable reference can not be null");
            }
        }

        [[nodiscard]] T State() const noexcept { return state_; }

        // Set the state of the indicator and notify all waiting threads.
        void Send(T status) noexcept
        {
            state_ = status;
            condition_->notify_all();
        }

        // Wait on the indicator's state to change and the predicate to be true.
        template<typename Predicate>
        void Wait(std::unique_lock<std::mutex>& lock, Predicate&& predicate)
        {
            static_assert(std::is_invocable_r_v<bool, Predicate>, "Predicate must be invocable as: bool(void)");

            condition_->wait(lock, std::forward<Predicate>(predicate));
        }

        // The operators provide the same functionality a normal atomic variable would
        // by delegating to the member variable.

        T operator=(T state) noexcept // NOLINT(misc-unconventional-assign-operator)
        {
            Send(state);
            return *this;
        }

        operator T() const noexcept // NOLINT(google-explicit-constructor)
        {
            return state_;
        }

        // Indicator is explicitly non-copyable and non-movable.

        ~Indicator() = default;

        Indicator(Indicator const&) = delete;
        Indicator(Indicator&&) noexcept = delete;

        Indicator& operator=(Indicator const&) = delete;
        Indicator& operator=(Indicator&&) noexcept = delete;

    private:
        std::atomic<T> state_;
        std::shared_ptr<std::condition_variable> condition_;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREADS_INDICATOR_H_
