#ifndef GRAVITY_INCLUDE_GRAVITY_THREAD_ITASK_H_
#define GRAVITY_INCLUDE_GRAVITY_THREAD_ITASK_H_

namespace gravity::threads
{
    class ITask
    {
    public:
        /// Execute the ITask
        virtual void Execute() = 0;

        ITask() = default;
        virtual ~ITask() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        ITask(ITask const&) = default;
        ITask(ITask&&) noexcept = default;

        ITask& operator=(ITask const&) = default;
        ITask& operator=(ITask&&) noexcept = default;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_ITASK_H_
