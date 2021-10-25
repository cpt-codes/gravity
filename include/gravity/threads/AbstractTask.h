#ifndef GRAVITY_INCLUDE_GRAVITY_THREAD_ABSTRACTTASK_H_
#define GRAVITY_INCLUDE_GRAVITY_THREAD_ABSTRACTTASK_H_

namespace gravity::threads
{
    class AbstractTask
    {
    public:
        virtual void Execute() = 0;

        AbstractTask() = default;
        virtual ~AbstractTask() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        AbstractTask(AbstractTask const&) = default;
        AbstractTask(AbstractTask&&) noexcept = default;

        AbstractTask& operator=(AbstractTask const&) = default;
        AbstractTask& operator=(AbstractTask&&) noexcept = default;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_THREAD_ABSTRACTTASK_H_
