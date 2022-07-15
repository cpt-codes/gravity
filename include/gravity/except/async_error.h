#ifndef GRAVITY_INCLUDE_GRAVITY_EXCEPT_ASYNC_ERROR_H_
#define GRAVITY_INCLUDE_GRAVITY_EXCEPT_ASYNC_ERROR_H_

#include <stdexcept>

namespace gravity::except
{
    /// Defines a type of object to be thrown as an exception. It reports
    /// errors that are consequences of exceptions being thrown in threads
    /// asynchronous to this one.
    class async_error : public std::logic_error
    {
    public:
        using std::logic_error::logic_error;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_EXCEPT_ASYNC_ERROR_H_
