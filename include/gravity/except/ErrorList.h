#ifndef GRAVITY_INCLUDE_GRAVITY_THREADS_ERRORLIST_H_
#define GRAVITY_INCLUDE_GRAVITY_THREADS_ERRORLIST_H_

#include <exception>
#include <sstream>
#include <string>

namespace gravity::except
{
    class ErrorList
    {
    public:
        bool Empty() const { return empty_; }
        std::string Message() const { return stream_.str(); }
        ErrorList& operator<<(std::string const& message);
        ErrorList& operator<<(std::exception_ptr const& except);

    private:
        bool empty_{ true };
        std::ostringstream stream_; // stream used to build error message
    };
}


#endif //GRAVITY_INCLUDE_GRAVITY_THREADS_ERRORLIST_H_
