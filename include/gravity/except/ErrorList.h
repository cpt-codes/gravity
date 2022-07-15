#ifndef GRAVITY_INCLUDE_GRAVITY_THREADS_ERRORLIST_H_
#define GRAVITY_INCLUDE_GRAVITY_THREADS_ERRORLIST_H_

#include <sstream>
#include <string>

namespace gravity::except
{
    class ErrorList
    {
    public:
        /// Returns @c true if the @c ErrorList contains any errors,
        /// @c false otherwise.
        bool Empty() const;

        /// Returns the error messages concatenated into a single message.
        std::string Message() const;

        /// Insert a new message into the ErrorList.
        ErrorList& operator<<(std::string const& message);

    private:
        bool empty_{ true }; /// @c true if the stream is empty, @c false otherwise.
        std::ostringstream stream_; /// string stream used to build error message.
    };
}


#endif //GRAVITY_INCLUDE_GRAVITY_THREADS_ERRORLIST_H_
