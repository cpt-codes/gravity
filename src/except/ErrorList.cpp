#include "gravity/except/ErrorList.h"

namespace gravity::except
{
    bool ErrorList::Empty() const
    {
        return empty_;
    }

    std::string ErrorList::Message() const
    {
        return stream_.str();
    }

    ErrorList& ErrorList::operator<<(std::string const& message)
    {
        if (empty_)
        {
            stream_ << "Exception(s) thrown:" << std::endl;
            empty_ = false;
        }

        stream_ << message << std::endl;

        return *this;
    }
}

