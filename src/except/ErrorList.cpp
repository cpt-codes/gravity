#include "gravity/except/ErrorList.h"

namespace gravity::except
{
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

    ErrorList& ErrorList::operator<<(std::exception_ptr const& except)
    {
        try
        {
            std::rethrow_exception(except);
        }
        catch(std::exception const& e)
        {
            return operator<<(e.what());
        }
        catch(...)
        {
            return operator<<("Non-standard exception caught");
        }
    }
}

