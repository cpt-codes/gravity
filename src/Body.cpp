#include "gravity/Body.h"

namespace gravity
{
    Body::Body(double m, Vector const& p, Vector const& v)
        : mass_(m), displacement_(p), velocity_(v)
    {
        if (m <= 0.0)
        {
            throw std::invalid_argument("Mass <= 0.0");
        }

        if (p.size() != Dimensions)
        {
            throw std::invalid_argument("Invalid size displacement vector");
        }

        if (v.size() != Dimensions)
        {
            throw std::invalid_argument("Invalid size velocity vector");
        }
    }

    void Body::Mass(double const m)
    {
        if (m <= 0.0)
        {
            throw std::invalid_argument("Mass <= 0.0");
        }

        mass_ = m;
    }
}