#ifndef GRAVITY_INCLUDE_GRAVITY_NEWTONIAN_H_
#define GRAVITY_INCLUDE_GRAVITY_NEWTONIAN_H_

#include "gravity/IGravity.h"

namespace gravity
{
    class Newtonian final : public IGravity
    {
    public:
        [[nodiscard]] geometry::Vector Acceleration(Particle const& p0, Particle const& p1) const override
        {
            auto r = p0.Displacement() - p1.Displacement();

            return -GravConst() * p0.Mass() * r / std::pow(geometry::ublas::norm_2(r), 3);
        }
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_NEWTONIAN_H_
