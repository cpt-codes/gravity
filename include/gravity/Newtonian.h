#ifndef GRAVITY_INCLUDE_GRAVITY_NEWTONIAN_H_
#define GRAVITY_INCLUDE_GRAVITY_NEWTONIAN_H_

#include <cmath>

#include "gravity/IGravity.h"
#include "gravity/Particle.h"

namespace gravity
{
    class Newtonian final : public IGravity
    {
    public:
        [[nodiscard]] geometry::Vector Acceleration(Particle const& source, Particle const& subject) const override
        {
            // See https://en.wikipedia.org/wiki/Newton%27s_law_of_universal_gravitation for reference

            auto r = source.Displacement() - subject.Displacement();

            return -GravConst() * source.Mass() * r / std::pow(geometry::ublas::norm_2(r), 3);
        }
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_NEWTONIAN_H_
