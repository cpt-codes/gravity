#ifndef GRAVITY_INCLUDE_GRAVITY_NEWTONIANFIELD_H_
#define GRAVITY_INCLUDE_GRAVITY_NEWTONIANFIELD_H_

#include <cmath>

#include "gravity/IGravitationalField.h"
#include "gravity/Particle.h"

namespace gravity
{
    class NewtonianField final : public IGravitationalField
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

#endif //GRAVITY_INCLUDE_GRAVITY_NEWTONIANFIELD_H_
