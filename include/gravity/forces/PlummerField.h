#ifndef GRAVITY_INCLUDE_GRAVITY_FORCES_PLUMMERFIELD_H_
#define GRAVITY_INCLUDE_GRAVITY_FORCES_PLUMMERFIELD_H_

#include <cmath>

#include "gravity/Particle.h"
#include "gravity/forces/IGravitationalField.h"

namespace gravity::forces
{
    class PlummerField final : public IGravitationalField
    {
    public:
        void AddAcceleration(
            Particle const& source, Particle const& subject, geometry::Vector& acceleration) const override
        {
            namespace ublas = geometry::ublas;

            // See https://en.wikipedia.org/wiki/Plummer_model for reference

            auto r = source.Displacement() - subject.Displacement();

            acceleration += -GravConst() * source.Mass() * r
                / std::pow(ublas::norm_2_square(r) + ublas::norm_2_square(source.Radius()), 1.5);
        }
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_FORCES_PLUMMERFIELD_H_
