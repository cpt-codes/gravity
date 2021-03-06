#ifndef GRAVITY_INCLUDE_GRAVITY_FORCES_NEWTONIANFIELD_H_
#define GRAVITY_INCLUDE_GRAVITY_FORCES_NEWTONIANFIELD_H_

#include <cmath>

#include "gravity/Particle.h"
#include "gravity/forces/IGravitationalField.h"

namespace gravity::forces
{
    class NewtonianField final : public IGravitationalField
    {
    public:
        void AddAcceleration(
            Particle const& source, Particle const& subject, geometry::Vector& acceleration) const override
        {
            // See https://en.wikipedia.org/wiki/Newton%27s_law_of_universal_gravitation for reference

            auto r = source.Displacement() - subject.Displacement();

            acceleration += -GravConst() * source.Mass() * r / std::pow(geometry::ublas::norm_2(r), 3);
        }
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_FORCES_NEWTONIANFIELD_H_
