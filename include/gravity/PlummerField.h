 #ifndef GRAVITY_INCLUDE_GRAVITY_PLUMMERFIELD_H_
#define GRAVITY_INCLUDE_GRAVITY_PLUMMERFIELD_H_

#include <cmath>

#include "gravity/IGravitationalField.h"
#include "gravity/Particle.h"

namespace gravity
{
    class PlummerField final : public IGravitationalField
    {
    public:
        [[nodiscard]] geometry::Vector Acceleration(Particle const& source, Particle const& subject) const override
        {
            namespace ublas = geometry::ublas;

            // See https://en.wikipedia.org/wiki/Plummer_model for reference

            auto r = source.Displacement() - subject.Displacement();

            return -GravConst() * source.Mass() * r
                / std::pow(ublas::norm_2_square(r) + ublas::norm_2_square(source.Radius()), 1.5);
        }
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_PLUMMERFIELD_H_