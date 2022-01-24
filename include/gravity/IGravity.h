#ifndef GRAVITY_INCLUDE_GRAVITY_IGRAVITY_H_
#define GRAVITY_INCLUDE_GRAVITY_IGRAVITY_H_

#include <cmath>

#include "gravity/Vector.h"
#include "gravity/Particle.h"

namespace gravity
{
    class IGravity
    {
    public:
        // Compute the gravitational field that particle p1 is subject to due to particle p0.
        [[nodiscard]] virtual Vector Field(IParticle const& p0, Particle const& p1) const = 0;

        // Compute the gravitational force that particle p1 is subject to due to particle p0.
        [[maybe_unused]] [[nodiscard]] Vector Force(IParticle const& p0, Particle const& p1) const
        {
            return p1.Mass() * Field(p0, p1);
        }

        [[nodiscard]] double GravConst() const { return grav_; }
        [[nodiscard]] double& GravConst() { return grav_; }

        IGravity() = default;

        virtual ~IGravity() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        IGravity(IGravity const&) = default;
        IGravity& operator=(IGravity const&) = default;

        IGravity(IGravity&&) noexcept = default;
        IGravity& operator=(IGravity&&) noexcept = default;

    private:
        double grav_{ 6.67430e10-11 }; // Gravitational constant
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_IGRAVITY_H_
