#ifndef GRAVITY_INCLUDE_GRAVITY_IGRAVITY_H_
#define GRAVITY_INCLUDE_GRAVITY_IGRAVITY_H_

#include "gravity/geometry/Vector.h"
#include "gravity/Particle.h"

namespace gravity
{
    class IGravity
    {
    public:
        /// Compute the gravitational acceleration that @p subject is subject
        /// to due to @p source.
        [[nodiscard]] virtual geometry::Vector Acceleration(Particle const& source, Particle const& subject) const = 0;

        /// Compute the gravitational force that @p subject is subject to due
        /// to @p source.
        [[maybe_unused]] [[nodiscard]] geometry::Vector Force(Particle const& source, Particle const& subject) const
        {
            return subject.Mass() * Acceleration(source, subject);
        }

        /// Compute the gravitational field that @p subject is subject to due to @p source.
        geometry::Vector operator()(Particle const& source, Particle const& subject) const
        {
            return Acceleration(source, subject);
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
