#ifndef GRAVITY_INCLUDE_GRAVITY_FORCES_IGRAVITATIONALFIELD_H_
#define GRAVITY_INCLUDE_GRAVITY_FORCES_IGRAVITATIONALFIELD_H_

#include "gravity/Particle.h"
#include "gravity/geometry/Vector.h"

namespace gravity::forces
{
    class IGravitationalField
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

        IGravitationalField() = default;

        virtual ~IGravitationalField() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        IGravitationalField(IGravitationalField const&) = default;
        IGravitationalField& operator=(IGravitationalField const&) = default;

        IGravitationalField(IGravitationalField&&) noexcept = default;
        IGravitationalField& operator=(IGravitationalField&&) noexcept = default;

    private:
        double grav_{ 6.67430e10-11 }; // Gravitational constant
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_FORCES_IGRAVITATIONALFIELD_H_
