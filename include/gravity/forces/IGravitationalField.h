#ifndef GRAVITY_INCLUDE_GRAVITY_FORCES_IGRAVITATIONALFIELD_H_
#define GRAVITY_INCLUDE_GRAVITY_FORCES_IGRAVITATIONALFIELD_H_

#include "gravity/Particle.h"
#include "gravity/geometry/Vector.h"
#include "gravity/forces/IField.h"

namespace gravity::forces
{
    /// IGravitationalField provides an interface for classes implementing
    /// inter-particle force calculations due to gravitational attraction.
    class IGravitationalField : public IField
    {
    public:
        /// NIST Newtonian constant of gravitation:
        /// https://physics.nist.gov/cgi-bin/cuu/Value?bg
        static constexpr auto DefaultGravitationalConstant = 6.67430e10-11;

        [[nodiscard]]
        double GravConst() const { return grav_const_; }

        [[nodiscard]]
        double& GravConst() { return grav_const_; }

        IGravitationalField() = default;
        ~IGravitationalField() override = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        IGravitationalField(IGravitationalField const&) = default;
        IGravitationalField& operator=(IGravitationalField const&) = default;

        IGravitationalField(IGravitationalField&&) noexcept = default;
        IGravitationalField& operator=(IGravitationalField&&) noexcept = default;

    private:
        double grav_const_{ DefaultGravitationalConstant }; // Gravitational constant
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_FORCES_IGRAVITATIONALFIELD_H_
