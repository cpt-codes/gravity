#ifndef GRAVITY_INCLUDE_GRAVITY_IPOINTMASS_H_
#define GRAVITY_INCLUDE_GRAVITY_IPOINTMASS_H_

#include "gravity/Vector.h"

namespace gravity
{
    // Abstract class for point masses
    class IPointMass
    {
    public:
        // PointMass's mass
        [[nodiscard]] virtual double Mass() const = 0;

        // PointMass's displacement Vector
        [[nodiscard]] virtual Vector const& Displacement() const = 0;

        IPointMass() = default;

        virtual ~IPointMass() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        IPointMass(IPointMass const&) = default;
        IPointMass& operator=(IPointMass const&) = default;

        IPointMass(IPointMass&&) noexcept = default;
        IPointMass& operator=(IPointMass&&) noexcept = default;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_IPOINTMASS_H_
