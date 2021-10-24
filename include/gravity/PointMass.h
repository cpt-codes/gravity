#ifndef GRAVITY_INCLUDE_GRAVITY_POINTMASS_H_
#define GRAVITY_INCLUDE_GRAVITY_POINTMASS_H_

#include "gravity/Vector.h"

namespace gravity
{
    // Abstract class for point masses
    class PointMass
    {
    public:
        // PointMass's mass
        [[nodiscard]] virtual double Mass() const = 0;

        // PointMass's displacement Vector
        [[nodiscard]] virtual Vector const& Displacement() const = 0;

        PointMass() = default;

        virtual ~PointMass() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        PointMass(PointMass const&) = default;
        PointMass& operator=(PointMass const&) = default;

        PointMass(PointMass&&) noexcept = default;
        PointMass& operator=(PointMass&&) noexcept = default;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_POINTMASS_H_
