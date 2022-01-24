#ifndef GRAVITY_INCLUDE_GRAVITY_IPARTICLE_H_
#define GRAVITY_INCLUDE_GRAVITY_IPARTICLE_H_

#include "gravity/Vector.h"

namespace gravity
{
    // Abstract class for static point masses
    class IParticle
    {
    public:
        // Particle's mass
        [[nodiscard]] virtual double Mass() const = 0;

        // Particle's displacement Vector
        [[nodiscard]] virtual Vector const& Displacement() const = 0;

        IParticle() = default;

        virtual ~IParticle() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        IParticle(IParticle const&) = default;
        IParticle& operator=(IParticle const&) = default;

        IParticle(IParticle&&) noexcept = default;
        IParticle& operator=(IParticle&&) noexcept = default;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_IPARTICLE_H_
