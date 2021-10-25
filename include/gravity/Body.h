#ifndef GRAVITY_INCLUDE_GRAVITY_BODY_H_
#define GRAVITY_INCLUDE_GRAVITY_BODY_H_

#include "gravity/Vector.h"
#include "gravity/IPointMass.h"

namespace gravity
{
    // Body with mass, displacement, velocity and acceleration
    class Body final : public IPointMass
    {
    public:
        // Body with mass m, displacement p and velocity v
        Body(double m, Vector const& p, Vector const& v);

        [[nodiscard]] double Mass() const override { return mass_; }

        [[nodiscard]] Vector const& Displacement() const override { return displacement_; }

        [[nodiscard]] Vector const& Velocity() const { return velocity_; }

        [[nodiscard]] Vector const& Acceleration() const { return acceleration_; }

        void Mass(double m);

        [[nodiscard]] Vector& Displacement() { return displacement_; }

        [[nodiscard]] Vector& Velocity() { return velocity_; }

        [[nodiscard]] Vector& Acceleration() { return acceleration_; }

        // Class is marked final, so we allow copy/move semantics. Rule of five applies.

        Body(Body const&) = default;
        Body(Body&&) noexcept = default;

        Body& operator=(Body const&) = default;
        Body& operator=(Body&&) noexcept = default;

        ~Body() override = default;

    private:
        double mass_{};
        Vector displacement_;
        Vector velocity_;
        Vector acceleration_;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BODY_H_
