#ifndef GRAVITY_INCLUDE_GRAVITY_PARTICLE_H_
#define GRAVITY_INCLUDE_GRAVITY_PARTICLE_H_

#include "gravity/Vector.h"
#include "gravity/IParticle.h"

namespace gravity
{
    // Particle with mass, displacement, velocity and acceleration
    class Particle final : public IParticle
    {
    public:
        // Particle with mass m, displacement p and velocity v
        Particle(double m, Vector const& p, Vector const& v)
            : mass_(m), displacement_(p), velocity_(v) {}

        [[nodiscard]] double Mass() const override { return mass_; }

        [[nodiscard]] Vector const& Displacement() const override { return displacement_; }

        [[nodiscard]] Vector const& Velocity() const { return velocity_; }

        [[nodiscard]] Vector const& Acceleration() const { return acceleration_; }

        [[nodiscard]] double& Mass() { return mass_; }

        [[nodiscard]] Vector& Displacement() { return displacement_; }

        [[nodiscard]] Vector& Velocity() { return velocity_; }

        [[nodiscard]] Vector& Acceleration() { return acceleration_; }

    private:
        double mass_{ 1.0 };
        Vector displacement_;
        Vector velocity_;
        Vector acceleration_;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_PARTICLE_H_
