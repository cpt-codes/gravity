#ifndef GRAVITY_INCLUDE_GRAVITY_PARTICLE_H_
#define GRAVITY_INCLUDE_GRAVITY_PARTICLE_H_

#include "gravity/geometry/Vector.h"
#include "gravity/geometry/BoundingBox.h"

namespace gravity
{
    /// An ellipsoid particle with mass, position, velocity and acceleration.
    class Particle
    {
    public:
        /// Mass of a default constructed @c Particle
        static constexpr auto DefaultMass = 1.0;

        explicit Particle(double mass = DefaultMass) : mass_(mass) {}

        /// Mass of the particle. Constant for the lifetime of the @c Particle.
        [[nodiscard]]
        double Mass() const { return mass_; }

        double& Mass() { return mass_; }

        /// Radius of the particle in each dimension, making it an ellipsoid
        [[nodiscard]]
        geometry::Vector const& Radius() const { return bounds_.Extents(); }

        void Radius(geometry::Vector const& radius) { bounds_.Extents(radius); }

        /// Displacement of the particle in space
        [[nodiscard]]
        geometry::Vector const& Displacement() const { return bounds_.Centre(); }

        [[nodiscard]]
        geometry::Vector& Displacement() { return bounds_.Centre(); }

        /// Velocity of the particle relative to the position reference frame
        [[nodiscard]]
        geometry::Vector const& Velocity() const { return velocity_; }

        [[nodiscard]]
        geometry::Vector& Velocity() { return velocity_; }

        /// Acceleration of the particle
        [[nodiscard]]
        geometry::Vector const& Acceleration() const { return acceleration_; }

        [[nodiscard]]
        geometry::Vector& Acceleration() { return acceleration_; }

        /// Bounding box of the ellipsoid
        [[nodiscard]]
        geometry::BoundingBox const& Bounds() const { return bounds_; }

    private:
        double mass_;
        geometry::Vector velocity_;
        geometry::Vector acceleration_;

        /// The bounds are used for the displacement and radii of the particle
        geometry::BoundingBox bounds_;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_PARTICLE_H_
