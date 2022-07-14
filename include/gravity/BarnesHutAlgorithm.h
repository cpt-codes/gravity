#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUTALGORITHM_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUTALGORITHM_H_

#include <cassert>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <utility>

#include "gravity/Particle.h"
#include "gravity/Octree.h"
#include "gravity/MassCalculator.h"
#include "gravity/forces/IField.h"
#include "gravity/geometry/Vector.h"

namespace gravity
{
    /// @brief
    ///     The Barnes Hut algorithm calculates the force on a singular
    ///     particle due a distribution of particles contained in an Octree.
    ///     The class is optimised to be used concurrently and is thread-safe.
    /// @details
    ///     To calculate the net force on a singular particle in a distribution
    ///     of particles the nodes of the Octree are traversed from the root
    ///     node. If the centre of mass of all particles in a node if
    ///     sufficiently far from the particle, then the particles in the node
    ///     are treated as a single particle and the force is approximated
    ///     using this centre of mass. Otherwise, the process is repeated and
    ///     the node's children are traversed. Whether a node is considered
    ///     sufficiently far from a particle, depends on the quotient S/D,
    ///     where S is the width (extents) of the region of space represented
    ///     by the node, and D is the distance between the particle and the
    ///     node's particle's centre of mass. A node is sufficiently far away
    ///     when this ratio is lower than the approximation threshold, hence
    ///     the parameter determines the accuracy of the force calculations. If
    ///     the approximation threshold is 0.0, no approximations are made and
    ///     the algorithm degenerates to a direct-sum of all interactions.
    ///     Generally, values between 1.0 and 2.0 give a reasonable trade-off
    ///     between speed and accuracy.
    class BarnesHutAlgorithm
    {
    public:
        static constexpr auto DefaultApproximationThreshold = 1.0;

        BarnesHutAlgorithm(std::unique_ptr<Octree> octree,
                           std::unique_ptr<forces::IField> field,
                           double threshold = DefaultApproximationThreshold);

        /// The acceleration the @p particle is subject to due to all of the
        /// particles within the @c Octree, given the @c IField interaction
        /// between them. This member function is concurrency safe.
        [[nodiscard]]
        geometry::Vector Acceleration(std::shared_ptr<Particle const> const& particle) const;

        /// The force the @p particle is subject to due to all of the particles
        /// within the @c Octree, given the @c IField interaction between them.
        [[nodiscard, maybe_unused]]
        geometry::Vector Force(std::shared_ptr<Particle const> const& particle) const;

        ///     The approximation threshold controls the accuracy of the force
        ///     calculated between particles in the tree and the particle
        ///     subject to their collective gravitational field. The threshold
        ///     must be >= 0.0. The higher the threshold the faster the
        ///     calculation, but the higher the error due to approximations
        ///     made.
        [[nodiscard]]
        double ApproximationThreshold() const;

        void ApproximationThreshold(double threshold);

        /// The @c Octree containing all of the particles in the simulation.
        /// Retrieving this value clears the cache of @c Octree node mass
        /// calculations to guarantee correctness. This will transfer ownership
        /// of the Octree to the caller.
        [[nodiscard]]
        std::unique_ptr<Octree> Tree();

        void Tree(std::unique_ptr<Octree> octree);

        /// The @c IField that computes forces due to inter-particle
        /// interactions. This will transfer ownership of the IField to the
        /// caller.
        [[nodiscard]]
        std::unique_ptr<forces::IField> Field();

        void Field(std::unique_ptr<forces::IField> field);

        /// Update the @c Octree to the particle's current positions. The cache
        /// of @c Octree node mass calculations is cleared. Removed particles
        /// are returned in a linked list.
        std::list<std::shared_ptr<Particle>> Update();

    private:
        /// Returns true if the Barnes Hut algorithm should approximate force
        /// calculations for all particles within a node's @p bounds for a
        /// particle at the given @p point, false otherwise.
        [[nodiscard]]
        bool ShouldApproximate(geometry::Vector const& point,
                               geometry::BoundingBox const& bounds) const;

        /// Returns the acceleration on @p subject due to @p source in the
        /// @c IField.
        void AddAcceleration(Particle const& source,
                             Particle const& subject,
                             geometry::Vector& acceleration) const;

        /// Returns the acceleration on @p subject due to the @p source centre
        /// of mass in the @c IField.
        void AddAcceleration(MassCalculator::PointMass const &source,
                             Particle const &subject,
                             geometry::Vector& acceleration) const;

        /// Adds the @p acceleration on the @p particle due to all particles
        /// within the @c node.
        void AddAcceleration(Node const& node,
                             std::shared_ptr<Particle const> const& particle,
                             geometry::Vector& acceleration) const;

        mutable std::shared_mutex mutex_; ///< Synchronizes all "read-write" operations on the classes interface.
        mutable MassCalculator mass_calculator_; ///< Octree node centre of mass calculator.
        double threshold_{ DefaultApproximationThreshold }; ///< BarnesHutAlgorithm::ApproximationThreshold
        std::unique_ptr<Octree> tree_; ///< Octree containing all particles in the simulation.
        std::unique_ptr<forces::IField> field_; ///< The field describing all forces between particles.
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUTALGORITHM_H_
