#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_

#include <array>
#include <list>
#include <memory>
#include <stdexcept>
#include <utility>

#include "gravity/IParticle.h"
#include "gravity/Particle.h"
#include "gravity/IGravity.h"
#include "gravity/barneshut/Orthant.h"
#include "gravity/barneshut/BoundingBox.h"

namespace gravity::barneshut
{
    /// @brief
    ///     @c Octree is an octree designed to hold static objects.
    ///
    /// @details
    ///     Each branch node (subtree) contains 2 ** N children contained
    ///     within an N dimensional @c BoundingBox, and is an orthant of its
    ///     parent's @c BoundingBox. Each leaf node represents a single @c
    ///     Particle with a mass, position, velocity and acceleration. Each
    ///     subtree then approximates its children by their centre of mass.
    class Octree final : public IParticle
    {
    public:
        explicit Octree(BoundingBox bounds);

        /// Total mass of all ancestors in the tree
        [[nodiscard]] double Mass() const override;

        /// Centre of mass of all the ancestors in the tree, from the origin
        [[nodiscard]] Vector const& Displacement() const override;

        /// @brief
        ///     Build the @c Octree from a std::vector of @p particles.
        /// @details
        ///     If any particle is not contained by the root node, then the
        ///     tree is grown to fit using @c DefaultGrowthLimit.
        /// @throws std::runtime_error
        ///     If the growth limit is reached.
        void Build(std::vector<std::shared_ptr<Particle>> const& particles);

        /// Insert a single @c Particle into the tree. If the @p particle was
        /// inserted, returns @c true, otherwise @c false.
        bool Insert(std::shared_ptr<Particle> const& particle);

        /// @brief
        ///     Grow the bounds of @c this, until it encapsulates @p point.
        /// @details
        ///     Does nothing if the @p point is already contained. Optionally,
        ///     the @p limit sets a upper bound on the number of times the tree
        ///     will attempt to grow to fit the particle.
        /// @return
        ///     @c true if the tree contains the @p point, otherwise @c false.
        bool GrowToFit(Vector const& point, unsigned int limit = DefaultGrowthLimit);

        /// Default limit on the number of times a tree can grow to fit a
        /// point within its bounds.
        static unsigned int DefaultGrowthLimit;

        Octree(Octree const&);
        Octree(Octree&&) noexcept = default;

        Octree& operator=(Octree const&);
        Octree& operator=(Octree&&) noexcept = default;

        ~Octree() override = default;

    private:
        /// Array of child nodes, these can be either @c Particle or @c Octree.
        using node_array_t = std::array<std::shared_ptr<IParticle>, Orthant::Max()>;

        /// Insert a @c Particle without updating the tree's total mass and
        /// centre of mass.
        void InsertWithoutUpdate(std::shared_ptr<Particle> const& particle);

        /// Updates the total mass and centre of mass of this tree, from the
        /// bottom-up, if the calculation is stale.
        void UpdateIfNeeded();

        /// Deep copy the other Octree's nodes
        void DeepCopy(node_array_t const& nodes);

        /// Shallow copy @c this. Pointers to children are shared with the
        /// new @c Octree.
        [[nodiscard]] Octree ShallowCopy() const;

        bool stale_{}; /// @c true if the tree needs updating, otherwise @c false
        double mass_{}; /// Total mass of the tree's children
        Vector displacement_; /// Centre of mass of the tree's children
        node_array_t children_; /// Child node for each orthant parented by this tree
        BoundingBox bounds_; /// The box that contains this tree's children
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_
