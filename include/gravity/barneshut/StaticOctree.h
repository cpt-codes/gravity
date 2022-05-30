#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_STATICOCTREE_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_STATICOCTREE_H_

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
    ///     @c StaticOctree is an octree designed to hold static objects.
    ///
    /// @details
    ///     Each branch node (subtree) contains 2 ** N children contained
    ///     within an N dimensional @c BoundingBox, and is an orthant of its
    ///     parent's @c BoundingBox. Each leaf node represents a single @c
    ///     Particle with a mass, position, velocity and acceleration. Each
    ///     subtree then approximates its children by their centre of mass.
    class StaticOctree final : public IParticle
    {
    public:
        /// Array of child nodes, these can be either @c Particle or
        /// @c StaticOctree.
        using node_array_t = std::array<std::shared_ptr<IParticle>, Orthant::Max()>;

        static constexpr auto DefaultGrowthLimit = 10U;

        explicit StaticOctree(BoundingBox bounds, unsigned growth_limit = DefaultGrowthLimit);

        StaticOctree(StaticOctree const&);
        StaticOctree(StaticOctree&&) noexcept = default;

        StaticOctree& operator=(StaticOctree const&);
        StaticOctree& operator=(StaticOctree&&) noexcept = default;

        ~StaticOctree() override = default;

        /// Total mass of all ancestors in the tree
        [[nodiscard]] double Mass() const override;

        /// Centre of mass of all the ancestors in the tree, from the origin
        [[nodiscard]] Vector const& Displacement() const override;

        /// @brief
        ///     Insert @p particles from a std::vector, growing where needed.
        /// @details
        ///     If any particle is not contained by the root node, then the
        ///     tree is grown to fit using @c GrowthLimit.
        /// @return
        ///     A boolean for each insertion operation. @c true if the particle
        ///     at the equivalent index in the given container was successfully
        ///     inserted, @c false otherwise.
        std::vector<bool> Insert(std::vector<std::shared_ptr<Particle>> const& particles);

        /// @brief
        ///     Insert a single @c Particle into the tree. The tree may be
        ///     grown up to the @c GrowthLimit to fit the particle.
        /// @return
        ///     If the @p particle was inserted, returns @c true, otherwise
        ///     @c false.
        bool Insert(std::shared_ptr<Particle> const& particle);

        /// A limit on the number of times a tree can grow to fit a point
        /// within its bounds.
        [[nodiscard]] std::size_t GrowthLimit() const;

        /// Bounds within which all children are contained
        [[nodiscard]] BoundingBox const& Bounds() const;

        /// Children of this node in the octree
        [[nodiscard]] node_array_t const& Children() const;

    private:
        /// Insert a @c Particle without updating the tree's total mass and
        /// centre of mass.
        void InsertWithoutUpdate(std::shared_ptr<Particle> const& particle);

        /// Updates the total mass and centre of mass of this tree, from the
        /// bottom-up, if the calculation is stale.
        void UpdateIfStale();

        /// Deep copy the other StaticOctree's nodes
        void DeepCopy(node_array_t const& nodes);

        /// Shallow copy @c this. Pointers to children are shared with the
        /// new @c StaticOctree.
        [[nodiscard]] StaticOctree ShallowCopy() const;

        /// @brief
        ///     If the @point is not bound by the octree, grow the octree,
        ///     within the growth limit, until it encapsulates the @p point.
        /// @details
        ///     If the octree is grown up to the growth limit and fails to
        ///     encapsulated the @p point, it is returned to its previous
        ///     state.
        /// @return
        ///     @c true if the tree contains the @p point, otherwise @c false.
        bool ContainsOrGrown(Vector const& point);

        bool stale_{}; /// @c true if the tree needs updating, otherwise @c false
        unsigned growth_limit_{}; /// A limit on the number of times a tree can grow to fit
        double mass_{}; /// Total mass of the tree's children
        Vector displacement_; /// Centre of mass of the tree's children
        node_array_t children_; /// Child node for each orthant parented by this tree
        BoundingBox bounds_; /// The box that contains this tree's children
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_STATICOCTREE_H_
