#ifndef GRAVITY_INCLUDE_GRAVITY_GEOMETRY_OCTREE_H_
#define GRAVITY_INCLUDE_GRAVITY_GEOMETRY_OCTREE_H_

#include <algorithm>
#include <list>
#include <memory>
#include <ranges>
#include <utility>
#include <vector>

#include "gravity/geometry/Vector.h"
#include "gravity/geometry/Orthant.h"
#include "gravity/geometry/BoundingBox.h"
#include "gravity/Particle.h"

namespace gravity::geometry
{
    /// The state describing how a particle is bounded by a node within a
    /// Octree
    enum class Bounded : bool
    {
        Tightly, ///< The particle is bounded exactly by the node
        Loosely ///< The particle is bounded by the loose bounds
    };

    /// @brief
    ///     A dynamic octree which automatically branches and merges its nodes
    ///     when particles are inserted or removed from the tree.
    /// @details
    ///     The dynamic octree has three configurable elements: the looseness
    ///     of the tree, minimum width of a node, and the maximum number of
    ///     children in a node. These elements determine the behaviour of the
    ///     tree during state changes (insertion, removal, etc).
    class Octree
    {
    public:
        static constexpr auto DefaultLooseness = 1.25;
        static constexpr auto DefaultMinWidth = 1.0;
        static constexpr auto DefaultMaxParticlesPerNode = 8U;

        explicit Octree(
            BoundingBox bounds,
            double looseness = DefaultLooseness,
            double min_width = DefaultMinWidth,
            unsigned max_per_node = DefaultMaxParticlesPerNode);

        /// @brief
        ///     Insert a @p particle into the tree. The tree will be resized
        ///     automatically where required.
        /// @details
        ///     If @c this node is already at maximum capacity of particles
        ///     it will branch. The particles within @c this node will be
        ///     distributed amongst its children. If @c this node's bounds are
        ///     equal to or below the minimum width it will not branch.
        ///     Multiple insertions of the same @c particle are not checked.
        /// @return
        ///     @c true if the @p particle was inserted, @c false otherwise.
        bool Insert(std::shared_ptr<Particle> const& particle, Bounded bounded = Bounded::Loosely);

        /// @brief
        ///     Remove a @p particle from the tree.
        /// @details
        ///     If the @p particle is successfully removed, child nodes will
        ///     be merged into parent nodes if the parent and child nodes
        ///     collectively contain less particles than the maximum capacity
        ///     of particles.
        bool Remove(std::shared_ptr<Particle> const& particle);

        /// @brief
        ///     Particles within the tree re-inserted using their current
        ///     @c BoundingBox.
        /// @details
        ///     Particles are removed bottom-up and re-inserted at higher level
        ///     nodes, thus letting them cascade back down into the correct
        ///     node. Each node will automatically branch and/or merge given
        ///     the same conditions in @c Insert and @c Remove are met. This
        ///     will be more efficient than removing and re-inserting where
        ///     there are incremental changes in bounds.
        /// @return
        ///     A list of particles that no longer fit within the tree.
        std::list<std::shared_ptr<Particle>> Update();

        /// Shrinks this node to one of its children, if possible.
        void Shrink();

        /// Grow the tree in the direction of the given point. A new root node
        /// is created and swapped with @c this.
        void Grow(Vector const& point);

        /// Returns @c true if the tree contains the @p particle, @c false
        /// otherwise. @p loosely determines whether @p particle is contained
        /// loosely or tightly.
        [[nodiscard]]
        bool Contains(BoundingBox const& bounds, Bounded bounded) const;

        /// Return @c true if any particle within @c this tree is colliding with
        /// @p bounds, @c false otherwise.
        [[nodiscard]]
        bool IsColliding(BoundingBox const& bounds) const;

        /// Return a list of particles within @c this tree colliding with
        /// @p bounds.
        [[maybe_unused, nodiscard]]
        std::list<std::shared_ptr<Particle>> Colliding(BoundingBox const& bounds) const;

        /// Returns @c true if the Octree contains any particles, otherwise
        /// @c false.
        [[nodiscard]]
        bool Empty() const;

        /// Direct child nodes of @c this tree. The children may be leaf or
        /// branch nodes.
        [[nodiscard]]
        std::vector<Octree> const& Children() const { return children_; }

        /// Particles contained in @c this node of the tree.
        [[nodiscard]]
        std::list<std::shared_ptr<Particle>> const& Particles() const { return particles_; }

        /// Bounds within which all children and particles of the tree are
        /// contained.
        [[nodiscard]]
        BoundingBox const& Bounds() const { return bounds_; }

        /// The looseness is a multiplier applied to the bounds of a node when
        /// determining whether a particle is contained by said node. Hence,
        /// the particle is said to be "loosely" contained.
        [[nodiscard]]
        double Looseness() const { return looseness_; }

        /// The minimum width of a node determines the maximum depth of a tree.
        /// It limits how many times the tree can branch from the root node.
        [[nodiscard]]
        double MinWidth() const { return min_width_; }

        /// The maximum particles per node node limits how many particles a
        /// leaf node can hold before it should to branch.
        [[nodiscard]]
        unsigned MaxParticlesPerNode() const { return max_per_node_; }

        /// Enable efficient swapping of Octrees.
        friend void swap(Octree& lhs, Octree& rhs);

    private:
        /// Returns @c true if the node's bounds are less than or equal to the
        /// minimum allowed width. @c false otherwise.
        [[nodiscard]]
        bool IsMinWidth() const;

        /// Returns @c true if the node should merge its child nodes into
        /// itself, @c false otherwise.
        [[nodiscard]]
        bool ShouldMerge() const;

        /// @brief
        ///     Get a reference to the child node whose axes encapsulate the
        ///     particle.
        /// @details
        ///     The child's bounds do not necessarily encapsulate the
        ///     particle, only its axes do, hence it is the nearest child.
        [[nodiscard]]
        Octree& NearestChild(std::shared_ptr<Particle> const& particle);

        /// Branches the current node. Particles within this node are inserted
        /// into children, where possible.
        void Branch();

        /// Particles within child nodes are merged into @c this node.
        void Merge();

        /// @copydoc Octree::Update()
        /// @param[out] removed
        ///     Nodes removed are back-inserted into the list.
        void Update(std::list<std::shared_ptr<Particle>>& removed);

        /// Returns @c true if only one of this node's children has particles,
        /// @c false otherwise.
        bool OneChildHasParticles(Orthant& child) const;

        /// Insert particles in @c this tree colliding with @p bounds into
        /// @p colliding.
        void GetColliding(BoundingBox const& bounds, std::list<std::shared_ptr<Particle>>& colliding) const;

        /// Returns @c true if this node is a leaf node (i.e. no children),
        /// @c false otherwise.
        [[nodiscard]]
        bool IsLeaf() const { return children_.empty(); }

        double looseness_{}; ///< @c Octree::Looseness
        double min_width_{}; ///< @ Octree::MinWidth
        unsigned max_per_node_{}; ///< @c Octree::MaxParticlesPerNode
        BoundingBox bounds_; ///< @c Octree::Bounds
        std::list<std::shared_ptr<Particle>> particles_; ///< Particles loosely contained by this node
        std::vector<Octree> children_; ///< Contiguous array of child nodes
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_GEOMETRY_OCTREE_H_
