#ifndef GRAVITY_INCLUDE_GRAVITY_OCTREE_H_
#define GRAVITY_INCLUDE_GRAVITY_OCTREE_H_

#include <list>
#include <memory>
#include <stdexcept>

#include "gravity/Particle.h"
#include "gravity/Node.h"
#include "gravity/geometry/BoundingBox.h"
#include "gravity/threads/ThreadPool.h"

namespace gravity
{
    /// @brief
    ///     A dynamic octree which automatically branches and merges its nodes
    ///     when particles are inserted or removed from the tree.
    /// @details
    ///     The dynamic octree has three configurable elements: the looseness
    ///     of the tree, minimum width of a node, and the capacity of a node.
    ///     These elements determine the behaviour of the tree during state
    ///     changes (insertion, removal, etc).
    class Octree
    {
    public:
        static constexpr auto DefaultGrowthLimit = 10U;
        static constexpr auto DefaultShrinkLimit = 10U;

        explicit Octree
        (
            geometry::BoundingBox const& bounds,
            double looseness = Node::DefaultLooseness,
            double min_width = Node::DefaultMinWidth,
            unsigned node_capacity = Node::DefaultCapacity,
            int growth_limit = DefaultGrowthLimit,
            int shrink_limit = DefaultShrinkLimit
        );

        /// @brief
        ///     Insert a @p particle into the tree. The tree will be resized
        ///     automatically where required.
        /// @details
        ///     If @c the destination node is at maximum capacity and its
        ///     bounds are not below the minimum width it will branch.
        ///     Multiple insertions of the same @c particle are not checked.
        /// @return
        ///     @c true if the @p particle was inserted, @c false otherwise.
        bool Insert(std::shared_ptr<Particle> const& particle);

        /// @brief
        ///     Remove a @p particle from the tree.
        /// @details
        ///     If the @p particle is successfully removed, child nodes will
        ///     be merged into parent nodes if the parent and child nodes
        ///     collectively contain less particles than the maximum capacity
        ///     of particles.
        [[maybe_unused]]
        bool Remove(std::shared_ptr<Particle> const& particle);

        /// @brief
        ///     The tree is updated to reflect changes in the particles
        ///     @c geometry::BoundingBox. This is more optimal than removing
        ///     and re-inserting nodes. Optionally, a thread pool may be
        ///     used to parallelize the operation.
        /// @return
        ///     A list of particles that no longer fit within the tree.
        std::list<std::shared_ptr<Particle>>
            Update(std::shared_ptr<threads::ThreadPool> const& pool = nullptr);

        /// Return @c true if any particle within @c this tree is colliding with
        /// @p bounds, @c false otherwise.
        [[nodiscard, maybe_unused]]
        bool IsColliding(geometry::BoundingBox const& bounds) const;

        /// Return a list of particles within @c this tree colliding with
        /// @p bounds.
        [[nodiscard, maybe_unused]]
        std::list<std::shared_ptr<Particle>>
            Colliding(geometry::BoundingBox const& bounds) const;

        /// Particles contained in @c this node of the tree.
        [[nodiscard, maybe_unused]]
        std::list<std::shared_ptr<Particle>> Particles() const;

        /// The root @c Node of the tree.
        [[nodiscard]]
        Node const& Root() const { return root_; }

        /// The looseness is a multiplier applied to the bounds of a node when
        /// determining whether a particle is contained by said node. Hence,
        /// the particle is said to be "loosely" contained.
        [[nodiscard, maybe_unused]]
        double Looseness() const { return looseness_; }

        /// The minimum width of a node determines the maximum depth of a tree.
        /// It limits how many times the tree can branch from the root node.
        [[nodiscard, maybe_unused]]
        double MinWidth() const { return min_width_; }

        /// The maximum number of particles each node can hold before it
        /// branches.
        [[nodiscard, maybe_unused]]
        unsigned NodeCapacity() const { return node_capacity_; }

        /// The maximum number of times the tree can grow its bounds from its
        /// original size.
        [[nodiscard, maybe_unused]]
        int GrowthLimit() const { return growth_limit_; }

        /// The maximum number of times the tree can shrink its bounds from its
        /// original size.
        [[nodiscard, maybe_unused]]
        int ShrinkLimit() const { return shrink_limit_; }

        /// Number of times the tree has been grown (+ve) or shrunk (-ve) from
        /// its original size.
        [[nodiscard, maybe_unused]]
        int Resized() const { return resized_; }

        friend void swap(Octree& lhs, Octree& rhs);

    private:
        void GetParticles(Node const& node,
                          std::list<std::shared_ptr<Particle>>& particles) const;

        /// Return @c true if any @c Particle contained by the @p node or its
        /// ancestors is colliding with @p bounds, @c false otherwise.
        [[nodiscard]]
        bool IsColliding(geometry::BoundingBox const& bounds,
                         Node const& node) const;

        /// Insert particles in the @p node and its ancestors intersecting with
        /// @p bounds into @p colliding.
        void GetColliding(geometry::BoundingBox const& bounds,
                          Node const& node,
                          std::list<std::shared_ptr<Particle>>& colliding) const;

        Node root_; ///< Root node of the Octree
        double looseness_{ Node::DefaultLooseness }; ///< @c Octree::Looseness
        double min_width_{ Node::DefaultMinWidth }; ///< @c Octree::MinWidth
        unsigned node_capacity_{ Node::DefaultCapacity }; ///< @c Octree::NodeCapacity
        int growth_limit_{ DefaultGrowthLimit }; ///< @c Octree::GrowthLimit
        int shrink_limit_{ DefaultShrinkLimit }; ///< @c Octree::ShrinkLimit
        int resized_{}; ///< @c Octree::Resized
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_OCTREE_H_
