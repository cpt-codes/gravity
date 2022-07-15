#ifndef GRAVITY_INCLUDE_GRAVITY_NODE_H_
#define GRAVITY_INCLUDE_GRAVITY_NODE_H_

#include <algorithm>
#include <list>
#include <memory>
#include <mutex>
#include <ranges>
#include <utility>
#include <vector>

#include "gravity/Particle.h"
#include "gravity/geometry/Vector.h"
#include "gravity/geometry/Orthant.h"
#include "gravity/geometry/BoundingBox.h"
#include "gravity/threads/ThreadPool.h"

namespace gravity
{
    /// @brief
    ///     A node in a dynamic octree which automatically branches and merges
    ///     its child nodes when particles are inserted or removed.
    /// @details
    ///     The dynamic octree has three configurable elements: the looseness
    ///     of the tree, minimum width of a node, and the maximum number of
    ///     children in a node. These elements determine the behaviour of the
    ///     tree during state changes (insertion, removal, etc). These
    ///     parameters must be supplied to the various member functions of the
    ///     node.
    class Node
    {
    public:
        static constexpr auto DefaultLooseness = 1.25;
        static constexpr auto DefaultMinWidth = 1.0;
        static constexpr auto DefaultCapacity = 8U;

        explicit Node(geometry::BoundingBox bounds);

        /// @brief
        ///     Insert a @p particle with @p looseness into the node or its
        ///     ancestors. Nodes branch when they reach @p capacity if
        ///     their children's bounds will not breach @p min_width.
        /// @return
        ///     @c true if the @p particle was inserted, @c false otherwise.
        bool Insert(std::shared_ptr<Particle> const& particle,
                    double looseness = DefaultLooseness,
                    double min_width = DefaultMinWidth,
                    unsigned capacity = DefaultCapacity);

        /// @brief
        ///     Remove a @p particle from the node or its ancestors.
        /// @details
        ///     If the @p particle is successfully removed, child nodes will
        ///     be merged into parent nodes if the parent and child nodes
        ///     collectively contain less particles than the @p capacity
        ///     particles.
        bool Remove(std::shared_ptr<Particle> const& particle,
                    unsigned capacity = DefaultCapacity);

        /// @brief
        ///     The node is updated to reflect changes in the particles
        ///     @c geometry::BoundingBox.
        /// @details
        ///     Particles are removed bottom-up from the nodes ancestors and
        ///     inserted at higher level ancestors, thus letting them cascade
        ///     back down into the correct ancestor.
        /// @return
        ///     A list of particles that no longer fit within the tree.
        std::list<std::shared_ptr<Particle>>
            Update(double looseness = DefaultLooseness,
                   double min_width = DefaultMinWidth,
                   unsigned capacity = DefaultCapacity,
                   std::shared_ptr<threads::ThreadPool> const& pool = nullptr);

        /// If possible, the bounds of the node are shrunk to one of its
        /// children. The child become the new root node. Return @c true if it
        /// was shrunk, @c false otherwise.
        bool Shrink();

        /// Grows the bounds of the node in the direction of the given point.
        /// A new root node is created and swapped with @c this. The bounds are
        /// grown by a factor of two, due to the design of an Octree.
        void Grow(geometry::Vector const& point,
                  double looseness = DefaultLooseness,
                  double min_width = DefaultMinWidth,
                  unsigned capacity = DefaultCapacity);

        /// Returns @c true if the Octree contains any particles, otherwise
        /// @c false.
        [[nodiscard]]
        bool Empty() const;

        /// Child nodes of this node. The children may be leaf or branch nodes.
        [[nodiscard]]
        std::vector<Node> const& Children() const { return children_; }

        /// Particles contained in @c this node only.
        [[nodiscard]]
        std::list<std::shared_ptr<Particle>> const& Particles() const { return particles_; }

        /// Bounds within which ancestors of this node are contained.
        [[nodiscard]]
        geometry::BoundingBox const& Bounds() const { return bounds_; }

        /// Enable efficient swapping of Nodes.
        friend void swap(Node& lhs, Node& rhs);

    private:
        /// Determines how a Node traverses its subtree.
        enum class Traverse : bool
        {
            Ancestors, ///< Traverse all ancestors of the Node.
            Children ///< Traverse only the direct children of the Node.
        };

        /// Returns @c true if the node's bounds are less than or equal to
        /// @p min_width, @c false otherwise.
        [[nodiscard]]
        bool IsMinWidth(double min_width) const;

        /// Returns @c true if the node should merge its child nodes into
        /// itself, @c false otherwise.
        [[nodiscard]]
        bool ShouldMerge(unsigned capacity) const;

        /// @brief
        ///     Get a reference to the child node whose axes encapsulate the
        ///     particle.
        /// @details
        ///     The child's bounds do not necessarily encapsulate the
        ///     particle, only its axes do, hence it is the nearest child.
        [[nodiscard]]
        Node& NearestChild(std::shared_ptr<Particle> const& particle);

        /// Branches the current node. Where possible, particles within this
        /// node are inserted into its instantiated children.
        void Branch(double looseness, double min_width, unsigned capacity);

        /// Particles within child nodes are merged into this node.
        void Merge();

        /// @brief
        ///     Particles that no longer fit within the node's bounds are
        ///     removed from the node and inserted at the front of the list.
        ///     The node will attempt to insert particles already present in
        ///     the list, and if successful they are removed from the list.
        /// @details
        ///     The operation can be performed recursively by passing
        ///     @c Traverse::Ancestors. Unbounded particles are removed from
        ///     the deepest ancestors first and re-inserted in higher level
        ///     ancestors. This will be more efficient than removing and re-
        ///     inserting particles from the root node when incremental changes
        ///     in bounds have occurred. Particles are inserted and removed
        ///     using @p looseness, @p min_width, and @p capacity.
        void Update(std::list<std::shared_ptr<Particle>>& removed,
                    double looseness,
                    double min_width,
                    unsigned capacity,
                    Traverse traverse);

        /// Returns @c true if only one of this node's children has particles,
        /// @c false otherwise.
        bool OneChildHasParticles(geometry::Orthant& child) const;

        /// Returns @c true if this node is a leaf node (i.e. no children),
        /// @c false otherwise.
        [[nodiscard]]
        bool IsLeaf() const { return children_.empty(); }

        geometry::BoundingBox bounds_; ///< @c Node::Bounds
        std::vector<Node> children_; ///< Contiguous array of child nodes
        std::list<std::shared_ptr<Particle>> particles_; ///< Particles contained by this node
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_NODE_H_
