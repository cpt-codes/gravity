#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_

#include <array>
#include <list>
#include <memory>
#include <utility>

#include <gravity/Vector.h>
#include <gravity/barneshut/Orthant.h>
#include <gravity/barneshut/BoundingBox.h>
#include <gravity/barneshut/IShape.h>

namespace gravity::barneshut
{
    /// The state describing how a shape is bounded by a node within a
    /// Octree
    enum class Bounded : bool
    {
        Tightly, ///< The shape is bounded exactly by the node
        Loosely ///< The shape is bounded by the loose bounds
    };

    /// @brief
    ///     A dynamic octree which automatically branches and merges its nodes
    ///     when shapes are inserted or removed from the tree.
    /// @details
    ///     The dynamic octree has three configurable elements: the looseness
    ///     of the tree, minimum width of a node, and the maximum number of
    ///     children in a node.
    class Octree
    {
    public:
        static constexpr auto DefaultLooseness = 1.25;
        static constexpr auto DefaultMinWidth = 1.0;
        static constexpr auto DefaultMaxShapes = 8U;

        explicit Octree(
            BoundingBox bounds,
            double looseness = DefaultLooseness,
            double min_width = DefaultMinWidth,
            unsigned max_shapes = DefaultMaxShapes);

        /// @brief
        ///     Insert a @p shape into the tree. The tree will be resized
        ///     automatically where required.
        /// @details
        ///     If @c this node is already at maximum capacity (@c MaxShapes)
        ///     it will branch. The shapes within @c this node will be
        ///     distributed amongst its children. If @c this node is equal to
        ///     or below the minimum width (@c MinWidth) it will not branch.
        ///     Multiple insertions of the same @c shape are not checked.
        /// @return
        ///     @c true if the @p shape was inserted, @c false otherwise.
        bool Insert(std::shared_ptr<IShape> const& shape, Bounded bounded = Bounded::Loosely);

        /// @brief
        ///     Remove a @p shape from the tree.
        /// @details
        ///     If the @p shape is successfully removed, child nodes will
        ///     be merged into parent nodes if the parent and child nodes
        ///     collectively contain less shapes than the maximum (MaxShapes).
        bool Remove(std::shared_ptr<IShape> const& shape);

        /// @brief
        ///     Shapes within the tree re-inserted using their current
        ///     @c Bounds.
        /// @details
        ///     Shapes are removed bottom-up and re-inserted at higher level
        ///     nodes, thus letting them cascade back down into the correct
        ///     node. Each node will automatically branch and/or merge given
        ///     the same conditions in @c Insert and @c Remove are met.
        /// @return
        ///     All shapes that no longer fit within the tree.
        std::list<std::shared_ptr<IShape>> Update();

        /// Returns @c true if this node contains shapes, @c false otherwise.
        [[nodiscard]] bool HasShapes() const;

        /// Shrinks this node to one of its children, if possible.
        void Shrink();

        /// Grow the tree in the direction of the given point. A new root node
        /// is created and swapped with @c this.
        void Grow(Vector const& point);

        /// Returns @c true if the tree contains the @p shape, @c false
        /// otherwise. @p loosely determines whether @p shape is contained
        /// loosely or tightly.
        [[nodiscard]] bool Contains(std::shared_ptr<IShape> const& shape, Bounded bounded) const;

        /// Bounds within which all children are contained
        [[nodiscard]] BoundingBox const& Bounds() const { return bounds_; }

        /// Returns @c true if this node is a leaf node (i.e. no children),
        /// @c false otherwise.
        [[nodiscard]] bool IsLeaf() const { return children_.empty(); }

        /// Children of this node in the octree. Empty if this node is a leaf.
        [[nodiscard]] std::vector<Octree> const& Children() const { return children_; }

        /// The looseness is a multiplier applied to the bounds of a node when
        /// determining whether a shape is contained by said node. Hence, the
        /// shape is said to be "loosely" contained. Shared between all nodes
        /// of a tree.
        [[nodiscard]] double Looseness() const { return looseness_; }

        /// The minimum width of a node determines the maximum depth of a tree.
        /// It limits how many times the tree can branch from the root node.
        /// Shared between all nodes of a tree.
        [[nodiscard]] double MinWidth() const { return min_width_; }

        /// The maximum number of children in a node determines how many shapes
        /// a leaf node can hold before it needs to branch. Shared between all
        /// nodes of a tree.
        [[nodiscard]] unsigned MaxShapes() const { return max_shapes_; }

        /// Enable efficient swapping of Octree with ADL use
        friend void swap(Octree& lhs, Octree& rhs);

    private:
        /// Returns @c true if the node's bounds are less than or equal to the
        /// minimum allowed width. @c false otherwise.
        [[nodiscard]] bool IsMinWidth() const;

        /// Returns @c true if the node should merge its child nodes into
        /// itself, @c false otherwise.
        [[nodiscard]] bool ShouldMerge() const;

        /// @brief
        ///     Get a reference to the child node whose axes encapsulate the shape.
        /// @details
        ///     The child's bounds do not necessarily encapsulate the shape,
        ///     only its axes do, hence it is the nearest child.
        [[nodiscard]] Octree& NearestChild(std::shared_ptr<IShape> const& shape);

        /// Branches the current node. Shapes within this node are inserted
        /// into children, where possible.
        void Branch();

        /// Shapes within child nodes are merged into @c this node.
        void Merge();

        /// @copydoc Octree::Update()
        /// @param[out] removed
        ///     Nodes removed are back-inserted into the list.
        void Update(std::list<std::shared_ptr<IShape>>& removed);

        /// Returns @c true if only one of this node's children has shapes,
        /// @c false otherwise.
        bool OneChildHasShapes(Orthant& child) const;

        double looseness_{}; ///< @c Octree::Looseness
        double min_width_{}; ///< @ Octree::MinWidth
        unsigned max_shapes_{}; ///< @c Octree::MaxShapes
        BoundingBox bounds_; ///< @c Octree::Bounds
        std::list<std::shared_ptr<IShape>> shapes_; ///< Shapes loosely contained by this node
        std::vector<Octree> children_; ///< Contiguous array of child nodes
    };
}



#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_
