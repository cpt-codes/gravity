#include "gravity/barneshut/Octree.h"

namespace gravity::barneshut
{
    Octree::Octree
    (
        BoundingBox bounds,
        double const looseness,
        double const min_width,
        unsigned const max_shapes
    )
        : bounds_(std::move(bounds)),
        looseness_(looseness),
        min_width_(min_width),
        max_shapes_(max_shapes)
    {

    }

    bool Octree::Insert(std::shared_ptr<IShape> const& shape, Bounded const bounded) // NOLINT(misc-no-recursion)
    {
        if (!shape || !Contains(shape->Bounds(), bounded))
        {
            return false;
        }

        // If this node is a leaf, check if we can insert into this node
        // or if it needs splitting

        if (IsLeaf())
        {
            if (shapes_.size() < MaxShapes() || IsMinWidth())
            {
                shapes_.push_back(shape);
                return true;
            }

            Branch();
        }

        // Try inserting into a child node, else keep it in this node

        if (!NearestChild(shape).Insert(shape))
        {
            shapes_.push_back(shape);
        }

        return true;
    }

    bool Octree::Remove(std::shared_ptr<IShape> const& shape) // NOLINT(misc-no-recursion)
    {
        if (!shape)
        {
            return false;
        }

        // Try to remove from this node first

        if (shapes_.remove(shape) > 0)
        {
            return true;
        }

        // Failed to remove from this node. If not a leaf, try removing from the nearest child

        if (IsLeaf() || !NearestChild(shape).Remove(shape))
        {
            return false;
        }

        // Successfully removed, now check if we should merge

        if (ShouldMerge())
        {
            Merge();
        }

        return true;
    }

    std::list<std::shared_ptr<IShape>> Octree::Update()
    {
        std::list<std::shared_ptr<IShape>> removed;

        Update(removed);

        return removed;
    }

    void Octree::Shrink()
    {
        // Enable ADL
        using std::swap;

        // Shrinking does not apply to leaf nodes or nodes with shapes
        if (IsLeaf() || !shapes_.empty())
        {
            return;
        }

        // We can shrink the tree to a child only if only one child contains
        // shapes.

        Orthant orthant;

        if (!OneChildHasShapes(orthant))
        {
            return;
        }

        auto& child = children_[orthant];

        auto subtree = Octree(child.bounds_.ShrinkTo(orthant),
                              looseness_, min_width_, max_shapes_);

        // Swap subtree with this, then this with child. this will contain the
        // previous contents of child (new root node), child will contain the
        // previous contents of subtree (empty leaf node), and subtree will
        // contain the previous contents of this (old root node). subtree will
        // be destructed at the end of this scope, containing the empty child.

        swap(subtree, *this);
        swap(*this, child);
    }

    void Octree::Grow(Vector const& point) // NOLINT(misc-no-recursion)
    {
        // Enable ADL
        using std::swap;

        // The BoundingBox must be expanded in the point of the
        // nearest Orthant. Hence, current BoundingBox must become the
        // inverse of the new root node.

        auto orthant = bounds_.Orthant(point).Invert();

        if (IsLeaf())
        {
            bounds_ = bounds_.ExpandFrom(orthant);
            return;
        }

        // Construct leaf node and branch to construct children

        auto root = Octree(bounds_.ExpandFrom(orthant), looseness_,
                           min_width_, max_shapes_);

        root.Branch();

        // Swap child with this, then this with root. this will contain the
        // previous contents of root (new root node), child will contain the
        // previous contents of this (to be discarded), and root will contain
        // the previous contents of child (empty leaf node). root will be
        // destructed at the end of this scope, containing the empty child.

        auto& child = root.children_.at(orthant);

        swap(child, *this);
        swap(*this, root);
    }

    bool Octree::Contains(BoundingBox const& bounds, Bounded const bounded) const
    {
        if (bounded == Bounded::Loosely)
        {
            return bounds_.Contains(bounds, Looseness());
        }

        return bounds_.Contains(bounds);
    }

    bool Octree::IsColliding(BoundingBox const& bounds) const // NOLINT(misc-no-recursion)
    {
        if (!bounds_.Intersects(bounds, Looseness())) // Do the bounds loosely intersect?
        {
            return false;
        }

        for (auto const& shape : shapes_)
        {
            if (shape && shape->Bounds().Intersects(bounds))
            {
                return true;
            }
        }

        for (auto const& child : children_)
        {
            if(child.IsColliding(bounds))
            {
                return true;
            }
        }

        return true;
    }

    std::list<std::shared_ptr<IShape>> Octree::Colliding(BoundingBox const& bounds) const
    {
        std::list<std::shared_ptr<IShape>> colliding;

        GetColliding(bounds, colliding);

        return colliding;
    }

    void swap(Octree& lhs, Octree& rhs)
    {
        // If a parent is swapped with its child, the parent will be contained
        // within its own vector of child nodes. If either destructor is
        // called, one will call the other and cause undefined behaviour.
        // Octree's public interface only provides a const reference to
        // a nodes children, so this is avoided.

        // Enable ADL
        using std::swap;

        swap(lhs.looseness_, rhs.looseness_);
        swap(lhs.min_width_, rhs.min_width_);
        swap(lhs.max_shapes_, rhs.max_shapes_);
        swap(lhs.bounds_, rhs.bounds_);
        swap(lhs.shapes_, rhs.shapes_);
        swap(lhs.children_, rhs.children_);
    }

    bool Octree::IsMinWidth() const
    {
        return any_less_than_or_equal_to(bounds_.Extents(), MinWidth() / 2.0);
    }

    bool Octree::ShouldMerge() const
    {
        auto count = shapes_.size();

        for (auto const& child : children_)
        {
            count += child.shapes_.size();

            if (count > MaxShapes())
            {
                return false;
            }
        }

        return count <= MaxShapes();
    }

    Octree& Octree::NearestChild(std::shared_ptr<IShape> const& shape)
    {
        assert(!IsLeaf());
        assert(shape != nullptr);

        return children_.at(bounds_.Orthant(shape->Bounds().Centre()));
    }

    void Octree::Branch() // NOLINT(misc-no-recursion)
    {
        assert(IsLeaf());

        // Allocate and instantiate the child nodes

        children_.reserve(Orthant::Max());

        for (auto orthant = 0U; orthant < children_.capacity(); ++orthant)
        {
            children_.emplace_back(bounds_.ShrinkTo(orthant),
                                   looseness_, min_width_, max_shapes_);
        }

        // Move shapes into child nodes, where possible

        auto it = shapes_.begin();

        while(it != shapes_.end())
        {
            auto const& shape = *it;

            if (!NearestChild(shape).Insert(shape))
            {
                ++it;
                continue;
            }

            shapes_.erase(it++);
        }
    }

    void Octree::Merge()
    {
        for (auto& child : children_)
        {
            shapes_.splice(shapes_.end(), child.shapes_);
        }

        // Clear the nodes, but hold onto the reserved memory as it might
        // save an extra allocation it later...

        children_.clear();
    }

    void Octree::Update(std::list<std::shared_ptr<IShape>>& removed) // NOLINT(misc-no-recursion)
    {
        // Recursively update children, collating all shapes, until we reach a leaf node

        for (auto& child : children_)
        {
            child.Update(removed);
        }

        // Iterator to shape removed from another node
        auto previously_removed = removed.begin();

        // Once a leaf node is reached, remove unbounded shapes and insert
        // at the end of the removed list
        {
            auto it = shapes_.begin();

            while (it != shapes_.end())
            {
                auto const &shape = *it;

                if (shape && Contains(shape->Bounds(), Bounded::Loosely))
                {
                    ++it;
                    continue;
                }

                removed.splice(removed.begin(), shapes_, it++);
            }
        }

        // Now, try to insert the shapes that were previously removed
        {
            auto it = previously_removed;

            while (it != removed.end())
            {
                auto const& shape = *it;

                if (shape && !Insert(shape, Bounded::Tightly))
                {
                    ++it;
                    continue;
                }

                removed.erase(it++);
            }
        }

        // Check for possible merger

        if (!IsLeaf() && ShouldMerge())
        {
            Merge();
        }
    }

    bool Octree::HasShapes() const // NOLINT(misc-no-recursion)
    {
        if (!shapes_.empty())
        {
            return true;
        }

        auto const has_shapes = [](Octree const& child) -> bool
        {
            return child.HasShapes();
        };

        return std::ranges::any_of(children_, has_shapes);
    }

    bool Octree::OneChildHasShapes(Orthant &child) const
    {
        bool last_had_shapes{};

        for (auto i = 0U; i < children_.size(); ++i)
        {
            if (!children_[i].HasShapes())
            {
                continue;
            }

            if (last_had_shapes)
            {
                return false;
            }

            last_had_shapes = true;
            child = i;
        }

        if (!last_had_shapes)
        {
            return false;
        }

        return true;
    }

    void Octree::GetColliding(BoundingBox const& bounds, // NOLINT(misc-no-recursion)
        std::list<std::shared_ptr<IShape>>& colliding) const
    {
        if (!bounds_.Intersects(bounds))
        {
            return;
        }

        for (auto const& shape : shapes_)
        {
            if (shape && shape->Bounds().Intersects(bounds))
            {
                colliding.push_back(shape);
            }
        }

        for (auto const& child : children_)
        {
            child.GetColliding(bounds, colliding);
        }
    }
}
