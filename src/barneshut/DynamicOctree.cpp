#include "gravity/barneshut/DynamicOctree.h"

namespace gravity::barneshut
{
    DynamicOctree::DynamicOctree(BoundingBox bounds, double const looseness,
                                 double const min_width, std::size_t const max_shapes)
        : bounds_(std::move(bounds)),
        looseness_(looseness),
        min_width_(min_width),
        max_shapes_(max_shapes)
    {

    }

    bool DynamicOctree::Insert(std::shared_ptr<IShape> const& shape) // NOLINT(misc-no-recursion)
    {
        if (!LooselyContains(shape))
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

    bool DynamicOctree::Remove(std::shared_ptr<IShape> const& shape) // NOLINT(misc-no-recursion)
    {
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

    std::list<std::shared_ptr<IShape>> DynamicOctree::Update()
    {
        std::list<std::shared_ptr<IShape>> removed;

        Update(removed);

        return removed;
    }

    bool DynamicOctree::LooselyContains(std::shared_ptr<IShape> const& shape) const
    {
        return bounds_.Contains(shape->Bounds(), Looseness());
    }

    bool DynamicOctree::IsLeaf() const
    {
        return children_.empty();
    }

    bool DynamicOctree::IsMinWidth() const
    {
        return any_less_than_or_equal_to(bounds_.Extents(), MinWidth() / 2.0);
    }

    bool DynamicOctree::ShouldMerge() const
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

    DynamicOctree& DynamicOctree::NearestChild(std::shared_ptr<IShape> const& shape)
    {
        assert(!IsLeaf());

        return children_.at(bounds_.Orthant(shape->Bounds().Centre()));
    }

    void DynamicOctree::Branch() // NOLINT(misc-no-recursion)
    {
        assert(IsLeaf());

        // Allocate and instantiate the child nodes

        children_.reserve(Orthant::Max());

        for (auto orthant = 0U; orthant < Orthant::Max(); ++orthant)
        {
            children_.emplace_back(bounds_.ShrinkTo(orthant), looseness_,
                                   min_width_, max_shapes_);
        }

        // Move shapes into child nodes, where possible

        auto it = shapes_.begin();

        while(it != shapes_.end())
        {
            auto const &shape = *it;

            if (!NearestChild(shape).Insert(shape))
            {
                ++it;
                continue;
            }

            shapes_.erase(it++);
        }
    }

    void DynamicOctree::Merge()
    {
        for (auto& child : children_)
        {
            shapes_.splice(shapes_.end(), child.shapes_);
        }

        // Clear the nodes, but hold onto the reserved memory as it might
        // save an extra allocation it later...

        children_.clear();
    }

    void DynamicOctree::Update(std::list<std::shared_ptr<IShape>>& removed) // NOLINT(misc-no-recursion)
    {
        // Recursively update children, collating all shapes, until we reach a leaf node

        for (auto& child : children_)
        {
            child.Update(removed);
        }

        // Once a leaf node is reached, remove unbounded shapes and insert
        // at the end of the removed list

        auto removed_from_this = removed.end();
        auto it = shapes_.begin();

        while (it != shapes_.end())
        {
            if (LooselyContains(*it))
            {
                ++it;
                continue;
            }

            removed.splice(removed.end(), shapes_, it++);
        }

        // Now, try to insert the shapes that were previously removed

        it = removed.begin();

        while (it != removed_from_this)
        {
            if(!Insert(*it))
            {
                ++it;
                continue;
            }

            removed.erase(it++);
        }

        // Check for possible merger

        if (!IsLeaf() && ShouldMerge())
        {
            Merge();
        }
    }
}
