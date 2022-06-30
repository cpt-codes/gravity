#include "gravity/geometry/Octree.h"

namespace gravity::geometry
{
    Octree::Octree
    (
        BoundingBox bounds,
        double const looseness,
        double const min_width,
        unsigned const max_per_node
    )
        : bounds_(std::move(bounds)),
          looseness_(looseness),
          min_width_(min_width),
          max_per_node_(max_per_node)
    {

    }

    bool Octree::Insert(std::shared_ptr<Particle> const& particle, Bounded const bounded) // NOLINT(misc-no-recursion)
    {
        if (!particle || !Contains(particle->Bounds(), bounded))
        {
            return false;
        }

        // If this node is a leaf, check if we can insert into this node
        // or if it needs splitting

        if (IsLeaf())
        {
            if (particles_.size() < MaxParticlesPerNode() || IsMinWidth())
            {
                particles_.push_back(particle);
                return true;
            }

            Branch();
        }

        // Try inserting into a child node, else keep it in this node

        if (!NearestChild(particle).Insert(particle))
        {
            particles_.push_back(particle);
        }

        return true;
    }

    bool Octree::Remove(std::shared_ptr<Particle> const& particle) // NOLINT(misc-no-recursion)
    {
        if (!particle)
        {
            return false;
        }

        // Try to remove from this node first

        if (particles_.remove(particle) > 0)
        {
            return true;
        }

        // Failed to remove from this node. If not a leaf, try removing from the nearest child

        if (IsLeaf() || !NearestChild(particle).Remove(particle))
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

    std::list<std::shared_ptr<Particle>> Octree::Update()
    {
        std::list<std::shared_ptr<Particle>> removed;

        Update(removed);

        return removed;
    }

    void Octree::Shrink()
    {
        // Enable ADL
        using std::swap;

        // Shrinking does not apply to leaf nodes or nodes with particles.
        if (IsLeaf() || !particles_.empty())
        {
            return;
        }

        // We can shrink the tree to a child only if only one child contains
        // particles.

        Orthant orthant;

        if (!OneChildHasParticles(orthant))
        {
            return;
        }

        auto* child = &children_[orthant];

        auto subtree = Octree(child->bounds_.ShrinkTo(orthant),
                              looseness_, min_width_, max_per_node_);

        // Swap subtree with this, then this with child. this will contain the
        // previous contents of child (new root node), child will contain the
        // previous contents of subtree (empty leaf node), and subtree will
        // contain the previous contents of this (old root node). subtree will
        // be destructed at the end of this scope, containing the empty child.

        swap(subtree, *this);
        swap(*this, *child);
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
                           min_width_, max_per_node_);

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

        for (auto const& particle : particles_)
        {
            if (particle && particle->Bounds().Intersects(bounds))
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

    std::list<std::shared_ptr<Particle>> Octree::Colliding(BoundingBox const& bounds) const
    {
        std::list<std::shared_ptr<Particle>> colliding;

        GetColliding(bounds, colliding);

        return colliding;
    }

    bool Octree::Empty() const // NOLINT(misc-no-recursion)
    {
        auto const empty = [](Octree const& child) -> bool
        {
            return child.Empty();
        };

        return std::ranges::all_of(children_, empty) && particles_.empty();
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
        swap(lhs.max_per_node_, rhs.max_per_node_);
        swap(lhs.bounds_, rhs.bounds_);
        swap(lhs.particles_, rhs.particles_);
        swap(lhs.children_, rhs.children_);
    }

    bool Octree::IsMinWidth() const
    {
        return any_less_than_or_equal_to(bounds_.Extents(), MinWidth() / 2.0);
    }

    bool Octree::ShouldMerge() const
    {
        auto count = particles_.size();

        for (auto const& child : children_)
        {
            count += child.particles_.size();

            if (count > MaxParticlesPerNode())
            {
                return false;
            }
        }

        return count <= MaxParticlesPerNode();
    }

    Octree& Octree::NearestChild(std::shared_ptr<Particle> const& particle)
    {
        assert(!IsLeaf());
        assert(particle != nullptr);

        return children_.at(bounds_.Orthant(particle->Bounds().Centre()));
    }

    void Octree::Branch() // NOLINT(misc-no-recursion)
    {
        assert(IsLeaf());

        // Allocate and instantiate the child nodes

        children_.reserve(Orthant::Max());

        for (auto orthant = 0U; orthant < children_.capacity(); ++orthant)
        {
            children_.emplace_back(bounds_.ShrinkTo(orthant),
                                   looseness_, min_width_, max_per_node_);
        }

        // Move particles into child nodes, where possible

        auto it = particles_.begin();

        while(it != particles_.end())
        {
            auto const& particle = *it;

            if (!NearestChild(particle).Insert(particle))
            {
                ++it;
                continue;
            }

            particles_.erase(it++);
        }
    }

    void Octree::Merge()
    {
        for (auto& child : children_)
        {
            particles_.splice(particles_.end(), child.particles_);
        }

        // Clear the nodes, but hold onto the reserved memory as it might
        // save an extra allocation it later...

        children_.clear();
    }

    void Octree::Update(std::list<std::shared_ptr<Particle>>& removed) // NOLINT(misc-no-recursion)
    {
        // Recursively update children, collating all particles, until we reach a leaf node

        for (auto& child : children_)
        {
            child.Update(removed);
        }

        // Iterator starting at the particles removed from another node
        auto previously_removed = removed.begin();

        // Once a leaf node is reached, remove unbounded particles and insert
        // at the end of the removed list
        {
            auto it = particles_.begin();

            while (it != particles_.end())
            {
                auto const &particle = *it;

                if (particle && Contains(particle->Bounds(), Bounded::Loosely))
                {
                    ++it;
                    continue;
                }

                removed.splice(removed.begin(), particles_, it++);
            }
        }

        // Now, try to insert the particles that were previously removed
        {
            auto it = previously_removed;

            while (it != removed.end())
            {
                auto const& particle = *it;

                if (particle && !Insert(particle, Bounded::Tightly))
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

    bool Octree::OneChildHasParticles(Orthant& child) const
    {
        bool has_particles{};

        for (auto i = 0U; i < children_.size(); ++i)
        {
            if (children_[i].Empty())
            {
                continue;
            }

            if (has_particles)
            {
                return false;
            }

            has_particles = true;
            child = i;
        }

        return has_particles;
    }

    void Octree::GetColliding(BoundingBox const& bounds, // NOLINT(misc-no-recursion)
        std::list<std::shared_ptr<Particle>>& colliding) const
    {
        if (!bounds_.Intersects(bounds))
        {
            return;
        }

        for (auto const& particle : particles_)
        {
            if (particle && particle->Bounds().Intersects(bounds))
            {
                colliding.push_back(particle);
            }
        }

        for (auto const& child : children_)
        {
            child.GetColliding(bounds, colliding);
        }
    }
}
