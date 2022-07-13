#include "gravity/Node.h"

namespace gravity
{
    Node::Node(geometry::BoundingBox bounds)
        : bounds_(std::move(bounds))
    {

    }

    bool Node::Insert(std::shared_ptr<Particle> const& particle, // NOLINT(misc-no-recursion)
                      double const looseness, double const min_width, unsigned const capacity)
    {
        if (!particle || !Contains(particle->Bounds(), looseness))
        {
            return false;
        }

        // If this node is a leaf, check if we can insert into this node
        // or if it needs splitting

        if (IsLeaf())
        {
            if (particles_.size() < capacity || IsMinWidth(min_width))
            {
                particles_.push_back(particle);
                return true;
            }

            Branch(looseness, min_width, capacity);
        }

        // Try inserting into a child node, else keep it in this node

        if (!NearestChild(particle).Insert(particle, looseness, min_width, capacity))
        {
            particles_.push_back(particle);
        }

        return true;
    }

    bool Node::Remove(std::shared_ptr<Particle> const& particle, // NOLINT(misc-no-recursion)
                      unsigned const capacity)
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

        if (IsLeaf() || !NearestChild(particle).Remove(particle, capacity))
        {
            return false;
        }

        // Successfully removed, now check if we should merge

        if (ShouldMerge(capacity))
        {
            Merge();
        }

        return true;
    }

    std::list<std::shared_ptr<Particle>> 
    Node::Update(double const looseness, double const min_width, unsigned const capacity)
    {
        std::list<std::shared_ptr<Particle>> removed;

        Update(removed, looseness, min_width, capacity);

        return removed;
    }

    bool Node::Shrink()
    {
        // Enable ADL
        using std::swap;

        // Shrinking does not apply to leaf nodes or nodes with particles.
        if (IsLeaf() || !particles_.empty())
        {
            return false;
        }

        // We can shrink the tree to a child only if only one child contains
        // particles.

        geometry::Orthant orthant;

        if (!OneChildHasParticles(orthant))
        {
            return false;
        }

        auto* child = &children_.at(orthant);

        auto subtree = Node(child->bounds_.ShrinkTo(orthant));

        // Swap subtree with this, then this with child. this will contain the
        // previous contents of child (new root node), child will contain the
        // previous contents of subtree (empty leaf node), and subtree will
        // contain the previous contents of this (old root node). subtree will
        // be destructed at the end of this scope, containing the empty child.

        swap(subtree, *this);
        swap(*this, *child);

        return true;
    }

    void Node::Grow(geometry::Vector const& point, double const looseness,
                    double const min_width, unsigned const capacity)
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

        auto root = Node(bounds_.ExpandFrom(orthant));

        root.Branch(looseness, min_width, capacity);

        // Swap child with this, then this with root. this will contain the
        // previous contents of root (new root node), child will contain the
        // previous contents of this (to be discarded), and root will contain
        // the previous contents of child (empty leaf node). root will be
        // destructed at the end of this scope, containing the empty child.

        auto& child = root.children_.at(orthant);

        swap(child, *this);
        swap(*this, root);
    }

    bool Node::Contains(geometry::BoundingBox const& bounds, double const looseness) const
    {
        return bounds_.Contains(bounds, looseness);
    }

    bool Node::IsColliding(geometry::BoundingBox const& bounds, // NOLINT(misc-no-recursion)
                           double const looseness) const
    {
        if (!bounds_.Intersects(bounds, looseness)) // particles might be loosely contained
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
            if(child.IsColliding(bounds, looseness))
            {
                return true;
            }
        }

        return true;
    }

    std::list<std::shared_ptr<Particle>>
    Node::Colliding(geometry::BoundingBox const& bounds, double const looseness) const
    {
        std::list<std::shared_ptr<Particle>> colliding;

        GetColliding(bounds, colliding, looseness);

        return colliding;
    }

    bool Node::Empty() const // NOLINT(misc-no-recursion)
    {
        auto const empty = [](Node const& child) -> bool
        {
            return child.Empty();
        };

        return particles_.empty() && std::ranges::all_of(children_, empty);
    }

    void swap(Node& lhs, Node& rhs)
    {
        // If a parent is swapped with its child, the parent will be contained
        // within its own vector of child nodes. If either destructor is
        // called, one will call the other and cause undefined behaviour.
        // Node's public interface only provides a const reference to
        // a nodes children, so this is avoided.

        // Enable ADL
        using std::swap;
        
        swap(lhs.bounds_, rhs.bounds_);
        swap(lhs.particles_, rhs.particles_);
        swap(lhs.children_, rhs.children_);
    }

    bool Node::IsMinWidth(double const min_width) const
    {
        return geometry::any_less_than_or_equal_to(bounds_.Extents(), min_width / 2.0);
    }

    bool Node::ShouldMerge(unsigned const capacity) const
    {
        auto count = particles_.size();

        for (auto const& child : children_)
        {
            count += child.particles_.size();

            if (count > capacity)
            {
                return false;
            }
        }

        return count <= capacity;
    }

    Node& Node::NearestChild(std::shared_ptr<Particle> const& particle)
    {
        assert(!IsLeaf());
        assert(particle != nullptr);

        return children_.at(bounds_.Orthant(particle->Bounds().Centre()));
    }

    void Node::Branch(double const looseness, // NOLINT(misc-no-recursion)
                      double const min_width,
                      unsigned const capacity)
    {
        assert(IsLeaf());

        // Allocate and instantiate the child nodes

        children_.reserve(geometry::Orthant::Max());

        for (auto orthant = 0U; orthant < children_.capacity(); ++orthant)
        {
            children_.emplace_back(bounds_.ShrinkTo(orthant));
        }

        // Move particles into child nodes, where possible

        for (auto it = particles_.begin(); it != particles_.end(); )
        {
            auto particle = *it;

            if (NearestChild(particle).Insert(particle, looseness, min_width, capacity))
            {
                it = particles_.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void Node::Merge()
    {
        for (auto& child : children_)
        {
            particles_.splice(particles_.end(), child.particles_);
        }

        // Clear the nodes, but hold onto the reserved memory as it might
        // save an extra allocation it later...

        children_.clear();
    }

    void Node::Update(std::list<std::shared_ptr<Particle>>& removed, // NOLINT(misc-no-recursion)
                      double const looseness, double const min_width, unsigned const capacity)
    {
        // Recursively update children, collating all particles, until we reach a leaf node

        for (auto& child : children_)
        {
            child.Update(removed, looseness, min_width, capacity);
        }

        // Iterator starting at the particles removed from another node
        auto previously_removed = removed.begin();

        // Once a leaf node is reached, remove unbounded particles and insert
        // at the end of the removed list

        for (auto it = particles_.begin(); it != particles_.end(); )
        {
            auto particle = *it;

            if (!particle || !Contains(particle->Bounds(), looseness))
            {
                removed.splice(removed.begin(), particles_, it++);
            }
            else
            {
                ++it;
            }
        }

        // Now, try to insert the particles that were previously removed

        for (; previously_removed != removed.end(); )
        {
            auto particle = *previously_removed;

            if (particle && Insert(particle, looseness, min_width, capacity))
            {
                previously_removed = removed.erase(previously_removed);
            }
            else
            {
                ++previously_removed;
            }
        }

        // Check for possible merger

        if (!IsLeaf() && ShouldMerge(capacity))
        {
            Merge();
        }
    }

    bool Node::OneChildHasParticles(geometry::Orthant& child) const
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

    void Node::GetColliding(geometry::BoundingBox const& bounds, // NOLINT(misc-no-recursion)
                            std::list<std::shared_ptr<Particle>>& colliding,
                            double const looseness) const
    {
        if (!bounds_.Intersects(bounds, looseness)) // particles might be loosely contained
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
            child.GetColliding(bounds, colliding, looseness);
        }
    }
}
