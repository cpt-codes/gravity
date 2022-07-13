#include "gravity/Octree.h"

namespace gravity
{
    Octree::Octree
    (
        geometry::BoundingBox const& bounds,
        double const looseness,
        double const min_width,
        unsigned const node_capacity,
        int growth_limit,
        int shrink_limit
    ) :
        root_(bounds),
        looseness_(looseness),
        min_width_(min_width),
        node_capacity_(node_capacity),
        growth_limit_(growth_limit),
        shrink_limit_(shrink_limit)
    {
        if (looseness < 1.0)
        {
            std::invalid_argument("Looseness cannot be less than 1.0");
        }

        if (min_width < 0.0)
        {
            std::invalid_argument("Minimum width cannot be less than 0.0");
        }

        if (node_capacity <= 0)
        {
            std::invalid_argument("Node capacity cannot be less than or equal to 0");
        }

        if (growth_limit < 0)
        {
            std::invalid_argument("Growth limit cannot be less than 0");
        }

        if (shrink_limit < 0)
        {
            std::invalid_argument("Shrink limit cannot be less than 0");
        }
    }

    bool Octree::Insert(std::shared_ptr<Particle> const& particle)
    {
        if (root_.Insert(particle, looseness_, min_width_, node_capacity_))
        {
            return true;
        }

        while (resized_ < growth_limit_)
        {
            root_.Grow(particle->Displacement(), looseness_, min_width_, node_capacity_);

            resized_++;

            if (root_.Insert(particle, looseness_, min_width_, node_capacity_))
            {
                return true;
            }
        }

        for (; resized_ > -shrink_limit_; --resized_)
        {
            if (!root_.Shrink())
            {
                break;
            }
        }

        return false;
    }

    bool Octree::Remove(std::shared_ptr<Particle> const& particle)
    {
        if (!root_.Remove(particle, node_capacity_))
        {
            return false;
        }

        for (; resized_ > -shrink_limit_; --resized_)
        {
            if (!root_.Shrink())
            {
                break;
            }
        }

        return true;
    }

    std::list<std::shared_ptr<Particle>> Octree::Update()
    {
        auto removed = root_.Update(looseness_, min_width_, node_capacity_);

        for (auto it = removed.begin(); it != removed.end(); )
        {
            auto particle = *it;

            if (Insert(particle))
            {
                it = removed.erase(it);
            }
            else
            {
                ++it;
            }
        }

        return removed;
    }

    bool Octree::Contains(geometry::BoundingBox const& bounds) const
    {
        return root_.Contains(bounds, looseness_);
    }

    bool Octree::IsColliding(geometry::BoundingBox const& bounds) const
    {
        return root_.IsColliding(bounds, looseness_);
    }

    std::list<std::shared_ptr<Particle>> Octree::Colliding(geometry::BoundingBox const& bounds) const
    {
        return root_.Colliding(bounds, looseness_);
    }

    bool Octree::Empty() const
    {
        return root_.Empty();
    }

    std::list<std::shared_ptr<Particle>> Octree::Particles() const
    {
        std::list<std::shared_ptr<Particle>> particles;

        GetParticles(root_, particles);

        return particles;
    }

    void swap(Octree& lhs, Octree& rhs)
    {
        // Enable ADL
        using std::swap;

        swap(lhs.looseness_, rhs.looseness_);
        swap(lhs.min_width_, rhs.min_width_);
        swap(lhs.node_capacity_, rhs.node_capacity_);
        swap(lhs.root_, rhs.root_);
    }

    void Octree::GetParticles(Node const& node, // NOLINT(misc-no-recursion)
                              std::list<std::shared_ptr<Particle>>& particles) const
    {
        for (auto const& particle : node.Particles())
        {
            particles.push_back(particle);
        }

        for (auto const& child : node.Children())
        {
            GetParticles(child, particles);
        }
    }
}
