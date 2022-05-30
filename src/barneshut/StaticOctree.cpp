#include "gravity/barneshut/StaticOctree.h"

namespace gravity::barneshut
{
    StaticOctree::StaticOctree(BoundingBox bounds, unsigned const growth_limit)
        : bounds_(std::move(bounds)),
        growth_limit_(growth_limit)
    {

    }

    double StaticOctree::Mass() const
    {
        assert(!stale_);
        return mass_;
    }

    Vector const& StaticOctree::Displacement() const
    {
        assert(!stale_);
        return displacement_;
    }

    std::vector<bool> StaticOctree::Insert(std::vector<std::shared_ptr<Particle>> const& particles)
    {
        std::vector<bool> success;

        success.reserve(particles.size());

        for (auto const& particle : particles)
        {
            if (!particle || !ContainsOrGrown(particle->Displacement()))
            {
                success.push_back(false);
                continue;
            }

            success.push_back(true);
            InsertWithoutUpdate(particle);
        }

        UpdateIfStale();

        return success;
    }

    bool StaticOctree::Insert(std::shared_ptr<Particle> const& particle)
    {
        if (!particle || !ContainsOrGrown(particle->Displacement()))
        {
            return false;
        }

        InsertWithoutUpdate(particle);
        UpdateIfStale();

        return true;
    }

    std::size_t StaticOctree::GrowthLimit() const
    {
        return growth_limit_;
    }

    const BoundingBox &StaticOctree::Bounds() const
    {
        return bounds_;
    }

    StaticOctree::node_array_t const& StaticOctree::Children() const
    {
        return children_;
    }

    StaticOctree::StaticOctree(StaticOctree const& other)
        : bounds_(other.bounds_),
          stale_(other.stale_),
          mass_(other.mass_),
          displacement_(other.displacement_),
          growth_limit_(other.growth_limit_)
    {
        DeepCopy(other.children_);
    }

    StaticOctree& StaticOctree::operator=(StaticOctree const& other)
    {
        if (this == &other)
        {
            return *this;
        }

        bounds_ = other.bounds_;
        stale_ = other.stale_;
        growth_limit_ = other.growth_limit_;
        mass_ = other.mass_;
        displacement_ = other.displacement_;

        DeepCopy(other.children_);

        return *this;
    }

    void StaticOctree::InsertWithoutUpdate(std::shared_ptr<Particle> const& particle) // NOLINT(misc-no-recursion)
    {
        assert(particle != nullptr);

        auto orthant = bounds_.Orthant(particle->Displacement());
        auto& node = children_.at(orthant);

        if (!node) // no node
        {
            node = particle;
        }
        else if (auto existing_particle = std::dynamic_pointer_cast<Particle>(node)) // leaf node
        {
            auto subtree = std::make_shared<StaticOctree>(bounds_.ShrinkTo(orthant), growth_limit_);

            subtree->InsertWithoutUpdate(particle);
            subtree->InsertWithoutUpdate(existing_particle);

            node = subtree;
        }
        else if (auto subtree = std::dynamic_pointer_cast<StaticOctree>(node)) // branch node
        {
            subtree->InsertWithoutUpdate(particle);
        }

        stale_ = true;
    }

    void StaticOctree::UpdateIfStale() // NOLINT(misc-no-recursion)
    {
        if (!stale_)
        {
            return;
        }

        for (auto& node : children_)
        {
            if (!node) // no node
            {
                continue;
            }

            if (auto subtree = std::dynamic_pointer_cast<StaticOctree>(node)) // branch node
            {
                subtree->UpdateIfStale();
            }

            mass_ += node->Mass();
            displacement_ += node->Mass() * node->Displacement();
        }

        displacement_ /= mass_;
        stale_ = false;
    }

    void StaticOctree::DeepCopy(node_array_t const& nodes) // NOLINT(misc-no-recursion)
    {
        for (auto i = 0; i < nodes.size(); i++)
        {
            auto& node = children_[i];
            auto const& other_node = nodes[i];

            if (!other_node)
            {
                node = nullptr;
            }
            else if (std::dynamic_pointer_cast<Particle>(other_node))
            {
                node = other_node; // shallow copy here is fine
            }
            else if (auto subtree = std::dynamic_pointer_cast<StaticOctree>(other_node))
            {
                node = std::make_shared<StaticOctree>(*subtree); // must deep copy StaticOctree
            }
        }
    }

    StaticOctree StaticOctree::ShallowCopy() const
    {
        StaticOctree copy(bounds_);

        copy.stale_ = stale_;
        copy.growth_limit_ = growth_limit_;
        copy.mass_ = mass_;
        copy.displacement_ = displacement_;
        copy.children_ = children_;

        return copy;
    }

    bool StaticOctree::ContainsOrGrown(const Vector &point)
    {
        auto original = this;

        for (auto i = 0U; i < GrowthLimit(); ++i)
        {
            if (bounds_.Contains(point))
            {
                return true;
            }

            // The BoundingBox must be expanded in the direction of the nearest
            // Orthant. If the current bounds were the orthant of a larger
            // BoundingBox, growing from the former to the latter would be
            // equivalent top growing the bounds in the direction of the point.
            // Hence, the nearest orthant inverted gives us the direction the
            // bounds must be expanded in.

            auto subtree = std::make_shared<StaticOctree>(ShallowCopy());
            auto orthant = bounds_.Orthant(point).Invert();

            children_.fill(nullptr);
            children_.at(orthant) = subtree;
            bounds_ = bounds_.ExpandFrom(orthant);
        }

        if (bounds_.Contains(point))
        {
            return true;
        }

        // Failed, so revert to the original via a shallow copy

        *this = original->ShallowCopy();

        return false;
    }
}
