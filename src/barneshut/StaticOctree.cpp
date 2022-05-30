#include "gravity/barneshut/StaticOctree.h"

namespace gravity::barneshut
{
    unsigned int StaticOctree::DefaultGrowthLimit = 10;

    StaticOctree::StaticOctree(BoundingBox bounds)
        : bounds_(std::move(bounds))
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

    void StaticOctree::Build(std::vector<std::shared_ptr<Particle>> const& particles)
    {
        for (auto const& particle : particles)
        {
            if (!particle)
            {
                continue;
            }

            if(!GrowToFit(particle->Displacement()))
            {
                throw std::runtime_error("Could not grow StaticOctree to fit particle.");
            }

            InsertWithoutUpdate(particle);
        }

        UpdateIfNeeded();
    }

    bool StaticOctree::Insert(std::shared_ptr<Particle> const& particle)
    {
        if (!particle || !bounds_.Contains(particle->Displacement()))
        {
            return false;
        }

        InsertWithoutUpdate(particle);
        UpdateIfNeeded();

        return true;
    }

    bool StaticOctree::GrowToFit(const Vector &point, unsigned int limit)
    {
        for (auto i = 0U; i < limit; ++i)
        {
            if (bounds_.Contains(point))
            {
                return true;
            }

            auto subtree = std::make_shared<StaticOctree>(ShallowCopy());
            auto orthant = bounds_.Orthant(point).Invert();

            children_.fill(nullptr);
            children_.at(orthant) = subtree;
            bounds_ = bounds_.ExpandFrom(orthant);
        }

        return bounds_.Contains(point);
    }

    StaticOctree::StaticOctree(StaticOctree const& other)
        : bounds_(other.bounds_),
          stale_(other.stale_),
          mass_(other.mass_),
          displacement_(other.displacement_)
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
        else if (auto existing_particle= std::dynamic_pointer_cast<Particle>(node)) // leaf node
        {
            auto subtree = std::make_shared<StaticOctree>(bounds_.ShrinkTo(orthant));

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

    void StaticOctree::UpdateIfNeeded() // NOLINT(misc-no-recursion)
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

            if (auto subtree= std::dynamic_pointer_cast<StaticOctree>(node)) // branch node
            {
                subtree->UpdateIfNeeded();
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
            auto& node= children_[i];
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
        copy.mass_ = mass_;
        copy.displacement_ = displacement_;
        copy.children_ = children_;

        return copy;
    }
}
