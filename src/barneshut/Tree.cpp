#include "gravity/barneshut/Tree.h"

namespace gravity::barneshut
{
    Tree::Tree(Hypercube cube)
        : cube_(std::move(cube))
    {

    }

    namespace
    {
        void ThrowIfStale(bool stale)
        {
            if (stale)
            {
                throw std::logic_error("Centre of mass must be updated");
            }
        }

        void ThrowIfNull(std::shared_ptr<Particle> const& ptr)
        {
            if (!ptr)
            {
                throw std::invalid_argument("Particle pointer cannot be null");
            }
        }
    }

    double Tree::Mass() const
    {
        ThrowIfStale(stale_);
        return mass_;
    }

    Vector const& Tree::Displacement() const
    {
        ThrowIfStale(stale_);
        return displacement_;
    }

    void Tree::Insert(std::shared_ptr<Particle> const& particle)
    {
        ThrowIfNull(particle);

        auto orthant = cube_.Contains(particle->Displacement());
        auto& [node, is_leaf] = nodes_[orthant];

        if (!node) // no node
        {
            node = particle;
            is_leaf = true;
        }
        else if (is_leaf) // leaf node
        {
            auto subtree = std::make_shared<Tree>(cube_.Subdivision(orthant));

            subtree->Insert(particle);
            subtree->Insert(std::static_pointer_cast<Particle>(node));

            node = subtree;
            is_leaf = false;
        }
        else // branch node
        {
            std::static_pointer_cast<Tree>(node)->Insert(particle);
        }

        stale_ = true;
    }

    void Tree::Update(bool const force)
    {
        if (!force && !stale_)
        {
            return;
        }

        for (auto& [node, is_leaf] : nodes_)
        {
            if (!node) // no node
            {
                continue;
            }
            else if (!is_leaf) // branch node
            {
                std::static_pointer_cast<Tree>(node)->Update();
            }

            mass_ += node->Mass();
            displacement_ += node->Mass() * node->Displacement();
        }

        displacement_ /= mass_;
        stale_ = false;
    }

    void Tree::ComputeAcceleration(std::shared_ptr<Particle> const& particle, double const threshold,
                            IGravity const& gravity) const
    {
        ThrowIfStale(stale_);
        ThrowIfNull(particle);

        if (threshold < 0.0)
        {
            throw std::invalid_argument("Approximation threshold < 0.0");
        }

        for (auto& [node, is_leaf] : nodes_)
        {
            if (node == particle)
            {
                continue;
            }
            else if(is_leaf) // leaf node
            {
                particle->Acceleration() += gravity.Acceleration(*node, *particle);
            }
            else // branch node
            {
                auto distance = ublas::norm_2(particle->Displacement() - node->Displacement());

                if (cube_.Width() / distance < threshold) // Approximate using centre of mass
                {
                    particle->Acceleration() += gravity.Acceleration(*node, *particle);
                }
                else // Recursively add forces
                {
                    std::static_pointer_cast<Tree>(node)->ComputeAcceleration(particle, threshold, gravity);
                }
            }
        }
    }
}
