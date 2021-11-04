#include "gravity/barneshut/Tree.h"

namespace gravity::barneshut
{
    Tree::Tree(Hypercube cube)
        : cube_(std::move(cube))
    {

    }

    void Tree::Insert(std::shared_ptr<Body> const& body)
    {
        if (!body)
        {
            throw std::invalid_argument("Body pointer cannot be null");
        }

        auto orthant = cube_.Contains(body->Displacement());
        auto& [particle, is_leaf] = nodes_[orthant];

        if (!particle) // no node
        {
            particle = body;
            is_leaf = true;
        }
        else if (is_leaf) // leaf node
        {
            auto subtree = std::make_shared<Tree>(cube_.Subdivision(orthant));

            subtree->Insert(body);
            subtree->Insert(std::static_pointer_cast<Body>(particle));

            particle = subtree;
            is_leaf = false;
        }
        else // branch node
        {
            std::static_pointer_cast<Tree>(particle)->Insert(body);
        }
    }

    void Tree::Update()
    {
        for (auto& [particle, is_leaf] : nodes_)
        {
            if (!particle) // no node
            {
                continue;
            }
            else if (!is_leaf) // branch node
            {
                std::static_pointer_cast<Tree>(particle)->Update();
            }

            mass_ += particle->Mass();
            displacement_ += particle->Mass() * particle->Displacement();
        }

        displacement_ /= mass_;
    }
}
