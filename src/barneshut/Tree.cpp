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

        void ThrowIfNull(std::shared_ptr<Body> const& p)
        {
            if (!p)
            {
                throw std::invalid_argument("Body pointer cannot be null");
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

    void Tree::Insert(std::shared_ptr<Body> const& body)
    {
        ThrowIfNull(body);

        auto orthant = cube_.Contains(body->Displacement());
        auto& [node, is_leaf] = nodes_[orthant];

        if (!node) // no node
        {
            node = body;
            is_leaf = true;
        }
        else if (is_leaf) // leaf node
        {
            auto subtree = std::make_shared<Tree>(cube_.Subdivision(orthant));

            subtree->Insert(body);
            subtree->Insert(std::static_pointer_cast<Body>(node));

            node = subtree;
            is_leaf = false;
        }
        else // branch node
        {
            std::static_pointer_cast<Tree>(node)->Insert(body);
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
}
