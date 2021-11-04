#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_TREE_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_TREE_H_

#include <array>
#include <memory>
#include <stdexcept>
#include <utility>

#include "gravity/IParticle.h"
#include "gravity/Body.h"
#include "gravity/barneshut/Orthant.h"
#include "gravity/barneshut/Hypercube.h"

namespace gravity::barneshut
{
    /*
     * Barnes-Hut tree. Each leaf node represents a single body with a mass, position, velocity and
     * acceleration. Each branch node (subtree) contains 2 ^ N children contained within an N
     * dimensional hypercube, which is an orthant of its parent's. Each subtree approximates its
     * children by computing their centre of mass.
     */
    class Tree final : public IParticle
    {
    public:
        explicit Tree(Hypercube cube);

        // Total mass of the tree (kg)
        [[nodiscard]] double Mass() const override { return mass_; }

        // Centre of mass of the tree from the origin (m)
        [[nodiscard]] Vector const& Displacement() const override { return displacement_; }

        // Insert a Body into the Tree
        void Insert(std::shared_ptr<Body> const& body);

        // Update the Tree's total mass and centre of mass
        void Update();

    private:
        // Pointer to a particle and a flag for whether it is a leaf node (true) or not (false)
        using node_t = std::pair<std::shared_ptr<IParticle>, bool>;

        // node_t per orthant
        using nodes_t = std::array<node_t, orthant_t::Max()>;

        double mass_{}; // Total mass of the tree's children
        Vector displacement_; // Centre of mass of the tree's children
        nodes_t nodes_; // Child node for each orthant parented by this tree
        Hypercube cube_; // The hypercube that contains this tree's children
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_TREE_H_
