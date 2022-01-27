#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_

#include <array>
#include <memory>
#include <stdexcept>
#include <utility>

#include "gravity/IParticle.h"
#include "gravity/Particle.h"
#include "gravity/IGravity.h"
#include "gravity/Plummer.h"
#include "gravity/barneshut/Orthant.h"
#include "gravity/barneshut/Hypercube.h"

namespace gravity::barneshut
{
    /*
     * Barnes-Hut tree. Each leaf node represents a single particle with a mass, position, velocity and
     * acceleration. Each branch node (subtree) contains 2 ^ N children contained within an N
     * dimensional hypercube, which is an orthant of its parent's. Each subtree approximates its
     * children by computing their centre of mass.
     */
    class Octree final : public IParticle
    {
    public:
        explicit Octree(Hypercube cube);

        // Total mass of all the particles in the tree
        [[nodiscard]] double Mass() const override;

        // Centre of mass of all the particles in the tree from the origin
        [[nodiscard]] Vector const& Displacement() const override;

        // Insert a Particle into the Octree
        void Insert(std::shared_ptr<Particle> const& particle);

        // Update the Octree's total mass and centre of mass
        void Update(bool force = false);

        /// @brief Compute the force acting on the given Particle using the Barnes-Hut algorithm.
        /// @param particle Particle to compute forces (acceleration) on.
        /// @param threshold Determines the accuracy of the force computations.
        /// @param gravity Method to compute gravitational forces.
        void ComputeAcceleration(std::shared_ptr<Particle> const& particle, double threshold,
                          IGravity const& gravity = Plummer()) const;

    private:
        // Pointer to a particle and a flag for whether it is a leaf node (true) or not (false)
        using node_t = std::pair<std::shared_ptr<IParticle>, bool>;

        // node_t per orthant
        using nodes_t = std::array<node_t, orthant_t::Max()>;

        bool stale_{}; // State of the tree's centre of mass is up-to-date
        double mass_{}; // Total mass of the tree's children
        Vector displacement_; // Centre of mass of the tree's children
        nodes_t nodes_; // Child node for each orthant parented by this tree
        Hypercube cube_; // The hypercube that contains this tree's children
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_OCTREE_H_
