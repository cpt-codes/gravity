#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_

#include <cmath>
#include <limits>
#include <stdexcept>

#include "gravity/Vector.h"
#include "gravity/barneshut/Orthant.h"

namespace gravity::barneshut
{
    // "Immutable" class representing an N dimensional hypercube.
    class Hypercube
    {
    public:
        // Hypercube with width and centre
        Hypercube(double width, Vector const& centre);

        // Width of the Hypercube in all dimensions
        [[nodiscard]] double Width() const { return width_; }

        // Centre of the Hypercube
        [[nodiscard]] Vector const& Centre() const { return centre_; }

        // If the point is bounded (inclusive) by the Hypercube return it's orthant.
        // Throws if the Vector isn't bounded by the Hypercube.
        [[nodiscard]] orthant_t Contains(Vector const& point) const;

        // Orthant subdivision of this Hypercube
        [[nodiscard]] Hypercube Subdivision(orthant_t orthant) const;

    private:
        double width_; // Width of the Hypercube
        Vector centre_; // Centre of the Hypercube
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_
