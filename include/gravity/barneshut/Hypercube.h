#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_

#include <cmath>
#include <limits>
#include <stdexcept>

#include "gravity/Vector.h"
#include "gravity/PointMass.h"

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

        // The orthant index of the node the Vector should belong to if it is bounded
        // (inclusive) by the Hypercube. Throws if the Vector isn't bounded by the Hypercube.
        [[nodiscard]] int Contains(Vector const& point) const;

        // Subdivision of the Hypercube
        [[nodiscard]] Hypercube Orthant(int orthant) const;

        // Maximum number of orthants in a Hypercube
        [[nodiscard]] static constexpr int Orthants()
        {
            using limits = std::numeric_limits<int>;

            static_assert(Dimensions < limits::digits, "Maximum number of orthants exceeded");

            return 1 << Dimensions;
        }

    private:
        double width_; // Width of the Hypercube
        Vector centre_; // Centre of the Hypercube
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_
