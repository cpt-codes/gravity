#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_

#include <stdexcept>

#include "gravity/Vector.h"
#include "gravity/barneshut/Orthant.h"

namespace gravity::barneshut
{
    /// Immutable class representing an N dimensional axis-aligned
    /// bounding box (AABB).
    class Hypercube
    {
    public:
        Hypercube() = default;

        /// Hypercube at @p centre with @p width (side-length/double-extents)
        Hypercube(Vector const& centre, Vector const& width);

        /// Extents (half-width) of the @c Hypercube
        [[nodiscard]] Vector const& Extents() const;

        /// Centre of the @c Hypercube
        [[nodiscard]] Vector const& Centre() const;

        /// If the @p point is bounded by (inclusive) @c this @c Hypercube,
        /// returns @c true, otherwise @c false.
        [[nodiscard]] bool Contains(Vector const& point, double looseness = -1.0) const;

        /// If the @p other @c Hypercube is encapsulated (inclusive) by
        /// @c this @c Hypercube, returns @c true, otherwise @c false.
        [[nodiscard]] bool Contains(Hypercube const& box, double looseness = -1.0) const;

        /// Computes the @c Orthant of this @c Hypercube that bounds the
        /// @p point.
        [[nodiscard]] class Orthant Orthant(Vector const& point) const;

        /// Computes the @c Hypercube encapsulated by this @c Hypercube in
        /// the given @c Orthant.
        [[nodiscard]] Hypercube ShrinkTo(class Orthant orthant) const;

        /// Computes the @c Hypercube that contains @c this @c Hypercube
        /// as an @c Orthant.
        [[nodiscard]] Hypercube ExpandFrom(class Orthant orthant) const;

    private:
        Vector extents_; /// Extents of the Hypercube
        Vector centre_; /// Centre of the Hypercube
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_HYPERCUBE_H_
