#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_BOUNDINGBOX_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_BOUNDINGBOX_H_

#include <algorithm>
#include <stdexcept>

#include "gravity/Vector.h"
#include "gravity/barneshut/Orthant.h"

namespace gravity::barneshut
{
    /// Immutable class representing an N dimensional axis-aligned
    /// bounding box (AABB).
    class BoundingBox
    {
    public:
        BoundingBox() = default;

        /// BoundingBox at @p centre with @p width (side-length/double-extents)
        BoundingBox(Vector const& centre, Vector const& width);

        /// Extents (half-width) of the @c BoundingBox
        [[nodiscard]]
        Vector const& Extents() const { return extents_; }

        /// Centre of the @c BoundingBox
        [[nodiscard]]
        Vector const& Centre() const { return centre_; }

        /// Returns @c true if @c this intersect with @p other, @c false otherwise.
        [[nodiscard]]
        bool Intersects(BoundingBox const& other, double looseness = 1.0) const;

        /// If the @p point is bounded by (inclusive) @c this @c BoundingBox,
        /// returns @c true, otherwise @c false.
        [[nodiscard]]
        bool Contains(Vector const& point, double looseness = 1.0) const;

        /// If the @p other @c BoundingBox is encapsulated (inclusive) by
        /// @c this @c BoundingBox, returns @c true, otherwise @c false.
        [[nodiscard]]
        bool Contains(BoundingBox const& box, double looseness = 1.0) const;

        /// Computes the @c Orthant of this @c BoundingBox that bounds the
        /// @p point.
        [[nodiscard]]
        class Orthant Orthant(Vector const& point) const;

        /// Computes the @c BoundingBox encapsulated by this @c BoundingBox in
        /// the given @c Orthant.
        [[nodiscard]]
        BoundingBox ShrinkTo(class Orthant orthant) const;

        /// Computes the @c BoundingBox that contains @c this @c BoundingBox
        /// as an @c Orthant.
        [[nodiscard]]
        BoundingBox ExpandFrom(class Orthant orthant) const;

    private:
        Vector extents_; ///< Extents of the BoundingBox
        Vector centre_; ///< Centre of the BoundingBox
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_BOUNDINGBOX_H_
