#ifndef GRAVITY_INCLUDE_GRAVITY_GEOMETRY_ISHAPE_H_
#define GRAVITY_INCLUDE_GRAVITY_GEOMETRY_ISHAPE_H_

#include "gravity/geometry/BoundingBox.h"

namespace gravity::geometry
{
    /// @c IShape is a base class for N dimensional objects whose shape is
    /// encapsulated by a @c BoundingBox.
    class IShape
    {
    public:
        /// The @c BoundingBox that encapsulates this shape
        [[nodiscard]] virtual BoundingBox const& Bounds() const = 0;

        IShape() = default;
        virtual ~IShape() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five
        // applies. CppCoreGuidelines C.67 suggests protecting these special
        // members to prevent object slicing while allowing subclasses to
        // implement their own.

        IShape(IShape const&) = default;
        IShape& operator=(IShape const&) = default;

        IShape(IShape&&) noexcept = default;
        IShape& operator=(IShape&&) noexcept = default;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_GEOMETRY_ISHAPE_H_
