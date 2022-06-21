#ifndef GRAVITY_INCLUDE_GRAVITY_GEOMETRY_ORTHANT_H_
#define GRAVITY_INCLUDE_GRAVITY_GEOMETRY_ORTHANT_H_

#include <bitset>
#include <cstddef>
#include <limits>

#include "gravity/geometry/Vector.h"

namespace gravity::geometry
{
    /// @brief
    ///     @c Orthant represents an orthant of an N dimensional box, which
    ///     is determined by the orthogonality of each axis (+-, +-, +-,...)
    ///     bounding the orthant.
    /// @details
    ///     An N dimensional box can be divided into 2 ** N orthants. Each
    ///     @c Orthant can be mapped to a index from 0 to 2 ** N - 1. The
    ///     alignment of each axis (aligned/positive or anti-aligned/negative)
    ///     is used to calculate the index of each orthant. @c Dimensions is
    ///     used as the dimensions of this class.
    class Orthant
    {
    public:
        Orthant() = default;

        /// Converting constructor from an unsigned integral type.
        template<std::unsigned_integral T>
        Orthant(T const orthant) noexcept // NOLINT(google-explicit-constructor)
            : orthant_(orthant)
        {}

        /// The number of orthants in an N dimensional box.
        static constexpr auto Max() noexcept
        {
            static_assert(Dimensions > 0, "Dimensions cannot be zero.");

            static_assert(
                Dimensions < std::numeric_limits<std::size_t>::digits,
                "Number of digits required cannot exceed that of the target architecture");

            return 1 << Dimensions;
        }

        /// Set the alignment of the i-th axis. @c true if aligned/positive,
        /// @c false if anti-aligned/negative.
        Orthant& AlignAxis(const std::size_t digit, bool aligned = true)
        {
            orthant_.set(digit, !aligned);
            return *this;
        }

        /// Returns the alignment of the i-th axis. @c true if aligned/positive,
        /// @c false if anti-aligned/negative.
        [[nodiscard]]
        bool IsAxisAligned(const std::size_t digit) const
        {
            return !orthant_.test(digit);
        }

        /// Invert all axes alignments. This will mirror the @c Orthant.
        Orthant& Invert() noexcept
        {
            orthant_.flip();
            return *this;
        }

        /// Implicit conversion function to std::size_t for indexing arrays.
        operator std::size_t() const // NOLINT(google-explicit-constructor)
        {
            return orthant_.to_ullong();
        }

    private:
        /// @brief
        ///     @c std::bitset representing the state of each axis (+-, +-, +-,...).
        /// @details
        ///     Each axis is represented by a single bit, whose alignment is either
        ///     aligned/positive or anti-aligned/negative. These states are stored
        ///     as 0s and 1s respectively. This is done so zero-initialisation
        ///     corresponds to all axis initialising in the aligned state.
        /// @example
        ///     For example, in 2D:
        ///         0 : 0b00 : (+x, +y),
        ///         1 : 0b01 : (-x, +y),
        ///         2 : 0b10 : (+x, -y),
        ///         3 : 0b11 : (-x, -y)
        std::bitset<Dimensions> orthant_{};
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_GEOMETRY_ORTHANT_H_
