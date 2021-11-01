#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_ORTHANT_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_ORTHANT_H_

#include <cstddef>
#include <concepts>
#include <stdexcept>
#include <type_traits>

#include "gravity/Vector.h"

namespace gravity::barneshut
{
    // Sign of an axis, may be Positive or Negative
    enum class Sign : bool
    {
        Positive,
        Negative
    };

    template<std::integral T>
    T operator<<(Sign sign, T digit)
    {
        return static_cast<T>(sign) << digit;
    }

    // Orthant of a hypercube represented as an index. If each axis in a hypercube is assigned
    // an index from 0 to N, then there are 2^N possible orthants. The index is computed by the
    // sign of each axis.
    template<std::integral T, std::size_t N>
    class Orthant
    {
    public:
        Orthant() = default;

        explicit Orthant(T orthant)
            : orthant_(orthant)
        {
            if (orthant < 0 || orthant > Max())
            {
                throw std::invalid_argument("Invalid orthant");
            }
        }

        // Maximum number of orthants in a hypercube
        static constexpr T Max()
        {
            using limits = std::numeric_limits<T>;

            static_assert(N < limits::digits, "Maximum number of orthants exceeded");

            return 1 << N;
        }

        // Index of the Orthant between 0 and 2^N - 1
        [[nodiscard]] T Index() const { return orthant_; }

        // Make the i-th axis negative
        void Axis(const T digit, Sign sign)
        {
            CheckDigit(digit);
            orthant_ ^= sign << digit;
        }

        // Whether the i-th axis is negative or not
        [[nodiscard]] Sign Axis(const T digit) const
        {
            CheckDigit(digit);
            return static_cast<Sign>(Sign::Negative << digit & orthant_);
        }

    private:
        // The orthant index is computed by mapping the i^th bit to an axis, where sign
        // is given by a 0 or 1. In 2D space: 0 : 0x00 : (+x, +y), 1 : 0x01 : (-x, +y),
        // 2 : 0x10: (+x, -y), 3 : 0x11 : (-x, -y).
        T orthant_{};

        // Throw an exception if the i-th digit is out of range
        static void CheckDigit(const T digit)
        {
#ifndef NDEBUG
            if (digit < 0 || digit >= N)
            {
                throw std::out_of_range("Digit index out of range");
            }
#endif
        }
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_ORTHANT_H_
