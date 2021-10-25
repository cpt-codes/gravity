#ifndef GRAVITY_INCLUDE_GRAVITY_BARNESHUT_ORTHANT_H_
#define GRAVITY_INCLUDE_GRAVITY_BARNESHUT_ORTHANT_H_

#include <cstddef>
#include <stdexcept>
#include <type_traits>

#include "gravity/Vector.h"

namespace gravity::barneshut
{
    // Orthant of a hypercube represented as an index. If each axis in a hypercube is assigned
    // an index from 0 to N, then there are 2^N possible orthants. The index is computed by the
    // sign of each axis.
    template<typename T, std::size_t N>
    class Orthant
    {
    public:
        static_assert(std::is_integral_v<T>, "T must be integral type");

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

        // Sign of an axis, may be Positive or Negative
        enum class Axis : bool
        {
            Positive,
            Negative
        };

        // Make the i-th axis negative
        void Axis(const T i, enum Axis sign)
        {
            CheckDigit(i);
            orthant_ ^= static_cast<T>(sign) << i;
        }

        // Whether the i-th axis is negative or not
        [[nodiscard]] enum Axis Axis(const T i) const
        {
            CheckDigit(i);
            return static_cast<enum Axis>(static_cast<T>(Axis::Negative) << i & orthant_);
        }

    private:
        // The orthant index is computed by mapping the i^th bit to an axis, where sign
        // is given by a 0 or 1. In 2D space: 0 : 0x00 : (+x, +y), 1 : 0x01 : (-x, +y),
        // 2 : 0x10: (+x, -y), 3 : 0x11 : (-x, -y).
        int orthant_{};

        // Throw an exception if the i-th digit is out of range
        static void CheckDigit(const T i)
        {
#ifndef NDEBUG
            if (i < 0 || i >= N)
            {
                throw std::out_of_range("Digit index out of range");
            }
#endif
        }
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_BARNESHUT_ORTHANT_H_
