#ifndef GRAVITY_INCLUDE_GRAVITY_GEOMETRY_VECTOR_H_
#define GRAVITY_INCLUDE_GRAVITY_GEOMETRY_VECTOR_H_

#include <algorithm>
#include <cstddef>
#include <concepts>

#include <boost/numeric/ublas/vector.hpp>

namespace gravity::geometry
{
    // Dimensions of spatial vectors in the simulation
    constexpr std::size_t Dimensions{ 3 };

    // Alias for boost::numeric::ublas
    namespace ublas = boost::numeric::ublas;

    // Boost uBLAS vector of doubles
    using Vector = ublas::vector<double, ublas::bounded_array<double, Dimensions>>;

    // Arithmetic concept
    template <typename T>
    concept arithmetic = std::is_arithmetic_v<T>;

    // True if any of the vector's elements are less than the
    // scalar, otherwise false
    template<arithmetic T>
    bool any_less_than(Vector const& vec, T scalar)
    {
        auto const less_than = [scalar](auto const value)
        {
            return value < scalar;
        };

        return std::ranges::any_of(vec, less_than);
    }

    // True if any of the vector's elements are less than or equal to the
    // scalar, otherwise false
    template<arithmetic T>
    bool any_less_than_or_equal_to(Vector const& vec, T scalar)
    {
        auto const less_than = [scalar](auto const value)
        {
            return value <= scalar;
        };

        return std::ranges::any_of(vec, less_than);
    }
}

#endif //GRAVITY_INCLUDE_GRAVITY_GEOMETRY_VECTOR_H_
