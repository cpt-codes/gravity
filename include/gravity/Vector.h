#ifndef GRAVITY_INCLUDE_GRAVITY_VECTOR_H_
#define GRAVITY_INCLUDE_GRAVITY_VECTOR_H_

#include <cstddef>

#include <boost/numeric/ublas/vector.hpp>

namespace gravity
{
    // Dimensions of spatial vectors in the simulation
    constexpr std::size_t Dimensions{ 3 };

    // Alias for boost::numeric::ublas
    namespace ublas = boost::numeric::ublas;

    // Boost uBLAS vector of doubles
    using Vector = ublas::vector<double>;
}

#endif //GRAVITY_INCLUDE_GRAVITY_VECTOR_H_
