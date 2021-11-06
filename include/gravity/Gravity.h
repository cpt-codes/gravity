#ifndef GRAVITY_INCLUDE_GRAVITY_EQUATIONS_H_
#define GRAVITY_INCLUDE_GRAVITY_EQUATIONS_H_

#include <cmath>

#include "gravity/Vector.h"
#include "gravity/IParticle.h"

namespace gravity
{
    class Gravity final
    {
    public:
        explicit Gravity(double g = UniGravConst)
            : grav_const_(g) {}

        // Compute the Newtonian gravitational field (acceleration) at displacement x from the origin
        // due to particle p using Newton's law of universal gravitation. The results are added to
        // the field vector in-place.
        void Newtonian(Vector const& x, IParticle const& p, Vector& field) const;

        // Compute the gravitational field (acceleration) at displacement x from the origin due to
        // particle p using the Plummer model. The results are added to the field vector in-place.
        void Plummer(Vector const& x, IParticle const& p, Vector& field) const;

        [[nodiscard]] double GravConst() const { return grav_const_; }

        [[nodiscard]] double Epsilon() const { return epsilon_; }

        // Universal Gravitational Constant. Source: https://physics.nist.gov/cgi-bin/cuu/Value?bg
        static constexpr double UniGravConst{ 6.67430e10-11 };

    private:
        double grav_const_; // Gravitational constant
        double epsilon_{}; // Plummer radius
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_EQUATIONS_H_
