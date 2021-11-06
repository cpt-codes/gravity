#include "gravity/Gravity.h"

namespace gravity
{
    void Gravity::Newtonian(Vector const& x, IParticle const& p, Vector& field) const
    {
        auto r = x - p.Displacement(); // displacement relative to particle
        auto R = ublas::norm_2(r); // distance relative to particle

        // Newton's universal law of gravitation
        field += -grav_const_ * p.Mass() * x / std::pow(R, 3);
    }

    void Gravity::Plummer(Vector const& x, IParticle const& p, Vector& field) const
    {
        auto r = x - p.Displacement(); // displacement relative to particle
        auto R = ublas::norm_2(r); // distance relative to particle

        // Gravitational field from the Plummer model
        field += -grav_const_ * p.Mass() * r / std::pow(std::pow(R, 2) + std::pow(epsilon_, 2), 1.5);
    }
}
