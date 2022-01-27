#ifndef GRAVITY_INCLUDE_GRAVITY_PLUMMER_H_
#define GRAVITY_INCLUDE_GRAVITY_PLUMMER_H_

#include "gravity/IGravity.h"

namespace gravity
{
    class Plummer final : public IGravity
    {
    public:
        [[nodiscard]] Vector Acceleration(IParticle const& p0, Particle const& p1) const override
        {
            auto r = p0.Displacement() - p1.Displacement();
            auto R = ublas::norm_2(r);

            return -GravConst() * p0.Mass() * r
                / std::pow(std::pow(R, 2) + std::pow(radius_, 2), 1.5);
        }

        // The Plummer radius. Used as a softening length in n-body simulations.
        [[nodiscard]] double Radius() const { return radius_; }
        [[nodiscard]] double& Radius() { return radius_; }

    private:
        double radius_{ 1.0 };
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_PLUMMER_H_
