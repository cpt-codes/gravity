#include "gravity/barneshut/Hypercube.h"

namespace gravity::barneshut
{
    Hypercube::Hypercube(double const width, Vector const& centre)
        : width_(width), centre_(centre)
    {
        if (width <= 0.0 || !std::isfinite(width))
        {
            throw std::invalid_argument("Width must finite and > 0.0");
        }

        if (centre.size() != Dimensions)
        {
            throw std::invalid_argument("Vector has invalid size");
        }
    }

    orthant_t Hypercube::Contains(Vector const& point) const
    {
        if (point.size() != Dimensions)
        {
            throw std::invalid_argument("Vector has invalid size");
        }

        orthant_t orthant;

        for (int i = 0; i < Dimensions; i++)
        {
            auto const& x = point[i];

            if (x > centre_[i] + width_ / 2.0 || x < centre_[i] - width_ / 2.0)
            {
                throw std::invalid_argument("Point out of bounds");
            }
            else if (x >= centre_[i])
            {
                orthant.Axis(i, Sign::Negative);
            }
            else
            {
                orthant.Axis(i, Sign::Positive);
            }
        }

        return orthant;
    }

    Hypercube Hypercube::ToOrthant(orthant_t const& orthant) const
    {
        double width = width_ / 2.0;

        Vector centre(Dimensions);

        for (int i = 0; i < Dimensions; i++)
        {
            if (orthant.Axis(i) == Sign::Negative)
            {
                centre[i] = centre_[i] - width / 2.0;
            }
            else
            {
                centre[i] = centre_[i] + width / 2.0;
            }
        }

        return { width, centre };
    }
}
