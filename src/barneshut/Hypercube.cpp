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

    int Hypercube::Contains(Vector const& point) const
    {
        if (point.size() != Dimensions)
        {
            throw std::invalid_argument("Vector has invalid size");
        }

        // The orthant index is computed by mapping the i^th bit to an axis, where sign
        // is given by a 0 or 1. In 2D space: 0 : 0x00 : (+x, +y), 1 : 0x01 : (-x, +y),
        // 2 : 0x10: (+x, -y), 3 : 0x11 : (-x, -y).

        int orthant{};

        for (int i = 0; i < Dimensions; i++)
        {
            auto const& x = point[i];

            if (x > centre_[i] + width_ / 2.0 || x < centre_[i] - width_ / 2.0)
            {
                throw std::invalid_argument("Point out of bounds");
            }
            else if (x >= centre_[i])
            {
                orthant ^= 1 << i; // Negative half of axis; set i^th bit to one
            }
            else
            {
                // Positive half of axis; i^th bit is already zero
            }
        }

        return orthant;
    }

    Hypercube Hypercube::Orthant(int orthant) const
    {
        if (orthant < 0 || orthant >= Orthants())
        {
            throw std::invalid_argument("Invalid orthant");
        }

        double width = width_ / 2.0;
        Vector centre(Dimensions);

        for (int i = 0; i < Dimensions; i++)
        {
            if (1 << i & orthant) // if the i^th bit is one
            {
                centre[i] = centre_[i] - width / 2.0;
            }
            else // if the i^th bit is zero
            {
                centre[i] = centre_[i] + width / 2.0;
            }
        }

        return { width, centre };
    }
}
