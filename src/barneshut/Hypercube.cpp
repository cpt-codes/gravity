#include "gravity/barneshut/Hypercube.h"

namespace gravity::barneshut
{
    Hypercube::Hypercube(Vector const& centre, Vector const& width)
        : extents_(width * 0.5),
        centre_(centre)
    {
        if(any_less_than_or_equal_to(extents_, 0.0))
        {
            throw std::invalid_argument("Extents must be > 0.0");
        }
    }

    Vector const& Hypercube::Extents() const
    {
        return extents_;
    }

    Vector const& Hypercube::Centre() const
    {
        return centre_;
    }

    bool Hypercube::Contains(Vector const& point, double const looseness) const
    {
        for (auto i = 0U; i < Dimensions; ++i)
        {
            auto half_width = extents_[i];

            if (looseness > 0.0)
            {
                half_width *= looseness;
            }

            if (point[i] > centre_[i] + half_width
                || point[i] < centre_[i] - half_width)
            {
                return false;
            }
        }

        return true;
    }

    bool Hypercube::Contains(Hypercube const& box, const double looseness) const
    {
        for (auto i = 0U; i < Dimensions; ++i)
        {
            auto half_width = extents_[i];

            if (looseness > 0.0)
            {
                half_width *= looseness;
            }

            auto other_min = box.centre_[i] - box.extents_[i];
            auto other_max = box.centre_[i] + box.extents_[i];
            auto this_min = centre_[i] - half_width;
            auto this_max = centre_[i] + half_width;

            if ((other_min > this_max || other_min < this_min)
                && (other_max > this_max || other_max < this_min))
            {
                return false;
            }
        }

        return true;
    }

    Orthant Hypercube::Orthant(Vector const& point) const
    {
        class Orthant orthant;

        for (auto i = 0U; i < Dimensions; ++i)
        {
            if (point[i] >= centre_[i])
            {
                orthant.AlignAxis(i, true); // positive
            }
            else
            {
                orthant.AlignAxis(i, false); // negative
            }
        }

        return orthant;
    }

    Hypercube Hypercube::ShrinkTo(class Orthant orthant) const
    {
        auto extents = extents_ / 2.0;
        auto centre(centre_);

        for (auto i = 0U; i < Dimensions; ++i)
        {
            if (orthant.IsAxisAligned(i)) // positive
            {
                centre[i] += extents[i];
            }
            else // negative
            {
                centre[i] -= extents[i];
            }
        }

        return { centre, extents };
    }

    Hypercube Hypercube::ExpandFrom(class Orthant orthant) const
    {
        auto extents = extents_ * 2.0;
        auto centre(centre_);

        for (auto i = 0U; i < Dimensions; ++i)
        {
            if (orthant.IsAxisAligned(i)) // positive
            {
                centre[i] -= extents[i];
            }
            else // negative
            {
                centre[i] += extents[i];
            }
        }

        return { centre, extents };
    }
}
