#include "gravity/geometry/BoundingBox.h"

namespace gravity::geometry
{
    namespace
    {
        void ValidateExtents(Vector const& extents)
        {
            if(any_less_than_or_equal_to(extents, 0.0))
            {
                throw std::invalid_argument("Extents must be > 0.0");
            }
        }
    }


    BoundingBox::BoundingBox(Vector const& centre, Vector const& width)
        : extents_(width * 0.5),
        centre_(centre)
    {
        ValidateExtents(extents_);
    }

    void BoundingBox::Extents(Vector const& extents)
    {
        ValidateExtents(extents);
        extents_ = extents;
    }

    bool BoundingBox::Intersects(BoundingBox const& other, double const looseness) const
    {
        // Two bounding boxes, X and Y, intersect if the minima of X are less
        // than or equal to the maxima of Y, and the maxima of X are greater
        // than or equal to the minima of Y. Hence, if the minima of X are
        // greater than the maxima of Y, or the maxima of X are less than the
        // minima of Y, the bounding boxes do not intersect.

        for (auto i = 0U; i < Dimensions; ++i)
        {
            auto half_width = extents_[i];

            if (looseness > 1.0)
            {
                half_width *= looseness;
            }

            auto other_min = other.centre_[i] - other.extents_[i];
            auto other_max = other.centre_[i] + other.extents_[i];
            auto this_min = centre_[i] - half_width;
            auto this_max = centre_[i] + half_width;

            if (this_min > other_max || this_max < other_min)
            {
                return false;
            }
        }

        return true;
    }

    bool BoundingBox::Contains(Vector const& point, double const looseness) const
    {
        for (auto i = 0U; i < Dimensions; ++i)
        {
            auto half_width = extents_[i];

            if (looseness > 1.0)
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

    bool BoundingBox::Contains(BoundingBox const& box, const double looseness) const
    {
        for (auto i = 0U; i < Dimensions; ++i)
        {
            auto half_width = extents_[i];

            if (looseness > 1.0)
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

    Orthant BoundingBox::Orthant(Vector const& point) const
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

    BoundingBox BoundingBox::ShrinkTo(class Orthant orthant) const
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

    BoundingBox BoundingBox::ExpandFrom(class Orthant orthant) const
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
