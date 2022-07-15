#ifndef GRAVITY_INCLUDE_GRAVITY_FORCES_IFIELD_H_
#define GRAVITY_INCLUDE_GRAVITY_FORCES_IFIELD_H_

#include "gravity/Particle.h"
#include "gravity/geometry/Vector.h"

namespace gravity::forces
{
    /// IField provides an interface for classes implementing inter-particle
    /// force calculations.
    class IField
    {
    public:
        /// Compute the acceleration that @p subject is subject to due to
        /// @p source and add it to @p acceleration.
        virtual void AddAcceleration(
            Particle const& source, Particle const& subject, geometry::Vector& acceleration) const = 0;

        void operator()(Particle const& source, Particle const& subject, geometry::Vector& acceleration) const;

        /// Return the acceleration that @p subject is subject to due to
        /// @p source.
        [[nodiscard]]
        geometry::Vector Acceleration(Particle const& source, Particle const& subject) const;

        geometry::Vector operator()(Particle const& source, Particle const& subject) const;

        IField() = default;

        virtual ~IField() = default;

    protected:
        // The destructor of this class must be virtual hence the rule of five applies.
        // CppCoreGuidelines C.67 suggests protecting these special members to prevent
        // object slicing while allowing subclasses to implement their own.

        IField(IField const&) = default;
        IField& operator=(IField const&) = default;

        IField(IField&&) noexcept = default;
        IField& operator=(IField&&) noexcept = default;
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_FORCES_IFIELD_H_
