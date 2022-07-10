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
        /// Compute the acceleration that @p subject is subject to due to @p source.
        [[nodiscard]]
        virtual geometry::Vector Acceleration(Particle const& source, Particle const& subject) const = 0;

        /// Compute the force that @p subject is subject to due to @p source.
        [[maybe_unused, nodiscard]]
        geometry::Vector Force(Particle const& source, Particle const& subject) const
        {
            return subject.Mass() * Acceleration(source, subject);
        }

        /// Compute the field that @p subject is subject to due to @p source.
        [[nodiscard]]
        geometry::Vector operator()(Particle const& source, Particle const& subject) const
        {
            return Acceleration(source, subject);
        }

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
