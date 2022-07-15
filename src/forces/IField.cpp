#include "gravity/forces/IField.h"

namespace gravity::forces
{
    void IField::operator()(Particle const& source, Particle const& subject, geometry::Vector& acceleration) const
    {
        AddAcceleration(source, subject, acceleration);
    }

    geometry::Vector IField::Acceleration(Particle const& source, Particle const& subject) const
    {
        geometry::Vector acceleration;

        AddAcceleration(source, subject, acceleration);

        return acceleration;
    }

    geometry::Vector IField::operator()(Particle const& source, Particle const& subject) const
    {
        return Acceleration(source, subject);
    }
}
