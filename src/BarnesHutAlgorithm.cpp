#include "gravity/BarnesHutAlgorithm.h"

namespace gravity
{
    BarnesHutAlgorithm::BarnesHutAlgorithm(std::unique_ptr<Octree> octree,
                                           std::unique_ptr<forces::IField> field,
                                           double const threshold) :
        threshold_(threshold),
        tree_(std::move(octree)),
        field_(std::move(field))
    {

    }

    geometry::Vector BarnesHutAlgorithm::Acceleration(std::shared_ptr<Particle const> const& particle) const
    {
        std::shared_lock lock(mutex_);

        if (!particle || !tree_ || !field_)
        {
            return {};
        }

        return Acceleration(tree_->Root(), *particle);
    }

    geometry::Vector BarnesHutAlgorithm::Force(std::shared_ptr<Particle const> const& particle) const
    {
        std::shared_lock lock(mutex_);

        if (!particle || !tree_ || !field_)
        {
            return {};
        }

        return particle->Mass() * Acceleration(tree_->Root(), *particle);
    }

    double BarnesHutAlgorithm::ApproximationThreshold() const
    {
        std::shared_lock lock(mutex_);

        return threshold_;
    }

    void BarnesHutAlgorithm::ApproximationThreshold(double const threshold)
    {
        std::lock_guard lock(mutex_);

        if (threshold < 0.0)
        {
            throw std::invalid_argument("Barnes-Hut algorithm threshold must be >= 0.0");
        }

        threshold_ = threshold;
    }

    std::unique_ptr<Octree> BarnesHutAlgorithm::Tree()
    {
        std::lock_guard lock(mutex_);

        // The caller may modify the Octree. In which case, the cache must be
        // cleared to avoid any invalid cached results where Octree nodes get
        // re-allocated and memory addresses re-used.

        mass_calculator_.ClearCache();

        return std::move(tree_);
    }

    void BarnesHutAlgorithm::Tree(std::unique_ptr<Octree> octree)
    {
        std::lock_guard lock(mutex_);

        mass_calculator_.ClearCache();

        tree_ = std::move(octree);
    }

    std::unique_ptr<forces::IField> BarnesHutAlgorithm::Field()
    {
        std::lock_guard lock(mutex_);

        return std::move(field_);
    }

    void BarnesHutAlgorithm::Field(std::unique_ptr<forces::IField> field)
    {
        std::lock_guard lock(mutex_);

        field_ = std::move(field);
    }

    void BarnesHutAlgorithm::Update()
    {
        std::lock_guard lock(mutex_);

        if (!tree_)
        {
            return;
        }

        mass_calculator_.ClearCache();

        auto removed_particles = tree_->Update();
    }

    bool BarnesHutAlgorithm::ShouldApproximate(geometry::Vector const& point,
                                               geometry::BoundingBox const& bounds) const
    {
        auto distance = geometry::ublas::norm_2(point - bounds.Centre());

        return geometry::any_less_than(bounds.Extents(), threshold_ * distance);
    }

    geometry::Vector
    BarnesHutAlgorithm::Acceleration(Particle const& source, Particle const& subject) const
    {
        assert(field_ != nullptr);

        return field_->Acceleration(source, subject);
    }

    geometry::Vector
    BarnesHutAlgorithm::Acceleration(MassCalculator::PointMass const& source,
                                     Particle const& subject) const
    {
        Particle source_particle;

        source_particle.Mass() = source.mass;
        source_particle.Displacement() = source.displacement;

        return Acceleration(source_particle, subject);
    }

    geometry::Vector
    BarnesHutAlgorithm::Acceleration(Node const& node, // NOLINT(misc-no-recursion)
                                     Particle const& particle) const

    {
        if (ShouldApproximate(particle.Displacement(), node.Bounds()))
        {
            return Acceleration(mass_calculator_(node), particle);
        }

        geometry::Vector acceleration;

        for (auto const& other_particle : node.Particles())
        {
            if (other_particle)
            {
                acceleration += Acceleration(*other_particle, particle);
            }
        }

        for (auto const& child : node.Children())
        {
            acceleration += Acceleration(child, particle);
        }

        return acceleration;
    }
}