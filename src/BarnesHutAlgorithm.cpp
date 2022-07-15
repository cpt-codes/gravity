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

        geometry::Vector acceleration;

        AddAcceleration(tree_->Root(), particle, acceleration);

        return acceleration;
    }

    geometry::Vector BarnesHutAlgorithm::Force(std::shared_ptr<Particle const> const& particle) const
    {
        if (!particle)
        {
            return {};
        }

        return particle->Mass() * Acceleration(particle);
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

    std::list<std::shared_ptr<Particle>>
        BarnesHutAlgorithm::Update(std::shared_ptr<threads::ThreadPool> const& pool)
    {
        std::lock_guard lock(mutex_);

        if (!tree_)
        {
            return {};
        }

        mass_calculator_.ClearCache();

        return tree_->Update(pool);
    }

    bool BarnesHutAlgorithm::ShouldApproximate(geometry::Vector const& point,
                                               geometry::BoundingBox const& bounds) const
    {
        auto distance = geometry::ublas::norm_2(point - bounds.Centre());

        return geometry::any_less_than(bounds.Extents(), threshold_ * distance);
    }

    void BarnesHutAlgorithm::AddAcceleration(Particle const& source,
                                             Particle const& subject,
                                             geometry::Vector& acceleration) const
    {
        assert(field_ != nullptr);

        field_->AddAcceleration(source, subject, acceleration);
    }

    void BarnesHutAlgorithm::AddAcceleration(MassCalculator::PointMass const &source,
                                             Particle const &subject,
                                             geometry::Vector& acceleration) const
    {
        Particle source_particle;

        source_particle.Mass() = source.mass;
        source_particle.Displacement() = source.displacement;

        AddAcceleration(source_particle, subject, acceleration);
    }

    void BarnesHutAlgorithm::AddAcceleration(Node const& node, // NOLINT(misc-no-recursion)
                                             std::shared_ptr<Particle const> const& particle,
                                             geometry::Vector& acceleration) const
    {
        if (ShouldApproximate(particle->Displacement(), node.Bounds()))
        {
            AddAcceleration(mass_calculator_(node), *particle, acceleration);
        }

        for (auto const& other_particle : node.Particles())
        {
            if (other_particle && particle != other_particle)
            {
                AddAcceleration(*other_particle, *particle, acceleration);
            }
        }

        for (auto const& child : node.Children())
        {
            AddAcceleration(child, particle, acceleration);
        }
    }
}