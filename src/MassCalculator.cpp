#include "gravity/MassCalculator.h"

namespace gravity
{
    void MassCalculator::ClearCache()
    {
        std::lock_guard lock(cache_mutex_);

        cache_.clear();
    }

    void MassCalculator::ClearCache(Node const& node)
    {
        std::lock_guard lock(cache_mutex_);

        cache_.unsafe_erase(&node);
    }

    MassCalculator::PointMass
    MassCalculator::Calculate(Node const& node) // NOLINT(misc-no-recursion)
    {
        std::shared_lock shared_lock(cache_mutex_);

        return FindOrCalculate(node);
    }

    MassCalculator::PointMass
    MassCalculator::WaitForResult(CacheEntry& entry)
    {
        std::unique_lock lock(entry.mutex);

        entry.until_cached.wait(lock, [&entry]()
        {
            return entry.is_cached;
        });

        return entry.result;
    }

    MassCalculator::PointMass
    MassCalculator::FindOrCalculate(Node const& node) // NOLINT(misc-no-recursion)
    {
        auto it = cache_.find(&node);

        if (it == cache_.end())
        {
            return CalculateAndCache(node);
        }

        return WaitForResult(it->second);
    }

    MassCalculator::PointMass
    MassCalculator::CalculateAndCache(Node const& node) // NOLINT(misc-no-recursion)
    {
        // We construct the CacheEntry in-place using std::piecewise_construct
        // because the mutex is non-copyable and non-movable.

        auto [it, success] = cache_.emplace(std::piecewise_construct,
                                            std::forward_as_tuple(&node),
                                            std::forward_as_tuple());

        // The operation may fail if multiple threads attempt to emplace the
        // element simultaneously. In which case, we must wait until the thread
        // doing the computation is done.

        if (!success)
        {
            return WaitForResult(it->second);
        }

        // Otherwise, this thread successfully constructed the CacheEntry, so
        // it calculates the PointMass for the Octree.

        PointMass point_mass;

        {
            std::lock_guard lock(it->second.mutex);

            Calculate(node, point_mass);

            it->second.result = point_mass;
            it->second.is_cached = true;
        }

        it->second.until_cached.notify_all();

        return point_mass;
    }

    void MassCalculator::Calculate(Node const& node, // NOLINT(misc-no-recursion)
                                   PointMass& point_mass)
    {
        for (auto const& child: node.Children())
        {
            auto [mass, displacement] = FindOrCalculate(child);

            point_mass.mass += mass;
            point_mass.displacement += mass * displacement;
        }

        for (auto const& particle: node.Particles())
        {
            point_mass.mass += particle->Mass();
            point_mass.displacement += particle->Mass() * particle->Displacement();
        }

        if (point_mass.mass != 0.0)
        {
            point_mass.displacement /= point_mass.mass;
        }
    }
}