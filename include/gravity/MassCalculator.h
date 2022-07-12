#ifndef GRAVITY_INCLUDE_GRAVITY_MASSCALCULATOR_H_
#define GRAVITY_INCLUDE_GRAVITY_MASSCALCULATOR_H_

#include <condition_variable>
#include <mutex>
#include <shared_mutex>

#include <tbb/concurrent_unordered_map.h>

#include "gravity/geometry/Vector.h"
#include "gravity/Octree.h"

namespace gravity
{
    /// @brief
    ///     A class that computes (and caches) the total mass and centre of
    ///     mass of Octree nodes. The calculator is thread-safe.
    /// @details
    ///     Results are cached only when the cache does not have an entry
    ///     for the given Octree node. All member functions are synchronised
    ///     with minimal locking to optimise performance.
    class MassCalculator
    {
    public:
        /// A point particle representing the total mass and centre of mass
        /// of all particles within an Octree.
        struct PointMass
        {
            double mass{}; ///< Total mass of all particles within the Octree
            geometry::Vector displacement; ///< Centre of mass of particles within the Octree
        };

        /// Remove all calculation results from the cache.
        void ClearCache();

        /// Remove the cached result for the given Octree only. Cached results
        /// for this Octree's children will not be removed.
        [[maybe_unused]]
        void ClearCache(Octree const& octree);

        /// @brief
        ///     Computes, caches and returns the total mass and centre of mass
        ///     of Octree nodes.
        /// @details
        ///     If threads simultaneously request the calculation for an Octree
        ///     which is not cached, the "winning" thread to the cache entry
        ///     will perform the calculation while all other threads wait on
        ///     the result.
        PointMass Calculate(Octree const& octree);

        /// "Shortcut" for MassCalculator::Calculate
        PointMass operator()(Octree const& octree) { return Calculate(octree); }

    private:
        struct CacheEntry
        {
            bool is_cached{}; ///< True if the the result has been cached, false otherwise.
            PointMass result; ///< The result of the calculation.
            std::mutex mutex; ///< Guards CacheEntry::is_cached and CacheEntry::result.
            std::condition_variable until_cached; ///< Used to wait on the calculation result.
        };

        std::shared_mutex cache_mutex_; ///< Guard the cache's un-synchronized operations.
        tbb::concurrent_unordered_map<Octree const*, CacheEntry> cache_; ///< Cache of calculation results.

        /// Wait until the CacheEntry's result is ready, then return it.
        static PointMass WaitForResult(CacheEntry& entry);

        /// Find the cache entry if present, otherwise calculate and cache.
        PointMass FindOrCalculate(Octree const& octree);

        /// @brief
        ///     Attempt to calculate the Octree's total mass and centre of
        ///     mass and cache it.
        /// @details
        ///     Where multiple threads attempt a calculation, threads that lose
        ///     the race to the cache entry mutex will wait on the result. Only
        ///     The winner will calculate the result.
        PointMass CalculateAndCache(Octree const& octree);

        /// Calculate the Octree's total mass and centre of mass. The result is
        /// stored in @p point_mass. The cache is used recursively to obtain
        /// the results of child nodes to the given Octree.
        void Calculate(Octree const& octree, PointMass& point_mass);
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_MASSCALCULATOR_H_
