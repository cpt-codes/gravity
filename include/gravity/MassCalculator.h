#ifndef GRAVITY_INCLUDE_GRAVITY_MASSCALCULATOR_H_
#define GRAVITY_INCLUDE_GRAVITY_MASSCALCULATOR_H_

#include <condition_variable>
#include <mutex>
#include <shared_mutex>

#include <tbb/concurrent_unordered_map.h>

#include "gravity/Node.h"
#include "gravity/geometry/Vector.h"

namespace gravity
{
    /// @brief
    ///     A class that computes (and caches) the total mass and centre of
    ///     mass of Nodes containing particles. The calculator is thread-safe.
    /// @details
    ///     Results are cached only when the cache does not have an entry
    ///     for the given Node. All member functions are synchronised with
    ///     minimal locking to optimise performance.
    class MassCalculator
    {
    public:
        /// A point particle representing the total mass and centre of mass
        /// of all particles within a node and its ancestors.
        struct PointMass
        {
            double mass{}; ///< Total mass of all particles
            geometry::Vector displacement; ///< Centre of mass of all particles
        };

        /// Remove all calculation results from the cache.
        void ClearCache();

        /// Remove the cached result for the given Octree only. Cached results
        /// for this Node's ancestors will not be removed.
        [[maybe_unused]]
        void ClearCache(Node const& node);

        /// @brief
        ///     Computes, caches and returns the total mass and centre of mass
        ///     of the given Node.
        /// @details
        ///     If threads simultaneously request the calculation for a Node
        ///     which is not cached, the "winning" thread to the cache entry
        ///     will perform the calculation while all other threads wait on
        ///     the result.
        PointMass Calculate(Node const& node);

        /// "Shortcut" for MassCalculator::Calculate
        PointMass operator()(Node const& node) { return Calculate(node); }

    private:
        struct CacheEntry
        {
            bool is_cached{}; ///< True if the the result has been cached, false otherwise.
            PointMass result; ///< The result of the calculation.
            std::mutex mutex; ///< Guards CacheEntry::is_cached and CacheEntry::result.
            std::condition_variable until_cached; ///< Used to wait on the calculation result.
        };

        /// Wait until the CacheEntry's result is ready, then return it.
        static PointMass WaitForResult(CacheEntry& entry);

        /// Find the cache entry if present, otherwise calculate and cache.
        PointMass FindOrCalculate(Node const& node);

        /// @brief
        ///     Attempt to calculate the Node's total mass and centre of mass
        ///     and cache it.
        /// @details
        ///     Where multiple threads attempt a calculation, threads that lose
        ///     the race to the cache entry mutex will wait on the result. Only
        ///     The winner will calculate the result.
        PointMass CalculateAndCache(Node const& node);

        /// Calculate the Node's total mass and centre of mass. The result is
        /// stored in @p point_mass. The cache is used recursively to obtain
        /// the results for the Node's ancestors.
        void Calculate(Node const& node, PointMass& point_mass);

        std::shared_mutex cache_mutex_; ///< Guard the cache's un-synchronized operations.
        tbb::concurrent_unordered_map<Node const*, CacheEntry> cache_; ///< Cache of calculation results.
    };
}

#endif //GRAVITY_INCLUDE_GRAVITY_MASSCALCULATOR_H_
