#include <rigidbodies/physics/world.hpp>

#include <stdexcept>

namespace rigidbodies::physics
{
    void World::set_parallel_settings(const ParallelSettings& settings)
    {
        if (settings.worker_count > 32 || settings.minimum_batch_size == 0)
            throw std::invalid_argument("Parallel settings require at most 32 workers and a positive batch size");
        if (settings.worker_count != parallel_settings_.worker_count)
            parallel_executor_.reset();
        parallel_settings_ = settings;
    }

    const ParallelSettings& World::parallel_settings() const
    {
        return parallel_settings_;
    }

    ParallelExecutor& World::parallel_executor()
    {
        if (!parallel_executor_)
            parallel_executor_ = std::make_shared<ParallelExecutor>(parallel_settings_.worker_count);
        return *parallel_executor_;
    }
}
