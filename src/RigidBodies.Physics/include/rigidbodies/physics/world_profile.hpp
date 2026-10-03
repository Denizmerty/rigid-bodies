#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace rigidbodies::physics
{
    // Wall-clock measurements of the last completed step. Disabled by default; counters
    // remain available without reading the clock. Timings are diagnostic, never simulated state.
    struct WorldProfile
    {
        double total_s {}, forces_s {}, wake_s {}, velocity_integration_s {};
        double broad_phase_s {}, narrow_phase_s {}, islands_s {}, velocity_solve_s {};
        double position_integration_s {}, ccd_s {}, position_solve_s {}, sleep_s {}, accounting_s {};
        std::uint64_t step_index {};
        std::size_t broad_phase_workers {}, narrow_phase_workers {};
    };

    class ScopedProfileTimer
    {
    public:
        ScopedProfileTimer(bool enabled, double& accumulated)
            : accumulated_(enabled ? &accumulated : nullptr)
        {
            if (accumulated_)
                start_ = Clock::now();
        }
        ~ScopedProfileTimer()
        {
            stop();
        }
        ScopedProfileTimer(const ScopedProfileTimer&) = delete;
        ScopedProfileTimer& operator=(const ScopedProfileTimer&) = delete;
        void stop()
        {
            if (accumulated_)
            {
                *accumulated_ += std::chrono::duration<double>(Clock::now() - start_).count();
                accumulated_ = nullptr;
            }
        }

    private:
        using Clock = std::chrono::steady_clock;
        double* accumulated_;
        Clock::time_point start_ {};
    };
}
