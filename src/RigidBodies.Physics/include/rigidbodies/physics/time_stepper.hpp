#pragma once

#include <rigidbodies/physics/units.hpp>

#include <cstdint>

namespace rigidbodies::physics
{

    // Paces a variable-rate caller against a fixed simulation step.
    //
    // Fixed steps support reproducible runs and stable collision response independently of display
    // frame rate. The renderer uses the leftover fraction to interpolate between the last two states.
    //
    // Headless and interactive runs share these pacing rules through the physics module.
    class TimeStepper
    {
    public:
        explicit TimeStepper(Real fixed_step_s = 1.0 / 120.0);

        [[nodiscard]] Real fixed_step_s() const;
        // Changing the duration preserves fractional progress towards the next fixed step.
        void set_fixed_step(Real value);

        // Each fixed step is divided into this many physics updates. advance() still returns
        // whole fixed steps; the caller runs substep_count() updates of substep_s() for each.
        [[nodiscard]] int substep_count() const;
        void set_substep_count(int value);
        [[nodiscard]] Real substep_s() const;

        // Bound actual physics updates, including substeps, in one advance(). Values are clamped
        // to [1, 4096]. A substep count greater than the budget is reduced to fit, so a requested
        // single fixed step always fits. Increasing the count cannot silently increase the budget.
        [[nodiscard]] int maximum_substeps_per_frame() const;
        void set_maximum_substeps_per_frame(int value);

        // Multiplier on real time. Values below one slow a demonstration down without changing
        // the physics, which is how a fast collision is made observable.
        [[nodiscard]] Real time_scale() const;
        void set_time_scale(Real value);

        [[nodiscard]] bool is_paused() const;
        void set_paused(bool value);

        // Upper bound on wall-clock time accepted by one call, before applying time_scale().
        // The separate substep budget also bounds work at high time scales or short fixed steps.
        [[nodiscard]] Real maximum_frame_time_s() const;
        void set_maximum_frame_time(Real value);

        // Takes the elapsed wall-clock time and returns how many fixed steps to run now. While
        // paused it returns zero unless a single step has been requested. Excess whole steps are
        // discarded rather than carried into later frames; only a fractional step remains.
        [[nodiscard]] int advance(Real frame_time_s);

        // Report an early stop inside the most recent advance() batch, for example a pause on
        // impact. The argument counts trailing physics substeps which were never run. Their
        // duration is discarded, and every incomplete fixed step (including a partially run one)
        // is removed from completed_steps(). The existing interpolation remainder is untouched.
        // A positive cancellation consumes the batch once; zero is a no-op. Negative counts,
        // excess counts, and reuse of a consumed/expired batch throw std::invalid_argument.
        // The batch retains its original durations even if configuration changes before reporting.
        void cancel_scheduled_substeps(int unrun_substeps);

        // Queues exactly one step, so that a paused simulation can be advanced deliberately.
        void request_single_step();
        // Discards a queued request before execution without changing completed/discarded time.
        void cancel_single_step_request();

        // Fraction of a step already elapsed, in [0, 1), for interpolating what is drawn.
        [[nodiscard]] Real interpolation_fraction() const;

        void reset();

        [[nodiscard]] std::uint64_t completed_steps() const;

        // Simulated seconds discarded by wall-time clamping, the work budget, or an early stop. The cumulative
        // value is cleared by reset(); the last-call value is cleared by each advance(). Both
        // saturate at the largest finite Real if an extreme time scale overflows the total.
        [[nodiscard]] Real discarded_time_s() const;
        [[nodiscard]] Real last_discarded_time_s() const;

    private:
        Real fixed_step_s_ { 1.0 / 120.0 };
        Real time_scale_ { 1.0 };
        Real maximum_frame_time_s_ { 0.25 };
        Real accumulator_s_ { 0.0 };
        Real discarded_time_s_ { 0.0 };
        Real last_discarded_time_s_ { 0.0 };
        int substep_count_ { 1 };
        int maximum_substeps_per_frame_ { 128 };
        bool paused_ { false };
        bool single_step_requested_ { false };
        std::uint64_t completed_steps_ { 0 };
        std::uint64_t completed_before_advance_ { 0 };
        int last_scheduled_substeps_ { 0 };
        int last_scheduled_substep_count_ { 1 };
        Real last_scheduled_substep_s_ { 0.0 };
    };

} // namespace rigidbodies::physics
