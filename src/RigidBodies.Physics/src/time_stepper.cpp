#include <rigidbodies/physics/time_stepper.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rigidbodies::physics
{

    namespace
    {

        Real saturating_add(Real first, Real second)
        {
            const auto maximum = std::numeric_limits<Real>::max();
            return first > maximum - second ? maximum : first + second;
        }

        Real saturating_multiply(Real first, Real second)
        {
            const auto maximum = std::numeric_limits<Real>::max();
            return second > 1.0 && first > maximum / second ? maximum : first * second;
        }

        void add_completed_steps(std::uint64_t& completed, int count)
        {
            const auto amount = static_cast<std::uint64_t>(count);
            completed += std::min(amount, std::numeric_limits<std::uint64_t>::max() - completed);
        }

    } // namespace

    TimeStepper::TimeStepper(Real fixed_step_s)
    {
        set_fixed_step(fixed_step_s);
    }

    Real TimeStepper::fixed_step_s() const
    {
        return fixed_step_s_;
    }

    void TimeStepper::set_fixed_step(Real value)
    {
        if (!math::is_finite(value) || value <= 0.0)
        {
            return;
        }
        // A step longer than a fiftieth of a second makes contact response visibly coarse, and one
        // shorter than a ten-thousandth costs time without changing what a viewer sees.
        const auto next_step_s = math::clamp(value, 1.0e-4, 0.02);
        if (next_step_s != fixed_step_s_)
        {
            // A new rate should not turn partial progress into an immediate burst of work. Keep
            // the interpolation phase and ensure rounding cannot make it a complete step.
            accumulator_s_ = std::min(interpolation_fraction() * next_step_s, std::nextafter(next_step_s, 0.0));
            fixed_step_s_ = next_step_s;
        }
    }

    int TimeStepper::substep_count() const
    {
        return substep_count_;
    }

    void TimeStepper::set_substep_count(int value)
    {
        substep_count_ = std::clamp(value, 1, maximum_substeps_per_frame_);
    }

    Real TimeStepper::substep_s() const
    {
        return fixed_step_s_ / static_cast<Real>(substep_count_);
    }

    int TimeStepper::maximum_substeps_per_frame() const
    {
        return maximum_substeps_per_frame_;
    }

    void TimeStepper::set_maximum_substeps_per_frame(int value)
    {
        maximum_substeps_per_frame_ = std::clamp(value, 1, 4096);
        substep_count_ = std::min(substep_count_, maximum_substeps_per_frame_);
    }

    Real TimeStepper::time_scale() const
    {
        return time_scale_;
    }

    void TimeStepper::set_time_scale(Real value)
    {
        if (!math::is_finite(value) || value < 0.0)
        {
            return;
        }
        time_scale_ = value;
    }

    bool TimeStepper::is_paused() const
    {
        return paused_;
    }

    void TimeStepper::set_paused(bool value)
    {
        paused_ = value;
        if (paused_)
        {
            // Time accumulated before the pause would otherwise be spent in a burst on resume.
            accumulator_s_ = 0.0;
        }
    }

    Real TimeStepper::maximum_frame_time_s() const
    {
        return maximum_frame_time_s_;
    }

    void TimeStepper::set_maximum_frame_time(Real value)
    {
        if (!math::is_finite(value) || value <= 0.0)
        {
            return;
        }
        maximum_frame_time_s_ = value;
    }

    int TimeStepper::advance(Real frame_time_s)
    {
        last_discarded_time_s_ = 0.0;
        completed_before_advance_ = completed_steps_;
        last_scheduled_substeps_ = 0;
        last_scheduled_substep_count_ = substep_count_;
        last_scheduled_substep_s_ = substep_s();

        if (single_step_requested_)
        {
            single_step_requested_ = false;
            accumulator_s_ = 0.0;
            add_completed_steps(completed_steps_, 1);
            last_scheduled_substeps_ = substep_count_;
            return 1;
        }

        if (paused_ || time_scale_ == 0.0 || !math::is_finite(frame_time_s) || frame_time_s <= 0.0)
        {
            return 0;
        }

        const auto accepted_frame_s = std::min(frame_time_s, maximum_frame_time_s_);
        last_discarded_time_s_ = saturating_multiply(frame_time_s - accepted_frame_s, time_scale_);
        accumulator_s_ = saturating_add(accumulator_s_, saturating_multiply(accepted_frame_s, time_scale_));

        const auto maximum_steps = maximum_substeps_per_frame_ / substep_count_;
        const auto maximum_duration_s = static_cast<Real>(maximum_steps) * fixed_step_s_;
        int steps = 0;
        if (accumulator_s_ <= maximum_duration_s)
        {
            // Dividing only after comparing durations keeps the quotient representable as an int,
            // even when a finite time scale would overflow a direct division or multiplication.
            steps = std::min(maximum_steps, static_cast<int>(accumulator_s_ / fixed_step_s_));
            accumulator_s_ = std::max(0.0, accumulator_s_ - static_cast<Real>(steps) * fixed_step_s_);
        }
        else
        {
            steps = maximum_steps;
            const auto backlog_s = accumulator_s_ - maximum_duration_s;
            accumulator_s_ = std::fmod(backlog_s, fixed_step_s_);
            last_discarded_time_s_ = saturating_add(last_discarded_time_s_, backlog_s - accumulator_s_);
        }

        discarded_time_s_ = saturating_add(discarded_time_s_, last_discarded_time_s_);
        add_completed_steps(completed_steps_, steps);
        last_scheduled_substeps_ = steps * substep_count_;
        return steps;
    }

    void TimeStepper::cancel_scheduled_substeps(int unrun_substeps)
    {
        if (unrun_substeps < 0 || unrun_substeps > last_scheduled_substeps_)
            throw std::invalid_argument("Cancelled substeps must belong to the most recent uncancelled advance batch");
        if (unrun_substeps == 0)
            return;

        const auto scheduled_steps = last_scheduled_substeps_ / last_scheduled_substep_count_;
        const auto incomplete_steps = (unrun_substeps + last_scheduled_substep_count_ - 1) / last_scheduled_substep_count_;
        // Reconstruct from the old total rather than subtracting from a saturated counter: if
        // saturation absorbed some of the newly scheduled steps, they were never credited.
        completed_steps_ = completed_before_advance_;
        add_completed_steps(completed_steps_, scheduled_steps - incomplete_steps);
        const auto cancelled_time_s = static_cast<Real>(unrun_substeps) * last_scheduled_substep_s_;
        discarded_time_s_ = saturating_add(discarded_time_s_, cancelled_time_s);
        last_discarded_time_s_ = saturating_add(last_discarded_time_s_, cancelled_time_s);
        last_scheduled_substeps_ = 0;
    }

    void TimeStepper::request_single_step()
    {
        single_step_requested_ = true;
    }

    void TimeStepper::cancel_single_step_request()
    {
        single_step_requested_ = false;
    }

    Real TimeStepper::interpolation_fraction() const
    {
        return math::clamp(accumulator_s_ / fixed_step_s_, 0.0, std::nextafter(1.0, 0.0));
    }

    void TimeStepper::reset()
    {
        accumulator_s_ = 0.0;
        single_step_requested_ = false;
        completed_steps_ = 0;
        discarded_time_s_ = 0.0;
        last_discarded_time_s_ = 0.0;
        completed_before_advance_ = 0;
        last_scheduled_substeps_ = 0;
        last_scheduled_substep_count_ = substep_count_;
        last_scheduled_substep_s_ = substep_s();
    }

    std::uint64_t TimeStepper::completed_steps() const
    {
        return completed_steps_;
    }

    Real TimeStepper::discarded_time_s() const
    {
        return discarded_time_s_;
    }

    Real TimeStepper::last_discarded_time_s() const
    {
        return last_discarded_time_s_;
    }

} // namespace rigidbodies::physics
