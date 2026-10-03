#include <rigidbodies/physics/time_stepper.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

    using namespace rigidbodies::physics;

    // A binary-exact duration keeps these pacing tests independent of decimal rounding.
    constexpr Real step_s = 1.0 / 128.0;

    bool cancellation_rejected(TimeStepper& stepper, int count)
    {
        try
        {
            stepper.cancel_scheduled_substeps(count);
        }
        catch (const std::invalid_argument&)
        {
            return true;
        }
        return false;
    }

    RIGIDBODIES_TEST("the work budget counts every physics substep")
    {
        TimeStepper stepper { step_s };
        stepper.set_maximum_substeps_per_frame(10);
        stepper.set_substep_count(3);
        stepper.set_time_scale(16.0);

        const auto steps = stepper.advance(0.25);
        RIGIDBODIES_EXPECT(steps == 3, "only complete visual steps fit the ten-update budget");
        RIGIDBODIES_EXPECT(steps * stepper.substep_count() <= 10, "actual physics work stays bounded");
        RIGIDBODIES_EXPECT_NEAR(stepper.substep_s() * stepper.substep_count(), step_s, 1.0e-15, "substeps span one fixed step");
        RIGIDBODIES_EXPECT_NEAR(stepper.last_discarded_time_s(), 4.0 - 3.0 * step_s, 1.0e-15, "unprocessed simulation time is reported");
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 3, "only scheduled fixed steps are counted");

        stepper.set_time_scale(1.0);
        RIGIDBODIES_EXPECT(stepper.advance(step_s / 4.0) == 0, "discarded work never causes a later catch-up burst");
        RIGIDBODIES_EXPECT_NEAR(stepper.last_discarded_time_s(), 0.0, 0.0, "the per-call diagnostic clears on the next frame");
    }

    RIGIDBODIES_TEST("dropping excess steps retains the fractional remainder")
    {
        TimeStepper stepper { step_s };
        stepper.set_maximum_substeps_per_frame(10);
        stepper.set_substep_count(3);

        RIGIDBODIES_EXPECT(stepper.advance(20.5 * step_s) == 3, "the first frame fills the budget");
        RIGIDBODIES_EXPECT_NEAR(stepper.interpolation_fraction(), 0.5, 0.0, "half a step remains available for interpolation");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 17.0 * step_s, 0.0, "only whole excess steps are discarded");
        RIGIDBODIES_EXPECT(stepper.advance(0.5 * step_s) == 1, "the remainder contributes to the next fixed step");
        RIGIDBODIES_EXPECT_NEAR(stepper.interpolation_fraction(), 0.0, 0.0, "the next fixed step consumes the remainder");
    }

    RIGIDBODIES_TEST("discarded simulation time includes wall-time clamping and the work budget")
    {
        TimeStepper stepper { step_s };
        stepper.set_maximum_frame_time(0.125);
        stepper.set_time_scale(2.0);
        stepper.set_maximum_substeps_per_frame(8);

        RIGIDBODIES_EXPECT(stepper.advance(0.5) == 8, "the accepted wall time still obeys the work budget");
        RIGIDBODIES_EXPECT_NEAR(stepper.last_discarded_time_s(), 1.0 - 8.0 * step_s, 0.0, "all lost simulated seconds are accounted for");
        RIGIDBODIES_EXPECT(stepper.advance(0.5) == 8, "a repeated stall remains bounded");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 2.0 - 16.0 * step_s, 0.0, "discarded time accumulates across stalls");
    }

    RIGIDBODIES_TEST("finite extreme time scales cannot overflow pacing state")
    {
        TimeStepper stepper { 1.0e-4 };
        const auto maximum = std::numeric_limits<Real>::max();
        stepper.set_maximum_frame_time(maximum);
        stepper.set_time_scale(maximum);
        stepper.set_maximum_substeps_per_frame(17);
        stepper.set_substep_count(4);

        for (int frame = 0; frame < 3; ++frame)
        {
            RIGIDBODIES_EXPECT(stepper.advance(maximum) == 4, "even overflowing scaled time schedules only bounded work");
            RIGIDBODIES_EXPECT(std::isfinite(stepper.last_discarded_time_s()), "per-frame loss stays finite");
            RIGIDBODIES_EXPECT(std::isfinite(stepper.discarded_time_s()), "cumulative loss saturates rather than overflowing");
            const auto fraction = stepper.interpolation_fraction();
            RIGIDBODIES_EXPECT(std::isfinite(fraction) && fraction >= 0.0 && fraction < 1.0, "interpolation remains a valid fraction");
        }

        stepper.set_time_scale(1.0);
        RIGIDBODIES_EXPECT(stepper.advance(1.0e-4) == 1, "normal pacing resumes without a retained backlog");
    }

    RIGIDBODIES_TEST("a paused single step spans one full fixed step with all configured substeps")
    {
        TimeStepper stepper { step_s };
        stepper.set_substep_count(4);
        stepper.set_paused(true);
        stepper.set_time_scale(0.0);
        RIGIDBODIES_EXPECT(stepper.advance(1.0) == 0, "wall time does not run a paused simulation");
        stepper.request_single_step();
        stepper.request_single_step();

        const auto steps = stepper.advance(0.0);
        Real simulated_s = 0.0;
        int physics_updates = 0;
        for (int visual_step = 0; visual_step < steps; ++visual_step)
        {
            for (int substep = 0; substep < stepper.substep_count(); ++substep)
            {
                simulated_s += stepper.substep_s();
                ++physics_updates;
            }
        }

        RIGIDBODIES_EXPECT(steps == 1 && physics_updates == 4, "a pending request schedules exactly one complete visual step");
        RIGIDBODIES_EXPECT_NEAR(simulated_s, step_s, 0.0, "single stepping advances exactly one fixed duration");
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 1, "completed steps count visual steps");
        RIGIDBODIES_EXPECT(stepper.advance(1.0) == 0, "the request is consumed once");
    }

    RIGIDBODIES_TEST("a substep configuration always leaves room for a complete single step")
    {
        TimeStepper stepper { step_s };
        stepper.set_maximum_substeps_per_frame(9);
        stepper.set_substep_count(std::numeric_limits<int>::max());
        RIGIDBODIES_EXPECT(stepper.substep_count() == 9, "substeps cannot exceed the work budget");
        stepper.set_maximum_substeps_per_frame(3);
        RIGIDBODIES_EXPECT(stepper.substep_count() == 3, "lowering the budget keeps a complete step executable");
        stepper.set_paused(true);
        stepper.request_single_step();
        RIGIDBODIES_EXPECT(stepper.advance(0.0) * stepper.substep_count() <= 3, "single stepping obeys the reduced budget");

        stepper.set_substep_count(0);
        RIGIDBODIES_EXPECT(stepper.substep_count() == 1, "zero substeps cannot divide the fixed duration by zero");
        stepper.set_maximum_substeps_per_frame(std::numeric_limits<int>::min());
        RIGIDBODIES_EXPECT(stepper.maximum_substeps_per_frame() == 1, "a nonpositive budget still permits a step");
        stepper.set_maximum_substeps_per_frame(std::numeric_limits<int>::max());
        RIGIDBODIES_EXPECT(stepper.maximum_substeps_per_frame() == 4096, "configuration cannot remove the hard work bound");
    }

    RIGIDBODIES_TEST("invalid frame times preserve accumulated valid time")
    {
        TimeStepper stepper { step_s };
        RIGIDBODIES_EXPECT(stepper.advance(step_s / 2.0) == 0, "a valid half step accumulates");
        RIGIDBODIES_EXPECT(stepper.advance(std::numeric_limits<Real>::infinity()) == 0, "infinite wall time is ignored");
        RIGIDBODIES_EXPECT(stepper.advance(std::numeric_limits<Real>::quiet_NaN()) == 0, "NaN wall time is ignored");
        RIGIDBODIES_EXPECT(stepper.advance(-1.0) == 0, "negative wall time is ignored");
        RIGIDBODIES_EXPECT(stepper.advance(step_s / 2.0) == 1, "invalid calls do not corrupt the valid half step");
    }

    RIGIDBODIES_TEST("changing the fixed rate preserves fractional progress without earning a burst of steps")
    {
        TimeStepper stepper { 2.0 * step_s };
        RIGIDBODIES_EXPECT(stepper.advance(1.5 * step_s) == 0, "three quarters of the old duration accumulates");
        stepper.set_time_scale(0.0);
        stepper.set_fixed_step(step_s / 4.0);

        RIGIDBODIES_EXPECT_NEAR(stepper.interpolation_fraction(), 0.75, 0.0, "the new rate keeps the same fractional progress");
        RIGIDBODIES_EXPECT(stepper.advance(1.0) == 0, "zero time scale cannot run a backlog after a rate change");
        RIGIDBODIES_EXPECT_NEAR(stepper.interpolation_fraction(), 0.75, 0.0, "a frozen frame retains its interpolation phase");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 0.0, 0.0, "frozen wall time is not discarded simulated time");
        stepper.set_time_scale(1.0);
        RIGIDBODIES_EXPECT(stepper.advance(step_s / 16.0) == 1, "only the remaining quarter of the new duration is needed");
        RIGIDBODIES_EXPECT_NEAR(stepper.interpolation_fraction(), 0.0, 0.0, "resuming consumes exactly the pending fraction");
    }

    RIGIDBODIES_TEST("reset clears pacing diagnostics and pending requests while retaining configuration")
    {
        TimeStepper stepper { step_s };
        stepper.set_maximum_substeps_per_frame(4);
        stepper.set_substep_count(2);
        RIGIDBODIES_EXPECT(stepper.advance(16.5 * step_s) == 2, "the frame produces work and discarded time");
        stepper.request_single_step();
        stepper.reset();

        RIGIDBODIES_EXPECT(stepper.completed_steps() == 0, "the counter restarts");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 0.0, 0.0, "cumulative loss restarts");
        RIGIDBODIES_EXPECT_NEAR(stepper.last_discarded_time_s(), 0.0, 0.0, "per-frame loss restarts");
        RIGIDBODIES_EXPECT_NEAR(stepper.interpolation_fraction(), 0.0, 0.0, "the fractional remainder clears");
        RIGIDBODIES_EXPECT(stepper.advance(0.0) == 0, "the pending request clears");
        RIGIDBODIES_EXPECT(stepper.substep_count() == 2 && stepper.maximum_substeps_per_frame() == 4, "reset retains pacing configuration");
    }

    RIGIDBODIES_TEST("early impact stop counts completed whole steps while discarding only unrun substeps")
    {
        TimeStepper stepper { step_s };
        stepper.set_substep_count(4);
        RIGIDBODIES_EXPECT(stepper.advance(2.0 * step_s) == 2, "earlier completed work establishes the counter");
        RIGIDBODIES_EXPECT(stepper.advance(3.5 * step_s) == 3, "next batch has twelve scheduled substeps and half a remainder");
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 5, "advance initially credits its entire scheduled batch");
        // Five substeps ran: one full fixed step and the first quarter of the next one.
        stepper.cancel_scheduled_substeps(7);
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 3, "a partially run fixed step is not counted as complete");
        RIGIDBODIES_EXPECT_NEAR(stepper.last_discarded_time_s(), 7.0 * step_s / 4.0, 0.0, "only seven unrun substeps are discarded, not the executed partial step");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 7.0 * step_s / 4.0, 0.0, "cumulative diagnostic receives the same unrun duration");
        RIGIDBODIES_EXPECT_NEAR(stepper.interpolation_fraction(), 0.5, 0.0, "cancellation leaves the previously accumulated fraction intact");
        stepper.set_paused(true);
        RIGIDBODIES_EXPECT_NEAR(stepper.interpolation_fraction(), 0.0, 0.0, "normal pause behavior still clears fractional wall time");
        stepper.set_paused(false);
        RIGIDBODIES_EXPECT(stepper.advance(step_s) == 1 && stepper.completed_steps() == 4, "resume has no hidden cancelled backlog");
        RIGIDBODIES_EXPECT_NEAR(stepper.last_discarded_time_s(), 0.0, 0.0, "a later frame starts a new per-frame loss diagnostic");
    }

    RIGIDBODIES_TEST("early stop without subdivision handles exact step boundaries and zero cancellation")
    {
        TimeStepper stepper { step_s };
        RIGIDBODIES_EXPECT(stepper.advance(4.0 * step_s) == 4, "four ordinary fixed steps are scheduled");
        stepper.cancel_scheduled_substeps(0);
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 4 && stepper.discarded_time_s() == 0.0, "zero cancellation is a harmless no-op");
        stepper.cancel_scheduled_substeps(3);
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 1, "one executed fixed step remains credited");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 3.0 * step_s, 0.0, "three whole steps are discarded without rounding");
        stepper.set_substep_count(4);
        RIGIDBODIES_EXPECT(stepper.advance(3.0 * step_s) == 3, "subdivided batch follows the ordinary batch");
        stepper.cancel_scheduled_substeps(8);
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 2, "cancelling an exact multiple of substeps preserves the last fully executed step");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 5.0 * step_s, 0.0, "whole and subdivided cancellations accumulate exact physical time");
    }

    RIGIDBODIES_TEST("paused single step may stop at an impact before completing its subdivisions")
    {
        TimeStepper stepper { step_s };
        stepper.set_substep_count(4);
        stepper.set_paused(true);
        stepper.request_single_step();
        RIGIDBODIES_EXPECT(stepper.advance(0.0) == 1, "single-step request schedules a complete fixed step");
        stepper.cancel_scheduled_substeps(3);
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 0, "one executed substep does not count as an entire fixed step");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 3.0 * step_s / 4.0, 0.0, "remaining single-step subdivisions are explicitly discarded");
        RIGIDBODIES_EXPECT(stepper.advance(1.0) == 0, "cancelled single-step work is not automatically rescheduled");
        stepper.request_single_step();
        RIGIDBODIES_EXPECT(stepper.advance(0.0) == 1 && stepper.completed_steps() == 1, "a later intentional single step works normally");
    }

    RIGIDBODIES_TEST("cancellation validation protects against underflow repeat use reset and expired batches")
    {
        TimeStepper stepper { step_s };
        RIGIDBODIES_EXPECT(cancellation_rejected(stepper, 1), "work cannot be cancelled before any advance");
        stepper.set_substep_count(3);
        RIGIDBODIES_EXPECT(stepper.advance(2.0 * step_s) == 2, "six substeps are available");
        for (const auto invalid : { -1, std::numeric_limits<int>::min(), 7, std::numeric_limits<int>::max() })
        {
            RIGIDBODIES_EXPECT(cancellation_rejected(stepper, invalid), "negative and excess cancellation counts are rejected");
            RIGIDBODIES_EXPECT(stepper.completed_steps() == 2 && stepper.discarded_time_s() == 0.0, "rejection changes neither completed work nor discarded time");
        }
        stepper.cancel_scheduled_substeps(6);
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 0, "cancelling every scheduled substep safely returns to the old count");
        RIGIDBODIES_EXPECT(cancellation_rejected(stepper, 1), "a positive cancellation consumes its batch once");
        stepper.cancel_scheduled_substeps(0);
        RIGIDBODIES_EXPECT(stepper.advance(step_s) == 1, "a new batch replaces consumed tracking");
        stepper.reset();
        RIGIDBODIES_EXPECT(cancellation_rejected(stepper, 1), "reset clears outstanding cancellation eligibility");
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 0 && stepper.discarded_time_s() == 0.0, "reset removes both scheduling and loss state");
        RIGIDBODIES_EXPECT(stepper.advance(step_s) == 1 && stepper.advance(0.0) == 0, "even an empty later advance expires the earlier batch");
        RIGIDBODIES_EXPECT(cancellation_rejected(stepper, 1), "stale cancellation cannot erase an earlier completed frame");
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 1, "expired cancellation rejection preserves completed history");
    }

    RIGIDBODIES_TEST("cancelled duration uses scheduled configuration even when settings have changed")
    {
        TimeStepper stepper { step_s };
        stepper.set_substep_count(4);
        RIGIDBODIES_EXPECT(stepper.advance(3.0 * step_s) == 3, "original batch schedules twelve quarter-step updates");
        stepper.set_substep_count(1);
        stepper.set_fixed_step(step_s / 2.0);
        stepper.cancel_scheduled_substeps(7);
        RIGIDBODIES_EXPECT(stepper.completed_steps() == 1, "completed accounting retains the batch's original grouping");
        RIGIDBODIES_EXPECT_NEAR(stepper.discarded_time_s(), 7.0 * step_s / 4.0, 0.0, "loss uses original duration rather than the newly configured duration");
    }

    RIGIDBODIES_TEST("early stop combines with budget loss and keeps saturated diagnostics finite")
    {
        TimeStepper stepper { step_s };
        stepper.set_maximum_substeps_per_frame(8);
        stepper.set_substep_count(4);
        RIGIDBODIES_EXPECT(stepper.advance(5.0 * step_s) == 2, "budget schedules two of five requested steps");
        stepper.cancel_scheduled_substeps(7);
        RIGIDBODIES_EXPECT_NEAR(stepper.last_discarded_time_s(), (3.0 + 7.0 / 4.0) * step_s, 0.0, "loss combines budget discard with only the unfinished substeps");
        const auto maximum = std::numeric_limits<Real>::max();
        stepper.set_time_scale(maximum);
        stepper.set_maximum_frame_time(maximum);
        for (int index = 0; index < 3; ++index)
        {
            RIGIDBODIES_EXPECT(stepper.advance(maximum) == 2, "extreme scaled time still respects the schedule bound");
            stepper.cancel_scheduled_substeps(8);
            RIGIDBODIES_EXPECT(stepper.completed_steps() == 0, "complete batch cancellation cannot underflow the completed counter");
            RIGIDBODIES_EXPECT(std::isfinite(stepper.discarded_time_s()) && std::isfinite(stepper.last_discarded_time_s()),
                "adding cancellation to an already saturated loss cannot overflow");
        }
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
