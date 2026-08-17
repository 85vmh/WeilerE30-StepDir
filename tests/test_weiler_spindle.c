/* Functional regression tests for components/weiler_spindle.comp
 *
 * Chuck cover safety for the spindle in manual/MDI mode: opening the cover with
 * the spindle running latches the inhibit and the reset request, and only the
 * selector going back to neutral WITH the cover closed clears them. AUTO mode is
 * deliberately excluded - there the cover is handled by program_state.
 */

#include "harness.h"

/* ===== Pins (inputs) ===== */
hal_bit_t   machineOn, spindleOn, spindleNeutral, spindleCoverOpened, isAutoMode;
hal_float_t outputPulseDuration = 0.10f;

/* ===== Pins (outputs) ===== */
hal_bit_t spindleInhibit, spindleStop, statusSpindleResetRequired;

#include "weiler_spindle_body.inc"

/* Machine on, manual mode, selector engaged, cover closed, spindle running */
static void spindle_running_manual(void)
{
    machineOn          = 1;
    isAutoMode         = 0;
    spindleNeutral     = 0;
    spindleCoverOpened = 0;
    spindleOn          = 1;
    tick(3);
}

/* ===== Latching ===== */

static void t_cover_with_spindle_running_latches(void)
{
    spindle_running_manual();
    CHECK(spindleInhibit == 0);

    spindleCoverOpened = 1;
    tick(3);
    CHECK(spindleInhibit == 1);
    CHECK(statusSpindleResetRequired == 1);
}

static void t_cover_with_spindle_stopped_does_not_latch(void)
{
    spindle_running_manual();
    spindleOn = 0;
    tick(3);

    spindleCoverOpened = 1;
    tick(3);
    CHECK(spindleInhibit == 0);
    CHECK(statusSpindleResetRequired == 0);
}

static void t_auto_mode_is_excluded(void)
{
    spindle_running_manual();
    isAutoMode = 1;
    tick(3);

    spindleCoverOpened = 1;
    tick(3);
    CHECK(spindleInhibit == 0);              /* program_state owns the cover in AUTO */
    CHECK(statusSpindleResetRequired == 0);
}

/* ===== Clearing the latch ===== */

static void t_closing_the_cover_alone_does_not_clear(void)
{
    spindle_running_manual();
    spindleCoverOpened = 1;
    tick(3);
    CHECK(spindleInhibit == 1);

    spindleCoverOpened = 0;                  /* selector still engaged */
    tick(3);
    CHECK(spindleInhibit == 1);              /* stays latched */
    CHECK(statusSpindleResetRequired == 1);
}

static void t_neutral_with_cover_open_does_not_clear(void)
{
    spindle_running_manual();
    spindleCoverOpened = 1;
    tick(3);

    spindleNeutral = 1;                      /* neutral, but cover still open */
    tick(3);
    CHECK(spindleInhibit == 1);
    CHECK(statusSpindleResetRequired == 1);
}

static void t_neutral_and_cover_closed_clears(void)
{
    spindle_running_manual();
    spindleCoverOpened = 1;
    tick(3);
    CHECK(spindleInhibit == 1);

    spindleCoverOpened = 0;
    spindleNeutral     = 1;
    tick(3);
    CHECK(spindleInhibit == 0);
    CHECK(statusSpindleResetRequired == 0);
}

static void t_latch_survives_machine_off_on(void)
{
    spindle_running_manual();
    spindleCoverOpened = 1;
    tick(3);
    CHECK(spindleInhibit == 1);

    machineOn = 0; tick(50);
    machineOn = 1; tick(50);
    CHECK(spindleInhibit == 1);              /* as in the ladder */
    CHECK(statusSpindleResetRequired == 1);
}

/* The latch clears on the LEVEL "neutral + cover closed". If the selector is
   already neutral - which is the case whenever the spindle is commanded by a
   program rather than by the selector - closing the cover alone releases the
   spindle. That is why isAutoMode must be wired: in AUTO this component has to
   stand down and leave the cover to program_state, which requires a deliberate
   press before the spindle comes back. */
static void t_neutral_selector_releases_on_cover_close(void)
{
    machineOn          = 1;
    isAutoMode         = 0;
    spindleNeutral     = 1;     /* selector neutral, spindle driven elsewhere */
    spindleCoverOpened = 0;
    spindleOn          = 1;
    tick(3);

    spindleCoverOpened = 1;
    tick(3);
    CHECK(spindleInhibit == 1);

    spindleCoverOpened = 0;
    tick(3);
    CHECK(spindleInhibit == 0);             /* released without any operator action */
    CHECK(statusSpindleResetRequired == 0);
}

static void t_auto_mode_leaves_the_cover_alone(void)
{
    machineOn          = 1;
    isAutoMode         = 1;     /* program running: program_state owns the cover */
    spindleNeutral     = 1;
    spindleCoverOpened = 0;
    spindleOn          = 1;
    tick(3);

    spindleCoverOpened = 1;
    tick(3);
    CHECK(spindleInhibit == 0);
    CHECK(statusSpindleResetRequired == 0);

    spindleCoverOpened = 0;
    tick(3);
    CHECK(spindleInhibit == 0);
}

/* ===== Stop pulse ===== */

static void t_neutral_pulses_stop(void)
{
    spindle_running_manual();
    CHECK(spindleStop == 0);

    spindleNeutral = 1;
    tick(2);
    CHECK(spindleStop == 1);

    tick(200);
    CHECK(spindleStop == 0);                 /* pulse decays */
}

static void t_stop_pulses_only_on_the_edge(void)
{
    spindle_running_manual();

    spindleNeutral = 1;
    tick(2);
    CHECK(spindleStop == 1);
    tick(200);
    CHECK(spindleStop == 0);

    tick(500);                               /* still neutral, no retrigger */
    CHECK(spindleStop == 0);
}

static void t_no_stop_pulse_in_auto(void)
{
    spindle_running_manual();
    isAutoMode = 1;
    tick(3);

    spindleNeutral = 1;
    tick(5);
    CHECK(spindleStop == 0);
}

static void t_no_stop_pulse_with_machine_off(void)
{
    spindle_running_manual();
    machineOn = 0;
    tick(3);

    spindleNeutral = 1;
    tick(5);
    CHECK(spindleStop == 0);
}

static void t_stop_pulse_when_machine_comes_on_in_neutral(void)
{
    /* Machine off, selector already neutral: the pulse fires when the machine
       comes on, cancelling any running command. */
    machineOn      = 0;
    isAutoMode     = 0;
    spindleNeutral = 1;
    tick(5);
    CHECK(spindleStop == 0);

    machineOn = 1;
    tick(2);
    CHECK(spindleStop == 1);
}

/* ===== Runner ===== */

static const struct scenario scenarios[] = {
    { "cover + spindle running latches",        t_cover_with_spindle_running_latches },
    { "cover + spindle stopped does not latch", t_cover_with_spindle_stopped_does_not_latch },
    { "auto mode is excluded",                  t_auto_mode_is_excluded },
    { "closing the cover alone does not clear", t_closing_the_cover_alone_does_not_clear },
    { "neutral with cover open does not clear", t_neutral_with_cover_open_does_not_clear },
    { "neutral + cover closed clears",          t_neutral_and_cover_closed_clears },
    { "latch survives machine off/on",          t_latch_survives_machine_off_on },
    { "neutral selector releases on close",     t_neutral_selector_releases_on_cover_close },
    { "auto mode leaves the cover alone",       t_auto_mode_leaves_the_cover_alone },
    { "neutral pulses stop",                    t_neutral_pulses_stop },
    { "stop pulses only on the edge",           t_stop_pulses_only_on_the_edge },
    { "no stop pulse in auto",                  t_no_stop_pulse_in_auto },
    { "no stop pulse with machine off",         t_no_stop_pulse_with_machine_off },
    { "stop pulse on machine on in neutral",    t_stop_pulse_when_machine_comes_on_in_neutral },
};

int main(void)
{
    return RUN_SCENARIOS(scenarios);
}
