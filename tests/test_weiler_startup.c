/* Functional regression tests for components/weiler_startup.comp
 *
 * Power-on interlock and machine on/off pulses. The interlock (nc-ready) is only
 * recomputed while the machine is off and then held; turn-on is filtered by a
 * sample debounce on the contactor feedback, turn-off is deliberately unfiltered.
 *
 * The .comp uses hyphenated pin names, which halcompile turns into underscores.
 */

#include "harness.h"

/* ===== Pins (inputs) ===== */
hal_bit_t   machine_on, powered_on, joystick_neutral, spindle_neutral;
hal_s32_t   powered_on_debounce   = 5;
hal_float_t output_pulse_duration = 0.10f;

/* ===== Pins (outputs) ===== */
hal_bit_t nc_ready, turn_on, turn_off;

/* ===== Params ===== */
hal_bit_t powered_on_dbg;

#include "weiler_startup_body.inc"

/* Machine off, both selectors on neutral, no contactor feedback */
static void ready_to_power_on(void)
{
    machine_on       = 0;
    powered_on       = 0;
    joystick_neutral = 1;
    spindle_neutral  = 1;
    tick(3);
}

/* ===== Interlock ===== */

static void t_interlock_needs_both_neutral(void)
{
    ready_to_power_on();
    CHECK(nc_ready == 1);

    joystick_neutral = 0;
    tick(3);
    CHECK(nc_ready == 0);

    joystick_neutral = 1;
    spindle_neutral  = 0;
    tick(3);
    CHECK(nc_ready == 0);

    spindle_neutral = 1;
    tick(3);
    CHECK(nc_ready == 1);
}

static void t_interlock_is_held_while_machine_is_on(void)
{
    ready_to_power_on();
    CHECK(nc_ready == 1);

    machine_on = 1;
    tick(3);

    joystick_neutral = 0;        /* leaving neutral must not drop the interlock */
    spindle_neutral  = 0;
    tick(10);
    CHECK(nc_ready == 1);

    machine_on = 0;              /* recomputed again once the machine is off */
    tick(3);
    CHECK(nc_ready == 0);
}

/* ===== Turn-on pulse ===== */

static void t_turn_on_after_debounce(void)
{
    ready_to_power_on();

    powered_on = 1;
    tick(5);                     /* debounce = 5 samples: not yet */
    CHECK(powered_on_dbg == 0);
    CHECK(turn_on == 0);

    tick(1);
    CHECK(powered_on_dbg == 1);
    CHECK(turn_on == 1);

    tick(200);
    CHECK(turn_on == 0);         /* pulse decays */
}

static void t_no_turn_on_without_interlock(void)
{
    ready_to_power_on();
    joystick_neutral = 0;
    tick(3);
    CHECK(nc_ready == 0);

    powered_on = 1;
    tick(50);
    CHECK(powered_on_dbg == 1);
    CHECK(turn_on == 0);         /* interlock blocks the pulse */
}

static void t_debounce_restarts_on_a_glitch(void)
{
    ready_to_power_on();

    powered_on = 1; tick(3);     /* short glitch, below the debounce */
    powered_on = 0; tick(3);
    CHECK(powered_on_dbg == 0);
    CHECK(turn_on == 0);

    powered_on = 1;
    tick(5);
    CHECK(turn_on == 0);         /* counter restarted from zero */
    tick(1);
    CHECK(turn_on == 1);
}

static void t_turn_on_pulses_once(void)
{
    ready_to_power_on();

    powered_on = 1;
    tick(10);
    CHECK(turn_on == 1);
    tick(200);
    CHECK(turn_on == 0);

    tick(500);                   /* condition still true, no retrigger */
    CHECK(turn_on == 0);
}

/* ===== Turn-off pulse ===== */

static void t_turn_off_is_unfiltered(void)
{
    ready_to_power_on();
    powered_on = 1;
    tick(10);
    machine_on = 1;              /* machine came up */
    tick(200);

    powered_on = 0;              /* contactor dropped */
    tick(1);
    CHECK(turn_off == 1);        /* immediate, no debounce on the way down */

    tick(200);
    CHECK(turn_off == 0);
}

static void t_no_turn_off_while_machine_is_off(void)
{
    ready_to_power_on();
    powered_on = 1;
    tick(10);

    powered_on = 0;              /* machine never came on */
    tick(5);
    CHECK(turn_off == 0);
}

static void t_turn_off_pulses_once(void)
{
    ready_to_power_on();
    powered_on = 1; tick(10);
    machine_on = 1; tick(200);

    powered_on = 0; tick(2);
    CHECK(turn_off == 1);
    tick(200);
    CHECK(turn_off == 0);

    tick(500);                   /* still off, no retrigger */
    CHECK(turn_off == 0);
}

/* ===== Runner ===== */

static const struct scenario scenarios[] = {
    { "interlock needs both neutral",          t_interlock_needs_both_neutral },
    { "interlock held while machine is on",    t_interlock_is_held_while_machine_is_on },
    { "turn-on after debounce",                t_turn_on_after_debounce },
    { "no turn-on without interlock",          t_no_turn_on_without_interlock },
    { "debounce restarts on a glitch",         t_debounce_restarts_on_a_glitch },
    { "turn-on pulses once",                   t_turn_on_pulses_once },
    { "turn-off is unfiltered",                t_turn_off_is_unfiltered },
    { "no turn-off while machine is off",      t_no_turn_off_while_machine_is_off },
    { "turn-off pulses once",                  t_turn_off_pulses_once },
};

int main(void)
{
    return RUN_SCENARIOS(scenarios);
}
