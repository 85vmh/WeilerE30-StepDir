#define nc_ready_set(v) (nc_ready = (v))
#define nc_ready_ptr (&nc_ready)
#define turn_on_set(v) (turn_on = (v))
#define turn_on_ptr (&turn_on)
#define turn_off_set(v) (turn_off = (v))
#define turn_off_ptr (&turn_off)
#define powered_on_dbg_set(v) (powered_on_dbg = (v))
#define powered_on_dbg_ptr (&powered_on_dbg)
#include <stdbool.h>
#include "rtapi.h"
#include "hal.h"

/* ===== Debounce state ===== */
static int db_count      = 0;
static int powered_on_db = 0;

/* ===== Edge detectors on the pulse conditions ===== */
static int last_on_cond  = 0;
static int last_off_cond = 0;

/* ===== Pulse timers (seconds) ===== */
static double t_on  = 0.0;
static double t_off = 0.0;

/* Helper: start a timed pulse on a HAL bit pin */
static inline void pulse_output_pin(double *timer_ref, hal_bool_t pin_ref, float duration_sec)
{
    double d = (double)duration_sec;
    if (d < 0.01) d = 0.01;
    if (d > 5.00) d = 5.00;
    *timer_ref = d;
    hal_set_bool(pin_ref, 1);
}

FUNCTION(_)
{
    int on_cond, off_cond;

    /* ===== Power-on interlock =====
       Recomputed only while the machine is off; once it is on the value is held,
       same as the SET/RESET coil pair in the ladder. */
    if (!machine_on)
        nc_ready_set((joystick_neutral && spindle_neutral) ? 1 : 0);

    /* ===== Debounce the contactor feedback =====
       Integrator in samples, like the stock debounce component. Only the rising
       edge is filtered; the fall passes straight through, like the ladder's TON. */
    if (powered_on) {
        const int delay = (powered_on_debounce > 0) ? powered_on_debounce : 0;
        if (db_count < delay)
            db_count++;
        else
            powered_on_db = 1;
    } else {
        db_count      = 0;
        powered_on_db = 0;
    }
    powered_on_dbg_set(powered_on_db);

    /* ===== On / off pulses =====
       Fired on the rising edge of the condition, which is what a non-retriggerable
       TP timer does in the ladder. */
    on_cond  = (!machine_on && powered_on_db && nc_ready) ? 1 : 0;
    off_cond = ( machine_on && !powered_on) ? 1 : 0;

    if (on_cond && !last_on_cond)
        pulse_output_pin(&t_on, turn_on_ptr, output_pulse_duration);
    if (off_cond && !last_off_cond)
        pulse_output_pin(&t_off, turn_off_ptr, output_pulse_duration);

    last_on_cond  = on_cond;
    last_off_cond = off_cond;

    /* ===== Pulse timers ===== */
    if (t_on > 0.0) {
        t_on -= fperiod;
        if (t_on <= 0.0) { t_on = 0.0; turn_on_set(0); }
    }
    if (t_off > 0.0) {
        t_off -= fperiod;
        if (t_off <= 0.0) { t_off = 0.0; turn_off_set(0); }
    }
}
