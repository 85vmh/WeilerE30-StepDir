#define spindleInhibit_set(v) (spindleInhibit = (v))
#define spindleInhibit_ptr (&spindleInhibit)
#define spindleStop_set(v) (spindleStop = (v))
#define spindleStop_ptr (&spindleStop)
#define statusSpindleResetRequired_set(v) (statusSpindleResetRequired = (v))
#define statusSpindleResetRequired_ptr (&statusSpindleResetRequired)
#include <stdbool.h>
#include "rtapi.h"
#include "hal.h"

/* ===== Edge detector on the stop condition ===== */
static int last_stop_cond = 0;

/* ===== Pulse timer (seconds) ===== */
static double t_stop = 0.0;

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
    int stop_cond;

    /* ===== Cover opened with the spindle running, in manual/MDI: latch ===== */
    if (spindleCoverOpened && spindleOn && !isAutoMode) {
        spindleInhibit_set(1);
        statusSpindleResetRequired_set(1);
    }

    /* ===== Selector back to neutral with the cover closed: clear the latch =====
       Mutually exclusive with the block above, so the order does not matter -
       same as the SET and RESET coils in the ladder. */
    if (spindleNeutral && !spindleCoverOpened) {
        statusSpindleResetRequired_set(0);
        spindleInhibit_set(0);
    }

    /* ===== Selector to neutral with the machine on, in manual/MDI: stop pulse ===== */
    stop_cond = (spindleNeutral && machineOn && !isAutoMode) ? 1 : 0;

    if (stop_cond && !last_stop_cond)
        pulse_output_pin(&t_stop, spindleStop_ptr, outputPulseDuration);

    last_stop_cond = stop_cond;

    /* ===== Pulse timer ===== */
    if (t_stop > 0.0) {
        t_stop -= fperiod;
        if (t_stop <= 0.0) { t_stop = 0.0; spindleStop_set(0); }
    }
}
