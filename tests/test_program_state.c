/* Functional regression tests for components/program_state.comp */

#include "harness.h"

/* ===== Pins (inputs) ===== */
hal_bit_t   programLoaded, cycleStart, cycleAbort, singleBlock;
hal_bit_t   programIdle, programPaused, programRunning;
hal_s32_t   motionType;
hal_bit_t   chuckCoverOpened, spindleAtSpeed;
hal_bit_t   toolChange, machineOn;
hal_float_t outputPulseDuration = 0.10f;

/* ===== Pins (outputs) ===== */
hal_bit_t startProgram, pauseProgram, resumeProgram, stepProgram, stopProgram;
hal_bit_t programCompleted, programAborted, toolChanged;
hal_bit_t feedInhibit, spindleInhibit;
hal_bit_t statusProgramIdle, statusProgramRunning, statusProgramPaused;
hal_bit_t statusIsSingleBlock, statusStepPaused;
hal_bit_t statusChuckCoverOpened, statusSpindleResumeRequired;
hal_bit_t statusFeedResumeRequired, statusToolChangeRequired;
hal_bit_t cycleStartLedStatus;

/* ===== Params ===== */
hal_s32_t state_dbg, last_action_dbg, overlay_dbg;
hal_bit_t postStartPausePending_dbg, toolChangeAckSent_dbg;

/* The component body, extracted from the .comp at build time */
#include "program_state_body.inc"

/* ===== Test plumbing ===== */

static void press_green(void) { cycleStart = 1; tick(2); cycleStart = 0; tick(2); }
static void press_red(void)   { cycleAbort = 1; tick(2); cycleAbort = 0; tick(2); }

static void halui_program_idle(void)    { programIdle = 1; programRunning = 0; programPaused = 0; }
static void halui_program_running(void) { programIdle = 0; programRunning = 1; programPaused = 0; }
static void halui_program_paused(void)  { programIdle = 0; programRunning = 0; programPaused = 1; }

/* Program loaded, machine on, sitting idle */
static void load_program(void)
{
    machineOn = 1;
    programLoaded = 1;
    halui_program_idle();
    tick(3);
}

/* Loaded and actually running, as halui would report it */
static void run_program(void)
{
    load_program();
    press_green();
    halui_program_running();
    tick(3);
}

/* ===== Scenarios: base state machine ===== */

static void t_start_from_idle(void)
{
    load_program();
    CHECK(state_dbg == ST_IDLE);

    press_green();
    CHECK(startProgram == 1);
    CHECK(last_action_dbg == 1);

    tick(200);
    CHECK(startProgram == 0);   /* pulse decays */
}

static void t_pause_and_resume(void)
{
    run_program();
    CHECK(statusProgramRunning == 1);

    press_green();
    CHECK(pauseProgram == 1);
    CHECK(last_action_dbg == 2);

    halui_program_paused();
    tick(200);
    CHECK(statusProgramPaused == 1);

    press_green();
    CHECK(resumeProgram == 1);
    CHECK(last_action_dbg == 3);
}

static void t_single_block_step_and_feed_hold(void)
{
    singleBlock = 1;            /* SB on before running: no pause on the rise */
    run_program();

    motionType = 0;             /* standstill -> GREEN steps */
    press_green();
    CHECK(stepProgram == 1);
    CHECK(last_action_dbg == 4);
    tick(200);

    motionType = 1;             /* moving -> GREEN toggles feed hold */
    press_green();
    CHECK(feedInhibit == 1);
    CHECK(last_action_dbg == 7);

    press_green();
    CHECK(feedInhibit == 0);
    CHECK(last_action_dbg == 8);
}

static void t_single_block_rise_pauses(void)
{
    run_program();
    singleBlock = 1;
    tick(2);
    CHECK(pauseProgram == 1);
    CHECK(last_action_dbg == 2);
}

static void t_first_green_in_sb_pause_steps(void)
{
    run_program();
    singleBlock = 1;
    halui_program_paused();
    motionType = 1;             /* would normally toggle feed hold */
    tick(3);
    CHECK(state_dbg == ST_PAUSED_SINGLE_BLOCK);

    press_green();              /* first GREEN after entering SB pause must STEP */
    CHECK(stepProgram == 1);
    CHECK(last_action_dbg == 4);
}

/* ===== Scenarios: chuck cover overlay ===== */

static void t_cover_while_running_full_resume(void)
{
    run_program();
    spindleAtSpeed = 1;         /* cutting: the spindle is actually turning */
    tick(3);

    chuckCoverOpened = 1;
    tick(3);
    CHECK(overlay_dbg == OV_CHUCK_COVER_OPENED);
    CHECK(statusChuckCoverOpened == 1);
    CHECK(feedInhibit == 1);
    CHECK(spindleInhibit == 1);

    spindleAtSpeed = 0;         /* our inhibit stops the spindle */
    tick(3);

    press_green();              /* ignored while the cover is open */
    CHECK(pauseProgram == 0);
    CHECK(stepProgram == 0);

    chuckCoverOpened = 0;
    tick(3);
    CHECK(overlay_dbg == OV_SPINDLE_RESUME_REQ);
    CHECK(statusSpindleResumeRequired == 1);

    press_green();              /* stage 1: allow the spindle */
    CHECK(spindleInhibit == 0);
    CHECK(feedInhibit == 1);    /* feed still held */

    spindleAtSpeed = 1;
    tick(3);
    CHECK(overlay_dbg == OV_FEED_RESUME_REQ);
    CHECK(statusFeedResumeRequired == 1);

    press_green();              /* stage 2: release the feed */
    CHECK(feedInhibit == 0);
    CHECK(overlay_dbg == OV_NONE);
}

static void t_cover_reopen_returns_to_interlock(void)
{
    run_program();
    spindleAtSpeed = 1; tick(3);   /* spindle turning when the cover opens */

    chuckCoverOpened = 1; tick(3);
    chuckCoverOpened = 0; tick(3);
    CHECK(overlay_dbg == OV_SPINDLE_RESUME_REQ);

    chuckCoverOpened = 1; tick(3);
    CHECK(overlay_dbg == OV_CHUCK_COVER_OPENED);
    CHECK(feedInhibit == 1);
    CHECK(spindleInhibit == 1);
}

/* Machine report: green pauses the program, the spindle keeps turning, then the
   cover is opened. Closing it must NOT restart the spindle on its own. */
static void t_cover_while_paused_needs_deliberate_restart(void)
{
    run_program();
    spindleAtSpeed = 1;
    motionType = 1;
    tick(3);

    press_green();                  /* pause */
    CHECK(pauseProgram == 1);
    halui_program_paused();
    tick(3);
    CHECK(state_dbg == ST_PAUSED_NO_SINGLE_BLOCK);
    CHECK(spindleInhibit == 0);     /* spindle still turning while paused */

    chuckCoverOpened = 1;
    tick(3);
    CHECK(overlay_dbg    == OV_CHUCK_COVER_OPENED);
    CHECK(spindleInhibit == 1);     /* cover open must stop the spindle */
    CHECK(feedInhibit    == 1);

    spindleAtSpeed = 0;
    chuckCoverOpened = 0;
    tick(3);
    CHECK(spindleInhibit == 1);     /* closing the cover restarts nothing */
    CHECK(overlay_dbg    == OV_SPINDLE_RESUME_REQ);

    press_green();                  /* only a deliberate press releases it */
    CHECK(spindleInhibit == 0);

    spindleAtSpeed = 1;
    tick(3);
    CHECK(overlay_dbg == OV_FEED_RESUME_REQ);

    press_green();
    CHECK(feedInhibit   == 0);
    CHECK(overlay_dbg   == OV_NONE);
    CHECK(resumeProgram == 1);      /* the paused program is resumed too */
}

static void t_cover_in_idle_blocks_start(void)
{
    load_program();
    chuckCoverOpened = 1;
    tick(3);
    CHECK(overlay_dbg == OV_CHUCK_COVER_OPENED);
    CHECK(feedInhibit == 0);    /* no interlock, we were not running */

    press_green();
    CHECK(startProgram == 0);   /* start blocked while the cover is open */

    chuckCoverOpened = 0;
    tick(3);
    CHECK(overlay_dbg == OV_NONE);

    press_green();
    CHECK(startProgram == 1);
}

/* ===== Scenarios: tool change ===== */

static void t_tool_change_confirmed_once(void)
{
    run_program();

    toolChange = 1;
    tick(3);
    CHECK(statusToolChangeRequired == 1);

    press_green();
    CHECK(toolChanged == 1);
    CHECK(last_action_dbg == 9);
    CHECK(pauseProgram == 0);   /* GREEN must not pause during a tool change */

    tick(200);
    CHECK(toolChanged == 0);

    press_green();              /* further presses do nothing while pending */
    CHECK(toolChanged == 0);
    CHECK(pauseProgram == 0);

    toolChange = 0;
    tick(3);
    press_green();              /* back to the base machine */
    CHECK(pauseProgram == 1);
}

static void t_tool_change_cover_cycled_leaves_no_interlock(void)
{
    run_program();

    toolChange = 1;
    tick(3);

    chuckCoverOpened = 1;       /* cover ignored while the change is pending */
    tick(3);
    CHECK(overlay_dbg == OV_NONE);
    CHECK(feedInhibit == 0);
    CHECK(spindleInhibit == 0);

    chuckCoverOpened = 0;
    tick(3);
    CHECK(overlay_dbg == OV_NONE);

    press_green();
    CHECK(toolChanged == 1);

    toolChange = 0;
    tick(3);
    CHECK(overlay_dbg == OV_NONE);   /* nothing left to resume */
    CHECK(feedInhibit == 0);
}

static void t_tool_change_confirmed_with_cover_open(void)
{
    run_program();

    toolChange = 1;       tick(3);
    chuckCoverOpened = 1; tick(3);
    CHECK(feedInhibit == 0);

    press_green();
    CHECK(toolChanged == 1);

    toolChange = 0;                  /* confirmed with the cover still open */
    tick(3);
    CHECK(overlay_dbg == OV_CHUCK_COVER_OPENED);
    CHECK(feedInhibit == 1);
    CHECK(spindleInhibit == 1);

    chuckCoverOpened = 0;
    tick(3);
    /* The spindle was stopped for the tool change, so there is nothing to spin
       back up: straight to the feed stage, one press away from running. */
    CHECK(overlay_dbg == OV_FEED_RESUME_REQ);

    press_green();
    CHECK(feedInhibit == 0);
    CHECK(overlay_dbg == OV_NONE);
}

static void t_tool_change_cover_open_with_spindle_running(void)
{
    run_program();
    spindleAtSpeed = 1;              /* spindle left turning across the M6 */
    tick(3);

    toolChange = 1;       tick(3);
    chuckCoverOpened = 1; tick(3);

    press_green();                   /* confirms the tool change */
    toolChange = 0;
    tick(3);
    CHECK(overlay_dbg == OV_CHUCK_COVER_OPENED);
    CHECK(spindleInhibit == 1);      /* spindle was turning: interlock it */

    spindleAtSpeed = 0;
    chuckCoverOpened = 0;
    tick(3);
    CHECK(overlay_dbg == OV_SPINDLE_RESUME_REQ);   /* full sequence required */

    press_green();
    CHECK(spindleInhibit == 0);
    spindleAtSpeed = 1;
    tick(3);
    CHECK(overlay_dbg == OV_FEED_RESUME_REQ);
}

static void t_tool_change_suppresses_sb_pause(void)
{
    run_program();

    toolChange = 1;
    tick(3);

    singleBlock = 1;            /* would normally pulse pause */
    tick(3);
    CHECK(pauseProgram == 0);
}

static void t_abort_during_tool_change(void)
{
    run_program();

    toolChange = 1;
    tick(3);

    press_red();
    CHECK(stopProgram == 1);
    CHECK(toolChanged == 0);    /* never confirm a tool that was not changed */
    CHECK(toolChangeAckSent_dbg == 0);
}

/* ===== Scenarios: reproduction of a state seen on the machine ===== */

/* Program running, interpreter waiting for the tool change to be confirmed.
   Everything else is at rest: cover closed, single block off, spindle stopped. */
static void machine_waiting_at_tool_change(void)
{
    run_program();

    toolChange = 1;
    tick(3);

    CHECK(statusProgramRunning     == 1);
    CHECK(statusToolChangeRequired == 1);
    CHECK(machineOn                == 1);
    CHECK(programLoaded            == 1);
    CHECK(programRunning           == 1);
    CHECK(toolChange               == 1);

    /* nothing else engaged */
    CHECK(statusProgramIdle        == 0);
    CHECK(statusProgramPaused      == 0);
    CHECK(statusChuckCoverOpened   == 0);
    CHECK(statusSpindleResumeRequired == 0);
    CHECK(statusFeedResumeRequired == 0);
    CHECK(feedInhibit              == 0);
    CHECK(spindleInhibit           == 0);
    CHECK(overlay_dbg              == OV_NONE);
}

static void t_machine_repro_waiting_at_tool_change(void)
{
    machine_waiting_at_tool_change();

    /* Operator opens the chuck cover to swap the tool: ignored while the change
       is pending, so nothing is engaged yet. */
    chuckCoverOpened = 1;
    tick(3);
    CHECK(overlay_dbg    == OV_NONE);
    CHECK(feedInhibit    == 0);
    CHECK(spindleInhibit == 0);

    /* Operator presses CycleStart: it confirms the tool change and nothing else.
       LinuxCNC then drops tool-change. */
    press_green();
    CHECK(toolChanged   == 1);
    CHECK(last_action_dbg == 9);
    CHECK(pauseProgram  == 0);      /* the component never asks for a pause here */

    toolChange = 0;
    tick(3);

    /* The cover is still open when the change is confirmed, so the interlock
       engages now, as if the cover had just been opened while running. */
    CHECK(statusChuckCoverOpened == 1);
    CHECK(feedInhibit            == 1);
    CHECK(spindleInhibit         == 1);

    /* On the machine the program ends up PAUSED at this point (reported by
       halui, not commanded by this component). */
    halui_program_paused();
    tick(3);

    CHECK(statusChuckCoverOpened   == 1);
    CHECK(statusProgramPaused      == 1);
    CHECK(machineOn                == 1);
    CHECK(programLoaded            == 1);
    CHECK(programPaused            == 1);
    CHECK(chuckCoverOpened         == 1);
    CHECK(spindleInhibit           == 1);
    CHECK(feedInhibit              == 1);

    CHECK(statusToolChangeRequired == 0);
    CHECK(statusProgramRunning     == 0);

    /* Operator closes the cover and presses CycleStart: the machine must go back
       to running, i.e. release the feed hold and resume the paused program.
       The spindle was NOT running when the interlock engaged (it was stopped for
       the tool change), so there is nothing to spin back up and no reason to ask
       for a second press. */
    chuckCoverOpened = 0;
    tick(3);

    press_green();
    CHECK(feedInhibit    == 0);
    CHECK(spindleInhibit == 0);
    CHECK(overlay_dbg    == OV_NONE);
    CHECK(resumeProgram  == 1);

    halui_program_running();
    tick(3);
    CHECK(statusProgramRunning == 1);
}

/* ===== Scenarios: end of program ===== */

static void t_completed_latches_and_restarts(void)
{
    run_program();

    halui_program_idle();               /* M2/M30 */
    tick(3);
    CHECK(programCompleted == 1);
    CHECK(last_action_dbg == 6);
    CHECK(programAborted == 0);

    tick(500);
    CHECK(programCompleted == 1);   /* level, not a pulse */

    press_green();                  /* restarts from the beginning */
    CHECK(startProgram == 1);
    CHECK(programCompleted == 0);
}

static void t_abort_from_completed(void)
{
    run_program();
    halui_program_idle();
    tick(3);
    CHECK(programCompleted == 1);

    press_red();
    CHECK(programCompleted == 0);
    CHECK(programAborted == 1);
    CHECK(last_action_dbg == 10);
}

static void t_abort_while_running(void)
{
    run_program();

    press_red();
    CHECK(stopProgram == 1);
    CHECK(programAborted == 0);     /* waits for IDLE */

    halui_program_idle();
    tick(3);
    CHECK(programAborted == 1);
    CHECK(last_action_dbg == 10);
    CHECK(programCompleted == 0);   /* an abort is not a completion */
}

static void t_estop_while_running_aborts(void)
{
    run_program();

    machineOn = 0;
    tick(3);
    CHECK(programAborted == 1);
    CHECK(last_action_dbg == 10);
    CHECK(programCompleted == 0);
}

static void t_abort_releases_feed_hold(void)
{
    run_program();

    chuckCoverOpened = 1;
    tick(3);
    CHECK(feedInhibit == 1);

    press_red();
    CHECK(feedInhibit == 0);
    CHECK(spindleInhibit == 0);
    CHECK(overlay_dbg == OV_NONE);
}

static void t_reload_clears_completed(void)
{
    run_program();
    halui_program_idle();
    tick(3);
    CHECK(programCompleted == 1);

    programLoaded = 0; tick(3);      /* new program loaded */
    programLoaded = 1; tick(3);
    CHECK(programCompleted == 0);
}

/* ===== Scenarios: LED ===== */

static void t_led_running_solid_cover_blinks(void)
{
    run_program();
    tick(50);
    int all_on = 1;
    for (int i = 0; i < 500; i++) { tick(1); if (!cycleStartLedStatus) all_on = 0; }
    CHECK(all_on == 1);

    chuckCoverOpened = 1;
    tick(3);
    int saw_on = 0, saw_off = 0;
    for (int i = 0; i < 500; i++) {
        tick(1);
        if (cycleStartLedStatus) saw_on = 1; else saw_off = 1;
    }
    CHECK(saw_on == 1);
    CHECK(saw_off == 1);
}

static void t_led_off_without_program(void)
{
    machineOn = 1;
    tick(500);
    CHECK(cycleStartLedStatus == 0);
}

/* ===== Runner ===== */

static const struct scenario scenarios[] = {
    { "start from idle",                      t_start_from_idle },
    { "pause and resume",                     t_pause_and_resume },
    { "single block: step and feed hold",     t_single_block_step_and_feed_hold },
    { "single block rise pauses",             t_single_block_rise_pauses },
    { "first green in SB pause steps",        t_first_green_in_sb_pause_steps },
    { "cover while running: full resume",     t_cover_while_running_full_resume },
    { "cover reopened re-arms interlock",     t_cover_reopen_returns_to_interlock },
    { "cover while paused: deliberate restart", t_cover_while_paused_needs_deliberate_restart },
    { "cover in idle blocks start",           t_cover_in_idle_blocks_start },
    { "tool change confirmed once",           t_tool_change_confirmed_once },
    { "tool change: cover cycled is ignored", t_tool_change_cover_cycled_leaves_no_interlock },
    { "tool change confirmed, cover open",    t_tool_change_confirmed_with_cover_open },
    { "tool change, cover open, spindle on",  t_tool_change_cover_open_with_spindle_running },
    { "tool change suppresses SB pause",      t_tool_change_suppresses_sb_pause },
    { "abort during tool change",             t_abort_during_tool_change },
    { "machine repro: waiting at tool change", t_machine_repro_waiting_at_tool_change },
    { "completed latches, green restarts",    t_completed_latches_and_restarts },
    { "abort from completed",                 t_abort_from_completed },
    { "abort while running",                  t_abort_while_running },
    { "e-stop while running aborts",          t_estop_while_running_aborts },
    { "abort releases feed hold",             t_abort_releases_feed_hold },
    { "reload clears completed",              t_reload_clears_completed },
    { "led: running solid, cover blinks",     t_led_running_solid_cover_blinks },
    { "led: off without a program",           t_led_off_without_program },
};

int main(void)
{
    return RUN_SCENARIOS(scenarios);
}
