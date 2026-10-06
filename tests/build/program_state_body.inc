#define startProgram_set(v) (startProgram = (v))
#define startProgram_ptr (&startProgram)
#define pauseProgram_set(v) (pauseProgram = (v))
#define pauseProgram_ptr (&pauseProgram)
#define resumeProgram_set(v) (resumeProgram = (v))
#define resumeProgram_ptr (&resumeProgram)
#define stepProgram_set(v) (stepProgram = (v))
#define stepProgram_ptr (&stepProgram)
#define stopProgram_set(v) (stopProgram = (v))
#define stopProgram_ptr (&stopProgram)
#define programCompleted_set(v) (programCompleted = (v))
#define programCompleted_ptr (&programCompleted)
#define programAborted_set(v) (programAborted = (v))
#define programAborted_ptr (&programAborted)
#define toolChanged_set(v) (toolChanged = (v))
#define toolChanged_ptr (&toolChanged)
#define feedInhibit_set(v) (feedInhibit = (v))
#define feedInhibit_ptr (&feedInhibit)
#define spindleInhibit_set(v) (spindleInhibit = (v))
#define spindleInhibit_ptr (&spindleInhibit)
#define statusProgramIdle_set(v) (statusProgramIdle = (v))
#define statusProgramIdle_ptr (&statusProgramIdle)
#define statusProgramRunning_set(v) (statusProgramRunning = (v))
#define statusProgramRunning_ptr (&statusProgramRunning)
#define statusProgramPaused_set(v) (statusProgramPaused = (v))
#define statusProgramPaused_ptr (&statusProgramPaused)
#define statusIsSingleBlock_set(v) (statusIsSingleBlock = (v))
#define statusIsSingleBlock_ptr (&statusIsSingleBlock)
#define statusStepPaused_set(v) (statusStepPaused = (v))
#define statusStepPaused_ptr (&statusStepPaused)
#define statusChuckCoverOpened_set(v) (statusChuckCoverOpened = (v))
#define statusChuckCoverOpened_ptr (&statusChuckCoverOpened)
#define statusSpindleResumeRequired_set(v) (statusSpindleResumeRequired = (v))
#define statusSpindleResumeRequired_ptr (&statusSpindleResumeRequired)
#define statusFeedResumeRequired_set(v) (statusFeedResumeRequired = (v))
#define statusFeedResumeRequired_ptr (&statusFeedResumeRequired)
#define statusToolChangeRequired_set(v) (statusToolChangeRequired = (v))
#define statusToolChangeRequired_ptr (&statusToolChangeRequired)
#define cycleStartLedStatus_set(v) (cycleStartLedStatus = (v))
#define cycleStartLedStatus_ptr (&cycleStartLedStatus)
#define state_dbg_set(v) (state_dbg = (v))
#define state_dbg_ptr (&state_dbg)
#define postStartPausePending_dbg_set(v) (postStartPausePending_dbg = (v))
#define postStartPausePending_dbg_ptr (&postStartPausePending_dbg)
#define last_action_dbg_set(v) (last_action_dbg = (v))
#define last_action_dbg_ptr (&last_action_dbg)
#define toolChangeAckSent_dbg_set(v) (toolChangeAckSent_dbg = (v))
#define toolChangeAckSent_dbg_ptr (&toolChangeAckSent_dbg)
#define overlay_dbg_set(v) (overlay_dbg = (v))
#define overlay_dbg_ptr (&overlay_dbg)
#include <stdbool.h>
#include "rtapi.h"
#include "hal.h"

/* ===== Base states (explicit, verbose names) ===== */
enum {
    ST_NOT_LOADED             = 0,
    ST_IDLE                   = 1,
    ST_RUNNING                = 2,
    ST_PAUSED_NO_SINGLE_BLOCK = 3,
    ST_PAUSED_SINGLE_BLOCK    = 4,
    ST_UNKNOWN                = 5
};

/* ===== Overlay states (safety) ===== */
enum {
    OV_NONE                   = 0,
    OV_CHUCK_COVER_OPENED     = 1,
    OV_SPINDLE_RESUME_REQ     = 2,
    OV_FEED_RESUME_REQ        = 3
};

/* ===== LED modes ===== */
enum {
    LED_OFF                   = 0,
    LED_IDLE_LOADED_DBLINK    = 1,  /* 100ms ON,100ms OFF,100ms ON,700ms OFF */
    LED_PAUSED_BLINK          = 2,  /* 500ms ON / 500ms OFF */
    LED_RUNNING_ON            = 3,
    LED_COVER_FAST            = 4,  /* 100ms ON / 100ms OFF (repeat) */
    LED_TOOLCHANGE_TRIPLE     = 5   /* 3x (100ms ON,100ms OFF) then 500ms OFF */
};

/* ===== Edge memories & latches ===== */
static bool prevCycleStartLevel        = false;
static bool prevCycleAbortLevel        = false;
static bool prevSingleBlockLevel       = false;
static bool prevChuckCoverLevel        = false;
static bool prevProgramIdleLevel       = false;
static bool prevToolChangeLevel        = false;
static bool prevMachineOnLevel         = false;
static bool prevProgramLoadedLevel     = false;

static bool postStartPausePending      = false;   /* after Start with SB=ON, pause when RUNNING appears */
static bool feedHoldLatched            = false;   /* mirror of the feedInhibit we drive */
static bool pendingAbortedAfterStop    = false;   /* after Stop: when we see IDLE, pulse programAborted once */
static bool programWasActive           = false;   /* program ran (or was paused) since it last went IDLE */
static bool firstStepAfterEnterSB      = false;   /* first GREEN in ST_PAUSED_SINGLE_BLOCK must STEP */
static bool toolChangeAckSent          = false;   /* already pulsed toolChanged for the current request */

static int  prevState                  = ST_UNKNOWN;
static bool firstCall                  = true;

/* Overlay machinery */
static int  overlayState               = OV_NONE;
static int  prevStateBeforeCover       = ST_UNKNOWN;
static bool spindleUninhibitCommanded  = false;   /* set when we drop spindleInhibit in OV_SPINDLE_RESUME_REQ */
static bool spindleWasRunningAtCover   = false;   /* spindle actually turning when the interlock engaged */
static bool coverInterlockEngaged      = false;   /* feed/spindle held because of the cover */

/* ===== Pulse timers (seconds) ===== */
static double t_start       = 0.0;
static double t_pause       = 0.0;
static double t_resume      = 0.0;
static double t_step        = 0.0;
static double t_stop        = 0.0;
static double t_aborted     = 0.0;
static double t_toolChanged = 0.0;

/* ===== LED state ===== */
static int    led_mode       = LED_OFF;
static int    led_mode_prev  = LED_OFF;
static double led_timer      = 0.0;

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
    /* ===== Derive BASE state from halui pins + SB ===== */
    int state = ST_UNKNOWN;

    if (!programLoaded) {
        state = ST_NOT_LOADED;
    } else if (programIdle) {
        state = ST_IDLE;
    } else if (programRunning) {
        state = ST_RUNNING;
    } else if (programPaused) {
        state = singleBlock ? ST_PAUSED_SINGLE_BLOCK : ST_PAUSED_NO_SINGLE_BLOCK;
    } else {
        state = ST_UNKNOWN;
    }

    state_dbg_set(state);

    /* Arm "first step" when entering ST_PAUSED_SINGLE_BLOCK */
    if ((state == ST_PAUSED_SINGLE_BLOCK) && (prevState != ST_PAUSED_SINGLE_BLOCK)) {
        firstStepAfterEnterSB = true;
    }
    if (state != ST_PAUSED_SINGLE_BLOCK) {
        firstStepAfterEnterSB = false;
    }

    /* ===== Edges ===== */
    bool cycleStartLevel    = cycleStart;
    bool cycleAbortLevel    = cycleAbort;
    bool singleBlockLevel   = singleBlock;
    bool chuckCoverLevel    = chuckCoverOpened;
    bool programIdleLevel   = programIdle;
    bool toolChangeLevel    = toolChange;
    bool machineOnLevel     = machineOn;
    bool programLoadedLevel = programLoaded;

    /* On first call, sync prev states to avoid false edges at startup */
    if (firstCall) {
        prevChuckCoverLevel = chuckCoverLevel;
        prevToolChangeLevel = toolChangeLevel;
        prevMachineOnLevel  = machineOnLevel;
        prevProgramLoadedLevel = programLoadedLevel;
        firstCall = false;
    }

    bool programIdleRise    = ( programIdleLevel && !prevProgramIdleLevel );
    bool cycleStartEdge     = ( cycleStartLevel  && !prevCycleStartLevel );
    bool cycleAbortEdge     = ( cycleAbortLevel  && !prevCycleAbortLevel );
    bool singleBlockRise    = ( singleBlockLevel && !prevSingleBlockLevel );
    bool singleBlockFall    = ( !singleBlockLevel && prevSingleBlockLevel );
    bool chuckCoverRise     = ( chuckCoverLevel && !prevChuckCoverLevel );
    bool chuckCoverFall     = ( !chuckCoverLevel && prevChuckCoverLevel );
    bool toolChangeRise     = ( toolChangeLevel && !prevToolChangeLevel );
    bool toolChangeFall     = ( !toolChangeLevel && prevToolChangeLevel );
    bool machineOnFall      = ( !machineOnLevel && prevMachineOnLevel );
    bool programLoadedRise  = ( programLoadedLevel && !prevProgramLoadedLevel );

    /* ===== Tool change guard =====
       While the interpreter waits for a tool change, GREEN confirms the change and
       nothing else. Both edges re-arm the latch, so it can never stay stuck. */
    bool toolChangePending  = ( programLoaded && toolChangeLevel );

    if (toolChangeRise || toolChangeFall) {
        toolChangeAckSent = false;
    }

    /* ===== Overlay: enter/exit logic for chuck cover =====
       The cover is ignored while a tool change is pending: the machine is stopped
       waiting for the operator, who legitimately needs the cover open to swap the
       tool. If it is still open when the change is confirmed, the falling edge of
       toolChange engages the overlay exactly like an opening would. */
    bool coverEngage = ( chuckCoverRise && !toolChangePending ) ||
                       ( toolChangeFall && chuckCoverLevel );

    if (coverEngage) {
        /* remember base state at opening */
        prevStateBeforeCover = state;

        /* A paused program counts as engaged too: the spindle keeps turning while
           paused, so closing the cover must not restart anything by itself. */
        if (state == ST_RUNNING ||
            state == ST_PAUSED_NO_SINGLE_BLOCK ||
            state == ST_PAUSED_SINGLE_BLOCK) {
            overlayState               = OV_CHUCK_COVER_OPENED;
            coverInterlockEngaged      = true;

            feedInhibit_set(1);
            feedHoldLatched            = true;

            spindleInhibit_set(1);
            spindleUninhibitCommanded  = false;

            /* Sampled before our inhibit takes effect: if the spindle was not
               turning, there is nothing to spin back up when the cover closes. */
            spindleWasRunningAtCover   = spindleAtSpeed;

            last_action_dbg_set(7); /* hold_on, informational */
        } else {
            coverInterlockEngaged = false;
            /* IDLE + programLoaded + cover open BEFORE start */
            if (state == ST_IDLE && programLoaded) {
                overlayState = OV_CHUCK_COVER_OPENED;  /* LED_COVER_FAST + CycleStart not allowed */
            } else {
                overlayState = OV_NONE;                /* all other scenarios like before */
            }
        }
    }

    if (chuckCoverFall && (overlayState == OV_CHUCK_COVER_OPENED)) {
        if (coverInterlockEngaged) {
            if (spindleWasRunningAtCover) {
                overlayState              = OV_SPINDLE_RESUME_REQ;
                spindleUninhibitCommanded = false;
            } else {
                /* Nothing to spin up: spindleAtSpeed is gated by spindle-is-on in
                   HAL, so waiting for it here would never end. Straight to feed. */
                overlayState              = OV_FEED_RESUME_REQ;
                spindleInhibit_set(0);
                spindleUninhibitCommanded = false;
            }
        } else {
            /* If we didn't engage because the program was not active, nothing to resume */
            overlayState              = OV_NONE;
        }
        coverInterlockEngaged = false;
    }

    /* Nothing from the cover may stay engaged while a tool change is pending, not
       even an overlay armed before the change was requested. */
    if (toolChangePending && overlayState != OV_NONE) {
        overlayState              = OV_NONE;
        coverInterlockEngaged     = false;
        spindleUninhibitCommanded = false;
        spindleInhibit_set(0);
        if (feedHoldLatched) {
            feedInhibit_set(0);
            feedHoldLatched = false;
            last_action_dbg_set(8); /* hold_off */
        }
    }

    if (!programLoaded) {
        overlayState = OV_NONE;
        coverInterlockEngaged = false;
        spindleUninhibitCommanded = false;

        /* no stale tool change may survive */
        toolChangeAckSent = false;
        toolChanged_set(0);
        t_toolChanged     = 0.0;
    }

    /* The GUI may drop programLoaded when the program ends, so the end-of-program
       latches must NOT be cleared by that. They are cleared when a program is
       (re)loaded, i.e. on the 0->1 edge of programLoaded. */
    if (programLoadedRise) {
        programWasActive        = false;
        pendingAbortedAfterStop = false;
        programCompleted_set(0);
    }

    /* ===== Overlay bootstrap when cover is already open at load =====
       If there was no rise (cover was already 1) and we are IDLE with a program loaded,
       force the overlay for LED_COVER_FAST and to block CycleStart. */
    if (overlayState == OV_NONE && chuckCoverLevel && programLoaded && state == ST_IDLE
        && !toolChangePending) {
        overlayState = OV_CHUCK_COVER_OPENED;   /* without feed/spindle inhibit */
        prevStateBeforeCover = ST_IDLE;
    }


    /* ===== Status pins (base + overlay) ===== */
    statusProgramIdle_set((state == ST_IDLE));
    statusProgramRunning_set((state == ST_RUNNING));
    statusProgramPaused_set((state == ST_PAUSED_NO_SINGLE_BLOCK) || (state == ST_PAUSED_SINGLE_BLOCK));
    statusIsSingleBlock_set(singleBlock);

    statusStepPaused_set(0);
    if (state == ST_PAUSED_SINGLE_BLOCK) {
        statusStepPaused_set((motionType != 0));
    }

    /* Overlay status */
    statusChuckCoverOpened_set((overlayState == OV_CHUCK_COVER_OPENED));
    statusSpindleResumeRequired_set((overlayState == OV_SPINDLE_RESUME_REQ));
    statusFeedResumeRequired_set((overlayState == OV_FEED_RESUME_REQ));
    statusToolChangeRequired_set(toolChangePending);

    overlay_dbg_set(overlayState);

    /* ===== Housekeeping on SB transitions (base behavior) ===== */
    if (singleBlockRise && (state == ST_RUNNING) && (overlayState == OV_NONE) && !toolChangePending) {
        pulse_output_pin(&t_pause, pauseProgram_ptr, outputPulseDuration);
        last_action_dbg_set(2);
        if (feedHoldLatched) {
            feedInhibit_set(0);
            feedHoldLatched = false;
            last_action_dbg_set(8);
        }
    }

    if (singleBlockFall) {
        if (feedHoldLatched) {
            feedInhibit_set(0);
            feedHoldLatched = false;
            last_action_dbg_set(8);
        }
        postStartPausePending = false;
    }

    if (postStartPausePending && singleBlockLevel && (state == ST_RUNNING) && (overlayState == OV_NONE) && !toolChangePending) {
        pulse_output_pin(&t_pause, pauseProgram_ptr, outputPulseDuration);
        last_action_dbg_set(2);
        postStartPausePending = false;
    }

    postStartPausePending_dbg_set(postStartPausePending);

    /* ===== Program end: completed vs aborted =====
       programWasActive is a latch, so we don't miss the end if halui goes through a
       ST_UNKNOWN cycle before IDLE.
         - e-stop / machine off while the program is active: pulse programAborted at once.
         - red button: handled in the abort block (immediate if already IDLE, otherwise
           armed via pendingAbortedAfterStop and pulsed on the first 0->1 of programIdle);
           if CycleStart is pressed meanwhile, cancel the pending pulse.
         - normal M2/M30 end: the program was active and reaches IDLE with no abort armed.
       programCompleted is a LEVEL (for the UI), held until the red button is pressed or
       the program is started again; only programAborted is a pulse. */
    if (state == ST_RUNNING || state == ST_PAUSED_NO_SINGLE_BLOCK || state == ST_PAUSED_SINGLE_BLOCK) {
        programWasActive = true;
        programCompleted_set(0);      /* the program runs again -> leave the completed state */
    }

    if (machineOnFall && programLoaded && (programWasActive || !programIdle)) {
        pulse_output_pin(&t_aborted, programAborted_ptr, outputPulseDuration);
        last_action_dbg_set(10); /* aborted */
        pendingAbortedAfterStop  = false;
        programWasActive         = false;
        programCompleted_set(0);
    }

    if (programIdleRise && (pendingAbortedAfterStop || programWasActive)) {
        if (pendingAbortedAfterStop) {
            pulse_output_pin(&t_aborted, programAborted_ptr, outputPulseDuration);
            last_action_dbg_set(10); /* aborted */
        } else {
            programCompleted_set(1);  /* latched level, cleared by the red button or a restart */
            last_action_dbg_set(6);  /* completed */
        }
        pendingAbortedAfterStop  = false;
        programWasActive         = false;
    }
    if (pendingAbortedAfterStop && cycleStartEdge) {
        pendingAbortedAfterStop  = false;
    }

    /* ===== GREEN behavior ===== */
    if (cycleStartEdge) {
        /* Tool change has priority over everything, including the cover overlay:
           only the button routing is affected, the safety inhibits stay as they are. */
        if (toolChangePending) {
            if (!toolChangeAckSent) {
                pulse_output_pin(&t_toolChanged, toolChanged_ptr, outputPulseDuration);
                last_action_dbg_set(9);
                toolChangeAckSent = true;
            }
            /* further presses while toolChange is still 1: ignored */
        }
        /* Overlay has priority */
        else if (overlayState == OV_CHUCK_COVER_OPENED) {
            /* Ignore CYCLestart while cover is open */
        }
        else if (overlayState == OV_SPINDLE_RESUME_REQ) {
            /* Allow spindle to run; then wait for spindleAtSpeed */
            spindleInhibit_set(0);
            spindleUninhibitCommanded = true;
            /* Transition will happen when spindleAtSpeed==1 */
        }
        else if (overlayState == OV_FEED_RESUME_REQ) {
            /* Respect Single-Block */
            if (!singleBlockLevel) {
                /* Normal: resume feed */
                if (feedHoldLatched) {
                    feedInhibit_set(0);
                    feedHoldLatched = false;
                    last_action_dbg_set(8);
                }
                overlayState = OV_NONE;  /* back to base; base should be Running naturally */

                /* If the program was actually paused while the cover was open (a
                   tool change confirmed in between does that), releasing the feed
                   is not enough - it has to be resumed as well. */
                if (state == ST_PAUSED_NO_SINGLE_BLOCK) {
                    pulse_output_pin(&t_resume, resumeProgram_ptr, outputPulseDuration);
                    last_action_dbg_set(3);
                }
            } else {
                /* SB ON: motionType != 0 -> release hold; motionType == 0 -> STEP */
                if (motionType != 0) {
                    if (feedHoldLatched) {
                        feedInhibit_set(0);
                        feedHoldLatched = false;
                        last_action_dbg_set(8);
                    }
                    /* stay in overlay until motion resolves into base state; then overlay ends */
                    overlayState = OV_NONE;
                } else {
                    if (feedHoldLatched) {
                        feedInhibit_set(0);
                        feedHoldLatched = false;
                        last_action_dbg_set(8);
                    }
                    pulse_output_pin(&t_step, stepProgram_ptr, outputPulseDuration);
                    last_action_dbg_set(4);
                    overlayState = OV_NONE;
                }
            }
        }
        else {
            /* === Base machine behavior (only when no overlay) === */
            switch (state) {

                case ST_IDLE: {
                    /* also the restart from the completed state: runs from the beginning */
                    programCompleted_set(0);
                    pulse_output_pin(&t_start, startProgram_ptr, outputPulseDuration);
                    last_action_dbg_set(1);
                    if (singleBlockLevel) {
                        postStartPausePending = true;
                    }
                } break;

                case ST_RUNNING: {
                    if (!singleBlockLevel) {
                        pulse_output_pin(&t_pause, pauseProgram_ptr, outputPulseDuration);
                        last_action_dbg_set(2);
                    } else {
                        if (motionType != 0) {
                            if (!feedHoldLatched) { feedInhibit_set(1); feedHoldLatched = true; last_action_dbg_set(7); }
                            else                  { feedInhibit_set(0); feedHoldLatched = false; last_action_dbg_set(8); }
                        } else {
                            if (feedHoldLatched) { feedInhibit_set(0); feedHoldLatched = false; last_action_dbg_set(8); }
                            pulse_output_pin(&t_step, stepProgram_ptr, outputPulseDuration);
                            last_action_dbg_set(4);
                        }
                    }
                } break;

                case ST_PAUSED_NO_SINGLE_BLOCK: {
                    pulse_output_pin(&t_resume, resumeProgram_ptr, outputPulseDuration);
                    last_action_dbg_set(3);
                } break;

                case ST_PAUSED_SINGLE_BLOCK: {
                    if (firstStepAfterEnterSB) {
                        if (feedHoldLatched) { feedInhibit_set(0); feedHoldLatched = false; last_action_dbg_set(8); }
                        pulse_output_pin(&t_step, stepProgram_ptr, outputPulseDuration);
                        last_action_dbg_set(4);
                        firstStepAfterEnterSB = false;
                        break;
                    }
                    if (motionType != 0) {
                        if (!feedHoldLatched) { feedInhibit_set(1); feedHoldLatched = true; last_action_dbg_set(7); }
                        else                  { feedInhibit_set(0); feedHoldLatched = false; last_action_dbg_set(8); }
                    } else {
                        if (feedHoldLatched) { feedInhibit_set(0); feedHoldLatched = false; last_action_dbg_set(8); }
                        pulse_output_pin(&t_step, stepProgram_ptr, outputPulseDuration);
                        last_action_dbg_set(4);
                    }
                } break;

                case ST_NOT_LOADED: {
                    /* The GUI may drop programLoaded at the end of the program, while the
                       interpreter still has the file open: GREEN restarts it from the
                       beginning, exactly like in ST_IDLE. */
                    if (programCompleted) {
                        programCompleted_set(0);
                        pulse_output_pin(&t_start, startProgram_ptr, outputPulseDuration);
                        last_action_dbg_set(1);
                        if (singleBlockLevel) {
                            postStartPausePending = true;
                        }
                    }
                } break;

                case ST_UNKNOWN:
                default: {
                    /* ignore */
                } break;
            }
        }
    }

    /* ===== RED behavior (Stop) — applies in ANY state / overlay ===== */
    if (cycleAbortEdge) {
        pulse_output_pin(&t_stop, stopProgram_ptr, outputPulseDuration);
        last_action_dbg_set(5);

        postStartPausePending = false;

        if (feedHoldLatched) {
            feedInhibit_set(0);
            feedHoldLatched = false;
            last_action_dbg_set(8);
        }

        /* reset overlay */
        overlayState = OV_NONE;
        coverInterlockEngaged = false;
        spindleInhibit_set(0);
        spindleUninhibitCommanded = false;

        /* reset tool change: LinuxCNC drops tool-change on abort, we must not
           confirm a tool that was never changed, and nothing may stay latched
           for the next program. */
        toolChangeAckSent = false;
        toolChanged_set(0);
        t_toolChanged     = 0.0;

        /* leaving the completed state: drop the level, then pulse aborted */
        programCompleted_set(0);

        /* arm aborted-after-stop */
        pendingAbortedAfterStop = true;
        /* if we are ALREADY IDLE, pulse aborted right away and disarm the pending flag */
        if (programIdle) {
            pulse_output_pin(&t_aborted, programAborted_ptr, outputPulseDuration);
            last_action_dbg_set(10); /* aborted */
            pendingAbortedAfterStop    = false;
            programWasActive           = false;
        }

        /* and disarm the SB first-step */
        firstStepAfterEnterSB = false;
    }

    /* ===== Overlay progress conditions (no button) ===== */
    if (overlayState == OV_SPINDLE_RESUME_REQ) {
        /* If cover re-opened, go back */
        if (chuckCoverLevel) {
            overlayState = OV_CHUCK_COVER_OPENED;
            spindleUninhibitCommanded = false;
            /* ensure safety levels */
            feedInhibit_set(1);
            feedHoldLatched = true;
            spindleInhibit_set(1);
        } else {
            /* If we already allowed spindle and speed is reached, move on */
            if (spindleUninhibitCommanded && spindleAtSpeed) {
                overlayState = OV_FEED_RESUME_REQ;
            }
        }
    }
    else if (overlayState == OV_FEED_RESUME_REQ) {
        /* If cover re-opens, back to cover-opened */
        if (chuckCoverLevel) {
            overlayState = OV_CHUCK_COVER_OPENED;
            /* safety */
            feedInhibit_set(1);
            feedHoldLatched = true;
            spindleInhibit_set(1);
        }
    }

    /* ===== LED state machine ===== */
    /* Choose LED mode from overlay first (priority), otherwise base state */
    if (!programLoaded) {
        led_mode = LED_OFF;
    } else if (toolChangePending) {
        led_mode = LED_TOOLCHANGE_TRIPLE;       /* 3x 100/100 + 500 off */
    } else if (overlayState == OV_CHUCK_COVER_OPENED) {
        led_mode = LED_COVER_FAST;              /* 100/100 */
    } else if (overlayState == OV_SPINDLE_RESUME_REQ || overlayState == OV_FEED_RESUME_REQ) {
        led_mode = LED_PAUSED_BLINK;            /* 500/500 */
    } else if (state == ST_PAUSED_NO_SINGLE_BLOCK || state == ST_PAUSED_SINGLE_BLOCK) {
        led_mode = LED_PAUSED_BLINK;
    } else if (state == ST_RUNNING) {
        led_mode = LED_RUNNING_ON;
    } else if (state == ST_IDLE && programLoaded) {
        led_mode = LED_IDLE_LOADED_DBLINK;
    } else {
        led_mode = LED_OFF;
    }

    if (led_mode != led_mode_prev) {
        led_timer     = 0.0;
        led_mode_prev = led_mode;
    }

    double dt = fperiod;
    led_timer += dt;
    /* We make a 1.0s cycle for all LED patterns; COVER_FAST uses 0.2s subcycle logic */
    if (led_timer >= 1.0) {
        led_timer -= 1.0;
        if (led_timer < 0.0) led_timer = 0.0;
    }

    switch (led_mode) {
        case LED_OFF:
            cycleStartLedStatus_set(0);
            break;
        case LED_RUNNING_ON:
            cycleStartLedStatus_set(1);
            break;
        case LED_PAUSED_BLINK:
            cycleStartLedStatus_set((led_timer < 0.5) ? 1 : 0);
            break;
        case LED_IDLE_LOADED_DBLINK:
            if ((led_timer < 0.1) || (led_timer >= 0.2 && led_timer < 0.3))
                cycleStartLedStatus_set(1);
            else
                cycleStartLedStatus_set(0);
            break;
        case LED_COVER_FAST: {
            /* 100ms ON / 100ms OFF repeating -> map led_timer [0..1) into 0.2s repeating window */
            double t = led_timer;
            /* Reduce to 0..0.2 window */
            while (t >= 0.2) t -= 0.2;
            cycleStartLedStatus_set((t < 0.1) ? 1 : 0);
        } break;
        case LED_TOOLCHANGE_TRIPLE:
            /* ON 0.0-0.1, 0.2-0.3, 0.4-0.5 ; OFF 0.5-1.0 */
            if ((led_timer < 0.1) ||
                (led_timer >= 0.2 && led_timer < 0.3) ||
                (led_timer >= 0.4 && led_timer < 0.5))
                cycleStartLedStatus_set(1);
            else
                cycleStartLedStatus_set(0);
            break;
        default:
            cycleStartLedStatus_set(0);
            break;
    }

    /* ===== Update pulse timers ===== */
    if (t_start  > 0.0) { t_start  -= dt; if (t_start  <= 0.0) { t_start  = 0.0; startProgram_set(0); } }
    if (t_pause  > 0.0) { t_pause  -= dt; if (t_pause  <= 0.0) { t_pause  = 0.0; pauseProgram_set(0); } }
    if (t_resume > 0.0) { t_resume -= dt; if (t_resume <= 0.0) { t_resume = 0.0; resumeProgram_set(0); } }
    if (t_step   > 0.0) { t_step   -= dt; if (t_step   <= 0.0) { t_step   = 0.0; stepProgram_set(0); } }
    if (t_stop   > 0.0) { t_stop   -= dt; if (t_stop   <= 0.0) { t_stop   = 0.0; stopProgram_set(0); } }
    if (t_aborted  > 0.0) { t_aborted  -= dt; if (t_aborted  <= 0.0) { t_aborted  = 0.0; programAborted_set(0); } }
    if (t_toolChanged > 0.0) { t_toolChanged -= dt; if (t_toolChanged <= 0.0) { t_toolChanged = 0.0; toolChanged_set(0); } }

    /* ===== Latch inputs & base state ===== */
    prevProgramIdleLevel = programIdleLevel;
    prevCycleStartLevel  = cycleStartLevel;
    prevCycleAbortLevel  = cycleAbortLevel;
    prevSingleBlockLevel = singleBlockLevel;
    prevChuckCoverLevel  = chuckCoverLevel;
    prevToolChangeLevel  = toolChangeLevel;
    prevMachineOnLevel   = machineOnLevel;
    prevProgramLoadedLevel = programLoadedLevel;
    prevState            = state;

    toolChangeAckSent_dbg_set(toolChangeAckSent);
}
