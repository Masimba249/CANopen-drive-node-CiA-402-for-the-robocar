/**
 * @file cia402.c
 * @brief CiA 402 state machine + Profile Velocity ramp generator.
 *
 * The transition numbers in the comments (T1..T16) are the ones from the
 * CiA 402 state diagram, so this file can be read side by side with the spec.
 */
#include "cia402.h"
#include <stddef.h>

/* ---- controlword decoding ---------------------------------------------- */

typedef enum {
    CMD_NONE = 0,
    CMD_SHUTDOWN,
    CMD_SWITCH_ON,       /* also "disable operation" - same bit pattern */
    CMD_ENABLE_OPERATION,
    CMD_DISABLE_VOLTAGE,
    CMD_QUICK_STOP,
} cmd_t;

/**
 * CiA 402 table 3. The masks matter as much as the values: bit 3
 * (enable operation) is "don't care" for Shutdown, and Disable Voltage is
 * simply "enable voltage = 0", which is why it has to be tested first.
 */
static cmd_t decode_command(uint16_t cw)
{
    if ((cw & 0x0082u) == 0x0000u) return CMD_DISABLE_VOLTAGE;   /* xxxx xx0x xxxx xx0x */
    if ((cw & 0x0086u) == 0x0002u) return CMD_QUICK_STOP;        /* quick stop is active low */
    if ((cw & 0x008Fu) == 0x000Fu) return CMD_ENABLE_OPERATION;
    if ((cw & 0x008Fu) == 0x0007u) return CMD_SWITCH_ON;
    if ((cw & 0x0087u) == 0x0006u) return CMD_SHUTDOWN;
    return CMD_NONE;
}

const char *cia402_state_name(cia402_state_t s)
{
    switch (s) {
    case CIA402_NOT_READY_TO_SWITCH_ON: return "NOT READY TO SWITCH ON";
    case CIA402_SWITCH_ON_DISABLED:     return "SWITCH ON DISABLED";
    case CIA402_READY_TO_SWITCH_ON:     return "READY TO SWITCH ON";
    case CIA402_SWITCHED_ON:            return "SWITCHED ON";
    case CIA402_OPERATION_ENABLED:      return "OPERATION ENABLED";
    case CIA402_QUICK_STOP_ACTIVE:      return "QUICK STOP ACTIVE";
    case CIA402_FAULT_REACTION_ACTIVE:  return "FAULT REACTION ACTIVE";
    case CIA402_FAULT:                  return "FAULT";
    default:                            return "?";
    }
}

cia402_state_t cia402_decode_status(uint16_t sw)
{
    if ((sw & 0x004Fu) == 0x0000u) return CIA402_NOT_READY_TO_SWITCH_ON;
    if ((sw & 0x004Fu) == 0x0040u) return CIA402_SWITCH_ON_DISABLED;
    if ((sw & 0x006Fu) == 0x0021u) return CIA402_READY_TO_SWITCH_ON;
    if ((sw & 0x006Fu) == 0x0023u) return CIA402_SWITCHED_ON;
    if ((sw & 0x006Fu) == 0x0027u) return CIA402_OPERATION_ENABLED;
    if ((sw & 0x006Fu) == 0x0007u) return CIA402_QUICK_STOP_ACTIVE;
    if ((sw & 0x004Fu) == 0x000Fu) return CIA402_FAULT_REACTION_ACTIVE;
    if ((sw & 0x004Fu) == 0x0008u) return CIA402_FAULT;
    return CIA402_NOT_READY_TO_SWITCH_ON;
}

/* ---- helpers ------------------------------------------------------------ */

static bool power_stage_wanted(cia402_state_t s)
{
    return s == CIA402_SWITCHED_ON       || s == CIA402_OPERATION_ENABLED ||
           s == CIA402_QUICK_STOP_ACTIVE || s == CIA402_FAULT_REACTION_ACTIVE;
}

static void goto_state(cia402_axis_t *a, cia402_state_t next)
{
    if (a->state == next) return;
    cia402_state_t prev = a->state;
    a->state = next;

    if (a->hw && a->hw->power_stage) {
        bool want = power_stage_wanted(next);
        if (want != power_stage_wanted(prev)) a->hw->power_stage(a->axis, want, a->hw->ctx);
    }
    if (!power_stage_wanted(next)) {
        /* Not powered: the ramp has no meaning, drop it so re-enabling the
         * drive always starts from standstill rather than from a stale demand. */
        a->ramp = 0;
        a->ramp_frac = 0;
        if (a->hw && a->hw->set_demand) a->hw->set_demand(a->axis, 0, a->hw->ctx);
    }
    if (a->hw && a->hw->on_state_change) {
        a->hw->on_state_change(a->axis, prev, next, a->hw->ctx);
    }
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

/* ---- public API --------------------------------------------------------- */

void cia402_init(cia402_axis_t *a, cia402_vars_t *v, uint8_t axis,
                 const cia402_hw_t *hw)
{
    a->v    = v;
    a->hw   = hw;
    a->axis = axis;
    a->state = CIA402_NOT_READY_TO_SWITCH_ON;
    a->prev_controlword = 0;
    a->voltage_enabled = false;
    a->remote = false;
    a->limited = false;
    a->target_reached_ms = 0;
    a->speed_zero_ms = 0;
    a->ramp = 0;
    a->ramp_frac = 0;

    v->controlword = 0;
    v->statusword  = 0;
    v->mode_of_operation = MODE_PROFILE_VELOCITY;
    v->mode_display      = MODE_NO_MODE;
    v->supported_drive_modes = SUPPORTED_DRIVE_MODES;
    v->target_velocity = 0;
    v->velocity_demand = 0;

    if (hw && hw->power_stage) hw->power_stage(axis, false, hw->ctx);
    if (hw && hw->set_demand)  hw->set_demand(axis, 0, hw->ctx);
}

void cia402_set_voltage_enabled(cia402_axis_t *a, bool on) { a->voltage_enabled = on; }
void cia402_set_remote(cia402_axis_t *a, bool on)          { a->remote = on; }

void cia402_set_warning(cia402_axis_t *a, bool on)
{
    if (on) a->v->statusword |=  SW_WARNING;
    else    a->v->statusword &= (uint16_t)~SW_WARNING;
}

void cia402_fault(cia402_axis_t *a)
{
    if (a->state == CIA402_FAULT || a->state == CIA402_FAULT_REACTION_ACTIVE) return;
    goto_state(a, CIA402_FAULT_REACTION_ACTIVE);   /* T13 */
}

/* ---- state machine ------------------------------------------------------ */

static void run_state_machine(cia402_axis_t *a)
{
    uint16_t cw = a->v->controlword;
    cmd_t cmd = decode_command(cw);

    /* Fault reset acts on the 0 -> 1 edge of bit 7 only, so a master that
     * parks the bit at 1 cannot hold the drive in a reset loop. */
    bool fault_reset_edge = ((cw & CW_FAULT_RESET) != 0) &&
                            ((a->prev_controlword & CW_FAULT_RESET) == 0);
    a->prev_controlword = cw;

    switch (a->state) {

    case CIA402_NOT_READY_TO_SWITCH_ON:
        /* T1: self-test and parameter load are complete by the time the
         * application calls cia402_init(), so this is unconditional. */
        goto_state(a, CIA402_SWITCH_ON_DISABLED);
        break;

    case CIA402_SWITCH_ON_DISABLED:
        if (cmd == CMD_SHUTDOWN) goto_state(a, CIA402_READY_TO_SWITCH_ON);   /* T2 */
        break;

    case CIA402_READY_TO_SWITCH_ON:
        if (cmd == CMD_ENABLE_OPERATION) {
            /* T3 + T4 in one step: legal, and what most masters do. */
            goto_state(a, CIA402_SWITCHED_ON);
            goto_state(a, CIA402_OPERATION_ENABLED);
        } else if (cmd == CMD_SWITCH_ON) {
            goto_state(a, CIA402_SWITCHED_ON);                               /* T3 */
        } else if (cmd == CMD_DISABLE_VOLTAGE || cmd == CMD_QUICK_STOP) {
            goto_state(a, CIA402_SWITCH_ON_DISABLED);                        /* T7 */
        }
        break;

    case CIA402_SWITCHED_ON:
        if (cmd == CMD_ENABLE_OPERATION) {
            goto_state(a, CIA402_OPERATION_ENABLED);                         /* T4 */
        } else if (cmd == CMD_SHUTDOWN) {
            goto_state(a, CIA402_READY_TO_SWITCH_ON);                        /* T6 */
        } else if (cmd == CMD_DISABLE_VOLTAGE || cmd == CMD_QUICK_STOP) {
            goto_state(a, CIA402_SWITCH_ON_DISABLED);                        /* T10 */
        }
        break;

    case CIA402_OPERATION_ENABLED:
        if (cmd == CMD_QUICK_STOP) {
            goto_state(a, CIA402_QUICK_STOP_ACTIVE);                         /* T11 */
        } else if (cmd == CMD_DISABLE_VOLTAGE) {
            goto_state(a, CIA402_SWITCH_ON_DISABLED);                        /* T9 */
        } else if (cmd == CMD_SHUTDOWN) {
            goto_state(a, CIA402_READY_TO_SWITCH_ON);                        /* T8 */
        } else if (cmd == CMD_SWITCH_ON) {
            goto_state(a, CIA402_SWITCHED_ON);                               /* T5 */
        }
        break;

    case CIA402_QUICK_STOP_ACTIVE:
        if (cmd == CMD_DISABLE_VOLTAGE) {
            goto_state(a, CIA402_SWITCH_ON_DISABLED);                        /* T12 */
        } else if (cmd == CMD_ENABLE_OPERATION &&
                   a->v->quick_stop_option >= 5 && a->v->quick_stop_option <= 7) {
            /* T16 only exists for option codes 5..7, which say "stay in
             * QUICK STOP ACTIVE when the ramp is finished". */
            goto_state(a, CIA402_OPERATION_ENABLED);
        } else if (a->ramp == 0 &&
                   a->v->quick_stop_option >= 1 && a->v->quick_stop_option <= 3) {
            goto_state(a, CIA402_SWITCH_ON_DISABLED);                        /* T12 */
        }
        break;

    case CIA402_FAULT_REACTION_ACTIVE:
        /* Option code 0 = coast: no ramp, straight to FAULT. */
        if (a->v->fault_reaction_option == 0 || a->ramp == 0) {
            goto_state(a, CIA402_FAULT);                                     /* T14 */
        }
        break;

    case CIA402_FAULT:
        if (fault_reset_edge) {
            if (a->hw && a->hw->on_fault_reset) a->hw->on_fault_reset(a->axis, a->hw->ctx);
            goto_state(a, CIA402_SWITCH_ON_DISABLED);                        /* T15 */
        }
        break;

    default:
        goto_state(a, CIA402_NOT_READY_TO_SWITCH_ON);
        break;
    }
}

/* ---- ramp generator ----------------------------------------------------- */

static void run_ramp(cia402_axis_t *a, uint32_t dt_ms)
{
    cia402_vars_t *v = a->v;
    int32_t desired;
    uint32_t rate;   /* velocity units per second */

    a->limited = false;

    switch (a->state) {
    case CIA402_OPERATION_ENABLED: {
        int32_t max = (v->max_profile_velocity > 0x7FFFFFFFu)
                        ? 0x7FFFFFFF : (int32_t)v->max_profile_velocity;
        desired = v->target_velocity;
        if (max > 0 && iabs32(desired) > max) {
            desired = clamp_i32(desired, -max, max);
            a->limited = true;
        }
        if (v->controlword & CW_HALT) desired = 0;
        if (v->mode_display != MODE_PROFILE_VELOCITY) desired = 0;

        bool speeding_up = (iabs32(desired) > iabs32(a->ramp)) &&
                           ((desired ^ a->ramp) >= 0);   /* same sign */
        rate = speeding_up ? v->profile_acceleration : v->profile_deceleration;
        break;
    }
    case CIA402_QUICK_STOP_ACTIVE:
        desired = 0;
        rate = v->quick_stop_deceleration;
        break;

    case CIA402_FAULT_REACTION_ACTIVE:
        desired = 0;
        /* Option code 2 = "quick stop ramp"; anything else stops instantly. */
        rate = (v->fault_reaction_option == 2) ? v->quick_stop_deceleration : 0;
        break;

    default:
        a->ramp = 0;
        a->ramp_frac = 0;
        v->velocity_demand = 0;
        return;
    }

    if (rate == 0) {
        a->ramp = desired;             /* no limit configured: step change */
        a->ramp_frac = 0;
    } else if (a->ramp != desired) {
        /* step = rate * dt / 1000, with the remainder carried so that a 1 ms
         * tick does not quantise the whole ramp away. */
        int64_t acc = (int64_t)a->ramp_frac + (int64_t)rate * (int64_t)dt_ms;
        int32_t step = (int32_t)(acc / 1000);
        a->ramp_frac = (int32_t)(acc % 1000);

        if (a->ramp < desired) {
            a->ramp = (int32_t)((int64_t)a->ramp + step > desired ? desired : a->ramp + step);
        } else {
            a->ramp = (int32_t)((int64_t)a->ramp - step < desired ? desired : a->ramp - step);
        }
        if (a->ramp == desired) a->ramp_frac = 0;
    }

    v->velocity_demand = a->ramp;
    if (a->hw && a->hw->set_demand) a->hw->set_demand(a->axis, a->ramp, a->hw->ctx);
}

/* ---- statusword --------------------------------------------------------- */

static uint16_t state_bits(cia402_state_t s)
{
    switch (s) {
    case CIA402_NOT_READY_TO_SWITCH_ON: return 0x0000u;
    case CIA402_SWITCH_ON_DISABLED:     return 0x0040u;
    case CIA402_READY_TO_SWITCH_ON:     return 0x0021u;
    case CIA402_SWITCHED_ON:            return 0x0023u;
    case CIA402_OPERATION_ENABLED:      return 0x0027u;
    case CIA402_QUICK_STOP_ACTIVE:      return 0x0007u;
    case CIA402_FAULT_REACTION_ACTIVE:  return 0x000Fu;
    case CIA402_FAULT:                  return 0x0008u;
    default:                            return 0x0000u;
    }
}

static void update_statusword(cia402_axis_t *a, uint32_t dt_ms)
{
    cia402_vars_t *v = a->v;
    uint16_t warning = v->statusword & SW_WARNING;   /* set by the application */
    uint16_t sw = state_bits(a->state) | warning;

    if (a->voltage_enabled) sw |= SW_VOLTAGE_ENABLED;
    if (a->remote)          sw |= SW_REMOTE;
    if (a->limited)         sw |= SW_INTERNAL_LIMIT;

    /* Bit 10 "target reached": inside 606Dh for at least 606Eh milliseconds. */
    int32_t err = v->velocity_actual - v->target_velocity;
    if (a->state == CIA402_OPERATION_ENABLED && !(v->controlword & CW_HALT)) {
        if (iabs32(err) <= (int32_t)v->velocity_window) {
            if (a->target_reached_ms < 0xFFFFu) {
                uint32_t t = a->target_reached_ms + dt_ms;
                a->target_reached_ms = (t > 0xFFFFu) ? 0xFFFFu : (uint16_t)t;
            }
        } else {
            a->target_reached_ms = 0;
        }
    } else {
        /* HALT and the non-operational states report "target reached" once
         * the drive has actually come to rest. */
        a->target_reached_ms = (v->velocity_actual == 0) ? 0xFFFFu : 0;
    }
    if (a->target_reached_ms >= v->velocity_window_time) sw |= SW_TARGET_REACHED;

    /* Bit 12 in pv mode is "Speed": 1 means the speed is (effectively) zero. */
    if (iabs32(v->velocity_actual) <= (int32_t)v->velocity_threshold) {
        uint32_t t = a->speed_zero_ms + dt_ms;
        a->speed_zero_ms = (t > 0xFFFFu) ? 0xFFFFu : (uint16_t)t;
    } else {
        a->speed_zero_ms = 0;
    }
    if (a->speed_zero_ms >= v->velocity_threshold_time) sw |= SW_SPEED_ZERO;

    v->statusword = sw;
}

/* ---- tick --------------------------------------------------------------- */

void cia402_tick(cia402_axis_t *a, uint32_t dt_ms)
{
    cia402_vars_t *v = a->v;

    /* 6060h -> 6061h. An unsupported mode is reported back as "no mode". */
    if (v->mode_of_operation == MODE_PROFILE_VELOCITY) {
        v->mode_display = MODE_PROFILE_VELOCITY;
    } else {
        v->mode_display = MODE_NO_MODE;
    }

    run_state_machine(a);
    run_ramp(a, dt_ms);
    update_statusword(a, dt_ms);
}
