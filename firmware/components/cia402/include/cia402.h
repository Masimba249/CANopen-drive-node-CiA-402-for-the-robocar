/**
 * @file cia402.h
 * @brief CiA 402 drive profile: state machine, controlword/statusword
 *        encoding and a Profile Velocity mode ramp generator.
 *
 * Scope
 * -----
 * One instance per axis. The robocar has two (left = axis 0 at 0x6000,
 * right = axis 1 at 0x6800, per the CiA 402 multi-axis offset of 0x800).
 *
 * Only Profile Velocity mode (-3 ... no; mode 3) is implemented, which is the
 * mode that matters for a differential-drive robot. Position and torque modes
 * are rejected at 6060h and 6061h reports 0, which is exactly what the
 * standard says an unsupported mode must do.
 *
 * Units
 * -----
 * Velocity: 0.1 rpm at the *wheel* (output shaft). +ve = forward.
 * Acceleration/deceleration: velocity units per second.
 * See docs/object_dictionary.md for the conversion to m/s.
 *
 * This file is plain C99 and is unit-tested on the host.
 */
#ifndef CIA402_H
#define CIA402_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- controlword (6040h) bits ------------------------------------------ */
#define CW_SWITCH_ON        0x0001u
#define CW_ENABLE_VOLTAGE   0x0002u
#define CW_QUICK_STOP       0x0004u   /* active low: 0 = quick stop requested */
#define CW_ENABLE_OPERATION 0x0008u
#define CW_OP_MODE_0        0x0010u
#define CW_OP_MODE_1        0x0020u
#define CW_OP_MODE_2        0x0040u
#define CW_FAULT_RESET      0x0080u   /* acted on at the rising edge only */
#define CW_HALT             0x0100u

/* ---- statusword (6041h) bits ------------------------------------------- */
#define SW_READY_TO_SWITCH_ON 0x0001u
#define SW_SWITCHED_ON        0x0002u
#define SW_OPERATION_ENABLED  0x0004u
#define SW_FAULT              0x0008u
#define SW_VOLTAGE_ENABLED    0x0010u
#define SW_QUICK_STOP         0x0020u   /* 0 = quick stop active */
#define SW_SWITCH_ON_DISABLED 0x0040u
#define SW_WARNING            0x0080u
#define SW_REMOTE             0x0200u
#define SW_TARGET_REACHED     0x0400u
#define SW_INTERNAL_LIMIT     0x0800u
#define SW_SPEED_ZERO         0x1000u   /* pv mode: "Speed" bit 12 */
#define SW_MAX_SLIPPAGE       0x2000u

/** Statusword masks used to decode a state (CiA 402 table 5). */
#define SW_STATE_MASK_A 0x004Fu   /* distinguishes Not-Ready / Switch-On-Disabled */
#define SW_STATE_MASK_B 0x006Fu   /* all the others                               */

/* ---- modes of operation (6060h / 6061h) -------------------------------- */
#define MODE_NO_MODE            0
#define MODE_PROFILE_POSITION   1
#define MODE_VELOCITY           2
#define MODE_PROFILE_VELOCITY   3
#define MODE_PROFILE_TORQUE     4
#define MODE_HOMING             6
#define MODE_INTERPOLATED       7

/** 6502h bit for each supported mode. Bit 2 = profile velocity. */
#define SUPPORTED_DRIVE_MODES   0x00000004u

typedef enum {
    CIA402_NOT_READY_TO_SWITCH_ON = 0,
    CIA402_SWITCH_ON_DISABLED,
    CIA402_READY_TO_SWITCH_ON,
    CIA402_SWITCHED_ON,
    CIA402_OPERATION_ENABLED,
    CIA402_QUICK_STOP_ACTIVE,
    CIA402_FAULT_REACTION_ACTIVE,
    CIA402_FAULT,
} cia402_state_t;

/**
 * Every profile object of one axis, laid out as the object dictionary sees
 * it. od_data.c points OD entries directly at these fields, so an SDO or
 * RPDO write lands here with no copying.
 */
typedef struct {
    /* written by the master */
    uint16_t controlword;           /* 6040h */
    int8_t   mode_of_operation;     /* 6060h */
    int32_t  target_velocity;       /* 60FFh */
    uint32_t max_profile_velocity;  /* 607Fh */
    uint32_t profile_acceleration;  /* 6083h */
    uint32_t profile_deceleration;  /* 6084h */
    uint32_t quick_stop_deceleration; /* 6085h */
    int16_t  quick_stop_option;     /* 605Ah */
    int16_t  shutdown_option;       /* 605Bh */
    int16_t  disable_op_option;     /* 605Ch */
    int16_t  fault_reaction_option; /* 605Eh */
    int16_t  abort_connection_option; /* 6007h */
    uint16_t velocity_window;       /* 606Dh */
    uint16_t velocity_window_time;  /* 606Eh */
    uint16_t velocity_threshold;    /* 606Fh */
    uint16_t velocity_threshold_time; /* 6070h */

    /* published by the drive */
    uint16_t statusword;            /* 6041h */
    int8_t   mode_display;          /* 6061h */
    int32_t  position_internal;     /* 6063h - raw encoder counts */
    int32_t  position_actual;       /* 6064h - user units             */
    int32_t  velocity_demand;       /* 606Bh - ramp generator output  */
    int32_t  velocity_actual;       /* 606Ch - measured or estimated  */
    int16_t  torque_actual;         /* 6077h - per mille of rated     */
    int16_t  current_actual;        /* 6078h - mA                     */
    uint32_t supported_drive_modes; /* 6502h */
} cia402_vars_t;

/** Hardware the profile drives. Supplied by the application. */
typedef struct {
    /** Energise or de-energise the power stage. */
    void (*power_stage)(uint8_t axis, bool on, void *ctx);
    /** Apply a velocity demand in profile units (only called when enabled). */
    void (*set_demand)(uint8_t axis, int32_t demand, void *ctx);
    /** Optional: called on the 0->1 edge of fault reset, to clear latched errors. */
    void (*on_fault_reset)(uint8_t axis, void *ctx);
    /** Optional: called on every state change (logging, EMCY, telemetry). */
    void (*on_state_change)(uint8_t axis, cia402_state_t from, cia402_state_t to, void *ctx);
    void *ctx;
} cia402_hw_t;

typedef struct {
    cia402_vars_t     *v;
    const cia402_hw_t *hw;
    uint8_t            axis;

    cia402_state_t     state;
    uint16_t           prev_controlword;
    bool               voltage_enabled;   /* DC link present and in range */
    bool               remote;            /* NMT operational: cw is honoured */
    bool               limited;           /* demand clipped by 607Fh        */
    uint16_t           target_reached_ms;
    uint16_t           speed_zero_ms;
    int32_t            ramp;              /* internal ramp state, = 606Bh   */
    int32_t            ramp_frac;         /* sub-unit accumulator           */
} cia402_axis_t;

/** Reset the axis to NOT READY TO SWITCH ON and load sane defaults. */
void cia402_init(cia402_axis_t *a, cia402_vars_t *v, uint8_t axis,
                 const cia402_hw_t *hw);

/**
 * Advance the state machine and the ramp generator.
 * @param dt_ms milliseconds since the previous call (the control period).
 */
void cia402_tick(cia402_axis_t *a, uint32_t dt_ms);

/** Report a device fault (over-current, stall, encoder loss, ...). */
void cia402_fault(cia402_axis_t *a);

/** DC-link / battery presence, drives statusword bit 4. */
void cia402_set_voltage_enabled(cia402_axis_t *a, bool on);

/** NMT OPERATIONAL drives statusword bit 9 ("remote"). */
void cia402_set_remote(cia402_axis_t *a, bool on);

/** Warning bit (bit 7) - a condition worth reporting that is not a fault. */
void cia402_set_warning(cia402_axis_t *a, bool on);

/** Decode a statusword back into a state (used by tests and by the master). */
cia402_state_t cia402_decode_status(uint16_t statusword);

const char *cia402_state_name(cia402_state_t s);

#ifdef __cplusplus
}
#endif
#endif /* CIA402_H */
