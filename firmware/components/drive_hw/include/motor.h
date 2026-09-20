/**
 * @file motor.h
 * @brief Two-channel H-bridge drive with quadrature feedback, a PI velocity
 *        loop, and current/stall/voltage monitoring.
 *
 * Velocity is in the profile unit defined by the CiA 402 layer:
 * 0.1 rpm at the wheel. Positive is forward.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "od_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Faults the motor layer can report, mirrored into EMCY by the app. */
typedef enum {
    MOTOR_FAULT_NONE        = 0,
    MOTOR_FAULT_OVERCURRENT = 1u << 0,
    MOTOR_FAULT_STALL       = 1u << 1,
    MOTOR_FAULT_ENCODER     = 1u << 2,
} motor_fault_t;

/** Configure GPIO, LEDC, PCNT and ADC from the object dictionary values. */
bool motor_init(void);

/** Re-apply PWM frequency / gains after a configuration change. */
bool motor_reconfigure(void);

/** Energise or de-energise one bridge (CiA 402 power stage). */
void motor_set_enable(uint8_t axis, bool on);

/** Set the velocity demand for one axis, in profile units. */
void motor_set_demand(uint8_t axis, int32_t demand);

/**
 * Run one control period: read encoders and ADC, close the velocity loop,
 * update 6063h/6064h/606Ch/6077h/6078h and 2004h in the object dictionary.
 *
 * @param dt_ms elapsed time since the last call.
 */
void motor_update(uint32_t dt_ms);

/** Latched faults for one axis; cleared by motor_clear_faults(). */
uint32_t motor_faults(uint8_t axis);
void     motor_clear_faults(uint8_t axis);

/** Supply voltage in mV, from the battery divider. */
uint16_t motor_supply_mv(void);

/** Stop both bridges immediately. Safe to call from anywhere. */
void motor_emergency_stop(void);

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_H */
