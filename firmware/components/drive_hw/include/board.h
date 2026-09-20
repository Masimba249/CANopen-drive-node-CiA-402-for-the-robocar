/**
 * @file board.h
 * @brief Pin map and board constants for the robocar drive node.
 *
 * Target: ESP32 (WROOM-32) + SN65HVD230 CAN transceiver + TB6612FNG dual
 * H-bridge + two quadrature-encoder gearmotors.
 *
 * Pin choices worth knowing about:
 *  - GPIO34/35/36/39 are input-only and are on ADC1, which is the ADC that
 *    keeps working while Wi-Fi is up. Encoders take 34/35 (input-only, no
 *    pull-ups needed with an encoder that drives both rails); the three
 *    analogue inputs take the remaining ADC1 channels.
 *  - Strapping pins (0, 2, 12, 15) and the flash pins (6-11) are left alone.
 *  - TWAI can be routed to almost any pin through the GPIO matrix; 21/22 are
 *    used here because they are free on every WROOM dev board.
 */
#ifndef BOARD_H
#define BOARD_H

/* --- CAN (TWAI) --------------------------------------------------------- */
#define BOARD_CAN_TX_GPIO     21
#define BOARD_CAN_RX_GPIO     22
/** Optional: SN65HVD230 Rs pin. -1 if it is hard-wired to GND (high speed). */
#define BOARD_CAN_STBY_GPIO   (-1)

/* --- motor driver (TB6612FNG) ------------------------------------------- */
#define BOARD_M0_IN1_GPIO     16   /* left wheel  */
#define BOARD_M0_IN2_GPIO     17
#define BOARD_M1_IN1_GPIO     18   /* right wheel */
#define BOARD_M1_IN2_GPIO     19
#define BOARD_MOTOR_STBY_GPIO 23   /* low = both bridges in standby */

/* --- quadrature encoders ------------------------------------------------ */
#define BOARD_ENC0_A_GPIO     34
#define BOARD_ENC0_B_GPIO     35
#define BOARD_ENC1_A_GPIO     26
#define BOARD_ENC1_B_GPIO     27

/* --- analogue inputs (all ADC1) ----------------------------------------- */
#define BOARD_ISENSE0_GPIO    36   /* ADC1_CH0 */
#define BOARD_ISENSE1_GPIO    39   /* ADC1_CH3 */
#define BOARD_VBAT_GPIO       32   /* ADC1_CH4 */

/**
 * Current sense scaling: shunt amplifier output in mV per ampere.
 * A 0.05 R shunt into an INA181A2 (x50) gives 2.5 mV/mA -> 2500 mV/A.
 */
#define BOARD_ISENSE_MV_PER_A 2500
/** Zero-current output of the amplifier, mV (0 for a single-ended sensor). */
#define BOARD_ISENSE_OFFSET_MV 0

/** Battery divider: Vbat * R2/(R1+R2). 100k/22k -> ratio 0.1803, x1000. */
#define BOARD_VBAT_DIV_X1000  180

/* --- LED --------------------------------------------------------------- */
#define BOARD_STATUS_LED_GPIO 2

/* --- timing ------------------------------------------------------------- */
/** CANopen + control task period. 1 ms gives the PDO event timer 1 ms jitter. */
#define BOARD_TICK_MS         1
/** Motor control loop period. */
#define BOARD_CONTROL_MS      5

#endif /* BOARD_H */
