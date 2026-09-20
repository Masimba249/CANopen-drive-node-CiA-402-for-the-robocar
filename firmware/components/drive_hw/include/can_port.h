/**
 * @file can_port.h
 * @brief TWAI (CAN 2.0) driver binding for the CANopen stack.
 *
 * Bit timing
 * ----------
 * The ESP32 TWAI peripheral is clocked from APB (80 MHz). A bit time is
 *
 *     t_bit = (BRP / 80 MHz) * (1 + TSEG1 + TSEG2)
 *
 * The ESP-IDF TWAI_TIMING_CONFIG_* macros pick BRP/TSEG1/TSEG2 for the
 * standard rates with the sample point at 75-87.5 %, which is what CiA 301
 * recommends. SJW is left at 3 for margin against oscillator drift.
 * See docs/bit_timing.md for the arithmetic and the cable-length limits.
 */
#ifndef CAN_PORT_H
#define CAN_PORT_H

#include "co_types.h"
#include "od_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Install and start the TWAI driver.
 * @param bitrate_kbps 125, 250, 500 or 1000.
 * @param listen_only  true installs the peripheral in listen-only mode, which
 *                     is handy for sniffing a live bus without ACKing.
 */
bool can_port_init(uint32_t bitrate_kbps, bool listen_only);

/** co_send_fn: queue one frame. Non-blocking, returns false if the TX queue is full. */
bool can_port_send(void *ctx, const co_msg_t *msg);

/**
 * Block until a frame arrives or the timeout expires.
 * @return true if @p msg was filled in.
 */
bool can_port_receive(co_msg_t *msg, uint32_t timeout_ms);

/**
 * Service driver alerts: error counters, RX overruns and bus-off recovery.
 * Updates @p diag in place and returns a bit mask of events:
 */
#define CAN_EVT_BUS_OFF      0x01u
#define CAN_EVT_RECOVERED    0x02u
#define CAN_EVT_ERR_PASSIVE  0x04u
#define CAN_EVT_RX_OVERRUN   0x08u

uint32_t can_port_service(od_diag_t *diag);

#ifdef __cplusplus
}
#endif
#endif /* CAN_PORT_H */
