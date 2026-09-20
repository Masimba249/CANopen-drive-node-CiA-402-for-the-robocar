/**
 * @file co_types.h
 * @brief Core types, COB-ID layout, data types and abort codes for the
 *        portable CANopen stack (CiA 301).
 *
 * This header — and everything else in the `canopen` component — is plain
 * C99 with no ESP-IDF dependency, so the whole protocol stack can be compiled
 * and unit-tested on the host (see host_tests/).
 */
#ifndef CO_TYPES_H
#define CO_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* CAN frame                                                                 */
/* ------------------------------------------------------------------------ */

/** A classic CAN 2.0A frame. CANopen only uses 11-bit identifiers. */
typedef struct {
    uint32_t id;        /**< 11-bit COB-ID */
    uint8_t  dlc;       /**< 0..8 */
    uint8_t  rtr;       /**< remote transmission request (we never send them) */
    uint8_t  data[8];
} co_msg_t;

/* ------------------------------------------------------------------------ */
/* Pre-defined connection set (CiA 301 §7.3.1)                               */
/* ------------------------------------------------------------------------ */

#define CO_COB_NMT          0x000u  /**< NMT control, broadcast            */
#define CO_COB_SYNC         0x080u  /**< SYNC, broadcast                   */
#define CO_COB_EMCY_BASE    0x080u  /**< + node-id                         */
#define CO_COB_TPDO1_BASE   0x180u
#define CO_COB_RPDO1_BASE   0x200u
#define CO_COB_TPDO2_BASE   0x280u
#define CO_COB_RPDO2_BASE   0x300u
#define CO_COB_TPDO3_BASE   0x380u
#define CO_COB_RPDO3_BASE   0x400u
#define CO_COB_TPDO4_BASE   0x480u
#define CO_COB_RPDO4_BASE   0x500u
#define CO_COB_SDO_TX_BASE  0x580u  /**< server -> client                  */
#define CO_COB_SDO_RX_BASE  0x600u  /**< client -> server                  */
#define CO_COB_HEARTBEAT_BASE 0x700u

/** Bit 31 of a PDO COB-ID entry: 1 = PDO does not exist / invalid. */
#define CO_COBID_INVALID    0x80000000u
#define CO_COBID_MASK       0x7FFu

/* ------------------------------------------------------------------------ */
/* NMT states (as transmitted in the heartbeat)                              */
/* ------------------------------------------------------------------------ */

typedef enum {
    CO_NMT_BOOTUP          = 0,
    CO_NMT_STOPPED         = 4,
    CO_NMT_OPERATIONAL     = 5,
    CO_NMT_PRE_OPERATIONAL = 127,
} co_nmt_state_t;

/** NMT commands (byte 0 of a frame on COB-ID 0x000). */
typedef enum {
    CO_NMT_CMD_START           = 1,
    CO_NMT_CMD_STOP            = 2,
    CO_NMT_CMD_ENTER_PREOP     = 128,
    CO_NMT_CMD_RESET_NODE      = 129,
    CO_NMT_CMD_RESET_COMM      = 130,
} co_nmt_cmd_t;

/** How many remote heartbeat producers this node can monitor (1016h). */
#ifndef CO_HB_CONSUMERS
#define CO_HB_CONSUMERS 2
#endif

/* ------------------------------------------------------------------------ */
/* Object dictionary data types (CiA 301 §7.4.7)                             */
/* ------------------------------------------------------------------------ */

#define CO_T_BOOLEAN        0x0001
#define CO_T_INTEGER8       0x0002
#define CO_T_INTEGER16      0x0003
#define CO_T_INTEGER32      0x0004
#define CO_T_UNSIGNED8      0x0005
#define CO_T_UNSIGNED16     0x0006
#define CO_T_UNSIGNED32     0x0007
#define CO_T_REAL32         0x0008
#define CO_T_VISIBLE_STRING 0x0009
#define CO_T_OCTET_STRING   0x000A
#define CO_T_DOMAIN         0x000F

/* ------------------------------------------------------------------------ */
/* Object dictionary access flags                                            */
/* ------------------------------------------------------------------------ */

#define CO_ACC_R        0x01u   /**< readable via SDO            */
#define CO_ACC_W        0x02u   /**< writable via SDO            */
#define CO_ACC_CONST    0x04u   /**< read-only and never changes */
#define CO_ACC_TPDO     0x08u   /**< mappable into a TPDO        */
#define CO_ACC_RPDO     0x10u   /**< mappable into an RPDO       */
#define CO_ACC_STREAM   0x20u   /**< writes/reads arrive in chunks (DOMAIN) */

#define CO_ACC_RO       (CO_ACC_R)
#define CO_ACC_WO       (CO_ACC_W)
#define CO_ACC_RW       (CO_ACC_R | CO_ACC_W)
#define CO_ACC_RO_T     (CO_ACC_R | CO_ACC_TPDO)          /**< ro + TPDO mappable */
#define CO_ACC_RW_R     (CO_ACC_R | CO_ACC_W | CO_ACC_RPDO) /**< rw + RPDO mappable */

/* ------------------------------------------------------------------------ */
/* SDO abort codes (CiA 301 table 22)                                        */
/* ------------------------------------------------------------------------ */

#define CO_SDO_OK                       0x00000000u
#define CO_ABORT_TOGGLE_BIT             0x05030000u
#define CO_ABORT_TIMEOUT                0x05040000u
#define CO_ABORT_CMD_INVALID            0x05040001u
#define CO_ABORT_BLKSIZE_INVALID        0x05040002u
#define CO_ABORT_SEQNO_INVALID          0x05040003u
#define CO_ABORT_CRC_ERROR              0x05040004u
#define CO_ABORT_OUT_OF_MEMORY          0x05040005u
#define CO_ABORT_UNSUPPORTED_ACCESS     0x06010000u
#define CO_ABORT_WRITEONLY              0x06010001u
#define CO_ABORT_READONLY               0x06010002u
#define CO_ABORT_NO_OBJECT              0x06020000u
#define CO_ABORT_NO_PDO_MAP             0x06040041u
#define CO_ABORT_PDO_LEN                0x06040042u
#define CO_ABORT_PARAM_INCOMPAT         0x06040043u
#define CO_ABORT_HW_ERROR               0x06060000u
#define CO_ABORT_DATA_LEN               0x06070010u
#define CO_ABORT_DATA_LEN_HIGH          0x06070012u
#define CO_ABORT_DATA_LEN_LOW           0x06070013u
#define CO_ABORT_NO_SUBINDEX            0x06090011u
#define CO_ABORT_VALUE_RANGE            0x06090030u
#define CO_ABORT_VALUE_TOO_HIGH         0x06090031u
#define CO_ABORT_VALUE_TOO_LOW          0x06090032u
#define CO_ABORT_GENERAL                0x08000000u
#define CO_ABORT_DATA_TRANSFER          0x08000020u
#define CO_ABORT_DATA_LOCAL_CTRL        0x08000021u
#define CO_ABORT_DATA_DEV_STATE         0x08000022u
#define CO_ABORT_NO_DATA                0x08000024u

/* ------------------------------------------------------------------------ */
/* Emergency error codes we use (CiA 301 table 24 + manufacturer specific)   */
/* ------------------------------------------------------------------------ */

#define CO_EMCY_NO_ERROR                0x0000u
#define CO_EMCY_GENERIC                 0x1000u
#define CO_EMCY_CURRENT_DEVICE_INT      0x2300u
#define CO_EMCY_CURRENT_OVERCURRENT     0x2310u
#define CO_EMCY_SHORT_CIRCUIT           0x2320u
#define CO_EMCY_VOLTAGE_MAINS           0x3100u
#define CO_EMCY_VOLTAGE_UNDER           0x3220u
#define CO_EMCY_VOLTAGE_OVER            0x3210u
#define CO_EMCY_TEMPERATURE             0x4000u
#define CO_EMCY_SENSOR_SPEED            0x7305u   /**< incremental encoder 1 fault */
#define CO_EMCY_CAN_OVERRUN             0x8110u
#define CO_EMCY_CAN_PASSIVE             0x8120u
#define CO_EMCY_HEARTBEAT_LOST          0x8130u   /**< life guard / heartbeat error */
#define CO_EMCY_CAN_RECOVERED_BUSOFF    0x8140u
#define CO_EMCY_PDO_LEN_ERROR           0x8210u
#define CO_EMCY_FOLLOWING_ERROR         0x8611u
#define CO_EMCY_MANU_STALL              0xFF01u   /**< manufacturer: rotor stalled */
#define CO_EMCY_MANU_FOTA_FAILED        0xFF02u   /**< manufacturer: FOTA aborted  */

/** Error register (object 1001h) bits. */
#define CO_ERRREG_GENERIC       0x01u
#define CO_ERRREG_CURRENT       0x02u
#define CO_ERRREG_VOLTAGE       0x04u
#define CO_ERRREG_TEMPERATURE   0x08u
#define CO_ERRREG_COMMUNICATION 0x10u
#define CO_ERRREG_DEV_PROFILE   0x20u
#define CO_ERRREG_MANUFACTURER  0x80u

/* ------------------------------------------------------------------------ */
/* Little-endian helpers.                                                    */
/*                                                                           */
/* CANopen is little-endian on the wire. Both targets (Xtensa/RISC-V ESP32   */
/* and x86-64 host) are little-endian, so a memcpy is correct. These helpers */
/* exist so the assumption is stated once and is easy to replace.            */
/* ------------------------------------------------------------------------ */

static inline uint16_t co_ld_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t co_ld_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void co_st_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void co_st_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/** Unsigned ms tick difference that is safe across wraparound. */
static inline uint32_t co_elapsed(uint32_t now, uint32_t since) { return now - since; }

#ifdef __cplusplus
}
#endif
#endif /* CO_TYPES_H */
