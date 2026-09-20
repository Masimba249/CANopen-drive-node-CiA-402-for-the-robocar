/**
 * @file od_data.h
 * @brief The robocar drive node's object dictionary: value storage, the
 *        const entry table, and the hooks the application registers.
 *
 * Layout
 * ------
 *   1000h-1FFFh  communication profile (CiA 301)
 *   2000h-2FFFh  manufacturer specific (motor config, protection, diagnostics,
 *                firmware update)
 *   6000h-67FFh  CiA 402 drive profile, axis 1 (left wheel)
 *   6800h-6FFFh  CiA 402 drive profile, axis 2 (right wheel)
 *
 * The 0x800 offset between axes is the multi-axis convention from CiA 402,
 * so 6040h/6840h are the two controlwords, 60FFh/68FFh the two targets.
 */
#ifndef OD_DATA_H
#define OD_DATA_H

#include "co_types.h"
#include "co_od.h"
#include "co_pdo.h"
#include "cia402.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOCAR_NUM_AXES 2
#define AXIS_OFFSET      0x0800u   /**< CiA 402 multi-axis index offset */

/** Per-motor configuration, object 2000h (axis 1) / 2001h (axis 2). */
typedef struct {
    uint32_t pwm_freq_hz;       /**< sub 1  */
    uint32_t encoder_cpr;       /**< sub 2 - quadrature counts per output-shaft turn */
    uint32_t gear_ratio_x1000;  /**< sub 3 - motor turns per output turn x1000 */
    uint8_t  invert;            /**< sub 4 - swap direction                   */
    uint8_t  use_encoder;       /**< sub 5 - 0 = estimate speed from PWM duty */
    uint16_t kp_x1000;          /**< sub 6 - velocity loop gains              */
    uint16_t ki_x1000;          /**< sub 7 */
    uint16_t no_load_speed;     /**< sub 8 - velocity units at 100 % duty     */
} od_motor_cfg_t;

/** Protection limits, object 2002h. */
typedef struct {
    uint16_t overcurrent_ma;    /**< sub 1 - trip instantly above this     */
    uint16_t stall_current_ma;  /**< sub 2 - current that counts as stalled*/
    uint16_t stall_time_ms;     /**< sub 3 - for this long, with no motion */
    uint16_t undervoltage_mv;   /**< sub 4 */
    uint16_t overvoltage_mv;    /**< sub 5 */
    uint16_t cmd_timeout_ms;    /**< sub 6 - RPDO watchdog, 0 = disabled   */
} od_protect_t;

/** Read-only diagnostics, object 2003h. Invaluable for bus debugging. */
typedef struct {
    uint8_t  can_state;         /**< sub 1 - 0 running 1 err-passive 2 bus-off */
    uint8_t  tx_err_counter;    /**< sub 2 */
    uint8_t  rx_err_counter;    /**< sub 3 */
    uint16_t bus_off_count;     /**< sub 4 */
    uint16_t rx_missed;         /**< sub 5 - driver RX queue overruns */
    uint32_t uptime_s;          /**< sub 6 */
    uint32_t free_heap;         /**< sub 7 */
} od_diag_t;

/** Live analogue measurements, object 2004h (TPDO mappable). */
typedef struct {
    int16_t  current_ma[ROBOCAR_NUM_AXES];  /**< sub 1, sub 2 */
    uint16_t supply_mv;                     /**< sub 3        */
} od_analog_t;

/**
 * Network configuration, object 2005h. Changes take effect after the next
 * reset, and only once 1010h has stored them - this node has no LSS, so this
 * is how a node-id or bit rate is changed over the bus.
 */
typedef struct {
    uint8_t  node_id;        /**< sub 1, 1..127      */
    uint16_t bitrate_kbps;   /**< sub 2, 125/250/500/1000 */
} od_netcfg_t;

/** Firmware-update control block, object 2100h. */
typedef struct {
    uint32_t image_size;        /**< sub 1 - bytes the master will send   */
    uint32_t image_crc32;       /**< sub 2 - CRC-32 of the whole image    */
    uint8_t  command;           /**< sub 3 - see fota.h                   */
    uint8_t  status;            /**< sub 4 - see fota.h                   */
    uint32_t bytes_received;    /**< sub 5                                 */
    char     running_slot[12];  /**< sub 6 - "ota_0" / "factory" ...      */
    char     running_version[20];/**< sub 7                                */
    uint16_t confirm_timeout_s; /**< sub 8 - rollback if not confirmed    */
} od_fota_t;

/** PDO communication parameter record (1400h/1800h family). */
typedef struct {
    uint32_t cob_id;
    uint8_t  trans_type;
    uint16_t inhibit_time;   /**< TPDO only, multiples of 100 us */
    uint16_t event_timer;    /**< ms */
} od_pdo_comm_t;

/** PDO mapping parameter record (1600h/1A00h family). */
typedef struct {
    uint8_t  count;
    uint32_t map[CO_PDO_MAX_MAP];
} od_pdo_map_t;

/** Everything the object dictionary points at. */
typedef struct {
    /* --- communication profile --- */
    uint32_t device_type;          /* 1000h */
    uint8_t  error_register;       /* 1001h */
    uint8_t  error_count;          /* 1003h:00 */
    uint32_t error_field[8];       /* 1003h:01..08 */
    uint32_t cob_id_sync;          /* 1005h */
    uint32_t comm_cycle_period;    /* 1006h */
    uint32_t store_param;          /* 1010h:01 */
    uint32_t restore_param;        /* 1011h:01 */
    uint32_t cob_id_emcy;          /* 1014h */
    uint16_t inhibit_time_emcy;    /* 1015h */
    uint32_t consumer_hb[CO_HB_CONSUMERS]; /* 1016h */
    uint16_t producer_hb;          /* 1017h */
    uint32_t vendor_id;            /* 1018h:01 */
    uint32_t product_code;         /* 1018h:02 */
    uint32_t revision_number;      /* 1018h:03 */
    uint32_t serial_number;        /* 1018h:04 */
    uint8_t  error_behaviour;      /* 1029h:01 */
    uint32_t sdo_server_rx;        /* 1200h:01 */
    uint32_t sdo_server_tx;        /* 1200h:02 */
    od_pdo_comm_t rpdo_comm[CO_NUM_RPDO];
    od_pdo_map_t  rpdo_map[CO_NUM_RPDO];
    od_pdo_comm_t tpdo_comm[CO_NUM_TPDO];
    od_pdo_map_t  tpdo_map[CO_NUM_TPDO];
    char     sw_version[20];       /* 100Ah */

    /* --- manufacturer specific --- */
    od_motor_cfg_t motor[ROBOCAR_NUM_AXES];
    od_protect_t   protect;
    od_diag_t      diag;
    od_analog_t    analog;
    od_netcfg_t    netcfg;
    od_fota_t      fota;

    /* --- CiA 402 profile, one block per axis --- */
    cia402_vars_t  axis[ROBOCAR_NUM_AXES];
} robocar_vars_t;

extern robocar_vars_t g_od;          /**< the values */
extern const co_od_t  robocar_od;    /**< the table  */

/** Load every default. Call before co_node_init(). */
void od_load_defaults(void);

/** Substitute $NODEID into the pre-defined connection set COB-IDs. */
void od_apply_node_id(uint8_t node_id);

/** 100Ah software version string. */
void od_set_sw_version(const char *s);

/* --- handlers the application registers -------------------------------- */

/** Streaming write of the firmware image (object 2101h). */
typedef uint32_t (*od_fota_data_fn)(uint32_t offset, const uint8_t *data,
                                    uint32_t len, bool last);
/** Firmware-update command (object 2100h:03). Returns an SDO abort code. */
typedef uint32_t (*od_fota_cmd_fn)(uint8_t cmd);

void od_register_fota(od_fota_data_fn data_fn, od_fota_cmd_fn cmd_fn);

/** Persist / reload parameters (objects 1010h and 1011h). */
typedef uint32_t (*od_store_fn)(void);
void od_register_store(od_store_fn save_fn, od_store_fn restore_fn);

/**
 * Verify the table really is sorted by (index, subindex). The binary search
 * in co_od_find() silently misbehaves otherwise, so the host test suite and
 * the firmware start-up both assert on this.
 */
bool od_check_sorted(void);

#ifdef __cplusplus
}
#endif
#endif /* OD_DATA_H */
