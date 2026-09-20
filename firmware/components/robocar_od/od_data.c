/**
 * @file od_data.c
 * @brief The object dictionary table and its write hooks.
 *
 * The table must stay sorted by (index, subindex) - co_od_find() binary
 * searches it. od_check_sorted() verifies that at start-up and in the tests.
 */
#include "od_data.h"
#include "co_node.h"
#include <string.h>

robocar_vars_t g_od;

static od_fota_data_fn s_fota_data;
static od_fota_cmd_fn  s_fota_cmd;
static od_store_fn     s_save;
static od_store_fn     s_restore;

void od_register_fota(od_fota_data_fn d, od_fota_cmd_fn c) { s_fota_data = d; s_fota_cmd = c; }
void od_register_store(od_store_fn s, od_store_fn r)       { s_save = s; s_restore = r; }

/* ---- write hooks -------------------------------------------------------- */

static uint32_t store_scalar(const co_od_entry_t *e, uint32_t off,
                             const uint8_t *buf, uint32_t len)
{
    if (off != 0 || len != e->size) return CO_ABORT_DATA_LEN;
    memcpy(e->data, buf, len);
    return CO_SDO_OK;
}

static void mark_dirty(void)
{
    if (co_node_active) co_node_mark_dirty(co_node_active);
}

/** Communication parameter: store it, then ask the stack to reload. */
static uint32_t wr_comm(const co_od_entry_t *e, uint32_t off,
                        const uint8_t *buf, uint32_t len, bool last)
{
    (void)last;
    uint32_t ab = store_scalar(e, off, buf, len);
    if (ab) return ab;
    mark_dirty();
    return CO_SDO_OK;
}

/** 1003h:00 - the only legal write is 0, which clears the error history. */
static uint32_t wr_error_field(const co_od_entry_t *e, uint32_t off,
                               const uint8_t *buf, uint32_t len, bool last)
{
    (void)e; (void)last;
    if (off != 0 || len != 1) return CO_ABORT_DATA_LEN;
    if (buf[0] != 0) return CO_ABORT_VALUE_RANGE;
    g_od.error_count = 0;
    memset(g_od.error_field, 0, sizeof(g_od.error_field));
    return CO_SDO_OK;
}

/**
 * A PDO COB-ID may only be changed while the PDO is marked invalid
 * (bit 31 = 1). This is the rule CANopen configuration tools rely on, and
 * getting it wrong is a classic source of "my mapping silently didn't apply".
 */
static uint32_t wr_pdo_cobid(const co_od_entry_t *e, uint32_t off,
                             const uint8_t *buf, uint32_t len, bool last)
{
    (void)last;
    if (off != 0 || len != 4) return CO_ABORT_DATA_LEN;
    uint32_t nv  = co_ld_u32(buf);
    uint32_t cur = *(uint32_t *)e->data;

    bool cur_valid = (cur & CO_COBID_INVALID) == 0;
    bool new_valid = (nv  & CO_COBID_INVALID) == 0;
    if (cur_valid && new_valid && (cur & CO_COBID_MASK) != (nv & CO_COBID_MASK)) {
        return CO_ABORT_PARAM_INCOMPAT;
    }
    if (new_valid && ((nv & CO_COBID_MASK) == 0 || (nv & CO_COBID_MASK) > 0x7FFu)) {
        return CO_ABORT_VALUE_RANGE;
    }
    *(uint32_t *)e->data = nv;
    mark_dirty();
    return CO_SDO_OK;
}

static uint32_t wr_trans_type(const co_od_entry_t *e, uint32_t off,
                              const uint8_t *buf, uint32_t len, bool last)
{
    (void)last;
    if (off != 0 || len != 1) return CO_ABORT_DATA_LEN;
    uint8_t v = buf[0];
    /* 0..240 synchronous, 252/253 RTR-only (not supported), 254/255 event. */
    if (v > 240u && v < 254u) return CO_ABORT_VALUE_RANGE;
    *(uint8_t *)e->data = v;
    mark_dirty();
    return CO_SDO_OK;
}

/** Writing the mapping entry count re-validates the whole mapping. */
static uint32_t wr_map_count(const co_od_entry_t *e, uint32_t off,
                             const uint8_t *buf, uint32_t len, bool last)
{
    (void)last;
    if (off != 0 || len != 1) return CO_ABORT_DATA_LEN;
    uint8_t v = buf[0];
    if (v > CO_PDO_MAX_MAP) return CO_ABORT_VALUE_TOO_HIGH;

    if (v != 0 && co_node_active) {
        uint32_t ab = co_pdo_validate_map(co_node_active, e->index, v);
        if (ab) return ab;
    }
    *(uint8_t *)e->data = v;
    mark_dirty();
    return CO_SDO_OK;
}

/** Mapping entries may only be edited while the mapping is disabled. */
static uint32_t wr_map_entry(const co_od_entry_t *e, uint32_t off,
                             const uint8_t *buf, uint32_t len, bool last)
{
    (void)last;
    if (off != 0 || len != 4) return CO_ABORT_DATA_LEN;
    const co_od_entry_t *cnt = co_od_find(&robocar_od, e->index, 0);
    if (cnt && cnt->data && *(uint8_t *)cnt->data != 0) {
        return CO_ABORT_DATA_DEV_STATE;   /* set sub-index 0 to 0 first */
    }
    *(uint32_t *)e->data = co_ld_u32(buf);
    return CO_SDO_OK;
}

/** 1010h:01 - write the ASCII signature "save". */
static uint32_t wr_store(const co_od_entry_t *e, uint32_t off,
                         const uint8_t *buf, uint32_t len, bool last)
{
    (void)e; (void)last;
    if (off != 0 || len != 4) return CO_ABORT_DATA_LEN;
    if (co_ld_u32(buf) != 0x65766173u) return CO_ABORT_DATA_TRANSFER;
    return s_save ? s_save() : CO_ABORT_HW_ERROR;
}

/** 1011h:01 - write the ASCII signature "load". */
static uint32_t wr_restore(const co_od_entry_t *e, uint32_t off,
                           const uint8_t *buf, uint32_t len, bool last)
{
    (void)e; (void)last;
    if (off != 0 || len != 4) return CO_ABORT_DATA_LEN;
    if (co_ld_u32(buf) != 0x64616F6Cu) return CO_ABORT_DATA_TRANSFER;
    uint32_t ab = s_restore ? s_restore() : CO_ABORT_HW_ERROR;
    if (ab == CO_SDO_OK) mark_dirty();
    return ab;
}

/** 2100h:03 - firmware update command. */
static uint32_t wr_fota_cmd(const co_od_entry_t *e, uint32_t off,
                            const uint8_t *buf, uint32_t len, bool last)
{
    (void)last;
    if (off != 0 || len != 1) return CO_ABORT_DATA_LEN;
    *(uint8_t *)e->data = buf[0];
    if (!s_fota_cmd) return CO_ABORT_HW_ERROR;
    return s_fota_cmd(buf[0]);
}

/** 2101h - the firmware image itself, streamed straight through to flash. */
static uint32_t wr_fota_image(const co_od_entry_t *e, uint32_t off,
                              const uint8_t *buf, uint32_t len, bool last)
{
    (void)e;
    if (!s_fota_data) return CO_ABORT_HW_ERROR;
    return s_fota_data(off, buf, len, last);
}

/* ---- table -------------------------------------------------------------- */

static const uint8_t k1 = 1, k2 = 2, k3 = 3, k4 = 4, k5 = 5, k6 = 6, k7 = 7, k8 = 8;
static const char s_device_name[] = "Robocar CiA402 Drive";
static const char s_hw_version[]  = "rev A";

#define ENT(i,s,acc,ty,sz,ptr)       { (i),(s),(acc),(ty),(sz),(void*)(ptr), NULL, NULL }
#define ENTW(i,s,acc,ty,sz,ptr,w)    { (i),(s),(acc),(ty),(sz),(void*)(ptr), NULL, (w)  }
#define NSUB(i,k)                    ENT((i),0,CO_ACC_R|CO_ACC_CONST,CO_T_UNSIGNED8,1,&(k))

#define U8   CO_T_UNSIGNED8
#define U16  CO_T_UNSIGNED16
#define U32  CO_T_UNSIGNED32
#define I8   CO_T_INTEGER8
#define I16  CO_T_INTEGER16
#define I32  CO_T_INTEGER32
#define VSTR CO_T_VISIBLE_STRING

/** One CiA 402 axis. B = index base (0x6000 or 0x6800), A = array slot. */
#define AXIS_BLOCK(B, A)                                                                        \
    ENT((B)+0x007, 0, CO_ACC_RW,   I16, 2, &g_od.axis[A].abort_connection_option),              \
    ENT((B)+0x040, 0, CO_ACC_RW_R, U16, 2, &g_od.axis[A].controlword),                          \
    ENT((B)+0x041, 0, CO_ACC_RO_T, U16, 2, &g_od.axis[A].statusword),                           \
    ENT((B)+0x05A, 0, CO_ACC_RW,   I16, 2, &g_od.axis[A].quick_stop_option),                    \
    ENT((B)+0x05B, 0, CO_ACC_RW,   I16, 2, &g_od.axis[A].shutdown_option),                      \
    ENT((B)+0x05C, 0, CO_ACC_RW,   I16, 2, &g_od.axis[A].disable_op_option),                    \
    ENT((B)+0x05E, 0, CO_ACC_RW,   I16, 2, &g_od.axis[A].fault_reaction_option),                \
    ENT((B)+0x060, 0, CO_ACC_RW|CO_ACC_RPDO, I8, 1, &g_od.axis[A].mode_of_operation),           \
    ENT((B)+0x061, 0, CO_ACC_RO_T, I8,  1, &g_od.axis[A].mode_display),                         \
    ENT((B)+0x063, 0, CO_ACC_RO_T, I32, 4, &g_od.axis[A].position_internal),                    \
    ENT((B)+0x064, 0, CO_ACC_RO_T, I32, 4, &g_od.axis[A].position_actual),                      \
    ENT((B)+0x06B, 0, CO_ACC_RO_T, I32, 4, &g_od.axis[A].velocity_demand),                      \
    ENT((B)+0x06C, 0, CO_ACC_RO_T, I32, 4, &g_od.axis[A].velocity_actual),                      \
    ENT((B)+0x06D, 0, CO_ACC_RW,   U16, 2, &g_od.axis[A].velocity_window),                      \
    ENT((B)+0x06E, 0, CO_ACC_RW,   U16, 2, &g_od.axis[A].velocity_window_time),                 \
    ENT((B)+0x06F, 0, CO_ACC_RW,   U16, 2, &g_od.axis[A].velocity_threshold),                   \
    ENT((B)+0x070, 0, CO_ACC_RW,   U16, 2, &g_od.axis[A].velocity_threshold_time),              \
    ENT((B)+0x077, 0, CO_ACC_RO_T, I16, 2, &g_od.axis[A].torque_actual),                        \
    ENT((B)+0x078, 0, CO_ACC_RO_T, I16, 2, &g_od.axis[A].current_actual),                       \
    ENT((B)+0x07F, 0, CO_ACC_RW,   U32, 4, &g_od.axis[A].max_profile_velocity),                 \
    ENT((B)+0x083, 0, CO_ACC_RW,   U32, 4, &g_od.axis[A].profile_acceleration),                 \
    ENT((B)+0x084, 0, CO_ACC_RW,   U32, 4, &g_od.axis[A].profile_deceleration),                 \
    ENT((B)+0x085, 0, CO_ACC_RW,   U32, 4, &g_od.axis[A].quick_stop_deceleration),              \
    ENT((B)+0x0FF, 0, CO_ACC_RW_R, I32, 4, &g_od.axis[A].target_velocity),                      \
    ENT((B)+0x502, 0, CO_ACC_R|CO_ACC_CONST, U32, 4, &g_od.axis[A].supported_drive_modes)

/** One PDO communication record. C = comm index, S = storage. */
#define TPDO_COMM(C, S)                                                                 \
    NSUB((C), k5),                                                                      \
    ENTW((C), 1, CO_ACC_RW, U32, 4, &(S).cob_id,       wr_pdo_cobid),                   \
    ENTW((C), 2, CO_ACC_RW, U8,  1, &(S).trans_type,   wr_trans_type),                  \
    ENTW((C), 3, CO_ACC_RW, U16, 2, &(S).inhibit_time, wr_comm),                        \
    ENTW((C), 5, CO_ACC_RW, U16, 2, &(S).event_timer,  wr_comm)

#define RPDO_COMM(C, S)                                                                 \
    NSUB((C), k5),                                                                      \
    ENTW((C), 1, CO_ACC_RW, U32, 4, &(S).cob_id,      wr_pdo_cobid),                    \
    ENTW((C), 2, CO_ACC_RW, U8,  1, &(S).trans_type,  wr_trans_type),                   \
    ENTW((C), 5, CO_ACC_RW, U16, 2, &(S).event_timer, wr_comm)

#define PDO_MAP(M, S)                                                                   \
    ENTW((M), 0, CO_ACC_RW, U8,  1, &(S).count,  wr_map_count),                         \
    ENTW((M), 1, CO_ACC_RW, U32, 4, &(S).map[0], wr_map_entry),                         \
    ENTW((M), 2, CO_ACC_RW, U32, 4, &(S).map[1], wr_map_entry),                         \
    ENTW((M), 3, CO_ACC_RW, U32, 4, &(S).map[2], wr_map_entry),                         \
    ENTW((M), 4, CO_ACC_RW, U32, 4, &(S).map[3], wr_map_entry),                         \
    ENTW((M), 5, CO_ACC_RW, U32, 4, &(S).map[4], wr_map_entry),                         \
    ENTW((M), 6, CO_ACC_RW, U32, 4, &(S).map[5], wr_map_entry),                         \
    ENTW((M), 7, CO_ACC_RW, U32, 4, &(S).map[6], wr_map_entry),                         \
    ENTW((M), 8, CO_ACC_RW, U32, 4, &(S).map[7], wr_map_entry)

static const co_od_entry_t s_entries[] = {
    /* ---------------- communication profile ---------------- */
    ENT (0x1000, 0, CO_ACC_R|CO_ACC_CONST, U32, 4, &g_od.device_type),
    ENT (0x1001, 0, CO_ACC_RO_T,           U8,  1, &g_od.error_register),

    ENTW(0x1003, 0, CO_ACC_RW, U8,  1, &g_od.error_count,    wr_error_field),
    ENT (0x1003, 1, CO_ACC_RO, U32, 4, &g_od.error_field[0]),
    ENT (0x1003, 2, CO_ACC_RO, U32, 4, &g_od.error_field[1]),
    ENT (0x1003, 3, CO_ACC_RO, U32, 4, &g_od.error_field[2]),
    ENT (0x1003, 4, CO_ACC_RO, U32, 4, &g_od.error_field[3]),
    ENT (0x1003, 5, CO_ACC_RO, U32, 4, &g_od.error_field[4]),
    ENT (0x1003, 6, CO_ACC_RO, U32, 4, &g_od.error_field[5]),
    ENT (0x1003, 7, CO_ACC_RO, U32, 4, &g_od.error_field[6]),
    ENT (0x1003, 8, CO_ACC_RO, U32, 4, &g_od.error_field[7]),

    ENTW(0x1005, 0, CO_ACC_RW, U32, 4, &g_od.cob_id_sync,       wr_comm),
    ENTW(0x1006, 0, CO_ACC_RW, U32, 4, &g_od.comm_cycle_period, wr_comm),

    ENT (0x1008, 0, CO_ACC_R|CO_ACC_CONST, VSTR, sizeof(s_device_name) - 1, s_device_name),
    ENT (0x1009, 0, CO_ACC_R|CO_ACC_CONST, VSTR, sizeof(s_hw_version) - 1,  s_hw_version),
    ENT (0x100A, 0, CO_ACC_RO,             VSTR, sizeof(g_od.sw_version) - 1, g_od.sw_version),

    NSUB(0x1010, k1),
    ENTW(0x1010, 1, CO_ACC_RW, U32, 4, &g_od.store_param, wr_store),
    NSUB(0x1011, k1),
    ENTW(0x1011, 1, CO_ACC_RW, U32, 4, &g_od.restore_param, wr_restore),

    ENTW(0x1014, 0, CO_ACC_RW, U32, 4, &g_od.cob_id_emcy,       wr_comm),
    ENTW(0x1015, 0, CO_ACC_RW, U16, 2, &g_od.inhibit_time_emcy, wr_comm),

    NSUB(0x1016, k2),
    ENTW(0x1016, 1, CO_ACC_RW, U32, 4, &g_od.consumer_hb[0], wr_comm),
    ENTW(0x1016, 2, CO_ACC_RW, U32, 4, &g_od.consumer_hb[1], wr_comm),
    ENTW(0x1017, 0, CO_ACC_RW, U16, 2, &g_od.producer_hb,    wr_comm),

    NSUB(0x1018, k4),
    ENT (0x1018, 1, CO_ACC_R|CO_ACC_CONST, U32, 4, &g_od.vendor_id),
    ENT (0x1018, 2, CO_ACC_R|CO_ACC_CONST, U32, 4, &g_od.product_code),
    ENT (0x1018, 3, CO_ACC_R|CO_ACC_CONST, U32, 4, &g_od.revision_number),
    ENT (0x1018, 4, CO_ACC_RO,             U32, 4, &g_od.serial_number),

    NSUB(0x1029, k1),
    ENT (0x1029, 1, CO_ACC_RW, U8, 1, &g_od.error_behaviour),

    NSUB(0x1200, k2),
    ENT (0x1200, 1, CO_ACC_R|CO_ACC_CONST, U32, 4, &g_od.sdo_server_rx),
    ENT (0x1200, 2, CO_ACC_R|CO_ACC_CONST, U32, 4, &g_od.sdo_server_tx),

    RPDO_COMM(0x1400, g_od.rpdo_comm[0]),
    RPDO_COMM(0x1401, g_od.rpdo_comm[1]),
    PDO_MAP  (0x1600, g_od.rpdo_map[0]),
    PDO_MAP  (0x1601, g_od.rpdo_map[1]),

    TPDO_COMM(0x1800, g_od.tpdo_comm[0]),
    TPDO_COMM(0x1801, g_od.tpdo_comm[1]),
    TPDO_COMM(0x1802, g_od.tpdo_comm[2]),
    PDO_MAP  (0x1A00, g_od.tpdo_map[0]),
    PDO_MAP  (0x1A01, g_od.tpdo_map[1]),
    PDO_MAP  (0x1A02, g_od.tpdo_map[2]),

    /* ---------------- manufacturer specific ---------------- */
    NSUB(0x2000, k8),
    ENT (0x2000, 1, CO_ACC_RW, U32, 4, &g_od.motor[0].pwm_freq_hz),
    ENT (0x2000, 2, CO_ACC_RW, U32, 4, &g_od.motor[0].encoder_cpr),
    ENT (0x2000, 3, CO_ACC_RW, U32, 4, &g_od.motor[0].gear_ratio_x1000),
    ENT (0x2000, 4, CO_ACC_RW, U8,  1, &g_od.motor[0].invert),
    ENT (0x2000, 5, CO_ACC_RW, U8,  1, &g_od.motor[0].use_encoder),
    ENT (0x2000, 6, CO_ACC_RW, U16, 2, &g_od.motor[0].kp_x1000),
    ENT (0x2000, 7, CO_ACC_RW, U16, 2, &g_od.motor[0].ki_x1000),
    ENT (0x2000, 8, CO_ACC_RW, U16, 2, &g_od.motor[0].no_load_speed),

    NSUB(0x2001, k8),
    ENT (0x2001, 1, CO_ACC_RW, U32, 4, &g_od.motor[1].pwm_freq_hz),
    ENT (0x2001, 2, CO_ACC_RW, U32, 4, &g_od.motor[1].encoder_cpr),
    ENT (0x2001, 3, CO_ACC_RW, U32, 4, &g_od.motor[1].gear_ratio_x1000),
    ENT (0x2001, 4, CO_ACC_RW, U8,  1, &g_od.motor[1].invert),
    ENT (0x2001, 5, CO_ACC_RW, U8,  1, &g_od.motor[1].use_encoder),
    ENT (0x2001, 6, CO_ACC_RW, U16, 2, &g_od.motor[1].kp_x1000),
    ENT (0x2001, 7, CO_ACC_RW, U16, 2, &g_od.motor[1].ki_x1000),
    ENT (0x2001, 8, CO_ACC_RW, U16, 2, &g_od.motor[1].no_load_speed),

    NSUB(0x2002, k6),
    ENT (0x2002, 1, CO_ACC_RW, U16, 2, &g_od.protect.overcurrent_ma),
    ENT (0x2002, 2, CO_ACC_RW, U16, 2, &g_od.protect.stall_current_ma),
    ENT (0x2002, 3, CO_ACC_RW, U16, 2, &g_od.protect.stall_time_ms),
    ENT (0x2002, 4, CO_ACC_RW, U16, 2, &g_od.protect.undervoltage_mv),
    ENT (0x2002, 5, CO_ACC_RW, U16, 2, &g_od.protect.overvoltage_mv),
    ENT (0x2002, 6, CO_ACC_RW, U16, 2, &g_od.protect.cmd_timeout_ms),

    NSUB(0x2003, k7),
    ENT (0x2003, 1, CO_ACC_RO_T, U8,  1, &g_od.diag.can_state),
    ENT (0x2003, 2, CO_ACC_RO,   U8,  1, &g_od.diag.tx_err_counter),
    ENT (0x2003, 3, CO_ACC_RO,   U8,  1, &g_od.diag.rx_err_counter),
    ENT (0x2003, 4, CO_ACC_RO,   U16, 2, &g_od.diag.bus_off_count),
    ENT (0x2003, 5, CO_ACC_RO,   U16, 2, &g_od.diag.rx_missed),
    ENT (0x2003, 6, CO_ACC_RO,   U32, 4, &g_od.diag.uptime_s),
    ENT (0x2003, 7, CO_ACC_RO,   U32, 4, &g_od.diag.free_heap),

    NSUB(0x2004, k3),
    ENT (0x2004, 1, CO_ACC_RO_T, I16, 2, &g_od.analog.current_ma[0]),
    ENT (0x2004, 2, CO_ACC_RO_T, I16, 2, &g_od.analog.current_ma[1]),
    ENT (0x2004, 3, CO_ACC_RO_T, U16, 2, &g_od.analog.supply_mv),

    NSUB(0x2005, k2),
    ENT (0x2005, 1, CO_ACC_RW, U8,  1, &g_od.netcfg.node_id),
    ENT (0x2005, 2, CO_ACC_RW, U16, 2, &g_od.netcfg.bitrate_kbps),

    NSUB(0x2100, k8),
    ENT (0x2100, 1, CO_ACC_RW, U32, 4, &g_od.fota.image_size),
    ENT (0x2100, 2, CO_ACC_RW, U32, 4, &g_od.fota.image_crc32),
    ENTW(0x2100, 3, CO_ACC_RW, U8,  1, &g_od.fota.command, wr_fota_cmd),
    ENT (0x2100, 4, CO_ACC_RO_T, U8, 1, &g_od.fota.status),
    ENT (0x2100, 5, CO_ACC_RO, U32, 4, &g_od.fota.bytes_received),
    ENT (0x2100, 6, CO_ACC_RO, VSTR, sizeof(g_od.fota.running_slot) - 1, g_od.fota.running_slot),
    ENT (0x2100, 7, CO_ACC_RO, VSTR, sizeof(g_od.fota.running_version) - 1, g_od.fota.running_version),
    ENT (0x2100, 8, CO_ACC_RW, U16, 2, &g_od.fota.confirm_timeout_s),

    { 0x2101, 0, CO_ACC_WO | CO_ACC_STREAM, CO_T_DOMAIN, 0, NULL, NULL, wr_fota_image },

    /* ---------------- CiA 402, axis 1 then axis 2 ---------------- */
    AXIS_BLOCK(0x6000, 0),
    AXIS_BLOCK(0x6800, 1),
};

const co_od_t robocar_od = { s_entries, (uint16_t)(sizeof(s_entries) / sizeof(s_entries[0])) };

bool od_check_sorted(void)
{
    for (uint16_t i = 1; i < robocar_od.count; i++) {
        uint32_t a = ((uint32_t)s_entries[i - 1].index << 8) | s_entries[i - 1].sub;
        uint32_t b = ((uint32_t)s_entries[i].index << 8) | s_entries[i].sub;
        if (b <= a) return false;
    }
    return true;
}

/* ---- defaults ----------------------------------------------------------- */

void od_set_sw_version(const char *s)
{
    strncpy(g_od.sw_version, s, sizeof(g_od.sw_version) - 1);
    g_od.sw_version[sizeof(g_od.sw_version) - 1] = '\0';
}

void od_load_defaults(void)
{
    memset(&g_od, 0, sizeof(g_od));

    /* 1000h: bits 0-15 = device profile 402, bits 16-31 = type 0x0002
     * ("servo drive"); 0x00020192. */
    g_od.device_type     = 0x00020192u;
    g_od.cob_id_sync     = CO_COB_SYNC;
    g_od.inhibit_time_emcy = 1000;       /* 100 ms, in 100 us units */
    g_od.producer_hb     = 500;          /* ms */
    g_od.vendor_id       = 0x0000FEEDu;  /* no CiA vendor ID; see docs */
    g_od.product_code    = 0x52430001u;  /* "RC" 0001 */
    g_od.revision_number = 0x00010000u;  /* 1.0 */
    g_od.serial_number   = 0;            /* filled from the eFuse MAC */
    g_od.error_behaviour = 0;            /* communication error -> pre-operational */
    od_set_sw_version("0.0.0");
    strncpy(g_od.fota.running_slot, "unknown", sizeof(g_od.fota.running_slot) - 1);
    strncpy(g_od.fota.running_version, "0.0.0", sizeof(g_od.fota.running_version) - 1);
    g_od.fota.confirm_timeout_s = 60;

    /* --- PDO defaults ---------------------------------------------------
     * TPDO1: axis 1 statusword + velocity actual, every 10 ms
     * TPDO2: axis 2 statusword + velocity actual, every 10 ms
     * TPDO3: error register + both currents + supply voltage, every 100 ms
     * RPDO1: axis 1 controlword + target velocity
     * RPDO2: axis 2 controlword + target velocity
     * COB-IDs get the node-id added by od_apply_node_id(). */
    for (unsigned i = 0; i < CO_NUM_TPDO; i++) {
        g_od.tpdo_comm[i].trans_type   = CO_PDO_TT_EVENT_PROFILE;  /* 255 */
        g_od.tpdo_comm[i].inhibit_time = 0;
        g_od.tpdo_comm[i].event_timer  = 10;
    }
    g_od.tpdo_comm[2].event_timer = 100;

    g_od.tpdo_map[0].count  = 2;
    g_od.tpdo_map[0].map[0] = 0x60410010u;  /* statusword,      16 bit */
    g_od.tpdo_map[0].map[1] = 0x606C0020u;  /* velocity actual, 32 bit */

    g_od.tpdo_map[1].count  = 2;
    g_od.tpdo_map[1].map[0] = 0x68410010u;
    g_od.tpdo_map[1].map[1] = 0x686C0020u;

    g_od.tpdo_map[2].count  = 4;
    g_od.tpdo_map[2].map[0] = 0x10010008u;  /* error register,  8 bit  */
    g_od.tpdo_map[2].map[1] = 0x20040110u;  /* current axis 1, 16 bit  */
    g_od.tpdo_map[2].map[2] = 0x20040210u;  /* current axis 2, 16 bit  */
    g_od.tpdo_map[2].map[3] = 0x20040310u;  /* supply mV,      16 bit  */

    for (unsigned i = 0; i < CO_NUM_RPDO; i++) {
        g_od.rpdo_comm[i].trans_type  = CO_PDO_TT_EVENT_PROFILE;
        g_od.rpdo_comm[i].event_timer = 100;   /* command watchdog, ms */
    }
    g_od.rpdo_map[0].count  = 2;
    g_od.rpdo_map[0].map[0] = 0x60400010u;  /* controlword,     16 bit */
    g_od.rpdo_map[0].map[1] = 0x60FF0020u;  /* target velocity, 32 bit */

    g_od.rpdo_map[1].count  = 2;
    g_od.rpdo_map[1].map[0] = 0x68400010u;
    g_od.rpdo_map[1].map[1] = 0x68FF0020u;

    /* --- manufacturer defaults ------------------------------------------ */
    for (unsigned a = 0; a < ROBOCAR_NUM_AXES; a++) {
        g_od.motor[a].pwm_freq_hz      = 20000;   /* above audible range */
        g_od.motor[a].encoder_cpr      = 1440;    /* 11 PPR x 4 x 32.8 gearbox, rounded */
        g_od.motor[a].gear_ratio_x1000 = 30000;   /* 30:1 */
        g_od.motor[a].invert           = (a == 1) ? 1 : 0;  /* right motor is mirrored */
        g_od.motor[a].use_encoder      = 1;
        g_od.motor[a].kp_x1000         = 800;
        g_od.motor[a].ki_x1000         = 2500;
        g_od.motor[a].no_load_speed    = 2000;    /* 200.0 rpm at the wheel */
    }
    g_od.protect.overcurrent_ma   = 2500;
    g_od.protect.stall_current_ma = 1500;
    g_od.protect.stall_time_ms    = 500;
    g_od.protect.undervoltage_mv  = 9000;
    g_od.protect.overvoltage_mv   = 14000;
    g_od.protect.cmd_timeout_ms   = 200;
    g_od.netcfg.bitrate_kbps      = 500;

    /* --- CiA 402 defaults ----------------------------------------------- */
    for (unsigned a = 0; a < ROBOCAR_NUM_AXES; a++) {
        cia402_vars_t *v = &g_od.axis[a];
        v->mode_of_operation        = MODE_PROFILE_VELOCITY;
        v->mode_display             = MODE_NO_MODE;
        v->supported_drive_modes    = SUPPORTED_DRIVE_MODES;
        v->max_profile_velocity     = 2000;   /* 200.0 rpm            */
        v->profile_acceleration     = 4000;   /* units per second     */
        v->profile_deceleration     = 4000;
        v->quick_stop_deceleration  = 20000;  /* ~0.1 s from full speed */
        v->quick_stop_option        = 2;      /* quick stop ramp, then switch on disabled */
        v->shutdown_option          = 0;
        v->disable_op_option        = 1;
        v->fault_reaction_option    = 2;      /* quick stop ramp       */
        v->abort_connection_option  = 2;      /* bus loss -> quick stop*/
        v->velocity_window          = 50;     /* 5.0 rpm               */
        v->velocity_window_time     = 20;     /* ms                    */
        v->velocity_threshold       = 20;     /* 2.0 rpm ~ standstill  */
        v->velocity_threshold_time  = 50;
    }
}

void od_apply_node_id(uint8_t node_id)
{
    g_od.netcfg.node_id = node_id;
    g_od.cob_id_emcy   = CO_COB_EMCY_BASE + node_id;
    g_od.sdo_server_rx = CO_COB_SDO_RX_BASE + node_id;
    g_od.sdo_server_tx = CO_COB_SDO_TX_BASE + node_id;

    static const uint32_t tpdo_base[4] = { CO_COB_TPDO1_BASE, CO_COB_TPDO2_BASE,
                                           CO_COB_TPDO3_BASE, CO_COB_TPDO4_BASE };
    static const uint32_t rpdo_base[4] = { CO_COB_RPDO1_BASE, CO_COB_RPDO2_BASE,
                                           CO_COB_RPDO3_BASE, CO_COB_RPDO4_BASE };

    for (unsigned i = 0; i < CO_NUM_TPDO && i < 4; i++) {
        g_od.tpdo_comm[i].cob_id = tpdo_base[i] + node_id;
    }
    for (unsigned i = 0; i < CO_NUM_RPDO && i < 4; i++) {
        g_od.rpdo_comm[i].cob_id = rpdo_base[i] + node_id;
    }
}
