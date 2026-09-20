/**
 * @file sim_node.c
 * @brief The real firmware stack, built as a shared library so a real
 *        CANopen master can talk to it with no ESP32 on the desk.
 *
 * Everything below the CAN driver is the production code: the same object
 * dictionary, the same SDO server, the same PDO engine and the same CiA 402
 * state machine that run on the target. Only three things are simulated:
 *
 *   - the CAN driver becomes a pair of ring buffers that Python pumps into a
 *     python-can "virtual" bus,
 *   - the motors become a first-order lag,
 *   - the firmware update writes to RAM instead of flash, but runs exactly
 *     the same command sequence and CRC check.
 *
 * See master/tools/sim_node.py.
 */
#include "co_node.h"
#include "od_data.h"
#include "cia402.h"
#include "co_crc.h"
#include <string.h>
#include <stdlib.h>

#define TXQ 256
#define SIM_IMAGE_CAP (512u * 1024u)

static co_node_t     s_node;
static cia402_axis_t s_axis[ROBOCAR_NUM_AXES];
static int32_t       s_demand[ROBOCAR_NUM_AXES];
static bool          s_powered[ROBOCAR_NUM_AXES];
static uint32_t      s_now;

static co_msg_t s_txq[TXQ];
static int      s_tx_head, s_tx_tail;

/* --- simulated firmware image sink -------------------------------------- */
static uint8_t *s_image;
static uint32_t s_image_len;
static uint32_t s_image_crc;
static bool     s_image_open;

/* --- CAN "driver" -------------------------------------------------------- */

static bool sim_send(void *ctx, const co_msg_t *m)
{
    (void)ctx;
    int next = (s_tx_head + 1) % TXQ;
    if (next == s_tx_tail) return false;     /* queue full, same as hardware */
    s_txq[s_tx_head] = *m;
    s_tx_head = next;
    return true;
}

/* --- CiA 402 hardware hooks ---------------------------------------------- */

static void hw_power(uint8_t a, bool on, void *ctx) { (void)ctx; s_powered[a] = on; }
static void hw_demand(uint8_t a, int32_t d, void *ctx) { (void)ctx; s_demand[a] = d; }

static const cia402_hw_t s_hw = { .power_stage = hw_power, .set_demand = hw_demand };

/* --- simulated firmware update ------------------------------------------- */

static uint32_t sim_fota_data(uint32_t off, const uint8_t *d, uint32_t len, bool last)
{
    if (!s_image_open) return CO_ABORT_DATA_DEV_STATE;
    if (off != s_image_len) return CO_ABORT_DATA_TRANSFER;
    if (off + len > SIM_IMAGE_CAP) return CO_ABORT_OUT_OF_MEMORY;
    memcpy(&s_image[off], d, len);
    s_image_len += len;
    s_image_crc = co_crc32(s_image_crc, d, len);
    g_od.fota.bytes_received = s_image_len;
    (void)last;
    return CO_SDO_OK;
}

static uint32_t sim_fota_cmd(uint8_t cmd)
{
    switch (cmd) {
    case 1: /* BEGIN */
        if (s_axis[0].state == CIA402_OPERATION_ENABLED ||
            s_axis[1].state == CIA402_OPERATION_ENABLED) {
            g_od.fota.status = 0x81;
            return CO_ABORT_DATA_DEV_STATE;
        }
        if (g_od.fota.image_size == 0 || g_od.fota.image_size > SIM_IMAGE_CAP) {
            g_od.fota.status = 0x82;
            return CO_ABORT_OUT_OF_MEMORY;
        }
        s_image_len = 0;
        s_image_crc = CO_CRC32_INIT;
        s_image_open = true;
        g_od.fota.bytes_received = 0;
        g_od.fota.status = 0x01;
        return CO_SDO_OK;

    case 2: /* ACTIVATE */
        if (!s_image_open) { g_od.fota.status = 0x81; return CO_ABORT_DATA_DEV_STATE; }
        if (s_image_len != g_od.fota.image_size) {
            g_od.fota.status = 0x82;
            s_image_open = false;
            return CO_ABORT_DATA_LEN_LOW;
        }
        if (co_crc32_final(s_image_crc) != g_od.fota.image_crc32) {
            g_od.fota.status = 0x83;
            s_image_open = false;
            return CO_ABORT_CRC_ERROR;
        }
        s_image_open = false;
        g_od.fota.status = 0x02;
        return CO_SDO_OK;

    case 3: /* ABORT */
        s_image_open = false;
        s_image_len = 0;
        g_od.fota.status = 0x00;
        return CO_SDO_OK;

    case 4: /* CONFIRM */
        g_od.fota.status = 0x00;
        return CO_SDO_OK;

    default:
        return CO_ABORT_VALUE_RANGE;
    }
}

static uint32_t sim_store(void)   { return CO_SDO_OK; }
static uint32_t sim_restore(void) { od_load_defaults(); return CO_SDO_OK; }

/* --- NMT callback --------------------------------------------------------- */

static void on_nmt(co_node_t *n, co_nmt_state_t state, void *ctx)
{
    (void)n; (void)ctx;
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        cia402_set_remote(&s_axis[i], state == CO_NMT_OPERATIONAL);
    }
}

/* --- exported API --------------------------------------------------------- */

void sim_init(uint8_t node_id)
{
    s_tx_head = s_tx_tail = 0;
    s_now = 0;
    memset(s_demand, 0, sizeof(s_demand));
    memset(s_powered, 0, sizeof(s_powered));

    if (!s_image) s_image = malloc(SIM_IMAGE_CAP);
    s_image_len = 0;
    s_image_crc = CO_CRC32_INIT;
    s_image_open = false;

    od_load_defaults();
    od_apply_node_id(node_id);
    od_set_sw_version("1.0.0-sim");
    strncpy(g_od.fota.running_slot, "ota_0", sizeof(g_od.fota.running_slot) - 1);
    strncpy(g_od.fota.running_version, "1.0.0-sim", sizeof(g_od.fota.running_version) - 1);
    g_od.serial_number = 0x5164C0DE;
    g_od.analog.supply_mv = 11800;

    od_register_fota(sim_fota_data, sim_fota_cmd);
    od_register_store(sim_store, sim_restore);

    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        cia402_init(&s_axis[i], &g_od.axis[i], (uint8_t)i, &s_hw);
        cia402_set_voltage_enabled(&s_axis[i], true);
    }

    co_node_cfg_t cfg = {
        .node_id = node_id,
        .od      = &robocar_od,
        .send    = sim_send,
        .on_nmt  = on_nmt,
    };
    co_node_init(&s_node, &cfg);
}

void sim_rx(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    co_msg_t m = { .id = id, .dlc = dlc, .rtr = 0 };
    memcpy(m.data, data, dlc > 8 ? 8 : dlc);
    co_node_rx(&s_node, &m);
}

/** Advance by @p dt_ms, running the stack and the motor model. */
void sim_tick(uint32_t dt_ms)
{
    for (uint32_t k = 0; k < dt_ms; k++) {
        s_now++;
        co_node_tick(&s_node, s_now);

        if (s_now % 5 == 0) {
            for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
                cia402_vars_t *v = &g_od.axis[i];
                /* First-order lag with a ~100 ms time constant, plus a
                 * little integration so 6063h/6064h move too. */
                int32_t target = s_powered[i] ? s_demand[i] : 0;
                v->velocity_actual += (target - v->velocity_actual) / 20;
                if (target != v->velocity_actual &&
                    (target - v->velocity_actual) < 20 &&
                    (v->velocity_actual - target) < 20) {
                    v->velocity_actual = target;
                }
                v->position_internal += v->velocity_actual / 100;
                v->position_actual = v->position_internal;
                g_od.analog.current_ma[i] = (int16_t)(abs((int)v->velocity_actual) / 4);
                v->current_actual = g_od.analog.current_ma[i];
                cia402_tick(&s_axis[i], 5);
            }
            g_od.diag.uptime_s = s_now / 1000;
        }
    }
}

/** Pop one queued transmit frame. Returns 0 when the queue is empty. */
int sim_pop_tx(uint32_t *id, uint8_t *data, uint8_t *dlc)
{
    if (s_tx_tail == s_tx_head) return 0;
    const co_msg_t *m = &s_txq[s_tx_tail];
    *id = m->id;
    *dlc = m->dlc;
    memcpy(data, m->data, 8);
    s_tx_tail = (s_tx_tail + 1) % TXQ;
    return 1;
}

/* Introspection for the demo / tests. */
uint32_t sim_image_len(void)   { return s_image_len; }
uint32_t sim_image_crc(void)   { return co_crc32_final(s_image_crc); }
uint8_t  sim_fota_status(void) { return g_od.fota.status; }
int32_t  sim_demand(uint8_t a) { return (a < ROBOCAR_NUM_AXES) ? s_demand[a] : 0; }
int      sim_powered(uint8_t a){ return (a < ROBOCAR_NUM_AXES) ? s_powered[a] : 0; }
uint16_t sim_statusword(uint8_t a) { return (a < ROBOCAR_NUM_AXES) ? g_od.axis[a].statusword : 0; }
void     sim_set_fault(uint8_t a)  { if (a < ROBOCAR_NUM_AXES) cia402_fault(&s_axis[a]); }
void     sim_raise_emcy(uint16_t code, uint8_t reg) { co_emcy_raise(&s_node, code, reg, NULL); }
