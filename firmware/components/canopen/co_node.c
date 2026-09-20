/**
 * @file co_node.c
 * @brief NMT slave, heartbeat producer/consumer, SYNC consumer, and the
 *        receive/tick plumbing that ties the stack together.
 */
#include "co_node.h"
#include "co_crc.h"

co_node_t *co_node_active = NULL;

bool co_node_send(co_node_t *n, const co_msg_t *m)
{
    if (!n->send) return false;
    if (!n->send(n->ctx, m)) { n->tx_dropped++; return false; }
    return true;
}

static void send_heartbeat(co_node_t *n, uint8_t state)
{
    co_msg_t m;
    m.id = CO_COB_HEARTBEAT_BASE + n->node_id;
    m.dlc = 1;
    m.rtr = 0;
    m.data[0] = state;
    memset(&m.data[1], 0, 7);
    co_node_send(n, &m);
}

/** Cache the communication parameters the hot path needs. */
static void reload_comm_params(co_node_t *n)
{
    n->hb_period_ms = co_od_get_u16(n->od, 0x1017, 0x00);
    n->sync_cob_id  = co_od_get_u32(n->od, 0x1005, 0x00) & CO_COBID_MASK;

    for (unsigned i = 0; i < CO_HB_CONSUMERS; i++) {
        uint32_t v = co_od_get_u32(n->od, 0x1016, (uint8_t)(i + 1));
        uint8_t  id = (uint8_t)(v >> 16);
        uint16_t t  = (uint16_t)v;
        if (n->hb_cons[i].node_id != id || n->hb_cons[i].timeout_ms != t) {
            n->hb_cons[i].seen = false;
            n->hb_cons[i].lost = false;
        }
        n->hb_cons[i].node_id    = (t == 0) ? 0 : id;  /* time 0 disables the slot */
        n->hb_cons[i].timeout_ms = t;
    }
    co_pdo_configure(n);
}

void co_node_set_nmt(co_node_t *n, co_nmt_state_t state)
{
    if (n->nmt_state == state) return;
    n->nmt_state = state;
    if (state == CO_NMT_OPERATIONAL) {
        /* Transmit every TPDO once on entering OPERATIONAL so the master has
         * a complete picture immediately instead of after the first event. */
        for (unsigned i = 0; i < CO_NUM_TPDO; i++) co_pdo_send_tpdo(n, i, n->now_ms);
    }
    if (n->on_nmt) n->on_nmt(n, state, n->ctx);
}

void co_node_init(co_node_t *n, const co_node_cfg_t *cfg)
{
    memset(n, 0, sizeof(*n));
    n->node_id    = cfg->node_id;
    n->od         = cfg->od;
    n->send       = cfg->send;
    n->ctx        = cfg->ctx;
    n->on_nmt     = cfg->on_nmt;
    n->on_reset   = cfg->on_reset;
    n->on_hb_lost = cfg->on_hb_lost;
    n->nmt_state  = CO_NMT_BOOTUP;

    co_sdo_init(&n->sdo);
    co_emcy_init(&n->emcy);
    co_node_active = n;

    reload_comm_params(n);

    /* Boot-up message: heartbeat COB-ID carrying state 0. */
    send_heartbeat(n, CO_NMT_BOOTUP);

    n->nmt_state = CO_NMT_PRE_OPERATIONAL;
    if (n->on_nmt) n->on_nmt(n, n->nmt_state, n->ctx);
}

/* ---- receive ------------------------------------------------------------ */

static void on_nmt_command(co_node_t *n, const co_msg_t *m)
{
    if (m->dlc < 2) return;
    uint8_t cmd = m->data[0];
    uint8_t target = m->data[1];
    if (target != 0 && target != n->node_id) return;

    switch (cmd) {
    case CO_NMT_CMD_START:       co_node_set_nmt(n, CO_NMT_OPERATIONAL);     break;
    case CO_NMT_CMD_STOP:        co_node_set_nmt(n, CO_NMT_STOPPED);         break;
    case CO_NMT_CMD_ENTER_PREOP: co_node_set_nmt(n, CO_NMT_PRE_OPERATIONAL); break;

    case CO_NMT_CMD_RESET_NODE:
        if (n->on_reset) n->on_reset(n, true, n->ctx);
        break;

    case CO_NMT_CMD_RESET_COMM:
        if (n->on_reset) n->on_reset(n, false, n->ctx);
        co_sdo_init(&n->sdo);
        reload_comm_params(n);
        send_heartbeat(n, CO_NMT_BOOTUP);
        n->nmt_state = CO_NMT_BOOTUP;
        co_node_set_nmt(n, CO_NMT_PRE_OPERATIONAL);
        break;

    default: break;
    }
}

static void on_heartbeat_rx(co_node_t *n, const co_msg_t *m)
{
    uint8_t id = (uint8_t)(m->id - CO_COB_HEARTBEAT_BASE);
    for (unsigned i = 0; i < CO_HB_CONSUMERS; i++) {
        if (n->hb_cons[i].node_id == id && n->hb_cons[i].timeout_ms) {
            n->hb_cons[i].last_ms = n->now_ms;
            n->hb_cons[i].seen = true;
            if (n->hb_cons[i].lost) {
                n->hb_cons[i].lost = false;
                co_emcy_clear(n, CO_EMCY_HEARTBEAT_LOST);
            }
        }
    }
}

void co_node_rx(co_node_t *n, const co_msg_t *m)
{
    if (m->id == CO_COB_NMT) { on_nmt_command(n, m); return; }

    /* In STOPPED only NMT and heartbeat/node guarding are served. */
    if (n->nmt_state == CO_NMT_STOPPED) {
        if (m->id >= CO_COB_HEARTBEAT_BASE && m->id <= CO_COB_HEARTBEAT_BASE + 127u) {
            on_heartbeat_rx(n, m);
        }
        return;
    }

    if (m->id == n->sync_cob_id && m->dlc <= 1) { co_pdo_sync(n); return; }

    if (m->id == (uint32_t)(CO_COB_SDO_RX_BASE + n->node_id)) { co_sdo_rx(n, m); return; }

    if (m->id >= CO_COB_HEARTBEAT_BASE && m->id <= CO_COB_HEARTBEAT_BASE + 127u) {
        on_heartbeat_rx(n, m);
        return;
    }

    co_pdo_rx(n, m);
}

/* ---- periodic ----------------------------------------------------------- */

void co_node_tick(co_node_t *n, uint32_t now_ms)
{
    n->now_ms = now_ms;

    if (n->cfg_dirty) {
        n->cfg_dirty = false;
        reload_comm_params(n);
    }

    co_sdo_tick(n, now_ms);
    co_pdo_tick(n, now_ms);
    co_emcy_tick(n, now_ms);

    /* Heartbeat producer (1017h). Period 0 disables it. */
    if (n->hb_period_ms && co_elapsed(now_ms, n->hb_last_ms) >= n->hb_period_ms) {
        n->hb_last_ms = now_ms;
        send_heartbeat(n, (uint8_t)n->nmt_state);
    }

    /* Heartbeat consumer (1016h): losing the master is a communication error. */
    for (unsigned i = 0; i < CO_HB_CONSUMERS; i++) {
        co_hb_consumer_t *c = &n->hb_cons[i];
        if (!c->node_id || !c->timeout_ms || !c->seen || c->lost) continue;
        if (co_elapsed(now_ms, c->last_ms) > c->timeout_ms) {
            c->lost = true;
            uint8_t info[5] = { c->node_id, 0, 0, 0, 0 };
            co_emcy_raise(n, CO_EMCY_HEARTBEAT_LOST, CO_ERRREG_COMMUNICATION, info);
            if (n->on_hb_lost) n->on_hb_lost(n, c->node_id, n->ctx);
        }
    }
}
