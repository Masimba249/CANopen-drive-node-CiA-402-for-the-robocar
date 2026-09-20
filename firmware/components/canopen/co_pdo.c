/**
 * @file co_pdo.c
 * @brief PDO engine (CiA 301 §7.2.2).
 *
 * The runtime structures are rebuilt from the object dictionary by
 * co_pdo_configure(), which runs at start-up, on NMT reset and whenever a
 * communication/mapping parameter is written. Everything the hot path needs
 * (resolved OD entry pointers, byte lengths, COB-IDs) is therefore already
 * flattened by the time co_pdo_tick() runs.
 */
#include "co_pdo.h"
#include "co_node.h"

#define TPDO_COMM_BASE 0x1800u
#define TPDO_MAP_BASE  0x1A00u
#define RPDO_COMM_BASE 0x1400u
#define RPDO_MAP_BASE  0x1600u

/* ---- configuration ------------------------------------------------------ */

static void load_one(co_node_t *n, co_pdo_t *p,
                     uint16_t comm_index, uint16_t map_index, bool is_tx)
{
    memset(p, 0, sizeof(*p));

    uint32_t cob = co_od_get_u32(n->od, comm_index, 1);
    p->valid  = (cob & CO_COBID_INVALID) == 0;
    p->cob_id = cob & CO_COBID_MASK;
    p->trans_type = co_od_get_u8(n->od, comm_index, 2);
    if (is_tx) {
        p->inhibit_100us = co_od_get_u16(n->od, comm_index, 3);
        p->event_ms      = co_od_get_u16(n->od, comm_index, 5);
    } else {
        p->event_ms      = co_od_get_u16(n->od, comm_index, 5); /* deadline */
    }

    uint8_t count = co_od_get_u8(n->od, map_index, 0);
    if (count > CO_PDO_MAX_MAP) count = CO_PDO_MAX_MAP;

    uint8_t total_bits = 0;
    uint8_t k = 0;
    for (uint8_t i = 0; i < count; i++) {
        uint32_t m = co_od_get_u32(n->od, map_index, (uint8_t)(i + 1));
        uint16_t idx  = (uint16_t)(m >> 16);
        uint8_t  sub  = (uint8_t)(m >> 8);
        uint8_t  bits = (uint8_t)m;

        const co_od_entry_t *e = co_od_find(n->od, idx, sub);
        if (!e || (bits & 0x07u) != 0 || bits == 0) continue;  /* validated on write */
        if (total_bits + bits > 64u) break;

        p->map[k]     = e;
        p->map_len[k] = (uint8_t)(bits / 8u);
        total_bits = (uint8_t)(total_bits + bits);
        k++;
    }
    p->n_map = k;
    p->len   = (uint8_t)(total_bits / 8u);
    if (p->len == 0) p->valid = false;
}

void co_pdo_configure(co_node_t *n)
{
    for (unsigned i = 0; i < CO_NUM_TPDO; i++) {
        load_one(n, &n->pdo.tpdo[i], (uint16_t)(TPDO_COMM_BASE + i),
                 (uint16_t)(TPDO_MAP_BASE + i), true);
        n->pdo.tpdo[i].last_tx_ms = n->now_ms;
    }
    for (unsigned i = 0; i < CO_NUM_RPDO; i++) {
        load_one(n, &n->pdo.rpdo[i], (uint16_t)(RPDO_COMM_BASE + i),
                 (uint16_t)(RPDO_MAP_BASE + i), false);
        n->pdo.rpdo_seen[i] = false;
    }
}

uint32_t co_pdo_validate_map(co_node_t *n, uint16_t map_index, uint8_t count)
{
    if (count == 0) return CO_SDO_OK;          /* mapping disabled */
    if (count > CO_PDO_MAX_MAP) return CO_ABORT_PDO_LEN;

    bool is_tx = (map_index >= TPDO_MAP_BASE);
    uint32_t bits = 0;

    for (uint8_t i = 0; i < count; i++) {
        uint32_t m = co_od_get_u32(n->od, map_index, (uint8_t)(i + 1));
        uint16_t idx  = (uint16_t)(m >> 16);
        uint8_t  sub  = (uint8_t)(m >> 8);
        uint8_t  nbit = (uint8_t)m;

        const co_od_entry_t *e = co_od_find(n->od, idx, sub);
        if (!e) return CO_ABORT_NO_PDO_MAP;
        if (nbit == 0 || (nbit & 0x07u) != 0) return CO_ABORT_NO_PDO_MAP;
        if ((nbit / 8u) != e->size) return CO_ABORT_NO_PDO_MAP;
        if (is_tx && !(e->access & CO_ACC_TPDO)) return CO_ABORT_NO_PDO_MAP;
        if (!is_tx && !(e->access & CO_ACC_RPDO)) return CO_ABORT_NO_PDO_MAP;

        bits += nbit;
        if (bits > 64u) return CO_ABORT_PDO_LEN;
    }
    return CO_SDO_OK;
}

/* ---- transmit ----------------------------------------------------------- */

static uint8_t build_payload(co_pdo_t *p, uint8_t *out)
{
    uint8_t off = 0;
    for (uint8_t i = 0; i < p->n_map; i++) {
        uint32_t got = 0;
        if (co_od_read(p->map[i], 0, &out[off], p->map_len[i], &got) != CO_SDO_OK) {
            got = p->map_len[i];
            memset(&out[off], 0, got);
        }
        off = (uint8_t)(off + p->map_len[i]);
    }
    return off;
}

void co_pdo_send_tpdo(co_node_t *n, unsigned i, uint32_t now_ms)
{
    if (i >= CO_NUM_TPDO) return;
    co_pdo_t *p = &n->pdo.tpdo[i];
    if (!p->valid) return;

    co_msg_t m;
    m.id  = p->cob_id;
    m.rtr = 0;
    m.dlc = build_payload(p, m.data);
    for (uint8_t b = m.dlc; b < 8; b++) m.data[b] = 0;

    if (co_node_send(n, &m)) {
        memcpy(p->shadow, m.data, 8);
        p->last_tx_ms = now_ms;
        if (p->inhibit_100us) {
            p->inhibit_until_ms = now_ms + (uint32_t)((p->inhibit_100us + 9u) / 10u);
        }
    }
}

void co_pdo_tick(co_node_t *n, uint32_t now_ms)
{
    if (n->nmt_state != CO_NMT_OPERATIONAL) return;

    for (unsigned i = 0; i < CO_NUM_TPDO; i++) {
        co_pdo_t *p = &n->pdo.tpdo[i];
        if (!p->valid) continue;
        if (p->trans_type <= CO_PDO_TT_RTR_SYNC) continue;  /* synchronous: driven by SYNC */

        bool due = false;

        /* Event timer (1800h sub 5): unconditional periodic transmission. */
        if (p->event_ms > 0 && co_elapsed(now_ms, p->last_tx_ms) >= p->event_ms) {
            due = true;
        }

        /* Change of state, subject to the inhibit time (1800h sub 3). */
        if (!due) {
            uint8_t tmp[8] = {0};
            build_payload(p, tmp);
            if (memcmp(tmp, p->shadow, p->len) != 0 &&
                (p->inhibit_100us == 0 || (int32_t)(now_ms - p->inhibit_until_ms) >= 0)) {
                due = true;
            }
        }

        if (due) co_pdo_send_tpdo(n, i, now_ms);
    }

    /* RPDO deadline monitoring: 1400h sub 5 is the "event timer" which for an
     * RPDO means "I expect one at least this often". Losing the master's
     * command stream must stop the motors, so this feeds the 402 layer. */
    for (unsigned i = 0; i < CO_NUM_RPDO; i++) {
        co_pdo_t *p = &n->pdo.rpdo[i];
        if (!p->valid || p->event_ms == 0 || !n->pdo.rpdo_seen[i]) continue;
        if (co_elapsed(now_ms, n->pdo.rpdo_last_ms[i]) > p->event_ms) {
            n->pdo.rpdo_seen[i] = false;   /* one-shot; 402 layer reads this */
        }
    }
}

void co_pdo_sync(co_node_t *n)
{
    if (n->nmt_state != CO_NMT_OPERATIONAL) return;
    n->sync_count++;

    for (unsigned i = 0; i < CO_NUM_TPDO; i++) {
        co_pdo_t *p = &n->pdo.tpdo[i];
        if (!p->valid) continue;

        if (p->trans_type == CO_PDO_TT_SYNC_ACYCLIC) {
            uint8_t tmp[8] = {0};
            build_payload(p, tmp);
            if (memcmp(tmp, p->shadow, p->len) != 0) co_pdo_send_tpdo(n, i, n->now_ms);
        } else if (p->trans_type >= CO_PDO_TT_SYNC_CYCLIC_MIN &&
                   p->trans_type <= CO_PDO_TT_SYNC_CYCLIC_MAX) {
            if (++p->sync_cnt >= p->trans_type) {
                p->sync_cnt = 0;
                co_pdo_send_tpdo(n, i, n->now_ms);
            }
        }
    }
}

/* ---- receive ------------------------------------------------------------ */

bool co_pdo_rx(co_node_t *n, const co_msg_t *msg)
{
    if (n->nmt_state != CO_NMT_OPERATIONAL) return false;

    for (unsigned i = 0; i < CO_NUM_RPDO; i++) {
        co_pdo_t *p = &n->pdo.rpdo[i];
        if (!p->valid || p->cob_id != msg->id) continue;

        if (msg->dlc < p->len) {
            /* Too short to unpack: report it instead of silently using junk. */
            uint8_t info[5] = { (uint8_t)(i + 1), msg->dlc, p->len, 0, 0 };
            co_emcy_raise(n, CO_EMCY_PDO_LEN_ERROR, CO_ERRREG_COMMUNICATION, info);
            return true;
        }

        uint8_t off = 0;
        for (uint8_t k = 0; k < p->n_map; k++) {
            (void)co_od_write(p->map[k], 0, &msg->data[off], p->map_len[k], true);
            off = (uint8_t)(off + p->map_len[k]);
        }
        n->pdo.rpdo_last_ms[i] = n->now_ms;
        n->pdo.rpdo_seen[i] = true;
        return true;
    }
    return false;
}
