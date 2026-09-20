/**
 * @file co_emcy.c
 * @brief Emergency producer (CiA 301 §7.2.7).
 *
 * Errors are *edge* events on the wire but *level* state in the device: the
 * error register (1001h) must reflect what is wrong right now, so the set of
 * active conditions is tracked here and the register is recomputed whenever
 * it changes. That is also what makes "error reset" (EMCY code 0x0000 once
 * the last condition clears) come out right.
 *
 * Raising is queued rather than transmitted inline so the control loop can
 * call co_emcy_raise() without caring about the state of the TX mailbox.
 */
#include "co_emcy.h"
#include "co_node.h"

void co_emcy_init(co_emcy_t *e)
{
    memset(e, 0, sizeof(*e));
}

static void recompute_error_register(co_node_t *n)
{
    uint8_t reg = 0;
    for (uint8_t i = 0; i < n->emcy.n_active; i++) reg |= n->emcy.reg_bit[i];
    /* Bit 0 (generic) is set whenever any error is present. */
    if (n->emcy.n_active) reg |= CO_ERRREG_GENERIC;
    co_od_set_u8(n->od, 0x1001, 0x00, reg);
}

/** Push a code onto the pre-defined error field 1003h (newest at sub-index 1). */
static void push_error_field(co_node_t *n, uint16_t code, uint16_t info)
{
    const co_od_entry_t *e0 = co_od_find(n->od, 0x1003, 0x00);
    if (!e0 || !e0->data) return;

    uint8_t max = 0;
    while (co_od_find(n->od, 0x1003, (uint8_t)(max + 1)) && max < 254) max++;

    uint8_t cnt = *(uint8_t *)e0->data;
    if (cnt < max) cnt++;

    for (uint8_t i = cnt; i > 1; i--) {
        const co_od_entry_t *dst = co_od_find(n->od, 0x1003, i);
        const co_od_entry_t *src = co_od_find(n->od, 0x1003, (uint8_t)(i - 1));
        if (dst && dst->data && src && src->data) {
            *(uint32_t *)dst->data = *(uint32_t *)src->data;
        }
    }
    co_od_set_u32(n->od, 0x1003, 1, ((uint32_t)info << 16) | code);
    *(uint8_t *)e0->data = cnt;
}

static void queue_push(co_node_t *n, uint16_t code, uint8_t reg, const uint8_t info[5])
{
    co_emcy_t *e = &n->emcy;
    uint8_t next = (uint8_t)((e->q_head + 1u) % (uint8_t)(sizeof(e->queue) / sizeof(e->queue[0])));
    if (next == e->q_tail) return;   /* queue full: drop, the condition persists */

    e->queue[e->q_head].code = code;
    e->queue[e->q_head].reg  = reg;
    if (info) memcpy(e->queue[e->q_head].info, info, 5);
    else      memset(e->queue[e->q_head].info, 0, 5);
    e->q_head = next;
}

bool co_emcy_is_active(co_node_t *n, uint16_t code)
{
    for (uint8_t i = 0; i < n->emcy.n_active; i++) {
        if (n->emcy.code[i] == code) return true;
    }
    return false;
}

void co_emcy_raise(co_node_t *n, uint16_t code, uint8_t reg_bit, const uint8_t info[5])
{
    if (co_emcy_is_active(n, code)) return;   /* already reported; do not spam */

    if (n->emcy.n_active < CO_EMCY_MAX_ACTIVE) {
        n->emcy.code[n->emcy.n_active]    = code;
        n->emcy.reg_bit[n->emcy.n_active] = reg_bit;
        n->emcy.n_active++;
    }
    recompute_error_register(n);
    push_error_field(n, code, info ? (uint16_t)(info[0] | (info[1] << 8)) : 0u);

    uint8_t reg = co_od_get_u8(n->od, 0x1001, 0x00);
    queue_push(n, code, reg, info);
}

void co_emcy_clear(co_node_t *n, uint16_t code)
{
    bool found = false;
    for (uint8_t i = 0; i < n->emcy.n_active; i++) {
        if (n->emcy.code[i] == code) {
            for (uint8_t k = (uint8_t)(i + 1); k < n->emcy.n_active; k++) {
                n->emcy.code[k - 1]    = n->emcy.code[k];
                n->emcy.reg_bit[k - 1] = n->emcy.reg_bit[k];
            }
            n->emcy.n_active--;
            found = true;
            break;
        }
    }
    if (!found) return;

    recompute_error_register(n);
    if (n->emcy.n_active == 0) {
        /* "Error reset / no error" carries the *previous* code in bytes 3-4. */
        uint8_t info[5] = { (uint8_t)code, (uint8_t)(code >> 8), 0, 0, 0 };
        queue_push(n, CO_EMCY_NO_ERROR, 0, info);
    }
}

void co_emcy_clear_all(co_node_t *n)
{
    if (n->emcy.n_active == 0) return;
    uint16_t last = n->emcy.code[0];
    n->emcy.n_active = 0;
    recompute_error_register(n);
    uint8_t info[5] = { (uint8_t)last, (uint8_t)(last >> 8), 0, 0, 0 };
    queue_push(n, CO_EMCY_NO_ERROR, 0, info);
}

void co_emcy_tick(co_node_t *n, uint32_t now_ms)
{
    co_emcy_t *e = &n->emcy;
    if (e->q_head == e->q_tail) return;

    /* 1015h inhibit time, in multiples of 100 us. */
    uint16_t inhibit_100us = co_od_get_u16(n->od, 0x1015, 0x00);
    if (inhibit_100us && (int32_t)(now_ms - e->inhibit_until_ms) < 0) return;

    uint32_t cob = co_od_get_u32(n->od, 0x1014, 0x00);
    if (cob & CO_COBID_INVALID) { e->q_tail = e->q_head; return; }

    co_msg_t m;
    m.id  = cob & CO_COBID_MASK;
    m.dlc = 8;
    m.rtr = 0;
    co_st_u16(&m.data[0], e->queue[e->q_tail].code);
    m.data[2] = e->queue[e->q_tail].reg;
    memcpy(&m.data[3], e->queue[e->q_tail].info, 5);

    if (co_node_send(n, &m)) {
        e->q_tail = (uint8_t)((e->q_tail + 1u) % (uint8_t)(sizeof(e->queue) / sizeof(e->queue[0])));
        if (inhibit_100us) e->inhibit_until_ms = now_ms + ((inhibit_100us + 9u) / 10u);
    }
}
