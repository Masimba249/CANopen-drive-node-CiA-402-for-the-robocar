#include "test_util.h"
#include "co_crc.h"

int g_checks = 0, g_failures = 0;
const char *g_suite = "";
fake_bus_t g_bus;
co_node_t  g_node;

static uint32_t s_now;

bool fake_send(void *ctx, const co_msg_t *m)
{
    (void)ctx;
    if (g_bus.full_fail) return false;
    if (g_bus.n >= BUS_CAP) return false;
    g_bus.msg[g_bus.n++] = *m;
    return true;
}

void bus_clear(void) { g_bus.n = 0; }

const co_msg_t *bus_last_id(uint32_t id)
{
    for (int i = g_bus.n - 1; i >= 0; i--) {
        if (g_bus.msg[i].id == id) return &g_bus.msg[i];
    }
    return NULL;
}

int bus_count_id(uint32_t id)
{
    int c = 0;
    for (int i = 0; i < g_bus.n; i++) if (g_bus.msg[i].id == id) c++;
    return c;
}

void test_node_reset(void)
{
    memset(&g_bus, 0, sizeof(g_bus));
    s_now = 0;
    od_load_defaults();
    od_apply_node_id(TEST_NODE_ID);

    co_node_cfg_t cfg = {
        .node_id = TEST_NODE_ID,
        .od      = &robocar_od,
        .send    = fake_send,
        .ctx     = NULL,
    };
    co_node_init(&g_node, &cfg);
    co_node_tick(&g_node, s_now);
}

void test_advance(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        s_now++;
        co_node_tick(&g_node, s_now);
    }
}

void test_nmt_start(void)
{
    co_msg_t m = { .id = CO_COB_NMT, .dlc = 2, .rtr = 0,
                   .data = { CO_NMT_CMD_START, TEST_NODE_ID } };
    co_node_rx(&g_node, &m);
}

/* ---- SDO client --------------------------------------------------------- */

const co_msg_t *sdo_raw(const uint8_t req[8])
{
    int before = g_bus.n;
    co_msg_t m = { .id = CO_COB_SDO_RX_BASE + TEST_NODE_ID, .dlc = 8, .rtr = 0 };
    memcpy(m.data, req, 8);
    co_node_rx(&g_node, &m);

    for (int i = g_bus.n - 1; i >= before; i--) {
        if (g_bus.msg[i].id == (uint32_t)(CO_COB_SDO_TX_BASE + TEST_NODE_ID)) {
            return &g_bus.msg[i];
        }
    }
    return NULL;
}

static uint32_t abort_code(const co_msg_t *r)
{
    if (!r) return CO_ABORT_TIMEOUT;
    if (r->data[0] == 0x80) return co_ld_u32(&r->data[4]);
    return CO_SDO_OK;
}

uint32_t sdo_write_exp(uint16_t idx, uint8_t sub, const void *data, uint8_t len)
{
    uint8_t req[8] = {0};
    req[0] = (uint8_t)(0x23u | ((4u - len) << 2));   /* ccs=1, e=1, s=1 */
    req[1] = (uint8_t)idx; req[2] = (uint8_t)(idx >> 8); req[3] = sub;
    memcpy(&req[4], data, len);
    return abort_code(sdo_raw(req));
}

uint32_t sdo_read(uint16_t idx, uint8_t sub, uint8_t *out, uint32_t max, uint32_t *len)
{
    uint8_t req[8] = {0};
    req[0] = 0x40;   /* ccs=2 */
    req[1] = (uint8_t)idx; req[2] = (uint8_t)(idx >> 8); req[3] = sub;

    const co_msg_t *r = sdo_raw(req);
    uint32_t ab = abort_code(r);
    if (ab) return ab;

    *len = 0;
    if (r->data[0] & 0x02u) {                 /* expedited */
        uint32_t n = (r->data[0] & 0x01u) ? (4u - ((r->data[0] >> 2) & 3u)) : 4u;
        if (n > max) n = max;
        memcpy(out, &r->data[4], n);
        *len = n;
        return CO_SDO_OK;
    }

    uint8_t toggle = 0;
    for (;;) {
        uint8_t sreq[8] = {0};
        sreq[0] = (uint8_t)(0x60u | (toggle << 4));   /* ccs=3 */
        const co_msg_t *sr = sdo_raw(sreq);
        ab = abort_code(sr);
        if (ab) return ab;

        uint32_t n = 7u - ((sr->data[0] >> 1) & 7u);
        if (*len + n > max) n = max - *len;
        memcpy(out + *len, &sr->data[1], n);
        *len += n;
        if (sr->data[0] & 0x01u) break;
        toggle ^= 1u;
    }
    return CO_SDO_OK;
}

uint32_t sdo_write_seg(uint16_t idx, uint8_t sub, const uint8_t *data, uint32_t len)
{
    uint8_t req[8] = {0};
    req[0] = 0x21;   /* ccs=1, e=0, s=1 */
    req[1] = (uint8_t)idx; req[2] = (uint8_t)(idx >> 8); req[3] = sub;
    co_st_u32(&req[4], len);
    uint32_t ab = abort_code(sdo_raw(req));
    if (ab) return ab;

    uint32_t off = 0;
    uint8_t toggle = 0;
    while (off < len) {
        uint32_t n = (len - off > 7u) ? 7u : (len - off);
        bool last = (off + n) >= len;
        uint8_t s[8] = {0};
        s[0] = (uint8_t)((toggle << 4) | ((7u - n) << 1) | (last ? 1u : 0u));
        memcpy(&s[1], data + off, n);
        ab = abort_code(sdo_raw(s));
        if (ab) return ab;
        off += n;
        toggle ^= 1u;
    }
    return CO_SDO_OK;
}

uint32_t sdo_write_block(uint16_t idx, uint8_t sub, const uint8_t *data,
                         uint32_t len, bool corrupt_crc)
{
    uint8_t req[8] = {0};
    req[0] = 0xC6;   /* ccs=6, cc=1 (CRC supported), s=1, cs=0 */
    req[1] = (uint8_t)idx; req[2] = (uint8_t)(idx >> 8); req[3] = sub;
    co_st_u32(&req[4], len);

    const co_msg_t *r = sdo_raw(req);
    uint32_t ab = abort_code(r);
    if (ab) return ab;
    bool server_crc = (r->data[0] & 0x04u) != 0;
    uint8_t blksize = r->data[4];
    if (blksize == 0 || blksize > 127) return CO_ABORT_BLKSIZE_INVALID;

    uint16_t crc = 0;
    uint32_t off = 0;
    uint8_t  seq = 0;
    uint8_t  tail_pad = 0;

    while (off < len) {
        uint8_t seg[8] = {0};
        uint32_t n = (len - off > 7u) ? 7u : (len - off);
        bool last = (off + n) >= len;
        seq++;
        seg[0] = (uint8_t)((last ? 0x80u : 0x00u) | seq);
        memcpy(&seg[1], data + off, n);
        crc = co_crc16(crc, data + off, n);
        if (last) tail_pad = (uint8_t)(7u - n);
        off += n;

        if (last || seq == blksize) {
            const co_msg_t *ack = sdo_raw(seg);
            ab = abort_code(ack);
            if (ab) return ab;
            if (ack->data[1] != seq) return CO_ABORT_SEQNO_INVALID;
            blksize = ack->data[2];
            seq = 0;
        } else {
            /* Sub-block segments are not individually answered. */
            co_msg_t m = { .id = CO_COB_SDO_RX_BASE + TEST_NODE_ID, .dlc = 8, .rtr = 0 };
            memcpy(m.data, seg, 8);
            co_node_rx(&g_node, &m);
        }
    }

    uint8_t end[8] = {0};
    end[0] = (uint8_t)(0xC1u | (tail_pad << 2));   /* ccs=6, n, cs=1 */
    uint16_t sent_crc = server_crc ? crc : 0;
    if (corrupt_crc) sent_crc = (uint16_t)(sent_crc ^ 0xFFFFu);
    co_st_u16(&end[1], sent_crc);
    return abort_code(sdo_raw(end));
}
