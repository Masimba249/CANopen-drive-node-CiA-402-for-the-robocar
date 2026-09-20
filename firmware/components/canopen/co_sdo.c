/**
 * @file co_sdo.c
 * @brief SDO server state machine (CiA 301 §7.2.4).
 *
 * Frame layouts implemented here, for reference while reading the code:
 *
 *   initiate download (ccs=1)  : [ 001 n n e s ] idx_lo idx_hi sub  d0..d3
 *   download segment  (ccs=0)  : [ 000 t n n n c ] d0..d6
 *   initiate upload   (ccs=2)  : [ 010 00000 ]    idx_lo idx_hi sub  -
 *   upload segment    (ccs=3)  : [ 011 t 0000 ]   -
 *   block download    (ccs=6)  : [ 110 00 cc s cs ] idx_lo idx_hi sub size
 *   block segment              : [ c  seqno(7) ]  d0..d6
 *   end block download (ccs=6) : [ 110 nnn 0 1 ]  crc_lo crc_hi
 *   abort             (ccs=4)  : [ 100 00000 ]    idx_lo idx_hi sub  code
 */
#include "co_sdo.h"
#include "co_node.h"
#include "co_crc.h"

/* ---- helpers ----------------------------------------------------------- */

static bool sdo_tx(co_node_t *n, const uint8_t d[8])
{
    co_msg_t m;
    m.id  = CO_COB_SDO_TX_BASE + n->node_id;
    m.dlc = 8;
    m.rtr = 0;
    memcpy(m.data, d, 8);
    return co_node_send(n, &m);
}

void co_sdo_abort(co_node_t *n, uint16_t index, uint8_t sub, uint32_t code)
{
    uint8_t d[8];
    d[0] = 0x80;
    d[1] = (uint8_t)index;
    d[2] = (uint8_t)(index >> 8);
    d[3] = sub;
    co_st_u32(&d[4], code);
    sdo_tx(n, d);
    n->sdo.state = CO_SDO_ST_IDLE;
    n->sdo.entry = NULL;
}

/** Abort using the index/sub remembered for the transfer in progress. */
static void abort_current(co_node_t *n, uint32_t code)
{
    co_sdo_abort(n, (uint16_t)(n->sdo.index_sub >> 8),
                 (uint8_t)(n->sdo.index_sub & 0xFF), code);
}

static uint32_t lookup(co_node_t *n, uint16_t idx, uint8_t sub,
                       const co_od_entry_t **out)
{
    const co_od_entry_t *e = co_od_find(n->od, idx, sub);
    if (!e) {
        return co_od_index_exists(n->od, idx) ? CO_ABORT_NO_SUBINDEX
                                              : CO_ABORT_NO_OBJECT;
    }
    *out = e;
    return CO_SDO_OK;
}

void co_sdo_init(co_sdo_server_t *s)
{
    memset(s, 0, sizeof(*s));
    s->state = CO_SDO_ST_IDLE;
}

/* ---- download (client -> server) --------------------------------------- */

static void on_initiate_download(co_node_t *n, const uint8_t *d)
{
    co_sdo_server_t *s = &n->sdo;
    uint16_t idx = co_ld_u16(&d[1]);
    uint8_t  sub = d[3];
    bool     expedited = (d[0] & 0x02u) != 0;
    bool     size_ind  = (d[0] & 0x01u) != 0;

    const co_od_entry_t *e = NULL;
    uint32_t ab = lookup(n, idx, sub, &e);
    if (ab) { co_sdo_abort(n, idx, sub, ab); return; }
    if (!(e->access & CO_ACC_W) || (e->access & CO_ACC_CONST)) {
        co_sdo_abort(n, idx, sub, CO_ABORT_READONLY);
        return;
    }

    if (expedited) {
        uint32_t len = size_ind ? (4u - ((d[0] >> 2) & 0x03u)) : 4u;
        ab = co_od_write(e, 0, &d[4], len, true);
        if (ab) { co_sdo_abort(n, idx, sub, ab); return; }
        s->state = CO_SDO_ST_IDLE;
    } else {
        s->state      = CO_SDO_ST_DOWNLOAD_SEG;
        s->entry      = e;
        s->index_sub  = ((uint32_t)idx << 8) | sub;
        s->offset     = 0;
        s->toggle     = 0;
        s->size_known = size_ind;
        s->total      = size_ind ? co_ld_u32(&d[4]) : 0u;
        s->last_ms    = n->now_ms;

        if (s->size_known && e->size != 0 && s->total > e->size) {
            co_sdo_abort(n, idx, sub, CO_ABORT_DATA_LEN_HIGH);
            return;
        }
    }

    uint8_t r[8] = {0x60, d[1], d[2], d[3], 0, 0, 0, 0};
    sdo_tx(n, r);
}

static void on_download_segment(co_node_t *n, const uint8_t *d)
{
    co_sdo_server_t *s = &n->sdo;

    uint8_t t = (d[0] >> 4) & 0x01u;
    if (t != s->toggle) { abort_current(n, CO_ABORT_TOGGLE_BIT); return; }

    uint32_t len  = 7u - ((d[0] >> 1) & 0x07u);
    bool     last = (d[0] & 0x01u) != 0;

    if (s->size_known) {
        if (s->offset + len > s->total)        { abort_current(n, CO_ABORT_DATA_LEN_HIGH); return; }
        if (last && (s->offset + len) != s->total) { abort_current(n, CO_ABORT_DATA_LEN_LOW); return; }
    }

    uint32_t ab = co_od_write(s->entry, s->offset, &d[1], len, last);
    if (ab) { abort_current(n, ab); return; }

    s->offset += len;
    s->last_ms = n->now_ms;

    uint8_t r[8] = {0};
    r[0] = (uint8_t)(0x20u | (t << 4));
    sdo_tx(n, r);

    s->toggle ^= 1u;
    if (last) { s->state = CO_SDO_ST_IDLE; s->entry = NULL; }
}

/* ---- upload (server -> client) ----------------------------------------- */

static void on_initiate_upload(co_node_t *n, const uint8_t *d)
{
    co_sdo_server_t *s = &n->sdo;
    uint16_t idx = co_ld_u16(&d[1]);
    uint8_t  sub = d[3];

    const co_od_entry_t *e = NULL;
    uint32_t ab = lookup(n, idx, sub, &e);
    if (ab) { co_sdo_abort(n, idx, sub, ab); return; }
    if (!(e->access & CO_ACC_R)) { co_sdo_abort(n, idx, sub, CO_ABORT_WRITEONLY); return; }

    uint32_t size = co_od_size(e);
    uint8_t  r[8] = {0, d[1], d[2], d[3], 0, 0, 0, 0};

    if (size > 0 && size <= 4) {
        uint32_t got = 0;
        ab = co_od_read(e, 0, &r[4], 4, &got);
        if (ab) { co_sdo_abort(n, idx, sub, ab); return; }
        /* expedited: e=1, s=1, n = number of unused bytes in d4..d7 */
        r[0] = (uint8_t)(0x40u | ((4u - got) << 2) | 0x03u);
        s->state = CO_SDO_ST_IDLE;
        sdo_tx(n, r);
        return;
    }

    /* Segmented upload. A DOMAIN object reports size 0, in which case the
     * size is simply not indicated and the client reads until c=1. */
    s->state      = CO_SDO_ST_UPLOAD_SEG;
    s->entry      = e;
    s->index_sub  = ((uint32_t)idx << 8) | sub;
    s->offset     = 0;
    s->toggle     = 0;
    s->size_known = (size > 0);
    s->total      = size;
    s->last_ms    = n->now_ms;

    r[0] = s->size_known ? 0x41u : 0x40u;
    if (s->size_known) co_st_u32(&r[4], size);
    sdo_tx(n, r);
}

static void on_upload_segment(co_node_t *n, const uint8_t *d)
{
    co_sdo_server_t *s = &n->sdo;

    uint8_t t = (d[0] >> 4) & 0x01u;
    if (t != s->toggle) { abort_current(n, CO_ABORT_TOGGLE_BIT); return; }

    uint8_t  r[8] = {0};
    uint32_t got = 0;
    uint32_t ab = co_od_read(s->entry, s->offset, &r[1], 7, &got);
    if (ab) { abort_current(n, ab); return; }

    bool last;
    if (s->size_known) {
        last = (s->offset + got) >= s->total;
    } else {
        last = (got < 7u);   /* a short read ends an unsized (DOMAIN) upload */
    }

    r[0] = (uint8_t)((t << 4) | ((7u - got) << 1) | (last ? 1u : 0u));
    sdo_tx(n, r);

    s->offset += got;
    s->toggle ^= 1u;
    s->last_ms = n->now_ms;
    if (last) { s->state = CO_SDO_ST_IDLE; s->entry = NULL; }
}

/* ---- block download ----------------------------------------------------- */

/**
 * Commit the in-sequence bytes of the current sub-block to the object
 * dictionary. The final 7-byte segment of the whole transfer is held back in
 * s->tail because only the "end block download" frame says how many of those
 * 7 bytes are real data.
 */
static uint32_t blk_commit(co_node_t *n)
{
    co_sdo_server_t *s = &n->sdo;
    uint32_t len = s->blkbuf_len;

    if (s->last_seen && len >= 7u) {
        len -= 7u;
        memcpy(s->tail, &s->blkbuf[len], 7);
    }
    if (len == 0) { s->blkbuf_len = 0; return CO_SDO_OK; }

    if (s->size_known && (s->offset + len) > s->total) return CO_ABORT_DATA_LEN_HIGH;

    uint32_t ab = co_od_write(s->entry, s->offset, s->blkbuf, len, false);
    if (ab) return ab;

    if (s->crc_enabled) s->crc = co_crc16(s->crc, s->blkbuf, len);
    s->offset += len;
    s->blkbuf_len = 0;
    return CO_SDO_OK;
}

static void blk_ack(co_node_t *n)
{
    co_sdo_server_t *s = &n->sdo;
    uint8_t r[8] = {0};
    r[0] = 0xA2u;              /* scs=5, ss=2 : block download sub-block ack */
    r[1] = s->ackseq;
    r[2] = s->blksize;
    sdo_tx(n, r);
}

static void on_block_download_init(co_node_t *n, const uint8_t *d)
{
    co_sdo_server_t *s = &n->sdo;
    uint16_t idx = co_ld_u16(&d[1]);
    uint8_t  sub = d[3];
    bool client_crc = (d[0] & 0x04u) != 0;
    bool size_ind   = (d[0] & 0x02u) != 0;

    const co_od_entry_t *e = NULL;
    uint32_t ab = lookup(n, idx, sub, &e);
    if (ab) { co_sdo_abort(n, idx, sub, ab); return; }
    if (!(e->access & CO_ACC_W) || (e->access & CO_ACC_CONST)) {
        co_sdo_abort(n, idx, sub, CO_ABORT_READONLY);
        return;
    }

    s->state       = CO_SDO_ST_BLK_DOWNLOAD;
    s->entry       = e;
    s->index_sub   = ((uint32_t)idx << 8) | sub;
    s->offset      = 0;
    s->size_known  = size_ind;
    s->total       = size_ind ? co_ld_u32(&d[4]) : 0u;
    s->crc_enabled = client_crc;
    s->crc         = 0;
    s->blksize     = CO_SDO_BLKSIZE;
    s->ackseq      = 0;
    s->blkbuf_len  = 0;
    s->last_seen   = false;
    s->last_ms     = n->now_ms;

    if (s->size_known && e->size != 0 && s->total > e->size) {
        co_sdo_abort(n, idx, sub, CO_ABORT_DATA_LEN_HIGH);
        return;
    }

    uint8_t r[8] = {0};
    r[0] = (uint8_t)(0xA0u | (client_crc ? 0x04u : 0x00u)); /* scs=5, ss=0, sc */
    r[1] = d[1];
    r[2] = d[2];
    r[3] = d[3];
    r[4] = s->blksize;
    sdo_tx(n, r);
}

static void on_block_segment(co_node_t *n, const uint8_t *d)
{
    co_sdo_server_t *s = &n->sdo;

    uint8_t seqno = d[0] & 0x7Fu;
    bool    c     = (d[0] & 0x80u) != 0;
    s->last_ms = n->now_ms;

    if (seqno == 0 || seqno > s->blksize) {
        abort_current(n, CO_ABORT_SEQNO_INVALID);
        return;
    }

    bool accepted = false;
    if (seqno == (uint8_t)(s->ackseq + 1u)) {
        /* In sequence: buffer it. Only in-sequence data is ever committed,
         * so a dropped frame simply makes the client resend from ackseq+1. */
        memcpy(&s->blkbuf[s->blkbuf_len], &d[1], 7);
        s->blkbuf_len += 7u;
        s->ackseq = seqno;
        accepted = true;
        if (c) s->last_seen = true;
    }

    /* Acknowledge when the sub-block is full, or when the client has just
     * delivered the final segment of the transfer. */
    if (seqno >= s->blksize || (accepted && c)) {
        uint32_t ab = blk_commit(n);
        if (ab) { abort_current(n, ab); return; }
        blk_ack(n);
        s->ackseq = 0;
        if (s->last_seen) s->state = CO_SDO_ST_BLK_END;
    }
}

static void on_block_download_end(co_node_t *n, const uint8_t *d)
{
    co_sdo_server_t *s = &n->sdo;

    uint32_t unused   = (d[0] >> 2) & 0x07u;  /* bytes in tail that are padding */
    uint16_t crc_rx   = co_ld_u16(&d[1]);
    uint32_t tail_len = (unused <= 7u) ? (7u - unused) : 0u;

    /* Validate before committing: CRC first, then the announced size. */
    if (s->crc_enabled) {
        uint16_t crc = co_crc16(s->crc, s->tail, tail_len);
        if (crc != crc_rx) { abort_current(n, CO_ABORT_CRC_ERROR); return; }
        s->crc = crc;
    }
    if (s->size_known && (s->offset + tail_len) != s->total) {
        abort_current(n, (s->offset + tail_len) > s->total ? CO_ABORT_DATA_LEN_HIGH
                                                           : CO_ABORT_DATA_LEN_LOW);
        return;
    }

    uint32_t ab = co_od_write(s->entry, s->offset, s->tail, tail_len, true);
    if (ab) { abort_current(n, ab); return; }
    s->offset += tail_len;

    uint8_t r[8] = {0};
    r[0] = 0xA1u;   /* scs=5, ss=1 : end block download response */
    sdo_tx(n, r);

    s->state = CO_SDO_ST_IDLE;
    s->entry = NULL;
}

/* ---- dispatch ----------------------------------------------------------- */

void co_sdo_rx(co_node_t *n, const co_msg_t *msg)
{
    if (msg->dlc < 8) {
        /* A short SDO frame is malformed; CiA 301 allows ignoring it, but
         * answering makes bus-level debugging far less mysterious. */
        co_sdo_abort(n, 0, 0, CO_ABORT_GENERAL);
        return;
    }

    const uint8_t *d = msg->data;
    co_sdo_server_t *s = &n->sdo;

    /* While a block download is running, every frame is a sub-block segment
     * (there is no command specifier to key off). */
    if (s->state == CO_SDO_ST_BLK_DOWNLOAD) {
        /* An abort from the client is the one exception. */
        if (d[0] == 0x80u && msg->dlc == 8 && (d[0] >> 5) == 4u) {
            s->state = CO_SDO_ST_IDLE;
            s->entry = NULL;
            return;
        }
        on_block_segment(n, d);
        return;
    }

    uint8_t ccs = (uint8_t)(d[0] >> 5);
    switch (ccs) {
    case 0: /* download segment */
        if (s->state != CO_SDO_ST_DOWNLOAD_SEG) {
            co_sdo_abort(n, 0, 0, CO_ABORT_CMD_INVALID);
        } else {
            on_download_segment(n, d);
        }
        break;

    case 1: /* initiate download */
        on_initiate_download(n, d);
        break;

    case 2: /* initiate upload */
        on_initiate_upload(n, d);
        break;

    case 3: /* upload segment */
        if (s->state != CO_SDO_ST_UPLOAD_SEG) {
            co_sdo_abort(n, 0, 0, CO_ABORT_CMD_INVALID);
        } else {
            on_upload_segment(n, d);
        }
        break;

    case 4: /* abort from client */
        s->state = CO_SDO_ST_IDLE;
        s->entry = NULL;
        break;

    case 5: /* block upload - not supported */
        co_sdo_abort(n, co_ld_u16(&d[1]), d[3], CO_ABORT_CMD_INVALID);
        break;

    case 6: /* block download: initiate (cs=0) or end (cs=1) */
        if ((d[0] & 0x01u) == 0) {
            on_block_download_init(n, d);
        } else if (s->state == CO_SDO_ST_BLK_END) {
            on_block_download_end(n, d);
        } else {
            co_sdo_abort(n, 0, 0, CO_ABORT_CMD_INVALID);
        }
        break;

    default:
        co_sdo_abort(n, co_ld_u16(&d[1]), d[3], CO_ABORT_CMD_INVALID);
        break;
    }
}

void co_sdo_tick(co_node_t *n, uint32_t now_ms)
{
    co_sdo_server_t *s = &n->sdo;
    if (s->state == CO_SDO_ST_IDLE) return;
    if (co_elapsed(now_ms, s->last_ms) > CO_SDO_TIMEOUT_MS) {
        abort_current(n, CO_ABORT_TIMEOUT);
    }
}
