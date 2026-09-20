#include "co_od.h"

const co_od_entry_t *co_od_find(const co_od_t *od, uint16_t index, uint8_t sub)
{
    if (!od || !od->entries) return NULL;

    uint32_t key = ((uint32_t)index << 8) | sub;
    int lo = 0, hi = (int)od->count - 1;
    while (lo <= hi) {
        int mid = lo + ((hi - lo) / 2);
        uint32_t k = ((uint32_t)od->entries[mid].index << 8) | od->entries[mid].sub;
        if (k == key)      return &od->entries[mid];
        else if (k < key)  lo = mid + 1;
        else               hi = mid - 1;
    }
    return NULL;
}

bool co_od_index_exists(const co_od_t *od, uint16_t index)
{
    if (!od || !od->entries) return false;

    /* Find any entry with this index: binary search on the index alone. */
    int lo = 0, hi = (int)od->count - 1;
    while (lo <= hi) {
        int mid = lo + ((hi - lo) / 2);
        uint16_t k = od->entries[mid].index;
        if (k == index)     return true;
        else if (k < index) lo = mid + 1;
        else                hi = mid - 1;
    }
    return false;
}

uint32_t co_od_size(const co_od_entry_t *e)
{
    return e ? e->size : 0u;
}

uint32_t co_od_read(const co_od_entry_t *e, uint32_t off,
                    uint8_t *buf, uint32_t max, uint32_t *len)
{
    if (!e) return CO_ABORT_NO_OBJECT;
    if (!(e->access & CO_ACC_R)) return CO_ABORT_WRITEONLY;

    if (e->rd) return e->rd(e, off, buf, max, len);
    if (!e->data) return CO_ABORT_NO_DATA;

    if (off > e->size) return CO_ABORT_DATA_LEN_HIGH;
    uint32_t remain = (uint32_t)e->size - off;
    uint32_t n = (remain < max) ? remain : max;
    memcpy(buf, (const uint8_t *)e->data + off, n);
    *len = n;
    return CO_SDO_OK;
}

uint32_t co_od_write(const co_od_entry_t *e, uint32_t off,
                     const uint8_t *buf, uint32_t len, bool last)
{
    if (!e) return CO_ABORT_NO_OBJECT;
    if (!(e->access & CO_ACC_W)) return CO_ABORT_READONLY;
    if (e->access & CO_ACC_CONST) return CO_ABORT_READONLY;

    if (e->wr) return e->wr(e, off, buf, len, last);
    if (!e->data) return CO_ABORT_NO_DATA;

    /* Default handler: a fixed-size object must be written in one piece of
     * exactly the right length. Strings may be shorter than their storage. */
    if (off + len > e->size) return CO_ABORT_DATA_LEN_HIGH;
    if (last && e->type != CO_T_VISIBLE_STRING && e->type != CO_T_OCTET_STRING
             && e->type != CO_T_DOMAIN && (off + len) != e->size) {
        return CO_ABORT_DATA_LEN_LOW;
    }
    memcpy((uint8_t *)e->data + off, buf, len);
    if (last && e->type == CO_T_VISIBLE_STRING && (off + len) < e->size) {
        ((uint8_t *)e->data)[off + len] = '\0';
    }
    return CO_SDO_OK;
}

/* --- convenience accessors --------------------------------------------- */

uint8_t co_od_get_u8(const co_od_t *od, uint16_t index, uint8_t sub)
{
    const co_od_entry_t *e = co_od_find(od, index, sub);
    return (e && e->data) ? *(const uint8_t *)e->data : 0u;
}

uint16_t co_od_get_u16(const co_od_t *od, uint16_t index, uint8_t sub)
{
    const co_od_entry_t *e = co_od_find(od, index, sub);
    return (e && e->data) ? *(const uint16_t *)e->data : 0u;
}

uint32_t co_od_get_u32(const co_od_t *od, uint16_t index, uint8_t sub)
{
    const co_od_entry_t *e = co_od_find(od, index, sub);
    return (e && e->data) ? *(const uint32_t *)e->data : 0u;
}

int32_t co_od_get_i32(const co_od_t *od, uint16_t index, uint8_t sub)
{
    const co_od_entry_t *e = co_od_find(od, index, sub);
    return (e && e->data) ? *(const int32_t *)e->data : 0;
}

bool co_od_set_u8(const co_od_t *od, uint16_t index, uint8_t sub, uint8_t v)
{
    const co_od_entry_t *e = co_od_find(od, index, sub);
    if (!e || !e->data) return false;
    *(uint8_t *)e->data = v;
    return true;
}

bool co_od_set_u16(const co_od_t *od, uint16_t index, uint8_t sub, uint16_t v)
{
    const co_od_entry_t *e = co_od_find(od, index, sub);
    if (!e || !e->data) return false;
    *(uint16_t *)e->data = v;
    return true;
}

bool co_od_set_u32(const co_od_t *od, uint16_t index, uint8_t sub, uint32_t v)
{
    const co_od_entry_t *e = co_od_find(od, index, sub);
    if (!e || !e->data) return false;
    *(uint32_t *)e->data = v;
    return true;
}
