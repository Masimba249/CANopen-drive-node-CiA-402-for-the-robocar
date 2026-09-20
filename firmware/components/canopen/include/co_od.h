/**
 * @file co_od.h
 * @brief Object dictionary: a flat, sorted, const table of entries.
 *
 * Design notes
 * ------------
 * The OD is a single `const` array sorted by (index, subindex) and searched
 * with a binary search. Storage for the *values* lives outside the table
 * (see robocar_od component) so the table itself can live in flash.
 *
 * Every access goes through co_od_read()/co_od_write(), which take a byte
 * offset. That is what lets one code path serve expedited SDO (one shot),
 * segmented SDO (7 bytes at a time) and block SDO (up to 889 bytes at a
 * time), and lets a DOMAIN object such as the firmware image stream straight
 * into flash without ever being buffered whole in RAM.
 */
#ifndef CO_OD_H
#define CO_OD_H

#include "co_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct co_od_entry co_od_entry_t;

/**
 * Optional read handler.
 *
 * @param e      entry being read
 * @param off    byte offset into the object
 * @param buf    destination
 * @param max    space available in @p buf
 * @param[out] len  bytes actually produced
 * @return 0 on success, otherwise an SDO abort code.
 */
typedef uint32_t (*co_od_read_fn)(const co_od_entry_t *e, uint32_t off,
                                  uint8_t *buf, uint32_t max, uint32_t *len);

/**
 * Optional write handler.
 *
 * @param e     entry being written
 * @param off   byte offset into the object
 * @param buf   source bytes
 * @param len   number of bytes
 * @param last  true when this is the final chunk of the transfer
 * @return 0 on success, otherwise an SDO abort code.
 *
 * For plain (non-streaming) objects the default handler has already copied
 * the value into e->data by the time a post-write hook would run; handlers
 * registered here replace that behaviour entirely.
 */
typedef uint32_t (*co_od_write_fn)(const co_od_entry_t *e, uint32_t off,
                                   const uint8_t *buf, uint32_t len, bool last);

struct co_od_entry {
    uint16_t        index;
    uint8_t         sub;
    uint8_t         access;   /**< CO_ACC_* bit mask */
    uint16_t        type;     /**< CO_T_* */
    uint16_t        size;     /**< bytes; 0 for DOMAIN (unknown length) */
    void           *data;     /**< storage, or NULL when handlers do the work */
    co_od_read_fn   rd;       /**< NULL -> default (memcpy from data)  */
    co_od_write_fn  wr;       /**< NULL -> default (memcpy into data)  */
};

/** A whole dictionary: the table plus its length. */
typedef struct {
    const co_od_entry_t *entries;
    uint16_t             count;
} co_od_t;

/** Binary search. Returns NULL if (index,sub) is absent. */
const co_od_entry_t *co_od_find(const co_od_t *od, uint16_t index, uint8_t sub);

/**
 * True if @p index exists at any subindex. Used to distinguish
 * "no such object" (0x06020000) from "no such subindex" (0x06090011).
 */
bool co_od_index_exists(const co_od_t *od, uint16_t index);

/** Total length in bytes of an object, or 0 when unknown (DOMAIN). */
uint32_t co_od_size(const co_od_entry_t *e);

uint32_t co_od_read(const co_od_entry_t *e, uint32_t off,
                    uint8_t *buf, uint32_t max, uint32_t *len);

uint32_t co_od_write(const co_od_entry_t *e, uint32_t off,
                     const uint8_t *buf, uint32_t len, bool last);

/* --- Convenience accessors for internal (non-SDO) use ------------------- */

uint8_t  co_od_get_u8 (const co_od_t *od, uint16_t index, uint8_t sub);
uint16_t co_od_get_u16(const co_od_t *od, uint16_t index, uint8_t sub);
uint32_t co_od_get_u32(const co_od_t *od, uint16_t index, uint8_t sub);
int32_t  co_od_get_i32(const co_od_t *od, uint16_t index, uint8_t sub);

bool co_od_set_u8 (const co_od_t *od, uint16_t index, uint8_t sub, uint8_t v);
bool co_od_set_u16(const co_od_t *od, uint16_t index, uint8_t sub, uint16_t v);
bool co_od_set_u32(const co_od_t *od, uint16_t index, uint8_t sub, uint32_t v);

#ifdef __cplusplus
}
#endif
#endif /* CO_OD_H */
