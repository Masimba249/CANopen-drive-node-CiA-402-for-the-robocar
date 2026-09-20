/**
 * @file co_pdo.h
 * @brief PDO engine: mapping, transmission types, event timer, inhibit time.
 *
 * Limitations (documented rather than hidden):
 *  - mapped objects must be byte aligned and a whole number of bytes
 *    (8/16/32 bits). Bit-granular packing is legal in CiA 301 but almost
 *    nobody uses it and it triples the mapping code.
 *  - up to 8 mapping entries and 64 bits per PDO, as per the standard.
 */
#ifndef CO_PDO_H
#define CO_PDO_H

#include "co_types.h"
#include "co_od.h"

#ifdef __cplusplus
extern "C" {
#endif

struct co_node;

#ifndef CO_NUM_TPDO
#define CO_NUM_TPDO 3
#endif
#ifndef CO_NUM_RPDO
#define CO_NUM_RPDO 2
#endif
#define CO_PDO_MAX_MAP 8

/** Transmission types. */
#define CO_PDO_TT_SYNC_ACYCLIC   0
#define CO_PDO_TT_SYNC_CYCLIC_MIN 1
#define CO_PDO_TT_SYNC_CYCLIC_MAX 240
#define CO_PDO_TT_RTR_SYNC       252
#define CO_PDO_TT_RTR_EVENT      253
#define CO_PDO_TT_EVENT_MANU     254
#define CO_PDO_TT_EVENT_PROFILE  255

/** Runtime image of one PDO, rebuilt from the OD by co_pdo_configure(). */
typedef struct {
    bool     valid;
    uint32_t cob_id;          /**< 11-bit, flags stripped        */
    uint8_t  trans_type;
    uint16_t inhibit_100us;
    uint16_t event_ms;
    uint8_t  n_map;
    uint8_t  len;             /**< total mapped length in bytes  */
    const co_od_entry_t *map[CO_PDO_MAX_MAP];
    uint8_t  map_len[CO_PDO_MAX_MAP];

    /* runtime */
    uint32_t last_tx_ms;
    uint32_t inhibit_until_ms;
    uint8_t  sync_cnt;
    uint8_t  shadow[8];       /**< last transmitted payload, for change detect */
} co_pdo_t;

typedef struct {
    co_pdo_t tpdo[CO_NUM_TPDO];
    co_pdo_t rpdo[CO_NUM_RPDO];
    uint32_t rpdo_last_ms[CO_NUM_RPDO];
    bool     rpdo_seen[CO_NUM_RPDO];
} co_pdo_engine_t;

/** Re-read every communication and mapping parameter from the OD. */
void co_pdo_configure(struct co_node *node);

/** Periodic: event timers, inhibit times, change-of-state transmission. */
void co_pdo_tick(struct co_node *node, uint32_t now_ms);

/** Dispatch a received frame to an RPDO if the COB-ID matches. */
bool co_pdo_rx(struct co_node *node, const co_msg_t *msg);

/** A SYNC object arrived: run synchronous PDOs. */
void co_pdo_sync(struct co_node *node);

/** Force transmission of TPDO @p i (0-based) regardless of its timer. */
void co_pdo_send_tpdo(struct co_node *node, unsigned i, uint32_t now_ms);

/**
 * Validate and apply a write to a mapping sub-index 0 (the entry count).
 * Exposed so the OD write hooks in the device layer can call it.
 * Returns an SDO abort code (0 = accepted).
 */
uint32_t co_pdo_validate_map(struct co_node *node, uint16_t map_index, uint8_t count);

#ifdef __cplusplus
}
#endif
#endif /* CO_PDO_H */
