/**
 * @file co_node.h
 * @brief The CANopen node: ties the OD, SDO server, PDO engine, NMT slave,
 *        heartbeat producer/consumer, SYNC consumer and EMCY producer
 *        together behind three functions.
 *
 * Threading model
 * ---------------
 * The node is *not* internally locked. Call co_node_rx() and co_node_tick()
 * from the same task (see main/main.c, which runs both from the 1 ms CANopen
 * task). Anything produced by another task reaches the node through the
 * object dictionary variables, which are word-sized and written atomically.
 */
#ifndef CO_NODE_H
#define CO_NODE_H

#include "co_types.h"
#include "co_od.h"
#include "co_sdo.h"
#include "co_pdo.h"
#include "co_emcy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct co_node co_node_t;

/** Transmit one frame. Return false if the frame could not be queued. */
typedef bool (*co_send_fn)(void *ctx, const co_msg_t *msg);

/** Called on every NMT state change (including the initial bootup). */
typedef void (*co_nmt_cb_fn)(co_node_t *node, co_nmt_state_t state, void *ctx);

/** Called when NMT "reset node" / "reset communication" is requested. */
typedef void (*co_reset_cb_fn)(co_node_t *node, bool full_reset, void *ctx);

/** Called when a configured heartbeat producer stops being heard. */
typedef void (*co_hb_lost_cb_fn)(co_node_t *node, uint8_t node_id, void *ctx);

typedef struct {
    uint8_t             node_id;      /**< 1..127 */
    const co_od_t      *od;
    co_send_fn          send;
    void               *ctx;
    co_nmt_cb_fn        on_nmt;
    co_reset_cb_fn      on_reset;
    co_hb_lost_cb_fn    on_hb_lost;
} co_node_cfg_t;

typedef struct {
    uint8_t  node_id;      /**< 0 = slot unused */
    uint16_t timeout_ms;
    uint32_t last_ms;
    bool     seen;
    bool     lost;
} co_hb_consumer_t;

struct co_node {
    uint8_t          node_id;
    co_nmt_state_t   nmt_state;
    const co_od_t   *od;
    co_send_fn       send;
    void            *ctx;
    co_nmt_cb_fn     on_nmt;
    co_reset_cb_fn   on_reset;
    co_hb_lost_cb_fn on_hb_lost;

    co_sdo_server_t  sdo;
    co_pdo_engine_t  pdo;
    co_emcy_t        emcy;

    uint32_t         hb_period_ms;     /**< cached 1017h */
    uint32_t         hb_last_ms;
    co_hb_consumer_t hb_cons[CO_HB_CONSUMERS];

    uint32_t         sync_cob_id;
    uint32_t         sync_count;

    uint32_t         now_ms;
    uint32_t         tx_dropped;       /**< frames the driver refused     */
    bool             cfg_dirty;        /**< PDO parameters need reloading */
};

/**
 * Initialise and send the boot-up message. The node enters PRE-OPERATIONAL,
 * exactly as CiA 301 §7.3.2 requires.
 */
void co_node_init(co_node_t *node, const co_node_cfg_t *cfg);

/** Feed one received CAN frame in. Safe to call with any frame. */
void co_node_rx(co_node_t *node, const co_msg_t *msg);

/** Call at a steady rate (1 ms in this project). Drives every timer. */
void co_node_tick(co_node_t *node, uint32_t now_ms);

/** Enter an NMT state locally (used by the abort-connection option code). */
void co_node_set_nmt(co_node_t *node, co_nmt_state_t state);

/** Queue a frame; counts drops. */
bool co_node_send(co_node_t *node, const co_msg_t *msg);

/** Mark PDO/heartbeat parameters as changed so they are reloaded next tick. */
static inline void co_node_mark_dirty(co_node_t *node) { node->cfg_dirty = true; }

/**
 * The stack keeps one active node pointer so that object-dictionary write
 * hooks (which only receive the entry) can reach it. Single-node device, so
 * this is a deliberate simplification rather than an oversight.
 */
extern co_node_t *co_node_active;

#ifdef __cplusplus
}
#endif
#endif /* CO_NODE_H */
