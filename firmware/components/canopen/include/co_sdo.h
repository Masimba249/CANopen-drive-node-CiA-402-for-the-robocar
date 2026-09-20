/**
 * @file co_sdo.h
 * @brief SDO server: expedited, segmented and block download, plus
 *        expedited/segmented upload.
 *
 * Block *download* (client -> server) is implemented because that is what the
 * firmware-over-CAN path needs: it moves 7 bytes per frame with one
 * acknowledge per 127 frames instead of one per frame, which is roughly a 2x
 * throughput win over segmented transfer.
 *
 * Block *upload* is deliberately not implemented; the server answers such a
 * request with abort 0x05040001 (command specifier not valid) and clients
 * fall back to segmented upload.
 */
#ifndef CO_SDO_H
#define CO_SDO_H

#include "co_types.h"
#include "co_od.h"

#ifdef __cplusplus
extern "C" {
#endif

struct co_node;

/** Default SDO protocol timeout: abort a stalled transfer after this long. */
#ifndef CO_SDO_TIMEOUT_MS
#define CO_SDO_TIMEOUT_MS 1000u
#endif

/** Sub-block size we advertise (segments per acknowledge). Max is 127. */
#ifndef CO_SDO_BLKSIZE
#define CO_SDO_BLKSIZE 127u
#endif

typedef enum {
    CO_SDO_ST_IDLE = 0,
    CO_SDO_ST_DOWNLOAD_SEG,   /**< segmented download in progress  */
    CO_SDO_ST_UPLOAD_SEG,     /**< segmented upload in progress    */
    CO_SDO_ST_BLK_DOWNLOAD,   /**< receiving block sub-blocks      */
    CO_SDO_ST_BLK_END,        /**< waiting for "end block download"*/
} co_sdo_state_t;

typedef struct {
    co_sdo_state_t       state;
    const co_od_entry_t *entry;
    uint32_t             index_sub;   /**< for abort reporting: idx<<8|sub  */
    uint32_t             offset;      /**< bytes committed so far           */
    uint32_t             total;       /**< indicated size, 0 = not indicated*/
    bool                 size_known;
    uint8_t              toggle;
    uint32_t             last_ms;

    /* --- block download --------------------------------------------- */
    uint8_t  blksize;       /**< segments per sub-block we asked for    */
    uint8_t  ackseq;        /**< last in-sequence seqno received        */
    bool     crc_enabled;   /**< both sides support the CRC             */
    uint16_t crc;
    bool     last_seen;     /**< c=1 segment has arrived                */
    uint8_t  tail[7];       /**< final segment, held back for trimming  */
    uint8_t  blkbuf[CO_SDO_BLKSIZE * 7];
    uint32_t blkbuf_len;

    /* --- segmented upload ------------------------------------------- */
    uint8_t  upbuf[7];
    uint32_t upbuf_len;
} co_sdo_server_t;

void co_sdo_init(co_sdo_server_t *s);

/** Handle one frame received on 0x600+node-id. */
void co_sdo_rx(struct co_node *node, const co_msg_t *msg);

/** Periodic tick: aborts a transfer that has gone quiet. */
void co_sdo_tick(struct co_node *node, uint32_t now_ms);

/** Send an SDO abort and return the server to idle. */
void co_sdo_abort(struct co_node *node, uint16_t index, uint8_t sub, uint32_t code);

#ifdef __cplusplus
}
#endif
#endif /* CO_SDO_H */
