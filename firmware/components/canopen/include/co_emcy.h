/**
 * @file co_emcy.h
 * @brief Emergency producer with error register (1001h), pre-defined error
 *        field (1003h) and inhibit time (1015h).
 */
#ifndef CO_EMCY_H
#define CO_EMCY_H

#include "co_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct co_node;

/** How many distinct error conditions can be active at once. */
#ifndef CO_EMCY_MAX_ACTIVE
#define CO_EMCY_MAX_ACTIVE 8
#endif

typedef struct {
    uint16_t code[CO_EMCY_MAX_ACTIVE];    /**< active (unresolved) error codes */
    uint8_t  reg_bit[CO_EMCY_MAX_ACTIVE]; /**< error-register bit each one owns */
    uint8_t  n_active;
    uint32_t inhibit_until_ms;
    /* Queue, so raising an error from the control loop never blocks. */
    struct { uint16_t code; uint8_t reg; uint8_t info[5]; } queue[8];
    uint8_t  q_head, q_tail;
} co_emcy_t;

void co_emcy_init(co_emcy_t *e);

/**
 * Raise an emergency. Sets the matching bit in the error register, pushes the
 * code into the pre-defined error field (1003h) and queues an EMCY frame.
 *
 * @param info 5 manufacturer-specific bytes, or NULL for zeros.
 */
void co_emcy_raise(struct co_node *node, uint16_t code, uint8_t err_reg_bit,
                   const uint8_t info[5]);

/**
 * Clear one error condition. When the last one goes away an
 * "error reset / no error" EMCY (code 0x0000) is emitted, as the standard
 * requires.
 */
void co_emcy_clear(struct co_node *node, uint16_t code);

/** Clear every active error (used by a CiA 402 fault reset). */
void co_emcy_clear_all(struct co_node *node);

/** True if @p code is currently active. */
bool co_emcy_is_active(struct co_node *node, uint16_t code);

/** Periodic: drains the queue, honouring the 1015h inhibit time. */
void co_emcy_tick(struct co_node *node, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif /* CO_EMCY_H */
