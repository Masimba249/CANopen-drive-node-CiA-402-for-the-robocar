/**
 * @file test_util.h
 * @brief Minimal test harness + a fake CAN bus, so the whole protocol stack
 *        can be exercised on the host with no ESP32 in the loop.
 */
#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "co_node.h"
#include "od_data.h"
#include "cia402.h"

extern int g_checks, g_failures;
extern const char *g_suite;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                      \
    } while (0)

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        long _a = (long)(a), _b = (long)(b);                                   \
        g_checks++;                                                            \
        if (_a != _b) {                                                        \
            g_failures++;                                                      \
            printf("  FAIL %s:%d  %s == %s  (0x%lX != 0x%lX)\n",               \
                   __FILE__, __LINE__, #a, #b, (unsigned long)_a,              \
                   (unsigned long)_b);                                         \
        }                                                                      \
    } while (0)

#define SUITE(name) do { g_suite = name; printf("[ %s ]\n", name); } while (0)

/* ---- fake bus ----------------------------------------------------------- */

#define BUS_CAP 512
typedef struct {
    co_msg_t msg[BUS_CAP];
    int      n;
    bool     full_fail;   /**< set to make every send fail (TX queue full) */
} fake_bus_t;

extern fake_bus_t g_bus;

bool  fake_send(void *ctx, const co_msg_t *m);
void  bus_clear(void);
/** Most recent frame with this COB-ID, or NULL. */
const co_msg_t *bus_last_id(uint32_t id);
/** Number of frames seen with this COB-ID. */
int   bus_count_id(uint32_t id);

/* ---- node under test ---------------------------------------------------- */

#define TEST_NODE_ID 0x10

extern co_node_t g_node;

/** Fresh defaults + a freshly initialised node in PRE-OPERATIONAL. */
void test_node_reset(void);
/** Advance the node clock by @p ms, ticking once per millisecond. */
void test_advance(uint32_t ms);
/** Drive the node to OPERATIONAL via an NMT start command. */
void test_nmt_start(void);

/* ---- SDO client helpers ------------------------------------------------- */

/** Send one raw SDO request and return the response (NULL if none). */
const co_msg_t *sdo_raw(const uint8_t req[8]);

/** Expedited download. Returns 0 on success or the abort code. */
uint32_t sdo_write_exp(uint16_t idx, uint8_t sub, const void *data, uint8_t len);
/** Expedited/segmented upload into @p out. Returns 0 or the abort code. */
uint32_t sdo_read(uint16_t idx, uint8_t sub, uint8_t *out, uint32_t max, uint32_t *len);
/** Segmented download of an arbitrary blob. */
uint32_t sdo_write_seg(uint16_t idx, uint8_t sub, const uint8_t *data, uint32_t len);
/** Block download. @p corrupt_crc deliberately sends a wrong CRC. */
uint32_t sdo_write_block(uint16_t idx, uint8_t sub, const uint8_t *data,
                         uint32_t len, bool corrupt_crc);

/* ---- suites ------------------------------------------------------------- */

void suite_od(void);
void suite_sdo(void);
void suite_cia402(void);
void suite_pdo(void);
void suite_nmt_emcy(void);

#endif /* TEST_UTIL_H */
