/** @file test_pdo.c - PDO mapping, event timers and the reconfiguration rules. */
#include "test_util.h"

#define TPDO1 (0x180u + TEST_NODE_ID)
#define TPDO2 (0x280u + TEST_NODE_ID)
#define TPDO3 (0x380u + TEST_NODE_ID)
#define RPDO1 (0x200u + TEST_NODE_ID)
#define RPDO2 (0x300u + TEST_NODE_ID)

void suite_pdo(void)
{
    SUITE("PDO engine");
    test_node_reset();

    /* Nothing is transmitted in PRE-OPERATIONAL. */
    bus_clear();
    test_advance(100);
    CHECK_EQ(bus_count_id(TPDO1), 0);

    /* Entering OPERATIONAL transmits every TPDO once straight away. */
    bus_clear();
    test_nmt_start();
    CHECK_EQ(g_node.nmt_state, CO_NMT_OPERATIONAL);
    CHECK_EQ(bus_count_id(TPDO1), 1);
    CHECK_EQ(bus_count_id(TPDO2), 1);
    CHECK_EQ(bus_count_id(TPDO3), 1);

    /* TPDO1 = statusword (16) + velocity actual (32) = 6 bytes. */
    const co_msg_t *m = bus_last_id(TPDO1);
    CHECK_EQ(m->dlc, 6);

    g_od.axis[0].statusword      = 0x0637;
    g_od.axis[0].velocity_actual = -4242;
    bus_clear();
    test_advance(1);
    m = bus_last_id(TPDO1);
    CHECK(m != NULL);
    CHECK_EQ(co_ld_u16(&m->data[0]), 0x0637);
    CHECK_EQ((int32_t)co_ld_u32(&m->data[2]), -4242);

    /* Event timer: 10 ms means ~10 frames in 100 ms of quiet bus. */
    g_od.axis[0].velocity_actual = 0;
    g_od.axis[1].velocity_actual = 0;
    bus_clear();
    test_advance(100);
    int n = bus_count_id(TPDO1);
    CHECK(n >= 9 && n <= 11);
    /* TPDO3 is on a 100 ms timer. */
    CHECK_EQ(bus_count_id(TPDO3), 1);

    /* Change of state transmits immediately, without waiting for the timer. */
    bus_clear();
    g_od.axis[1].statusword = 0x1234;
    test_advance(1);
    CHECK_EQ(bus_count_id(TPDO2), 1);

    /* --- RPDO reception ------------------------------------------------ */
    co_msg_t r = { .id = RPDO1, .dlc = 6, .rtr = 0 };
    co_st_u16(&r.data[0], 0x000F);
    co_st_u32(&r.data[2], (uint32_t)(int32_t)-777);
    co_node_rx(&g_node, &r);
    CHECK_EQ(g_od.axis[0].controlword, 0x000F);
    CHECK_EQ(g_od.axis[0].target_velocity, -777);

    r.id = RPDO2;
    co_st_u16(&r.data[0], 0x0006);
    co_st_u32(&r.data[2], 321);
    co_node_rx(&g_node, &r);
    CHECK_EQ(g_od.axis[1].controlword, 0x0006);
    CHECK_EQ(g_od.axis[1].target_velocity, 321);

    /* A short RPDO is reported as an emergency instead of being unpacked. */
    g_od.axis[0].controlword = 0x1111;
    co_msg_t s = { .id = RPDO1, .dlc = 3, .rtr = 0 };
    bus_clear();
    co_node_rx(&g_node, &s);
    test_advance(2);
    CHECK_EQ(g_od.axis[0].controlword, 0x1111);   /* unchanged */
    const co_msg_t *e = bus_last_id(0x80u + TEST_NODE_ID);
    CHECK(e != NULL);
    CHECK_EQ(co_ld_u16(&e->data[0]), CO_EMCY_PDO_LEN_ERROR);

    /* --- remapping at runtime ------------------------------------------ */
    test_node_reset();
    test_nmt_start();

    /* The count must be zeroed before entries may be changed. */
    uint32_t entry = 0x60410010u;
    CHECK_EQ(sdo_write_exp(0x1A00, 1, &entry, 4), CO_ABORT_DATA_DEV_STATE);

    uint8_t zero = 0;
    CHECK_EQ(sdo_write_exp(0x1A00, 0, &zero, 1), CO_SDO_OK);

    /* Map just the statusword: 2 bytes. */
    CHECK_EQ(sdo_write_exp(0x1A00, 1, &entry, 4), CO_SDO_OK);
    uint8_t one = 1;
    CHECK_EQ(sdo_write_exp(0x1A00, 0, &one, 1), CO_SDO_OK);
    bus_clear();
    test_advance(20);
    const co_msg_t *t = bus_last_id(TPDO1);
    CHECK(t != NULL);
    CHECK_EQ(t->dlc, 2);

    /* An object that is not TPDO-mappable is rejected. */
    CHECK_EQ(sdo_write_exp(0x1A00, 0, &zero, 1), CO_SDO_OK);
    entry = 0x60400010u;                      /* controlword: RPDO only */
    CHECK_EQ(sdo_write_exp(0x1A00, 1, &entry, 4), CO_SDO_OK);
    CHECK_EQ(sdo_write_exp(0x1A00, 0, &one, 1), CO_ABORT_NO_PDO_MAP);

    /* A wrong bit length is rejected too. */
    entry = 0x60410020u;                      /* statusword is 16 bit, not 32 */
    CHECK_EQ(sdo_write_exp(0x1A00, 1, &entry, 4), CO_SDO_OK);
    CHECK_EQ(sdo_write_exp(0x1A00, 0, &one, 1), CO_ABORT_NO_PDO_MAP);

    /* Put a valid mapping back before looking at the COB-ID rules. */
    entry = 0x60410010u;
    CHECK_EQ(sdo_write_exp(0x1A00, 1, &entry, 4), CO_SDO_OK);
    CHECK_EQ(sdo_write_exp(0x1A00, 0, &one, 1), CO_SDO_OK);

    /* --- COB-ID may only change while the PDO is invalid ---------------- */
    uint32_t cob = 0x1FFu;
    CHECK_EQ(sdo_write_exp(0x1800, 1, &cob, 4), CO_ABORT_PARAM_INCOMPAT);
    uint32_t invalid = 0x80000000u | (0x180u + TEST_NODE_ID);
    CHECK_EQ(sdo_write_exp(0x1800, 1, &invalid, 4), CO_SDO_OK);
    CHECK_EQ(sdo_write_exp(0x1800, 1, &cob, 4), CO_SDO_OK);
    bus_clear();
    test_advance(20);
    CHECK(bus_count_id(0x1FFu) > 0);

    /* An invalid transmission type is refused. */
    uint8_t tt = 245;
    CHECK_EQ(sdo_write_exp(0x1800, 2, &tt, 1), CO_ABORT_VALUE_RANGE);
    tt = 1;
    CHECK_EQ(sdo_write_exp(0x1800, 2, &tt, 1), CO_SDO_OK);

    /* --- synchronous transmission --------------------------------------- */
    /* TPDO1 is now transmission type 1: one frame per SYNC. */
    bus_clear();
    test_advance(50);
    CHECK_EQ(bus_count_id(0x1FFu), 0);       /* no SYNC yet, no frames */

    co_msg_t sync = { .id = CO_COB_SYNC, .dlc = 0, .rtr = 0 };
    co_node_rx(&g_node, &sync);
    CHECK_EQ(bus_count_id(0x1FFu), 1);
    co_node_rx(&g_node, &sync);
    CHECK_EQ(bus_count_id(0x1FFu), 2);

    /* Transmission type 3 = every third SYNC. */
    tt = 3;
    CHECK_EQ(sdo_write_exp(0x1800, 2, &tt, 1), CO_SDO_OK);
    test_advance(1);
    bus_clear();
    co_node_rx(&g_node, &sync);
    co_node_rx(&g_node, &sync);
    CHECK_EQ(bus_count_id(0x1FFu), 0);
    co_node_rx(&g_node, &sync);
    CHECK_EQ(bus_count_id(0x1FFu), 1);
}
