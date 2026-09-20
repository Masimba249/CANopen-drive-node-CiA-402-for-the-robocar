/** @file test_nmt_emcy.c - NMT slave, heartbeat and the emergency producer. */
#include "test_util.h"

#define HB_ID   (0x700u + TEST_NODE_ID)
#define EMCY_ID (0x080u + TEST_NODE_ID)

static void nmt(uint8_t cmd, uint8_t node)
{
    co_msg_t m = { .id = CO_COB_NMT, .dlc = 2, .rtr = 0, .data = { cmd, node } };
    co_node_rx(&g_node, &m);
}

void suite_nmt_emcy(void)
{
    SUITE("NMT, heartbeat and EMCY");
    test_node_reset();

    /* --- boot-up message ------------------------------------------------ */
    const co_msg_t *m = bus_last_id(HB_ID);
    CHECK(m != NULL);
    CHECK_EQ(m->dlc, 1);
    CHECK_EQ(m->data[0], CO_NMT_BOOTUP);
    CHECK_EQ(g_node.nmt_state, CO_NMT_PRE_OPERATIONAL);

    /* --- heartbeat producer (1017h = 500 ms) ---------------------------- */
    bus_clear();
    test_advance(1100);
    CHECK_EQ(bus_count_id(HB_ID), 2);
    CHECK_EQ(bus_last_id(HB_ID)->data[0], CO_NMT_PRE_OPERATIONAL);

    /* --- NMT transitions ------------------------------------------------ */
    nmt(CO_NMT_CMD_START, TEST_NODE_ID);
    CHECK_EQ(g_node.nmt_state, CO_NMT_OPERATIONAL);
    bus_clear();
    test_advance(600);
    CHECK_EQ(bus_last_id(HB_ID)->data[0], CO_NMT_OPERATIONAL);

    nmt(CO_NMT_CMD_ENTER_PREOP, 0);          /* broadcast */
    CHECK_EQ(g_node.nmt_state, CO_NMT_PRE_OPERATIONAL);

    /* A command addressed to a different node is ignored. */
    nmt(CO_NMT_CMD_START, TEST_NODE_ID + 1);
    CHECK_EQ(g_node.nmt_state, CO_NMT_PRE_OPERATIONAL);

    /* --- STOPPED: heartbeat keeps running, SDO does not ------------------ */
    nmt(CO_NMT_CMD_STOP, TEST_NODE_ID);
    CHECK_EQ(g_node.nmt_state, CO_NMT_STOPPED);
    bus_clear();
    uint8_t req[8] = {0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0};
    CHECK(sdo_raw(req) == NULL);
    test_advance(600);
    CHECK(bus_count_id(HB_ID) >= 1);
    CHECK_EQ(bus_last_id(HB_ID)->data[0], CO_NMT_STOPPED);

    nmt(CO_NMT_CMD_START, TEST_NODE_ID);
    CHECK(sdo_raw(req) != NULL);

    /* --- reset communication re-sends the boot-up message ---------------- */
    bus_clear();
    nmt(CO_NMT_CMD_RESET_COMM, TEST_NODE_ID);
    m = bus_last_id(HB_ID);
    CHECK(m && m->data[0] == CO_NMT_BOOTUP);
    CHECK_EQ(g_node.nmt_state, CO_NMT_PRE_OPERATIONAL);

    /* --- EMCY ------------------------------------------------------------ */
    test_node_reset();
    bus_clear();
    uint8_t info[5] = { 1, 2, 3, 4, 5 };
    co_emcy_raise(&g_node, CO_EMCY_CURRENT_OVERCURRENT, CO_ERRREG_CURRENT, info);
    test_advance(2);

    m = bus_last_id(EMCY_ID);
    CHECK(m != NULL);
    CHECK_EQ(m->dlc, 8);
    CHECK_EQ(co_ld_u16(&m->data[0]), CO_EMCY_CURRENT_OVERCURRENT);
    CHECK_EQ(m->data[2], CO_ERRREG_CURRENT | CO_ERRREG_GENERIC);
    CHECK_EQ(m->data[3], 1);
    CHECK_EQ(g_od.error_register, CO_ERRREG_CURRENT | CO_ERRREG_GENERIC);

    /* 1003h records it, newest first. */
    CHECK_EQ(g_od.error_count, 1);
    CHECK_EQ(g_od.error_field[0] & 0xFFFFu, CO_EMCY_CURRENT_OVERCURRENT);

    /* Raising the same code twice does not produce a second frame. */
    int before = bus_count_id(EMCY_ID);
    co_emcy_raise(&g_node, CO_EMCY_CURRENT_OVERCURRENT, CO_ERRREG_CURRENT, info);
    test_advance(200);
    CHECK_EQ(bus_count_id(EMCY_ID), before);

    /* A second, different condition is reported and stacks in 1003h. */
    co_emcy_raise(&g_node, CO_EMCY_MANU_STALL, CO_ERRREG_MANUFACTURER, NULL);
    test_advance(200);
    CHECK_EQ(g_od.error_count, 2);
    CHECK_EQ(g_od.error_field[0] & 0xFFFFu, CO_EMCY_MANU_STALL);
    CHECK_EQ(g_od.error_field[1] & 0xFFFFu, CO_EMCY_CURRENT_OVERCURRENT);
    CHECK_EQ(g_od.error_register,
             CO_ERRREG_CURRENT | CO_ERRREG_MANUFACTURER | CO_ERRREG_GENERIC);

    /* Clearing one leaves the other; clearing the last emits "no error". */
    co_emcy_clear(&g_node, CO_EMCY_CURRENT_OVERCURRENT);
    test_advance(200);
    CHECK_EQ(g_od.error_register, CO_ERRREG_MANUFACTURER | CO_ERRREG_GENERIC);
    bus_clear();
    co_emcy_clear(&g_node, CO_EMCY_MANU_STALL);
    test_advance(200);
    m = bus_last_id(EMCY_ID);
    CHECK(m != NULL);
    CHECK_EQ(co_ld_u16(&m->data[0]), CO_EMCY_NO_ERROR);
    CHECK_EQ(g_od.error_register, 0);

    /* Writing 0 to 1003h:00 clears the history; anything else is refused. */
    uint8_t five = 5, zero = 0;
    CHECK_EQ(sdo_write_exp(0x1003, 0, &five, 1), CO_ABORT_VALUE_RANGE);
    CHECK_EQ(sdo_write_exp(0x1003, 0, &zero, 1), CO_SDO_OK);
    CHECK_EQ(g_od.error_count, 0);

    /* --- heartbeat consumer ---------------------------------------------- */
    test_node_reset();
    uint32_t cons = (0x7Fu << 16) | 200u;   /* watch node 0x7F, 200 ms */
    CHECK_EQ(sdo_write_exp(0x1016, 1, &cons, 4), CO_SDO_OK);
    test_advance(1);

    co_msg_t hb = { .id = 0x700u + 0x7Fu, .dlc = 1, .rtr = 0, .data = { 5 } };
    co_node_rx(&g_node, &hb);
    bus_clear();
    test_advance(100);
    CHECK(bus_last_id(EMCY_ID) == NULL);     /* still alive */

    test_advance(300);                       /* master goes quiet */
    m = bus_last_id(EMCY_ID);
    CHECK(m != NULL);
    CHECK_EQ(co_ld_u16(&m->data[0]), CO_EMCY_HEARTBEAT_LOST);

    /* It recovers when the heartbeat comes back. */
    bus_clear();
    co_node_rx(&g_node, &hb);
    test_advance(10);
    m = bus_last_id(EMCY_ID);
    CHECK(m != NULL);
    CHECK_EQ(co_ld_u16(&m->data[0]), CO_EMCY_NO_ERROR);
}
