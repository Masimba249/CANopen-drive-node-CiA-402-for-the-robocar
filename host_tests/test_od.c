/** @file test_od.c - object dictionary structure and access rules. */
#include "test_util.h"

void suite_od(void)
{
    SUITE("object dictionary");
    test_node_reset();

    /* The binary search depends on this, so it is worth asserting. */
    CHECK(od_check_sorted());
    CHECK(robocar_od.count > 100);

    /* Both axes exist, 0x800 apart, and point at different storage. */
    const co_od_entry_t *cw1 = co_od_find(&robocar_od, 0x6040, 0);
    const co_od_entry_t *cw2 = co_od_find(&robocar_od, 0x6840, 0);
    CHECK(cw1 && cw2);
    CHECK(cw1->data == &g_od.axis[0].controlword);
    CHECK(cw2->data == &g_od.axis[1].controlword);
    CHECK(co_od_find(&robocar_od, 0x6502, 0) != NULL);
    CHECK(co_od_find(&robocar_od, 0x6D02, 0) != NULL);   /* axis 2 supported modes */

    /* Missing object vs missing sub-index are different errors. */
    CHECK(co_od_find(&robocar_od, 0x5555, 0) == NULL);
    CHECK(co_od_index_exists(&robocar_od, 0x1018));
    CHECK(!co_od_index_exists(&robocar_od, 0x5555));
    CHECK(co_od_find(&robocar_od, 0x1018, 9) == NULL);

    /* Device type: CiA 402 servo drive. */
    CHECK_EQ(co_od_get_u32(&robocar_od, 0x1000, 0), 0x00020192u);

    /* Access flags: statusword is TPDO-mappable and read-only, controlword
     * is RPDO-mappable and writable. */
    const co_od_entry_t *sw = co_od_find(&robocar_od, 0x6041, 0);
    CHECK(sw->access & CO_ACC_TPDO);
    CHECK(!(sw->access & CO_ACC_W));
    CHECK(cw1->access & CO_ACC_RPDO);
    CHECK(cw1->access & CO_ACC_W);

    /* Default read/write through the generic API. */
    uint8_t buf[8]; uint32_t len = 0;
    CHECK_EQ(co_od_read(cw1, 0, buf, sizeof(buf), &len), CO_SDO_OK);
    CHECK_EQ(len, 2);

    uint16_t v = 0x000F;
    CHECK_EQ(co_od_write(cw1, 0, (uint8_t *)&v, 2, true), CO_SDO_OK);
    CHECK_EQ(g_od.axis[0].controlword, 0x000F);

    /* Writing a read-only object is refused. */
    CHECK_EQ(co_od_write(sw, 0, (uint8_t *)&v, 2, true), CO_ABORT_READONLY);
    /* Wrong length is refused. */
    CHECK_EQ(co_od_write(cw1, 0, (uint8_t *)&v, 1, true), CO_ABORT_DATA_LEN_LOW);

    /* Node-id substitution filled the pre-defined connection set. */
    CHECK_EQ(g_od.sdo_server_rx, 0x600u + TEST_NODE_ID);
    CHECK_EQ(g_od.cob_id_emcy,   0x080u + TEST_NODE_ID);
    CHECK_EQ(g_od.tpdo_comm[0].cob_id, 0x180u + TEST_NODE_ID);
    CHECK_EQ(g_od.rpdo_comm[1].cob_id, 0x300u + TEST_NODE_ID);
}
