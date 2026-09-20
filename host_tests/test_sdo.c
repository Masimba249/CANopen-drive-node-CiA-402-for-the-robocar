/** @file test_sdo.c - SDO server: expedited, segmented and block transfer. */
#include "test_util.h"
#include "co_crc.h"

/* A stand-in for the FOTA sink, so object 2101h (DOMAIN) can be exercised
 * without any flash driver. */
#define IMG_CAP 8192
static uint8_t  s_img[IMG_CAP];
static uint32_t s_img_len;
static bool     s_img_last;
static bool     s_img_fail;

static uint32_t img_write(uint32_t off, const uint8_t *data, uint32_t len, bool last)
{
    if (s_img_fail) return CO_ABORT_HW_ERROR;
    if (off + len > IMG_CAP) return CO_ABORT_OUT_OF_MEMORY;
    memcpy(&s_img[off], data, len);
    if (off + len > s_img_len) s_img_len = off + len;
    if (last) s_img_last = true;
    return CO_SDO_OK;
}

static uint8_t s_last_cmd;
static uint32_t img_cmd(uint8_t c) { s_last_cmd = c; return CO_SDO_OK; }

static void reset_img(void)
{
    memset(s_img, 0, sizeof(s_img));
    s_img_len = 0; s_img_last = false; s_img_fail = false;
    od_register_fota(img_write, img_cmd);
}

void suite_sdo(void)
{
    SUITE("SDO server");
    test_node_reset();
    reset_img();

    uint8_t  out[64];
    uint32_t len = 0;

    /* --- expedited download + upload --------------------------------- */
    uint16_t cw = 0x0006;
    CHECK_EQ(sdo_write_exp(0x6040, 0, &cw, 2), CO_SDO_OK);
    CHECK_EQ(g_od.axis[0].controlword, 0x0006);

    CHECK_EQ(sdo_read(0x6040, 0, out, sizeof(out), &len), CO_SDO_OK);
    CHECK_EQ(len, 2);
    CHECK_EQ(co_ld_u16(out), 0x0006);

    int32_t tv = -1234;
    CHECK_EQ(sdo_write_exp(0x68FF, 0, &tv, 4), CO_SDO_OK);
    CHECK_EQ(g_od.axis[1].target_velocity, -1234);

    /* --- abort codes -------------------------------------------------- */
    CHECK_EQ(sdo_write_exp(0x5555, 0, &cw, 2), CO_ABORT_NO_OBJECT);
    CHECK_EQ(sdo_write_exp(0x1018, 9, &cw, 2), CO_ABORT_NO_SUBINDEX);
    CHECK_EQ(sdo_write_exp(0x6041, 0, &cw, 2), CO_ABORT_READONLY);
    CHECK_EQ(sdo_write_exp(0x1000, 0, &tv, 4), CO_ABORT_READONLY);  /* const */

    /* --- segmented upload of a string (1008h) ------------------------- */
    CHECK_EQ(sdo_read(0x1008, 0, out, sizeof(out), &len), CO_SDO_OK);
    CHECK_EQ(len, strlen("Robocar CiA402 Drive"));
    CHECK(memcmp(out, "Robocar CiA402 Drive", len) == 0);

    /* --- segmented download into the DOMAIN object -------------------- */
    uint8_t blob[100];
    for (unsigned i = 0; i < sizeof(blob); i++) blob[i] = (uint8_t)(i * 7 + 1);
    CHECK_EQ(sdo_write_seg(0x2101, 0, blob, sizeof(blob)), CO_SDO_OK);
    CHECK_EQ(s_img_len, sizeof(blob));
    CHECK(s_img_last);
    CHECK(memcmp(s_img, blob, sizeof(blob)) == 0);

    /* Toggle-bit violation must abort. */
    uint8_t init[8] = {0x21, 0x01, 0x21, 0x00, 4, 0, 0, 0};
    CHECK(sdo_raw(init) != NULL);
    uint8_t bad_toggle[8] = {0x13, 1, 2, 3, 4, 0, 0, 0};  /* t=1 when 0 expected */
    const co_msg_t *r = sdo_raw(bad_toggle);
    CHECK(r && r->data[0] == 0x80);
    CHECK_EQ(co_ld_u32(&r->data[4]), CO_ABORT_TOGGLE_BIT);

    /* --- block download ----------------------------------------------- */
    reset_img();
    uint8_t big[3000];
    for (unsigned i = 0; i < sizeof(big); i++) big[i] = (uint8_t)(i ^ (i >> 8));
    CHECK_EQ(sdo_write_block(0x2101, 0, big, sizeof(big), false), CO_SDO_OK);
    CHECK_EQ(s_img_len, sizeof(big));
    CHECK(s_img_last);
    CHECK(memcmp(s_img, big, sizeof(big)) == 0);

    /* A block whose length is an exact multiple of 7 exercises the
     * "no padding in the final segment" path. */
    reset_img();
    CHECK_EQ(sdo_write_block(0x2101, 0, big, 7 * 130, false), CO_SDO_OK);
    CHECK_EQ(s_img_len, 7u * 130u);
    CHECK(memcmp(s_img, big, 7 * 130) == 0);

    /* A corrupted CRC must be caught and reported as 0x05040004. */
    reset_img();
    CHECK_EQ(sdo_write_block(0x2101, 0, big, 500, true), CO_ABORT_CRC_ERROR);

    /* A downstream failure (flash write error) propagates as an abort. */
    reset_img();
    s_img_fail = true;
    CHECK_EQ(sdo_write_block(0x2101, 0, big, 500, false), CO_ABORT_HW_ERROR);
    reset_img();

    /* --- transfer timeout --------------------------------------------- */
    uint8_t init2[8] = {0x21, 0x01, 0x21, 0x00, 100, 0, 0, 0};
    CHECK(sdo_raw(init2) != NULL);
    bus_clear();
    test_advance(CO_SDO_TIMEOUT_MS + 5);
    r = bus_last_id(CO_COB_SDO_TX_BASE + TEST_NODE_ID);
    CHECK(r && r->data[0] == 0x80);
    CHECK_EQ(co_ld_u32(&r->data[4]), CO_ABORT_TIMEOUT);

    /* --- block upload is not supported and says so -------------------- */
    uint8_t blkup[8] = {0xA0, 0x08, 0x10, 0x00, 0, 0, 0, 0};   /* ccs=5 */
    r = sdo_raw(blkup);
    CHECK(r && r->data[0] == 0x80);
    CHECK_EQ(co_ld_u32(&r->data[4]), CO_ABORT_CMD_INVALID);

    /* --- 1010h store needs the "save" signature ----------------------- */
    uint32_t sig = 0x65766173u;
    uint32_t bad = 0x12345678u;
    CHECK_EQ(sdo_write_exp(0x1010, 1, &bad, 4), CO_ABORT_DATA_TRANSFER);
    CHECK_EQ(sdo_write_exp(0x1010, 1, &sig, 4), CO_ABORT_HW_ERROR); /* no handler */

    /* --- 2100h:03 reaches the FOTA command handler -------------------- */
    uint8_t cmd = 3;
    CHECK_EQ(sdo_write_exp(0x2100, 3, &cmd, 1), CO_SDO_OK);
    CHECK_EQ(s_last_cmd, 3);
}
