/**
 * @file fota.h
 * @brief Firmware update over CANopen: SDO block download into the spare OTA
 *        slot, CRC-32 verification, activation, and automatic rollback if the
 *        new image never confirms itself.
 *
 * Protocol, as seen from the master (see master/fota_client.py):
 *
 *   1. SDO write  2100h:01 = image size in bytes
 *   2. SDO write  2100h:02 = CRC-32 (zlib) of the whole image
 *   3. SDO write  2100h:03 = 1 (BEGIN)      -> erases the spare slot
 *   4. SDO block download of 2101h          -> streams into that slot
 *   5. SDO write  2100h:03 = 2 (ACTIVATE)   -> checks size + CRC, sets boot
 *                                              partition, reboots
 *   6. node reboots into the new image, which comes up in "pending verify"
 *   7. SDO write  2100h:03 = 4 (CONFIRM)    -> marks the image good
 *
 * If step 7 does not happen within 2100h:08 seconds, the node marks the image
 * invalid and reboots back into the previous one. That covers the case that
 * matters most on a fieldbus: an image that boots but cannot talk CAN.
 *
 * The rollback itself is done by the ESP-IDF second-stage bootloader, which
 * reads the OTA data partition and refuses an image flagged invalid. Writing
 * a bootloader from scratch would add risk without adding capability, so this
 * project uses the ROM + second-stage bootloader and owns everything above it.
 */
#ifndef FOTA_H
#define FOTA_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Values written to 2100h:03. */
typedef enum {
    FOTA_CMD_NONE     = 0,
    FOTA_CMD_BEGIN    = 1,
    FOTA_CMD_ACTIVATE = 2,
    FOTA_CMD_ABORT    = 3,
    FOTA_CMD_CONFIRM  = 4,
    FOTA_CMD_REBOOT   = 5,
} fota_cmd_t;

/** Values reported in 2100h:04. */
typedef enum {
    FOTA_ST_IDLE            = 0,
    FOTA_ST_RECEIVING       = 1,
    FOTA_ST_ACTIVATED       = 2,  /**< verified, boot partition set, rebooting */
    FOTA_ST_PENDING_CONFIRM = 3,  /**< running a new image, awaiting CONFIRM  */
    FOTA_ST_ERR_STATE       = 0x81, /**< drive not safe / wrong sequence      */
    FOTA_ST_ERR_SIZE        = 0x82,
    FOTA_ST_ERR_CRC         = 0x83,
    FOTA_ST_ERR_FLASH       = 0x84,
} fota_status_t;

/**
 * Read the partition table, publish 2100h:06/07, and detect whether this boot
 * is a freshly flashed image awaiting confirmation.
 */
void fota_init(void);

/** Object 2101h write handler. Registered with od_register_fota(). */
uint32_t fota_on_data(uint32_t offset, const uint8_t *data, uint32_t len, bool last);

/** Object 2100h:03 write handler. Registered with od_register_fota(). */
uint32_t fota_on_command(uint8_t cmd);

/**
 * A predicate the application supplies: true when it is safe to start an
 * update (motors de-energised). An update while the wheels are turning is
 * refused with abort 0x08000022.
 */
void fota_set_safety_check(bool (*fn)(void));

/** Periodic. Handles the confirm timeout and the deferred reboot. */
void fota_tick(uint32_t now_ms);

/** True while an image is being received (the app throttles other work). */
bool fota_is_active(void);

#ifdef __cplusplus
}
#endif
#endif /* FOTA_H */
