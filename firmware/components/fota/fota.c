/**
 * @file fota.c
 * @brief Implementation of the CANopen firmware-update state machine.
 */
#include "fota.h"
#include "od_data.h"
#include "co_types.h"
#include "co_crc.h"

#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "fota";

/** Flash is written in whole pages; buffering here keeps the SDO path fast. */
#define FLASH_CHUNK 4096

static esp_ota_handle_t s_handle;
static const esp_partition_t *s_target;
static bool     s_open;
static uint32_t s_written;
static uint32_t s_crc;
static uint8_t  s_buf[FLASH_CHUNK];
static uint32_t s_buf_len;

static bool (*s_safe_fn)(void);
static uint32_t s_now_ms;            /* refreshed by fota_tick() */
static uint32_t s_reboot_at_ms;      /* 0 = no reboot pending */
static uint32_t s_confirm_deadline;  /* 0 = nothing to confirm */
static bool     s_pending_verify;

static void set_status(fota_status_t st) { g_od.fota.status = (uint8_t)st; }

static void close_and_discard(void)
{
    if (s_open) {
        esp_ota_abort(s_handle);
        s_open = false;
    }
    s_written = 0;
    s_buf_len = 0;
    s_crc = CO_CRC32_INIT;
    g_od.fota.bytes_received = 0;
}

void fota_set_safety_check(bool (*fn)(void)) { s_safe_fn = fn; }

bool fota_is_active(void) { return s_open; }

void fota_init(void)
{
    s_crc = CO_CRC32_INIT;

    const esp_partition_t *run = esp_ota_get_running_partition();
    if (run) {
        strncpy(g_od.fota.running_slot, run->label, sizeof(g_od.fota.running_slot) - 1);
        g_od.fota.running_slot[sizeof(g_od.fota.running_slot) - 1] = '\0';
    }

    const esp_app_desc_t *desc = esp_app_get_description();
    if (desc) {
        strncpy(g_od.fota.running_version, desc->version,
                sizeof(g_od.fota.running_version) - 1);
        g_od.fota.running_version[sizeof(g_od.fota.running_version) - 1] = '\0';
        od_set_sw_version(desc->version);
    }

    esp_ota_img_states_t state;
    if (run && esp_ota_get_state_partition(run, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        /* We booted a freshly downloaded image. Until the master confirms it
         * over CAN, the bootloader will roll back on the next reset. */
        s_pending_verify = true;
        set_status(FOTA_ST_PENDING_CONFIRM);
        ESP_LOGW(TAG, "running '%s' on probation; CONFIRM required within %u s",
                 run->label, (unsigned)g_od.fota.confirm_timeout_s);
    } else {
        set_status(FOTA_ST_IDLE);
        ESP_LOGI(TAG, "running '%s' version %s",
                 run ? run->label : "?", g_od.fota.running_version);
    }
}

/* ---- data path ---------------------------------------------------------- */

static uint32_t flush_buffer(void)
{
    if (s_buf_len == 0) return CO_SDO_OK;
    esp_err_t err = esp_ota_write(s_handle, s_buf, s_buf_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        set_status(FOTA_ST_ERR_FLASH);
        close_and_discard();
        return CO_ABORT_HW_ERROR;
    }
    s_buf_len = 0;
    return CO_SDO_OK;
}

uint32_t fota_on_data(uint32_t offset, const uint8_t *data, uint32_t len, bool last)
{
    if (!s_open) {
        set_status(FOTA_ST_ERR_STATE);
        return CO_ABORT_DATA_DEV_STATE;     /* BEGIN was never issued */
    }
    /* Only a sequential stream is supported; a gap means frames were lost in
     * a way the SDO layer should already have caught. */
    if (offset != s_written) {
        ESP_LOGE(TAG, "non-sequential chunk: got %u expected %u",
                 (unsigned)offset, (unsigned)s_written);
        set_status(FOTA_ST_ERR_STATE);
        close_and_discard();
        return CO_ABORT_DATA_TRANSFER;
    }
    if (g_od.fota.image_size && (s_written + len) > g_od.fota.image_size) {
        set_status(FOTA_ST_ERR_SIZE);
        close_and_discard();
        return CO_ABORT_DATA_LEN_HIGH;
    }

    s_crc = co_crc32(s_crc, data, len);
    s_written += len;
    g_od.fota.bytes_received = s_written;

    while (len) {
        uint32_t space = FLASH_CHUNK - s_buf_len;
        uint32_t n = (len < space) ? len : space;
        memcpy(&s_buf[s_buf_len], data, n);
        s_buf_len += n;
        data += n;
        len  -= n;
        if (s_buf_len == FLASH_CHUNK) {
            uint32_t ab = flush_buffer();
            if (ab) return ab;
        }
    }

    if (last) return flush_buffer();
    return CO_SDO_OK;
}

/* ---- commands ----------------------------------------------------------- */

static uint32_t cmd_begin(void)
{
    if (s_pending_verify) {
        /* Chaining updates before the current one is confirmed would leave no
         * known-good image to roll back to. */
        set_status(FOTA_ST_ERR_STATE);
        return CO_ABORT_DATA_DEV_STATE;
    }
    if (s_safe_fn && !s_safe_fn()) {
        ESP_LOGW(TAG, "refused: drive is not in a safe state");
        set_status(FOTA_ST_ERR_STATE);
        return CO_ABORT_DATA_DEV_STATE;
    }
    if (g_od.fota.image_size == 0) {
        set_status(FOTA_ST_ERR_SIZE);
        return CO_ABORT_DATA_LEN_LOW;       /* 2100h:01 must be set first */
    }

    close_and_discard();

    s_target = esp_ota_get_next_update_partition(NULL);
    if (!s_target) {
        set_status(FOTA_ST_ERR_FLASH);
        return CO_ABORT_HW_ERROR;
    }
    if (g_od.fota.image_size > s_target->size) {
        ESP_LOGE(TAG, "image %u B does not fit in '%s' (%u B)",
                 (unsigned)g_od.fota.image_size, s_target->label,
                 (unsigned)s_target->size);
        set_status(FOTA_ST_ERR_SIZE);
        return CO_ABORT_OUT_OF_MEMORY;
    }

    /* Erasing the slot takes a few hundred ms; it happens here rather than
     * mid-transfer so the SDO timeouts during the download stay short. */
    esp_err_t err = esp_ota_begin(s_target, g_od.fota.image_size, &s_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        set_status(FOTA_ST_ERR_FLASH);
        return CO_ABORT_HW_ERROR;
    }
    s_open = true;
    s_crc = CO_CRC32_INIT;
    set_status(FOTA_ST_RECEIVING);
    ESP_LOGI(TAG, "receiving %u bytes into '%s'",
             (unsigned)g_od.fota.image_size, s_target->label);
    return CO_SDO_OK;
}

static uint32_t cmd_activate(uint32_t now_ms)
{
    if (!s_open) {
        set_status(FOTA_ST_ERR_STATE);
        return CO_ABORT_DATA_DEV_STATE;
    }
    if (s_written != g_od.fota.image_size) {
        ESP_LOGE(TAG, "size mismatch: got %u want %u",
                 (unsigned)s_written, (unsigned)g_od.fota.image_size);
        set_status(FOTA_ST_ERR_SIZE);
        close_and_discard();
        return CO_ABORT_DATA_LEN_LOW;
    }

    uint32_t crc = co_crc32_final(s_crc);
    if (crc != g_od.fota.image_crc32) {
        ESP_LOGE(TAG, "CRC mismatch: computed %08X, expected %08X",
                 (unsigned)crc, (unsigned)g_od.fota.image_crc32);
        set_status(FOTA_ST_ERR_CRC);
        close_and_discard();
        return CO_ABORT_CRC_ERROR;
    }

    /* esp_ota_end() also validates the image header and its own SHA-256, so
     * the CRC above is a transport check and this is a content check. */
    esp_err_t err = esp_ota_end(s_handle);
    s_open = false;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        set_status(err == ESP_ERR_OTA_VALIDATE_FAILED ? FOTA_ST_ERR_CRC : FOTA_ST_ERR_FLASH);
        return CO_ABORT_HW_ERROR;
    }
    err = esp_ota_set_boot_partition(s_target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition: %s", esp_err_to_name(err));
        set_status(FOTA_ST_ERR_FLASH);
        return CO_ABORT_HW_ERROR;
    }

    set_status(FOTA_ST_ACTIVATED);
    /* Reboot after a short delay so the SDO response and one last heartbeat
     * reach the master before the bus goes quiet. */
    s_reboot_at_ms = now_ms + 250;
    ESP_LOGW(TAG, "image verified, rebooting into '%s'", s_target->label);
    return CO_SDO_OK;
}

static uint32_t cmd_confirm(void)
{
    if (!s_pending_verify) {
        set_status(FOTA_ST_ERR_STATE);
        return CO_ABORT_DATA_DEV_STATE;
    }
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        set_status(FOTA_ST_ERR_FLASH);
        return CO_ABORT_HW_ERROR;
    }
    s_pending_verify = false;
    s_confirm_deadline = 0;
    set_status(FOTA_ST_IDLE);
    ESP_LOGI(TAG, "image confirmed; rollback cancelled");
    return CO_SDO_OK;
}

uint32_t fota_on_command(uint8_t cmd)
{
    /* The only use for "now" here is scheduling the deferred reboot, and
     * fota_tick() refreshes it every millisecond. */
    uint32_t now = s_now_ms;

    switch ((fota_cmd_t)cmd) {
    case FOTA_CMD_NONE:     return CO_SDO_OK;
    case FOTA_CMD_BEGIN:    return cmd_begin();
    case FOTA_CMD_ACTIVATE: return cmd_activate(now);
    case FOTA_CMD_ABORT:
        close_and_discard();
        set_status(FOTA_ST_IDLE);
        ESP_LOGW(TAG, "update aborted by master");
        return CO_SDO_OK;
    case FOTA_CMD_CONFIRM:  return cmd_confirm();
    case FOTA_CMD_REBOOT:
        s_reboot_at_ms = now + 250;
        return CO_SDO_OK;
    default:
        return CO_ABORT_VALUE_RANGE;
    }
}

void fota_tick(uint32_t now_ms)
{
    s_now_ms = now_ms;

    if (s_reboot_at_ms && (int32_t)(now_ms - s_reboot_at_ms) >= 0) {
        ESP_LOGW(TAG, "restarting");
        esp_restart();
    }

    if (s_pending_verify && g_od.fota.confirm_timeout_s) {
        if (s_confirm_deadline == 0) {
            s_confirm_deadline = now_ms + g_od.fota.confirm_timeout_s * 1000u;
        } else if ((int32_t)(now_ms - s_confirm_deadline) >= 0) {
            ESP_LOGE(TAG, "no CONFIRM within %u s - rolling back",
                     (unsigned)g_od.fota.confirm_timeout_s);
            /* Marks this slot invalid and reboots; the bootloader then picks
             * the previous, known-good image. */
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
    }
}
