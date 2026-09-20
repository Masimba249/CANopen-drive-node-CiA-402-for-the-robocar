/**
 * @file can_port.c
 * @brief ESP32 TWAI binding.
 */
#include "can_port.h"
#include "board.h"

#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "can";
static bool s_running;
static bool s_recovering;

bool can_port_init(uint32_t bitrate_kbps, bool listen_only)
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)BOARD_CAN_TX_GPIO, (gpio_num_t)BOARD_CAN_RX_GPIO,
        listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    g.tx_queue_len = 32;
    g.rx_queue_len = 64;
    g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                       TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL |
                       TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;

    twai_timing_config_t t;
    switch (bitrate_kbps) {
    case 125:  t = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();  break;
    case 250:  t = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();  break;
    case 500:  t = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();  break;
    case 1000: t = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();    break;
    default:
        ESP_LOGE(TAG, "unsupported bitrate %u kbit/s", (unsigned)bitrate_kbps);
        return false;
    }

    /* Accept everything: CANopen filtering is done in software because a
     * node must see NMT (0x000), SYNC (0x080), its own SDO/RPDO COB-IDs and
     * any heartbeat it consumes - not a range a single hardware filter
     * covers. The dual-filter mode could catch two ranges, but at 500 kbit/s
     * the CPU cost of filtering in software is negligible. */
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

#if BOARD_CAN_STBY_GPIO >= 0
    /* Preprocessor rather than an if: with the pin disabled the shift below
     * would be a shift by a negative constant, which the compiler rejects
     * even in unreachable code. */
    gpio_config_t stby = {
        .pin_bit_mask = 1ULL << BOARD_CAN_STBY_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&stby);
    gpio_set_level((gpio_num_t)BOARD_CAN_STBY_GPIO, 0);   /* high-speed mode */
#endif

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed");
        twai_driver_uninstall();
        return false;
    }
    s_running = true;
    ESP_LOGI(TAG, "TWAI up at %u kbit/s on TX=%d RX=%d%s",
             (unsigned)bitrate_kbps, BOARD_CAN_TX_GPIO, BOARD_CAN_RX_GPIO,
             listen_only ? " (listen-only)" : "");
    return true;
}

bool can_port_send(void *ctx, const co_msg_t *msg)
{
    (void)ctx;
    if (!s_running || s_recovering) return false;

    twai_message_t tx = {0};
    tx.identifier = msg->id;
    tx.data_length_code = msg->dlc;
    tx.rtr = msg->rtr;
    /* CANopen is 11-bit only; .extd stays 0. */
    for (int i = 0; i < msg->dlc && i < 8; i++) tx.data[i] = msg->data[i];

    /* Never block the CANopen task on a full TX queue: a PDO that cannot go
     * out now is stale by the time the queue drains anyway. */
    return twai_transmit(&tx, 0) == ESP_OK;
}

bool can_port_receive(co_msg_t *msg, uint32_t timeout_ms)
{
    if (!s_running) return false;

    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) return false;
    if (rx.extd) return false;   /* 29-bit frames are not CANopen traffic here */

    msg->id  = rx.identifier;
    msg->dlc = rx.data_length_code;
    msg->rtr = rx.rtr;
    for (int i = 0; i < 8; i++) msg->data[i] = (i < rx.data_length_code) ? rx.data[i] : 0;
    return true;
}

uint32_t can_port_service(od_diag_t *diag)
{
    if (!s_running) return 0;

    uint32_t events = 0;
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK) {
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
            diag->rx_missed++;
            events |= CAN_EVT_RX_OVERRUN;
        }
        if (alerts & TWAI_ALERT_ERR_PASS) events |= CAN_EVT_ERR_PASSIVE;
        if (alerts & TWAI_ALERT_BUS_OFF) {
            diag->bus_off_count++;
            events |= CAN_EVT_BUS_OFF;
            /* Bus-off recovery needs 128 occurrences of 11 recessive bits.
             * Kick it off here and report recovery when the alert arrives. */
            s_recovering = true;
            twai_initiate_recovery();
            ESP_LOGW(TAG, "bus-off, recovery started (count=%u)", diag->bus_off_count);
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            s_recovering = false;
            twai_start();
            events |= CAN_EVT_RECOVERED;
            ESP_LOGW(TAG, "bus recovered");
        }
    }

    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
        diag->tx_err_counter = (uint8_t)(st.tx_error_counter > 255 ? 255 : st.tx_error_counter);
        diag->rx_err_counter = (uint8_t)(st.rx_error_counter > 255 ? 255 : st.rx_error_counter);
        switch (st.state) {
        case TWAI_STATE_RUNNING:    diag->can_state = 0; break;
        case TWAI_STATE_BUS_OFF:    diag->can_state = 2; break;
        case TWAI_STATE_RECOVERING: diag->can_state = 3; break;
        case TWAI_STATE_STOPPED:    diag->can_state = 4; break;
        default:                    diag->can_state = 1; break;
        }
        /* Error-passive is not its own driver state; derive it from the
         * error counters, which is what a bus analyser shows you too. */
        if (diag->can_state == 0 &&
            (diag->tx_err_counter > 127 || diag->rx_err_counter > 127)) {
            diag->can_state = 1;
        }
    }
    return events;
}
