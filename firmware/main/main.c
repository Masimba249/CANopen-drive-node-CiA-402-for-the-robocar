/**
 * @file main.c
 * @brief Robocar CANopen drive node: application wiring.
 *
 * Task model
 * ----------
 * One task does everything time-critical, at a 1 ms period:
 *
 *   drain the TWAI receive queue  ->  co_node_rx()
 *   co_node_tick()                     SDO timeouts, PDO timers, heartbeat
 *   every 5 ms: motor_update() + cia402_tick()
 *   every 100 ms: CAN diagnostics, supply voltage, housekeeping
 *
 * Running the protocol stack and the control loop in the same task removes
 * every lock between them: the object dictionary is only ever touched from
 * here. The cost is that a slow operation in this task delays PDOs, which is
 * why flash erasing during a firmware update happens in one explicit step
 * (FOTA BEGIN) and why the update is refused while the drive is enabled.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"

#include "app_config.h"
#include "board.h"
#include "co_node.h"
#include "od_data.h"
#include "cia402.h"
#include "can_port.h"
#include "motor.h"
#include "fota.h"

static const char *TAG = "app";

static co_node_t     s_node;
static cia402_axis_t s_axis[ROBOCAR_NUM_AXES];
static bool          s_cmd_lost[ROBOCAR_NUM_AXES];

/* ---------------------------------------------------------------------- */
/* Persisted parameters (object 1010h / 1011h)                             */
/* ---------------------------------------------------------------------- */

typedef struct {
    uint32_t       version;
    od_motor_cfg_t motor[ROBOCAR_NUM_AXES];
    od_protect_t   protect;
    od_netcfg_t    netcfg;
    uint16_t       producer_hb;
    uint16_t       confirm_timeout_s;
    struct {
        uint32_t max_profile_velocity;
        uint32_t profile_acceleration;
        uint32_t profile_deceleration;
        uint32_t quick_stop_deceleration;
        int16_t  quick_stop_option;
        int16_t  abort_connection_option;
        uint16_t velocity_window;
        uint16_t velocity_window_time;
    } axis[ROBOCAR_NUM_AXES];
} persist_t;

static void params_gather(persist_t *p)
{
    memset(p, 0, sizeof(*p));
    p->version = APP_PARAM_VERSION;
    memcpy(p->motor, g_od.motor, sizeof(p->motor));
    p->protect           = g_od.protect;
    p->netcfg            = g_od.netcfg;
    p->producer_hb       = g_od.producer_hb;
    p->confirm_timeout_s = g_od.fota.confirm_timeout_s;
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        p->axis[i].max_profile_velocity    = g_od.axis[i].max_profile_velocity;
        p->axis[i].profile_acceleration    = g_od.axis[i].profile_acceleration;
        p->axis[i].profile_deceleration    = g_od.axis[i].profile_deceleration;
        p->axis[i].quick_stop_deceleration = g_od.axis[i].quick_stop_deceleration;
        p->axis[i].quick_stop_option       = g_od.axis[i].quick_stop_option;
        p->axis[i].abort_connection_option = g_od.axis[i].abort_connection_option;
        p->axis[i].velocity_window         = g_od.axis[i].velocity_window;
        p->axis[i].velocity_window_time    = g_od.axis[i].velocity_window_time;
    }
}

static void params_apply(const persist_t *p)
{
    memcpy(g_od.motor, p->motor, sizeof(g_od.motor));
    g_od.protect                 = p->protect;
    g_od.netcfg                  = p->netcfg;
    g_od.producer_hb             = p->producer_hb;
    g_od.fota.confirm_timeout_s  = p->confirm_timeout_s;
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        g_od.axis[i].max_profile_velocity    = p->axis[i].max_profile_velocity;
        g_od.axis[i].profile_acceleration    = p->axis[i].profile_acceleration;
        g_od.axis[i].profile_deceleration    = p->axis[i].profile_deceleration;
        g_od.axis[i].quick_stop_deceleration = p->axis[i].quick_stop_deceleration;
        g_od.axis[i].quick_stop_option       = p->axis[i].quick_stop_option;
        g_od.axis[i].abort_connection_option = p->axis[i].abort_connection_option;
        g_od.axis[i].velocity_window         = p->axis[i].velocity_window;
        g_od.axis[i].velocity_window_time    = p->axis[i].velocity_window_time;
    }
}

static bool params_load(void)
{
    nvs_handle_t h;
    if (nvs_open(APP_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    persist_t p;
    size_t len = sizeof(p);
    esp_err_t err = nvs_get_blob(h, APP_NVS_KEY_PARAMS, &p, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(p)) return false;
    if (p.version != APP_PARAM_VERSION) {
        ESP_LOGW(TAG, "stored parameters are version %u, expected %u - ignoring",
                 (unsigned)p.version, APP_PARAM_VERSION);
        return false;
    }
    params_apply(&p);
    ESP_LOGI(TAG, "parameters restored from NVS");
    return true;
}

/** 1010h:01 handler. */
static uint32_t on_store(void)
{
    persist_t p;
    params_gather(&p);

    nvs_handle_t h;
    if (nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return CO_ABORT_HW_ERROR;
    esp_err_t err = nvs_set_blob(h, APP_NVS_KEY_PARAMS, &p, sizeof(p));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) return CO_ABORT_HW_ERROR;
    ESP_LOGI(TAG, "parameters stored");
    return CO_SDO_OK;
}

/** 1011h:01 handler: back to compiled-in defaults (after the next reset). */
static uint32_t on_restore(void)
{
    nvs_handle_t h;
    if (nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return CO_ABORT_HW_ERROR;
    nvs_erase_key(h, APP_NVS_KEY_PARAMS);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGW(TAG, "defaults restored; they take effect after a reset");
    return CO_SDO_OK;
}

/* ---------------------------------------------------------------------- */
/* CiA 402 hardware hooks                                                  */
/* ---------------------------------------------------------------------- */

static void hw_power_stage(uint8_t axis, bool on, void *ctx)
{
    (void)ctx;
    motor_set_enable(axis, on);
}

static void hw_set_demand(uint8_t axis, int32_t demand, void *ctx)
{
    (void)ctx;
    motor_set_demand(axis, demand);
}

static void hw_fault_reset(uint8_t axis, void *ctx)
{
    (void)ctx;
    motor_clear_faults(axis);
    /* Clearing the drive fault also clears the emergencies that caused it,
     * which produces the "error reset" EMCY the master is waiting for. */
    co_emcy_clear(&s_node, CO_EMCY_CURRENT_OVERCURRENT);
    co_emcy_clear(&s_node, CO_EMCY_MANU_STALL);
    co_emcy_clear(&s_node, CO_EMCY_SENSOR_SPEED);
    co_emcy_clear(&s_node, CO_EMCY_VOLTAGE_UNDER);
    co_emcy_clear(&s_node, CO_EMCY_VOLTAGE_OVER);
    ESP_LOGI(TAG, "axis %u fault reset", axis);
}

static void hw_state_change(uint8_t axis, cia402_state_t from, cia402_state_t to, void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "axis %u: %s -> %s", axis, cia402_state_name(from), cia402_state_name(to));
}

static const cia402_hw_t s_hw = {
    .power_stage     = hw_power_stage,
    .set_demand      = hw_set_demand,
    .on_fault_reset  = hw_fault_reset,
    .on_state_change = hw_state_change,
};

/* ---------------------------------------------------------------------- */
/* CANopen callbacks                                                       */
/* ---------------------------------------------------------------------- */

/**
 * Apply object 6007h (abort connection option code) when the master goes
 * away, whether that shows up as a lost heartbeat, an NMT state change or a
 * silent RPDO.
 */
static void apply_abort_connection(unsigned i, const char *why)
{
    if (s_cmd_lost[i]) return;
    s_cmd_lost[i] = true;

    int16_t opt = g_od.axis[i].abort_connection_option;
    ESP_LOGW(TAG, "axis %u: %s, 6007h option %d", i, why, opt);

    switch (opt) {
    case 0: break;                                   /* no action           */
    case 1: cia402_fault(&s_axis[i]); break;          /* fault signal        */
    case 2: g_od.axis[i].controlword = 0x0002; break; /* quick stop          */
    case 3: g_od.axis[i].controlword = 0x0000; break; /* disable voltage     */
    default: g_od.axis[i].controlword = 0x0000; break;
    }
}

static void on_nmt(co_node_t *n, co_nmt_state_t state, void *ctx)
{
    (void)n; (void)ctx;
    bool operational = (state == CO_NMT_OPERATIONAL);
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        cia402_set_remote(&s_axis[i], operational);
        if (!operational) {
            /* RPDOs are not processed outside OPERATIONAL, so the drive must
             * not keep running on the last command it received. */
            apply_abort_connection(i, "NMT left OPERATIONAL");
        } else {
            s_cmd_lost[i] = false;
        }
    }
}

static void on_reset(co_node_t *n, bool full_reset, void *ctx)
{
    (void)n; (void)ctx;
    motor_emergency_stop();
    if (full_reset) {
        ESP_LOGW(TAG, "NMT reset node");
        vTaskDelay(pdMS_TO_TICKS(20));   /* let the response reach the bus */
        esp_restart();
    }
}

static void on_hb_lost(co_node_t *n, uint8_t node_id, void *ctx)
{
    (void)n; (void)ctx;
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        apply_abort_connection(i, "heartbeat lost");
    }
    ESP_LOGW(TAG, "heartbeat from node %u lost", node_id);
}

/** FOTA safety gate: no update while a power stage is live. */
static bool fota_safe(void)
{
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        if (s_axis[i].state == CIA402_OPERATION_ENABLED ||
            s_axis[i].state == CIA402_SWITCHED_ON ||
            s_axis[i].state == CIA402_QUICK_STOP_ACTIVE) {
            return false;
        }
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* Periodic work                                                           */
/* ---------------------------------------------------------------------- */

static void raise_axis_fault(unsigned i, uint16_t code, uint8_t reg)
{
    uint8_t info[5] = { (uint8_t)(i + 1), 0, 0, 0, 0 };
    co_st_u16(&info[1], (uint16_t)g_od.axis[i].current_actual);
    co_emcy_raise(&s_node, code, reg, info);
    cia402_fault(&s_axis[i]);
}

static void control_step(uint32_t dt_ms, uint32_t now_ms)
{
    motor_update(dt_ms);

    /* Supply voltage: only judged once something that looks like a battery is
     * connected, so bench-powering the board over USB does not fault it. */
    uint16_t mv = motor_supply_mv();
    bool have_supply = mv > 3000;
    if (have_supply) {
        if (mv < g_od.protect.undervoltage_mv) {
            co_emcy_raise(&s_node, CO_EMCY_VOLTAGE_UNDER, CO_ERRREG_VOLTAGE, NULL);
            for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) cia402_fault(&s_axis[i]);
        } else if (mv > g_od.protect.overvoltage_mv) {
            co_emcy_raise(&s_node, CO_EMCY_VOLTAGE_OVER, CO_ERRREG_VOLTAGE, NULL);
            for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) cia402_fault(&s_axis[i]);
        }
    }

    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        cia402_set_voltage_enabled(&s_axis[i], have_supply);

        uint32_t f = motor_faults((uint8_t)i);
        if (f & MOTOR_FAULT_OVERCURRENT) {
            raise_axis_fault(i, CO_EMCY_CURRENT_OVERCURRENT, CO_ERRREG_CURRENT);
        }
        if (f & MOTOR_FAULT_STALL) {
            raise_axis_fault(i, CO_EMCY_MANU_STALL, CO_ERRREG_MANUFACTURER);
        }
        if (f & MOTOR_FAULT_ENCODER) {
            raise_axis_fault(i, CO_EMCY_SENSOR_SPEED, CO_ERRREG_DEV_PROFILE);
        }

        /* Command watchdog: an RPDO that stops arriving is just as dangerous
         * as a lost heartbeat, and much more common in practice. */
        if (i < CO_NUM_RPDO && g_od.protect.cmd_timeout_ms &&
            s_node.nmt_state == CO_NMT_OPERATIONAL) {
            if (s_node.pdo.rpdo_seen[i]) {
                if (co_elapsed(now_ms, s_node.pdo.rpdo_last_ms[i]) > g_od.protect.cmd_timeout_ms) {
                    apply_abort_connection(i, "RPDO watchdog expired");
                } else {
                    s_cmd_lost[i] = false;
                }
            }
        }

        cia402_tick(&s_axis[i], dt_ms);
    }
}

static void housekeeping(uint32_t now_ms)
{
    uint32_t events = can_port_service(&g_od.diag);

    if (events & CAN_EVT_BUS_OFF) {
        co_emcy_raise(&s_node, CO_EMCY_CAN_PASSIVE, CO_ERRREG_COMMUNICATION, NULL);
        /* Bus-off means no command can reach us: stop, do not coast. */
        for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
            apply_abort_connection(i, "CAN bus-off");
        }
    }
    if (events & CAN_EVT_RECOVERED) {
        co_emcy_clear(&s_node, CO_EMCY_CAN_PASSIVE);
        co_emcy_raise(&s_node, CO_EMCY_CAN_RECOVERED_BUSOFF, CO_ERRREG_COMMUNICATION, NULL);
        co_emcy_clear(&s_node, CO_EMCY_CAN_RECOVERED_BUSOFF);
    }
    if (events & CAN_EVT_RX_OVERRUN) {
        co_emcy_raise(&s_node, CO_EMCY_CAN_OVERRUN, CO_ERRREG_COMMUNICATION, NULL);
        co_emcy_clear(&s_node, CO_EMCY_CAN_OVERRUN);
    }

    g_od.diag.uptime_s  = now_ms / 1000u;
    g_od.diag.free_heap = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT);

    /* Slow blink in pre-operational, fast when operational, solid on fault. */
    static bool led;
    bool fault = (g_od.error_register != 0);
    uint32_t period = fault ? 0 : (s_node.nmt_state == CO_NMT_OPERATIONAL ? 200 : 1000);
    if (period == 0) {
        led = true;
    } else if ((now_ms % period) < 100) {
        led = !led;
    }
    gpio_set_level((gpio_num_t)BOARD_STATUS_LED_GPIO, led);
}

/* ---------------------------------------------------------------------- */

static void canopen_task(void *arg)
{
    (void)arg;
    uint32_t last_control = 0, last_house = 0;

    for (;;) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        /* Drain the RX queue. The bound keeps one busy millisecond from
         * starving the control loop; anything left waits for the next tick. */
        co_msg_t m;
        for (int k = 0; k < 16 && can_port_receive(&m, 0); k++) {
            co_node_rx(&s_node, &m);
        }

        co_node_tick(&s_node, now);

        if (co_elapsed(now, last_control) >= BOARD_CONTROL_MS) {
            uint32_t dt = co_elapsed(now, last_control);
            last_control = now;
            control_step(dt, now);
        }

        if (co_elapsed(now, last_house) >= 100) {
            last_house = now;
            housekeeping(now);
        }

        fota_tick(now);

        vTaskDelay(pdMS_TO_TICKS(BOARD_TICK_MS));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    gpio_config_t led = {
        .pin_bit_mask = 1ULL << BOARD_STATUS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led);

    od_load_defaults();
    g_od.netcfg.node_id      = APP_NODE_ID_DEFAULT;
    g_od.netcfg.bitrate_kbps = APP_BITRATE_KBPS_DEFAULT;
    params_load();                       /* may override node-id and bit rate */

    /* A serial number that is actually unique, straight from the eFuse MAC. */
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    g_od.serial_number = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                         ((uint32_t)mac[4] << 8) | mac[5];

    uint8_t node_id = g_od.netcfg.node_id;
    if (node_id == 0 || node_id > 127) node_id = APP_NODE_ID_DEFAULT;
    od_apply_node_id(node_id);

    if (!od_check_sorted()) {
        /* A programming error in od_data.c: the binary search in
         * co_od_find() would misbehave silently. Fail loudly instead. */
        ESP_LOGE(TAG, "object dictionary is not sorted - refusing to start");
        abort();
    }

    fota_init();
    fota_set_safety_check(fota_safe);
    od_register_fota(fota_on_data, fota_on_command);
    od_register_store(on_store, on_restore);

    if (!motor_init()) {
        ESP_LOGE(TAG, "motor init failed");
        /* Do not confirm a freshly flashed image that cannot drive: letting
         * the confirm window expire rolls us back automatically. */
    }
    motor_emergency_stop();

    if (!can_port_init(g_od.netcfg.bitrate_kbps, false)) {
        /* Without the bus this node has no purpose and no way to be told
         * anything, so a panic-and-restart is the useful behaviour. It also
         * means a freshly flashed image that cannot bring up TWAI never
         * confirms itself and gets rolled back. */
        ESP_LOGE(TAG, "CAN init failed at %u kbit/s", (unsigned)g_od.netcfg.bitrate_kbps);
        abort();
    }

    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        cia402_init(&s_axis[i], &g_od.axis[i], (uint8_t)i, &s_hw);
    }

    co_node_cfg_t cfg = {
        .node_id    = node_id,
        .od         = &robocar_od,
        .send       = can_port_send,
        .ctx        = NULL,
        .on_nmt     = on_nmt,
        .on_reset   = on_reset,
        .on_hb_lost = on_hb_lost,
    };
    co_node_init(&s_node, &cfg);

    ESP_LOGI(TAG, "node-id %u (0x%02X), %u kbit/s, %u OD entries, fw %s",
             node_id, node_id, (unsigned)g_od.netcfg.bitrate_kbps,
             robocar_od.count, g_od.sw_version);

    xTaskCreatePinnedToCore(canopen_task, "canopen", 6144, NULL,
                            configMAX_PRIORITIES - 3, NULL, 0);
}
