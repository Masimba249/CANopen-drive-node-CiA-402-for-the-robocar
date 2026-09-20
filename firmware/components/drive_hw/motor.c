/**
 * @file motor.c
 * @brief H-bridge PWM, quadrature feedback, PI velocity loop and protection.
 *
 * Units, once, so the arithmetic below is readable:
 *   velocity : 0.1 rpm at the wheel      (CiA 402 profile unit)
 *   position : 6063h raw encoder counts, 6064h milli-revolutions of the wheel
 *   current  : mA
 *   duty     : -1023..+1023, sign = direction
 */
#include "motor.h"
#include "board.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include <stdlib.h>

/* ADC_ATTEN_DB_11 was renamed ADC_ATTEN_DB_12 in ESP-IDF 5.2 (the hardware
 * never changed; the old name was simply wrong about the range). */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define ADC_ATTEN_FULL_RANGE ADC_ATTEN_DB_12
#else
#define ADC_ATTEN_FULL_RANGE ADC_ATTEN_DB_11
#endif

static const char *TAG = "motor";

#define DUTY_RES      LEDC_TIMER_10_BIT
#define DUTY_MAX      1023
#define PCNT_HIGH     30000
#define PCNT_LOW      (-30000)
#define ENC_FAULT_MS  1000

typedef struct {
    int in1_gpio, in2_gpio;
    ledc_channel_t ch1, ch2;
    int enc_a_gpio, enc_b_gpio;
    int isense_gpio;
} motor_pins_t;

static const motor_pins_t s_pins[ROBOCAR_NUM_AXES] = {
    { BOARD_M0_IN1_GPIO, BOARD_M0_IN2_GPIO, LEDC_CHANNEL_0, LEDC_CHANNEL_1,
      BOARD_ENC0_A_GPIO, BOARD_ENC0_B_GPIO, BOARD_ISENSE0_GPIO },
    { BOARD_M1_IN1_GPIO, BOARD_M1_IN2_GPIO, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
      BOARD_ENC1_A_GPIO, BOARD_ENC1_B_GPIO, BOARD_ISENSE1_GPIO },
};

typedef struct {
    pcnt_unit_handle_t pcnt;
    int32_t  last_count;
    int32_t  position;      /* accumulated counts, survives PCNT wraparound */
    int32_t  demand;
    int32_t  integral;      /* PI integrator, in duty counts x1000 */
    int32_t  duty;
    bool     enabled;
    uint32_t faults;
    uint32_t stall_ms;
    uint32_t no_motion_ms;
    adc_channel_t adc_ch;
} motor_ch_t;

static motor_ch_t s_m[ROBOCAR_NUM_AXES];
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_cali_ok;
static adc_channel_t s_vbat_ch;
static uint16_t s_supply_mv;

/* ---- low level ---------------------------------------------------------- */

static void apply_duty(uint8_t i, int32_t duty)
{
    const motor_pins_t *p = &s_pins[i];
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    if (duty < -DUTY_MAX) duty = -DUTY_MAX;

    uint32_t d1 = (duty > 0) ? (uint32_t)duty : 0;
    uint32_t d2 = (duty < 0) ? (uint32_t)(-duty) : 0;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, p->ch1, d1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, p->ch1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, p->ch2, d2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, p->ch2);
}

static bool setup_pwm(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = DUTY_RES,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = g_od.motor[0].pwm_freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&t) != ESP_OK) {
        ESP_LOGE(TAG, "PWM %u Hz is not reachable at %d-bit resolution",
                 (unsigned)t.freq_hz, 10);
        return false;
    }

    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        const ledc_channel_t chs[2] = { s_pins[i].ch1, s_pins[i].ch2 };
        const int gpios[2] = { s_pins[i].in1_gpio, s_pins[i].in2_gpio };
        for (int k = 0; k < 2; k++) {
            ledc_channel_config_t c = {
                .gpio_num   = gpios[k],
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel    = chs[k],
                .intr_type  = LEDC_INTR_DISABLE,
                .timer_sel  = LEDC_TIMER_0,
                .duty       = 0,
                .hpoint     = 0,
            };
            if (ledc_channel_config(&c) != ESP_OK) return false;
        }
    }
    return true;
}

static bool setup_encoder(unsigned i)
{
    /* accum_count + watch points at the limits makes the driver keep a
     * 32-bit total for us, so a fast wheel cannot silently wrap a 16-bit
     * hardware counter between control periods. */
    pcnt_unit_config_t uc = {
        .high_limit = PCNT_HIGH,
        .low_limit  = PCNT_LOW,
        .flags.accum_count = 1,
    };
    if (pcnt_new_unit(&uc, &s_m[i].pcnt) != ESP_OK) return false;

    pcnt_glitch_filter_config_t gf = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(s_m[i].pcnt, &gf);

    /* Full x4 quadrature decoding: both edges of both channels. */
    pcnt_chan_config_t ca = {
        .edge_gpio_num  = s_pins[i].enc_a_gpio,
        .level_gpio_num = s_pins[i].enc_b_gpio,
    };
    pcnt_channel_handle_t cha = NULL, chb = NULL;
    if (pcnt_new_channel(s_m[i].pcnt, &ca, &cha) != ESP_OK) return false;
    pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                      PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                       PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_chan_config_t cb = {
        .edge_gpio_num  = s_pins[i].enc_b_gpio,
        .level_gpio_num = s_pins[i].enc_a_gpio,
    };
    if (pcnt_new_channel(s_m[i].pcnt, &cb, &chb) != ESP_OK) return false;
    pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                      PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                       PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_add_watch_point(s_m[i].pcnt, PCNT_HIGH);
    pcnt_unit_add_watch_point(s_m[i].pcnt, PCNT_LOW);
    pcnt_unit_enable(s_m[i].pcnt);
    pcnt_unit_clear_count(s_m[i].pcnt);
    pcnt_unit_start(s_m[i].pcnt);
    return true;
}

static bool setup_adc(void)
{
    adc_oneshot_unit_init_cfg_t uc = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&uc, &s_adc) != ESP_OK) return false;

    adc_oneshot_chan_cfg_t cc = {
        .atten    = ADC_ATTEN_FULL_RANGE,   /* ~0..3.1 V full scale */
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    adc_unit_t unit;
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        if (adc_oneshot_io_to_channel(s_pins[i].isense_gpio, &unit, &s_m[i].adc_ch) != ESP_OK) {
            ESP_LOGE(TAG, "GPIO%d is not an ADC1 pin", s_pins[i].isense_gpio);
            return false;
        }
        adc_oneshot_config_channel(s_adc, s_m[i].adc_ch, &cc);
    }
    if (adc_oneshot_io_to_channel(BOARD_VBAT_GPIO, &unit, &s_vbat_ch) != ESP_OK) return false;
    adc_oneshot_config_channel(s_adc, s_vbat_ch, &cc);

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t lf = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_FULL_RANGE,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    s_cali_ok = (adc_cali_create_scheme_line_fitting(&lf, &s_cali) == ESP_OK);
#endif
    if (!s_cali_ok) {
        ESP_LOGW(TAG, "no ADC calibration: currents and voltage are approximate");
    }
    return true;
}

static int read_mv(adc_channel_t ch)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc, ch, &raw) != ESP_OK) return 0;
    int mv = 0;
    if (s_cali_ok && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) return mv;
    /* Fallback: 12-bit over the 12 dB attenuation range. */
    return (raw * 3100) / 4095;
}

/* ---- public ------------------------------------------------------------- */

bool motor_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOARD_MOTOR_STBY_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level((gpio_num_t)BOARD_MOTOR_STBY_GPIO, 0);   /* bridges off */

    if (!setup_pwm()) return false;
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        if (!setup_encoder(i)) {
            ESP_LOGE(TAG, "encoder %u init failed", i);
            return false;
        }
        apply_duty((uint8_t)i, 0);
    }
    if (!setup_adc()) return false;

    ESP_LOGI(TAG, "motors ready: PWM %u Hz, %u cpr",
             (unsigned)g_od.motor[0].pwm_freq_hz, (unsigned)g_od.motor[0].encoder_cpr);
    return true;
}

bool motor_reconfigure(void)
{
    return setup_pwm();
}

void motor_set_enable(uint8_t axis, bool on)
{
    if (axis >= ROBOCAR_NUM_AXES) return;
    s_m[axis].enabled = on;
    if (!on) {
        s_m[axis].demand = 0;
        s_m[axis].integral = 0;
        s_m[axis].duty = 0;
        apply_duty(axis, 0);
    }
    /* The TB6612 standby pin gates both bridges, so it follows the OR. */
    bool any = false;
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) any = any || s_m[i].enabled;
    gpio_set_level((gpio_num_t)BOARD_MOTOR_STBY_GPIO, any ? 1 : 0);
}

void motor_set_demand(uint8_t axis, int32_t demand)
{
    if (axis >= ROBOCAR_NUM_AXES) return;
    s_m[axis].demand = demand;
}

void motor_emergency_stop(void)
{
    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        s_m[i].enabled = false;
        s_m[i].demand = 0;
        s_m[i].duty = 0;
        s_m[i].integral = 0;
        apply_duty((uint8_t)i, 0);
    }
    gpio_set_level((gpio_num_t)BOARD_MOTOR_STBY_GPIO, 0);
}

uint32_t motor_faults(uint8_t axis)
{
    return (axis < ROBOCAR_NUM_AXES) ? s_m[axis].faults : 0;
}

void motor_clear_faults(uint8_t axis)
{
    if (axis >= ROBOCAR_NUM_AXES) return;
    s_m[axis].faults = 0;
    s_m[axis].stall_ms = 0;
    s_m[axis].no_motion_ms = 0;
}

uint16_t motor_supply_mv(void) { return s_supply_mv; }

/* ---- control loop -------------------------------------------------------- */

static int32_t measure_velocity(unsigned i, int32_t delta_counts, uint32_t dt_ms)
{
    const od_motor_cfg_t *cfg = &g_od.motor[i];
    if (!cfg->use_encoder || cfg->encoder_cpr == 0) {
        /* Open-loop estimate from the applied duty cycle. Good enough to
         * publish a plausible 606Ch when no encoder is fitted; documented as
         * such in 2000h:05. */
        return (int32_t)(((int64_t)s_m[i].duty * cfg->no_load_speed) / DUTY_MAX);
    }
    if (dt_ms == 0) return g_od.axis[i].velocity_actual;

    /* counts -> 0.1 rpm:  (delta / cpr) rev * (60000 / dt_ms) min^-1 * 10 */
    int64_t v = ((int64_t)delta_counts * 600000LL) / ((int64_t)cfg->encoder_cpr * (int64_t)dt_ms);
    return (int32_t)v;
}

static int32_t run_pi(unsigned i, int32_t demand, int32_t actual, uint32_t dt_ms)
{
    const od_motor_cfg_t *cfg = &g_od.motor[i];

    /* Feed-forward first: the loop then only has to correct the error, which
     * keeps the integrator small and the response quick. */
    int32_t ff = 0;
    if (cfg->no_load_speed) {
        ff = (int32_t)(((int64_t)demand * DUTY_MAX) / cfg->no_load_speed);
    }
    if (!cfg->use_encoder) return ff;

    int32_t err = demand - actual;
    int32_t p = (int32_t)(((int64_t)cfg->kp_x1000 * err) / 1000);

    int32_t out = ff + p + s_m[i].integral / 1000;

    /* Conditional integration: stop winding up once the output saturates. */
    bool saturated = (out > DUTY_MAX && err > 0) || (out < -DUTY_MAX && err < 0);
    if (!saturated) {
        int64_t inc = ((int64_t)cfg->ki_x1000 * err * (int64_t)dt_ms) / 1000;
        s_m[i].integral = (int32_t)(s_m[i].integral + inc);
        int32_t lim = DUTY_MAX * 1000;
        if (s_m[i].integral > lim) s_m[i].integral = lim;
        if (s_m[i].integral < -lim) s_m[i].integral = -lim;
        out = ff + p + s_m[i].integral / 1000;
    }

    if (out > DUTY_MAX) out = DUTY_MAX;
    if (out < -DUTY_MAX) out = -DUTY_MAX;
    return out;
}

static void check_protection(unsigned i, int32_t actual, int16_t current_ma, uint32_t dt_ms)
{
    const od_protect_t *pr = &g_od.protect;

    if (current_ma > (int16_t)pr->overcurrent_ma) {
        if (!(s_m[i].faults & MOTOR_FAULT_OVERCURRENT)) {
            ESP_LOGE(TAG, "axis %u over-current: %d mA", i, current_ma);
        }
        s_m[i].faults |= MOTOR_FAULT_OVERCURRENT;
    }

    bool commanded = abs(s_m[i].demand) > (int32_t)g_od.axis[i].velocity_threshold;
    bool stopped   = abs(actual) <= (int32_t)g_od.axis[i].velocity_threshold;

    if (s_m[i].enabled && commanded && stopped) {
        s_m[i].no_motion_ms += dt_ms;
        if (current_ma >= (int16_t)pr->stall_current_ma) {
            s_m[i].stall_ms += dt_ms;
            if (s_m[i].stall_ms >= pr->stall_time_ms) {
                if (!(s_m[i].faults & MOTOR_FAULT_STALL)) {
                    ESP_LOGE(TAG, "axis %u stalled: %d mA, demand %d",
                             i, current_ma, (int)s_m[i].demand);
                }
                s_m[i].faults |= MOTOR_FAULT_STALL;
            }
        } else {
            s_m[i].stall_ms = 0;
            /* Commanded, drawing current normally, but the counter never
             * moves: that is a broken encoder, not a stalled rotor. */
            if (g_od.motor[i].use_encoder && s_m[i].no_motion_ms >= ENC_FAULT_MS &&
                abs(s_m[i].duty) > DUTY_MAX / 4) {
                s_m[i].faults |= MOTOR_FAULT_ENCODER;
            }
        }
    } else {
        s_m[i].stall_ms = 0;
        s_m[i].no_motion_ms = 0;
    }
}

void motor_update(uint32_t dt_ms)
{
    s_supply_mv = (uint16_t)((read_mv(s_vbat_ch) * 1000) / BOARD_VBAT_DIV_X1000);
    g_od.analog.supply_mv = s_supply_mv;

    for (unsigned i = 0; i < ROBOCAR_NUM_AXES; i++) {
        const od_motor_cfg_t *cfg = &g_od.motor[i];
        cia402_vars_t *v = &g_od.axis[i];

        /* --- encoder --- */
        int count = 0;
        pcnt_unit_get_count(s_m[i].pcnt, &count);
        int32_t delta = (int32_t)count - s_m[i].last_count;
        s_m[i].last_count = (int32_t)count;
        if (cfg->invert) delta = -delta;
        s_m[i].position += delta;

        int32_t actual = measure_velocity(i, delta, dt_ms);

        v->position_internal = s_m[i].position;
        v->position_actual   = (cfg->encoder_cpr)
                                 ? (int32_t)(((int64_t)s_m[i].position * 1000) / cfg->encoder_cpr)
                                 : 0;
        v->velocity_actual = actual;

        /* --- current --- */
        int mv = read_mv(s_m[i].adc_ch) - BOARD_ISENSE_OFFSET_MV;
        if (mv < 0) mv = 0;
        int16_t ma = (int16_t)(((int64_t)mv * 1000) / BOARD_ISENSE_MV_PER_A);
        g_od.analog.current_ma[i] = ma;
        v->current_actual = ma;
        /* 6077h is per mille of rated torque; rated current is 2002h:02. */
        v->torque_actual = g_od.protect.stall_current_ma
                             ? (int16_t)(((int32_t)ma * 1000) / g_od.protect.stall_current_ma)
                             : 0;

        /* --- control --- */
        int32_t duty = 0;
        if (s_m[i].enabled) {
            duty = run_pi(i, s_m[i].demand, actual, dt_ms);
        } else {
            s_m[i].integral = 0;
        }
        s_m[i].duty = duty;
        apply_duty((uint8_t)i, cfg->invert ? -duty : duty);

        check_protection(i, actual, ma, dt_ms);
    }
}
