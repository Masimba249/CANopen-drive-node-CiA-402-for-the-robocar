/**
 * @file test_cia402.c
 * @brief The CiA 402 state machine, transition by transition.
 *
 * This is the part interviewers probe, so every transition in the standard
 * diagram that this drive implements has a check here.
 */
#include "test_util.h"

static cia402_axis_t a;
static cia402_vars_t *v;

static int   s_power_calls;
static bool  s_powered;
static int32_t s_demand;
static int   s_fault_resets;

static void hw_power(uint8_t ax, bool on, void *ctx) { (void)ax; (void)ctx; s_powered = on; s_power_calls++; }
static void hw_demand(uint8_t ax, int32_t d, void *ctx) { (void)ax; (void)ctx; s_demand = d; }
static void hw_reset(uint8_t ax, void *ctx) { (void)ax; (void)ctx; s_fault_resets++; }

static const cia402_hw_t s_hw = {
    .power_stage = hw_power, .set_demand = hw_demand, .on_fault_reset = hw_reset,
};

static void setup(void)
{
    od_load_defaults();
    v = &g_od.axis[0];
    s_power_calls = 0; s_powered = false; s_demand = 0; s_fault_resets = 0;
    cia402_init(&a, v, 0, &s_hw);
    cia402_set_voltage_enabled(&a, true);
    cia402_set_remote(&a, true);
    cia402_tick(&a, 10);   /* T1: NOT READY -> SWITCH ON DISABLED */
}

/** Run the state machine for @p ms at a 10 ms control period. */
static void run(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 10) cia402_tick(&a, 10); }

void suite_cia402(void)
{
    SUITE("CiA 402 state machine");
    setup();

    /* --- T1 and the power-on state ----------------------------------- */
    CHECK_EQ(a.state, CIA402_SWITCH_ON_DISABLED);
    CHECK_EQ(v->statusword & 0x004F, 0x0040);
    CHECK(v->statusword & SW_VOLTAGE_ENABLED);
    CHECK(v->statusword & SW_REMOTE);
    CHECK(!s_powered);

    /* Statusword round-trips through the decoder the master uses. */
    CHECK_EQ(cia402_decode_status(v->statusword), CIA402_SWITCH_ON_DISABLED);

    /* --- T2: shutdown -> READY TO SWITCH ON --------------------------- */
    v->controlword = 0x0006;
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_READY_TO_SWITCH_ON);
    CHECK_EQ(v->statusword & 0x006F, 0x0021);
    CHECK(!s_powered);

    /* --- T3: switch on -> SWITCHED ON (power stage energised) --------- */
    v->controlword = 0x0007;
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_SWITCHED_ON);
    CHECK_EQ(v->statusword & 0x006F, 0x0023);
    CHECK(s_powered);
    CHECK_EQ(s_demand, 0);

    /* --- T4: enable operation -> OPERATION ENABLED -------------------- */
    v->controlword = 0x000F;
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_OPERATION_ENABLED);
    CHECK_EQ(v->statusword & 0x006F, 0x0027);
    CHECK_EQ(cia402_decode_status(v->statusword), CIA402_OPERATION_ENABLED);
    CHECK_EQ(v->mode_display, MODE_PROFILE_VELOCITY);

    /* --- ramp generator ------------------------------------------------
     * 6083h = 4000 units/s, control period 10 ms -> 40 units per tick. */
    v->target_velocity = 1000;
    cia402_tick(&a, 10);
    CHECK_EQ(v->velocity_demand, 40);
    cia402_tick(&a, 10);
    CHECK_EQ(v->velocity_demand, 80);
    run(1000);
    CHECK_EQ(v->velocity_demand, 1000);   /* reached and clamped to target */
    CHECK_EQ(s_demand, 1000);

    /* 607Fh limits the demand and raises the "internal limit active" bit. */
    v->target_velocity = 99999;
    run(2000);
    CHECK_EQ(v->velocity_demand, (int32_t)v->max_profile_velocity);
    CHECK(v->statusword & SW_INTERNAL_LIMIT);

    /* --- HALT (controlword bit 8) ramps to zero but stays enabled ----- */
    v->controlword = 0x010F;
    run(2000);
    CHECK_EQ(v->velocity_demand, 0);
    CHECK_EQ(a.state, CIA402_OPERATION_ENABLED);
    v->controlword = 0x000F;
    v->target_velocity = 500;
    run(1000);
    CHECK_EQ(v->velocity_demand, 500);

    /* --- target reached (bit 10) -------------------------------------- */
    v->velocity_actual = 500;
    run(100);
    CHECK(v->statusword & SW_TARGET_REACHED);
    v->velocity_actual = 0;
    cia402_tick(&a, 10);
    CHECK(!(v->statusword & SW_TARGET_REACHED));

    /* --- T5: disable operation -> SWITCHED ON ------------------------- */
    v->controlword = 0x0007;
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_SWITCHED_ON);
    CHECK_EQ(v->velocity_demand, 0);   /* demand dropped on leaving enabled */
    CHECK(s_powered);

    /* --- T4 again, then T11: quick stop ------------------------------- */
    v->controlword = 0x000F;
    v->target_velocity = 2000;
    run(1000);
    CHECK_EQ(a.state, CIA402_OPERATION_ENABLED);
    CHECK_EQ(v->velocity_demand, 2000);

    v->controlword = 0x0002;            /* quick stop: bit 2 cleared */
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_QUICK_STOP_ACTIVE);
    CHECK_EQ(v->statusword & 0x006F, 0x0007);
    /* 6085h = 20000 units/s -> 200 units per 10 ms tick. */
    CHECK_EQ(v->velocity_demand, 2000 - 200);

    /* --- T12: ramp finished, option code 2 -> SWITCH ON DISABLED ------ */
    run(200);
    CHECK_EQ(a.state, CIA402_SWITCH_ON_DISABLED);
    CHECK(!s_powered);

    /* --- option codes 5..7 keep the drive in QUICK STOP ACTIVE (T16) -- */
    setup();
    v->quick_stop_option = 6;
    v->controlword = 0x000F;            /* READY + SWITCHED ON + ENABLED */
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_SWITCH_ON_DISABLED);  /* needs shutdown first */
    v->controlword = 0x0006; cia402_tick(&a, 10);
    v->controlword = 0x000F; cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_OPERATION_ENABLED);   /* T3+T4 shortcut */

    v->target_velocity = 1000;
    run(1000);
    v->controlword = 0x0002;
    run(500);
    CHECK_EQ(a.state, CIA402_QUICK_STOP_ACTIVE);   /* stays put */
    CHECK_EQ(v->velocity_demand, 0);
    v->controlword = 0x000F;
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_OPERATION_ENABLED);   /* T16 */

    /* --- T9: disable voltage from OPERATION ENABLED ------------------- */
    v->controlword = 0x0000;
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_SWITCH_ON_DISABLED);

    /* --- T13/T14/T15: fault, fault reaction, reset -------------------- */
    setup();
    v->controlword = 0x0006; cia402_tick(&a, 10);
    v->controlword = 0x000F; cia402_tick(&a, 10);
    v->target_velocity = 1000;
    run(1000);
    CHECK_EQ(a.state, CIA402_OPERATION_ENABLED);

    cia402_fault(&a);                                   /* T13 */
    CHECK_EQ(a.state, CIA402_FAULT_REACTION_ACTIVE);
    cia402_tick(&a, 10);                                /* publish the new state */
    CHECK_EQ(a.state, CIA402_FAULT_REACTION_ACTIVE);    /* 6085h ramp still running */
    CHECK_EQ(v->statusword & 0x004F, 0x000F);
    CHECK(s_powered);                                   /* still braking */
    run(200);
    CHECK_EQ(a.state, CIA402_FAULT);                    /* T14 */
    CHECK_EQ(v->statusword & 0x004F, 0x0008);
    CHECK(!s_powered);

    /* A controlword that merely *has* bit 7 set does not reset: the
     * standard requires the 0 -> 1 edge. */
    v->controlword = 0x0080;
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_SWITCH_ON_DISABLED);       /* T15 on the edge */
    CHECK_EQ(s_fault_resets, 1);

    setup();
    v->controlword = 0x0080;            /* bit already set before the fault */
    cia402_tick(&a, 10);
    cia402_fault(&a);
    run(200);
    CHECK_EQ(a.state, CIA402_FAULT);
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_FAULT);    /* no edge -> still in FAULT */
    v->controlword = 0x0000; cia402_tick(&a, 10);
    v->controlword = 0x0080; cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_SWITCH_ON_DISABLED);

    /* --- fault reaction option 0 = coast, no ramp --------------------- */
    setup();
    v->fault_reaction_option = 0;
    v->controlword = 0x0006; cia402_tick(&a, 10);
    v->controlword = 0x000F; cia402_tick(&a, 10);
    v->target_velocity = 2000; run(2000);
    cia402_fault(&a);
    cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_FAULT);
    CHECK_EQ(v->velocity_demand, 0);

    /* --- an unsupported mode of operation reports 0 and does not move -- */
    setup();
    v->controlword = 0x0006; cia402_tick(&a, 10);
    v->controlword = 0x000F; cia402_tick(&a, 10);
    v->mode_of_operation = MODE_PROFILE_POSITION;
    v->target_velocity = 1000;
    run(1000);
    CHECK_EQ(v->mode_display, MODE_NO_MODE);
    CHECK_EQ(v->velocity_demand, 0);

    /* --- controlword decoding corner cases ----------------------------- */
    setup();
    /* 0x000E is "shutdown" because bit 3 is don't-care for that command. */
    v->controlword = 0x000E; cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_READY_TO_SWITCH_ON);
    /* Clearing "enable voltage" is disable-voltage regardless of other bits. */
    v->controlword = 0x000D; cia402_tick(&a, 10);
    CHECK_EQ(a.state, CIA402_SWITCH_ON_DISABLED);
}
