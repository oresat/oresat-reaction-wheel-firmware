#include "self_test.h"

#include <stdarg.h>

#include "aux_adc.h"
#include "bootloader.h"
#include "config.h"
#include "console.h"
#include "crash_record.h"
#include "encoder.h"
#include "inverter.h"
#include "ma702.h"
#include "mathutil.h"
#include "OD.h"
#include "periphmap.h"
#include "printf.h"
#include "pwm.h"
#include "util.h"

/*
 * FSM_STATE_SELF_TEST — see self_test.h for the shape, docs/control-laws.md for the per-check
 * reference, docs/board-bringup.md for the operator view.
 *
 * Thresholds are first guesses from the design, NOT bench-calibrated yet (2026-11-17). The
 * phase-voltage fractions depend on where the injected rank-2 (PHx_V) conversion lands in the
 * PWM period: at 5 % duty the high pulse is ±0.26 µs around the counter crest, at 95 % the low
 * pulse is ±0.26 µs around the valley; rank 2 converts after the rank-1 current burst, so 5 %
 * should read near 0 and 95 % near Vbus whether or not the divider is RC-filtered.
 */

// ---------------------------------------------------------------- tunables

#define ST_MS_TO_TICKS(ms) ((u32) ((ms) * LOOP_UPDATE_FREQ / 1000.f))
#define ST_FLOAT_SETTLE_TICKS ST_MS_TO_TICKS(2.f)   // after floating the bridge
#define ST_FLOAT_MEAS_TICKS   ST_MS_TO_TICKS(100.f) // bridge-off measurement window
#define ST_DRIVE_SETTLE_TICKS ST_MS_TO_TICKS(2.f)   // after a duty step (bootstrap, filters)
#define ST_DRIVE_MEAS_TICKS   ST_MS_TO_TICKS(10.f)  // per drive step

// All three legs share one duty per step: zero volts across a motor, so no current flows.
static const f32 ST_DUTY_LO = 0.05f;
static const f32 ST_DUTY_HI = 0.95f;
static const f32 ST_LO_FRAC_MAX = 0.25f;        // phase V / Vbus at 5 % must be below this
static const f32 ST_HI_FRAC_MIN = 0.65f;        // ... at 95 % must be above this
// Bridge off, per phase: back-EMF shows as swing (max - min); a leak to the bus (shorted high
// side) as a high mean. A static offset below the leak level is allowed: with all FETs off the
// high-side driver's bias current flows through the 10.9 kOhm PHx_V divider (size unknown).
static const f32 ST_FLOAT_SWING_MAX_V = 0.5f;   // V p-p. Above: rotor turning (~25 rev/s+)
static const f32 ST_FLOAT_LEAK_FRAC = 0.3f;     // mean / Vbus. Above: phase pulled to the bus
static const f32 ST_ENC_SPIN_MAX_REV_S = 2.f;   // encoder-seen rotation that blocks driving
static const f32 ST_ABORT_CURRENT_A = 5.f;      // 95 % step: float the bridge above this
static const f32 ST_HI_CURRENT_MAX_A = 1.f;     // 95 % step: more than this is a FAIL
static const f32 ST_SAG_ABORT_FRAC = 0.80f;     // drive steps: float the bridge below this × Vbus
static const f32 ST_SAG_FAIL_FRAC = 0.95f;      // ... and FAIL below this
static const f32 ST_VBUS_RIPPLE_WARN_V = 0.5f;  // bridge off, max - min
static const f32 ST_ADC_MID_LIMIT_mV = 50.f;    // same bound as the boot offset cal
static const u32 ST_VREF_MIN_mV = 2900u;        // VREF+ is a 3.0 V reference on rev 8.1.x
static const u32 ST_VREF_MAX_mV = 3100u;
static const u32 ST_VREF_ABS_MIN_mV = 2500u;    // outside 2.5..3.4 V the firmware falls back
static const u32 ST_VREF_ABS_MAX_mV = 3400u;    // to nominal and every reading is off
static const i32 ST_TEMP_MIN_cC = -2000;        // plausible bench temperatures
static const i32 ST_TEMP_MAX_cC = 8500;
static const i32 ST_TEMP_SPREAD_cC = 1500;      // board thermistor vs median of the four
static const u32 ST_STRAP_MARGIN = 50u;         // of 1000 votes: closer to 500 = marginal strap
static const f32 ST_LOOP_WARN_FRAC = 0.90f;     // fast-loop high-water vs budget

// ------------------------------------------------------------ bridge half (fast loop)

typedef enum {
    ST_PH_FLOAT = 0, // bridge off, measuring
    ST_PH_LO,        // all legs at ST_DUTY_LO
    ST_PH_HI,        // all legs at ST_DUTY_HI
    ST_PH_DONE,      // bridge off, law complete
} st_phase_t;

typedef enum {
    ST_BRIDGE_NOT_RUN = 0,   // report-only run (boot shunt-offset cal failed)
    ST_BRIDGE_DONE,          // both drive steps measured
    ST_BRIDGE_SKIP_MOVING,   // bridge-off window: back-EMF swing, encoder rotation, or a leak
    ST_BRIDGE_ABORT_CURRENT, // current above ST_ABORT_CURRENT_A during a drive step
    ST_BRIDGE_ABORT_SAG,     // Vbus below ST_SAG_ABORT_FRAC during a drive step
    ST_BRIDGE_CANCELLED,     // left early: host request or controller error
} st_bridge_outcome_t;

static const char *const st_phase_names[] = {"bridge-off", "5 % duty", "95 % duty", "done"};

// Written by the fast loop while armed; read by the report after disarm (self_test_exit is
// the handoff). volatile because three contexts touch it across the run.
static volatile struct {
    st_phase_t phase;
    u32 tick; // ticks in the current phase

    // bridge-off window
    u32 f_n;
    f32 f_raw_sum[3]; // shunt ADC counts (12-bit)
    f32 f_vbus_sum, f_vbus_min, f_vbus_max, f_vbus_mean;
    f32 f_phv_min[3], f_phv_max[3], f_phv_sum[3]; // V
    u32 f_enc_n;
    u16 f_enc_prev, f_enc_min, f_enc_max;
    i32 f_enc_net;  // sum of wrapped deltas [counts]
    u32 f_enc_path; // sum of |wrapped deltas| [counts]

    // drive steps: [0] = 5 %, [1] = 95 %
    u32 d_n[2];
    f32 d_frac_sum[2][3]; // phase V / Vbus
    f32 d_vbus_min[2];
    f32 d_idm_max[2]; // max |differential-mode phase current| [A]

    st_bridge_outcome_t outcome;
    st_phase_t abort_phase;
    f32 abort_value;
} st;

static volatile bool st_bridge_planned = false;
// Shunt selection is off for the run: with all three duties equal it always rebuilds phase A
// as -(B+C), which turns an edge kick common to all three shunts into 2x "current" for the
// fast loop's trip. Plain common-mode removal cancels it. Restored in self_test_exit().
static volatile bool st_shunt_sel_saved = false;
static volatile bool st_shunt_sel_prev = false;
static volatile bool st_report_pending = false;
static volatile u32 st_exit_errors = 0;
static volatile bool st_exit_cancelled = false;

static struct {
    u8 wheel, node_id;
    u32 addr0_high, addr1_high, rcc_csr;
} st_boot;

static void st_reset() {
    st.phase = ST_PH_FLOAT;
    st.tick = 0;
    st.f_n = 0;
    st.f_vbus_sum = 0.f;
    st.f_vbus_min = 1e9f;
    st.f_vbus_max = 0.f;
    st.f_vbus_mean = 0.f;
    st.f_enc_n = 0;
    st.f_enc_prev = 0;
    st.f_enc_min = 0;
    st.f_enc_max = 0;
    st.f_enc_net = 0;
    st.f_enc_path = 0;
    for (u32 x = 0; x < 3; x++) {
        st.f_raw_sum[x] = 0.f;
        st.f_phv_min[x] = 1e9f;
        st.f_phv_max[x] = 0.f;
        st.f_phv_sum[x] = 0.f;
    }
    for (u32 s = 0; s < 2; s++) {
        st.d_n[s] = 0;
        st.d_vbus_min[s] = 1e9f;
        st.d_idm_max[s] = 0.f;
        for (u32 x = 0; x < 3; x++)
            st.d_frac_sum[s][x] = 0.f;
    }
    st.outcome = ST_BRIDGE_NOT_RUN;
    st.abort_phase = ST_PH_FLOAT;
    st.abort_value = 0.f;
}

static __ccmram void st_enter(st_phase_t p) {
    if (p == ST_PH_DONE)
        hrtim_outputs_disable();
    st.phase = p;
    st.tick = 0;
}

static __ccmram void st_abort(st_bridge_outcome_t why, f32 value) {
    hrtim_outputs_disable(); // first: lock-free ODISR write
    st.outcome = why;
    st.abort_phase = st.phase;
    st.abort_value = value;
    st_enter(ST_PH_DONE);
}

// Encoder rotation seen in the bridge-off window [rev/s]. 0 unless the motion is consistent:
// with no magnet the MA732 returns noise whose wrapped deltas random-walk, so require the net
// motion to be at least half the path length.
static f32 st_float_enc_vel() {
    if (st.f_enc_n < 2u || st.f_enc_path == 0u)
        return 0.f;
    f32 net = (f32) st.f_enc_net;
    if (abs_f32(net) < 0.5f * (f32) st.f_enc_path)
        return 0.f;
    f32 seconds = (f32) st.f_enc_n / LOOP_UPDATE_FREQ;
    return net / (f32) ENCODER_CPR / seconds;
}

static bool st_float_swing(u32 x) {
    return st.f_phv_max[x] - st.f_phv_min[x] > ST_FLOAT_SWING_MAX_V;
}

static bool st_float_leak(u32 x) {
    // Vbus from the running sum, not f_vbus_mean: that is only set when the window ends, and a
    // run cut short mid-window (error, host cancel) would compare against 0.
    if (st.f_n == 0u)
        return false;
    f32 n = (f32) st.f_n;
    return st.f_phv_sum[x] / n > ST_FLOAT_LEAK_FRAC * (st.f_vbus_sum / n);
}

// Not safe to drive: the rotor turns (equal duty on all legs would short its back-EMF), or a
// phase sits near the bus with all FETs off (a shorted high side: the 5 % step would shoot through).
static bool st_float_says_unsafe() {
    for (u32 x = 0; x < 3; x++)
        if (st_float_swing(x) || st_float_leak(x))
            return true;
    return abs_f32(st_float_enc_vel()) > ST_ENC_SPIN_MAX_REV_S;
}

static __ccmram bool st_encoder_consumer(u16 enc) {
    encoder_update(enc); // keep the PLL (and TPDO velocity) alive while armed
    if (st.phase != ST_PH_FLOAT || st.tick < ST_FLOAT_SETTLE_TICKS)
        return true;
    if (st.f_enc_n == 0u) {
        st.f_enc_min = enc;
        st.f_enc_max = enc;
    } else {
        i32 d = (i32) enc - (i32) st.f_enc_prev;
        if (d > (i32) (ENCODER_CPR / 2u))
            d -= (i32) ENCODER_CPR;
        else if (d < -(i32) (ENCODER_CPR / 2u))
            d += (i32) ENCODER_CPR;
        st.f_enc_net += d;
        st.f_enc_path += (u32) (d < 0 ? -d : d);
        if (enc < st.f_enc_min) st.f_enc_min = enc;
        if (enc > st.f_enc_max) st.f_enc_max = enc;
    }
    st.f_enc_prev = enc;
    st.f_enc_n++;
    return true;
}

static __ccmram bool st_current_consumer(f32 A_i, f32 B_i, f32 C_i, f32 Vbus) {
    (void) A_i; // CM-removed / shunt-selected: unusable when all three edges share a duty
    (void) B_i;
    (void) C_i;
    u32 raw[3], vbus_raw, phv[3];
    inverter_get_adc_raw(&raw[0], &raw[1], &raw[2], &vbus_raw);
    inverter_get_phase_voltages_raw(&phv[0], &phv[1], &phv[2]);
    // Phase V uses the same divider as Vbus (config.h PHASE_V_COUNTS_TO_VOLTS), so the raw ratio
    // is the fraction of the bus — no VREF or divider value involved.
    f32 inv_vbus_raw = (vbus_raw != 0u) ? 1.f / (f32) vbus_raw : 0.f;

    if (st.phase == ST_PH_FLOAT) {
        if (st.tick < ST_FLOAT_SETTLE_TICKS)
            return true;
        st.f_n++;
        st.f_vbus_sum += Vbus;
        if (Vbus < st.f_vbus_min) st.f_vbus_min = Vbus;
        if (Vbus > st.f_vbus_max) st.f_vbus_max = Vbus;
        for (u32 x = 0; x < 3; x++) {
            st.f_raw_sum[x] += (f32) raw[x];
            f32 v = (f32) phv[x] * inv_vbus_raw * Vbus;
            st.f_phv_sum[x] += v;
            if (v < st.f_phv_min[x]) st.f_phv_min[x] = v;
            if (v > st.f_phv_max[x]) st.f_phv_max[x] = v;
        }
        return true;
    }

    if (st.phase != ST_PH_LO && st.phase != ST_PH_HI)
        return true;
    // tick 0 of LO: outputs not enabled yet; tick 0 of HI: sample still from the 5 % duty
    if (st.tick < 1u)
        return true;

    // Differential-mode current from the raw samples: subtract the three-phase mean, so a
    // switching artifact common to all three (all edges share one duty here) cancels.
    i16 off[3];
    inverter_get_offset_correction(&off[0], &off[1], &off[2]);
    f32 vvcc = (f32) aux_adc_conv_vdd_voltage_mV(aux_adc_last_reading_raw.vvcc_sns) * 0.001f;
    f32 k = vvcc / 4095.f * SHUNT_AMP_CONDUCTANCE;
    f32 i_ph[3];
    for (u32 x = 0; x < 3; x++)
        i_ph[x] = (f32) ((i32) raw[x] - (i32) off[x] - 2048) * k;
    f32 cm = (i_ph[0] + i_ph[1] + i_ph[2]) * (1.f / 3.f);
    f32 idm = 0.f;
    for (u32 x = 0; x < 3; x++) {
        f32 a = abs_f32(i_ph[x] - cm);
        if (a > idm) idm = a;
    }

    // Current abort only in the 95 % step: at 5 % every switching edge sits within ±0.26 µs of
    // the crest, inside the current-sample burst, so a per-phase difference in the INA296 kick
    // reads as current there. The 5 % step stays covered by the sag abort below and by the fast
    // loop's own MOTOR_CURRENT_TRIP_LIMIT trip.
    if (st.phase == ST_PH_HI && idm > ST_ABORT_CURRENT_A) {
        st_abort(ST_BRIDGE_ABORT_CURRENT, idm);
        return true;
    }
    if (Vbus < ST_SAG_ABORT_FRAC * st.f_vbus_mean) {
        st_abort(ST_BRIDGE_ABORT_SAG, Vbus);
        return true;
    }

    if (st.tick >= ST_DRIVE_SETTLE_TICKS && st.tick < ST_DRIVE_SETTLE_TICKS + ST_DRIVE_MEAS_TICKS) {
        u32 s = (st.phase == ST_PH_LO) ? 0u : 1u;
        st.d_n[s]++;
        for (u32 x = 0; x < 3; x++)
            st.d_frac_sum[s][x] += (f32) phv[x] * inv_vbus_raw;
        if (Vbus < st.d_vbus_min[s]) st.d_vbus_min[s] = Vbus;
        if (idm > st.d_idm_max[s]) st.d_idm_max[s] = idm;
    }
    return true;
}

static __ccmram void st_exec() {
    u32 t = ++st.tick;
    switch (st.phase) {
        case ST_PH_FLOAT:
            if (t >= ST_FLOAT_SETTLE_TICKS + ST_FLOAT_MEAS_TICKS) {
                st.f_vbus_mean = (st.f_n != 0u) ? st.f_vbus_sum / (f32) st.f_n : 0.f;
                if (st.f_n == 0u) {
                    st.outcome = ST_BRIDGE_CANCELLED; // no ADC data: don't drive blind
                    st_enter(ST_PH_DONE);
                } else if (st_float_says_unsafe()) {
                    st.outcome = ST_BRIDGE_SKIP_MOVING;
                    st_enter(ST_PH_DONE);
                } else {
                    // The producer below writes the 5 % duty at the end of this tick; it is
                    // live after the next repetition event, where tick 1 enables the outputs.
                    st_enter(ST_PH_LO);
                }
            }
            break;
        case ST_PH_LO:
            if (t == 1u && controller_is_armed()) // not if this tick's ADC/encoder pass disarmed
                hrtim_outputs_enable();
            if (t >= ST_DRIVE_SETTLE_TICKS + ST_DRIVE_MEAS_TICKS)
                st_enter(ST_PH_HI);
            break;
        case ST_PH_HI:
            if (t >= ST_DRIVE_SETTLE_TICKS + ST_DRIVE_MEAS_TICKS) {
                st.outcome = ST_BRIDGE_DONE;
                st_enter(ST_PH_DONE);
            }
            break;
        case ST_PH_DONE:
        default:
            break;
    }
}

static __ccmram bool st_pwm_producer(svm_vec_t *out) {
    f32 d = 0.5f;
    if (st.phase == ST_PH_LO)
        d = ST_DUTY_LO;
    else if (st.phase == ST_PH_HI)
        d = ST_DUTY_HI;
    *out = (svm_vec_t) {d, d, d};
    return true;
}

static __ccmram bool st_is_complete() {
    return st.phase == ST_PH_DONE;
}

static void st_init() {
    hrtim_outputs_disable(); // float before anything else this tick
    st_reset();
}

static bool st_finish(bool cancelled) {
    (void) cancelled;
    return false; // the FSM routes SELF_TEST through self_test_exit, not this
}

control_law_t self_test_ctrl_law = {
    .encoder_consumer = st_encoder_consumer,
    .current_consumer = st_current_consumer,
    .pwm_producer = st_pwm_producer,
    .control_law_exec = st_exec,
    .is_complete = st_is_complete,
    .control_law_init = st_init,
    .control_law_finish = st_finish,
};

// ------------------------------------------------------------------ FSM hooks

void self_test_set_boot_info(u8 wheel_number, u8 node_id, u32 addr0_high, u32 addr1_high,
                             u32 rcc_csr) {
    st_boot.wheel = wheel_number;
    st_boot.node_id = node_id;
    st_boot.addr0_high = addr0_high;
    st_boot.addr1_high = addr1_high;
    st_boot.rcc_csr = rcc_csr;
}

bool self_test_prepare() {
    st_reset();
    st_exit_errors = 0;
    st_exit_cancelled = false;
    st_bridge_planned = inverter_get_calibrated();
    if (st_bridge_planned && !st_shunt_sel_saved) {
        st_shunt_sel_prev = shunt_selection_enabled;
        st_shunt_sel_saved = true;
        shunt_selection_enabled = false;
    }
    return st_bridge_planned;
}

void self_test_after_arm() {
    // controller_arm() just enabled the outputs at 50 %; the law's init only runs on the first
    // fast-loop tick (up to 31 µs later). Float now so a turning rotor isn't shorted meanwhile.
    hrtim_outputs_disable();
}

void self_test_exit(bool cancelled) {
    if (st_bridge_planned && st.phase != ST_PH_DONE) {
        st.outcome = ST_BRIDGE_CANCELLED;
        st.abort_phase = st.phase;
    }
    st_exit_errors = controller_get_errors();
    st_exit_cancelled = cancelled;
    if (st_shunt_sel_saved) {
        shunt_selection_enabled = st_shunt_sel_prev; // disarmed by now: no tick reads it mid-change
        st_shunt_sel_saved = false;
    }
    st_report_pending = true;
}

bool self_test_report_pending() {
    return st_report_pending;
}

void self_test_report_clear() {
    st_report_pending = false;
}

// ------------------------------------------------------------ report half (foreground)

typedef enum { V_PASS = 0, V_FAIL, V_WARN, V_SKIP, V_INFO } st_verdict_t;

static u32 st_n_fail, st_n_warn;

static void st_line(st_verdict_t v, const char *id, const char *fmt, ...) _ATTR_PRINTF(3, 4);

// One parseable line: "SELFTEST <VERDICT> <id>: <text>". Thread context → synchronous UART.
// Built in one buffer and printed with ONE printf: every thread-context printf first pumps the
// ISR ring, so a multi-call line could get an FSM message spliced into its middle.
static void st_line(st_verdict_t v, const char *id, const char *fmt, ...) {
    static const char *const names[] = {"PASS", "FAIL", "WARN", "SKIP", "INFO"};
    static char line[CONSOLE_LINE_MAX - 2u]; // + "\r\n" = the console's line limit
    if (v == V_FAIL) st_n_fail++;
    if (v == V_WARN) st_n_warn++;
    int n = snprintf(line, sizeof(line), "SELFTEST %s %s: ", names[v], id);
    if (n > 0 && (u32) n < sizeof(line)) {
        va_list va;
        va_start(va, fmt);
        vsnprintf(line + n, sizeof(line) - (u32) n, fmt, va);
        va_end(va);
    }
    printf("%s\r\n", line);
}

static f32 st_counts_to_mV(f32 counts) {
    return counts * (f32) aux_adc_conv_vdd_voltage_mV(aux_adc_last_reading_raw.vvcc_sns) / 4095.f;
}

static void st_check_clock() {
    bool hse_rdy = (RCC->CR & RCC_CR_HSERDY) != 0u;
    bool sys_pll = (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL;
    bool pll_hse = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE;
    bool ok = hse_rdy && sys_pll && pll_hse;
    st_line(ok ? V_PASS : V_FAIL, "clock", "%s, core %lu MHz",
            ok ? "runs from the crystal (HSE -> PLL)" : "NOT running from the crystal",
            (unsigned long) (SystemCoreClock / 1000000u));
}

static void st_check_option_bytes() {
    option_bytes_status_t s = get_option_bytes_status();
    st_line(s.all_ok ? V_PASS : V_FAIL, "option_bytes", "nBOOT0 %s, nBOOT1 %s, nSWBOOT0 %s, BOR level %s",
            s.nboot0_ok ? "ok" : "BAD", s.nboot1_ok ? "ok" : "BAD",
            s.nswboot0_ok ? "ok" : "BAD", s.bor_ok ? "ok" : "BAD");
}

static bool st_strap_clean(u32 high) {
    return high < ST_STRAP_MARGIN || high > 1000u - ST_STRAP_MARGIN;
}

static void st_check_node_id() {
    bool clean = st_strap_clean(st_boot.addr0_high) && st_strap_clean(st_boot.addr1_high);
    st_line(clean ? V_PASS : V_WARN, "node_id",
            "wheel %u = node 0x%02X; CAN_ADDR0 read high %lu/1000, CAN_ADDR1 %lu/1000 "
            "(1000 = open, 0 = strapped)%s",
            (unsigned) st_boot.wheel, (unsigned) st_boot.node_id,
            (unsigned long) st_boot.addr0_high, (unsigned long) st_boot.addr1_high,
            clean ? "" : " -- a strap is marginal or floating");
}

static void st_check_vref() {
    u32 mv = aux_adc_conv_vref_mV_unclamped(aux_adc_last_reading_raw.vvcc_sns);
    st_verdict_t v = V_PASS;
    if (mv < ST_VREF_ABS_MIN_mV || mv > ST_VREF_ABS_MAX_mV)
        v = V_FAIL;
    else if (mv < ST_VREF_MIN_mV || mv > ST_VREF_MAX_mV)
        v = V_WARN;
    st_line(v, "vref", "ADC reference VREF+ = %lu mV (expect %lu-%lu)%s", (unsigned long) mv,
            (unsigned long) ST_VREF_MIN_mV, (unsigned long) ST_VREF_MAX_mV,
            v == V_FAIL ? " -- out of range: firmware falls back to 3.0 V, readings are off" : "");
}

static void st_check_temps() {
    i32 t[4] = {
        aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm_phase_a_sns),
        aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm_phase_b_sns),
        aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm_phase_c_sns),
        aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm_aux_sns),
    };
    i32 die = aux_adc_conv_thermint_cDegC(aux_adc_last_reading_raw.thermint_sns);
    static const char *const names[] = {"phase A", "phase B", "phase C", "aux"};

    // median of four = mean of the middle two
    i32 s[4] = {t[0], t[1], t[2], t[3]};
    for (u32 i = 1; i < 4; i++)
        for (u32 j = i; j > 0 && s[j - 1] > s[j]; j--) {
            i32 tmp = s[j];
            s[j] = s[j - 1];
            s[j - 1] = tmp;
        }
    i32 median = (s[1] + s[2]) / 2;

    st_verdict_t v = V_PASS;
    const char *bad = NULL;
    for (u32 i = 0; i < 4; i++) {
        if (t[i] < ST_TEMP_MIN_cC || t[i] > ST_TEMP_MAX_cC) {
            v = V_FAIL;
            bad = names[i];
        } else if (v == V_PASS && (t[i] - median > ST_TEMP_SPREAD_cC || median - t[i] > ST_TEMP_SPREAD_cC)) {
            v = V_WARN;
            bad = names[i];
        }
    }
    if (die < ST_TEMP_MIN_cC || die > ST_TEMP_MAX_cC) {
        v = V_FAIL;
        bad = "MCU die";
    }
    st_line(v, "temps", "phase A %.1f, B %.1f, C %.1f, aux %.1f, MCU die %.1f C%s%s%s",
            (f64) t[0] / 100., (f64) t[1] / 100., (f64) t[2] / 100., (f64) t[3] / 100.,
            (f64) die / 100.,
            bad ? " -- " : "", bad ? bad : "",
            v == V_FAIL ? " unphysical (open/shorted sensor?)" : v == V_WARN ? " disagrees with the others" : "");
}

static void st_check_encoder_spi() {
    u8 fw = 0, flags = 0;
    bool ok_fw = false, ok_flags = false, was_armed = false;
    // Exclusive SPI: the fast loop's angle read must not interleave with these frames, and
    // arming happens in the TIM17 ISR — mask, then check (same pattern as the disarmed sampler).
    u32 primask = __get_PRIMASK();
    __disable_irq();
    if (controller_is_armed()) {
        was_armed = true;
    } else {
        ok_fw = ma732_read_reg_bounded(MA732_REG_FW, &fw);
        ok_flags = ma732_read_reg_bounded(MA732_REG_FIELD_FLAGS, &flags);
    }
    __set_PRIMASK(primask);

    if (was_armed) {
        st_line(V_SKIP, "encoder_spi", "controller armed, SPI busy");
        return;
    }
    if (!ok_fw)
        st_line(V_FAIL, "encoder_spi", "MA732 did not answer on SPI (timeout)");
    else if (fw != MA732_FILTER_WINDOW_REG)
        st_line(V_FAIL, "encoder_spi", "MA732 register 0xE read %u, expected %u (SPI data bad, or the boot NVM write failed)",
                (unsigned) fw, (unsigned) MA732_FILTER_WINDOW_REG);
    else
        st_line(V_PASS, "encoder_spi", "MA732 answers: register 0xE = %u as expected", (unsigned) fw);

    if (ok_flags) {
        bool mgh = (flags & 0x80u) != 0u, mgl = (flags & 0x40u) != 0u;
        st_line(V_INFO, "magnet", "field flags 0x%02X: %s", (unsigned) flags,
                mgh ? "field HIGH (magnet too close?)"
                    : mgl ? "field LOW (no magnet over the chip?)" : "field in range");
    }
}

static void st_check_can() {
    u32 ecr = FDCAN1->ECR, psr = FDCAN1->PSR;
    u32 tec = (ecr & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos;
    u32 rec = (ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos;
    bool bo = (psr & FDCAN_PSR_BO) != 0u, ep = (psr & FDCAN_PSR_EP) != 0u, ew = (psr & FDCAN_PSR_EW) != 0u;
    st_verdict_t v = bo ? V_FAIL : (tec || rec || ep || ew) ? V_WARN : V_PASS;
    st_line(v, "can", "TX errors %lu, RX errors %lu%s%s%s%s", (unsigned long) tec, (unsigned long) rec,
            bo ? ", BUS-OFF" : "", ep ? ", error-passive" : "", ew ? ", error-warning" : "",
            v == V_WARN ? " (normal for a while after booting with nobody else on the bus; "
                          "if it stays, check wiring and termination)" : "");
}

static void st_check_adc_mid() {
    i16 boot[3];
    inverter_offset_cal_status_t cal = inverter_get_boot_offset_cal(&boot[0], &boot[1], &boot[2]);
    f32 boot_mV[3];
    for (u32 x = 0; x < 3; x++)
        boot_mV[x] = st_counts_to_mV((f32) boot[x]);

    if (cal == INVERTER_OFFSET_CAL_TIMED_OUT || cal == INVERTER_OFFSET_CAL_NOT_RUN) {
        st_line(V_FAIL, "adc_mid", "boot offset cal %s (no HRTIM/ADC ticks)",
                cal == INVERTER_OFFSET_CAL_TIMED_OUT ? "timed out" : "never ran");
        return;
    }

    if (st.f_n == 0u) {
        // No bridge-off data (report-only or cancelled early): judge on the boot measurement.
        bool ok = cal == INVERTER_OFFSET_CAL_OK;
        st_line(ok ? V_PASS : V_FAIL, "adc_mid",
                "boot offset cal (bridge switching at 50 %%): A %+.1f, B %+.1f, C %+.1f mV (limit +-%.0f)%s",
                (f64) boot_mV[0], (f64) boot_mV[1], (f64) boot_mV[2], (f64) ST_ADC_MID_LIMIT_mV,
                ok ? "" : " -- REJECTED, arming refused: a current-sense channel is bad");
        return;
    }

    f32 mid_mV[3];
    bool ok = true;
    for (u32 x = 0; x < 3; x++) {
        mid_mV[x] = st_counts_to_mV(st.f_raw_sum[x] / (f32) st.f_n - 2048.f);
        if (abs_f32(mid_mV[x]) > ST_ADC_MID_LIMIT_mV)
            ok = false;
    }
    st_line(ok ? V_PASS : V_FAIL, "adc_mid",
            "current-sense midpoints, bridge off: A %+.1f, B %+.1f, C %+.1f mV (limit +-%.0f); "
            "boot cal, switching: A %+.1f, B %+.1f, C %+.1f mV",
            (f64) mid_mV[0], (f64) mid_mV[1], (f64) mid_mV[2], (f64) ST_ADC_MID_LIMIT_mV,
            (f64) boot_mV[0], (f64) boot_mV[1], (f64) boot_mV[2]);
}

static void st_check_vbus() {
    if (st.f_n == 0u) {
        st_line(V_SKIP, "vbus", "no bridge-off data (bus voltage is only sampled while armed)");
        return;
    }
    f32 mean = st.f_vbus_sum / (f32) st.f_n, ripple = st.f_vbus_max - st.f_vbus_min;
    st_verdict_t v = V_PASS;
    if (mean < DC_BUS_UNDERVOLTAGE_THRESHOLD_V || mean > DC_BUS_OVERVOLTAGE_THRESHOLD_V)
        v = V_FAIL;
    else if (ripple > ST_VBUS_RIPPLE_WARN_V)
        v = V_WARN;
    st_line(v, "vbus", "bus %.2f V (min %.2f, max %.2f); trips below %.1f V and above %.1f V%s",
            (f64) mean, (f64) st.f_vbus_min, (f64) st.f_vbus_max,
            (f64) DC_BUS_UNDERVOLTAGE_THRESHOLD_V, (f64) DC_BUS_OVERVOLTAGE_THRESHOLD_V,
            v == V_WARN ? " -- noisy" : "");
}

static void st_check_float() {
    if (st.f_n == 0u)
        return; // nothing measured; the bridge line says why
    f32 vel = st_float_enc_vel();
    bool swing = false, leak = false;
    f32 mean[3];
    for (u32 x = 0; x < 3; x++) {
        mean[x] = st.f_phv_sum[x] / (f32) st.f_n;
        swing = swing || st_float_swing(x);
        leak = leak || st_float_leak(x);
    }
    bool turning = swing || abs_f32(vel) > ST_ENC_SPIN_MAX_REV_S;
    st_line(turning || leak ? V_FAIL : V_PASS, "bridge_off",
            "bridge off: phase mean/swing A %.2f/%.2f, B %.2f/%.2f, C %.2f/%.2f V; encoder %.1f rev/s%s",
            (f64) mean[0], (f64) (st.f_phv_max[0] - st.f_phv_min[0]),
            (f64) mean[1], (f64) (st.f_phv_max[1] - st.f_phv_min[1]),
            (f64) mean[2], (f64) (st.f_phv_max[2] - st.f_phv_min[2]), (f64) vel,
            leak ? " -- a phase sits near the bus: shorted high-side FET? Drive steps skipped"
                 : turning ? " -- something is turning (back-EMF/encoder). Drive steps skipped" : "");
    st_line(V_INFO, "encoder_angle", "bridge-off window: angle %u..%u of %u, net %ld counts over %lu samples",
            (unsigned) st.f_enc_min, (unsigned) st.f_enc_max, (unsigned) ENCODER_CPR,
            (long) st.f_enc_net, (unsigned long) st.f_enc_n);
}

static void st_check_bridge() {
    switch (st.outcome) {
        case ST_BRIDGE_NOT_RUN:
            st_line(V_SKIP, "bridge", "drive steps not run: arming is refused while the boot offset cal is bad (see adc_mid)");
            return;
        case ST_BRIDGE_SKIP_MOVING:
            st_line(V_FAIL, "bridge", "drive steps skipped: see bridge_off");
            return;
        case ST_BRIDGE_CANCELLED:
            st_line(V_FAIL, "bridge", "stopped early during the %s step (%s)",
                    st_phase_names[st.abort_phase],
                    st_exit_errors ? "controller error, see errors" : "state change requested");
            return;
        case ST_BRIDGE_ABORT_CURRENT:
            st_line(V_FAIL, "bridge", "stopped: %.2f A flowed during the %s step (limit %.1f A) -- "
                                      "phase-to-phase short or a motor that was turning",
                    (f64) st.abort_value, st_phase_names[st.abort_phase], (f64) ST_ABORT_CURRENT_A);
            return;
        case ST_BRIDGE_ABORT_SAG:
            st_line(V_FAIL, "bridge", "stopped: bus fell to %.2f V during the %s step (from %.2f V) -- "
                                      "shoot-through (shorted FET?) or supply current limit",
                    (f64) st.abort_value, st_phase_names[st.abort_phase], (f64) st.f_vbus_mean);
            return;
        case ST_BRIDGE_DONE:
        default:
            break;
    }

    static const char *const ids[] = {"bridge_A", "bridge_B", "bridge_C"};
    for (u32 x = 0; x < 3; x++) {
        f32 lo = st.d_n[0] ? st.d_frac_sum[0][x] / (f32) st.d_n[0] : -1.f;
        f32 hi = st.d_n[1] ? st.d_frac_sum[1][x] / (f32) st.d_n[1] : -1.f;
        bool lo_ok = lo >= 0.f && lo < ST_LO_FRAC_MAX;
        bool hi_ok = hi > ST_HI_FRAC_MIN;
        const char *why = "";
        if (!lo_ok && !hi_ok)
            why = " -- phase does not follow its leg";
        else if (!hi_ok)
            why = " -- never goes high: high-side FET or its gate drive/bootstrap not switching";
        else if (!lo_ok)
            why = " -- never goes low: low-side FET not switching, or high-side shorted";
        st_line(lo_ok && hi_ok ? V_PASS : V_FAIL, ids[x],
                "phase %c at 5 %% duty %.2f V (%.0f %% of bus, max %.0f), at 95 %% %.2f V (%.0f %%, min %.0f)%s",
                (char) ('A' + x), (f64) (lo * st.f_vbus_mean), (f64) (lo * 100.f), (f64) (ST_LO_FRAC_MAX * 100.f),
                (f64) (hi * st.f_vbus_mean), (f64) (hi * 100.f), (f64) (ST_HI_FRAC_MIN * 100.f), why);
    }

    bool i_ok = st.d_idm_max[1] <= ST_HI_CURRENT_MAX_A;
    st_line(i_ok ? V_PASS : V_FAIL, "bridge_current",
            "peak phase current %.2f A at 95 %% duty (limit %.1f); %.2f A at 5 %% (info: switching edges sit on the sample there)%s",
            (f64) st.d_idm_max[1], (f64) ST_HI_CURRENT_MAX_A, (f64) st.d_idm_max[0],
            i_ok ? "" : " -- current where none should flow");

    f32 vmin = st.d_vbus_min[0] < st.d_vbus_min[1] ? st.d_vbus_min[0] : st.d_vbus_min[1];
    bool v_ok = vmin >= ST_SAG_FAIL_FRAC * st.f_vbus_mean;
    st_line(v_ok ? V_PASS : V_FAIL, "bridge_vbus", "bus stayed at %.2f V or above while switching (bridge off: %.2f V)%s",
            (f64) vmin, (f64) st.f_vbus_mean,
            v_ok ? "" : " -- sagged: shoot-through or supply current limit");
}

static void st_check_errors() {
    if (st_exit_errors == 0u)
        return;
    char buf[160];
    buf[0] = '\0';
    STRINGIFY_CONTROLLER_ERRORS(buf, sizeof(buf), st_exit_errors);
    st_line(V_FAIL, "errors", "latched during the test: 0x%08lX %s", (unsigned long) st_exit_errors, buf);
}

static void st_info_boot() {
    u32 c = st_boot.rcc_csr;
    bool crashed = CRASH_RECORD->magic == CRASH_RECORD_MAGIC;
    bool iwdg = (c & RCC_CSR_IWDGRSTF) != 0u;
    st_line(crashed || iwdg ? V_WARN : V_INFO, "boot",
            "reset cause:%s%s%s%s%s%s%s; boot %lu since power-on%s",
            (c & RCC_CSR_LPWRRSTF) ? " LPWR" : "", (c & RCC_CSR_WWDGRSTF) ? " WWDG" : "",
            iwdg ? " IWDG (watchdog)" : "", (c & RCC_CSR_SFTRSTF) ? " SOFT" : "",
            (c & RCC_CSR_BORRSTF) ? " BOR" : "", (c & RCC_CSR_OBLRSTF) ? " OBL" : "",
            (c & RCC_CSR_PINRSTF) ? " PIN" : "",
            (unsigned long) noinit_cell_get(BOOT_COUNT_CELL),
            crashed ? "; a crash record exists since power-on (boot banner: LAST FAULT)" : "");
}

static void st_info_boot0() {
    u32 high = 0;
    for (u32 i = 0; i < 100u; i++)
        high += (HAL_GPIO_ReadPin(boot0_pin.port, boot0_pin.pin) != GPIO_PIN_RESET) ? 1u : 0u;
    st_line(V_INFO, "boot0", "BOOT0 (PB8) reads %s (%lu/100 high)",
            high == 0u ? "low: a reset boots this app"
                       : high == 100u ? "HIGH: a reset stays in the DFU bootloader" : "floating",
            (unsigned long) high);
}

static void st_info_cal_and_loop() {
    st_line(V_INFO, "encoder_cal", "direction cal %s, offset cal %s, LUT %s (none is normal on a bare board)",
            encoder_get_dir_cal_valid() ? "valid" : "none", encoder_get_offset_cal_valid() ? "valid" : "none",
            encoder_lut_enabled_get() ? "on" : "off");
    f32 budget = (f32) SystemCoreClock / LOOP_UPDATE_FREQ;
    f32 frac = (f32) isr_cycles_max / budget;
    st_line(frac > ST_LOOP_WARN_FRAC ? V_WARN : V_INFO, "fast_loop",
            "worst fast-loop tick since boot %lu cycles = %.0f %% of the 32 kHz budget",
            (unsigned long) isr_cycles_max, (f64) (frac * 100.f));
}

bool self_test_run_report() {
    st_n_fail = 0;
    st_n_warn = 0;
    __DMB(); // the fast loop's last writes to st are visible (it is disarmed by now)

    printf("\r\nSELFTEST BEGIN wheel %u node 0x%02X serial 0x%08lX\r\n", (unsigned) st_boot.wheel,
           (unsigned) st_boot.node_id, (unsigned long) OD_RAM.x1018_identity.serial_number);
    st_check_clock();
    st_check_option_bytes();
    st_check_node_id();
    st_check_vref();
    st_check_temps();
    st_check_encoder_spi();
    st_check_can();
    st_check_adc_mid();
    st_check_vbus();
    st_check_float();
    st_check_bridge();
    st_check_errors();
    st_info_boot();
    st_info_boot0();
    st_info_cal_and_loop();

    bool pass = st_n_fail == 0u;
    printf("SELFTEST END %s (%lu fail, %lu warn)\r\n\r\n", pass ? "PASS" : "FAIL",
           (unsigned long) st_n_fail, (unsigned long) st_n_warn);
    return pass;
}
