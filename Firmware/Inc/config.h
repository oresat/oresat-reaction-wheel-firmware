#pragma once

#include "typedefs.h"

//#define IS_CORRECTED_VOLTAGE_DIVIDER

// DCBus
static const f32 DC_BUS_OVERVOLTAGE_THRESHOLD_V = 4.3f * 2.f; // V
static const f32 DC_BUS_UNDERVOLTAGE_THRESHOLD_V = 2.7f; // TODO ************ REMEMBER TO CHANGE THIS ************

// IBus
static const f32 IBUS_OVERCURRENT_THRESHOLD_A = 5.f; // A
static const f32 IBUS_OVERREGEN_THRESHOLD_A = 5.f; // A

// Motor current
static const f32 MOTOR_CURRENT_CMD_LIMIT = 5.0f; // A
static const f32 MOTOR_CURRENT_TRIP_LIMIT = 15.f; // A


// Inverter constants
static const f32 SHUNT_AMP_GAIN = -50.f; // V/V
static const f32 SHUNT_VALUE_OHM = 0.002f; // Ohm
static const f32 SHUNT_AMP_CONDUCTANCE = 1 / (SHUNT_AMP_GAIN * SHUNT_VALUE_OHM); // A/V

#ifdef IS_CORRECTED_VOLTAGE_DIVIDER
#define VBUS_DIVIDER_R_HI 8200.f
#define VBUS_DIVIDER_R_LO 2700.f
#else
#define VBUS_DIVIDER_R_HI 5100.f
#define VBUS_DIVIDER_R_LO 3300.f
#endif

static const f32 VSNS_VOLTAGE_DIVIDER = VBUS_DIVIDER_R_LO / (VBUS_DIVIDER_R_LO + VBUS_DIVIDER_R_HI); // V/V

//static const f32 VSNS_VOLTAGE_DIVIDER = 0.4; // V/V

// Safety
static const f32 MAX_CURRENT_LEAKAGE_AMOUNT = 1.0f; // 100mA max phase current imbalance allowed

// Encoder
static const f32 ENCODER_ESTIMATOR_BANDWIDTH = 1000.f; // tau (1/s)
static const u16 ENCODER_CPR = 16384;

// Motor calibration parameters
static const f32 MAX_RESISTANCE_CAL_VOLTAGE = 4.f; // V
static const f32 MOTOR_CALIBRATION_CURRENT = 2.f; // A
static const u32 RESISTANCE_CAL_TIME = 3000; // ms

static const f32 MOTOR_PHASE_RESISTANCE_MIN = 0.01f; //ohm
static const f32 MOTOR_PHASE_RESISTANCE_MAX = 1.0f; //ohm

static const u32 INDUCTANCE_CAL_TIME = 1000; // ms
static const f32 MOTOR_PHASE_INDUCTANCE_MIN = 1 * (0.001f) * (0.001f); // uH -> H
static const f32 MOTOR_PHASE_INDUCTANCE_MAX = 1000 * (0.001f) * (0.001f); // uH -> H
static const f32 MAX_INDUCTANCE_CAL_VOLTAGE = 1.0f; // V

static const f32 HARDCODE_PHASE_INDUCTANCE = 6.39e-6f / 2.f; // H
static const f32 HARDCODE_PHASE_RESISTANCE = 196.0e-3f / 2.f; // Ohm
static const f32 CURRENT_CONTROLLER_BANDWIDTH = 4000.f; // 1/s

#define MOTOR_NUM_POLEPAIRS 6 //fuck

static const f32 LOCKIN_SPIN_CURRENT = 4.f;

static const f32 ENCODER_DIR_CAL_ROTATION_DIST = 1.f; // rev
static const f32 ENCODER_DIR_CAL_ROTATION_VEL = 0.5f; // rev/s
static const f32 ENCODER_DIR_CAL_ROTATION_TOLERANCE = 0.1f; // 10% error ok

static const f32 ENCODER_PHASE_CAL_ROTATION_DIST = 4.f; // rev
static const f32 ENCODER_PHASE_CAL_ROTATION_VEL = 0.5f; // rev/s
static const f32 ENCODER_PHASE_CAL_RAMP_FRAC = 0.2f;     // fraction of each sweep leg spent in the accel/decel ramp (trapezoidal profile)
static const f32 ENCODER_PHASE_CAL_VEL_GATE_FRAC = 0.5f; // only accumulate offset error when |speed| exceeds this fraction of cruise speed


static const f32 MOTOR_KV = 6300.f;
static const f32 MOTOR_TORQUE_CONSTANT = 8.27f / MOTOR_KV;


static const u8 CANID_WHEEL1 = 0x3C;
static const u8 CANID_WHEEL2 = 0x40;
static const u8 CANID_WHEEL3 = 0x44;
static const u8 CANID_WHEEL4 = 0x48;

static const f32 VEL_RAMP_RATE = 20.f; //rev/s^2

static const f32 VEL_GAIN = 0.001f; // Nm/rev/s
static const f32 VEL_INTEGRATOR_GAIN = 0.001f; // Nm/rev
static const f32 VEL_INTEGRATOR_LIMIT = 1.f * MOTOR_TORQUE_CONSTANT; // Nm

// --- MA732 encoder filter window (register 0xE, stored in the SENSOR's NVM) ---
// Checked/written once at boot by ma732_ensure_filter_window() (option-bytes pattern: write only
// on mismatch; sensor NVM endures ~1k writes, each takes 20 ms + readback). Datasheet Table 18:
//   FW: 51=6kHz/9.5bit/tau 64us | 68=3kHz/10b | 85=1.5kHz/10.5b | 102=740Hz/11b |
//       119=370Hz/11.5bit/tau 1024us (factory) | 136=185Hz/12b | ... | 187=23Hz/13.5b
// We run 6 kHz: the coast-cal measured k=+1.9us total sensing delay, proving the tracking filter
// has ~zero constant-velocity lag either way — but 6 kHz keeps the per-rev eccentricity pattern
// (the future angle-LUT content) phase-correct across the whole speed envelope (370 Hz shifts it
// ~8 deg of pattern position at 51 rev/s, worse at flight speed) and puts the sensor filter
// (64 us) cleanly inside our 1 ms PLL. Cost: 4x raw angle noise (9.5 vs 11.5 ENOB), mostly
// absorbed by the PLL. Revert to 119 here if velocity noise bites — the boot check writes it back.
static const u8 MA732_FILTER_WINDOW_REG = 51;

// --- Back-EMF coast-down encoder-offset calibration (FSM_STATE_BACKEMF_CAL) ---
// Spin up closed-loop, float the bridge, log encoder + phase-voltage back-EMF during the coast;
// offset computed offline from the SWD dump. See docs/backemf-coastdown-implementation.md.
static const f32 BACKEMF_CAL_SPIN_TARGET = -50.f;        // rev/s mech; sign = direction. 0 = hand-spin
                                                        // mode: no spin-up, bridge floats on arm, logs
                                                        // whatever motion appears (bench bring-up).
static const f32 BACKEMF_CAL_OMEGA_MIN = 4.f;           // rev/s mech; logging gate & completion floor
                                                        // (back-EMF SNR collapses below this)
static const f32 BACKEMF_CAL_SPIN_REACHED_FRAC = 0.95f; // |omega| >= frac*target counts as at-speed
static const u32 BACKEMF_CAL_SETTLE_MS = 500;           // contiguous at-speed time before the cut
static const u32 BACKEMF_CAL_SPINUP_TIMEOUT_MS = 10000; // give up if target never reached
static const u32 BACKEMF_CAL_CUT_BLANK_MS = 2;          // post-cut blank: winding current decays
                                                        // through body diodes before samples count
static const u32 BACKEMF_CAL_COAST_TIMEOUT_MS = 30000;  // coast/hand-spin patience

// On-board fit gates (BIDIR state only — one-sided fits carry a measured Δ/k-collinearity
// artifact and are never applied by firmware; state 17 stays log-only for offline analysis).
static const u32 BACKEMF_FIT_MIN_PER_DIR = 100;   // gated samples required in EACH spin direction
static const f32 BACKEMF_FIT_MIN_VMAG = 15.f;     // |v_alphabeta| SNR gate [ADC counts]
static const f32 BACKEMF_FIT_MAX_DELTA = 0.8f;    // |Δ| sanity bound [rad elec] (~46°): bigger
                                                  // means the bootstrap offset was garbage
static const f32 BACKEMF_FIT_MAX_K = 200e-6f;     // |k| sanity bound [s] (measured ~2 µs)
// LUT build gates: bins are 1/128 mech rev, filled from the post-fit residual.
static const f32 BACKEMF_LUT_MAX_ABS = 0.01f;     // |entry| bound [mech turns] (3.6° mech;
                                                  // eccentricity-class errors are ~10x smaller)
static const f32 BACKEMF_LUT_MAX_EMPTY_FRAC = 0.25f; // refuse the table if angular coverage is
                                                  // this sparse (keeps any previous LUT)
#define BACKEMF_LUT_MAX_HARMONIC 4u               // keep only 1..K cycles/rev in the LUT. Real
                                                  // encoder error is low-order (eccentricity =
                                                  // 1/rev + small 2/rev); the TRAPEZOIDAL back-EMF
                                                  // wobbles the measured angle at 6*polepairs =
                                                  // 36/rev, which would otherwise bake into the
                                                  // table as fake encoder error and inject real
                                                  // 6f_e ripple into FOC. Spatial low-pass kills
                                                  // it regardless of amplitude (lut-design §3.5).

static const bool FOC_USE_bEMF_FF = false;
static const bool FOC_USE_wL_FF = false;
static const bool FOC_USE_R_FF = false;
static const bool FOC_USE_DECOUPLING = true;

// --- High-modulation phase-current reconstruction ("best 2 of 3" shunt selection) ---
// At high modulation the phase whose PWM edge crowds the crest ADC sample point sees a
// common-mode kick and reads wrong. When enabled, that phase (smallest SVM magnitude →
// largest compare → edge nearest the crest) is dropped each tick and reconstructed from
// the trusted two via KCL (Ia+Ib+Ic=0), instead of the symmetric (zero/3) CM removal.
// EXPERIMENTAL, A/B-gated — seeds the runtime `shunt_selection_enabled` flag (control_ll).
// See docs/high-speed-foc-worklog.md. Validate on a scope before trusting it.
static const bool FOC_USE_SHUNT_SELECTION = true;
// Pipeline lag (in ISR ticks) between writing a duty and reading the current it produced.
// Currents are read live from JDR1 (inverter_get_adc_raw) while the current crest's
// conversion is still in flight, so the sample reflects a duty written ~2 ticks earlier;
// with the natural 1-tick call ordering (adc before pwm) that means 1 history slot back.
// 0..3. CONFIRMED ON BENCH 2026-06-09 (worklog Q2-A closed): tools/shuntsel_delay.sh
// multi-edge torque-step measured K=2 (sample read at tick M reflects duty written at M-2,
// margin 1.6x) -> DELAY = K-1 = 1, matching this reasoned value. Re-measure if the loop rate,
// ISR call ordering, ADC timing/oversampling, or CMP4 position changes —
// see docs/shuntsel-delay-calibration.md.
#define SHUNT_SEL_DELAY_TICKS 1

// --- ADC sample-point trigger position (HRTIM Timer A CMP4, in HRTIM ticks @ 1/4.608 GHz) ---
// The injected-ADC trigger fires when Timer A's up-count hits this value (ADROM_CREST gates it
// to the up-slope); the 4x-oversampling burst centroid then lands a delay D after it. Chosen so
// the burst centers on the crest (counter = PWM_CNTR = 24000 = temporal middle of the switching
// period, mid null-vector). Current value uses the ANALYTIC D ~= 3232 ticks (trigger latency
// ASSUMED ~42 ns + S&H centroid + 1.5*Tconv) — measure the real D scope-free with
// tools/cmp4_sweep.sh + tools/cmp4_sweep_analyze.py and set this to 24000 - D_measured.
// See docs/adc-sampling-tuning.md. Applied live by the fast loop (control_ll.c) on change;
// the boot-time literal in generated hrtim.c is a don't-care once armed.
#define ADC_TRIG_CMP4_TICKS 20908u

// --- Open-loop fixed-duty bench mode (CMP4 sweep support) ---
// Safety clamp on |modulation| in the SWD-driven fixed-duty mode (open_loop_current_controller).
// 0.10 at Vbus 8 V -> ~0.53 V differential -> ~5.4 A locked-rotor on this motor (R ~= 0.098 ohm):
// at the command-limit scale, well under the 15 A trip.
static const f32 OPEN_LOOP_FIXED_MOD_LIMIT = 0.10f;
