#include "backemf_cal.h"

#include "closed_loop_controller.h"
#include "config.h"
#include "encoder.h"
#include "foc.h"
#include "inverter.h"
#include "mathutil.h"
#include "printf.h"
#include "pwm.h"

/*
 * Phased law: SPINUP (closed-loop velocity, delegated to vel_ramp/foc) -> CUT (float the bridge,
 * counters/ISR/ADC triggers keep running) -> COAST (log encoder + phase-voltage back-EMF, gated
 * on speed) -> DONE (is_complete -> disarm -> FSM exit).
 *
 * During COAST the fast loop stays armed, so the producer/consumer contract must stay satisfied:
 * currents are consumed (~0 A at open circuit), and the PWM producer returns a valid-but-inert
 * {0.5,0.5,0.5} (compare writes land in registers; outputs are off). All trip checks in
 * adc_update_callback (overvoltage etc.) keep running -- they are the coast safety net.
 */

typedef enum {
    BEMF_PHASE_SPINUP = 0,
    BEMF_PHASE_CUT,    // single tick: float bridge, stamp metadata
    BEMF_PHASE_COAST,
    BEMF_PHASE_DRAIN,  // bidir only: bridge stays floated, wait for the rotor to slow before
                       // re-engaging FOC for the reverse leg (gentle re-entry at ~zero back-EMF)
    BEMF_PHASE_DONE,
} bemf_phase_t;

static volatile bemf_phase_t bemf_phase = BEMF_PHASE_DONE;

// Bidirectional mode (FSM_STATE_BACKEMF_CAL_BIDIR): run the whole spin->cut->coast sequence
// forward (leg 0, sign of BACKEMF_CAL_SPIN_TARGET), drain, then again reversed (leg 1) into the
// same capture buffer. Leg 0 is capped at half the buffer so leg 1 always has equal room; the
// offline fit needs no changes (it pools by per-sample sign of omega_e — one bidir run IS the
// gold-standard fwd+rev dataset, intercept interpolated across omega_e = 0).
static volatile bool bidir_mode = false;
static volatile u8 leg = 0;            // 0 = first leg, 1 = reversed leg
static volatile f32 leg_target = 0.f;  // current leg's signed spin target [rev/s mech]

// Capture storage (linear, no wrap; SWD-dumped by tools/backemf_capture.sh)
volatile backemf_sample_t backemf_capture_buf[BACKEMF_CAPTURE_LEN];
volatile u32 backemf_capture_idx = 0;

// Runtime-adjustable decimation (SWD: tools/backemf_capture.sh decim N). Default 64 = 500 Hz,
// sized so a bidir cal's coast fits the buffer. Poke to 1 for a full-rate (32 kHz) spindown
// burst — 1024 samples = 32 ms ≈ 10 elec cycles at 50 rev/s, resolving BEMF harmonics to h53
// alias-free (the 500 Hz default undersamples h11/h13 stroboscopically). Sticky until reboot;
// NOT reset by init so it can be poked before requesting the state.
volatile u32 backemf_capture_decim = BACKEMF_CAPTURE_DECIM;
volatile f32 backemf_meta_phase_offset = 0.f;
volatile f32 backemf_meta_enc_dir = 0.f;
volatile f32 backemf_meta_omega_cut = 0.f;
volatile u32 backemf_meta_cut_loop = 0;

// Tick bookkeeping (all in fast-loop ticks)
static volatile u32 phase_start_loop = 0;     // loop count at entry to current phase
static volatile u32 at_speed_ticks = 0;       // contiguous ticks within tolerance of target
static volatile u32 decim_ctr = 0;
static volatile u16 last_enc_raw = 0;         // cached by encoder_cb each tick
static volatile bool saw_motion = false;      // |omega| exceeded the gate at least once in COAST
static volatile bool spinup_timed_out = false;

static const u32 settle_ticks = (u32) (LOOP_UPDATE_FREQ / 1000.f * (f32) BACKEMF_CAL_SETTLE_MS);
static const u32 spinup_timeout_ticks = (u32) (LOOP_UPDATE_FREQ / 1000.f * (f32) BACKEMF_CAL_SPINUP_TIMEOUT_MS);
static const u32 cut_blank_ticks = (u32) (LOOP_UPDATE_FREQ / 1000.f * (f32) BACKEMF_CAL_CUT_BLANK_MS);
static const u32 coast_timeout_ticks = (u32) (LOOP_UPDATE_FREQ / 1000.f * (f32) BACKEMF_CAL_COAST_TIMEOUT_MS);

// Require at least this many samples for finish() to report success
#define BACKEMF_MIN_SAMPLES 50u

static __fn_inline bool is_hand_spin_mode() {
    return BACKEMF_CAL_SPIN_TARGET == 0.f;
}

static void backemf_common_init(bool bidir) {
    // Full FOC + velocity-ramp init (vel_ramp -> vel -> torque -> init_foc), even in hand-spin
    // mode -- harmless there, and it zeroes all the integrators either way.
    vel_ramp_ctrl_control_law.control_law_init();

    backemf_capture_idx = 0;
    decim_ctr = 0;
    at_speed_ticks = 0;
    saw_motion = false;
    spinup_timed_out = false;

    leg = 0;
    leg_target = BACKEMF_CAL_SPIN_TARGET;
    bidir_mode = bidir && !is_hand_spin_mode(); // bidir + hand-spin is meaningless: run one leg

    backemf_meta_phase_offset = encoder_get_phase_offset();
    backemf_meta_enc_dir = encoder_get_direction();
    backemf_meta_omega_cut = 0.f;
    backemf_meta_cut_loop = 0;

    phase_start_loop = controller_get_loop_count();

    if (is_hand_spin_mode()) {
        bemf_phase = BEMF_PHASE_CUT; // float the bridge on the first exec tick
    } else {
        bemf_phase = BEMF_PHASE_SPINUP;
        set_vel_ramp_setpoint(leg_target);
    }
}

void backemf_cal_init() {
    backemf_common_init(false);
}

// Separate init (not fsm_get_current_state() in the common one): control_law_init runs from the
// fast-loop ISR on the first tick after arm, which can preempt fsm_enter_state BEFORE it stores
// the new fsm_state — reading it there would race. Two law structs, zero ambiguity.
void backemf_cal_bidir_init() {
    backemf_common_init(true);
}

__ccmram bool backemf_cal_encoder_cb(u16 meas) {
    last_enc_raw = meas; // raw angle for the capture; estimator update happens in foc's consumer
    return foc_control_law.encoder_consumer(meas);
}

__ccmram bool backemf_cal_current_cb(f32 A_i, f32 B_i, f32 C_i, f32 Vbus) {
    // Delegate in every phase: keeps Vbus/current bookkeeping alive during COAST (currents ~0 at
    // open circuit; foc's pwm producer isn't called then, so its PI state is inert).
    return foc_control_law.current_consumer(A_i, B_i, C_i, Vbus);
}

__ccmram bool backemf_cal_pwm_cb(svm_vec_t *mag) {
    if (bemf_phase == BEMF_PHASE_SPINUP)
        return foc_control_law.pwm_producer(mag);

    // Bridge is (about to be) floated: produce a valid, inert SVM so the fast-loop validators
    // stay satisfied. The compare registers are written but the outputs are disabled.
    *mag = (svm_vec_t) {0.5f, 0.5f, 0.5f};
    return true;
}

__ccmram void backemf_cal_exec_cb() {
    u32 now = controller_get_loop_count();
    f32 omega = encoder_get_pos_vel().vel; // rev/s mechanical, signed
    f32 omega_abs = abs_f32(omega);

    switch (bemf_phase) {
        case BEMF_PHASE_SPINUP: {
            vel_ramp_ctrl_control_law.control_law_exec(); // slew + velocity PI -> torque -> iq

            if (omega_abs >= abs_f32(leg_target) * BACKEMF_CAL_SPIN_REACHED_FRAC) {
                if (++at_speed_ticks >= settle_ticks) {
                    bemf_phase = BEMF_PHASE_CUT;
                }
            } else {
                at_speed_ticks = 0; // settle must be contiguous
            }

            if (now - phase_start_loop > spinup_timeout_ticks) {
                // Never reached speed (load? bad bootstrap offset? target too high for Vbus?).
                // Skip the coast entirely; normal disarm turns the bridge off. (On a bidir leg-1
                // timeout the leg-0 samples are still in the buffer and dumpable.)
                spinup_timed_out = true;
                bemf_phase = BEMF_PHASE_DONE;
            }
            break;
        }

        case BEMF_PHASE_CUT: {
            hrtim_outputs_disable(); // hi-Z; HRTIM counters / ISR / ADC triggers keep running
            backemf_meta_omega_cut = omega;
            backemf_meta_cut_loop = now;
            phase_start_loop = now;
            decim_ctr = 0;
            bemf_phase = BEMF_PHASE_COAST;
            break;
        }

        case BEMF_PHASE_COAST: {
            u32 ticks_in_coast = now - phase_start_loop;

            if (ticks_in_coast < cut_blank_ticks)
                break; // let the inductive current decay through the body diodes first

            // Bidir leg 0 only fills half the buffer so the reverse leg always has equal room
            // (leg 1 just keeps appending — the fit pools by per-sample omega_e sign).
            u32 cap = (bidir_mode && leg == 0) ? (BACKEMF_CAPTURE_LEN / 2u) : BACKEMF_CAPTURE_LEN;

            if (omega_abs >= BACKEMF_CAL_OMEGA_MIN) {
                saw_motion = true;

                if (++decim_ctr >= backemf_capture_decim) {
                    decim_ctr = 0;
                    u32 idx = backemf_capture_idx;
                    if (idx < cap) {
                        u32 va, vb, vc;
                        inverter_get_phase_voltages_raw(&va, &vb, &vc);
                        backemf_capture_buf[idx] = (backemf_sample_t) {
                                .enc_raw = last_enc_raw,
                                .va_raw = (u16) va,
                                .vb_raw = (u16) vb,
                                .vc_raw = (u16) vc,
                                .omega_mech = omega,
                                .loop_count = now,
                        };
                        backemf_capture_idx = idx + 1;
                    }
                }
            }

            // Leg end: this leg's buffer share full, rotor back below the gate (with hysteresis)
            // after having moved, or timeout (hand-spin patience / stuck-coast guard).
            if (backemf_capture_idx >= cap ||
                (saw_motion && omega_abs < BACKEMF_CAL_OMEGA_MIN * 0.8f) ||
                ticks_in_coast > coast_timeout_ticks) {
                if (bidir_mode && leg == 0) {
                    bemf_phase = BEMF_PHASE_DRAIN; // bridge stays floated; slow down first
                    phase_start_loop = now;
                } else {
                    bemf_phase = BEMF_PHASE_DONE;
                }
            }
            break;
        }

        case BEMF_PHASE_DRAIN: {
            // Wait (still hi-Z) until the rotor is slow before re-engaging FOC for the reverse
            // leg: re-entry at ~3 rev/s means near-zero back-EMF, a small velocity error for the
            // ramp, and fresh integrators — no current-clamp braking event from coast speed.
            // On timeout, proceed anyway (ramp + iq clamp bound the worst case).
            if (omega_abs < BACKEMF_CAL_OMEGA_MIN * 0.8f ||
                now - phase_start_loop > coast_timeout_ticks) {
                leg = 1;
                leg_target = -leg_target;
                vel_ramp_ctrl_control_law.control_law_init(); // zero FOC/vel integrators + ramp
                set_vel_ramp_setpoint(leg_target);
                saw_motion = false;
                at_speed_ticks = 0;
                decim_ctr = 0;
                hrtim_outputs_enable(); // compares held {0.5,0.5,0.5} from coast; FOC output
                                        // takes over this same tick (exec runs before producer)
                phase_start_loop = now;
                bemf_phase = BEMF_PHASE_SPINUP;
            }
            break;
        }

        case BEMF_PHASE_DONE:
        default:
            break; // is_complete() ends the run this tick
    }
}

__ccmram bool backemf_cal_is_complete() {
    return bemf_phase == BEMF_PHASE_DONE;
}

/*
 * ==================== On-board bidirectional fit (BIDIR state only) ====================
 *
 * Runs over the FROZEN capture buffer in finish() (FSM-tick context, controller already
 * disarmed): the same algorithm as tools/backemf_cal_fit.py — per-channel zeros from the record
 * mean, full 3-term Clarke (floating-neutral CM rejection), per-sample
 * delta = wrap(atan2(s*v_d, s*v_q)), circular-mean centering, then ONE pooled regression
 * delta = D + k*omega_e. The bidir capture brackets omega_e = 0, so D (the genuine offset
 * error) and k (lumped sensing delay) separate cleanly — one-sided fits don't (measured ~26 us
 * of D/k collinearity artifact), which is why state 17 never applies its fit.
 *
 * After applying D to the live offset, a final pass bins the residual wrap(delta - D - k*we) by
 * RAW mechanical angle into the ENC_LUT_N-entry table: the per-rev encoder error map
 * (eccentricity etc.), converted to mech-turn corrections and installed via encoder_set_lut().
 *
 * ~4 passes x <=1024 samples of sincos/atan2f ~= a few ms in the 1 kHz FSM ISR — same blocking
 * class as encoder_validation's UART dumps (docs/footguns.md); acceptable for a deliberate,
 * rare cal action.
 */

static f32 lut_sum[ENC_LUT_N];
static u16 lut_cnt[ENC_LUT_N];
static f32 lut_table[ENC_LUT_N];

// Per-sample delta/omega_e from the capture buffer, with SNR + speed gates. Returns false if
// the sample is gated out. theta is recomputed from the RAW angle (LUT-independent), so a
// re-cal always measures the FULL error and produces a complete replacement offset+LUT.
static bool fit_sample(u32 i, f32 za, f32 zb, f32 zc, f32 off_used, f32 dir,
                       f32 *delta_out, f32 *we_out) {
    const f32 erad = (f32) MOTOR_NUM_POLEPAIRS * 2.f * M_PI_f32;

    f32 om = backemf_capture_buf[i].omega_mech;
    if (abs_f32(om) < BACKEMF_CAL_OMEGA_MIN)
        return false;

    f32 va = (f32) backemf_capture_buf[i].va_raw - za;
    f32 vb = (f32) backemf_capture_buf[i].vb_raw - zb;
    f32 vc = (f32) backemf_capture_buf[i].vc_raw - zc;

    f32 valpha = (2.f / 3.f) * (va - 0.5f * vb - 0.5f * vc);
    f32 vbeta = (vb - vc) * one_over_sqrt3;
    if (valpha * valpha + vbeta * vbeta < BACKEMF_FIT_MIN_VMAG * BACKEMF_FIT_MIN_VMAG)
        return false;

    f32 pos = (f32) backemf_capture_buf[i].enc_raw / (f32) ENCODER_CPR;
    f32 theta = wrap_pm_pi(erad * (pos - off_used)) * dir; // mirror encoder_get_phase()
    f32 we = erad * dir * om;

    sincos_t sc = sincos_cmath(theta); // cmath, not sincos_f32: fit results get burned to flash — keep full f32 precision independent of the runtime backend (and off the single-context CORDIC)
    f32 vd = valpha * sc.cos + vbeta * sc.sin; // mirror park_transform()
    f32 vq = vbeta * sc.cos - valpha * sc.sin;
    f32 sgn = (we >= 0.f) ? 1.f : -1.f;

    *delta_out = wrap_pm_pi(atan2f(sgn * vd, sgn * vq));
    *we_out = we;
    return true;
}

static bool backemf_fit_and_apply() {
    const u32 n = backemf_capture_idx;
    const f32 off_used = backemf_meta_phase_offset;
    const f32 dir = backemf_meta_enc_dir;
    const f32 erad = (f32) MOTOR_NUM_POLEPAIRS * 2.f * M_PI_f32;
    f32 d, we;

    // Pass 0: per-channel zeros = record means (back-EMF averages ~0 over many elec cycles)
    f32 za = 0.f, zb = 0.f, zc = 0.f;
    for (u32 i = 0; i < n; i++) {
        za += (f32) backemf_capture_buf[i].va_raw;
        zb += (f32) backemf_capture_buf[i].vb_raw;
        zc += (f32) backemf_capture_buf[i].vc_raw;
    }
    za /= (f32) n; zb /= (f32) n; zc /= (f32) n;

    // Pass 1: circular mean of delta + per-direction sample counts
    f32 ssin = 0.f, scos = 0.f;
    u32 nf = 0, nr = 0;
    for (u32 i = 0; i < n; i++) {
        if (!fit_sample(i, za, zb, zc, off_used, dir, &d, &we)) continue;
        sincos_t sc = sincos_cmath(d);
        ssin += sc.sin; scos += sc.cos;
        if (we >= 0.f) nf++; else nr++;
    }
    if (nf < BACKEMF_FIT_MIN_PER_DIR || nr < BACKEMF_FIT_MIN_PER_DIR) {
        printf("BEMF fit: not enough gated samples (fwd %u rev %u, need %u each)\r\n",
               (unsigned) nf, (unsigned) nr, (unsigned) BACKEMF_FIT_MIN_PER_DIR);
        return false;
    }
    f32 cmean = atan2f(ssin, scos);

    // Pass 2: pooled regression of wrap(delta - cmean) against omega_e
    f32 Sn = 0.f, Sw = 0.f, Sd = 0.f, Swd = 0.f, Sww = 0.f;
    for (u32 i = 0; i < n; i++) {
        if (!fit_sample(i, za, zb, zc, off_used, dir, &d, &we)) continue;
        f32 dc = wrap_pm_pi(d - cmean);
        Sn += 1.f; Sw += we; Sd += dc; Swd += we * dc; Sww += we * we;
    }
    f32 den = Sn * Sww - Sw * Sw; // bidir balance -> Sw ~ 0, den ~ Sn*Sww: well conditioned
    f32 k = (Sn * Swd - Sw * Sd) / den;
    f32 D = wrap_pm_pi(cmean + (Sd - k * Sw) / Sn);

    if (!f32_finite_bits(D) || !f32_finite_bits(k) ||
        abs_f32(D) > BACKEMF_FIT_MAX_DELTA || abs_f32(k) > BACKEMF_FIT_MAX_K) {
        printf("BEMF fit: REJECTED D=%.4f rad k=%.1f us (bounds %.2f rad / %.0f us) - bad bootstrap?\r\n",
               D, k * 1e6f, BACKEMF_FIT_MAX_DELTA, BACKEMF_FIT_MAX_K * 1e6f);
        return false;
    }

    // Apply the offset correction (same sign convention as encoder_phase_cal_finish; verified
    // empirically per the method doc §7 on 2026-06-09).
    f32 delta_mech = (D / dir) / erad;
    encoder_set_phase_offset(off_used + delta_mech);
    encoder_set_offset_cal_valid(true); // gates passed — this offset is a real calibration

    // Pass 3: LUT — bin the residual by RAW mech angle, convert to mech-turn corrections.
    for (u32 i = 0; i < ENC_LUT_N; i++) { lut_sum[i] = 0.f; lut_cnt[i] = 0; }
    for (u32 i = 0; i < n; i++) {
        if (!fit_sample(i, za, zb, zc, off_used, dir, &d, &we)) continue;
        f32 r = wrap_pm_pi(d - D - k * we);
        u32 bin = backemf_capture_buf[i].enc_raw / (ENCODER_CPR / ENC_LUT_N);
        if (bin < ENC_LUT_N) {
            lut_sum[bin] += r;
            lut_cnt[bin]++;
        }
    }

    u32 empty = 0;
    for (u32 i = 0; i < ENC_LUT_N; i++)
        if (lut_cnt[i] == 0) empty++;

    bool lut_ok = (f32) empty <= BACKEMF_LUT_MAX_EMPTY_FRAC * (f32) ENC_LUT_N;
    if (lut_ok) {
        // residual [rad elec] -> correction [mech turns]: pos += t shifts theta_hat by
        // erad*dir*t; we want -r, so t = -(r/dir)/erad (same conversion as the offset).
        for (u32 i = 0; i < ENC_LUT_N; i++)
            lut_table[i] = (lut_cnt[i] > 0) ? -((lut_sum[i] / (f32) lut_cnt[i]) / dir) / erad : 0.f;

        // Fill empty bins by circular linear interpolation between populated neighbors
        for (u32 i = 0; i < ENC_LUT_N; i++) {
            if (lut_cnt[i] > 0) continue;
            u32 p = i, q = i, dp = 0, dq = 0;
            do { p = (p + ENC_LUT_N - 1u) & (ENC_LUT_N - 1u); dp++; } while (lut_cnt[p] == 0);
            do { q = (q + 1u) & (ENC_LUT_N - 1u); dq++; } while (lut_cnt[q] == 0);
            lut_table[i] = (lut_table[p] * (f32) dq + lut_table[q] * (f32) dp) / (f32) (dp + dq);
        }

        // Spatial-harmonic limit: keep ONLY 1..BACKEMF_LUT_MAX_HARMONIC cycles/rev. Real encoder
        // error is low-order (eccentricity = 1/rev + a small 2/rev); the trapezoidal back-EMF
        // makes the measured alpha-beta vector wobble at 6*polepairs = 36 cycles/rev, which the
        // raw bins capture as FAKE encoder error — applying it would inject genuine 6f_e current
        // ripple into FOC. A Fourier reconstruction without the DC term is also inherently
        // zero-mean (the mean belongs to the scalar offset) and smooths bin noise + interp seams.
        {
            f32 ak[BACKEMF_LUT_MAX_HARMONIC], bk[BACKEMF_LUT_MAX_HARMONIC];
            for (u32 h = 0; h < BACKEMF_LUT_MAX_HARMONIC; h++) {
                f32 a = 0.f, b = 0.f;
                for (u32 i = 0; i < ENC_LUT_N; i++) {
                    sincos_t sc = sincos_cmath(2.f * M_PI_f32 * (f32) (h + 1u) * (f32) i / (f32) ENC_LUT_N);
                    a += lut_table[i] * sc.cos;
                    b += lut_table[i] * sc.sin;
                }
                ak[h] = a * (2.f / (f32) ENC_LUT_N);
                bk[h] = b * (2.f / (f32) ENC_LUT_N);
            }
            for (u32 i = 0; i < ENC_LUT_N; i++) {
                f32 v = 0.f;
                for (u32 h = 0; h < BACKEMF_LUT_MAX_HARMONIC; h++) {
                    sincos_t sc = sincos_cmath(2.f * M_PI_f32 * (f32) (h + 1u) * (f32) i / (f32) ENC_LUT_N);
                    v += ak[h] * sc.cos + bk[h] * sc.sin;
                }
                lut_table[i] = v;
            }
        }

        f32 maxabs = 0.f;
        for (u32 i = 0; i < ENC_LUT_N; i++)
            maxabs = max_f32(maxabs, abs_f32(lut_table[i]));

        if (maxabs > BACKEMF_LUT_MAX_ABS) {
            printf("BEMF fit: LUT rejected, max |entry| %.5f turns > %.5f (keeping previous)\r\n",
                   maxabs, BACKEMF_LUT_MAX_ABS);
            lut_ok = false;
        } else {
            encoder_set_lut(lut_table);
            printf("BEMF fit: LUT installed, %u/%u bins measured, harmonics 1-%u, peak %.5f turns (%.2f deg mech)\r\n",
                   (unsigned) (ENC_LUT_N - empty), (unsigned) ENC_LUT_N,
                   (unsigned) BACKEMF_LUT_MAX_HARMONIC, maxabs, maxabs * 360.f);
        }
    } else {
        printf("BEMF fit: LUT skipped, %u/%u bins empty (keeping previous)\r\n",
               (unsigned) empty, (unsigned) ENC_LUT_N);
    }

    // Pass 5: 6θ_e harmonic feedforward coefficients. Demodulate the coast e_dq (the hi-Z
    // terminal voltage IS the BEMF) against the FINAL runtime angle — new offset plus the
    // just-installed LUT — so the runtime reconstruction in foc_gen_pwm_cb phase-aligns by
    // construction (all sign/dir conventions cancel: we project v/omega_signed onto
    // {cos 6θ', sin 6θ'} and replay omega*(A c + B s) against the same θ'). Normalizing by
    // signed omega makes both bidir legs consistent and the coefficients speed-invariant.
    {
        const f32 off_new = off_used + delta_mech;
        f32 Sxd = 0, Sxq = 0, Sc = 0, Ss = 0;
        f32 Sxdc = 0, Sxds = 0, Sxqc = 0, Sxqs = 0;
        u32 nh = 0;

        for (u32 i = 0; i < n; i++) {
            f32 om = backemf_capture_buf[i].omega_mech;
            if (abs_f32(om) < BACKEMF_CAL_OMEGA_MIN) continue;
            f32 va = (f32) backemf_capture_buf[i].va_raw - za;
            f32 vb = (f32) backemf_capture_buf[i].vb_raw - zb;
            f32 vc = (f32) backemf_capture_buf[i].vc_raw - zc;
            f32 valpha = (2.f / 3.f) * (va - 0.5f * vb - 0.5f * vc);
            f32 vbeta = (vb - vc) * one_over_sqrt3;
            if (valpha * valpha + vbeta * vbeta < BACKEMF_FIT_MIN_VMAG * BACKEMF_FIT_MIN_VMAG)
                continue;

            f32 pos = (f32) backemf_capture_buf[i].enc_raw / (f32) ENCODER_CPR;
            if (lut_ok) { // mirror the runtime LUT lerp so θ' == what foc.c will see
                f32 x = pos * (f32) ENC_LUT_N;
                u32 bi = (u32) x;
                f32 fr = x - (f32) bi;
                pos += lut_table[bi & (ENC_LUT_N - 1u)] * (1.f - fr) +
                       lut_table[(bi + 1u) & (ENC_LUT_N - 1u)] * fr;
            }
            f32 thp = wrap_pm_pi(erad * (pos - off_new)) * dir;
            f32 wes = erad * dir * om;

            sincos_t sct = sincos_cmath(thp);
            f32 xd = (valpha * sct.cos + vbeta * sct.sin) / wes;  // e_d/omega [counts*s/rad]
            f32 xq = (vbeta * sct.cos - valpha * sct.sin) / wes;
            sincos_t sc6 = sincos_cmath(wrap_pm_pi(6.f * thp));

            Sxd += xd; Sxq += xq; Sc += sc6.cos; Ss += sc6.sin;
            Sxdc += xd * sc6.cos; Sxds += xd * sc6.sin;
            Sxqc += xq * sc6.cos; Sxqs += xq * sc6.sin;
            nh++;
        }

        if (nh >= 200) {
            f32 fn = (f32) nh;
            f32 scale = 2.f / fn * PHASE_V_COUNTS_TO_VOLTS;
            bemf_h6_coeff_t cfs = {
                    .Ad = (Sxdc - (Sxd / fn) * Sc) * scale,
                    .Bd = (Sxds - (Sxd / fn) * Ss) * scale,
                    .Aq = (Sxqc - (Sxq / fn) * Sc) * scale,
                    .Bq = (Sxqs - (Sxq / fn) * Ss) * scale,
            };
            f32 m = max_f32(max_f32(abs_f32(cfs.Ad), abs_f32(cfs.Bd)),
                            max_f32(abs_f32(cfs.Aq), abs_f32(cfs.Bq)));
            if (m <= BACKEMF_FF_MAX_COEFF) {
                foc_set_bemf_harmonic_coeffs(&cfs);
                printf("BEMF fit: h6 FF coeffs [uV*s] d:(%+.1f,%+.1f) q:(%+.1f,%+.1f), n=%u\r\n",
                       cfs.Ad * 1e6f, cfs.Bd * 1e6f, cfs.Aq * 1e6f, cfs.Bq * 1e6f, (unsigned) nh);
            } else {
                printf("BEMF fit: h6 FF coeffs REJECTED (max %.1f uV*s > %.0f; keeping previous)\r\n",
                       m * 1e6f, BACKEMF_FF_MAX_COEFF * 1e6f);
            }
        } else {
            printf("BEMF fit: h6 FF skipped (only %u gated samples)\r\n", (unsigned) nh);
        }
    }

    printf("BEMF fit: D=%+.4f rad elec (%.2f deg), k=%+.1f us, n=%u fwd / %u rev\r\n",
           D, D * (180.f / M_PI_f32), k * 1e6f, (unsigned) nf, (unsigned) nr);
    printf("BEMF fit: offset %.6f -> %.6f turns (request SAVE_CALIBRATION to persist)\r\n",
           off_used, off_used + delta_mech);
    return true;
}

bool backemf_cal_finish(bool was_cancelled) {
    // Buffer + metadata stay frozen for SWD dump regardless of how we exited.
    u32 n = backemf_capture_idx;
    printf("BEMF cal: %u samples%s, cut @ %.1f rev/s, %s%s\r\n",
           (unsigned) n, bidir_mode ? (leg == 1 ? " (bidir, both legs)" : " (bidir, LEG 0 ONLY)") : "",
           backemf_meta_omega_cut,
           was_cancelled ? "CANCELLED" : "complete",
           spinup_timed_out ? " (SPINUP TIMEOUT)" : "");

    if (was_cancelled || spinup_timed_out) return false;
    if (bidir_mode && leg != 1) return false; // reverse leg never ran
    if (n < BACKEMF_MIN_SAMPLES) return false;

    // BIDIR: full on-board self-cal — fit offset (+ delay diagnostic) and rebuild the LUT from
    // the frozen buffer, applying both to the live encoder state. Persist with SAVE_CALIBRATION.
    // Single-direction (state 17) stays log-only: its one-sided fit is collinearity-biased.
    if (bidir_mode)
        return backemf_fit_and_apply();
    return true;
}

control_law_t backemf_cal_ctrl_law = {
        .control_law_init = backemf_cal_init,
        .encoder_consumer = backemf_cal_encoder_cb,
        .current_consumer = backemf_cal_current_cb,
        .pwm_producer = backemf_cal_pwm_cb,
        .control_law_exec = backemf_cal_exec_cb,
        .is_complete = backemf_cal_is_complete,
        .control_law_finish = backemf_cal_finish,
};

// Bidirectional variant (FSM_STATE_BACKEMF_CAL_BIDIR): identical callbacks, init arms both legs.
control_law_t backemf_cal_bidir_ctrl_law = {
        .control_law_init = backemf_cal_bidir_init,
        .encoder_consumer = backemf_cal_encoder_cb,
        .current_consumer = backemf_cal_current_cb,
        .pwm_producer = backemf_cal_pwm_cb,
        .control_law_exec = backemf_cal_exec_cb,
        .is_complete = backemf_cal_is_complete,
        .control_law_finish = backemf_cal_finish,
};
