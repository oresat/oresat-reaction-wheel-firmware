#pragma once

#include "control_ll.h"
#include "typedefs.h"

/*
 * Back-EMF coast-down encoder-offset calibration (FSM_STATE_BACKEMF_CAL).
 *
 * Spin the rotor up closed-loop, float the bridge (hrtim_outputs_disable -- counters/ISR/ADC
 * triggers keep running), and log raw encoder angle + the three phase-voltage dividers while the
 * rotor coasts. The offset is computed OFFLINE from the dump (v1 is log-only; no write-back).
 * Method: docs/backemf_coastdown_calibration.md. Firmware plan + status:
 * docs/backemf-coastdown-implementation.md.
 *
 * BACKEMF_CAL_SPIN_TARGET = 0 selects "hand-spin mode": no spin-up, bridge floats immediately on
 * arm, and the law logs whatever motion appears (flick the wheel by hand). Bench bring-up tool.
 */

// One capture sample. 16 bytes, no padding -- layout is mirrored by tools/backemf_capture.sh's
// decoder; keep the two in sync (same contract as foc_capture).
typedef struct backemf_sample {
    u16 enc_raw;     // raw MA702 angle this tick, 0..ENCODER_CPR-1
    u16 va_raw;      // PHA_V divider, 12-bit counts (4x HW oversampled / 4)
    u16 vb_raw;      // PHB_V
    u16 vc_raw;      // PHC_V
    f32 omega_mech;  // PLL velocity estimate, rev/s mechanical, signed
    u32 loop_count;  // controller_get_loop_count() at sample (timebase / gap check)
} backemf_sample_t;

#define BACKEMF_CAPTURE_LEN   1024u  // 1024 x 16 B = 16 KB
#define BACKEMF_CAPTURE_DECIM 64u    // log every Nth fast-loop tick: 32 kHz / 64 = 500 Hz

// Linear (non-wrapping) capture: buf[0..idx) are valid, idx saturates at LEN. Read over SWD.
extern volatile backemf_sample_t backemf_capture_buf[BACKEMF_CAPTURE_LEN];
extern volatile u32 backemf_capture_idx;

// Run metadata for the offline fit (values in effect during the run). SWD-read with the buffer.
extern volatile f32 backemf_meta_phase_offset; // encoder phase offset (mech turns) during run
extern volatile f32 backemf_meta_enc_dir;      // encoder direction (+/-1) during run
extern volatile f32 backemf_meta_omega_cut;    // rev/s mech when outputs were floated
extern volatile u32 backemf_meta_cut_loop;     // loop count at outputs-off

extern control_law_t backemf_cal_ctrl_law;       // single direction (sign of BACKEMF_CAL_SPIN_TARGET)
extern control_law_t backemf_cal_bidir_ctrl_law; // fwd leg -> drain -> reverse leg, one buffer —
                                                 // a complete fwd+rev dataset in one run
