#pragma once

#include "control_ll.h"

typedef struct {
    f32 pos;
    f32 vel;
} posvel_t;

void encoder_init();

void encoder_update(u16 meas);

posvel_t encoder_get_phase();

posvel_t encoder_get_pos_vel();

void encoder_set_phase_offset(f32 offset);
f32 encoder_get_phase_offset();

// Per-revolution angle-error LUT: correction [mech turns] indexed by RAW measured angle, applied
// (wrap-safe lerp) at the top of encoder_update() before the PLL — kills the MA702/732
// once-per-rev eccentricity that a scalar offset can't. Zero-mean by construction (the mean is
// the phase offset's job). Built by the bidir back-EMF cal, persisted by cal_store.
// Set/clear ONLY while disarmed: enabling steps enc_pos by up to ~LUT_MAX turns (harmless to the
// PLL's 0.5-turn stability bound, but don't step the angle under a live FOC loop).
#define ENC_LUT_N 128u
void encoder_set_lut(const f32 *table); // copies ENC_LUT_N entries, enables correction
void encoder_clear_lut();               // zero + disable
bool encoder_lut_enabled_get();
void encoder_lut_copy(f32 *dst);        // current table -> dst[ENC_LUT_N] (for persistence)

void encoder_set_direction(f32 dir);

f32 encoder_get_direction();

// Which calibrations the LIVE dir/offset values came from: set by a successful cal procedure
// or a flash restore, snapshotted+rolled back with the values on a failed/cancelled cal.
// Gates arming of the closed-loop states (fsm_state_requires_encoder_cal): running FOC on the
// power-on defaults (dir +1, offset 0) gives anything from zero-torque heating to
// velocity-loop positive feedback, depending on the true offset.
void encoder_set_dir_cal_valid(bool valid);
void encoder_set_offset_cal_valid(bool valid);
bool encoder_get_dir_cal_valid();
bool encoder_get_offset_cal_valid();
bool encoder_cal_valid(); // dir && offset

//bool get_phase_resistance_valid();
extern control_law_t encoder_test_control_law;
f32 encoder_get_multiturn_pos_raw();
f32 encoder_get_multiturn_pos_est();

