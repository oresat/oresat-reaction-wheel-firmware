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

void encoder_set_direction(f32 dir);

f32 encoder_get_direction();

//bool get_phase_resistance_valid();
extern control_law_t encoder_test_control_law;
f32 encoder_get_multiturn_pos_raw();
f32 encoder_get_multiturn_pos_est();

