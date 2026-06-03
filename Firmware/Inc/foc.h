//
// Created by solomon on 5/24/23.
//

#pragma once

#include "bldc_math.h"
#include "control_ll.h"
#include "config.h"

void init_foc();
void set_iq_setpoint(f32 iq);
ab_vec_t get_alpha_beta_currents();
dq_vec_t get_idq_measured();
f32 get_foc_ibus();
f32 get_foc_vbus();

extern control_law_t foc_control_law;
extern volatile dq_vec_t v_current_ctrl_integral;
