//
// Created by solomon on 5/24/23.
//

#pragma once

#include "bldc_math.h"
#include "control_ll.h"
#include "config.h"

void init_foc();

void foc_print_gains(); // boot-time gains banner (moved out of the in-ISR init_foc)
void set_iq_setpoint(f32 iq);
ab_vec_t get_alpha_beta_currents();
dq_vec_t get_idq_measured();
f32 get_foc_ibus();
f32 get_foc_vbus();

extern control_law_t foc_control_law;
extern volatile dq_vec_t v_current_ctrl_integral;

// 6f_e BEMF harmonic feedforward (see config.h): v_dq_ff = omega_e*(A*cos6θ + B*sin6θ) per axis.
typedef struct bemf_h6_coeff {
    f32 Ad, Bd; // d-axis cos/sin coefficients [V*s/rad]
    f32 Aq, Bq; // q-axis
} bemf_h6_coeff_t;

void foc_set_bemf_harmonic_coeffs(const bemf_h6_coeff_t *c);
void foc_get_bemf_harmonic_coeffs(bemf_h6_coeff_t *out);
extern volatile bool bemf_harmonic_ff_enabled; // runtime A/B toggle (SWD-pokeable)
