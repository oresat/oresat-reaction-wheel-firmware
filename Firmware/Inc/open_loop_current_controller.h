#pragma once

#include "typedefs.h"
#include "control_ll.h"
#include "bldc_math.h"

void init_open_loop_current_controller();
void set_open_loop_current_setpoint(ab_vec_t current_setpoint);
ab_vec_t get_open_loop_currents();
f32 get_open_loop_ibus();

extern control_law_t open_loop_ctrl_law;
