#pragma once

#include "control_ll.h"
#include "config.h"

extern control_law_t resistance_cal_ctrl_law;

void init_resistance_cal();

f32 get_phase_resistance();

extern control_law_t inductance_cal_ctrl_law;

void init_inductange_cal();

f32 get_phase_inductance();
