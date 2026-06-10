#pragma once
#include "control_ll.h"
#include "typedefs.h"

void set_torque_setpoint(f32 torque_nm);
void set_vel_setpoint(f32 vel);
void set_vel_ramp_setpoint(f32 vel_ramp);

extern control_law_t torque_ctrl_control_law;
extern control_law_t vel_ctrl_control_law;
extern control_law_t vel_ramp_ctrl_control_law;

