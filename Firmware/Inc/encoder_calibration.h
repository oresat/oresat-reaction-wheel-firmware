#pragma once

#include "control_ll.h"
#include "config.h"

extern control_law_t encoder_dir_cal_ctrl_law;
extern control_law_t encoder_phase_cal_ctrl_law;
extern control_law_t encoder_validation_ctrl_law;
extern control_law_t encoder_single_point_cal_ctrl_law;

//void init_encoder_dir_cal();
//void init_encoder_phase_cal();

//bool run_encoder_direction_find();
//bool run_encoder_offset_eccentricity_cal();
//f32 get_calibrated_encoder_phase();
//bool get_calibrated_encoder_direction();
//bool is_encoder_calibrated();
