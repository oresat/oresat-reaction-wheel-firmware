#pragma once

#include "typedefs.h"
#include "hrtim.h"
#include "periphmap.h"

#define PWM_CNTR 24000
#define HRTIM_DLL_MUL 32

// 4x HRTIM clock freq, so that up/down set/reset can be changed
#define PWM_CNTR_MARGIN (4*(HRTIM_DLL_MUL))
#define PWM_CNTR_MAX_ALLOWED (PWM_CNTR - PWM_CNTR_MARGIN)
#define PWM_CNTR_MIN_ALLOWED (PWM_CNTR_MARGIN)
#define PWM_FREQ 192000
//#define PWM_RCR 12
#define PWM_RCR 6

void hrtim_pwm_setup();

void hrtim_pwm_start();

void hrtim_pwm_stop();

void hrtim_pwm_set_dcycle_f32(f32 a_dcycle, f32 b_dcycle, f32 c_dcycle);

//void hrtim_pwm_set_dcycle_u16(u16 a_dcycle, u16 b_dcycle, u16 c_dcycle);