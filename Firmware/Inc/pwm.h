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
// Actual switching frequency: HRTIM counter clock (144 MHz × DLL_MUL) / PWM_CNTR / 2 (up-down).
// 144e6 * 32 / 24000 / 2 = 96 kHz.
#define PWM_SWITCHING_HZ   96000u

// PWM cycles between fast-loop ISRs. Matches HW RepetitionCounter+1 in hrtim.c.
// One ISR every 6 switching cycles → 16 kHz fast-loop ISR.
#define PWM_CYCLES_PER_ISR 6u

void hrtim_pwm_setup();

void hrtim_pwm_start();

void hrtim_pwm_stop();

void hrtim_pwm_set_dcycle_f32(f32 a_dcycle, f32 b_dcycle, f32 c_dcycle);

//void hrtim_pwm_set_dcycle_u16(u16 a_dcycle, u16 b_dcycle, u16 c_dcycle);