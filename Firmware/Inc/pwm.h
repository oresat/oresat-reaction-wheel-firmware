#pragma once

#include "typedefs.h"
#include "hrtim.h"
#include "periphmap.h"

#define PWM_CNTR 24000
#define HRTIM_DLL_MUL 32

// Up-down mode requires output set/reset compare values to stay >= 0xC0 ticks from the period
// boundaries at CKPSC=0/MUL32 (RM0440 p918: max compare = TIMx_PER - 0xC0); a compare nearer the
// crest/valley can drop its set/reset event. 0xC0 = 192 = 6*HRTIM_DLL_MUL -> clamps duty to
// [192, 23808]. (Was 4*DLL_MUL=128, i.e. 64 ticks inside the forbidden zone.)
#define PWM_CNTR_MARGIN (6*(HRTIM_DLL_MUL))
#define PWM_CNTR_MAX_ALLOWED (PWM_CNTR - PWM_CNTR_MARGIN)
#define PWM_CNTR_MIN_ALLOWED (PWM_CNTR_MARGIN)
// Actual switching frequency: HRTIM counter clock (144 MHz × DLL_MUL) / PWM_CNTR / 2 (up-down).
// 144e6 * 32 / 24000 / 2 = 96 kHz.
#define PWM_SWITCHING_HZ   96000u

// PWM cycles between fast-loop ISRs. MUST match HW RepetitionCounter+1 in hrtim.c
// (MX_HRTIM1_Init, the shared pTimeBaseCfg.RepetitionCounter for TIMER_A/B/E) — these are
// two independent constants with no compile-time link; if they drift, TIMER_A's ISR/duty-latch
// rate and the loop-period math here disagree and the FOC dt is silently wrong.
// One ISR every 3 switching cycles → 32 kHz fast-loop ISR (was 6 → 16 kHz).
#define PWM_CYCLES_PER_ISR 3u

void hrtim_pwm_setup();

void hrtim_pwm_start();

void hrtim_pwm_stop();

// Disable/enable only the six bridge outputs (hi-Z the bridge) while leaving the HRTIM counters,
// repetition ISR, and ADC triggers running. For back-EMF coast-down measurement; see
// docs/backemf-coastdown-implementation.md.
void hrtim_outputs_disable();

void hrtim_outputs_enable();

void hrtim_pwm_set_dcycle_f32(f32 a_dcycle, f32 b_dcycle, f32 c_dcycle);

//void hrtim_pwm_set_dcycle_u16(u16 a_dcycle, u16 b_dcycle, u16 c_dcycle);