#pragma once

#include "typedefs.h"
#include "hrtim.h"
#include "periphmap.h"
#include "mathutil.h" // CLAMP, for the duty-cycle setters below

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

// Defined here as static inline (not in pwm.c): the hot caller is pwm_update_callback
// in control_ll.c — __fn_inline can't cross a TU boundary, so a .c-resident definition
// left every fast-loop tick taking a call from CCM out to a flash-resident copy
// (~90 cycles/tick, measured by PC-sampling profile 2026-06-09).
static __fn_flatten __fn_inline void hrtim_pwm_set_dcycle_u16(u16 a_dcycle, u16 b_dcycle, u16 c_dcycle) {
    //assert(a_dcycle >= PWM_CNTR_MIN_ALLOWED && a_dcycle <= PWM_CNTR_MAX_ALLOWED);
    //assert(b_dcycle >= PWM_CNTR_MIN_ALLOWED && b_dcycle <= PWM_CNTR_MAX_ALLOWED);
    //assert(c_dcycle >= PWM_CNTR_MIN_ALLOWED && c_dcycle <= PWM_CNTR_MAX_ALLOWED);

    a_dcycle = PWM_CNTR - a_dcycle;
    b_dcycle = PWM_CNTR - b_dcycle;
    c_dcycle = PWM_CNTR - c_dcycle;

    u16 a_compare = CLAMP(a_dcycle, PWM_CNTR_MIN_ALLOWED, PWM_CNTR_MAX_ALLOWED);
    u16 b_compare = CLAMP(b_dcycle, PWM_CNTR_MIN_ALLOWED, PWM_CNTR_MAX_ALLOWED);
    u16 c_compare = CLAMP(c_dcycle, PWM_CNTR_MIN_ALLOWED, PWM_CNTR_MAX_ALLOWED);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, a_compare);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, b_compare);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, c_compare);
}

static __fn_flatten __fn_inline void hrtim_pwm_set_dcycle_f32(const f32 a_dcycle, const f32 b_dcycle, const f32 c_dcycle) {
    //assert(a_dcycle >= 0.f && a_dcycle <= 1.f);
    //assert(b_dcycle >= 0.f && b_dcycle <= 1.f);
    //assert(c_dcycle >= 0.f && c_dcycle <= 1.f);
    hrtim_pwm_set_dcycle_u16(
            (u16) (a_dcycle * ((f32) PWM_CNTR)),
            (u16) (b_dcycle * ((f32) PWM_CNTR)),
            (u16) (c_dcycle * ((f32) PWM_CNTR))
    );
}