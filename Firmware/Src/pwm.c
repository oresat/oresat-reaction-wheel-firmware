#include <math.h>
#include "pwm.h"
#include "stm32g4xx_hal_hrtim.h"
#include "util.h"
#include "mathutil.h"

void hrtim_pwm_setup() {

}

__fn_flatten __fn_inline void hrtim_pwm_start() {

    //HAL_HRTIM_UpdateDisable(&hhrtim1,HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E | HRTIM_TIMERUPDATE_F);
    hrtim_pwm_set_dcycle_f32(0.5f, 0.5f, 0.5f);

    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, 0);

    assert(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E) ==
           HAL_OK);

    assert(HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                                         HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                         HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                                         HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2
    ) == HAL_OK);
    assert(HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,
                                           HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E) ==
           HAL_OK);

    assert(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E) ==
           HAL_OK);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, PWM_CNTR_MAX_ALLOWED);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, PWM_CNTR_MAX_ALLOWED);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, PWM_CNTR_MAX_ALLOWED);

    HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E);

    //HAL_HRTIM_UpdateEnable(&hhrtim1,HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E | HRTIM_TIMERUPDATE_F);
}

__fn_flatten __fn_inline void hrtim_pwm_stop() {

    //HAL_HRTIM_UpdateDisable(&hhrtim1,HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E | HRTIM_TIMERUPDATE_F);

    assert(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E) ==
           HAL_OK);


    assert(HAL_HRTIM_WaveformCountStop(&hhrtim1,
                                       HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E) ==
           HAL_OK);

    assert(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E) ==
           HAL_OK);

    assert(HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                                        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                                        HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2
    ) == HAL_OK);

    //assert(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E | HRTIM_TIMERUPDATE_F) == HAL_OK);

    //u32 c_B, c_E, c_F;
    //c_B = __HAL_HRTIM_GETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B);
    //c_E = __HAL_HRTIM_GETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E);
    //c_F = __HAL_HRTIM_GETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F);

    //printf("B: %lu E: %lu F: %lu\r\n", c_B, c_E, c_F);

    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, 0);

    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_INACTIVE);
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, HRTIM_OUTPUTLEVEL_INACTIVE);
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, HRTIM_OUTPUTLEVEL_INACTIVE);
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, HRTIM_OUTPUTLEVEL_INACTIVE);
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, HRTIM_OUTPUTLEVEL_INACTIVE);
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2, HRTIM_OUTPUTLEVEL_INACTIVE);

    hrtim_pwm_set_dcycle_f32(0.5f, 0.5f, 0.5f);

    assert(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E) ==
           HAL_OK);
    assert(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E) ==
           HAL_OK);
    assert(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E) ==
           HAL_OK);

    //HAL_HRTIM_UpdateEnable(&hhrtim1,HRTIM_TIMERUPDATE_B | TIMERUPDATE_E | TIMERUPDATE_F);
    //HAL_HRTIM_UpdateEnable(&hhrtim1,HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E | HRTIM_TIMERUPDATE_F);

    //assert(HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 1 - 1) == HAL_OK);
    //assert(HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 1 - 1) == HAL_OK);
    //assert(HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, 1 - 1) == HAL_OK);
    //assert(HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_4, 1 - 1) == HAL_OK);
}

__fn_flatten __fn_inline void hrtim_pwm_set_dcycle_u16(u16 a_dcycle, u16 b_dcycle, u16 c_dcycle) {
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

__fn_flatten __fn_inline void hrtim_pwm_set_dcycle_f32(const f32 a_dcycle, const f32 b_dcycle, const f32 c_dcycle) {
    //assert(a_dcycle >= 0.f && a_dcycle <= 1.f);
    //assert(b_dcycle >= 0.f && b_dcycle <= 1.f);
    //assert(c_dcycle >= 0.f && c_dcycle <= 1.f);
    //f32 a_dcycle_local = CLAMP(a_dcycle, 0.f, 1.f);
    //f32 b_dcycle_local = CLAMP(b_dcycle, 0.f, 1.f);
    //f32 c_dcycle_local = CLAMP(c_dcycle, 0.f, 1.f);
    hrtim_pwm_set_dcycle_u16(
            (u16) (a_dcycle * ((f32) PWM_CNTR)),
            (u16) (b_dcycle * ((f32) PWM_CNTR)),
            (u16) (c_dcycle * ((f32) PWM_CNTR))
    );
}
