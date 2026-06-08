#include <math.h>
#include "app_main.h"
#include "bldc_math.h"
#include "foc.h"
#include "open_loop_current_controller.h"
#include "stm32g4xx_hal_dac.h"
#include "typedefs.h"

#include "printf.h"
#include "stm32g4xx_hal.h"

#include "usart.h"
#include "util.h"
#include "pwm.h"
#include "aux_adc.h"
#include "inverter.h"
#include "mathutil.h"
#include "control_ll.h"
#include "encoder.h"
#include "motor_calibration.h"
#include "encoder_calibration.h"
#include "dac.h"
#include "dsp/controller_functions.h"
#include "can_comms.h"
#include "closed_loop_controller.h"
#include "fsm.h"
#include "ma702.h"


#include "OD.h"
#include "bootloader.h"
#include "CO_app_STM32.h"

bool run_state(fsm_state_t state) {
    printf("Starting %s\r\n", state_names[state].string);
    fsm_request_state(state);

    while (fsm_get_current_state() != state);
    while (fsm_get_current_state() == state);

    if (fsm_get_last_state_exit_ok())
        return true;

    char err[256];
    STRINGIFY_CONTROLLER_ERRORS(err, 256, controller_get_errors());
    printf("Controller errors: %s\r\n", err);

    return false;
}

volatile u32 MAGIC_NUMBER = 0xFFEEDDCC;


__fn_noreturn void app_main() {

    __disable_irq();
    HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn, 0, 0);   // top
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 1, 0); // FSM
    HAL_NVIC_SetPriority(TIM4_IRQn,        2, 0);   // CAN tick
    HAL_NVIC_SetPriority(TIM7_DAC_IRQn,    3, 0);   // aux ADC
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,  3, 0);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn,  3, 0);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);

    __enable_irq();

    __maybe_unused volatile u32 n = MAGIC_NUMBER;

    blank_console();
    printf("        THIS FIRMWARE IS DEDICATED        \r\n");
    printf("     TO THE BRAVE MUJAHIDEEN FIGHTERS     \r\n");
    printf("             OF AFGHANISTAN               \r\n");

    printf("Checking option bytes...\r\n");


    bool bytes_ok = check_option_bytes_ok();

    if (!bytes_ok) {
        printf("Writing option bytes...\r\n");
        bool wrote_ok = write_option_bytes();
        printf("Writing option bytes %s\r\n", wrote_ok ? "SUCCESS" : "FAIL");
        check_option_bytes_ok();
        HAL_NVIC_SystemReset();
    }

    // THIS IS ALL HANDLED IN THE BOOTLOADER SECTION
    //printf("Checking BOOT0 pin...\r\n");
    //init_boot0();
    //bool boot0_state = get_boot0_pin_state();
    //printf("BOOT0 State: %s\r\n", boot0_state ? "BOOTLOADER" : "RUN");
    //if (boot0_state) {
    //    printf("Jumping to USART bootloader\r\n");
    //    jump_to_bootloader();
    //}

    printf("Starting DAC and CORDIC...\r\n");

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
    HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

    cordic_setup(CORDIC_MODE_SINCOS);
    printf("Testing CORDIC...\r\n");

    // See
    u32 CORDIC_MAGIC[8] = {
        1447508071, 1412584040, 1648913530, 1279476340,
        1699955305, 1515681910, 1684887147, 1130839357
    };
    bool cordic_ok = true;
    for (u8 i = 0; i < 8; i++) {
        f32 magic = wrap_pm_pi(*((f32 *) CORDIC_MAGIC));
        sincos_t cordic = cordic_sincos_f32(magic);
        sincos_t cmath = sincos_cmath(magic);
        cordic_ok = (abs_f32(cordic.sin - cmath.sin) < 1e-3) && (abs_f32(cordic.cos - cmath.cos) < 1e-3);
    }

    printf(cordic_ok ? "CORDIC OK\r\n" : "CORDIC FAIL\r\n");

    set_nCS(true);
    HAL_Delay(100);

    //char buf[] = "meowww\r\n";
    //uart_transmit(USART2,buf,sizeof(buf));


    printf("Setting up encoder...\r\n");
    //as5047p_preheat();
    //as5047p_preheat();
    ma702_preheat();
    ma702_preheat();
    ma702_preheat();
    ma702_preheat();

    printf("Setting up inverter...\r\n");
    inverter_setup();

    printf("Setting up encoder...\r\n");
    encoder_init();

    printf("Starting aux ADC...\r\n");
    aux_adc_start();

    printf("Starting amp offset calibration...\r\n");
    inverter_amp_offset_correction();

    printf("Setting up controller...\r\n");
    controller_init();
    fsm_init();
    HAL_TIM_Base_Start_IT(&CONTROLLER_FSM_TIM_HANDLE);

    //HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2000);
    //dac_debug0_V(2.f);
    //*(volatile u32 *) ((u32) hdac1.Instance + DAC_DHR12R1_ALIGNMENT(DAC_ALIGN_12B_R)) = 2000;
    //*(volatile u32 *) (hdac1.Instance + DAC_DHR12R1_ALIGNMENT(DAC_ALIGN_12B_R)) = 2000;
    //volatile u32 tmp = 0;
    //tmp = (u32) (&hdac1)->Instance;
    //tmp += DAC_DHR12R1_ALIGNMENT(DAC_ALIGN_12B_R);
    //*(volatile u32 *) tmp = 2000;

    printf("Reading CAN ID strapping...\r\n");
    u8 NODE_CANID;
    u8 wheel_number; {
        HAL_GPIO_WritePin(DEBUG0_GPIO_Port,DEBUG0_Pin, GPIO_PIN_RESET);
        u32 debug1_avg = 0.f;
        u32 debug2_avg = 0.f;
        u32 debug3_avg = 0.f;
        for (u32 n = 0; n < 1000; n++) {
            debug1_avg += (HAL_GPIO_ReadPin(DEBUG1_GPIO_Port,DEBUG1_Pin) != GPIO_PIN_RESET) ? 1 : 0;
            debug2_avg += (HAL_GPIO_ReadPin(DEBUG2_GPIO_Port,DEBUG2_Pin) != GPIO_PIN_RESET) ? 1 : 0;
            debug3_avg += (HAL_GPIO_ReadPin(DEBUG3_GPIO_Port,DEBUG3_Pin) != GPIO_PIN_RESET) ? 1 : 0;
        }
        bool debug1 = debug1_avg < 500;
        bool debug2 = debug2_avg < 500;
        bool debug3 = debug3_avg < 500;

        if (debug3 && debug2 && debug1) // Wheel 4
            wheel_number = 4;
        else if (debug2 && debug1) // Wheel 3
            wheel_number = 3;
        else if (debug1) // Wheel 2
            wheel_number = 2;
        else // Wheel 1
            wheel_number = 1;

        u8 canids[] = {CANID_WHEEL1, CANID_WHEEL2, CANID_WHEEL3, CANID_WHEEL4};
        NODE_CANID = canids[wheel_number - 1];
    }

    printf("We are wheel number %u\r\n", wheel_number);

    printf("Setting up CAN with node ID 0x%X...\r\n", NODE_CANID);
    canopen_init(NODE_CANID);
    HAL_TIM_Base_Start_IT(&CAN_TIM_HANDLE);


    printf("Init complete!\r\n\r\n");


    //fsm_request_state(FSM_STATE_SHITTY_OFFSET_CAL);
    //HAL_Delay(10);
    //while (fsm_get_current_state() != FSM_STATE_IDLE)
    //    HAL_Delay(10);

    //fsm_request_state(FSM_STATE_ENCODER_VALIDATION);
    //HAL_Delay(10);
    //while (fsm_get_current_state() != FSM_STATE_IDLE)
    //    HAL_Delay(10);

    //fsm_request_state(FSM_STATE_ENCODER_OFFSET_CAL);
    //HAL_Delay(10);
    //while (fsm_get_current_state() != FSM_STATE_IDLE)
    //    HAL_Delay(10);

    //fsm_request_state(FSM_STATE_ENCODER_VALIDATION);
    //HAL_Delay(10);
    //while (fsm_get_current_state() != FSM_STATE_IDLE)
    //    HAL_Delay(10);


    //fsm_request_state(FSM_STATE_OPEN_LOOP_CONTROL);
    //set_open_loop_current_setpoint((ab_vec_t) {1.0f, 0.0f});


    printf("Entering main tpdo update loop...\r\n");
    //fsm_request_state(FSM_STATE_ENCODER_DIR_CAL);

    set_tpdo_send_rate(5000);

    //fsm_request_state(FSM_STATE_ENCODER_DIR_CAL);
    //HAL_Delay(10);
    //while (fsm_get_current_state() != FSM_STATE_IDLE)
    //    HAL_Delay(10);

    //fsm_request_state(FSM_STATE_VEL_RAMP_CONTROL);
    //HAL_Delay(10);
    //set_vel_ramp_setpoint(400.f);

    //u32 t0 = HAL_GetTick() - 1;
    //f32 j = 0;
    //HAL_Delay(10);
    while (1) {
        update_tpdo_od_vals();
        canopen_process();
        process_sdo_vals();

        //if (HAL_GetTick() - t0 > 1) {
        //    set_open_loop_current_setpoint(
        //        inv_park_transform(
        //            (dq_vec_t) { 2.0f ,0.f}, j
        //            )
        //    );
        //    t0 = HAL_GetTick();
        //    j += 0.05f;
        //    if (j > M_PI_f32*2) j -= M_PI_f32*2;
        //}

        //if (HAL_GetTick()-t0 > 5000) {
        //    t0 = HAL_GetTick();
        //    set_tpdo_send_rate(toggle ? 100 : 1000);
        //    printf("rate: %d\r\n", toggle ? 100 : 1000);
        //    toggle = !toggle;
        //}

        //j++;
        //if ((HAL_GetTick() - t0) > 1000) {
        //    printf("%u\r\n", j );
        //    t0 = HAL_GetTick();
        //    j = 0;
        //}
        HAL_Delay(1);
    }
}
