#include <math.h>
#include "app_main.h"
#include "bldc_math.h"
#include "foc.h"
#include "open_loop_current_controller.h"
#include "stm32g4xx_hal_dac.h"
#include "typedefs.h"

#include "console.h"
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
#include "cal_store.h"
#include "can_comms.h"
#include "closed_loop_controller.h"
#include "fsm.h"
#include "ma702.h"


#include "OD.h"
#include "bootloader.h"
#include "CO_app_STM32.h"
#include "crash_record.h"

void iwdg_foreground_alive(void); // Core/Src/iwdg.c — foreground heartbeat for the IWDG gate
void fault_record_and_reset(u32 kind, u32 *stack_frame); // Core/Src/stm32g4xx_it.c

// ===== CCM SEU scrub =====
// Optimized builds run the entire fast loop — FOC, SVM, the trips themselves — from CCM SRAM,
// which has no parity as configured. Flash ECC can't see the code after the boot copy, so a
// radiation upset in CCM persists until reboot, silently corrupting live control code
// (flight-review RE-7). The pristine image sits at the flash LMA (ECC-protected): CRC both
// with the hardware CRC unit (~2200 words, tens of µs) and reboot through the startup copy
// loop on mismatch. Comparing against the LMA each pass (not a boot-time golden) also catches
// flash-side rot: single-bit flash errors are ECC-corrected to a stable value, double-bit
// raises the (now-handled) NMI.
extern u32 _sccmram, _eccmram, _siccmram; // linker: CCM VMA start/end, flash LMA start

static u32 hw_crc32_words(const u32 *data, u32 n_words) {
    CRC->CR = CRC_CR_RESET;
    for (u32 i = 0; i < n_words; i++)
        CRC->DR = data[i];
    return CRC->DR;
}

static void ccm_scrub_run() {
    static u32 last_scrub_ms = 0;
    u32 now = HAL_GetTick();
    if (now - last_scrub_ms < 5000u)
        return;
    last_scrub_ms = now;

    u32 n_words = ((u32) &_eccmram - (u32) &_sccmram) / 4u;
    if (n_words == 0u)
        return;
    u32 crc_ccm = hw_crc32_words((const u32 *) &_sccmram, n_words);
    u32 crc_flash = hw_crc32_words((const u32 *) &_siccmram, n_words);
    if (crc_ccm != crc_flash) {
        printf("CCM SCRUB MISMATCH: ccm 0x%08lx flash 0x%08lx — rebooting to re-copy\r\n",
               (unsigned long) crc_ccm, (unsigned long) crc_flash);
        console_flush();
        fault_record_and_reset(CRASH_KIND_CCM_SCRUB, NULL); // bridge off + breadcrumb + reset
    }
}

// (run_state() used to live here: an unbounded foreground busy-wait on FSM state — an error
// diversion made its target unreachable and wedged the loop. Zero callers; deleted.)

volatile u32 MAGIC_NUMBER = 0xFFEEDDCC;


__fn_noreturn void app_main() {
    __disable_irq();
    HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn, 0, 0); // top
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 1, 0); // FSM
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0); // CAN tick
    HAL_NVIC_SetPriority(TIM7_DAC_IRQn, 3, 0); // aux ADC
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 3, 0);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
    // dma.c (generated) leaves channels 2–5 at priority 0, tying with the fast loop — re-rank
    // them all to match the documented ladder (HRTIM 0 > FSM 1 > CAN tick 2 > DMA/aux 3).
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 3, 0);

    __enable_irq();

    // Free-running cycle counter — feeds isr_cycles_last/max (control_ll.c) so the fast-loop
    // budget can be checked live over SWD.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_CRC_CLK_ENABLE(); // hardware CRC unit — used by the CCM SEU scrub

    __maybe_unused volatile u32 n = MAGIC_NUMBER;

    blank_console();
    printf("        THIS FIRMWARE IS DEDICATED        \r\n");
    printf("     TO THE BRAVE MUJAHIDEEN FIGHTERS     \r\n");
    printf("             OF AFGHANISTAN               \r\n");

    //printf("Build2\r\n");

    // The bootloader's exit path (System_DeInit -> HAL_RCC_DeInit) clears the hardware flags
    // (RCC_CSR RMVF) before this app ever runs, so it snapshots RCC->CSR at its entry and
    // parks it in a magic-tagged mailbox at the top of RAM2 — untouched by either image's
    // .data/.bss (see bootloader/Src/main.c, keep address+magic in sync). Fall back to the
    // live register if the mailbox is absent (old bootloader / direct-boot debug session).
    // PIN tags along on most internal resets (NRST is pulled low), so the interesting bit is
    // what accompanies it: IWDG = starved watchdog (fast-loop overrun or a spun fault handler),
    // SOFT = NVIC_SystemReset (reboot SDO, option-byte write), OBL = option-byte reload.
    volatile u32 *const reset_cause_mailbox = (volatile u32 *) 0x20017FF8;
    u32 rcc_csr;
    if (reset_cause_mailbox[0] == 0x52435352u) {
        // "RCSR"
        rcc_csr = reset_cause_mailbox[1];
        reset_cause_mailbox[0] = 0; // consume
    } else {
        rcc_csr = RCC->CSR;
    }
    printf("Reset cause:%s%s%s%s%s%s%s\r\n",
           (rcc_csr & RCC_CSR_LPWRRSTF) ? " LPWR" : "",
           (rcc_csr & RCC_CSR_WWDGRSTF) ? " WWDG" : "",
           (rcc_csr & RCC_CSR_IWDGRSTF) ? " IWDG" : "",
           (rcc_csr & RCC_CSR_SFTRSTF) ? " SOFT" : "",
           (rcc_csr & RCC_CSR_BORRSTF) ? " BOR" : "",
           (rcc_csr & RCC_CSR_OBLRSTF) ? " OBL" : "",
           (rcc_csr & RCC_CSR_PINRSTF) ? " PIN" : "");
    __HAL_RCC_CLEAR_RESET_FLAGS();

    // Boot counter (RAM2 cell, survives warm resets, lost on power cycle): turns "the wheel
    // rebooted" into "the wheel has rebooted N times since power-on" — the boot-loop signature.
    noinit_cell_set(BOOT_COUNT_CELL,
                    noinit_cell_valid(BOOT_COUNT_CELL) ? noinit_cell_get(BOOT_COUNT_CELL) + 1u : 1u);
    printf("Boot count since power-on: %lu\r\n", (unsigned long) noinit_cell_get(BOOT_COUNT_CELL));

    // Crash record from the fault handlers (left in place until the next fault overwrites it,
    // so it stays readable over CAN via 0x4006 — see publish_boot_diagnostics).
    if (CRASH_RECORD->magic == CRASH_RECORD_MAGIC) {
        printf("LAST FAULT: kind %lu cfsr 0x%08lx hfsr 0x%08lx pc 0x%08lx lr 0x%08lx errs 0x%08lx\r\n",
               (unsigned long) CRASH_RECORD->kind, (unsigned long) CRASH_RECORD->cfsr,
               (unsigned long) CRASH_RECORD->hfsr, (unsigned long) CRASH_RECORD->stacked_pc,
               (unsigned long) CRASH_RECORD->stacked_lr, (unsigned long) CRASH_RECORD->controller_errors);
    }

    printf("Checking option bytes...\r\n");


    bool bytes_ok = check_option_bytes_ok();

    if (!bytes_ok) {
        // Bounded enforcement: a chip where the write never sticks used to reboot-loop
        // forever — wearing OB flash (~10k cycles) and never reaching CAN (flight-review
        // P2-8). After 3 attempts since power-on, fly with mismatched OBs: boot continues,
        // CAN comes up, and the BAD lines above say why. (The flight harness straps PB8 low,
        // so even wrong boot bits can't divert the boot path.)
        u32 ob_attempts = noinit_cell_get(OB_ATTEMPT_CELL);
        if (ob_attempts >= 3u) {
            printf("Option bytes still mismatched after %lu write attempts — continuing anyway\r\n",
                   (unsigned long) ob_attempts);
        } else {
            noinit_cell_set(OB_ATTEMPT_CELL, ob_attempts + 1u);
            printf("Writing option bytes (attempt %lu/3)...\r\n", (unsigned long) (ob_attempts + 1u));
            console_flush(); // success resets inside OB_Launch — flush first or lose the tail
            write_option_bytes();
            // Only reachable on write FAILURE (success = system reset inside OB_Launch):
            printf("Option byte write FAILED\r\n");
            console_flush();
            HAL_NVIC_SystemReset();
        }
    } else {
        noinit_cell_clear(OB_ATTEMPT_CELL);
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
    for (u8 i = 1; i < 8; i++) {
        volatile f32 magic = wrap_pm_pi(*(f32 *) &CORDIC_MAGIC[i]);
        while (magic > 1e2) magic /= 2.f;
        sincos_t cordic = cordic_sincos_f32(magic);
        sincos_t cmath = sincos_cmath(magic);
        cordic_ok = cordic_ok && (abs_f32(cordic.sin - cmath.sin) < 1e-3) && (abs_f32(cordic.cos - cmath.cos) < 1e-3);
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

    // MA732: make sure the sensor's NVM filter window matches config (writes once per sensor on
    // mismatch — option-bytes pattern). Must run before anything consumes angle data seriously.
    ma732_ensure_filter_window();

    printf("Setting up inverter...\r\n");
    inverter_setup();

    printf("Setting up encoder...\r\n");
    encoder_init();

    // Apply persisted encoder cal (offset/dir) from the flash cal page, if a valid blob exists.
    // Absence is normal pre-commissioning (bootstrap cals still work); host can SAVE/ERASE/
    // RESTORE over CAN via FSM states 18/19/20. See cal_store.h.
    printf("Loading cal store... ");
    cal_store_restore();

    printf("Starting aux ADC...\r\n");
    aux_adc_start();

    printf("Starting amp offset calibration...\r\n");
    inverter_amp_offset_correction();

    printf("Setting up controller...\r\n");
    controller_init();
    fsm_init();
    // (FSM tick starts AFTER canopen_init below: a transition in the gap would have called
    // into the CANopen layer with CO == NULL — latent, but free to close.)

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
        // Node ID comes from the dedicated CAN_ADDR0/1 straps (PB7/PB5, inputs). Strap-to-GND
        // = bit set; both open = wheel 1. The previous code read DEBUG1/2/3, but those are
        // push-pull outputs driven LOW (they're the gpio_debug scope pins), so every board
        // voted "all strapped" = wheel 4 (flight-review P0-1). The ioc configures CAN_ADDR as
        // NOPULL — enable internal pull-ups here (we never regenerate) so an open strap reads
        // high deterministically.
        GPIO_InitTypeDef addr_cfg = {
            .Pin = CAN_ADDR0_Pin | CAN_ADDR1_Pin,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_PULLUP,
        };
        HAL_GPIO_Init(CAN_ADDR0_GPIO_Port, &addr_cfg); // both pins live on GPIOB
        HAL_Delay(1);                                  // let the pulls charge an open net

        u32 addr0_high = 0, addr1_high = 0;
        for (u32 n = 0; n < 1000; n++) {
            addr0_high += (HAL_GPIO_ReadPin(CAN_ADDR0_GPIO_Port, CAN_ADDR0_Pin) != GPIO_PIN_RESET) ? 1 : 0;
            addr1_high += (HAL_GPIO_ReadPin(CAN_ADDR1_GPIO_Port, CAN_ADDR1_Pin) != GPIO_PIN_RESET) ? 1 : 0;
        }
        bool addr0_strapped = addr0_high < 500; // majority LOW = strapped to GND
        bool addr1_strapped = addr1_high < 500;

        // none=wheel1, ADDR0=wheel2, ADDR1=wheel3, both=wheel4
        wheel_number = (u8) (1u + (addr0_strapped ? 1u : 0u) + (addr1_strapped ? 2u : 0u));

        // Raw counts on the console so a marginal/floating strap is diagnosable at the bench
        printf("CAN_ADDR raw: addr0 %lu/1000 high, addr1 %lu/1000 high\r\n",
               (unsigned long) addr0_high, (unsigned long) addr1_high);

        u8 canids[] = {CANID_WHEEL1, CANID_WHEEL2, CANID_WHEEL3, CANID_WHEEL4};
        NODE_CANID = canids[wheel_number - 1];
    }

    printf("We are wheel number %u\r\n", wheel_number);

    printf("Setting up CAN with node ID 0x%X...\r\n", NODE_CANID);
    canopen_init(NODE_CANID);
    HAL_TIM_Base_Start_IT(&CAN_TIM_HANDLE);
    HAL_TIM_Base_Start_IT(&CONTROLLER_FSM_TIM_HANDLE); // FSM tick — only with CANopen alive

    // Reset cause + boot count + last-fault summary → the spare 0x4006 floats, so all of the
    // above is readable over the only in-flight interface (encoding in can-interface.md).
    publish_boot_diagnostics(rcc_csr);


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


    foc_print_gains(); // compile-time constants; printed once here instead of every arm tick

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
        iwdg_foreground_alive(); // watchdog gate heartbeat — see Core/Src/iwdg.c
        console_pump(); // drain ISR-context printf output (FSM transitions, cal results, ...)
        update_tpdo_od_vals();
        canopen_process();
        process_sdo_vals();
        fsm_run_pending_calstore_op(); // blocking flash ops live here, not in the FSM ISR
        ccm_scrub_run(); // every 5 s: CCM-resident control code vs its flash copy

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
