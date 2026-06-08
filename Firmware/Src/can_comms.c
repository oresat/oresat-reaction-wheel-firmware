#include "can_comms.h"

#include "aux_adc.h"
#include "closed_loop_controller.h"
#include "control_ll.h"
#include "OD.h"
#include "periphmap.h"
#include "CO_app_STM32.h"
#include "encoder.h"
#include "foc.h"
#include "foc_capture.h"
#include "inverter.h"

CANopenNodeSTM32 canOpenNodeSTM32;

void canopen_init(u8 node_id) {
    //HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_TX_COMPLETE,FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
    //HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_,FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);

    canOpenNodeSTM32.CANHandle = &hfdcan1;
    canOpenNodeSTM32.HWInitFunction = MX_FDCAN1_Init;
    canOpenNodeSTM32.timerHandle = &CAN_TIM_HANDLE;
    canOpenNodeSTM32.desiredNodeID = node_id;
    canOpenNodeSTM32.baudrate = 1000;
    canopen_app_init(&canOpenNodeSTM32);

    init_od();
    init_sdo();

    //HAL_TIM_Base_Start(&CAN_TIM_HANDLE); // CANOpen stuff takes care of this for us
}

// Calls at ~1kHz by TIM4
void canopen_1khz_isr() {
    canopen_app_interrupt();
}

void canopen_send_test_message() {
}

// Should be called in a loop-ish at ~100Hz
void canopen_process() {
    canopen_app_process();
}

void init_od() {
    update_control_status_od(FSM_STATE_IDLE, 0, 0);
    update_motor_vel_current_od(0.f, 0.f);
    update_bus_volt_current_od(0.f, 0.f);
    update_rxw_temp_od(0, 0, 0);
}

void update_control_status_od(fsm_state_t fsm_state, u32 controller_errors, u8 procedure_result) {
    OD_RAM.x4000_rxw_ctrl_stat.current_state = fsm_state;
    OD_RAM.x4000_rxw_ctrl_stat.errors = controller_errors;
    OD_RAM.x4000_rxw_ctrl_stat.procedure_result = procedure_result;
}

void update_motor_vel_current_od(f32 vel, f32 iq) {
    OD_RAM.x4001_rxw_motor_vel_current.motor_vel = vel;
    OD_RAM.x4001_rxw_motor_vel_current.motor_current = iq;
}

void update_bus_volt_current_od(f32 ibus, f32 vbus) {
    OD_RAM.x4002_rxw_bus_volt_current.bus_current = ibus;
    OD_RAM.x4002_rxw_bus_volt_current.bus_voltage = vbus;
}

void update_rxw_temp_od(i16 temp1, i16 temp2, i16 temp3) {
    OD_RAM.x4003_rxw_temp.temp_1 = temp1;
    OD_RAM.x4003_rxw_temp.temp_2 = temp2;
    OD_RAM.x4003_rxw_temp.temp_3 = temp3;
}

void init_sdo() {
    OD_RAM.x4004_rxw_requested_state.req_state = FSM_STATE_IDLE;
    OD_RAM.x4005_rxw_setpoint.setpoint_1 = 0.f;
    OD_RAM.x4005_rxw_setpoint.setpoint_2 = 0.f;
    OD_RAM.x4006_rxw_reserved.reserved_1 = 0.f;
    OD_RAM.x4006_rxw_reserved.reserved_2 = 0.f;
    OD_RAM.x4007_rxw_reboot.reboot_request = 0;
}

fsm_state_t get_sdo_requested_state() {
    return OD_RAM.x4004_rxw_requested_state.req_state;
}

void set_sdo_requested_state(fsm_state_t state) {
    OD_RAM.x4004_rxw_requested_state.req_state = (u8) state;
}

bool get_sdo_reboot_request() {
    return OD_RAM.x4007_rxw_reboot.reboot_request == 1;
}

f32 get_sdo_setpoint_1() {
    return OD_RAM.x4005_rxw_setpoint.setpoint_1;
}

f32 get_sdo_setpoint_2() {
    return OD_RAM.x4005_rxw_setpoint.setpoint_2;
}

f32 get_sdo_reserved_1() {
    return OD_RAM.x4006_rxw_reserved.reserved_1;
}

f32 get_sdo_reserved_2() {
    return OD_RAM.x4006_rxw_reserved.reserved_2;
}

// Should be called quite often, ideally consistently >100Hz (maybe in main loop w/ canopen_app_process())
void update_tpdo_od_vals() {
    // TODO bus and motor vals only show up in FOC / closed loop control
    fsm_state_t current_state = fsm_get_current_state();
    u32 controller_errors = controller_get_errors();
    u8 procedure_result = fsm_get_last_state_exit_ok();

    f32 motor_vel = 0.f;
    f32 motor_current = 0.f;
    f32 bus_voltage = 0.f;
    f32 bus_current = 0.f;

    if (current_state == FSM_STATE_POS_CONTROL ||
        current_state == FSM_STATE_VEL_CONTROL ||
        current_state == FSM_STATE_TORQUE_CONTROL ||
        current_state == FSM_STATE_VEL_RAMP_CONTROL) {
        motor_vel = encoder_get_pos_vel().vel;
        motor_current = get_idq_measured().q;
        bus_voltage = get_foc_vbus();
        bus_current = get_foc_ibus();
    }

    i16 therm1 = aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm1_sns);
    i16 therm2 = aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm2_sns);
    i16 therm3 = aux_adc_conv_thermint_cDegC(aux_adc_last_reading_raw.thermint_sns);

    update_control_status_od(current_state, controller_errors, procedure_result);
    update_motor_vel_current_od(motor_vel, motor_current);
    update_bus_volt_current_od(bus_current, bus_voltage);
    update_rxw_temp_od(therm1, therm2, therm3);
}

static fsm_state_t last_sdo_requested_state = FSM_STATE_IDLE;
static f32 last_sdo_setpoint_1 = 0.f;
static f32 last_sdo_setpoint_2 = 0.f;
static f32 last_sdo_reserved_1 = 0.f;
static f32 last_sdo_reserved_2 = 0.f;

// Should be called quite often, ideally consistently >100Hz (maybe in main loop w/ canopen_app_process())
void process_sdo_vals() {
    fsm_state_t sdo_requested_state = (fsm_state_t) OD_RAM.x4004_rxw_requested_state.req_state;
    f32 sdo_setpoint_1 = OD_RAM.x4005_rxw_setpoint.setpoint_1;
    f32 sdo_setpoint_2 = OD_RAM.x4005_rxw_setpoint.setpoint_2;
    f32 sdo_reserved_1 = OD_RAM.x4006_rxw_reserved.reserved_1;
    f32 sdo_reserved_2 = OD_RAM.x4006_rxw_reserved.reserved_2;
    u8 sdo_reboot_request = OD_RAM.x4007_rxw_reboot.reboot_request;

    if (sdo_requested_state != last_sdo_requested_state) {
        last_sdo_requested_state = sdo_requested_state;
        fsm_request_state(sdo_requested_state);
    }

    if (sdo_setpoint_1 != last_sdo_setpoint_1) {
        last_sdo_setpoint_1 = sdo_setpoint_1;
        fsm_state_t current_state = fsm_get_current_state();
        switch (current_state) {
            case FSM_STATE_TORQUE_CONTROL:
                set_torque_setpoint(sdo_setpoint_1);
                break;
            case FSM_STATE_VEL_CONTROL:
                set_vel_setpoint(sdo_setpoint_1);
                break;
            case FSM_STATE_POS_CONTROL:
                set_pos_setpoint(sdo_setpoint_1);
                break;
            case FSM_STATE_VEL_RAMP_CONTROL:
                set_vel_ramp_setpoint(sdo_setpoint_1);
            default: break;
        }
    }

    if (sdo_setpoint_2 != last_sdo_setpoint_2) {
        last_sdo_setpoint_2 = sdo_setpoint_2;
    }
    if (sdo_reserved_1 != last_sdo_reserved_1) {
        last_sdo_reserved_1 = sdo_reserved_1;
        // Non-zero write to reserved_1 arms a one-shot FOC capture run.
        // See Firmware/Inc/foc_capture.h + tools/foc_capture_dump.sh for readout.
        if (sdo_reserved_1 != 0.f) foc_capture_arm();
    }
    if (sdo_reserved_2 != last_sdo_reserved_2) {
        last_sdo_reserved_2 = sdo_reserved_2;
    }

    if (sdo_reboot_request == 1) {
        controller_disarm();
        HAL_NVIC_SystemReset();
    }
}

//extern CANopenNodeSTM32 *canopenNodeSTM32;

// Update TPDO comm parameter event timer. Will come into effect on next NMT 130 RESET_COMMUNICATION command.
void set_tpdo_send_rate(u32 timer_ms) {
    u16 timer_cnt = timer_ms;
    OD_RAM.x1800_tpdo_1_communication_parameters.event_timer = timer_cnt;
    OD_RAM.x1801_tpdo_2_communication_parameters.event_timer = timer_cnt;
    OD_RAM.x1802_tpdo_3_communication_parameters.event_timer = timer_cnt;
    OD_RAM.x1803_tpdo_4_communication_parameters.event_timer = timer_cnt;
    canopen_app_resetCommunication();
}