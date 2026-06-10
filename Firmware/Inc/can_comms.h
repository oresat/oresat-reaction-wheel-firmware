#pragma once

#include "typedefs.h"
#include "fsm.h"

void canopen_init(u8 node_id);

void canopen_1khz_isr();

void canopen_process();

void update_control_status_od(fsm_state_t fsm_state, u32 controller_errors, u8 procedure_result);

void update_motor_vel_current_od(f32 vel, f32 iq);

void update_bus_volt_current_od(f32 ibus, f32 vbus);

void update_rxw_temp_od(i16 temp1, i16 temp2, i16 temp3);

void update_tpdo_od_vals();
void process_sdo_vals();

void init_od();

void init_sdo();

void init_od_extensions();

// Last disarmed velocity estimate [rev/s mech] from the foreground SPI sampler — the bumpless
// seed for vel/vel-ramp law entry (the PLL is frozen while disarmed).
f32 get_disarmed_vel();

void publish_boot_diagnostics(u32 rcc_csr);

fsm_state_t get_sdo_requested_state();

void set_sdo_requested_state(fsm_state_t state);

bool get_sdo_reboot_request();

f32 get_sdo_setpoint_1();

f32 get_sdo_setpoint_2();

f32 get_sdo_reserved_1();

f32 get_sdo_reserved_2();

void set_tpdo_send_rate(u32 timer_ms);