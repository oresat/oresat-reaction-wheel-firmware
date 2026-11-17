
#pragma once

#include "typedefs.h"

void inverter_setup();

void inverter_start();

void inverter_stop();

void inverter_adc_process_dma();

void inverter_get_phase_currents(f32 *A, f32 *B, f32 *C);

void inverter_get_vbus_voltage(f32 *Vbus);

void inverter_amp_offset_correction();

bool inverter_get_calibrated();

void inverter_blank_dma_buf();
void inverter_get_offset_correction(i16 *A, i16 *B, i16 *C);
void inverter_get_adc_raw(u32 *A, u32 *B, u32 *C, u32 *vbus);
void inverter_get_phase_voltages_raw(u32 *A, u32 *B, u32 *C); // injected rank 2 (enabled via MANUAL EDITs in adc.c); meaningful only while bridge is hi-Z
void inverter_set_processed(f32 A, f32 B, f32 C, f32 Vbus);

// Outcome of the boot-time shunt-amp offset cal (inverter_amp_offset_correction). The measured
// offsets [12-bit counts from 2048] are returned even when the cal was rejected.
typedef enum {
    INVERTER_OFFSET_CAL_NOT_RUN = 0,
    INVERTER_OFFSET_CAL_OK,
    INVERTER_OFFSET_CAL_REJECTED,  // an offset was outside ±50 mV; arming is refused
    INVERTER_OFFSET_CAL_TIMED_OUT, // no HRTIM ticks; arming is refused
} inverter_offset_cal_status_t;

inverter_offset_cal_status_t inverter_get_boot_offset_cal(i16 *A, i16 *B, i16 *C);
