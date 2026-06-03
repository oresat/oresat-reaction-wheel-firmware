
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
void inverter_set_processed(f32 A, f32 B, f32 C, f32 Vbus);
