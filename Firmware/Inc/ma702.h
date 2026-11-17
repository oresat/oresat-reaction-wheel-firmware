#pragma once

#include "typedefs.h"
#include "periphmap.h"

static __fn_inline __fn_flatten void set_nCS(bool level) {
    if (level)
        spi_ncs.port->BSRR = (u32) spi_ncs.pin;
    else
        spi_ncs.port->BRR = (u32) spi_ncs.pin;
}


void ma702_preheat();

// MA732 only: verify the sensor's non-volatile filter-window register matches
// MA732_FILTER_WINDOW_REG (config.h), writing it (20 ms NVM op + readback) on mismatch.
// Init-context only (blocking). Returns false if the write failed to verify.
bool ma732_ensure_filter_window();

// Bounded (HAL-free, never hangs) MA732 register read for runtime use. Caller must hold the SPI
// exclusively: controller disarmed AND interrupts masked. Returns false if the SPI never answered.
bool ma732_read_reg_bounded(u8 addr, u8 *val);

#define MA732_REG_FW 0xE          // filter window (config.h MA732_FILTER_WINDOW_REG)
#define MA732_REG_FIELD_FLAGS 0x1B // MGH (bit 7) / MGL (bit 6) magnetic-field flags

typedef enum ma702_field_strength_flag {
    FIELD_HIGH, FIELD_LOW, FIELD_IN_RANGE, FIELD_UNK
} ma702_field_strength_flag_t;

ma702_field_strength_flag_t ma702_read_field_strength_flags();

u8 ma702_get_estimated_field_strength();

u16 ma702_read_angle();

// (The old ma702_fast_read_start/_finish inline twins of the macros below were deleted: the
// dead _finish masked `& 0x3FFF` WITHOUT the `>> 2` shift the live macro applies — swapping it
// in would have silently scrambled every angle. The macros are the only fast-read path.)

#define __MA702_FAST_ANGLE_READ_START() ({\
    set_nCS(0);\
    CLEAR_BIT((&ENCODER_SPI_HANDLE)->Instance->CR2, SPI_RXFIFO_THRESHOLD);\
    (&ENCODER_SPI_HANDLE)->Instance->DR = 0;\
})

#define __MA702_FAST_ANGLE_READ_FINISH() ({\
    while (__HAL_SPI_GET_FLAG(&ENCODER_SPI_HANDLE, SPI_FLAG_RXNE) == false);\
    const u16 __angle_data = (uint16_t) ((&ENCODER_SPI_HANDLE)->Instance->DR);\
    set_nCS(1);\
    __angle_data>>2;\
})


extern volatile u16 current_angle_dma_raw;
extern volatile u32 angle_timestamp_dma;
extern volatile u32 angle_timestamp_completed_dma;
extern volatile bool encoder_read_success_dma;
extern volatile bool encoder_dma_in_progress;
static volatile bool angle_measurement_called_last = false;