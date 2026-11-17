#pragma once

#include "typedefs.h"
#include "periphmap.h"

typedef struct {
    bool nboot0_ok, nboot1_ok, nswboot0_ok, bor_ok, all_ok;
} option_bytes_status_t;

// Quiet read of the option bytes vs. what the firmware requires (the self-test uses this).
option_bytes_status_t get_option_bytes_status();

// Same check, printing one OK/BAD line per field (boot path).
bool check_option_bytes_ok();

bool write_option_bytes();

static void init_boot0() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = boot0_pin.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(boot0_pin.port, &GPIO_InitStruct);
}

static bool get_boot0_pin_state() {
    return (HAL_GPIO_ReadPin(boot0_pin.port, boot0_pin.pin) != GPIO_PIN_RESET);
}


