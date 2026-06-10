#pragma once

#include "typedefs.h"
#include "periphmap.h"

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


