#include "typedefs.h"
#include "periphmap.h"
#include "util.h"

void _putchar(char character) {
    uart_transmit_one(DEBUG_UART_HANDLE, character);
}

