#include "util.h"

#include <inttypes.h>

#include "printf.h"
#include "periphmap.h"

void user_assert_fail(uint8_t* file, uint32_t line) {
    printf("Assert failed: file %s on line %lu\r\n", file, line);
}


u32 micros() { return 0; }
//u32 micros() {
//    u32 ms;
//    u32 cycle_cnt;
//    do {
//        ms = HAL_GetTick();
//        cycle_cnt = SYSTICK_HANDLE->CNT;
////    } while (0);
//} while (ms != HAL_GetTick());
//
//    return (ms * 1000) + cycle_cnt;
//}

void delay_us(const u32 us) {
    u32 start = micros();
    while (micros() - start < us) {
        asm volatile ("nop");
    }
}

//void uart_transmit(USART_TypeDef* uart, u8* buf, u8 len) {
//    for (u8 i=0; i<len; i++) {
//        while (LL_USART_IsActiveFlag_TXE_TXFNF(uart) == 0);
//        LL_USART_TransmitData8(uart,buf[i]);
//    }
//    while (LL_USART_IsActiveFlag_TC(uart) == 0);
//}

void uart_transmit(USART_TypeDef* uart, u8* buf, u8 len) {
    for (u8 i = 0; i < len; i++) {
        while (LL_USART_IsActiveFlag_TXE_TXFNF(uart) == 0);
        LL_USART_TransmitData8(uart, buf[i]);
    }
    while (LL_USART_IsActiveFlag_TC(uart) == 0);
}
