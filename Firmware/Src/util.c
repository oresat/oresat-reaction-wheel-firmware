#include "util.h"

#include <inttypes.h>

#include "printf.h"
#include "periphmap.h"

void user_assert_fail(uint8_t* file, uint32_t line) {
    printf("Assert failed: file %s on line %lu\r\n", file, line);
}


// (micros()/delay_us() deleted: micros was stubbed to return 0, so delay_us(n>0) never
// terminated — a foreground wedge one careless revival away. Zero callers. Use DWT->CYCCNT
// for sub-ms timing if ever needed.)

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
