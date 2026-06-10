#include "console.h"

#include <string.h>

#include "periphmap.h"
#include "printf.h"
#include "stm32g4xx.h"
#include "util.h"

/*
 * Deferred debug console.
 *
 * Why: every UART char at 921600 baud is ~10.9 µs of blocked polling. An FSM state-transition
 * print pair stalled the TIM17 ISR ~1 ms (and the lower-priority TIM4 CANopen tick with it),
 * and init_foc's two lines inside the priority-0 fast loop cost ~141k cycles ≈ 31 dropped
 * control ticks at every arm (measured 2026-06-09 via isr_cycles_*). Deferring the wire moves
 * all of that to the foreground: an ISR printf now costs format + memcpy (~2k cycles).
 *
 * Concurrency model (bare metal, single core — no true parallelism, only preemption):
 *  - Producers: ISR-context console_write() at any NVIC priority. A higher-priority ISR's
 *    printf can preempt a lower-priority one mid-append (HRTIM pri 0 over TIM17 pri 1), so
 *    reserve+copy+commit runs under PRIMASK. The critical section is a bounded memcpy
 *    (≤ CONSOLE_LINE_MAX B ≈ 1 µs) — negligible against the 31.25 µs fast-loop budget.
 *  - Consumer: console_pump(), thread mode only. `tail` is only written here; `head` only by
 *    producers. Exception entry/return orders memory on the M4, so no fences are needed.
 *  - Thread-mode console_write() never touches the ring: it pumps (older queued messages go
 *    out first) and then writes the UART directly. Boot output is therefore synchronous,
 *    exactly as before this existed.
 *
 * Overflow: drop the whole message (never block, never half-print), count in console_dropped;
 * the next pump prints "[console: N dropped]". The ring keeps the last CONSOLE_RING_LEN bytes
 * of history (drained bytes are not erased) — dump postmortem via tools/console_tail.sh.
 */

#define CONSOLE_RING_LEN 2048u // power of two — indices are free-running and masked

static u8 console_ring[CONSOLE_RING_LEN];
static volatile u32 console_head = 0; // total bytes queued (producers)
static volatile u32 console_tail = 0; // total bytes drained (consumer)
volatile u32 console_dropped = 0;

// Thread-side TX byte. Spin on TXE with interrupts ON (a full byte-time is 10.9 µs — far too
// long to mask), then recheck-and-write under a ~tens-of-ns PRIMASK window: the encoder-cal
// finish() dumps write this UART directly from TIM17 context, and one of them preempting
// between a bare TXE check and the TDR write would clobber the byte in TDR.
static void console_tx_byte(u8 b) {
    for (;;) {
        while (LL_USART_IsActiveFlag_TXE_TXFNF(DEBUG_UART_HANDLE) == 0);
        u32 primask = __get_PRIMASK();
        __disable_irq();
        if (LL_USART_IsActiveFlag_TXE_TXFNF(DEBUG_UART_HANDLE) != 0) {
            LL_USART_TransmitData8(DEBUG_UART_HANDLE, b);
            __set_PRIMASK(primask);
            return;
        }
        __set_PRIMASK(primask); // an ISR consumed TXE between check and lock — re-spin
    }
}

void console_write(const char *msg, u32 len) {
    if (len == 0u)
        return;
    if (len >= CONSOLE_RING_LEN) { // can never fit; printf.c caps at CONSOLE_LINE_MAX anyway
        console_dropped++;
        return;
    }

    if (__get_IPSR() != 0u) {
        u32 primask = __get_PRIMASK();
        __disable_irq();
        u32 head = console_head;
        u32 free = CONSOLE_RING_LEN - (head - console_tail);
        if (free < len) {
            console_dropped++;
        } else {
            u32 idx = head & (CONSOLE_RING_LEN - 1u);
            u32 first = CONSOLE_RING_LEN - idx;
            if (first > len)
                first = len;
            memcpy(&console_ring[idx], msg, first);
            memcpy(&console_ring[0], msg + first, len - first);
            console_head = head + len;
        }
        __set_PRIMASK(primask);
        return;
    }

    console_pump();
    for (u32 i = 0; i < len; i++)
        console_tx_byte((u8) msg[i]);
}

void console_pump(void) {
    // Budgeted to one full ring per call so a runaway ISR producer (an accidental printf in
    // the fast loop refilling faster than 921600 baud drains) cannot wedge the foreground.
    u32 budget = CONSOLE_RING_LEN;
    while (console_tail != console_head && budget-- != 0u) {
        // Volatile-qualified read: keeps the byte load ordered after the volatile head check
        // above (a hoisted plain load could race an ISR append and transmit a stale byte).
        console_tx_byte(*(volatile u8 *) &console_ring[console_tail & (CONSOLE_RING_LEN - 1u)]);
        console_tail++;
    }

    if (console_dropped != 0u) {
        u32 primask = __get_PRIMASK();
        __disable_irq();
        u32 n = console_dropped;
        console_dropped = 0;
        __set_PRIMASK(primask);

        char msg[48];
        int len = snprintf(msg, sizeof(msg), "\r\n[console: %u msgs dropped]\r\n", (unsigned) n);
        u32 send = (len > 0) ? ((u32) len < sizeof(msg) ? (u32) len : sizeof(msg) - 1u) : 0u;
        for (u32 i = 0; i < send; i++)
            console_tx_byte((u8) msg[i]);
    }
}

void console_flush(void) {
    console_pump();
    while (LL_USART_IsActiveFlag_TC(DEBUG_UART_HANDLE) == 0); // last byte fully off the wire
}
