#pragma once

#include "typedefs.h"

// Deferred debug console (console.c). printf() formats into a per-call stack buffer and hands
// the whole message to console_write(), which routes by context:
//  - ISR (IPSR != 0):  copy into a RAM ring under a short PRIMASK critical section and return.
//                      Never waits on the UART. If the ring is full the WHOLE message is
//                      dropped (console_dropped counts it; the pump reports & clears it).
//  - thread:           drain the ring first (an already-queued ISR message is older than this
//                      one), then transmit directly — blocking is fine in the foreground, and
//                      this keeps exactly one UART-writing context.
// console_pump() runs once per app_main foreground pass. console_flush() additionally waits
// for the UART TC flag — call it before any reset so the last bytes actually leave the wire.
//
// Lines longer than CONSOLE_LINE_MAX-1 are truncated (the only realistic case is a full
// STRINGIFY_CONTROLLER_ERRORS dump). The ring also doubles as a postmortem black box: the
// last CONSOLE_RING_LEN bytes of output stay readable over SWD even after a wedge/fault —
// see tools/console_tail.sh.

#define CONSOLE_LINE_MAX 256u

void console_write(const char *msg, u32 len);
void console_pump(void);
void console_flush(void);

// Messages dropped because the ring was full. SWD-visible; console_pump() prints and clears.
extern volatile u32 console_dropped;

// Bytes dropped because the UART TXE spin timed out (wedged peripheral). SWD-visible.
extern volatile u32 console_tx_timeouts;
