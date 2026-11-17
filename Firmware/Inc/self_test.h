#pragma once

#include "typedefs.h"
#include "control_ll.h"

/*
 * FSM_STATE_SELF_TEST (22): desk bring-up self-test. See docs/control-laws.md and
 * docs/board-bringup.md.
 *
 * Two halves:
 *  1. Bridge half — a control law, ~0.2 s, runs armed in the fast loop. Floats the bridge and
 *     measures (shunt-amp midpoints, Vbus, phase voltages, encoder motion); if nothing is
 *     moving, switches all three legs together at 5 % and then 95 % duty and checks each
 *     phase-voltage sense follows (low, then high), with no current and no Vbus sag. All legs
 *     share one duty, so a motor at rest sees zero volts across it.
 *     Skipped when the boot shunt-offset cal was rejected (the fast loop would refuse to arm).
 *  2. Report half — foreground, disarmed: clock, option bytes, node-ID straps, VREF+, temps,
 *     MA732 register readback, CAN error counters, then the bridge results. Printed on the debug
 *     UART as "SELFTEST <PASS|FAIL|WARN|SKIP|INFO> <id>: <text>" lines between BEGIN/END lines.
 *
 * procedure_result (TPDO1) = overall pass. It lands when the report finishes (tens of ms after
 * the state returns to IDLE); the FSM holds new state requests until then.
 */

extern control_law_t self_test_ctrl_law;

// app_main, at boot: what the strap vote and reset-cause read saw.
void self_test_set_boot_info(u8 wheel_number, u8 node_id, u32 addr0_high, u32 addr1_high,
                             u32 rcc_csr);

// FSM entry (TIM17 ISR): reset results. Returns true if the bridge half may run (the boot
// shunt-offset cal passed); false = report-only run, the FSM must not arm.
bool self_test_prepare();

// FSM, right after controller_arm(): float the bridge before the first fast-loop tick.
void self_test_after_arm();

// FSM exit (TIM17 ISR), after controller_disarm(): latch the bridge outcome, park the report.
void self_test_exit(bool cancelled);

// True from self_test_exit() until self_test_report_clear(). The FSM holds new requests while
// it is, so procedure_result can't be overwritten before the report lands.
bool self_test_report_pending();

// Foreground, when pending: run the report half (blocking UART + SPI, tens of ms). Returns the
// overall pass/fail. Does not clear the pending flag: the caller latches the result first.
bool self_test_run_report();

void self_test_report_clear();
