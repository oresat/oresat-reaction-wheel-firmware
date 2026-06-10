#include "can_comms.h"

#include <string.h>

#include "aux_adc.h"
#include "closed_loop_controller.h"
#include "config.h"
#include "console.h"
#include "control_ll.h"
#include "ma702.h"
#include "crash_record.h"
#include "OD.h"
#include "periphmap.h"
#include "printf.h"
#include "CO_app_STM32.h"
#include "encoder.h"
#include "foc.h"
#include "inverter.h"

CANopenNodeSTM32 canOpenNodeSTM32;

void canopen_init(u8 node_id) {
    //HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_TX_COMPLETE,FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
    //HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_,FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);

    // 0x1018 identity serial from the silicon UID — written BEFORE canopen_app_init so
    // CO_LSSinit picks it up. All four wheels used to be LSS-identical with serial 0, which
    // made a duplicate-node-ID failure undiagnosable from the bus (flight-review RE-5).
    const u32* uid = (const u32*) UID_BASE;
    OD_RAM.x1018_identity.serial_number = uid[0] ^ uid[1] ^ uid[2];

    canOpenNodeSTM32.CANHandle = &hfdcan1;
    canOpenNodeSTM32.HWInitFunction = MX_FDCAN1_Init;
    canOpenNodeSTM32.timerHandle = &CAN_TIM_HANDLE;
    canOpenNodeSTM32.desiredNodeID = node_id;
    canOpenNodeSTM32.baudrate = 1000;
    canopen_app_init(&canOpenNodeSTM32);

    init_od();
    init_sdo();
    init_od_extensions(); // OD untouched by comm resets → extensions persist for node lifetime

    //HAL_TIM_Base_Start(&CAN_TIM_HANDLE); // CANOpen stuff takes care of this for us
}

// Calls at ~1kHz by TIM4
void canopen_1khz_isr() {
    canopen_app_interrupt();
}

void canopen_send_test_message() {
}

static void apply_pending_tpdo_rate();
static void emcy_report_errors();

// Should be called in a loop-ish at ~100Hz
void canopen_process() {
    canopen_app_process();
    apply_pending_tpdo_rate();
    emcy_report_errors();
}

// CANopen EMCY on controller-error edges: faults used to surface only via TPDO1 (5000 ms
// period in CONTROLLER_ERROR — one lost burst frame meant 5 s of blindness). EMCY is the
// mechanism designed for this: immediate, per-wheel (0x1014 base 0x80 + node ID), logged in
// the 0x1003 history. infoCode carries the raw controller_errors bitmask in EMCY bytes 4–7.
static void emcy_report_errors() {
    static u32 emcy_prev_errors = 0;
    u32 errors = controller_get_errors();
    if (errors == emcy_prev_errors || CO == NULL || CO->em == NULL)
        return;
    if (errors != 0) {
        // CO_errorReport with an already-set errorBit is a no-op — pulse a reset first so a
        // CHANGED nonzero mask (new bit added mid-episode) still emits a fresh frame with the
        // updated infoCode (review minor).
        if (emcy_prev_errors != 0)
            CO_errorReset(CO->em, CO_EM_GENERIC_ERROR, emcy_prev_errors);
        CO_errorReport(CO->em, CO_EM_GENERIC_ERROR, CO_EMC_DEVICE_SPECIFIC, errors);
    } else {
        CO_errorReset(CO->em, CO_EM_GENERIC_ERROR, 0);
    }
    emcy_prev_errors = errors;
}

void init_od() {
    update_control_status_od(FSM_STATE_IDLE, 0, 0);
    update_motor_vel_current_od(0.f, 0.f);
    update_bus_volt_current_od(0.f, 0.f);
    update_rxw_temp_od(0, 0, 0);
}

void update_control_status_od(fsm_state_t fsm_state, u32 controller_errors, u8 procedure_result) {
    OD_RAM.x4000_rxw_ctrl_stat.current_state = fsm_state;
    OD_RAM.x4000_rxw_ctrl_stat.errors = controller_errors;
    OD_RAM.x4000_rxw_ctrl_stat.procedure_result = procedure_result;
}

void update_motor_vel_current_od(f32 vel, f32 iq) {
    OD_RAM.x4001_rxw_motor_vel_current.motor_vel = vel;
    OD_RAM.x4001_rxw_motor_vel_current.motor_current = iq;
}

void update_bus_volt_current_od(f32 ibus, f32 vbus) {
    OD_RAM.x4002_rxw_bus_volt_current.bus_current = ibus;
    OD_RAM.x4002_rxw_bus_volt_current.bus_voltage = vbus;
}

void update_rxw_temp_od(i16 temp1, i16 temp2, i16 temp3) {
    OD_RAM.x4003_rxw_temp.temp_1 = temp1;
    OD_RAM.x4003_rxw_temp.temp_2 = temp2;
    OD_RAM.x4003_rxw_temp.temp_3 = temp3;
}

void init_sdo() {
    OD_RAM.x4004_rxw_requested_state.req_state = FSM_STATE_IDLE;
    OD_RAM.x4005_rxw_setpoint.setpoint_1 = 0.f;
    OD_RAM.x4005_rxw_setpoint.setpoint_2 = 0.f;
    OD_RAM.x4006_rxw_reserved.reserved_1 = 0.f;
    OD_RAM.x4006_rxw_reserved.reserved_2 = 0.f;
    OD_RAM.x4007_rxw_reboot.reboot_request = 0;
}

// Boot diagnostics → the spare 0x4006 "reserved" floats, bit-packed (the host reinterprets the
// raw 4 bytes, it's not a meaningful float):
//   reserved_1 = RCC_CSR reset-cause flags [31:24] | last crash kind [19:16] | boot count [15:0]
//   reserved_2 = stacked PC of the last fault (0 = no crash record)
// Written once after canopen_init; a host write to 0x4006 clobbers it until the next boot.
void publish_boot_diagnostics(u32 rcc_csr) {
    u32 kind = (CRASH_RECORD->magic == CRASH_RECORD_MAGIC) ? CRASH_RECORD->kind : 0u;
    u32 word1 = (rcc_csr & 0xFF000000u) | ((kind & 0xFu) << 16) |
                (noinit_cell_get(BOOT_COUNT_CELL) & 0xFFFFu);
    u32 word2 = (CRASH_RECORD->magic == CRASH_RECORD_MAGIC) ? CRASH_RECORD->stacked_pc : 0u;
    memcpy((void*) &OD_RAM.x4006_rxw_reserved.reserved_1, &word1, sizeof(word1));
    memcpy((void*) &OD_RAM.x4006_rxw_reserved.reserved_2, &word2, sizeof(word2));
}

fsm_state_t get_sdo_requested_state() {
    return OD_RAM.x4004_rxw_requested_state.req_state;
}

void set_sdo_requested_state(fsm_state_t state) {
    OD_RAM.x4004_rxw_requested_state.req_state = (u8) state;
}

bool get_sdo_reboot_request() {
    return OD_RAM.x4007_rxw_reboot.reboot_request == 1;
}

f32 get_sdo_setpoint_1() {
    return OD_RAM.x4005_rxw_setpoint.setpoint_1;
}

f32 get_sdo_setpoint_2() {
    return OD_RAM.x4005_rxw_setpoint.setpoint_2;
}

f32 get_sdo_reserved_1() {
    return OD_RAM.x4006_rxw_reserved.reserved_1;
}

f32 get_sdo_reserved_2() {
    return OD_RAM.x4006_rxw_reserved.reserved_2;
}

// ===== Disarmed velocity estimate =====
// The PLL only runs inside the armed fast loop, so after any disarm/trip the published
// velocity froze at its last value while the wheel physically coasted — a cancelled backemf
// cal left a wheel at tens of rev/s reporting vel=0 (flight-review P2-18). While disarmed,
// sample the encoder over SPI at ~200 Hz from the foreground and differentiate. The SPI
// transaction runs under PRIMASK with an armed-check INSIDE: arming happens in the TIM17 ISR,
// and the fast-read macros must never interleave with this read — masking makes
// check-then-read atomic. ±0.5 turn per 5 ms sample = ±100 rev/s unambiguous range.
static volatile f32 disarmed_vel = 0.f;

// Last disarmed velocity estimate [rev/s mech] — the seed for bumpless control-law entry
// (vel_ctrl_init): freshest available measurement at the moment of arming, ≤ ~5 ms old.
f32 get_disarmed_vel() {
    return disarmed_vel;
}

static void update_disarmed_velocity() {
    static u32 last_sample_ms = 0;
    static f32 last_pos = 0.f;
    static bool have_prev = false;

    if (controller_is_armed()) {
        have_prev = false; // the PLL owns the estimate; reseed on the next disarm
        return;
    }

    u32 now = HAL_GetTick();
    u32 dt_ms = now - last_sample_ms;
    if (dt_ms < 5u)
        return;

    u16 raw = 0;
    bool sampled = false;
    u32 primask = __get_PRIMASK();
    __disable_irq();
    if (!controller_is_armed()) {
        // Bounded inline variant of __MA702_FAST_ANGLE_READ_START/_FINISH (ma702.h) — a
        // wedged SPI must not spin forever under PRIMASK (~7 µs nominal; the 1500-iteration
        // bound is ~60 µs worst case, masking SysTick/TIM17/TIM4 briefly — acceptable while
        // disarmed). First drain any RX residue: an earlier timed-out transaction that
        // completed late would otherwise leave every later read — including the armed fast
        // loop's — one frame behind (review F4).
        u32 flush = 0;
        while (__HAL_SPI_GET_FLAG(&ENCODER_SPI_HANDLE, SPI_FLAG_RXNE) != 0u && ++flush < 8u)
            (void) (&ENCODER_SPI_HANDLE)->Instance->DR;
        set_nCS(0);
        CLEAR_BIT((&ENCODER_SPI_HANDLE)->Instance->CR2, SPI_RXFIFO_THRESHOLD);
        (&ENCODER_SPI_HANDLE)->Instance->DR = 0;
        u32 spins = 0;
        while (__HAL_SPI_GET_FLAG(&ENCODER_SPI_HANDLE, SPI_FLAG_RXNE) == 0u && ++spins < 1500u);
        if (spins < 1500u) {
            raw = (u16) ((u16) ((&ENCODER_SPI_HANDLE)->Instance->DR) >> 2);
            sampled = true;
        }
        set_nCS(1);
    }
    __set_PRIMASK(primask);
    if (!sampled)
        return;

    f32 pos = (f32) raw / (f32) ENCODER_CPR;
    if (have_prev) {
        f32 d = pos - last_pos;
        if (d > 0.5f) d -= 1.f;
        else if (d < -0.5f) d += 1.f;
        disarmed_vel = d * 1000.f / (f32) dt_ms; // rev/s mech
    } else {
        disarmed_vel = 0.f;
        have_prev = true;
    }
    last_pos = pos;
    last_sample_ms = now;
}

// Should be called quite often, ideally consistently >100Hz (maybe in main loop w/ canopen_app_process())
void update_tpdo_od_vals() {
    fsm_state_t current_state = fsm_get_current_state();
    u32 controller_errors = controller_get_errors();
    u8 procedure_result = fsm_get_last_state_exit_ok();

    update_disarmed_velocity();

    // Velocity is published in EVERY state: armed → the PLL (live in any law, including the
    // backemf coast with a floated bridge), disarmed → the foreground sampler above. The old
    // version zeroed it outside the four control states, hiding a physically spinning wheel
    // from the ADCS (flight-review P2-18).
    f32 motor_vel = controller_is_armed() ? encoder_get_pos_vel().vel : disarmed_vel;

    // iq / Vbus / ibus only mean anything while the FOC consumers run (closed-loop states).
    f32 motor_current = 0.f;
    f32 bus_voltage = 0.f;
    f32 bus_current = 0.f;

    if (current_state == FSM_STATE_VEL_CONTROL ||
        current_state == FSM_STATE_TORQUE_CONTROL ||
        current_state == FSM_STATE_VEL_RAMP_CONTROL) {
        motor_current = get_idq_measured().q;
        bus_voltage = get_foc_vbus();
        bus_current = get_foc_ibus();
    }

    // 0x4003: temp_1 = phase A/B/C thermistor average, temp_2 = aux thermistor, temp_3 = die.
    // Averaged after conversion — the NTC divider is nonlinear in ADC counts.
    i16 temp_phase_avg = (i16) ((aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm_phase_a_sns)
                                 + aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm_phase_b_sns)
                                 + aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm_phase_c_sns)) / 3);
    i16 temp_aux = (i16) aux_adc_conv_therm_cDegC(aux_adc_last_reading_raw.therm_aux_sns);
    i16 temp_die = (i16) aux_adc_conv_thermint_cDegC(aux_adc_last_reading_raw.thermint_sns);

    update_control_status_od(current_state, controller_errors, procedure_result);
    update_motor_vel_current_od(motor_vel, motor_current);
    update_bus_volt_current_od(bus_current, bus_voltage);
    update_rxw_temp_od(temp_phase_avg, temp_aux, temp_die);
}

/* ===== Host command ingestion: OD write extensions =====
 *
 * 0x4004 (requested state), 0x4005 (setpoints) and 0x4007 (reboot) carry write extensions, so
 * every SDO download invokes its handler exactly once, REGARDLESS of value. The previous
 * design polled the OD and compared against shadow copies — re-sending the same state to
 * re-run a procedure, or the same setpoint after a state-entry zeroed the internal one, was
 * silently eaten (footguns.md "edge-triggered setpoints"; flight-review §3.2). Validation
 * lives here too: a bad write is answered with an SDO abort instead of silence.
 *
 * Handlers run in SDO-server context = the foreground loop (canopen_process). No RPDO maps
 * these objects, so there is no ISR-context writer.
 */

static bool fsm_state_requestable(u8 s) {
    switch ((fsm_state_t) s) {
        case FSM_STATE_IDLE:
        case FSM_STATE_TORQUE_CONTROL:
        case FSM_STATE_VEL_CONTROL:
        case FSM_STATE_MOTOR_RESISTANCE_CAL:
        case FSM_STATE_MOTOR_INDUCTANCE_CAL:
        case FSM_STATE_ENCODER_DIR_CAL:
        case FSM_STATE_ENCODER_OFFSET_CAL:
        case FSM_STATE_ENCODER_TEST:
        case FSM_STATE_OPEN_LOOP_CONTROL:
        case FSM_STATE_CLEAR_ERRORS:
        case FSM_STATE_ENCODER_VALIDATION:
        case FSM_STATE_SHITTY_OFFSET_CAL:
        case FSM_STATE_VEL_RAMP_CONTROL:
        case FSM_STATE_BACKEMF_CAL:
        case FSM_STATE_SAVE_CALIBRATION:
        case FSM_STATE_ERASE_CALIBRATION:
        case FSM_STATE_RESTORE_CALIBRATION:
        case FSM_STATE_BACKEMF_CAL_BIDIR:
            return true;
        default:
            // NONE (request sentinel), SYSTEM_ERROR/CONTROLLER_ERROR (the FSM enters these,
            // hosts don't), POS_CONTROL (unimplemented stub), and everything past the enum —
            // previously these were forwarded blind and produced zombie states + an
            // out-of-bounds state_names[] read in an ISR printf.
            return false;
    }
}

static ODR_t od_write_requested_state(OD_stream_t* stream, const void* buf,
                                      OD_size_t count, OD_size_t* countWritten) {
    if (stream->subIndex != 1)
        return OD_writeOriginal(stream, buf, count, countWritten);
    if (count != 1)
        return ODR_TYPE_MISMATCH;
    u8 req = *(const u8*) buf;
    if (!fsm_state_requestable(req))
        return ODR_INVALID_VALUE; // SDO abort 0x06090030 — host hears the rejection synchronously
    // Closed-loop states need valid encoder cal: reject with a "device state" abort
    // (0x08000022) so the host learns NOW instead of via a CONTROLLER_ERROR detour. Run or
    // restore cal first (runbook: 9 → 15 → 21 → 18).
    if (fsm_state_requires_encoder_cal((fsm_state_t) req) && !encoder_cal_valid())
        return ODR_DATA_DEV_STATE;
    ODR_t r = OD_writeOriginal(stream, buf, count, countWritten);
    if (r == ODR_OK)
        fsm_request_state((fsm_state_t) req);
    return r;
}

static ODR_t od_write_setpoint(OD_stream_t* stream, const void* buf,
                               OD_size_t count, OD_size_t* countWritten) {
    if (stream->subIndex != 1) // setpoint_2 is reserved: stored + readable, applied nowhere
        return OD_writeOriginal(stream, buf, count, countWritten);
    if (count != 4)
        return ODR_TYPE_MISMATCH;
    f32 v;
    memcpy(&v, buf, sizeof(v));
    // Non-finite values must die here: clamp_f32(lo, NaN, hi) returns lo, so a NaN torque
    // setpoint would be laundered into FULL NEGATIVE current with no error raised
    // (flight-review P1-4). The magnitude gate is an insanity bound, not an ops limit —
    // units are state-dependent (Nm or rev/s).
    if (!f32_finite_bits(v) || v < -1e5f || v > 1e5f)
        return ODR_INVALID_VALUE;
    ODR_t r = OD_writeOriginal(stream, buf, count, countWritten);
    if (r != ODR_OK)
        return r;
    switch (fsm_get_current_state()) {
        case FSM_STATE_TORQUE_CONTROL:
            set_torque_setpoint(v);
            break;
        case FSM_STATE_VEL_CONTROL:
            set_vel_setpoint(v);
            break;
        case FSM_STATE_VEL_RAMP_CONTROL:
            set_vel_ramp_setpoint(v);
            break;
        default:
            // Not in a setpoint-consuming state: accepted (readable back at 0x4005) but
            // applied to nothing. Hosts should confirm the state via 0x4000 before writing
            // setpoints — and a write that raced a transition can simply be re-sent now,
            // since there is no edge-trigger left to poison.
            break;
    }
    return ODR_OK;
}

static volatile bool reboot_pending = false;

static ODR_t od_write_reboot(OD_stream_t* stream, const void* buf,
                             OD_size_t count, OD_size_t* countWritten) {
    ODR_t r = OD_writeOriginal(stream, buf, count, countWritten);
    if (r == ODR_OK && stream->subIndex == 1 && count == 1 && *(const u8*) buf != 0)
        reboot_pending = true; // any nonzero reboots (was: exactly 1, silently else); executed
                               // by process_sdo_vals so the SDO response can leave the wire
    return r;
}

void init_od_extensions() {
    static OD_extension_t ext_4004 = {.object = NULL, .read = OD_readOriginal, .write = od_write_requested_state};
    static OD_extension_t ext_4005 = {.object = NULL, .read = OD_readOriginal, .write = od_write_setpoint};
    static OD_extension_t ext_4007 = {.object = NULL, .read = OD_readOriginal, .write = od_write_reboot};
    OD_extension_init(OD_ENTRY_H4004, &ext_4004);
    OD_extension_init(OD_ENTRY_H4005, &ext_4005);
    OD_extension_init(OD_ENTRY_H4007, &ext_4007);
}

// Host-command epilogue, called from the foreground loop after canopen_process(). Ingestion
// itself moved into the OD write extensions above — only actions that must not run inside the
// SDO server's callstack live here.
void process_sdo_vals() {
    if (reboot_pending) {
        controller_disarm();
        // Let the just-queued SDO response win arbitration and leave the wire (~130 µs frame
        // at 1 Mbit) — the host then sees a confirmed write instead of a guaranteed timeout.
        HAL_Delay(3);
        console_flush(); // last bytes off the wire before the reset eats them
        HAL_NVIC_SystemReset();
    }
}

// Per-state TPDO event-timer rate. Called from the FSM tick (TIM17 ISR), which must NOT touch
// the CANopen stack: the old implementation ran a full canopen_app_resetCommunication() from
// interrupt context on every state entry — TX queue purged, NMT re-init + spurious boot-up
// frame, SDO server reset under the foreground's feet (flight-review §3.3). The rate is parked
// here; the foreground applies it below.
static volatile u32 tpdo_rate_pending_ms = 0;
static volatile bool tpdo_rate_pending = false;

void set_tpdo_send_rate(u32 timer_ms) {
    tpdo_rate_pending_ms = timer_ms;
    tpdo_rate_pending = true;
}

// Foreground half: write the event timers through the OD *interface* (odOrig=false), which
// routes through CANopenNode's TPDO comm-param write extension — that updates the live
// TPDO->eventTime_us and zeroes the running event timer (so all four TPDOs still burst
// promptly after a state transition), with no comm reset. Plain OD_RAM pokes would only take
// effect at the next CO_CANopenInitPDO.
static void apply_pending_tpdo_rate() {
    if (!tpdo_rate_pending || CO == NULL)
        return;
    tpdo_rate_pending = false; // clear before reading: a racing FSM tick re-pends, never lost
    u16 timer_ms = (u16) tpdo_rate_pending_ms;

    const OD_entry_t* tpdo_comm[4] = {OD_ENTRY_H1800, OD_ENTRY_H1801, OD_ENTRY_H1802, OD_ENTRY_H1803};
    ODR_t fail_r = ODR_OK;
    u32 fail_i = 0;
    CO_LOCK_OD(CO->CANmodule); // vs. the TIM4 TPDO pass reading eventTime/eventTimer
    for (u32 i = 0; i < 4; i++) {
        ODR_t r = OD_set_u16(tpdo_comm[i], 5, timer_ms, false); // sub 5 = event timer [ms]
        if (r != ODR_OK && fail_r == ODR_OK) {
            fail_r = r;
            fail_i = i;
        }
    }
    CO_UNLOCK_OD(CO->CANmodule);
    if (fail_r != ODR_OK) // print OUTSIDE the lock: sync UART under BASEPRI(2) masks SysTick/TIM4
        printf("TPDO%lu event-timer write failed (%d)\r\n", (unsigned long) (fail_i + 1u), (int) fail_r);
}