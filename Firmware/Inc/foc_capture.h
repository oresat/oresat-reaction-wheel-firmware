#pragma once

#include "typedefs.h"

// Per-tick FOC state snapshot. Written from foc_gen_pwm_cb() at the end of
// each fast-loop ISR, when foc_capture_state == FOC_CAPTURE_RUNNING.
//
// Operates as a ring buffer: writes go to buf[idx % FOC_CAPTURE_LEN], and
// idx is a monotonically-increasing u32 counter. When stopped (via SWD
// memory write or FSM exit), idx tells the decoder how far we got and how
// to unwrap.
//
// All fields are little-endian; layout is fixed (no padding because all fields
// are 4-byte aligned). Matches tools/decode_foc_capture.py's dtype.
typedef struct {
    u32 loop_count;        // controller_get_loop_count() at capture time
    f32 Ia, Ib, Ic;        // phase currents, after zero-sum correction OR best-2-of-3 reconstruction (A)
    f32 Ialpha, Ibeta;     // Clarke output (A)
    f32 Id_meas, Iq_meas;  // Park output, this tick's measurement (A)
    f32 Id_set, Iq_set;    // d/q current setpoint (A)
    f32 motor_phase;       // electrical angle used for Park (rad)
    f32 motor_vel;         // electrical angular velocity (rad/s)
    f32 Vd_int, Vq_int;    // PI integrator state (V)
    f32 mod_d, mod_q;      // PI output before inv-Park (modulation, 0..1)
    f32 mod_alpha, mod_beta; // inv-Park output (modulation)
    f32 svm_A, svm_B, svm_C; // SVM output → goes to PWM duty (modulation, 0..1)
    f32 Vbus;              // bus voltage at sample time (V)
    i32 shunt_drop;        // best-2-of-3: phase reconstructed this tick (-1 none, 0/1/2 = A/B/C)
} foc_capture_t;

#define FOC_CAPTURE_LEN 256u  // 512 × 92 B = 46 KB → 16 ms at 32 kHz ISR

typedef enum {
    FOC_CAPTURE_IDLE    = 0,  // not writing
    FOC_CAPTURE_RUNNING = 1,  // ring-mode writes active
    FOC_CAPTURE_STOPPED = 2,  // frozen, buffer ready for dump
} foc_capture_state_t;

extern volatile foc_capture_t   foc_capture_buf[FOC_CAPTURE_LEN];
extern volatile u32             foc_capture_idx;
extern volatile foc_capture_state_t foc_capture_state;

// Start ring-mode capture. Resets idx to 0 so the buffer fills from the top.
// Safe to call from foreground; ISR reads state atomically.
void foc_capture_arm(void);

// Stop / freeze capture. Buffer contents and idx are preserved for readout.
// Called from FSM exit on control states (fsm_exit_state) and from the bench
// via SWD memory write (tools/foc_capture.sh stop).
void foc_capture_stop(void);
