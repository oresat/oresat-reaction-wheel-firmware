#pragma once

#include "typedefs.h"

// Per-tick FOC state snapshot. Written from foc_gen_pwm_cb() at the end of
// each fast-loop ISR, when foc_capture_state == FOC_CAPTURE_ARMED.
//
// All fields are little-endian; layout is fixed (no padding because all fields
// are 4-byte aligned). Matches tools/decode_foc_capture.py's dtype.
typedef struct {
    u32 loop_count;        // controller_get_loop_count() at capture time
    f32 Ia, Ib, Ic;        // phase currents, after zero-sum correction (A)
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
} foc_capture_t;

#define FOC_CAPTURE_LEN 256u  // 256 × ~88 B = ~22.5 KB → 16 ms at 16 kHz ISR

typedef enum {
    FOC_CAPTURE_IDLE  = 0,
    FOC_CAPTURE_ARMED = 1,
    FOC_CAPTURE_DONE  = 2,
} foc_capture_state_t;

extern volatile foc_capture_t   foc_capture_buf[FOC_CAPTURE_LEN];
extern volatile u32             foc_capture_idx;
extern volatile foc_capture_state_t foc_capture_state;

// Arm a one-shot capture. Next FOC_CAPTURE_LEN ISR ticks fill the buffer
// starting at index 0; state transitions to FOC_CAPTURE_DONE when full.
// Safe to call from foreground; ISR reads state atomically.
void foc_capture_arm(void);
