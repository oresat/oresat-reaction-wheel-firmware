#pragma once
#include "typedefs.h"
#include "mathutil.h"
#include "bldc_math.h"
#include "pwm.h"

void controller_init();
void controller_arm();
void controller_disarm();

// Fast-loop ISR rate in Hz. All four aliases below are the same number;
// kept as separate names so call sites read with the intent they're expressing
// (encoder-update rate vs current-meas rate vs pwm-update rate vs generic loop rate).
#define LOOP_UPDATE_FREQ  ((f32)PWM_SWITCHING_HZ / (f32)PWM_CYCLES_PER_ISR)
#define ENC_MEAS_FREQ     LOOP_UPDATE_FREQ
#define CURRENT_MEAS_FREQ LOOP_UPDATE_FREQ
#define PWM_UPDATE_FREQ   LOOP_UPDATE_FREQ
#define SECONDS_PER_LOOP  (1.f / LOOP_UPDATE_FREQ)


typedef enum CONTROLLER_STATE {
    CTRLR_STATE_IDLE,
    CTRLR_STATE_ERR,
    CTRLR_STATE_SHUNT_OFFSET_CAL,
    CTRLR_STATE_MOTOR_RESISTANCE_CAL,
    CTRLR_STATE_MOTOR_INDUCTANCE_CAL,
    CTRLR_STATE_MOTOR_ENCODER_DIR_CAL,
    CTRLR_STATE_MOTOR_ENCODER_PHASE_CAL,
    CTRLR_STATE_LOCKIN_SPIN,
    CTRLR_STATE_VEL_CTRL,
    CTRLR_STATE_TORQUE_CTRL
} CONTROLLER_STATE_t;

typedef enum CONTROLLER_ERRORS {
    CTRLR_ERR_NONE = 0,
    CTRLR_ERR_INVERTER_CALIBRATION_INVALID=1<<0,
    CTRLR_ERR_PHASE_CURRENTS_INVALID=1<<1,
    CTRLR_ERR_PHASE_CURRENTS_MEASUREMENT_MISSING=1<<2,
    CTRLR_ERR_PWM_TIMING_INVALID=1<<3,
    CTRLR_ERR_PWM_TIMING_UPDATE_MISSING=1<<4,
    CTRLR_ERR_VBUS_OVERVOLTAGE=1<<5,
    CTRLR_ERR_VBUS_UNDERVOLTAGE=1<<6,
    CTRLR_ERR_IBUS_OVERCURRENT=1<<7,
    CTRLR_ERR_MOTOR_OVERCURRENT=1<<8,
    CTRLR_ERR_MOTOR_PHASE_LEAKAGE=1<<9,
    CTRLR_ERR_MOTOR_RESISTANCE_OUT_OF_RANGE=1<<10,
    CTRLR_ERR_MOTOR_INDUCTANCE_OUT_OF_RANGE=1<<11,
    CTRLR_ERR_ENCODER_READING_MISSING=1<<12,
    CTRLR_ERR_ENCODER_ESTIMATE_MISSING=1<<13,
    CTRLR_ERR_ENCODER_READING_INVALID=1<<14,
    CTRLR_ERR_ENCODER_FAILURE=1<<15,
    CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING=1<<16,
    CTRLR_ERR_PWM_TIMING_USAGE_MISSING=1<<17,
    CTRLR_ERR_PHASE_CURRENT_LEAKAGE=1<<18,
    CTRLR_ERR_ENCODER_READING_USAGE_MISSING=1<<19,
    CTRLR_ERR_MOTOR_UNBALANCED_PHASES=1<<20,
    CTRLR_ERR_MODULATION=1<<21,
    CTRLR_ERR_NOT_CALIBRATED=1<<22, // closed-loop state requested without valid encoder dir/offset cal
} CONTROLLER_ERRORS_t;

//void controller_request_state(CONTROLLER_STATE_t state);
//CONTROLLER_STATE_t controller_get_current_state();
//CONTROLLER_STATE_t controller_get_requested_state();

u32 controller_get_errors();
bool controller_is_armed();


u32 controller_get_loop_count();
u32 controller_get_micros();

// Best-2-of-3 shunt selection (see control_ll.c / docs/high-speed-foc-worklog.md).
extern volatile bool shunt_selection_enabled;   // A/B toggle, seeded from FOC_USE_SHUNT_SELECTION
extern volatile i8 last_shunt_drop_phase;        // -1 none, 0/1/2 = phase reconstructed this tick

// Fast-loop ISR duration in CPU cycles (DWT->CYCCNT deltas; counter enabled in app_main).
// Budget = SystemCoreClock / LOOP_UPDATE_FREQ (4500 cycles at 144 MHz / 32 kHz). If `last`
// hovers at/above budget the ISR tail-chains at priority 0, SysTick (priority 15, the sole
// IWDG feeder) starves, and the watchdog reboots ~512 ms after arming. SWD-read these live
// while armed; write 0 to isr_cycles_max to re-arm the high-water mark. `max` excludes the
// arm tick (in-ISR control_law_init + its printfs, ~1 ms); `last` does not.
// Measured 2026-06-09, RelWithDebInfo (CCM hot path): steady state ≈3.2k cycles = 72% of
// the 32 kHz budget — Debug (-Og, flash-resident) does not fit and watchdog-reboots on arm.
extern volatile u32 isr_cycles_last;
extern volatile u32 isr_cycles_max;

//void controller_set_current_valid(bool is_valid);

void controller_set_new_pwm_timings(svm_vec_t *timings);
//void controller_set_pwm_timings_valid(bool is_valid);

//void controller_set_encoder_valid(bool is_valid);

// Producer/consumer fcns return bool of if there's any error
// Makes it easy to hit 8khz timing constraint (I think?)
typedef bool (*ll_encoder_consumer_t)(u16 enc_val); // Pointer to a callback function that "consumes" an encoder value.
typedef bool (*ll_phase_current_consumer_t)(f32 A_i, f32 B_i, f32 C_i, f32 V_vbus); // Pointer to a callback function that "consumes" a phase current reading.
typedef bool (*ll_pwm_timings_producer_t)(svm_vec_t *svm_magnitudes); // Pointer to a callback function that "produces" PWM timings
typedef void (*ll_control_law_exec_t)(); // Pointer to a callback function that executes a low-level control law
typedef bool (*ll_control_law_is_complete_t)(); // pointer to a callback function that returns whether or not the ll law is done. if so, we disarm and idle.
typedef void (*ll_control_law_init_t)(); // pointer to a callback function that gets called once on/before control law start
typedef bool (*ll_control_law_finish_t)(bool cancelled); // pointer to a callback function that gets called once after the control law ends (for de-init/final calcs
// Argument for whether or not we ended the procedure early (or if it was ended as a result of is_complete returning true, returns whether or not the procedure was successful

typedef struct {
    ll_encoder_consumer_t encoder_consumer;
    ll_phase_current_consumer_t current_consumer;
    ll_pwm_timings_producer_t pwm_producer;
    ll_control_law_exec_t control_law_exec;
    ll_control_law_is_complete_t is_complete;
    ll_control_law_init_t control_law_init;
    ll_control_law_finish_t control_law_finish;
} control_law_t;

void controller_set_control_law(const control_law_t *_control_law);
void controller_clear_control_law();
bool controller_is_control_law_done();
volatile control_law_t* controller_get_control_law();

void controller_add_errors(u32 err);
void controller_clear_errors();

void controller_set_torque_setpoint(f32 torque);
void controller_set_vel_setpoint(f32 vel);
void controller_set_pi_gains(f32 kp, f32 ki);
