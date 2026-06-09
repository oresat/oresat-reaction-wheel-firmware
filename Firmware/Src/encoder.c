#include <math.h>
#include "encoder.h"
#include "config.h"
#include "mathutil.h"
#include "printf.h"
#include "typedefs.h"


// Estimator gains, placed for a critically damped response @ desired bandwidth
static const f32 pll_kp = 2.f * ENCODER_ESTIMATOR_BANDWIDTH;
static const f32 pll_ki = 0.25f * (2.f * ENCODER_ESTIMATOR_BANDWIDTH) * (2.f * ENCODER_ESTIMATOR_BANDWIDTH);

static const f32 enc_meas_period = 1.f / ((f32)ENC_MEAS_FREQ); // How often we expect update() to be called, in seconds

static volatile f32 last_enc_pos = 0.f; // Last 0-1 position, for delta calc

static volatile posvel_t posvel_estimate = {0.f, 0.f}; // Last filtered estimate of our singleturn position/velocity

static volatile f32 multiturn_pos_raw = 0.f; // Multiturn count directly off of encoder sense angle
static volatile f32 multiturn_pos_estimate = 0.f; // Multiturn count off of estimator angle

static volatile bool is_first_run = true; // Do we need to reset encoder vel on start?

static volatile f32 encoder_dir = 1; // Encoder direction - +1 or -1
static volatile f32 phase_offset = 0.f; // Phase offset in electrical radians

// Per-revolution angle-error LUT (see encoder.h). Default all-zeros + disabled = exact no-op.
static volatile f32 enc_lut[ENC_LUT_N];
static volatile bool enc_lut_enabled = false;

void encoder_init() {
    is_first_run = true;
}

// Predict new pos/vel `t_pred` seconds into the future from the last estimate.
static __fn_inline posvel_t encoder_predict(const f32 t_pred) {
    posvel_t posvel_estimate_ = posvel_estimate;

    return (posvel_t){
        .pos = posvel_estimate_.pos + t_pred * posvel_estimate_.vel,
        .vel = posvel_estimate_.vel
    };
}

// Takes a delta difference in a 0-1 wrapping coordinate systems, returns the wraparound-corrected difference (-0.5-0.5)
// If delta > 0 and delta < 0.5, then it's a positive non-wraparound difference => return delta
// If delta > 0 and delta > 0.5, then it's a negative difference with wraparound => return delta-1
// If delta < 0 and delta > -0.5, then it's a negative non-wraparound difference => return delta
// If delta < 0 and delta < -0.5 then it's a positive difference with wraparound => return delta+1
__ccmram f32 circular_delta(f32 delta) {
    f32 wrap_adj = roundf(delta); // Delta < -0.5 => -1, -0.5 < delta < 0.5 => 0, 0.5 < delta => 1
    return delta - wrap_adj;
}

// VERY RESTRICTIVE circular wrap to 0-1
// x MUST be [-1, 2) or this gonna fuck up
__ccmram f32 fast_fmod_01(f32 x) {
    return x + (f32)(x < 0.f) - (f32)(x >= 1.f);
}

// Nice clock math enabled encoder estimator
// Keeps all values within 0-1 to avoid float issues
// IF POS_ESTIMATE EVER DIVERGES MORE THAN 0.5 TURNS FROM POS THEN THIS SHIT WILL GO UNSTABLE
__ccmram void encoder_update(u16 meas) {
    f32 enc_pos = (f32)meas / (f32)ENCODER_CPR; // 0-1

    // Per-rev angle-error correction, indexed by the RAW measurement (the only thing we have at
    // runtime), wrap-safe lerp between bins. Applied before the PLL so position, velocity,
    // multiturn and the electrical phase all benefit. ~15 cycles; no-op until a LUT is loaded.
    if (enc_lut_enabled) {
        f32 x = enc_pos * (f32)ENC_LUT_N;
        u32 i = (u32)x;
        f32 frac = x - (f32)i;
        f32 a = enc_lut[i & (ENC_LUT_N - 1u)];
        f32 b = enc_lut[(i + 1u) & (ENC_LUT_N - 1u)];
        enc_pos = fast_fmod_01(enc_pos + a + frac * (b - a)); // |corr| << 1 -> stays in [-1,2)
    }

    // If this is the first time we're running the estimator as part of the control loop, we need to reset the
    // pos/vel estimate so that it's not off by more than half a turn
    // TODO check if this is needed
    if (is_first_run) {
        posvel_estimate = (posvel_t){.pos = enc_pos, .vel = 0.f};
        multiturn_pos_raw = 0;
        multiturn_pos_estimate = 0;
        last_enc_pos = enc_pos;
        is_first_run = false;
    }

    f32 enc_delta = circular_delta(enc_pos - last_enc_pos);
    last_enc_pos = enc_pos;

    posvel_t posvel_predict = encoder_predict(enc_meas_period); // Get new pos/vel predictions
    posvel_predict.pos = fast_fmod_01(posvel_predict.pos); // Wrap prediction to 0-1

    // This could be changed to a +/- 0.5 saturation function
    f32 pos_err = circular_delta(enc_pos - posvel_predict.pos);

    posvel_t posvel_est_corrected = {
        .pos = fast_fmod_01(posvel_predict.pos + enc_meas_period * pll_kp * pos_err),
        .vel = posvel_predict.vel + enc_meas_period * pll_ki * pos_err
    };

    multiturn_pos_raw += enc_delta;
    multiturn_pos_estimate += circular_delta(posvel_est_corrected.pos - posvel_estimate.pos);

    posvel_estimate = posvel_est_corrected;
}


__ccmram posvel_t encoder_get_phase() {
    posvel_t posvel_estimate_ = posvel_estimate;

    // 0-1 -> 2pi * pole_pairs
    f32 elec_rad_per_rev = (f32)MOTOR_NUM_POLEPAIRS * 2 * M_PI_f32;

    // Map 0-1 -> 2pi*pole_pairs and correct for direction
    //f32 phase = elec_rad_per_rev * posvel_estimate_.pos * encoder_dir;
    f32 phase = wrap_pm_pi(elec_rad_per_rev * (posvel_estimate_.pos - phase_offset)) * encoder_dir;

    // Subtract phase offset and wrap to -pi-pi
    //f32 phase_pos = wrap_pm_pi(phase - phase_offset);
    f32 phase_pos = /*wrap_pm_pi(phase)*/ phase;

    // Mul vel by 2pi*pole_pairs (so that it's in rad/s) and correct for direction
    f32 phase_vel = elec_rad_per_rev * encoder_dir * posvel_estimate_.vel;

    return (posvel_t){phase_pos, phase_vel};
}

__ccmram posvel_t encoder_get_pos_vel() {
    return posvel_estimate;
}

__ccmram f32 encoder_get_multiturn_pos_raw() {
    return multiturn_pos_raw;
}

f32 encoder_get_multiturn_pos_est() {
    return multiturn_pos_estimate;
}

void encoder_set_phase_offset(f32 offset) {
    phase_offset = offset;
}

f32 encoder_get_phase_offset() {
    return phase_offset;
}

void encoder_set_lut(const f32 *table) {
    for (u32 i = 0; i < ENC_LUT_N; i++)
        enc_lut[i] = table[i];
    enc_lut_enabled = true;
}

void encoder_clear_lut() {
    enc_lut_enabled = false;
    for (u32 i = 0; i < ENC_LUT_N; i++)
        enc_lut[i] = 0.f;
}

bool encoder_lut_enabled_get() {
    return enc_lut_enabled;
}

void encoder_lut_copy(f32 *dst) {
    for (u32 i = 0; i < ENC_LUT_N; i++)
        dst[i] = enc_lut[i];
}

void encoder_set_direction(f32 dir) {
    encoder_dir = dir;
}

__ccmram f32 encoder_get_direction() {
    return encoder_dir;
}


__ccmram bool test_encoder_consumer(u16 enc_val) {
    encoder_update(enc_val);
    return true;
}

__ccmram bool test_current_consumer(f32 A_i, f32 B_i, f32 C_i, f32 V_vbus) {
    return true;
}

__ccmram bool test_pwm_producer(svm_vec_t* mag) {
    *mag = SVM((ab_vec_t){0.f, 0.f});
    return true;
}

__ccmram bool test_is_complete() {
    return false;
}

void test_init() {
}

bool test_finish(bool cancelled) {
    return true;
}

__ccmram void test_exec() {
}

control_law_t encoder_test_control_law = {
    .encoder_consumer = test_encoder_consumer,
    .current_consumer = test_current_consumer,
    .pwm_producer = test_pwm_producer,
    .control_law_exec = test_exec,
    .is_complete = test_is_complete,
    .control_law_init = test_init,
    .control_law_finish = test_finish
};
