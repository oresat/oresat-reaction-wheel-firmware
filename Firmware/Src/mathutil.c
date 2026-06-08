#include "mathutil.h"

#include "cordic.h"

#include "dsp/fast_math_functions.h"
#include "periphmap.h"

#include "util.h"

#include "arm_math.h"
#include "arm_common_tables.h"
#include <math.h>

//#define M_PI_f32 ((f32) M_PI)


// Decls
sincos_t sincos_cordic(f32 x);

sincos_t sincos_cordic_raw(f32 x);

sincos_t sincos_cmsis(f32 x);

//sincos_t sincos_cmath(f32 x);

f32 sin_cmsis(f32 x);

f32 cos_cmsis(f32 x);

f32 sin_cmath(f32 x);

f32 cos_cmath(f32 x);

// Our chosen function implementations of sin/cos
__sincos_attr f32

sin_f32(const f32 x) {
#if defined(SINCOS_FNCS_USE_CMSIS)
    return sin_cmsis(x);
#elif defined(SINCOS_FNCS_USE_CMATH)
    return sin_cmath(x);
#elif defined(SINCOS_FNCS_USE_CORDIC)
    return sin_cordic(x);
#endif
}

__sincos_attr f32

cos_f32(const f32 x) {
#if defined(SINCOS_FNCS_USE_CMSIS)
    return cos_cmsis(x);
#elif defined(SINCOS_FNCS_USE_CMATH)
    return cos_cmath(x);
#elif defined(SINCOS_FNCS_USE_CORDIC)
    return cos_cordic(x);
#endif
}

__ccmram __sincos_attr sincos_t

sincos_f32(const f32 x) {
    //return cordic_sincos_f32(x);

    //return cordic_sincos_f32(x);
    //gpio_debug2(true);
#if defined(SINCOS_FNCS_USE_CMSIS)
    sincos_t sincos = sincos_cmsis(x);
#elif defined(SINCOS_FNCS_USE_CMATH)
    sincos_t  sincos = sincos_cmath(x);
#elif defined(SINCOS_FNCS_USE_CORDIC)
    sincos_t sincos = sincos_cordic(x);
#endif
    //gpio_debug2(false);
    return sincos;
}

__cordic_fn_attr sincos_t
sincos_cordic(const f32 x) { // TODO should we flatten this? squish da cordic.
    // x is angle, in radians
    //f32 x_local = x;

    //// Wrap to +/- 2pi (+/-360deg)
    //while (x_local > 2.f * M_PI_f32) x_local -= 2.f * M_PI_f32;
    //while (x_local < -2.f * M_PI_f32) x_local += 2.f * M_PI_f32;

    //// Wrap to +/- pi (+/-180deg)
    //if (x_local > M_PI_f32) x_local = x_local - 2.f * M_PI_f32;
    //if (x_local < -M_PI_f32) x_local = x_local + 2.f * M_PI_f32;

    //// Divide x by pi to get the +/- 1 range CORDIC uses

    //f32 x_scale1 = x_local / M_PI_f32;

    //q31_t x_q31;
    //arm_float_to_q31(&x_scale1, &x_q31, 1u);

    //// CORDIC config in user code cordic.c. Pray to god it got set properly.

    //q31_t cordic_in[1] = {x_q31};
    //q31_t cordic_out[2] = {0, 0};

    //assert_param(HAL_CORDIC_GetState(cordic_handle) == HAL_CORDIC_STATE_READY);

    //HAL_CORDIC_Calculate(&CORDIC_HANDLE, cordic_in, cordic_out, 1, 0xFFFF);

    //f32 cordic_out_float[2] = {0.f, 0.f};
    //arm_q31_to_float(cordic_out, cordic_out_float, 2);

    //sincos_t ret;
    //ret.cos = cordic_out_float[0];
    //ret.sin = cordic_out_float[1];
    //return ret;
}

// Don't even touch this
/*
sincos_t sincos_cordic_raw(const f32 x) {
    sincos_t ret;
    u32 cordicin = 0x7FFF0000;
    i16 thetashort = x * 10430;
    cordicin += thetashort;
    hcordic.Instance->WDATA = cordicin;
    u32 out = hcordic.Instance->RDATA;
    i16 out2 = (out & 0xFFFF0000) >> 16;
    i16 out1 = (out & 0xFFFF);
    ret.cos = (f32) out1 / 32768.f;
    ret.sin = (f32) out2 / 32768.f;
    return ret;
}
 */

__ccmram __cmsis_fn_attr sincos_t
sincos_cmsis(const f32 x) {
    sincos_t ret;
    arm_sin_cos_f32(x * 180.f / M_PI_f32, &ret.sin, &ret.cos);
    return ret;
}

__cmath_fn_attr sincos_t
sincos_cmath(const f32 x) {
    sincos_t ret;
    ret.sin = sin_cmath(x);
    ret.cos = cos_cmath(x);
    return ret;
}

__cmsis_fn_attr f32
sin_cmsis(const f32 x) {
    return arm_sin_f32(x);
}

__cmsis_fn_attr f32

cos_cmsis(const f32 x) {
    return arm_cos_f32(x);
}

__cmath_fn_attr f32

sin_cmath(const f32 x) {
    return sinf(x);
}

__cmath_fn_attr f32

cos_cmath(const f32 x) {
    return cosf(x);
}

__fn_const i32 round_f32(const f32 x) {
    return (i32) roundf(x);
}

__fn_const f32 abs_f32(const f32 x) {
    return fabsf(x);
}

__fn_const __fn_leaf f32 min_f32(f32 a, f32 b) {
    if (a < b) return a;
    return b;
}

__fn_const __fn_leaf f32 max_f32(f32 a, f32 b) {
    if (a > b) return a;
    return b;
}

__fn_const __fn_flatten f32 clamp_f32(f32 lower, f32 x, f32 upper) {
    return max_f32(min_f32(upper, x), lower);
}

__fn_const __fn_flatten f32 fast_atan2(f32 y, f32 x) {
    f32 abs_y = abs_f32(y);
    f32 abs_x = abs_f32(x);
    f32 a = min_f32(abs_x, abs_y) / (max_f32(abs_x, abs_y) + FLT_MIN);
    f32 s = a * a;
    f32 r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (abs_y > abs_x) r = 1.57079637f - r;
    if (x < 0.f) r = 3.14159274f - r;
    if (y < 0.f) r = -r;
    return r;
}

__fn_const __fn_flatten f32 sqrt_f32(f32 x) {
    return sqrtf(x);
}
