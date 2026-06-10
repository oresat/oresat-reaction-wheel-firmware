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

f32 sin_cordic(f32 x);

f32 cos_cordic(f32 x);

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

__ccmram __cordic_fn_attr sincos_t
sincos_cordic(const f32 x) {
    return cordic_sincos_f32(x);
}

__cordic_fn_attr f32
sin_cordic(const f32 x) {
    return cordic_sincos_f32(x).sin;
}

__cordic_fn_attr f32
cos_cordic(const f32 x) {
    return cordic_sincos_f32(x).cos;
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

// Compiled only when selected: otherwise the dead sincos_cmsis stub keeps
// arm_sin_cos_f32 + its 2 KB sinTable_f32 linked AND placed in CCM (the .ld pulls
// *arm_sin_cos_f32.c.obj into .ccmram, and individual .ccmram functions can't be
// gc-section'd since they share one section).
#if defined(SINCOS_FNCS_USE_CMSIS)
__ccmram __cmsis_fn_attr sincos_t
sincos_cmsis(const f32 x) {
    sincos_t ret;
    arm_sin_cos_f32(x * 180.f / M_PI_f32, &ret.sin, &ret.cos);
    return ret;
}
#endif

__cmath_fn_attr sincos_t
sincos_cmath(const f32 x) {
    sincos_t ret;
    ret.sin = sin_cmath(x);
    ret.cos = cos_cmath(x);
    return ret;
}

#if defined(SINCOS_FNCS_USE_CMSIS)
__cmsis_fn_attr f32
sin_cmsis(const f32 x) {
    return arm_sin_f32(x);
}

__cmsis_fn_attr f32

cos_cmsis(const f32 x) {
    return arm_cos_f32(x);
}
#endif

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
