#pragma once

#include <math.h>
#include "arm_math_types.h"
#include "dsp/none.h"
#include "dsp/support_functions.h"
#include "stm32g4xx_ll_cordic.h"
#include "typedefs.h"

//#define SINCOS_FNCS_USE_CMSIS
#define SINCOS_FNCS_USE_CORDIC
//#define SINCOS_FNCS_USE_CMATH


#if defined(SINCOS_FNCS_USE_CMSIS) || defined(SINCOS_FNCS_USE_CMATH)
#define SINCOS_FNCS_ARE_PURE
#endif


#define __cordic_fn_attr
#define __cmath_fn_attr __fn_const
#define __cmsis_fn_attr __fn_const

#if defined(SINCOS_FNCS_USE_CMSIS)
#define __sincos_attr __cmsis_fn_attr

#elif defined(SINCOS_FNCS_USE_CMATH)
#define __sincos_attr __cmath_fn_attr

#elif defined(SINCOS_FNCS_USE_CORDIC)
#define __sincos_attr __cordic_fn_attr
#endif

#if defined(SINCOS_FNCS_ARE_PURE)
#define __sincos_reliant_attr __sincos_attr
#endif



/*
typedef struct three_phase_current {
    f32 A;
    f32 B;
    f32 C;
} three_phase_current_t;

typedef struct alphabeta_frame_current {
    f32 alpha;
    f32 beta;
} alphabeta_frame_current_t;

three_phase_current_t (alphabeta_frame_current_t alphabeta_currents);

alphabeta_frame_current_t clarke_transform(three_phase_current_t phase_currents);
 */


// Return type from simultanious sin/cos function
typedef struct sincos {
    f32 sin;
    f32 cos;
} sincos_t;

// Our chosen function implementations of sin/cos
f32 sin_f32(f32 x);

f32 cos_f32(f32 x);

f32 sqrt_f32(f32 x);

sincos_t sincos_f32(f32 x);

i32 round_f32(f32 x);

f32 fast_atan2(f32 y, f32 x);

// One-liners defined here, not in mathutil.c: out-of-line they cost the fast loop a
// CCM→flash veneer call (showed up as __clamp_f32_veneer in the 2026-06-09 per-line profile).
static __fn_inline __fn_const f32 min_f32(const f32 a, const f32 b) {
    return a < b ? a : b;
}

static __fn_inline __fn_const f32 max_f32(const f32 a, const f32 b) {
    return a > b ? a : b;
}

static __fn_inline __fn_const f32 clamp_f32(const f32 lower, const f32 x, const f32 upper) {
    return max_f32(min_f32(upper, x), lower);
}

static __fn_inline __fn_const f32 abs_f32(const f32 x) {
    return fabsf(x); // vabs.f32 inline at any -O level
}


static const f32 M_PI_f32 = 3.14159265358979323846f;
static const f32 one_over_sqrt3 = 0.57735026919f;
static const f32 two_over_sqrt3 = 1.15470053838f;
static const f32 sqrt3_over_2 = 0.86602540378f;


//__attribute__((optimize("-fno-finite-math-only")))
inline bool is_nan(f32 x) {
    return __builtin_isnan(x);
}


#define MAX(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define MIN(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define CLAMP(x, min, max) \
   ({ __typeof__ (x) _x = (x); \
    __typeof__ (min) _min = (min); \
       __typeof__ (max) _max = (max); \
     _x < _min ? _min : (_x > _max ? _max : x); })

// Convert 12-bit ADC reading to millivolts, with a voltage reference
static __fn_inline u32 adc_reading_to_mV(u16 adc_reading, u16 vvdd_mV) {
    return (((u32) vvdd_mV) * ((u32) adc_reading)) >> 12;
}

#define VVCC_CORRECT(val_12bit, vvcc) (((f32) val_12bit)/4095.f * vvcc)
#define VVCC_CORRECT_mV(val_12bit, vvcc_mV) ((f32) (((u32) val_12bit) * ((u32) vvcc_mV)) / 1000.f)


// FPv4-SP has no float-rounding instructions, and arm-gcc 14 won't expand
// roundf/rintf/lrintf to vcvtr either — they stay libm calls (they showed up in
// PC-sampling profiles of the fast loop). Hand-roll the one-instruction versions.
// vcvtr rounds per FPSCR.RMode: round-to-nearest-even at reset, never changed by
// this firmware. Valid for |x| < 2^31.
static __fn_inline f32 rint_f32(const f32 x) { // round to nearest integral f32
    f32 r;
    __asm__ ("vcvtr.s32.f32 %0, %1\n\tvcvt.f32.s32 %0, %0" : "=t" (r) : "t" (x));
    return r;
}

static __fn_inline i32 lrint_f32(const f32 x) { // round to nearest i32
    f32 t;
    i32 r;
    __asm__ ("vcvtr.s32.f32 %1, %2\n\tvmov %0, %1" : "=r" (r), "=&t" (t) : "t" (x));
    return r;
}

static __fn_inline __fn_const f32 wrap_pm(f32 x, f32 y) {
    // rint_f32, not nearbyintf: same round-to-nearest-even, no libm call.
    // Requires |x/y| < 2^31 — always true for the angle wrapping this is used for.
    f32 intval = rint_f32(x / y);
    return x - intval * y;
}


static __fn_inline __fn_const __fn_flatten f32 wrap_pm_pi(f32 x) {
    //f32 a = fmodf(x, 2.f*M_PI_f32);
    //return (a >= 0) ? a : (a + 2.f*M_PI_f32);
    return wrap_pm(x, 2.f * M_PI_f32);
}

static __fn_inline f32 square_f32(f32 x) {
    return x * x;
}

typedef enum {
    CORDIC_MODE_SINCOS,
    CORDIC_MODE_SQRT
} cordic_mode_t;

static __fn_inline void cordic_setup(cordic_mode_t mode) {
    LL_CORDIC_Config(CORDIC,
                     LL_CORDIC_FUNCTION_COSINE,
                     LL_CORDIC_PRECISION_6CYCLES,
                     LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_1,
                     LL_CORDIC_NBREAD_1,
                     LL_CORDIC_INSIZE_16BITS,
                     LL_CORDIC_OUTSIZE_16BITS
    );

}

// Hardware sincos via the CORDIC peripheral, q1.15 packed mode (cordic_setup must have
// run with CORDIC_MODE_SINCOS — done once at boot in app_main, never reconfigured).
// Single-context only: WDATA/RDATA is a stateful two-step, so concurrent use from a
// preempting ISR would interleave and corrupt both results. Today the only runtime
// caller is fast-loop context (sincos_f32); the backemf fit deliberately uses
// sincos_cmath instead.
static const u32 cordicin = 0x7fff0000; // ARG2 (high half) = modulus, ~+1.0 in q1.15
static __fn_inline sincos_t cordic_sincos_f32(const f32 theta) {
    // Angle in q1.15 units of pi: truncating the i32 to u16 wraps mod 2^16 == mod 2*pi,
    // so no float-side wrap needed (valid for |theta| < ~2^31 LSBs ≈ 2e5 rad).
    // OR (not +) keeps a negative angle's sign bits out of ARG2.
    u16 thetaq15 = (u16) (u32) lrint_f32(theta * (32768.f / M_PI_f32));
    CORDIC->WDATA = cordicin | thetaq15;
    u32 out0 = CORDIC->RDATA; // read stalls the bus until the result is ready (~6 cyc)
    i16 sin_q15 = (i16) (out0 >> 16);
    i16 cos_q15 = (i16) (out0 & 0xFFFF);
    // Outputs are modulus-scaled: q15 result = sincos * 0x7fff, so normalize by 32767.
    f32 cos_out = (f32) cos_q15 / 32767.0f;
    f32 sin_out = (f32) sin_q15 / 32767.0f;

    return (sincos_t) {sin_out, cos_out};
}

/*
static __fn_inline sincos_t cordic_sincos_f32(f32 x) {

    f32 x_wrapped = wrap_pm_pi(x);
    f32 x_pm_one = x_wrapped / M_PI_f32;

    q31_t x_q31;
    q63_t x_q63;
    //arm_float_to_q31(&x_pm_one,&x_pm_one);

    f32 in = x_pm_one * 2147483648.0f;
    in += in > 0.f ? 0.5f : -0.5f;
    x_q63 = (q63_t) in;
    //x_q31 = (((q31_t) (x_q63 >> 32) != ((q31_t) x >> 31)) ?
    //         ((0x7FFFFFFF ^ ((q31_t) (x_q63 >> 63)))) : (q31_t) x);
    x_q31 = clip_q63_to_q31(x_q63);
    //x_q31 = (q31_t) (((q31_t) (x_q63 >> 32) != ((q31_t) x >> 31)) ?
           //((0x7FFFFFFF ^ ((q31_t) (x_q63 >> 63)))) : (q31_t) x);

    //arm_float_to_q31(&x_pm_one,&x_q31,1);




    LL_CORDIC_WriteData(CORDIC, x_q31);
    q31_t cordic_out_cos = (q31_t) LL_CORDIC_ReadData(CORDIC);
    q31_t cordic_out_sin = (q31_t) LL_CORDIC_ReadData(CORDIC);

    f32 cordic_out_cos_f32 = ((f32) cordic_out_cos) / 2147483648.0f;
    f32 cordic_out_sin_f32 = ((f32) cordic_out_sin) / 2147483648.0f;

    return (sincos_t) {cordic_out_sin_f32, cordic_out_cos_f32};

}
*/

sincos_t sincos_cmath(f32 x);