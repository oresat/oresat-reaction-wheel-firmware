#include "bldc_math.h"

// SVM transform: A/B stator-oriented quadrature to PWM timings
__fn_const __fn_leaf svm_vec_t SVM(const ab_vec_t ab) {
    f32 tA, tB, tC;
    i32 sextant;
//#define one_over_sqrt3 0.57735026919f
//#define two_over_sqrt3 1.15470053838f
    f32 beta = ab.beta;
    f32 alpha = ab.alpha;

    if (beta >= 0.f) {
        if (alpha >= 0.f)
            sextant = (one_over_sqrt3 * beta > alpha) ? 2 : 1;
        else
            sextant = (-one_over_sqrt3 * beta > alpha) ? 3 : 2;
    } else {
        if (alpha >= 0.f)
            sextant = (-one_over_sqrt3 * beta > alpha) ? 5 : 6;
        else {
            sextant = (one_over_sqrt3 * beta > alpha) ? 4 : 5;
        }
    }

    switch (sextant) {
        case 1: {
            f32 t1 = alpha - one_over_sqrt3 * beta;
            f32 t2 = two_over_sqrt3 * beta;
            tA = (1.f - t1 - t2) * .5f;
            tB = tA + t1;
            tC = tB + t2;
        }
            break;
        case 2 : {
            f32 t2 = alpha + one_over_sqrt3 * beta;
            f32 t3 = -alpha + one_over_sqrt3 * beta;

            tB = (1.f - t2 - t3) * .5f;
            tA = tB + t3;
            tC = tA + t2;
        }
            break;
        case 3: {
            f32 t3 = two_over_sqrt3 * beta;
            f32 t4 = -alpha - one_over_sqrt3 * beta;

            tB = (1.f - t3 - t4) * .5f;
            tC = tB + t3;
            tA = tC + t4;
        }
            break;
        case 4: {
            f32 t4 = -alpha + one_over_sqrt3 * beta;
            f32 t5 = -two_over_sqrt3 * beta;

            tC = (1.0f - t4 - t5) * .5f;
            tB = tC + t5;
            tA = tB + t4;
        }
            break;
        case 5: {
            f32 t5 = -alpha - one_over_sqrt3 * beta;
            f32 t6 = alpha - one_over_sqrt3 * beta;

            tC = (1.f - t5 - t6) * .5f;
            tA = tC + t5;
            tB = tA + t6;
        }
            break;
        case 6: {
            f32 t6 = -two_over_sqrt3 * beta;
            f32 t1 = alpha + one_over_sqrt3 * beta;

            tA = (1.f - t6 - t1) * .5f;
            tC = tA + t1;
            tB = tC + t6;
        }
            break;
    }

    svm_vec_t ret = {1.f - tA, 1.f - tB, 1.f - tC};
    return ret;

}

__fn_const __fn_leaf bool svm_output_valid(const svm_vec_t svm_magnitudes) {
    return svm_magnitudes.A >= 0.f && svm_magnitudes.A <= 1.f
           && svm_magnitudes.B >= 0.f && svm_magnitudes.B <= 1.f
           && svm_magnitudes.C >= 0.f && svm_magnitudes.C <= 1.f;

}

__fn_const __fn_leaf ab_vec_t clarke_transform(const svm_vec_t three_phase_currents) {
    return (ab_vec_t) {
            .alpha = three_phase_currents.A,
            .beta  = one_over_sqrt3 * (three_phase_currents.B - three_phase_currents.C)
    };
}

__fn_const __fn_leaf dq_vec_t park_transform(const ab_vec_t ab_phase_currents, const f32 motor_phase) {
    f32 Ia = ab_phase_currents.alpha;
    f32 Ib = ab_phase_currents.beta;
    sincos_t phase_sincos = sincos_f32(motor_phase);
    return (dq_vec_t) { // Effectively just a rotation. Not sure why we named it after that Park guy.
            .d = Ia * phase_sincos.cos + Ib * phase_sincos.sin,
            .q = Ib * phase_sincos.cos - Ia * phase_sincos.sin
    };
}

__fn_const __fn_leaf ab_vec_t inv_park_transform(const dq_vec_t dq_voltage_mod, const f32 motor_phase) {
    sincos_t phase_sincos = sincos_f32(motor_phase);
    return (ab_vec_t) { // Just an inverse rotation
            .alpha = phase_sincos.cos * dq_voltage_mod.d - phase_sincos.sin * dq_voltage_mod.q,
            .beta = phase_sincos.cos * dq_voltage_mod.q + phase_sincos.sin * dq_voltage_mod.d
    };
}