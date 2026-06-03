#pragma once

#include "typedefs.h"
#include "mathutil.h"


// ABC phase magnitude
typedef struct svm_vec {
    f32 A;
    f32 B;
    f32 C;
} svm_vec_t;

typedef struct ab_vec {
    f32 alpha;
    f32 beta;
} ab_vec_t;

typedef struct dq_vec {
    f32 d;
    f32 q;
} dq_vec_t;

// Clarke transform - three phase currents (Ia/Ib/Ic) to two phase (Ialpha/Ibeta)
ab_vec_t clarke_transform(svm_vec_t three_phase_currents);

// Park transform - two phase currents (Ialpha/Ibeta) to stationary frame d/q. Motor phase is electrical phase in 0-2pi
dq_vec_t park_transform(ab_vec_t ab_phase_currents, f32 motor_phase);

// Inverse park transform - stationary frame voltages (Vd/Vq) to Valpha/Vbeta.
ab_vec_t inv_park_transform(dq_vec_t dq_voltage_mod, f32 motor_phase);

// Space vector modulation - effectively inverse clarke w/ third harmonic injection. Valpha/Vbeta to Va/Vb/Vc (PWM timings)
svm_vec_t SVM(ab_vec_t mod);


bool svm_output_valid(svm_vec_t svm_magnitudes);

