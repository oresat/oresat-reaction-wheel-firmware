#pragma once

#include "typedefs.h"

#include "stm32g474xx.h"
#include "core_cm4.h"
#include "tpiu.h"
//#include "dwt.h"
#include "itm.h"

bool swo_setup();

typedef enum debug_channels {
    _hrtim_isr ,
    _enc_vel ,
    _enc_pos ,
    _Id_meas ,
    _Iq_meas ,
    _Id_setpoint ,
    _Iq_setpoint ,
    _Vq_integral ,
    _phA_current ,
    _phB_current ,
    _phC_current ,
    _enc_phase_vel ,
    _enc_phase_pos ,
    _vbus,
    _Vd_integral ,
    _Ialpha_meas ,
    _Ibeta_meas ,
    _mod_d ,
    _mod_q ,
    _mod_alpha ,
    _mod_beta ,
    _svm_A ,
    _svm_B ,
    _svm_C ,
    _ibus ,
    _adc_callback ,
    _pwm_callback ,
    _encoder_callback ,
} debug_channel_t;


static __fn_inline void swo_print_f32(const u32 port, const f32 val) {
    //union {
    //    f32 f;
    //    u32 u;
    //} x;
    //x.f = val;
    //ITM->PORT[port].u32 = x.u;
    ITM->PORT[port].u32 = *(u32 *) &val;
}

static __fn_inline void swo_print_bool(const u32 port, const bool val) {
    ITM->PORT[port].u8 = val ? 0xaa : 0xbb;
}

#define tmp(enum_name) \
static __fn_inline __fn_flatten void debug##enum_name(const f32 val) {}
/* to enable SWO emit, swap the no-op tmp() above for this one:
#define tmp(enum_name) \
static __fn_inline __fn_flatten void debug##enum_name(const f32 val) {swo_print_f32(enum_name,val);}
*/

//@formatter:off

tmp(_phA_current)
tmp(_phB_current)
tmp(_phC_current)
tmp(_enc_vel)
tmp(_enc_pos)
tmp(_enc_phase_vel)
tmp(_enc_phase_pos)
tmp(_vbus)
tmp(_Vd_integral)
tmp(_Vq_integral)
tmp(_Ialpha_meas)
tmp(_Ibeta_meas)
tmp(_Id_meas)
tmp(_Iq_meas)
tmp(_Id_setpoint)
tmp(_Iq_setpoint)
tmp(_mod_d)
tmp(_mod_q)
tmp(_mod_alpha)
tmp(_mod_beta)
tmp(_svm_A)
tmp(_svm_B)
tmp(_svm_C)
tmp(_ibus)

//@formatter:on

#undef tmp

#define tmp(enum_name) \
static __fn_inline __fn_flatten void debug##enum_name(const bool val) {}
/* to enable SWO emit, swap the no-op tmp() above for this one:
#define tmp(enum_name) \
static __fn_inline __fn_flatten void debug##enum_name(const bool val) {swo_print_bool(enum_name,val);}
*/
//@formatter:off
tmp(_hrtim_isr)
tmp(_adc_callback)
tmp(_pwm_callback)
tmp(_encoder_callback)
//@formatter:on
#undef tmp
