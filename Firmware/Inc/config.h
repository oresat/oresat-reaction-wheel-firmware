#pragma once

#include "typedefs.h"

// DCBus
static const f32 DC_BUS_OVERVOLTAGE_THRESHOLD_V = 4.3f * 2.f; // V
static const f32 DC_BUS_UNDERVOLTAGE_THRESHOLD_V = 2.7f; // TODO ************ REMEMBER TO CHANGE THIS ************

// IBus
static const f32 IBUS_OVERCURRENT_THRESHOLD_A = 5.f; // A
static const f32 IBUS_OVERREGEN_THRESHOLD_A = 5.f; // A

// Motor current
static const f32 MOTOR_CURRENT_CMD_LIMIT = 5.0f; // A
static const f32 MOTOR_CURRENT_TRIP_LIMIT = 15.f; // A


// Inverter constants
static const f32 SHUNT_AMP_GAIN = -50.f; // V/V
static const f32 SHUNT_VALUE_OHM = 0.002f; // Ohm
static const f32 SHUNT_AMP_CONDUCTANCE = 1 / (SHUNT_AMP_GAIN * SHUNT_VALUE_OHM); // A/V
static const f32 VSNS_VOLTAGE_DIVIDER = 3240.f / (3240.f + 5100.f); // V/V
//static const f32 VSNS_VOLTAGE_DIVIDER = 0.4; // V/V

// Safety
static const f32 MAX_CURRENT_LEAKAGE_AMOUNT = 1.0f; // 100mA max phase current imbalance allowed

// Encoder
static const f32 ENCODER_ESTIMATOR_BANDWIDTH = 1000.f; // tau (1/s)
static const u16 ENCODER_CPR = 16384;

// Motor calibration parameters
static const f32 MAX_RESISTANCE_CAL_VOLTAGE = 4.f; // V
static const f32 MOTOR_CALIBRATION_CURRENT = 2.f; // A
static const u32 RESISTANCE_CAL_TIME = 3000; // ms

static const f32 MOTOR_PHASE_RESISTANCE_MIN = 0.01f; //ohm
static const f32 MOTOR_PHASE_RESISTANCE_MAX = 1.0f; //ohm

static const u32 INDUCTANCE_CAL_TIME = 1000; // ms
static const f32 MOTOR_PHASE_INDUCTANCE_MIN = 1 * (0.001f) * (0.001f); // uH -> H
static const f32 MOTOR_PHASE_INDUCTANCE_MAX = 1000 * (0.001f) * (0.001f); // uH -> H
static const f32 MAX_INDUCTANCE_CAL_VOLTAGE = 1.0f; // V

static const f32 HARDCODE_PHASE_INDUCTANCE = 6.39e-6f / 2.f; // H
static const f32 HARDCODE_PHASE_RESISTANCE = 196.0e-3f / 2.f; // Ohm
static const f32 CURRENT_CONTROLLER_BANDWIDTH = 4000.f; // 1/s

#define MOTOR_NUM_POLEPAIRS 6 //fuck

static const f32 LOCKIN_SPIN_CURRENT = 4.f;

static const f32 ENCODER_DIR_CAL_ROTATION_DIST = 1.f; // rev
static const f32 ENCODER_DIR_CAL_ROTATION_VEL = 0.5f; // rev/s
static const f32 ENCODER_DIR_CAL_ROTATION_TOLERANCE = 0.1f; // 10% error ok

static const f32 ENCODER_PHASE_CAL_ROTATION_DIST = 4.f; // rev
static const f32 ENCODER_PHASE_CAL_ROTATION_VEL = 0.5f; // rev/s


static const f32 MOTOR_KV = 6300.f;
static const f32 MOTOR_TORQUE_CONSTANT = 8.27f / MOTOR_KV;


static const u8 CANID_WHEEL1 = 0x3C;
static const u8 CANID_WHEEL2 = 0x40;
static const u8 CANID_WHEEL3 = 0x44;
static const u8 CANID_WHEEL4 = 0x48;

static const f32 VEL_RAMP_RATE = 200.f; //rev/s^2

static const f32 VEL_GAIN = 0.002f; // Nm/rev/s
static const f32 VEL_INTEGRATOR_GAIN = 0.001f; // Nm/rev
static const f32 VEL_INTEGRATOR_LIMIT = 1.f * MOTOR_TORQUE_CONSTANT; // Nm

static const bool FOC_USE_bEMF_FF = false;
static const bool FOC_USE_wL_FF = false;
static const bool FOC_USE_R_FF = false;
static const bool FOC_USE_DECOUPLING = true;
