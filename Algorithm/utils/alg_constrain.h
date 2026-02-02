#pragma once

#include "common/alg_common.h"

#include <cstdint>

namespace alg {

float abs_limit(float num, float limit);
float sign(float value);
float float_deadband(float value, float minValue, float maxValue);
int16_t int16_deadline(int16_t value, int16_t minValue, int16_t maxValue);
float float_constrain(float value, float minValue, float maxValue);
int16_t int16_constrain(int16_t value, int16_t minValue, int16_t maxValue);
float loop_float_constrain(float input, float minValue, float maxValue);
float theta_format(float ang);
int float_rounding(float raw);

} // namespace alg

// Legacy exports
using alg::abs_limit;
using alg::sign;
using alg::float_deadband;
using alg::int16_deadline;
using alg::float_constrain;
using alg::int16_constrain;
using alg::loop_float_constrain;
using alg::theta_format;
using alg::float_rounding;

// 弧度格式化为 -PI ~ PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)
