#pragma once

#include <cstdint>

namespace alg {

struct Ramp {
    float input = 0.0f;
    float out = 0.0f;
    float min_value = 0.0f;
    float max_value = 0.0f;
    float frame_period = 0.0f;
    uint8_t is_completed = 1;

    void init(float frame_period_s, float max, float min);
    float step(float input_per_s);
};

void ramp_init(Ramp* ramp, float frame_period, float max, float min);
float ramp_calc(Ramp* ramp, float input);

} // namespace alg

// Legacy aliases/exports
using ramp_function_source_t = alg::Ramp;
using alg::ramp_init;
using alg::ramp_calc;
