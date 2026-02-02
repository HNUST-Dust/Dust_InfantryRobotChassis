#ifndef LOW_PASS_FILTER_H_
#define LOW_PASS_FILTER_H_

#include "common/alg_common.h"

namespace alg {

struct LowPassFilterConfig {
    float cutoff_hz = 0.0f;
    float dt = 0.0f;
};

class LowPassFilter {
public:
    constexpr LowPassFilter() = default;

    void configure(const LowPassFilterConfig& cfg)
    {
        configure(cfg.cutoff_hz, cfg.dt);
    }

    void configure(float cutoff_hz, float dt)
    {
        if (dt <= 0.0f) {
            alpha_ = 1.0f;
        } else if (cutoff_hz <= 0.0f) {
            alpha_ = 1.0f;
        } else {
            const float rc = 1.0f / (TWO_PI * cutoff_hz);
            alpha_ = dt / (dt + rc);
        }
        output_ = 0.0f;
        configured_ = true;
    }

    float update(float input)
    {
        if (!configured_) {
            return input;
        }
        output_ = alpha_ * input + (1.0f - alpha_) * output_;
        return output_;
    }

    float value() const { return output_; }

    void reset(float value = 0.0f)
    {
        output_ = value;
    }

    bool configured() const { return configured_; }

private:
    float alpha_ = 1.0f;
    float output_ = 0.0f;
    bool configured_ = false;
};

} // namespace alg

// Legacy wrapper: keep old method names to minimize churn.
class LowPassFilter {
public:
    void Init(float cutoff_freq, float dt) { impl_.configure(cutoff_freq, dt); }
    float Update(float input) { return impl_.update(input); }
    float GetOutput() const { return impl_.value(); }
    void Reset(float value = 0.0f) { impl_.reset(value); }

private:
    alg::LowPassFilter impl_{};
};

#endif // LOW_PASS_FILTER_H_
