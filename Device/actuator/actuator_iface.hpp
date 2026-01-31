#pragma once

#include <cstdint>

#include "bsp_can_port.h"

namespace actuator {

enum class Mode : uint8_t {
    Disabled = 0,
    Current,
    Omega,
    Angle,
    Torque,
};

struct Cmd {
    Mode mode = Mode::Disabled;

    float target_angle = 0.0f;
    float target_omega = 0.0f;
    float target_current = 0.0f;
    float target_torque = 0.0f;
};

struct State {
    float angle = 0.0f;
    float omega = 0.0f;
    float current = 0.0f;
    float torque = 0.0f;
    float temperature = 0.0f;
    bool online = false;
};

class IActuator {
public:
    virtual ~IActuator();

    // called during bring-up
    virtual void Bind(BspCanHandle can) = 0;

    // called by CAN dispatch
    virtual void OnRx(const BspCanFrame* frame) = 0;

    // control
    virtual void SetCmd(const Cmd& cmd) = 0;

    // optional periodic update (PID etc)
    virtual void Update(float dt_s) = 0;

    // publish to low-level tx topics or call Output()
    virtual void PublishTx() = 0;

    virtual const State& GetState() const = 0;
};

} // namespace actuator
