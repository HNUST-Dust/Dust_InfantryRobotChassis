#include "dji_c6xx_min.hpp"

#include "cmsis_os2.h"

#include "utils/alg_constrain.h"

extern "C" {
#include "FreeRTOS.h" // NOLINT(misc-include-cleaner)
#include "task.h"
}

static_assert(configASSERT_DEFINED == 1, "configASSERT_DEFINED expected");

#include "../../../communication_topic/actuator_cmd_topics.hpp"


#include <cmath>
#include <cstring>

namespace actuator::drivers {

namespace {
inline void pack_i16_be(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}

inline uint16_t u16_be(const uint8_t* p) {
    return (static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]);
}
inline int16_t i16_be(const uint8_t* p) {
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

using alg::float_constrain;

struct DjiPidCtx {
    uint8_t index = 0;
    DjiC6xxMin* motor = nullptr;
};

struct DjiGroupTxCtx {
    DjiC6xxGroupTxConfig cfg{};
};

static volatile int16_t s_current_raw[4] = {0, 0, 0, 0};
static volatile bool s_dirty = false;

static DjiGroupTxCtx s_group_ctx{};

static DjiC6xxMin* s_motors[4] = {nullptr, nullptr, nullptr, nullptr};

static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[512];
static osThreadId_t s_task_thread = nullptr;

static void dji_group_task(void*) {
    RingSub<orb::DjiC6xxOmegaCmd, 32> sub{orb::dji_c6xx_omega_cmd};

    for (;;) {
        bool updated = false;

        // consume all pending target updates
        orb::DjiC6xxOmegaCmd cmd{};
        while (sub.copy(cmd)) {
            if (cmd.bus != s_group_ctx.cfg.bus) {
                continue;
            }

            DjiC6xxMin* m = nullptr;
            for (auto* candidate : s_motors) {
                if (candidate && candidate->rx_std_id() == cmd.rx_std_id) {
                    m = candidate;
                    break;
                }
            }
            if (!m) {
                continue;
            }
            m->SetTargetOmega(cmd.omega);
            updated = true;
        }

        // run PID update for all motors
        for (uint8_t i = 0; i < 4; ++i) {
            auto* m = s_motors[i];
            if (!m) {
                continue;
            }
            m->Update();
            s_current_raw[i] = m->target_current_raw();
        }

        if (updated) {
            s_dirty = true;
        }

        // publish group current frame
        if (s_dirty) {
            s_dirty = false;

            orb::CanTxFrame out{};
            out.bus = s_group_ctx.cfg.bus;
            out.id = s_group_ctx.cfg.group_std_id;
            out.id_type = orb::CanIdType::Std;
            out.frame_type = orb::CanFrameType::Data;
            out.is_fd = false;
            out.brs = false;
            out.len = 8;
            std::memset(out.data, 0, sizeof(out.data));

            pack_i16_be(&out.data[0], s_current_raw[0]);
            pack_i16_be(&out.data[2], s_current_raw[1]);
            pack_i16_be(&out.data[4], s_current_raw[2]);
            pack_i16_be(&out.data[6], s_current_raw[3]);

            orb::can_tx.publish(out);
        }

        osDelay(1);
    }
}
} // namespace

void DjiC6xxMin::Init(BspCanHandle can, const Config& cfg) {
    can_ = can;
    cfg_ = cfg;

    {
        alg::PidConfig pid_cfg{};
        pid_cfg.kp = cfg_.kp;
        pid_cfg.ki = cfg_.ki;
        pid_cfg.kd = cfg_.kd;
        pid_omega_.configure(pid_cfg);
    }

    last_enc_ = 0;
    total_round_ = 0;
    now_angle_ = 0.0f;
    now_omega_out_ = 0.0f;
    now_current_ = 0.0f;
    temperature_ = 0.0f;

    target_omega_out_ = 0.0f;
    target_current_ = 0.0f;
}

void DjiC6xxMin::CanRxCpltCallback(const BspCanFrame* frame) {
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }
    if (frame->id != cfg_.rx_std_id) {
        return;
    }

    const uint8_t* data = frame->data;
    const uint16_t enc = u16_be(&data[0]);
    const int16_t omega_rpm = i16_be(&data[2]);
    const int16_t current_raw = i16_be(&data[4]);
    const uint8_t temp = data[6];

    // unwrap encoder (same idea as legacy driver, but minimal)
    if (last_enc_ != 0) {
        int32_t diff = static_cast<int32_t>(enc) - static_cast<int32_t>(last_enc_);
        if (diff > 4096) {
            total_round_ -= 1;
        } else if (diff < -4096) {
            total_round_ += 1;
        }
    }
    last_enc_ = enc;

    const int32_t total_enc = total_round_ * static_cast<int32_t>(cfg_.enc_per_round) + static_cast<int32_t>(enc);
    const float motor_angle = (static_cast<float>(total_enc) / static_cast<float>(cfg_.enc_per_round)) * k2pi;
    now_angle_ = (cfg_.gearbox_ratio != 0.0f) ? (motor_angle / cfg_.gearbox_ratio) : motor_angle;

    // omega: rpm -> rad/s (motor side) then / gearbox_ratio to output side
    const float omega_motor = (static_cast<float>(omega_rpm) * k2pi) / 60.0f;
    now_omega_out_ = (cfg_.gearbox_ratio != 0.0f) ? (omega_motor / cfg_.gearbox_ratio) : omega_motor;

    // current: legacy uses 16384/20 scale; keep raw->A mapping consistent
    now_current_ = static_cast<float>(current_raw) * (20.0f / 16384.0f);
    temperature_ = static_cast<float>(temp);
}

void DjiC6xxMin::SetTargetOmega(float omega) {
    target_omega_out_ = omega;
    cfg_.method = ControlMethod::Omega;
}

void DjiC6xxMin::SetTargetCurrent(float current) {
    target_current_ = current;
    cfg_.method = ControlMethod::Current;
}

void DjiC6xxMin::Update() {
    if (cfg_.method == ControlMethod::Omega) {
        target_current_ = pid_omega_.update(target_omega_out_, now_omega_out_);
    }

    target_current_ = float_constrain(target_current_, -cfg_.current_limit, cfg_.current_limit);
}

int16_t DjiC6xxMin::target_current_raw() const {
    const float a = float_constrain(target_current_, -cfg_.current_limit, cfg_.current_limit);
    return static_cast<int16_t>(a * (16384.0f / 20.0f));
}

void DjiC6xxMin::JoinOmegaGroup(const DjiC6xxGroupTxConfig& tx_cfg) {
    configASSERT(can_ != nullptr);

    s_group_ctx.cfg = tx_cfg;

    const int32_t slot = static_cast<int32_t>(cfg_.rx_std_id) - (static_cast<int32_t>(tx_cfg.group_std_id) + 1);
    configASSERT(slot >= 0 && slot < 4);
    s_motors[slot] = this;

    if (!s_task_thread) {
        static const osThreadAttr_t attr = {
            .name = "dji_group",
            .cb_mem = &s_task_tcb,
            .cb_size = sizeof(s_task_tcb),
            .stack_mem = s_task_stack,
            .stack_size = sizeof(s_task_stack),
            .priority = (osPriority_t)osPriorityAboveNormal,
        };
        s_task_thread = osThreadNew(dji_group_task, nullptr, &attr);
        configASSERT(s_task_thread != nullptr);
    }
}

} // namespace actuator::drivers

namespace actuator::instances {

actuator::drivers::DjiC6xxMin dji_201{};
actuator::drivers::DjiC6xxMin dji_202{};
actuator::drivers::DjiC6xxMin dji_203{};
actuator::drivers::DjiC6xxMin dji_204{};

} // namespace actuator::instances
