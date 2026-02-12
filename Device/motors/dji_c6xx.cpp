#include "dji_c6xx.hpp"

#include "motor_registry.hpp"

#include "cmsis_os2.h"

#include "utils/alg_constrain.h"

extern "C" {
#include "FreeRTOS.h" // NOLINT(misc-include-cleaner)
#include "task.h"
}

static_assert(configASSERT_DEFINED == 1, "configASSERT_DEFINED expected");

#include "actuator_cmd_topics.hpp"

#include <cstring>

namespace actuator::drivers {

namespace {
using alg::float_constrain;

struct DjiGroupTxCtx {
    orb::CanBus bus = orb::CanBus::CAN1;
    uint16_t tx_id = 0x200;
};

// 4 个电机的目标电流 raw（将被打包进 0x200 组帧，单位是 DJI 协议的 raw current）
// 由运行时线程在每个周期调用 DjiC6xxMin::Update() 后写入。
static volatile int16_t s_current_raw[4] = {0, 0, 0, 0};

// “需要发送组帧”的脏标记：当收到新的 omega 命令并完成一次 Update 后置位。
// 当前策略是：只要有新命令到来，就触发一次组电流帧发送。
static volatile bool s_dirty = false;

// 组帧发送配置（CAN 总线 + tx_id），由任意一个电机 JoinRuntime 时写入。
static DjiGroupTxCtx s_group_ctx{};

// 电机注册表：key = (bus + tx_id + rx_id)
static MotorRegistry<DjiC6xxMin, 4> s_registry{};

// 共享运行时线程（CMSIS-RTOS2）：负责消费命令、更新 PID、发布组电流帧。
static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[512];
static osThreadId_t s_task_thread = nullptr;

static void publish_group_current()
{
    // 发布 DJI C6xx 组电流帧（0x200 默认）：每个电机 16-bit 有符号大端
    // data[0..1]=m0, [2..3]=m1, [4..5]=m2, [6..7]=m3
    orb::CanTxFrame out{};
    out.bus = s_group_ctx.bus;
    out.id = s_group_ctx.tx_id;
    out.id_type = orb::CanIdType::Std;
    out.frame_type = orb::CanFrameType::Data;
    out.is_fd = false;
    out.brs = false;
    out.len = 8;
    std::memset(out.data, 0, sizeof(out.data));

    const uint16_t c0 = static_cast<uint16_t>(static_cast<int16_t>(s_current_raw[0]));
    const uint16_t c1 = static_cast<uint16_t>(static_cast<int16_t>(s_current_raw[1]));
    const uint16_t c2 = static_cast<uint16_t>(static_cast<int16_t>(s_current_raw[2]));
    const uint16_t c3 = static_cast<uint16_t>(static_cast<int16_t>(s_current_raw[3]));

    out.data[0] = static_cast<uint8_t>((c0 >> 8) & 0xFF);
    out.data[1] = static_cast<uint8_t>(c0 & 0xFF);
    out.data[2] = static_cast<uint8_t>((c1 >> 8) & 0xFF);
    out.data[3] = static_cast<uint8_t>(c1 & 0xFF);
    out.data[4] = static_cast<uint8_t>((c2 >> 8) & 0xFF);
    out.data[5] = static_cast<uint8_t>(c2 & 0xFF);
    out.data[6] = static_cast<uint8_t>((c3 >> 8) & 0xFF);
    out.data[7] = static_cast<uint8_t>(c3 & 0xFF);

    orb::can_tx.publish(out);
}

static int32_t rx_id_to_slot(uint16_t tx_id, uint16_t rx_id)
{
    // DJI C6xx 常规映射：0x201..0x204 对应 tx_id(0x200)+1..+4
    return static_cast<int32_t>(rx_id) - (static_cast<int32_t>(tx_id) + 1);
}

static void dji_group_task(void*)
{
    // 单线程消费 orb::dji_c6xx_omega_cmd（RingTopic）并驱动 4 个电机更新 + 组帧发送。
    // 注意：RingTopic 语义下建议单消费者 drain，避免多线程“分着读”。
    RingSub<orb::DjiC6xxOmegaCmd, 32> sub{orb::dji_c6xx_omega_cmd};

    for (;;) {
        bool updated = false;

        // consume all pending target updates
        orb::DjiC6xxOmegaCmd cmd{};
        while (sub.copy(cmd)) {
            if (cmd.bus != s_group_ctx.bus) {
                continue;
            }

            DjiC6xxMin* m = s_registry.FindByBusRx(cmd.bus, cmd.rx_std_id);
            if (!m) {
                continue;
            }
            m->SetTargetOmega(cmd.omega);
            updated = true;
        }

        // run PID update for all motors
        s_registry.ForEach([&](const MotorKey& key, DjiC6xxMin& m) {
            (void)key;
            m.Update();
            const int32_t slot = rx_id_to_slot(s_group_ctx.tx_id, m.rx_id());
            if (slot >= 0 && slot < 4) {
                s_current_raw[slot] = m.target_current_raw();
            }
        });

        if (updated) {
            s_dirty = true;
        }

        // publish group current frame
        if (s_dirty) {
            s_dirty = false;
            publish_group_current();
        }

        osDelay(1);
    }
}

} // namespace

void DjiC6xxMin::Init(BspCanHandle can, const Config& cfg) {
    can_ = can;
    cfg_ = cfg;

    alg::PidConfig pid_cfg{};
    pid_cfg.kp = cfg_.kp;
    pid_cfg.ki = cfg_.ki;
    pid_cfg.kd = cfg_.kd;
    pid_omega_.configure(pid_cfg);

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
    const uint16_t enc = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
    const int16_t omega_rpm =
        static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]));
    const int16_t current_raw =
        static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5]));
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

void DjiC6xxMin::JoinRuntime(uint16_t tx_id) {
    configASSERT(can_ != nullptr);

    // 全局组帧配置：默认只支持一个 DJI C6xx 组（同一 bus + tx_id）
    s_group_ctx.bus = cfg_.bus;
    s_group_ctx.tx_id = tx_id;

    joined_tx_id_ = tx_id;

    const int32_t slot = rx_id_to_slot(tx_id, cfg_.rx_std_id);
    configASSERT(slot >= 0 && slot < 4);
    const MotorKey key{cfg_.bus, tx_id, cfg_.rx_std_id};
    const bool stored = s_registry.RegisterOrReplace(key, this);
    configASSERT(stored);

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
