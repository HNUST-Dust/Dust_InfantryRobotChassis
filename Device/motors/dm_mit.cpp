#include "dm_mit.hpp"

#include "motor_registry.hpp"

#include "cmsis_os2.h"

#include "utils/alg_constrain.h"
#include "utils/alg_quantize.hpp"

extern "C" {
#include "FreeRTOS.h" // NOLINT(misc-include-cleaner)
#include "task.h"
}

static_assert(configASSERT_DEFINED == 1, "configASSERT_DEFINED expected");

#include "actuator_cmd_topics.hpp"
#include "mcu_topics.hpp"
#include "actuator_state_topics.hpp"

#include "bsp_dwt.h"

#include <cstring>

namespace actuator::drivers {

namespace {
using alg::float_constrain;
using alg::float_to_uint;
using alg::uint_to_float;

static inline alg::PidConfig make_pid_cfg(float kp,
                                         float ki,
                                         float kd,
                                         float kf,
                                         float i_out_max,
                                         float out_max,
                                         float dt,
                                         float dead_zone,
                                         float i_variable_speed_A,
                                         float i_variable_speed_B,
                                         float i_separate_threshold,
                                         alg::DFirst d_first,
                                         float d_lpf_tau)
{
    alg::PidConfig cfg{};
    cfg.kp = kp;
    cfg.ki = ki;
    cfg.kd = kd;
    cfg.kf = kf;
    cfg.i_out_max = i_out_max;
    cfg.out_max = out_max;
    cfg.dt = dt;
    cfg.dead_zone = dead_zone;
    cfg.i_variable_speed_A = i_variable_speed_A;
    cfg.i_variable_speed_B = i_variable_speed_B;
    cfg.i_separate_threshold = i_separate_threshold;
    cfg.d_first = d_first;
    cfg.d_lpf_tau = d_lpf_tau;
    return cfg;
}

static constexpr size_t kMaxMotors = 8;

struct ActiveCascade {
    DmMitMin* motor = nullptr;
    orb::DmMitCascadeCmd cmd{};
};

static ActiveCascade s_active_cascade[kMaxMotors] = {};

static inline ActiveCascade* find_active(DmMitMin* m)
{
    for (auto &e : s_active_cascade) {
        if (e.motor == m) {
            return &e;
        }
    }
    return nullptr;
}

static inline ActiveCascade* upsert_active(DmMitMin* m)
{
    if (!m) {
        return nullptr;
    }
    if (auto *e = find_active(m)) {
        return e;
    }
    for (auto &e : s_active_cascade) {
        if (e.motor == nullptr) {
            e.motor = m;
            return &e;
        }
    }
    return nullptr;
}

static inline void remove_active(DmMitMin* m)
{
    if (!m) {
        return;
    }
    for (auto &e : s_active_cascade) {
        if (e.motor == m) {
            e.motor = nullptr;
            e.cmd = {};
            return;
        }
    }
}

// 电机注册表：key = (bus + tx_id(base_std_id) + rx_id(can_rx_id))
static MotorRegistry<DmMitMin, kMaxMotors> s_registry{};

// 共享运行时线程（CMSIS-RTOS2）：消费 target/admin 两个 RingTopic。
static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[512];
static osThreadId_t s_task_thread = nullptr;

static void dm_mit_task(void*) {
    // - orb::dm_mit_target_cmd：MIT 控制指令（角度/速度/力矩 + kp/kd）
    // - orb::dm_mit_admin_cmd：进入/退出/清错/存零点
    RingSub<orb::DmMitTargetCmd, 16> target_sub{orb::dm_mit_target_cmd};
    RingSub<orb::DmMitAdminCmd, 8> admin_sub{orb::dm_mit_admin_cmd};
    RingSub<orb::DmMitCascadeCmd, 16> cascade_sub{orb::dm_mit_cascade_cmd};

    Subscription<orb::McuImu> imu_sub(orb::mcu_imu);
    orb::McuImu imu{};

    for (;;) {
        // 最新 IMU（用于云台级串级 PID 反馈）
        (void)imu_sub.copy(imu);

        // 串级目标：更新 active 列表
        {
            orb::DmMitCascadeCmd c{};
            while (cascade_sub.copy(c)) {
                DmMitMin* m = s_registry.FindByBusRx(c.bus, static_cast<uint16_t>(c.can_rx_id));
                if (!m) {
                    continue;
                }
                if (!c.enable) {
                    remove_active(m);
                    continue;
                }

                ActiveCascade* e = upsert_active(m);
                if (!e) {
                    // active 表满了：忽略该命令（防御性处理）
                    continue;
                }
                e->cmd = c;
            }
        }

        // 串级 PID 计算与下发：每 1ms 对 active motors 刷新一次力矩
        for (auto &e : s_active_cascade) {
            if (!e.motor) {
                continue;
            }
            DmMitMin* m = e.motor;
            if (m->cascade_axis() == DmMitMin::CascadeAxis::Unknown) {
                continue;
            }

            float measured_angle = 0.0f;
            float measured_omega = 0.0f;
            if (m->cascade_axis() == DmMitMin::CascadeAxis::Yaw) {
                measured_angle = imu.yaw_total_angle_f;
                measured_omega = imu.yaw_omega_f;
            } else {
                measured_angle = imu.pitch_f;
                measured_omega = imu.pitch_omega_f;
            }

            const auto mode = (e.cmd.mode == orb::DmMitCascadeMode::Omega)
                                  ? DmMitMin::CascadeMode::Omega
                                  : DmMitMin::CascadeMode::Angle;

            const float torque = m->UpdateCascadeTorque(
                mode,
                e.cmd.target_angle,
                e.cmd.target_omega,
                e.cmd.omega_ff,
                measured_angle,
                measured_omega,
                nullptr);

            m->SetTarget(0.0f, 0.0f, torque);
            m->PublishMitTx(0.0f, 0.0f);
        }

        orb::DmMitTargetCmd t{};
        while (target_sub.copy(t)) {
            DmMitMin* m = s_registry.FindByBusRx(t.bus, static_cast<uint16_t>(t.can_rx_id));
            if (!m) {
                continue;
            }
            m->SetTarget(t.angle, t.omega, t.torque);
            m->PublishMitTx(t.kp, t.kd);
        }

        orb::DmMitAdminCmd c{};
        while (admin_sub.copy(c)) {
            DmMitMin* m = s_registry.FindByBusRx(c.bus, static_cast<uint16_t>(c.can_rx_id));
            if (!m) {
                continue;
            }
            switch (c.op) {
            case orb::DmMitAdminOp::Enter: m->Enter(); break;
            case orb::DmMitAdminOp::Exit: m->Exit(); break;
            case orb::DmMitAdminOp::ClearError: m->ClearError(); break;
            case orb::DmMitAdminOp::SaveZero: m->SaveZero(); break;
            default: break;
            }
        }

        osDelay(1);
    }
}
} // namespace

void DmMitMin::Init(BspCanHandle can, const Config& cfg) {
    can_ = can;
    cfg_ = cfg;
    if (cfg_.base_std_id == 0) {
        // 默认 std_id 基址：以 master_id 左移 4bit 形成高位（低 4bit 放 can_rx_id）。
        cfg_.base_std_id = static_cast<uint16_t>(cfg_.master_id) << 4;
    }
    joined_runtime_ = false;
}

void DmMitMin::JoinRuntime()
{
    configASSERT(can_ != nullptr);
    configASSERT(joined_runtime_ == false);
    if (joined_runtime_) {
        return;
    }

    // 注册或替换：key = (bus + base_std_id + can_rx_id)
    const MotorKey key{cfg_.bus, cfg_.base_std_id, static_cast<uint16_t>(cfg_.can_rx_id)};
    const bool stored = s_registry.RegisterOrReplace(key, this);
    configASSERT(stored);

    if (!s_task_thread) {
        static const osThreadAttr_t attr = {
            .name = "dm_mit",
            .cb_mem = &s_task_tcb,
            .cb_size = sizeof(s_task_tcb),
            .stack_mem = s_task_stack,
            .stack_size = sizeof(s_task_stack),
            .priority = (osPriority_t)osPriorityAboveNormal,
        };
        s_task_thread = osThreadNew(dm_mit_task, nullptr, &attr);
        configASSERT(s_task_thread != nullptr);
    }
    joined_runtime_ = true;
}

void DmMitMin::CanRxCpltCallback(const BspCanFrame* frame) {
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }

    const uint8_t* data = frame->data;

    // DM 普通反馈帧（8 bytes）：
    // byte0: [id(4) | status(4)]
    // byte1-2: angle (16)
    // byte3-4: omega (12)
    // byte4-5: torque (12)
    // byte6: mos temp
    // byte7: rotor temp
    const uint8_t id4 = (data[0] & 0x0F);
    if (id4 != (cfg_.can_rx_id & 0x0F)) {
        return;
    }

    const uint16_t angle_u16 = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    const uint16_t omega_u12 = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    const uint16_t torque_u12 = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];

    const uint8_t mos_temp = data[6];
    const uint8_t rotor_temp = data[7];

    const float raw_angle = uint_to_float(angle_u16, -cfg_.angle_max, cfg_.angle_max, 16);
    now_angle_ = raw_angle;
    now_omega_ = uint_to_float(omega_u12, -cfg_.omega_max, cfg_.omega_max, 12);
    now_torque_ = uint_to_float(torque_u12, -cfg_.torque_max, cfg_.torque_max, 12);

    // 角度展开得到累计角：基于环绕范围 [-angle_max, angle_max]。
    // delta 采用最短跨越（当 raw_angle 翻转时修正）。
    if (!raw_angle_inited_) {
        raw_angle_inited_ = true;
        last_raw_angle_ = raw_angle;
        now_total_angle_ = raw_angle;
    } else {
        float delta = raw_angle - last_raw_angle_;
        const float wrap = 2.0f * cfg_.angle_max;
        if (delta > cfg_.angle_max) {
            delta -= wrap;
        } else if (delta < -cfg_.angle_max) {
            delta += wrap;
        }
        now_total_angle_ += delta;
        last_raw_angle_ = raw_angle;
    }

    // 发布电机反馈（每电机单独 Topic，避免多写者冲突）
    orb::DmMitFeedback fb{};
    fb.bus = cfg_.bus;
    fb.can_rx_id = (cfg_.can_rx_id & 0x0F);
    fb.angle_rad = now_angle_;
    fb.total_angle_rad = now_total_angle_;
    fb.omega_rad_s = now_omega_;
    fb.torque_nm = now_torque_;
    fb.mos_temp = mos_temp;
    fb.rotor_temp = rotor_temp;

    const int idx = orb::dm_mit_feedback_index(fb.bus, fb.can_rx_id);
    orb::dm_mit_feedback[idx].publish(fb);
}

void DmMitMin::SetControl(float angle, float omega, float torque) {
    SetTarget(angle, omega, torque);
}

void DmMitMin::SetTarget(float angle_rad, float omega_rad_s, float torque_nm) {
    ctrl_angle_ = float_constrain(angle_rad, -cfg_.angle_max, cfg_.angle_max);
    ctrl_omega_ = float_constrain(omega_rad_s, -cfg_.omega_max, cfg_.omega_max);
    ctrl_torque_ = float_constrain(torque_nm, -cfg_.torque_max, cfg_.torque_max);
}

void DmMitMin::PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                       float pmax, float vmax, float kpmax, float kdmax, float tmax) {
    // MIT 协议打包：
    // p(16) v(12) kp(12) kd(12) t(12) 按官方格式拼到 8 字节。
    std::memset(out, 0, 8);

    const uint16_t p_u16 = static_cast<uint16_t>(float_to_uint(p, -pmax, pmax, 16));
    const uint16_t v_u12 = static_cast<uint16_t>(float_to_uint(v, -vmax, vmax, 12));
    const uint16_t kp_u12 = static_cast<uint16_t>(float_to_uint(kp, 0, kpmax, 12));
    const uint16_t kd_u12 = static_cast<uint16_t>(float_to_uint(kd, 0, kdmax, 12));
    const uint16_t t_u12 = static_cast<uint16_t>(float_to_uint(t, -tmax, tmax, 12));

    out[0] = static_cast<uint8_t>((p_u16 >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(p_u16 & 0xFF);
    out[2] = static_cast<uint8_t>((v_u12 >> 4) & 0xFF);
    out[3] = static_cast<uint8_t>(((v_u12 & 0x0F) << 4) | ((kp_u12 >> 8) & 0x0F));
    out[4] = static_cast<uint8_t>(kp_u12 & 0xFF);
    out[5] = static_cast<uint8_t>((kd_u12 >> 4) & 0xFF);
    out[6] = static_cast<uint8_t>(((kd_u12 & 0x0F) << 4) | ((t_u12 >> 8) & 0x0F));
    out[7] = static_cast<uint8_t>(t_u12 & 0xFF);
}

void DmMitMin::PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len) {
    if (!can_) {
        return;
    }

    orb::CanTxFrame f{};
    f.bus = cfg_.bus;
    f.id = std_id;
    f.id_type = orb::CanIdType::Std;
    f.frame_type = orb::CanFrameType::Data;
    f.is_fd = false;
    f.brs = false;
    f.len = len;
    std::memset(f.data, 0, sizeof(f.data));
    if (len > 0) {
        const uint8_t n = (len <= 8) ? len : 8;
        std::memcpy(f.data, data, n);
    }
    orb::can_tx.publish(f);
}

void DmMitMin::PublishMitTx(float kp, float kd) {
    // kp/kd limit use typical DM ranges
    uint8_t data[8];
    PackMit(ctrl_angle_, ctrl_omega_, kp, kd, ctrl_torque_, data,
            cfg_.angle_max, cfg_.omega_max, 500.0f, 5.0f, cfg_.torque_max);

    const uint16_t std_id = static_cast<uint16_t>(cfg_.base_std_id) | static_cast<uint16_t>(cfg_.can_rx_id & 0x0F);
    PublishFrame(std_id, data, 8);
}

void DmMitMin::ConfigureCascadePid(const alg::PidConfig& angle_cfg,
                                   const alg::PidConfig& omega_cfg,
                                   float angle_to_omega_sign,
                                   float omega_feedback_sign)
{
    pid_angle_.configure(angle_cfg);
    pid_omega_.configure(omega_cfg);
    angle_to_omega_sign_ = angle_to_omega_sign;
    omega_feedback_sign_ = omega_feedback_sign;
    cascade_pid_inited_ = true;
}

void DmMitMin::ConfigureGimbalCascadePidDefaults(CascadeAxis axis, CascadeFeedback fb)
{
    // 参数来自历史 app_gimbal.cpp：
    // - yaw/pitch 角度环（IMU/ENCODER 各一套）
    // - yaw/pitch 速度环（各一套）
    // 同时固化符号约定：angle_to_omega_sign=-1；yaw 速度反馈取反。

    alg::PidConfig angle_cfg{};
    alg::PidConfig omega_cfg{};

    cascade_axis_ = axis;

    if (axis == CascadeAxis::Yaw) {
        if (fb == CascadeFeedback::Encoder) {
            angle_cfg = make_pid_cfg(
                100.0f, 5.0f, 15.0f, 0.0f,
                0.0f, 44.0f,
                0.001f,
                0.0f,
                0.0f, 0.0f,
                0.0f,
                alg::DFirst::Disable,
                0.01f);
        } else {
            angle_cfg = make_pid_cfg(
                1.8f, 0.1f, 0.4f, 1.0f,
                44.0f, 44.0f,
                0.001f,
                0.0f,
                0.0f, 0.0f,
                0.0f,
                alg::DFirst::Disable,
                0.01f);
        }

        omega_cfg = make_pid_cfg(
            0.04f, 0.008f, 0.00015f, 0.1f,
            3.0f, 9.9f,
            0.001f,
            0.0f,
            0.0f, 0.0f,
            0.0f,
            alg::DFirst::Disable,
            0.01f);

        ConfigureCascadePid(angle_cfg, omega_cfg, -1.0f, -1.0f);
        return;
    }

    // Pitch
    if (fb == CascadeFeedback::Encoder) {
        angle_cfg = make_pid_cfg(
            350.0f, 20.0f, 20.0f, 0.0f,
            0.0f, 44.0f,
            0.001f,
            0.0f,
            0.0f, 0.0f,
            0.0f,
            alg::DFirst::Disable,
            0.02f);
    } else {
        angle_cfg = make_pid_cfg(
            4.0f, 1.50f, 0.25f, 0.0f,
            44.0f, 44.0f,
            0.001f,
            0.0f,
            0.0f, 0.0f,
            0.0f,
            alg::DFirst::Disable,
            0.02f);
    }

    omega_cfg = make_pid_cfg(
        0.08f, 0.008f, 0.0000f, 1.0f,
        3.0f, 9.9f,
        0.001f,
        0.0f,
        0.0f, 0.0f,
        0.0f,
        alg::DFirst::Disable,
        0.01f);

    ConfigureCascadePid(angle_cfg, omega_cfg, -1.0f, 1.0f);
}

float DmMitMin::UpdateAngleToOmega(float target_angle_rad, float measured_angle_rad)
{
    configASSERT(cascade_pid_inited_);
    if (!cascade_pid_inited_) {
        return 0.0f;
    }
    const float omega_sp = angle_to_omega_sign_ * pid_angle_.update(target_angle_rad, measured_angle_rad);
    return omega_sp;
}

float DmMitMin::UpdateOmegaToTorque(float target_omega_rad_s, float measured_omega_rad_s)
{
    configASSERT(cascade_pid_inited_);
    if (!cascade_pid_inited_) {
        return 0.0f;
    }

    // 保持与历史业务层符号约定一致：允许对反馈角速度整体取反。
    float torque = pid_omega_.update(target_omega_rad_s, omega_feedback_sign_ * measured_omega_rad_s);

    // 额外防御：扭矩命令不超过电机允许范围。
    torque = float_constrain(torque, -cfg_.torque_max, cfg_.torque_max);
    last_torque_cmd_ = torque;
    return torque;
}

float DmMitMin::UpdateCascadeTorqueAngleMode(float target_angle_rad,
                                             float measured_angle_rad,
                                             float measured_omega_rad_s)
{
    const float omega_sp = UpdateAngleToOmega(target_angle_rad, measured_angle_rad);
    return UpdateOmegaToTorque(omega_sp, measured_omega_rad_s);
}

float DmMitMin::UpdateCascadeTorque(CascadeMode mode,
                                    float target_angle_rad,
                                    float target_omega_rad_s,
                                    float omega_ff_rad_s,
                                    float measured_angle_rad,
                                    float measured_omega_rad_s,
                                    float* out_omega_sp_rad_s)
{
    float omega_sp = 0.0f;
    if (mode == CascadeMode::Angle) {
        omega_sp = UpdateAngleToOmega(target_angle_rad, measured_angle_rad);
    } else {
        omega_sp = target_omega_rad_s + omega_ff_rad_s;
    }

    if (out_omega_sp_rad_s) {
        *out_omega_sp_rad_s = omega_sp;
    }

    return UpdateOmegaToTorque(omega_sp, measured_omega_rad_s);
}

void DmMitMin::PublishAdminTail(uint8_t tail)
{
    // 管理帧：前 7 字节固定 0xFF，最后 1 字节是操作码 tail。
    uint8_t data[8];
    for (int i = 0; i < 7; ++i) {
        data[i] = 0xFF;
    }
    data[7] = tail;
    PublishFrame(static_cast<uint16_t>(cfg_.base_std_id) | (cfg_.can_rx_id & 0x0F), data, 8);
}

void DmMitMin::Enter() {
    PublishAdminTail(0xFC);
}
void DmMitMin::Exit() {
    PublishAdminTail(0xFD);
}
void DmMitMin::ClearError() {
    PublishAdminTail(0xFB);
}
void DmMitMin::SaveZero() {
    PublishAdminTail(0xFE);
}

void DmMitMin::BringUpDefault()
{
    static constexpr float kDelaySaveZeroS = 1.0f;
    static constexpr float kDelayClearErrorS = 1.0f;
    static constexpr float kDelayEnterS = 1.0f;

    SaveZero();
    dwt_delay(kDelaySaveZeroS);
    ClearError();
    dwt_delay(kDelayClearErrorS);
    Enter();
    dwt_delay(kDelayEnterS);
}

} // namespace actuator::drivers

namespace actuator::instances {

actuator::drivers::DmMitMin dm_01{};
actuator::drivers::DmMitMin dm_02{};

} // namespace actuator::instances
