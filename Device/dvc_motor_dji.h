/**
 * @file dvc_motor_dji.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef MODULES_MOTOR_DJI_H_
#define MODULES_MOTOR_DJI_H_

#include "alg_pid.h"
#include "bsp_can.h"

/**
 * @brief 大疆状态
 *
 */
enum MotorDjiStatus
{
    MOTOR_DJI_STATUS_DISABLE = 0,
    MOTOR_DJI_STATUS_ENABLE,
};

/**
 * @brief 大疆电机控制方式
 *
 */
enum MotorDjiControlMethod
{
    MOTOR_DJI_CONTROL_METHOD_VOLTAGE = 0,
    MOTOR_DJI_CONTROL_METHOD_CURRENT,
    MOTOR_DJI_CONTROL_METHOD_TORQUE,
    MOTOR_DJI_CONTROL_METHOD_OMEGA,
    MOTOR_DJI_CONTROL_METHOD_ANGLE,
};

/**
 * @brief 大疆电机驱动版本, 影响GM6020电机驱动方式
 *
 */
enum MotorDjiGm6020DriverVersion
{
    MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT = 0,
    MOTOR_DJI_GM6020_DRIVER_VERSION_2023,
};

/**
 * @brief 大疆电机的ID枚举类型
 *
 */
enum MotorDjiId
{
    MOTOR_DJI_ID_0x201 = 1,
    MOTOR_DJI_ID_0x202,
    MOTOR_DJI_ID_0x203,
    MOTOR_DJI_ID_0x204,
    MOTOR_DJI_ID_0x205,
    MOTOR_DJI_ID_0x206,
    MOTOR_DJI_ID_0x207,
    MOTOR_DJI_ID_0x208,
    MOTOR_DJI_ID_0x209,
    MOTOR_DJI_ID_0x20A,
    MOTOR_DJI_ID_0x20B,
};

/**
 * @brief 大疆电机源数据
 *
 */
struct MotorDjiCanRxData
{
    uint16_t encoder_reverse;
    int16_t omega_reverse;
    int16_t current_reverse;
    uint8_t temperature;
    uint8_t reserved;
} __attribute__((packed));

/**
 * @brief 是否开启功率控制, 此时电机须电流作为输出模式, 不可电压控制
 *
 */
enum MotorDjiPowerLimitStatus
{
    MOTOR_DJI_POWER_LIMIT_STATUS_DISABLE = 0,
    MOTOR_DJI_POWER_LIMIT_STATUS_ENABLE,
};

/**
 * @brief 大疆电机经过处理的数据
 *
 */
struct MotorDjiRxData
{
    float now_angle;
    float now_omega;
    float now_current;
    float now_temperature;
    float now_power;
    uint32_t pre_encoder;
    int32_t total_encoder;
    int32_t total_round;
};

class MotorDjiC620
{
public:
    // PID角度环控制
    Pid pid_angle_;
    // PID角速度环控制
    Pid pid_omega_;

    void Init(
        FDCAN_HandleTypeDef *hcan,
        MotorDjiId can_rx_id,
        MotorDjiControlMethod motor_dji_control_method = MOTOR_DJI_CONTROL_METHOD_OMEGA,
        float gearbox_rate = 268.0f / 17.0f,
        MotorDjiPowerLimitStatus power_limit_status = MOTOR_DJI_POWER_LIMIT_STATUS_DISABLE,
        float current_max = 20.0f);

    inline float GetCurrentMax();

    inline float GetPowerK0();

    inline float GetPowerK1();

    inline float GetPowerK2();

    inline float GetPowerA();

    inline float GetTheoreticalOutputCurrentMax();

    inline MotorDjiStatus GetStatus();

    inline float GetNowAngle();

    inline float GetNowOmega();

    inline float GetNowCurrent();

    inline float GetNowTemperature();

    inline float GetNowPower();

    inline float GetPowerEstimate();

    inline MotorDjiControlMethod GetControlMethod();

    inline float GetTargetAngle();

    inline float GetTargetOmega();

    inline float GetTargetCurrent();

    inline float GetFeedforwardOmega();

    inline float GetFeedforwardCurrent();

    inline void SetControlMethod(MotorDjiControlMethod motor_dji_control_method);

    inline void SetTargetAngle(float target_angle);

    inline void SetTargetOmega(float target_omega);

    inline void SetTargetCurrent(float target_current);

    inline void SetFeedforwardOmega(float feedforward_omega);

    inline void SetFeedforwardCurrent(float feedforward_current);

    inline void SetPowerFactor(float power_factor);

    void CanRxCpltCallback(uint8_t *rx_data);

    void AlivePeriodElapsedCallback();

    void CalculatePeriodElapsedCallback();

    void PowerLimitAfterCalculatePeriodElapsedCallback();

    void Output(); // 暂时开放接口调试用
protected:
    // 初始化相关变量

    // 绑定的CAN
    CanManageObject *can_manage_object_;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    MotorDjiId can_rx_id_;
    // 发送缓存区
    uint8_t *tx_data_;
    // 减速比, 默认带减速箱
    float gearbox_rate_ = 268.0f / 17.0f;
    // 是否开启功率控制
    MotorDjiPowerLimitStatus power_limit_status_;
    // 最大电流
    float current_max_;

    // 常量

    // 功率计算系数
    float power_k_0_ = 0.0171687401695532f;
    float power_k_1_ = 0.00004319062337601348f;
    float power_k_2_ = 0.12785662198126574f;
    float power_A_   = 1.598658371452403f;

    // 一圈编码器刻度
    uint16_t encoder_num_per_round_ = 8192;

    // 电流到输出的转化系数
    float current_to_out_ = 16384.0f / 20.0f;
    // 理论最大输出电流
    float theoretical_output_current_max_ = 20.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t flag_ = 0;
    // 前一时刻的电机接收flag
    uint32_t pre_flag_ = 0;
    // 输出量
    float out_ = 0.0f;

    // 读变量

    // 电机状态
    MotorDjiStatus motor_dji_status_ = MOTOR_DJI_STATUS_DISABLE;
    // 电机对外接口信息
    MotorDjiRxData rx_data_ = {0.f,0.f,0.f,0.f,0.f,0,0,0};
    // 下一时刻的功率估计值, W
    float power_estimate_;

    // 写变量

    // 读写变量

    // 电机控制方式
    MotorDjiControlMethod motor_dji_control_method_ = MOTOR_DJI_CONTROL_METHOD_ANGLE;
    // 目标的角度, rad
    float target_angle_ = 0.0f;
    // 目标的速度, rad/s
    float target_omega_ = 0.0f;
    // 目标的电流
    float target_current_ = 0.0f;
    // 前馈的速度, rad/s
    float feedforward_omega_ = 0.0f;
    // 前馈的电流, A
    float feedforward_current_ = 0.0f;
    // 功率衰减因数
    float power_factor_ = 1.0f;

    // 内部函数

    void DataProcess();

    void PidCalculate();

    void PowerLimitControl();

    // void Output();
};

class MotorDjiC610
{
public:
    // PID角度环控制
    Pid pid_angle_;
    // PID角速度环控制
    Pid pid_omega_;

    void Init(
        FDCAN_HandleTypeDef *hcan,
        MotorDjiId can_rx_id,
        MotorDjiControlMethod motor_dji_control_method = MOTOR_DJI_CONTROL_METHOD_OMEGA,
        float gearbox_rate = 36.0f,
        float current_max = 10.0f);

    inline float GetCurrentMax();

    inline float GetTheoreticalOutputCurrentMax();

    inline MotorDjiStatus GetStatus();

    inline float GetNowAngle();

    inline float GetNowOmega();

    inline float GetNowCurrent();

    inline uint8_t GetNowTemperature();

    inline MotorDjiControlMethod GetControlMethod();

    inline float GetTargetAngle();

    inline float GetTargetOmega();

    inline float GetTargetCurrent();

    inline float GetFeedforwardOmega();

    inline float GetFeedforwardCurrent();

    inline void SetControlMethod(MotorDjiControlMethod motor_dji_control_method);

    inline void SetTargetAngle(float target_angle);

    inline void SetTargetOmega(float target_omega);

    inline void SetTargetCurrent(float target_current);

    inline void SetFeedforwardOmega(float feedforward_omega);

    inline void SetFeedforwardCurrent(float feedforward_current);

    void CanRxCpltCallback(uint8_t *rx_data);

    void CalculatePeriodElapsedCallback();

    void Output(); // 暂时开放接口用于调试
protected:
    // 初始化相关常量

    // 绑定的CAN
    CanManageObject *can_manage_object_;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    MotorDjiId can_rx_id_;
    // 发送缓存区
    uint8_t *tx_data_;
    // 减速比, 默认带减速箱
    float gearbox_rate_;
    // 最大电流
    float current_max_;

    // 常量

    // 一圈编码器刻度
    uint16_t encoder_num_per_round_ = 8192;

    // 电流到输出的转化系数
    float current_to_out_ = 10000.0f / 10.0f;
    // 理论最大输出电流
    float theoretical_output_current_max_ = 10.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t flag_ = 0;
    // 前一时刻的电机接收flag
    uint32_t pre_flag_ = 0;
    // 输出量
    float out_ = 0.0f;

    // 读变量

    // 电机状态
    MotorDjiStatus motor_dji_status_ = MOTOR_DJI_STATUS_DISABLE;
    // 电机对外接口信息
    MotorDjiRxData rx_data_;

    // 写变量

    // 读写变量

    // 电机控制方式
    MotorDjiControlMethod motor_dji_control_method_ = MOTOR_DJI_CONTROL_METHOD_ANGLE;
    // 目标的角度, rad
    float target_angle_ = 0.0f;
    // 目标的速度, rad/s
    float target_omega_ = 0.0f;
    // 目标的电流, A
    float target_current_ = 0.0f;
    // 前馈的速度, rad/s
    float feedforward_omega_ = 0.0f;
    // 前馈的电流, A
    float feedforward_current_ = 0.0f;

    // 内部函数

    void DataProcess();

    void PidCalculate();

    // void Output();
};

/**
 * @brief 获取最大电流
 *
 * @return float 最大电流
 */
inline float MotorDjiC620::GetCurrentMax()
{
    return (current_max_);
}

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
inline float MotorDjiC620::GetPowerK0()
{
    return (power_k_0_);
}

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
inline float MotorDjiC620::GetPowerK1()
{
    return (power_k_1_);
}

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
inline float MotorDjiC620::GetPowerK2()
{
    return (power_k_2_);
}

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
inline float MotorDjiC620::GetPowerA()
{
    return (power_A_);
}

/**
 * @brief 获取理论最大输出电流
 *
 * @return float 理论最大输出电流
 */
inline float MotorDjiC620::GetTheoreticalOutputCurrentMax()
{
    return (theoretical_output_current_max_);
}

/**
 * @brief 获取电机状态
 *
 * @return MotorDjiStatus 电机状态
 */
inline MotorDjiStatus MotorDjiC620::GetStatus()
{
    return (motor_dji_status_);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float MotorDjiC620::GetNowAngle()
{
    return (rx_data_.now_angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float MotorDjiC620::GetNowOmega()
{
    return (rx_data_.now_omega);
}

/**
 * @brief 获取当前的电流, A
 *
 * @return 当前的电流, A
 */
inline float MotorDjiC620::GetNowCurrent()
{
    return (rx_data_.now_current);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return float 当前的温度, K
 */
inline float MotorDjiC620::GetNowTemperature()
{
    return (rx_data_.now_temperature);
}

/**
 * @brief 获取当前的功率, W
 *
 * @return float 当前的功率, W
 */
inline float MotorDjiC620::GetNowPower()
{
    return (rx_data_.now_power);
}

/**
 * @brief 获取下一时刻的功率估计值, W
 *
 * @return float 下一时刻的功率估计值, W
 */
inline float MotorDjiC620::GetPowerEstimate()
{
    return (power_estimate_);
}

/**
 * @brief 获取电机控制方式
 *
 * @return MotorDjiControlMethod 电机控制方式
 */
inline MotorDjiControlMethod MotorDjiC620::GetControlMethod()
{
    return (motor_dji_control_method_);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float MotorDjiC620::GetTargetAngle()
{
    return (target_angle_);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float MotorDjiC620::GetTargetOmega()
{
    return (target_omega_);
}

/**
 * @brief 获取目标的电流
 *
 * @return float 目标的电流
 */
inline float MotorDjiC620::GetTargetCurrent()
{
    return (target_current_);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float MotorDjiC620::GetFeedforwardOmega()
{
    return (feedforward_omega_);
}

/**
 * @brief 获取前馈的电流, A
 *
 * @return float 前馈的电流, A
 */
inline float MotorDjiC620::GetFeedforwardCurrent()
{
    return (feedforward_current_);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __motor_dji_control_method_ 电机控制方式
 */
inline void MotorDjiC620::SetControlMethod(MotorDjiControlMethod motor_dji_control_method)
{
    motor_dji_control_method_ = motor_dji_control_method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void MotorDjiC620::SetTargetAngle(float target_angle)
{
    target_angle_ = target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void MotorDjiC620::SetTargetOmega(float target_omega)
{
    target_omega_ = target_omega;
}

/**
 * @brief 设定目标的电流
 *
 * @param __Target_Current 目标的电流
 */
inline void MotorDjiC620::SetTargetCurrent(float target_current)
{
    target_current_ = target_current;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void MotorDjiC620::SetFeedforwardOmega(float feedforward_omega)
{
    feedforward_omega_ = feedforward_omega;
}

/**
 * @brief 设定前馈的电流, A
 *
 * @param __Feedforward_Current 前馈的电流, A
 */
inline void MotorDjiC620::SetFeedforwardCurrent(float feedforward_current)
{
    feedforward_current_ = feedforward_current;
}

/**
 * @brief 设定功率衰减因数
 *
 * @param __Power_Factor 功率衰减因数
 */
inline void MotorDjiC620::SetPowerFactor(float power_factor)
{
    power_factor_ = power_factor;
}
/**
 * @brief 获取最大电流
 *
 * @return float 最大电流
 */
inline float MotorDjiC610::GetCurrentMax()
{
    return (current_max_);
}

/**
 * @brief 获取理论最大输出电流
 *
 * @return float 理论最大输出电流
 */
inline float MotorDjiC610::GetTheoreticalOutputCurrentMax()
{
    return (theoretical_output_current_max_);
}

/**
 * @brief 获取电机状态
 *
 * @return MotorDjiStatus 电机状态
 */
inline MotorDjiStatus MotorDjiC610::GetStatus()
{
    return (motor_dji_status_);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float MotorDjiC610::GetNowAngle()
{
    return (rx_data_.now_angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float MotorDjiC610::GetNowOmega()
{
    return (rx_data_.now_omega);
}

/**
 * @brief 获取当前的电流, A
 *
 * @return 当前的电流, A
 */
inline float MotorDjiC610::GetNowCurrent()
{
    return (rx_data_.now_current);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return uint8_t 当前的温度, K
 */
inline uint8_t MotorDjiC610::GetNowTemperature()
{
    return (rx_data_.now_temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return MotorDjiControlMethod 电机控制方式
 */
inline MotorDjiControlMethod MotorDjiC610::GetControlMethod()
{
    return (motor_dji_control_method_);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float MotorDjiC610::GetTargetAngle()
{
    return (target_angle_);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float MotorDjiC610::GetTargetOmega()
{
    return (target_omega_);
}

/**
 * @brief 获取目标的电流
 *
 * @return float 目标的电流
 */
inline float MotorDjiC610::GetTargetCurrent()
{
    return (target_current_);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float MotorDjiC610::GetFeedforwardOmega()
{
    return (feedforward_omega_);
}

/**
 * @brief 获取前馈的电流, A
 *
 * @return float 前馈的电流, A
 */
inline float MotorDjiC610::GetFeedforwardCurrent()
{
    return (feedforward_current_);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __motor_dji_control_method_ 电机控制方式
 */
inline void MotorDjiC610::SetControlMethod(MotorDjiControlMethod motor_dji_control_method)
{
    motor_dji_control_method_ = motor_dji_control_method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void MotorDjiC610::SetTargetAngle(float target_angle)
{
    target_angle_ = target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void MotorDjiC610::SetTargetOmega(float target_omega)
{
    target_omega_ = target_omega;
}

/**
 * @brief 设定目标的电流, A
 *
 * @param __Target_Current 目标的电流, A
 */
inline void MotorDjiC610::SetTargetCurrent(float target_current)
{
    target_current_ = target_current;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void MotorDjiC610::SetFeedforwardOmega(float feedforward_omega)
{
    feedforward_omega_ = feedforward_omega;
}

/**
 * @brief 设定前馈的电流, A
 *
 * @param __Feedforward_Current 前馈的电流, A
 */
inline void MotorDjiC610::SetFeedforwardCurrent(float feedforward_current)
{
    feedforward_current_ = feedforward_current;
}

#endif //MOTOR_DJI_H
