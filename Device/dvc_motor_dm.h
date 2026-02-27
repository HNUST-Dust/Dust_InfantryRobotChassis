/**
 * @file dvc_motor_dm.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef MODULES_MOTOR_DM_H_
#define MODULES_MOTOR_DM_H_
/* Includes ------------------------------------------------------------------*/

#include "control/alg_pid.h"
#include "bsp_can.h"
#include "math/alg_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 达妙电机状态
 *
 */
enum MotorDmStatus
{
    MOTOR_DM_STATUS_DISABLE = 0,
    MOTOR_DM_STATUS_ENABLE,
};

/**
 * @brief 达妙电机的ID枚举类型, 一拖四模式用
 *
 */
enum MotorDmMotorId1To4 : uint8_t
{
    MOTOR_DM_ID_0x301 = 1,
    MOTOR_DM_ID_0x302,
    MOTOR_DM_ID_0x303,
    MOTOR_DM_ID_0x304,
    MOTOR_DM_ID_0x305,
    MOTOR_DM_ID_0x306,
    MOTOR_DM_ID_0x307,
    MOTOR_DM_ID_0x308,
};

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
enum MotorDmControlStatusNormal
{
    MOTOR_DM_CONTROL_STATUS_DISABLE = 0x0,
    MOTOR_DM_CONTROL_STATUS_ENABLE,
    MOTOR_DM_CONTROL_STATUS_OVERVOLTAGE = 0x8,
    MOTOR_DM_CONTROL_STATUS_UNDERVOLTAGE,
    MOTOR_DM_CONTROL_STATUS_OVERCURRENT,
    MOTOR_DM_CONTROL_STATUS_MOS_OVERTEMPERATURE,
    MOTOR_DM_CONTROL_STATUS_ROTOR_OVERTEMPERATURE,
    MOTOR_DM_CONTROL_STATUS_LOSE_CONNECTION,
    MOTOR_DM_CONTROL_STATUS_MOS_OVERLOAD,
};

/**
 * @brief 达妙电机控制方式
 *
 */
enum MotorDmControlMethod
{
    MOTOR_DM_CONTROL_METHOD_NORMAL_MIT = 0,
    MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA,
    MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA,
    MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT,
    MOTOR_DM_CONTROL_METHOD_1_TO_4_CURRENT,
    MOTOR_DM_CONTROL_METHOD_1_TO_4_OMEGA,
    MOTOR_DM_CONTROL_METHOD_1_TO_4_ANGLE,
};

/**
 * @brief 达妙电机传统模式源数据
 *
 */
struct MotorDmCanRxDataNormal
{
    uint8_t can_id : 4;
    uint8_t control_status_enum : 4;
    uint16_t angle_reverse;
    uint8_t omega_11_4;
    uint8_t omega_3_0_torque_11_8;
    uint8_t torque_7_0;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机一拖四模式源数据
 *
 */
struct MotorDmCanRxData1To4
{
    uint16_t encoder_reverse;
    // 角速度100倍
    int16_t omega_reverse;
    // 电流值, mA
    int16_t current_reverse;
    uint8_t rotor_temperature;
    uint8_t mos_temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, MIT控制报文
 *
 */
struct MotorDmCanTxDataNormalMit
{
    uint16_t control_angle_reverse;
    uint8_t control_omega_11_4;
    uint8_t control_omega_3_0_k_p_11_8;
    uint8_t k_p_7_0;
    uint8_t k_d_11_4;
    uint8_t k_d_3_0_control_torque_11_8;
    uint8_t control_torque_7_0;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 位置速度控制报文
 *
 */
struct MotorDmCanTxDataNormalAngleOmega
{
    float control_angle_;
    float control_omega_;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 速度控制报文
 *
 */
struct MotorDmCanTxDataNormalOmega
{
    float control_omega_;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, EMIT控制报文
 *
 */
struct MotorDmCanTxDataNormalEmit
{
    float control_angle;
    // 限定速度用, rad/s的100倍
    uint16_t control_omega;
    // 限定电流用, 电流最大值的10000倍
    uint16_t control_current;
} __attribute__((packed));

/**
 * @brief 达妙电机经过处理的数据, 传统模式有效
 *
 */
struct MotorDmRxDataNormal
{
    enum MotorDmControlStatusNormal control_status;
    float now_angle;
    float now_omega;
    float now_torque;
    float now_mos_temperature;
    float now_rotor_temperature;
    uint32_t pre_encoder;
    int32_t total_encoder;
    int32_t total_round;
    float now_angle_noncumulative;
    float start_angle;
};

/**
 * @brief 达妙电机经过处理的数据, 一拖四模式有效
 *
 */
struct MotorDmRxData1to4
{
    float now_angle;
    float now_omega;
    float now_current;
    float now_mos_temperature;
    float now_rotor_temperature;
    uint32_t pre_encoder;
    int32_t total_encoder;
    int32_t total_round;
};

/**
 * @brief Reusable, 达妙电机, 传统模式
 * 没有零点, 可在上位机调零点
 * 初始化的角度, 角速度, 扭矩, 电流等参数是J4310电机默认值
 *
 */
class MotorDmNormal
{
public:
  void Init(FDCAN_HandleTypeDef *hcan,
            uint8_t can_rx_id,
            uint8_t can_tx_id,
            MotorDmControlMethod motor_dm_control_method =
                MOTOR_DM_CONTROL_METHOD_NORMAL_MIT,
            float angle_max = 12.5f,
            float omega_max = 45.0f,
            float torque_max = 10.0f,
            float current_max = 10.261194f
            );

    inline float GetAngleMax();

    inline float GetOmegaMax();

    inline float GetTorqueMax();

    inline float GetCurrentMax();

    inline enum MotorDmStatus GetStatus();

    inline enum MotorDmControlStatusNormal GetControlStatus();

    inline float GetNowAngle();

    inline float GetNowAngleNoncumulative();

    inline float GetNowOmega();

    inline float GetNowTorque();

    inline float GetNowMosTemperature();

    inline float GetNowRotorTemperature();

    inline enum MotorDmControlMethod GetControlMethod();

    inline float GetControlAngle();

    inline float GetControlOmega();

    inline float GetControlTorque();

    inline float GetControlCurrent();

    inline float GetKp();

    inline float GetKd();

    inline void SetControlAngle(float control_angle);

    inline void SetControlOmega(float control_omega);

    inline void SetControlTorque(float control_torque);

    inline void SetControlCurrent(float control_current);

    inline void SetKp(float k_p);

    inline void SetKd(float k_d);

    void CanRxCpltCallback(uint8_t *rx_data_);

    void CanSendClearError();

    void CanSendEnter();

    void CanSendExit();

    void CanSendSaveZero();

    void AlivePeriodElapsedCallback();

    void SendPeriodElapsedCallback();

    // 单元测试临时添加输出函数
    void Output();
protected:
    // 初始化相关变量

    // 绑定的CAN
    CanManageObject *can_manage_object_;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t can_rx_id_;
    // 发数据绑定的CAN ID, 是上位机驱动参数can_id加上控制模式的偏移量
    uint16_t can_tx_id_;
    // 最大位置, 与上位机控制幅值PMAX保持一致
    float angle_max_;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float omega_max_;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float torque_max_;
    // 最大电流, 与上位机串口中上电打印电流保持一致
    float current_max_;

    // 常量

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t flag_ = 0;
    // 前一时刻的电机接收flag
    uint32_t pre_flag_ = 0;

    // 发送缓冲区
    uint8_t tx_data_[8];

    // 读变量

    // 电机状态
    MotorDmStatus motor_dm_status_ = MOTOR_DM_STATUS_DISABLE;
    // 电机对外接口信息
    MotorDmRxDataNormal rx_data_;

    // 写变量

    // 读写变量

    // 电机控制方式
    MotorDmControlMethod motor_dm_control_method_ = MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

    // 角度, rad, 目标角度
    float control_angle_ = 0.0f;
    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
    float control_omega_ = 0.0f;
    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
    float control_torque_ = 0.0f;
    // 电流, A, EMIT模式是限幅, 其余模式无效
    float control_current_ = 0.0f;
    // k_p_, 0~500, MIT模式有效
    float k_p_ = 0.0f;
    // k_d_, 0~5, MIT模式有效
    float k_d_ = 0.0f;

    // 内部函数

    void DataProcess();


};

/**
 * @brief Reusable, 达妙电机, 一拖四模式
 * 没有零点, 可在上位机调零点
 *
 */
class MotorDm1To4
{
public:

    // PID角度环控制
    alg::Pid pid_angle_;
    // PID角速度环控制
    alg::Pid pid_omega_;

    void Init(FDCAN_HandleTypeDef *hcan,
              MotorDmMotorId1To4 can_rx_id,
              MotorDmControlMethod motor_dm_control_method =
                  MOTOR_DM_CONTROL_METHOD_1_TO_4_ANGLE,
              int32_t encoder_offset = 0, 
              float current_max = 10.261194f);

    inline float GetCurrentMax();

    inline float Get_TheoreticalOutputCurrentMax();

    inline MotorDmStatus GetStatus();

    inline float GetNowAngle();

    inline float GetNowOmega();

    inline float GetNowCurrent();

    inline float GetNowMosTemperature();

    inline float GetNowRotorTemperature();

    inline MotorDmControlMethod GetControlMethod();

    inline float GetTargetAngle();

    inline float GetTargetOmega();

    inline float GetTargetCurrent();

    inline float GetFeedforwardOmega();

    inline float GetFeedforwardCurrent();

    inline void SetControlMethod(MotorDmControlMethod dm_motor_control_method);

    inline void SetTargetAngle(float target_angle);

    inline void SetTargetOmega(float target_omega);

    inline void SetTargetCurrent(float target_current);

    inline void SetFeedforwardOmega(float feedforward_omega);

    inline void SetFeedforwardCurrent(float feedforward_current);

    void CanRxCpltCallback(uint8_t *rx_data_);

    void AlivePeriodElapsedCallback();

    void CalculatePeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    CanManageObject *can_manage_object_;
    // 收数据绑定的CAN ID, 达妙系列0x301~0x308
    MotorDmMotorId1To4 can_rx_id_;
    // 编码器偏移
    int32_t encoder_offset_;
    // 发送缓存区
    uint8_t *tx_data_;
    // 最大电流
    float current_max_;

    // 常量

    // 一圈编码器刻度
    uint16_t encoder_num_per_round_ = 8192;

    // 电流到输出的转化系数
    float current_to_out_ = 16384.0f / 10.261194f;
    // 理论最大输出电流
    float theoretical_output_current_max_ = 10.261194f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t flag_ = 0;
    // 前一时刻的电机接收flag
    uint32_t pre_flag_ = 0;
    // 输出量
    float out_ = 0.0f;

    // 读变量

    // 电机状态
    MotorDmStatus motor_dm_status_ = MOTOR_DM_STATUS_DISABLE;
    // 电机对外接口信息
    MotorDmRxData1to4 rx_data_;

    // 写变量

    // 读写变量
    
    // 电机控制方式
    MotorDmControlMethod motor_dm_control_method_ = MOTOR_DM_CONTROL_METHOD_1_TO_4_ANGLE;
    // 目标的角度
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

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取角度最大值
 *
 * @return float 角度最大值
 */
inline float MotorDmNormal::GetAngleMax()
{
    return (angle_max_);
}

/**
 * @brief 获取角速度最大值
 *
 * @return float 角速度最大值
 */
inline float MotorDmNormal::GetOmegaMax()
{
    return (omega_max_);
}

/**
 * @brief 获取扭矩最大值
 *
 * @return float 扭矩最大值
 */
inline float MotorDmNormal::GetTorqueMax()
{
    return (torque_max_);
}

/**
 * @brief 获取电流最大值
 *
 * @return float 电流最大值
 */
inline float MotorDmNormal::GetCurrentMax()
{
    return (current_max_);
}

/**
 * @brief 获取电机状态
 *
 * @return MotorDmStatus 电机状态
 */
inline MotorDmStatus MotorDmNormal::GetStatus()
{
    return (motor_dm_status_);
}

/**
 * @brief 获取电机控制状态
 *
 * @return MotorDmControlStatusNormal 电机控制状态
 */
inline MotorDmControlStatusNormal MotorDmNormal::GetControlStatus()
{
    return (rx_data_.control_status);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
inline float MotorDmNormal::GetNowAngle()
{
    return (rx_data_.now_angle);
}

inline float MotorDmNormal::GetNowAngleNoncumulative()
{
    return (rx_data_.now_angle_noncumulative);
}
/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float MotorDmNormal::GetNowOmega()
{
    return (rx_data_.now_omega);
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩
 */
inline float MotorDmNormal::GetNowTorque()
{
    return (rx_data_.now_torque);
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
inline float MotorDmNormal::GetNowMosTemperature()
{
    return (rx_data_.now_mos_temperature);
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
inline float MotorDmNormal::GetNowRotorTemperature()
{
    return (rx_data_.now_rotor_temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return MotorDmControlMethod 电机控制方式
 */
inline MotorDmControlMethod MotorDmNormal::GetControlMethod()
{
    return (motor_dm_control_method_);
}

/**
 * @brief 获取角度, rad, 目标角度
 *
 * @return float 角度, rad, 目标角度
 */
inline float MotorDmNormal::GetControlAngle()
{
    return (control_angle_);
}

/**
 * @brief 获取角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @return float 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline float MotorDmNormal::GetControlOmega()
{
    return (control_omega_);
}

/**
 * @brief 获取扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @return float 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline float MotorDmNormal::GetControlTorque()
{
    return (control_torque_);
}

/**
 * @brief 获取电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @return float 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline float MotorDmNormal::GetControlCurrent()
{
    return (control_current_);
}

/**
 * @brief 获取k_p_, 0~500, MIT模式有效
 *
 * @return float k_p_, 0~500, MIT模式有效
 */
inline float MotorDmNormal::GetKp()
{
    return (k_p_);
}

/**
 * @brief 获取k_d_, 0~5, MIT模式有效
 *
 * @return float k_d_, 0~5, MIT模式有效
 */
inline float MotorDmNormal::GetKd()
{
    return (k_d_);
}

/**
 * @brief 设定角度, rad, 目标角度
 *
 * @param control_angle 角度, rad, 目标角度
 */
inline void MotorDmNormal::SetControlAngle(float control_angle)
{
    control_angle_ = control_angle;
}

/**
 * @brief 设定角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @param control_omega 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline void MotorDmNormal::SetControlOmega(float control_omega)
{
    control_omega_ = control_omega;
}

/**
 * @brief 设定扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @param control_torque 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline void MotorDmNormal::SetControlTorque(float control_torque)
{
    control_torque_ = control_torque;
}

/**
 * @brief 设定电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @param control_current 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline void MotorDmNormal::SetControlCurrent(float control_current)
{
    control_current_ = control_current;
}

/**
 * @brief 设定k_p_, 0~500, MIT模式有效
 *
 * @param k_p K_P, 0~500, MIT模式有效
 */
inline void MotorDmNormal::SetKp(float k_p)
{
    k_p_ = k_p;
}

/**
 * @brief 设定k_d_, 0~5, MIT模式有效
 *
 * @param k_d K_D, 0~5, MIT模式有效
 */
inline void MotorDmNormal::SetKd(float k_d)
{
    k_d_ = k_d;
}

/**
 * @brief 获取电流最大值
 *
 * @return float 电流最大值
 */
inline float MotorDm1To4::GetCurrentMax()
{
    return (current_max_);
}

/**
 * @brief 获取理论最大输出电流
 *
 * @return float 理论最大输出电流
 */
inline float MotorDm1To4::Get_TheoreticalOutputCurrentMax()
{
    return (theoretical_output_current_max_);
}

/**
 * @brief 获取电机状态
 *
 * @return MotorDmStatus 电机状态
 */
inline MotorDmStatus MotorDm1To4::GetStatus()
{
    return (motor_dm_status_);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
inline float MotorDm1To4::GetNowAngle()
{
    return (rx_data_.now_angle);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float MotorDm1To4::GetNowOmega()
{
    return (rx_data_.now_omega);
}

/**
 * @brief 获取当前电流
 *
 * @return float 当前电流
 */
inline float MotorDm1To4::GetNowCurrent()
{
    return (rx_data_.now_current);
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
inline float MotorDm1To4::GetNowMosTemperature()
{
    return (rx_data_.now_mos_temperature);
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
inline float MotorDm1To4::GetNowRotorTemperature()
{
    return (rx_data_.now_rotor_temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return MotorDmControlMethod 电机控制方式
 */
inline MotorDmControlMethod MotorDm1To4::GetControlMethod()
{
    return (motor_dm_control_method_);
}

/**
 * @brief 获取目标的角度
 *
 * @return float 目标的角度
 */
inline float MotorDm1To4::GetTargetAngle()
{
    return (target_angle_);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float MotorDm1To4::GetTargetOmega()
{
    return (target_omega_);
}

/**
 * @brief 获取目标的电流, A
 *
 * @return float 目标的电流, A
 */
inline float MotorDm1To4::GetTargetCurrent()
{
    return (target_current_);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float MotorDm1To4::GetFeedforwardOmega()
{
    return (feedforward_omega_);
}

/**
 * @brief 获取前馈的电流, A
 *
 * @return float 前馈的电流, A
 */
inline float MotorDm1To4::GetFeedforwardCurrent()
{
    return (feedforward_current_);
}

/**
 * @brief 设定电机控制方式
 *
 * @param dm_motor_control_method 电机控制方式
 */
inline void MotorDm1To4::SetControlMethod(MotorDmControlMethod dm_motor_control_method)
{
    motor_dm_control_method_ = dm_motor_control_method;
}

/**
 * @brief 设定目标的角度
 *
 * @param target_angle 目标的角度
 */
inline void MotorDm1To4::SetTargetAngle(float target_angle)
{
    target_angle_ = target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param target_omega 目标的速度, rad/s
 */
inline void MotorDm1To4::SetTargetOmega(float target_omega)
{
    target_omega_ = target_omega;
}

/**
 * @brief 设定目标的电流, A
 *
 * @param target_current 目标的电流, A
 */
inline void MotorDm1To4::SetTargetCurrent(float target_current)
{
    target_current_ = target_current;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param feedforward_omega 前馈的速度, rad/s
 */
inline void MotorDm1To4::SetFeedforwardOmega(float feedforward_omega)
{
    feedforward_omega_ = feedforward_omega;
}

/**
 * @brief 设定前馈的电流, A
 *
 * @param feedforward_current 前馈的电流, A
 */
inline void MotorDm1To4::SetFeedforwardCurrent(float feedforward_current)
{
    feedforward_current_ = feedforward_current;
}

#endif

/************************ COPYRIGHT(C) HNUST_DUST **************************/
