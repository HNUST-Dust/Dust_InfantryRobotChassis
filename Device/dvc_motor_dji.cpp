/**
 * @file dvc_motor_dji.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji.h"
#include "arm_math.h"
#include "alg_math.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 估计功率值
 *
 * @param k_0 电机建模系数
 * @param k_1 电机建模系数
 * @param k_2 电机建模系数
 * @param A 电机建模系数
 * @param current 电流
 * @param omega 角速度
 * @return
 */
float power_calculate(float k_0, float k_1, float k_2, float A, float current, float omega)
{
    return (k_0 * current * omega + k_1 * omega * omega + k_2 * current * current + A);
}

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param can_id CAN ID
 * @param dji_motor_driver_version 大疆驱动版本, 当且仅当当前被分配电机为6020, 且是电流驱动新版本时选2023, 否则都是default
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(
    FDCAN_HandleTypeDef *hcan, 
    MotorDjiId can_id, 
    MotorDjiGm6020DriverVersion dji_motor_driver_version = MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hfdcan1)
    {
        switch (can_id)
        {
            case (MOTOR_DJI_ID_0x201):
            {
                tmp_tx_data_ptr = &(g_can1_0x200_tx_data[0]);

                break;
            }
            case (MOTOR_DJI_ID_0x202):
            {
                tmp_tx_data_ptr = &(g_can1_0x200_tx_data[2]);

                break;
            }
            case (MOTOR_DJI_ID_0x203):
            {
                tmp_tx_data_ptr = &(g_can1_0x200_tx_data[4]);

                break;
            }
            case (MOTOR_DJI_ID_0x204):
            {
                tmp_tx_data_ptr = &(g_can1_0x200_tx_data[6]);

                break;
            }
            case (MOTOR_DJI_ID_0x205):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can1_0x1ff_tx_data[0]);
                }
                break;
            }
            case (MOTOR_DJI_ID_0x206):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can1_0x1ff_tx_data[2]);
                }
                break;
            }
            case (MOTOR_DJI_ID_0x207):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can1_0x1ff_tx_data[4]);
                }
                break;
            }
            case (MOTOR_DJI_ID_0x208):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can1_0x1ff_tx_data[6]);
                }
                break;
            }
            case (MOTOR_DJI_ID_0x209):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can1_0x2ff_tx_data[0]);
                }
                break;
            }
            case (MOTOR_DJI_ID_0x20A):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can1_0x2ff_tx_data[2]);
                }
                break;
            }
            case (MOTOR_DJI_ID_0x20B):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can1_0x2ff_tx_data[4]);
                }
                break;
            }
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (can_id)
        {
            case (MOTOR_DJI_ID_0x201):
            {
                tmp_tx_data_ptr = &(g_can2_0x200_tx_data[0]);

                break;
            }
            case (MOTOR_DJI_ID_0x202):
            {
                tmp_tx_data_ptr = &(g_can2_0x200_tx_data[2]);

                break;
            }
            case (MOTOR_DJI_ID_0x203):
            {
                tmp_tx_data_ptr = &(g_can2_0x200_tx_data[4]);

                break;
            }
            case (MOTOR_DJI_ID_0x204):
            {
                tmp_tx_data_ptr = &(g_can2_0x200_tx_data[6]);

                break;
            }
            case (MOTOR_DJI_ID_0x205):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can2_0x1ff_tx_data[0]);
                }

                break;
            }
            case (MOTOR_DJI_ID_0x206):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can2_0x1ff_tx_data[2]);
                }

                break;
            }
            case (MOTOR_DJI_ID_0x207):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can2_0x1ff_tx_data[4]);
                }

                break;
            }
            case (MOTOR_DJI_ID_0x208):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can2_0x1ff_tx_data[6]);
                }

                break;
            }
            case (MOTOR_DJI_ID_0x209):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can2_0x2ff_tx_data[0]);
                }

                break;
            }
            case (MOTOR_DJI_ID_0x20A):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can2_0x2ff_tx_data[2]);
                }

                break;
            }
            case (MOTOR_DJI_ID_0x20B):
            {
                if (dji_motor_driver_version == MOTOR_DJI_GM6020_DRIVER_VERSION_DEFAULT)
                {
                    tmp_tx_data_ptr = &(g_can2_0x2ff_tx_data[4]);
                }

                break;
            }
        }
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param can_rx_id CAN ID
 * @param MOTOR_DJI_CONTROL_METHOD 电机控制方式, 默认角度
 * @param gearbox_rata 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param current_max 最大电流
 */
void MotorDjiC610::Init(
    FDCAN_HandleTypeDef *hcan, 
    MotorDjiId can_rx_id, 
    MotorDjiControlMethod motor_dji_control_method, 
    float gearbox_rata, 
    float current_max)
{
    if (hcan->Instance == FDCAN1)
    {
        can_manage_object_ = &g_can1_manage_object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        can_manage_object_ = &g_can2_manage_object;
    }
    can_rx_id_ = can_rx_id;
    motor_dji_control_method_ = motor_dji_control_method;
    gearbox_rate_ = gearbox_rata;
    current_max_ = current_max;
    tx_data_ = allocate_tx_data(hcan, can_rx_id);
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data_ 接收的数据
 */
void MotorDjiC610::CanRxCpltCallback(uint8_t *rx_data)
{
    // 滑动窗口, 判断电机是否在线
    flag_ += 1;

    DataProcess();
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void MotorDjiC610::CalculatePeriodElapsedCallback()
{
    PidCalculate();

    float tmp_value = target_current_ + feedforward_current_;
    math_constrain(&tmp_value, -current_max_, current_max_);
    out_ = tmp_value * current_to_out_;

    Output();

    feedforward_current_ = 0.0f;
    feedforward_omega_ = 0.0f;
}

/**
 * @brief 数据处理过程
 *
 */
void MotorDjiC610::DataProcess()
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    MotorDjiCanRxData *tmp_buffer = (MotorDjiCanRxData *) can_manage_object_->rx_buffer.data;

    // 处理大小端
    math_endian_reverse_16((void *) &tmp_buffer->encoder_reverse, (void *) &tmp_encoder);
    math_endian_reverse_16((void *) &tmp_buffer->omega_reverse, (void *) &tmp_omega);
    math_endian_reverse_16((void *) &tmp_buffer->current_reverse, (void *) &tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - rx_data_.pre_encoder;
    if (delta_encoder < -encoder_num_per_round_ / 2)
    {
        // 正方向转过了一圈
        rx_data_.total_round++;
    }
    else if (delta_encoder > encoder_num_per_round_ / 2)
    {
        // 反方向转过了一圈
        rx_data_.total_round--;
    }
    rx_data_.total_encoder = rx_data_.total_round * encoder_num_per_round_ + tmp_encoder;

    // 计算电机本身信息
    rx_data_.now_angle = (float) rx_data_.total_encoder / (float) encoder_num_per_round_ * 2.0f * PI / gearbox_rate_;
    rx_data_.now_omega = (float) tmp_omega * RPM_TO_RADPS / gearbox_rate_;
    rx_data_.now_current = tmp_current / current_to_out_;
    rx_data_.now_temperature = tmp_buffer->temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    rx_data_.pre_encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void MotorDjiC610::PidCalculate()
{
    switch (motor_dji_control_method_)
    {
    case (MOTOR_DJI_CONTROL_METHOD_CURRENT):
    {
        break;
    }
    case (MOTOR_DJI_CONTROL_METHOD_OMEGA):
    {
        pid_omega_.SetTarget(target_omega_ + feedforward_omega_);
        pid_omega_.SetNow(rx_data_.now_omega);
        pid_omega_.CalculatePeriodElapsedCallback();

        target_current_ = pid_omega_.GetOut();

        break;
    }
    case (MOTOR_DJI_CONTROL_METHOD_ANGLE):
    {
        pid_angle_.SetTarget(target_angle_);
        pid_angle_.SetNow(rx_data_.now_angle);
        pid_angle_.CalculatePeriodElapsedCallback();

        target_omega_ = pid_angle_.GetOut();

        pid_omega_.SetTarget(target_omega_ + feedforward_omega_);
        pid_omega_.SetNow(rx_data_.now_omega);
        pid_omega_.CalculatePeriodElapsedCallback();

        target_current_ = pid_omega_.GetOut();

        break;
    }
    default:
    {
        target_current_ = 0.0f;

        break;
    }
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void MotorDjiC610::Output()
{
    tx_data_[0] = (int16_t) out_ >> 8;
    tx_data_[1] = (int16_t) out_;
}

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param can_rx_id CAN ID
 * @param motor_dji_control_method 电机控制方式, 默认速度
 * @param gearbox_rata 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param power_limit_status 是否开启功率控制
 * @param current_max 最大电流
 */
void MotorDjiC620::Init(
    FDCAN_HandleTypeDef *hcan, 
    MotorDjiId can_rx_id, 
    MotorDjiControlMethod motor_dji_control_method, 
    float gearbox_rata, 
    MotorDjiPowerLimitStatus power_limit_status, 
    float current_max)
{
    if (hcan->Instance == FDCAN1)
    {
        can_manage_object_ = &g_can1_manage_object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        can_manage_object_ = &g_can2_manage_object;
    }
    can_rx_id_ = can_rx_id;
    motor_dji_control_method_ = motor_dji_control_method;
    power_limit_status_ = power_limit_status;
    gearbox_rate_ = gearbox_rata;
    current_max_ = current_max;
    tx_data_ = allocate_tx_data(hcan, can_rx_id);
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data_ 接收的数据
 */
void MotorDjiC620::CanRxCpltCallback(uint8_t *rx_data_)
{
    // 滑动窗口, 判断电机是否在线
    flag_ += 1;

    DataProcess();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void MotorDjiC620::AlivePeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (flag_ == pre_flag_)
    {
        // 电机断开连接
        motor_dji_status_ = MOTOR_DJI_STATUS_DISABLE;
        motor_dji_control_method_ = MOTOR_DJI_CONTROL_METHOD_CURRENT;
        pid_angle_.SetIntegralError(0.0f);
        pid_omega_.SetIntegralError(0.0f);
        target_current_ = 0.0f;
    }
    else
    {
        // 电机保持连接
        motor_dji_status_ = MOTOR_DJI_STATUS_ENABLE;
    }
    pre_flag_ = flag_;
}

/**
 * @brief 中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void MotorDjiC620::CalculatePeriodElapsedCallback()
{
    PidCalculate();

    float tmp_value = target_current_ + feedforward_current_;
    math_constrain(&tmp_value, -current_max_, current_max_);
    out_ = tmp_value * current_to_out_;

    // 计算功率估计值
    power_estimate_ = power_calculate(
                        power_k_0_, 
                        power_k_1_, 
                        power_k_2_, 
                        power_A_, 
                        target_current_, 
                        rx_data_.now_omega / gearbox_rate_);

    Output();

    if (power_limit_status_ == MOTOR_DJI_POWER_LIMIT_STATUS_DISABLE)
    {
        feedforward_current_ = 0.0f;
        feedforward_omega_ = 0.0f;
    }
}

/**
 * @brief TIM定时器中断功率控制善后计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void MotorDjiC620::PowerLimitAfterCalculatePeriodElapsedCallback()
{
    if (power_limit_status_ == MOTOR_DJI_POWER_LIMIT_STATUS_ENABLE)
    {
        PowerLimitControl();
    }

    math_constrain(&target_current_, -current_max_, current_max_);
    out_ = target_current_ * current_to_out_;

    Output();

    feedforward_current_ = 0.0f;
    feedforward_omega_ = 0.0f;
}

/**
 * @brief 数据处理过程
 *
 */
void MotorDjiC620::DataProcess()
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    MotorDjiCanRxData *tmp_buffer = (MotorDjiCanRxData *) can_manage_object_->rx_buffer.data;

    // 处理大小端
    math_endian_reverse_16((void *) &tmp_buffer->encoder_reverse, (void *) &tmp_encoder);
    math_endian_reverse_16((void *) &tmp_buffer->omega_reverse, (void *) &tmp_omega);
    math_endian_reverse_16((void *) &tmp_buffer->current_reverse, (void *) &tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - rx_data_.pre_encoder;
    if (delta_encoder < -encoder_num_per_round_ / 2)
    {
        // 正方向转过了一圈
        rx_data_.total_round++;
    }
    else if (delta_encoder > encoder_num_per_round_ / 2)
    {
        // 反方向转过了一圈
        rx_data_.total_round--;
    }
    rx_data_.total_encoder = rx_data_.total_round * encoder_num_per_round_ + tmp_encoder;

    // 计算电机本身信息
    rx_data_.now_angle = 
        (float) rx_data_.total_encoder / (float) encoder_num_per_round_ * 2.0f * PI / gearbox_rate_;
    rx_data_.now_omega = (float) tmp_omega * RPM_TO_RADPS / gearbox_rate_;
    rx_data_.now_current = tmp_current / current_to_out_;
    rx_data_.now_temperature = tmp_buffer->temperature + CELSIUS_TO_KELVIN;
    rx_data_.now_power = power_calculate(
                            power_k_0_, 
                            power_k_1_, 
                            power_k_2_, 
                            power_A_, 
                            rx_data_.now_current, 
                            rx_data_.now_omega * gearbox_rate_);

    // 存储预备信息
    rx_data_.pre_encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void MotorDjiC620::PidCalculate()
{
    switch (motor_dji_control_method_)
    {
    case (MOTOR_DJI_CONTROL_METHOD_CURRENT):
    {
        break;
    }
    case (MOTOR_DJI_CONTROL_METHOD_OMEGA):
    {
        pid_omega_.SetTarget(target_omega_ + feedforward_omega_);
        pid_omega_.SetNow(rx_data_.now_omega);
        pid_omega_.CalculatePeriodElapsedCallback();

        target_current_ = pid_omega_.GetOut();

        break;
    }
    case (MOTOR_DJI_CONTROL_METHOD_ANGLE):
    {
        pid_angle_.SetTarget(target_angle_);
        pid_angle_.SetNow(rx_data_.now_angle);
        pid_angle_.CalculatePeriodElapsedCallback();

        target_omega_ = pid_angle_.GetOut();

        pid_omega_.SetTarget(target_omega_ + feedforward_omega_);
        pid_omega_.SetNow(rx_data_.now_omega);
        pid_omega_.CalculatePeriodElapsedCallback();

        target_current_ = pid_omega_.GetOut();

        break;
    }
    default:
    {
        target_current_ = 0.0f;

        break;
    }
    }
}

/**
 * @brief 功率控制算法, 修改电流目标值
 *
 */
void MotorDjiC620::PowerLimitControl()
{
    // 若功率为正则考虑功率控制限制
    if (power_estimate_ > 0.0f)
    {
        if (power_factor_ >= 1.0f)
        {
            // 无需功率控制
        }
        else
        {
            // 需要功率控制

            // 根据功率估计公式解一元二次方程求电流值
            float a = power_k_2_;
            float b = power_k_0_ * rx_data_.now_omega;
            float c = power_A_ 
                    + power_k_1_ * rx_data_.now_omega * rx_data_.now_omega 
                    - power_factor_ * power_estimate_;
            float delta, h;
            delta = b * b - 4 * a * c;
            if (delta < 0.0f)
            {
                // 无解
                target_current_ = 0.0f;
            }
            else
            {
                arm_sqrt_f32(delta, &h);
                float result_1, result_2;
                result_1 = (-b + h) / (2.0f * a);
                result_2 = (-b - h) / (2.0f * a);

                // 两个潜在的可行电流值, 取绝对值最小的那个
                if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f))
                {
                    if ((target_current_ > 0.0f && result_1 > 0.0f) || (target_current_ < 0.0f && result_1 < 0.0f))
                    {
                        target_current_ = result_1;
                    }
                    else
                    {
                        target_current_ = result_2;
                    }
                }
                else
                {
                    if (math_abs(result_1) < math_abs(result_2))
                    {
                        target_current_ = result_1;
                    }
                    else
                    {
                        target_current_ = result_2;
                    }
                }
            }
        }
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void MotorDjiC620::Output()
{
    tx_data_[0] = (int16_t) out_ >> 8;
    tx_data_[1] = (int16_t) out_;
}

/************************ COPYRIGHT(C) HNUST-DUST **************************/
