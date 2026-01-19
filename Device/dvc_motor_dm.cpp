/**
 * @file dvc_motor_dm.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 清除电机错误信息, 传统模式有效
uint8_t kDmMotorCanMessageClearError[8] = { 0xff,
                                            0xff,
                                            0xff,
                                            0xff,
                                            0xff,
                                            0xff,
                                            0xff,
                                            0xfb};
// 使能电机, 传统模式有效
uint8_t kDmMotorCANMessageEnter[8] = {  0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xfc};
// 失能电机, 传统模式有效
uint8_t kDmMotorCanMessageExit[8] = {   0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xfd};
// 保存当前电机位置为零点, 传统模式有效
uint8_t kDmMotorCANMessageSaveZero[8] = {   0xff,
                                            0xff,
                                            0xff,
                                            0xff,
                                            0xff,
                                            0xff,
                                            0xff,
                                            0xfe};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区, 一拖四模式有效
 *
 * @param hcan CAN编号
 * @param __can_id CAN ID
 * @param __DJI_Motor_Driver_Version 大疆驱动版本, 当且仅当当前被分配电机为6020, 且是电流驱动新版本时选2023, 否则都是default
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(FDCAN_HandleTypeDef *hcan, enum MotorDmMotorId1To4 can_rx_id_1_to_4)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hfdcan1)
    {
        switch (can_rx_id_1_to_4)
        {
        case (MOTOR_DM_ID_0x301):
        {
            tmp_tx_data_ptr = &(g_can1_0x3fe_tx_data[0]);

            break;
        }
        case (MOTOR_DM_ID_0x302):
        {
            tmp_tx_data_ptr = &(g_can1_0x3fe_tx_data[2]);

            break;
        }
        case (MOTOR_DM_ID_0x303):
        {
            tmp_tx_data_ptr = &(g_can1_0x3fe_tx_data[4]);

            break;
        }
        case (MOTOR_DM_ID_0x304):
        {
            tmp_tx_data_ptr = &(g_can1_0x3fe_tx_data[6]);

            break;
        }
        case (MOTOR_DM_ID_0x305):
        {
            tmp_tx_data_ptr = &(g_can1_0x4fe_tx_data[0]);

            break;
        }
        case (MOTOR_DM_ID_0x306):
        {
            tmp_tx_data_ptr = &(g_can1_0x4fe_tx_data[2]);

            break;
        }
        case (MOTOR_DM_ID_0x307):
        {
            tmp_tx_data_ptr = &(g_can1_0x4fe_tx_data[4]);

            break;
        }
        case (MOTOR_DM_ID_0x308):
        {
            tmp_tx_data_ptr = &(g_can1_0x4fe_tx_data[6]);

            break;
        }
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (can_rx_id_1_to_4)
        {
        case (MOTOR_DM_ID_0x301):
        {
            tmp_tx_data_ptr = &(g_can2_0x3fe_tx_data[0]);

            break;
        }
        case (MOTOR_DM_ID_0x302):
        {
            tmp_tx_data_ptr = &(g_can2_0x3fe_tx_data[2]);

            break;
        }
        case (MOTOR_DM_ID_0x303):
        {
            tmp_tx_data_ptr = &(g_can2_0x3fe_tx_data[4]);

            break;
        }
        case (MOTOR_DM_ID_0x304):
        {
            tmp_tx_data_ptr = &(g_can2_0x3fe_tx_data[6]);

            break;
        }
        case (MOTOR_DM_ID_0x305):
        {
            tmp_tx_data_ptr = &(g_can2_0x4fe_tx_data[0]);

            break;
        }
        case (MOTOR_DM_ID_0x306):
        {
            tmp_tx_data_ptr = &(g_can2_0x4fe_tx_data[2]);

            break;
        }
        case (MOTOR_DM_ID_0x307):
        {
            tmp_tx_data_ptr = &(g_can2_0x4fe_tx_data[4]);

            break;
        }
        case (MOTOR_DM_ID_0x308):
        {
            tmp_tx_data_ptr = &(g_can2_0x4fe_tx_data[6]);

            break;
        }
        }
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param can_rx_id 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param can_tx_id 发数据绑定的CAN ID, 是上位机驱动参数can_id加上控制模式的偏移量, 传统模式有效
 * @param motor_dm_control_method 电机控制方式
 * @param angle_max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param omega_max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param torque_max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void MotorDmNormal::Init(
    FDCAN_HandleTypeDef *hcan,
    uint8_t can_rx_id,
    uint8_t can_tx_id,
    enum MotorDmControlMethod motor_dm_control_method,
    float angle_max, 
    float omega_max,
    float torque_max,
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
    else if (hcan->Instance == FDCAN3)
    {
        can_manage_object_ = &g_can3_manage_object;
    }

    can_rx_id_ = can_rx_id;
    switch (motor_dm_control_method)
    {
    case (MOTOR_DM_CONTROL_METHOD_NORMAL_MIT):
    {
        can_tx_id_ = can_tx_id;
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA):
    {
        can_tx_id_ = can_tx_id + 0x100;
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA):
    {
        can_tx_id_ = can_tx_id + 0x200;
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT):
    {
        can_tx_id_ = can_tx_id + 0x300;
        break;
    }
    }
    motor_dm_control_method_ = motor_dm_control_method;
    angle_max_ = angle_max;
    omega_max_ = omega_max;
    torque_max_ = torque_max;
    current_max_ = current_max;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data_ 接收的数据
 */
void MotorDmNormal::CanRxCpltCallback(uint8_t *rx_data_)
{
    // 滑动窗口, 判断电机是否在线
    flag_ += 1;

    DataProcess();
}

/**
 * @brief 发送清除错误信息
 *
 */
void MotorDmNormal::CanSendClearError()
{
    can_send_data(can_manage_object_->can_handler, can_tx_id_, kDmMotorCanMessageClearError, 8);
}

/**
 * @brief 发送使能电机
 *
 */
void MotorDmNormal::CanSendEnter()
{
    can_send_data(can_manage_object_->can_handler, can_tx_id_, kDmMotorCANMessageEnter, 8);
}

/**
 * @brief 发送失能电机
 *
 */
void MotorDmNormal::CanSendExit()
{
    can_send_data(can_manage_object_->can_handler, can_tx_id_, kDmMotorCanMessageExit, 8);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void MotorDmNormal::CanSendSaveZero()
{
    can_send_remote_data(can_manage_object_->can_handler, can_tx_id_, kDmMotorCANMessageSaveZero, 8);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void MotorDmNormal::AlivePeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (flag_ == pre_flag_)
    {
        // 电机断开连接
        motor_dm_status_ = MOTOR_DM_STATUS_DISABLE;
    }
    else
    {
        // 电机保持连接
        motor_dm_status_ = MOTOR_DM_STATUS_ENABLE;
    }

    pre_flag_ = flag_;

    if(motor_dm_status_ == MOTOR_DM_STATUS_DISABLE)
    {
        CanSendEnter();
    }
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void MotorDmNormal::SendPeriodElapsedCallback()
{
    if (rx_data_.control_status == MOTOR_DM_CONTROL_STATUS_ENABLE)
    {
        // 电机在线, 正常控制
        math_constrain(&control_angle_, -angle_max_, angle_max_);
        math_constrain(&control_omega_, -omega_max_, omega_max_);
        math_constrain(&control_torque_, -torque_max_, torque_max_);
        math_constrain(&control_current_, -current_max_, current_max_);
        math_constrain(&k_p_, 0.0f, 500.0f);
        math_constrain(&k_d_, 0.0f, 5.0f);

        Output();
    }
    else if (rx_data_.control_status == MOTOR_DM_CONTROL_STATUS_DISABLE)
    {
        // 电机可能掉线, 使能电机
        CanSendEnter();
    }
    else
    {
        // 电机错误, 发送清除错误帧
        CanSendClearError();
    }
}

/**
 * @brief 数据处理过程
 *
 */
void MotorDmNormal::DataProcess()
{
    // 数据处理过程
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    MotorDmCanRxDataNormal *tmp_buffer = (MotorDmCanRxDataNormal *)can_manage_object_->rx_buffer.data;

    // 电机ID不匹配, 则不进行处理
    if(tmp_buffer->can_id != (can_tx_id_ & 0x0f))
    {
        return;
    }

    // 处理大小端
    math_endian_reverse_16((void *)&tmp_buffer->angle_reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->omega_11_4 << 4) | (tmp_buffer->omega_3_0_torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->omega_3_0_torque_11_8 & 0x0f) << 8) | (tmp_buffer->torque_7_0);

    rx_data_.control_status = static_cast<MotorDmControlStatusNormal>(tmp_buffer->control_status_enum);

    // 计算圈数与总角度值
    delta_encoder = tmp_encoder - rx_data_.pre_encoder;
    if (delta_encoder < -(1 << 15))
    {
        // 正方向转过了一圈
        rx_data_.total_round++;
    }
    else if (delta_encoder > (1 << 15))
    {
        // 反方向转过了一圈
        rx_data_.total_round--;
    }
    rx_data_.total_encoder = rx_data_.total_round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);

    // 计算电机本身信息
    rx_data_.now_angle_noncumulative = ((float)((tmp_encoder / 65535.0f) * (angle_max_ * 2.0f)) - angle_max_);
    rx_data_.now_angle = (float)(rx_data_.total_encoder) / (float)((1 << 16) - 1) * angle_max_ * 2.0f;
    rx_data_.now_omega = math_int_to_float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, omega_max_);
    rx_data_.now_torque = math_int_to_float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, torque_max_);
    rx_data_.now_mos_temperature = tmp_buffer->mos_temperature + CELSIUS_TO_KELVIN;
    rx_data_.now_rotor_temperature = tmp_buffer->rotor_temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    rx_data_.pre_encoder = tmp_encoder;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
void MotorDmNormal::Output()
{
    // 电机控制
    switch (motor_dm_control_method_)
    {
    case (MOTOR_DM_CONTROL_METHOD_NORMAL_MIT):
    {
        MotorDmCanTxDataNormalMit *tmp_buffer = (MotorDmCanTxDataNormalMit *)tx_data_;

        uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

        tmp_angle = math_float_to_int(control_angle_, 0, angle_max_, 0x7fff, (1 << 16) - 1);
        tmp_omega = math_float_to_int(control_omega_, 0, omega_max_, 0x7ff, (1 << 12) - 1);
        tmp_torque = math_float_to_int(control_torque_, 0, torque_max_, 0x7ff, (1 << 12) - 1);
        tmp_k_p = math_float_to_int(k_p_, 0, 500.0f, 0, (1 << 12) - 1);
        tmp_k_d = math_float_to_int(k_d_, 0, 5.0f, 0, (1 << 12) - 1);

        tmp_buffer->control_angle_reverse = math_endian_reverse_16(&tmp_angle, nullptr);
        tmp_buffer->control_omega_11_4 = tmp_omega >> 4;
        tmp_buffer->control_omega_3_0_k_p_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
        tmp_buffer->k_p_7_0 = tmp_k_p & 0xff;
        tmp_buffer->k_d_11_4 = tmp_k_d >> 4;
        tmp_buffer->k_d_3_0_control_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        tmp_buffer->control_torque_7_0 = tmp_torque & 0xff;

        can_send_data(can_manage_object_->can_handler, can_tx_id_, tx_data_, 8);

        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA):
    {
        MotorDmCanTxDataNormalAngleOmega *tmp_buffer = (MotorDmCanTxDataNormalAngleOmega *)tx_data_;

        tmp_buffer->control_angle_ = control_angle_;
        tmp_buffer->control_omega_ = control_omega_;

        can_send_data(can_manage_object_->can_handler, can_tx_id_, tx_data_, 8);

        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA):
    {
        MotorDmCanTxDataNormalOmega *tmp_buffer = (MotorDmCanTxDataNormalOmega *)tx_data_;

        tmp_buffer->control_omega_ = control_omega_;

        can_send_data(can_manage_object_->can_handler, can_tx_id_, tx_data_, 4);

        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT):
    {
        MotorDmCanTxDataNormalEmit *tmp_buffer = (MotorDmCanTxDataNormalEmit *)tx_data_;

        tmp_buffer->control_angle = control_angle_;
        tmp_buffer->control_omega = (uint16_t)(control_omega_ * 100.0f);
        tmp_buffer->control_current = (uint16_t)(control_current_ / current_max_ * 10000.0f);

        can_send_data(can_manage_object_->can_handler, can_tx_id_, tx_data_, 8);

        break;
    }
    }
}

/**
 * @brief 电机初始化
 * 
 * @param hcan 绑定的CAN总线
 * @param can_rx_id 绑定的CAN ID
 * @param motor_dm_control_method 电机控制方式, 默认角度
 * @param encoder_offset 编码器偏移, 默认0
 * @param current_max 最大电流
 */
void MotorDm1To4::Init(
    FDCAN_HandleTypeDef *hcan, 
    MotorDmMotorId1To4 can_rx_id, 
    MotorDmControlMethod motor_dm_control_method, 
    int32_t encoder_offset, 
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
    motor_dm_control_method_ = motor_dm_control_method;
    encoder_offset_ = encoder_offset;
    current_max_ = current_max;
    tx_data_ = allocate_tx_data(hcan, can_rx_id);
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data_ 接收的数据
 */
void MotorDm1To4::CanRxCpltCallback(uint8_t *rx_data_)
{
    // 滑动窗口, 判断电机是否在线
    flag_ += 1;

    DataProcess();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void MotorDm1To4::AlivePeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (flag_ == pre_flag_)
    {
        // 电机断开连接
        motor_dm_status_ = MOTOR_DM_STATUS_DISABLE;
        pid_angle_.SetIntegralError(0.0f);
        pid_omega_.SetIntegralError(0.0f);
    }
    else
    {
        // 电机保持连接
        motor_dm_status_ = MOTOR_DM_STATUS_ENABLE;
    }
    pre_flag_ = flag_;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void MotorDm1To4::CalculatePeriodElapsedCallback()
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
void MotorDm1To4::DataProcess()
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    MotorDmCanRxData1To4 *tmp_buffer = (MotorDmCanRxData1To4 *) can_manage_object_->rx_buffer.data;

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
    rx_data_.total_encoder = rx_data_.total_round * encoder_num_per_round_ + tmp_encoder + encoder_offset_;

    // 计算电机本身信息
    rx_data_.now_angle = (float) rx_data_.total_encoder / (float) encoder_num_per_round_ * 2.0f * PI;
    rx_data_.now_omega = tmp_omega / 100.0f * RPM_TO_RADPS;
    rx_data_.now_current = tmp_current / 1000.0f;
    rx_data_.now_mos_temperature = tmp_buffer->mos_temperature + CELSIUS_TO_KELVIN;
    rx_data_.now_rotor_temperature = tmp_buffer->rotor_temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    rx_data_.pre_encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void MotorDm1To4::PidCalculate()
{
    switch (motor_dm_control_method_)
    {
    case (MOTOR_DM_CONTROL_METHOD_1_TO_4_CURRENT):
    {
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_1_TO_4_OMEGA):
    {
        pid_omega_.SetTarget(target_omega_ + feedforward_omega_);
        pid_omega_.SetNow(rx_data_.now_omega);
        pid_omega_.CalculatePeriodElapsedCallback();

        target_current_ = pid_omega_.GetOut();

        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_1_TO_4_ANGLE):
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
void MotorDm1To4::Output()
{
    *(int16_t *) tx_data_ = (int16_t)(out_);
}

/************************ COPYRIGHT(C) HNUST_DUST **************************/
