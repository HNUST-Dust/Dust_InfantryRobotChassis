/**
 * @file dvc_mcu_comm.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef MODULES_COMM_DVC_MCU_COMM_H
#define MODULES_COMM_DVC_MCU_COMM_H

#include "bsp_can.h"
#include "supercap.h"
#include <cstdint>

enum ChassisSpinMode
{
    CHASSIS_SPIN_CLOCKWISE          = 0,
    CHASSIS_SPIN_DISABLE            = 1,
    CHASSIS_SPIN_COUNTER_CLOCK_WISE = 2,
};

struct McuCommData
{
    uint8_t         yaw              = 127;                     // yaw
    uint8_t         pitch_angle      = 127;                     // 俯仰角度
    uint8_t         chassis_speed_x  = 127;                     // 平移方向：前、后、左、右
    uint8_t         chassis_speed_y  = 127;                     // 底盘移动总速度
    uint8_t         chassis_rotation = 127;                     // 自转：不转、顺时针转、逆时针转
    ChassisSpinMode chassis_spin     = CHASSIS_SPIN_DISABLE;    // 小陀螺：不转、顺时针转、逆时针转
    uint8_t         supercap         = SUPERCAP_STATUS_DISABLE; // 超级电容：充电、放电
};
constexpr uint8_t REMOTE_CONTROL_ID = 0xAB;

struct McuSendData
{
    float yaw_angle;   
    float yaw_omega;  
    float pitch_angle; 
    float pitch_omega; 
};
constexpr uint16_t GIMBAL_INFO_ID      = 0x0A;

struct McuAutoaimData
{
    float yaw_angle;
    float yaw_omega;
    float yaw_torque;
    float pitch_angle;
    float pitch_omega;
    float pitch_torque;
};
constexpr uint8_t AUTOAIM_INFO_ID    = 0xFA;

struct McuImuData
{
    float yaw_total_angle_f;
    float pitch_f;
    float yaw_omega_f;
};
constexpr uint8_t IMU_INFO_ID    = 0xAE;

class McuComm
{
public:

    volatile McuCommData mcu_comm_data_ = {
            127,
            127,
            127,
            127,
            127,
            CHASSIS_SPIN_DISABLE,
            0,
    };

    McuSendData mcu_send_data_ = {
            0.0f,
            0.0f,
            0.0f,
            0.0f,
    };

    McuAutoaimData mcu_autoaim_data_ = {
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
    };

    McuImuData mcu_imu_data_ = {
            0,
            0,
            0,
    };

    void Init(
        FDCAN_HandleTypeDef *hcan,
        uint8_t can_rx_id,
        uint8_t can_tx_id
    );

    void CanRemoteControlRxCpltCallback(uint8_t *rx_data);
    void CanAutoAimInfoRxCpltCallback(uint8_t *rx_data);
    void CanImuInfoRxCpltCallback(uint8_t *rx_data);
    
    void CanSendCommand();

    void Task();

protected:
    // 绑定的CAN
    CanManageObject *can_manage_object_;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t can_rx_id_;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t can_tx_id_;
    // 发送缓冲区
    uint8_t tx_data_[8];
    // 内部函数
    void DataProcess();

    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

#endif //MODULES_COMM_DVC_MCU_COMM_H
