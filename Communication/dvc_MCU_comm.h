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



enum ChassisSpinMode{
    CHASSIS_SPIN_CLOCKWISE          = 0,
    CHASSIS_SPIN_DISABLE            = 1,
    CHASSIS_SPIN_COUNTER_CLOCK_WISE = 2,
};

struct McuCommData
{
    uint8_t         start_of_frame;                             // 帧头
    uint8_t         yaw              = 127;                     // yaw
    uint8_t         pitch_angle      = 127;                     // 俯仰角度
    uint8_t         chassis_speed_x  = 127;                     // 平移方向：前、后、左、右
    uint8_t         chassis_speed_y  = 127;                     // 底盘移动总速度
    uint8_t         chassis_rotation = 127;                     // 自转：不转、顺时针转、逆时针转
    ChassisSpinMode chassis_spin     = CHASSIS_SPIN_DISABLE;    // 小陀螺：不转、顺时针转、逆时针转
    uint8_t         supercap         = SUPERCAP_STATUS_DISABLE; // 超级电容：充电、放电
};

struct McuSendData
{
    uint8_t SOF1 = 0x0A;
    float yaw_angle;   // 4字节浮点数
    uint8_t SOF2 = 0x0B;
    float yaw_omega;  // 4字节浮点数
    uint8_t SOF3 = 0x0C;
    float pitch_angle; // 4字节浮点数
    uint8_t SOF4 = 0x0D;
    float pitch_omega; // 4字节浮点数
};

struct McuAutoaimData
{
    uint8_t SOF1 = 0xFA;
    float yaw_angle;
    uint8_t SOF2 = 0xFB;
    float yaw_omega;
    uint8_t SOF3 = 0xFC;
    float yaw_torque;
    uint8_t SOF4 = 0xFD;
    float pitch_angle;
    uint8_t SOF5 = 0xFE;
    float pitch_omega;
    uint8_t SOF6 = 0xFF;
    float pitch_torque;
};

struct McuImuData
{
    uint8_t start_of_yaw_frame;
    uint8_t start_of_pitch_frame;
    uint8_t yaw_total_angle[4];
    float yaw_total_angle_f;
    uint8_t pitch[4];
    float pitch_f;
};

class McuComm
{
public:

    volatile McuCommData mcu_comm_data_ = {
            0xAB,
            127,
            127,
            127,
            127,
            127,
            CHASSIS_SPIN_DISABLE,
            0,
    };
    McuSendData mcu_send_data_ = {
            0x0A,
            0.0f,
            0x0B,
            0.0f,
            0x0C,
            0.0f,
            0x0D,
            0.0f,
    };

    McuAutoaimData mcu_autoaim_data_ = {
            0xFA,
            0.0f,
            0xFB,
            0.0f,
            0xFC,
            0.0f,
            0xFD,
            0.0f,
            0xFE,
            0.0f,
            0xFF,
            0.0f,
    };
    McuImuData mcu_imu_data_ = {
            0xAE,
            0xAF,
            {0x00,0x00,0x00,0x00},
            0,
            {0x00,0x00,0x00,0x00},
            0,
    };

    void Init(FDCAN_HandleTypeDef *hcan,
              uint8_t can_rx_id,
              uint8_t can_tx_id
              );

    void CanRxCpltCallback(uint8_t *rx_data);

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
