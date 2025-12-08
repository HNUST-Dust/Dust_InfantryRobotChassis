#include "dvc_mcu_comm.h"
#include "cmsis_os2.h"
#include <cstring>
void McuComm::Init(
     FDCAN_HandleTypeDef* hcan,
     uint8_t can_rx_id,
     uint8_t can_tx_id)
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
     can_tx_id_ = can_tx_id;
}


void McuComm::CanSendCommand()
{
     static uint8_t can_tx_frame[8];
     can_tx_frame[0] = mcu_send_data_.start_of_frame;
     can_tx_frame[1] = mcu_send_data_.armor;
     // 把 float 转换成字节
     union { float f; uint8_t b[4]; } conv;

     conv.f = mcu_send_data_.yaw;
     can_tx_frame[2] = conv.b[0];
     can_tx_frame[3] = conv.b[1];
     can_tx_frame[4] = conv.b[2];
     can_tx_frame[5] = conv.b[3];
     can_tx_frame[6] = 0x00;
     can_tx_frame[7] = 0x00;

     // 发送第一帧（8字节）
     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);

     // ---- 第2帧：pitch 的 4 个字节 ----
     conv.f = mcu_send_data_.pitch;
     can_tx_frame[0] = conv.b[0];
     can_tx_frame[1] = conv.b[1];
     can_tx_frame[2] = conv.b[2];
     can_tx_frame[3] = conv.b[3];
     can_tx_frame[4] = 0xBA;
     can_tx_frame[5] = 0x00;
     can_tx_frame[6] = 0x00;
     can_tx_frame[7] = 0x00;

     // 发送第二帧（8字节）
     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}


void McuComm::CanRxCpltCallback(uint8_t* rx_data)
{
     // 判断在线

     // 处理数据 , 解包
     switch (rx_data[0])
     {
          case 0xAB: // 遥控包
               mcu_comm_data_.start_of_frame       = rx_data[0];
               mcu_comm_data_.yaw                  = rx_data[1];
               mcu_comm_data_.pitch_angle          = rx_data[2];
               mcu_comm_data_.chassis_speed_x      = rx_data[3];
               mcu_comm_data_.chassis_speed_y      = rx_data[4];
               mcu_comm_data_.chassis_rotation     = rx_data[5];
               switch(rx_data[6])
               {
                    case 0:
                    mcu_comm_data_.chassis_spin = CHASSIS_SPIN_CLOCKWISE;
                    break;
                    case 1:
                    mcu_comm_data_.chassis_spin = CHASSIS_SPIN_DISABLE;
                    break;
                    case 2:
                    mcu_comm_data_.chassis_spin = CHASSIS_SPIN_COUNTER_CLOCK_WISE;
                    break;
                    default:
                    mcu_comm_data_.chassis_spin = CHASSIS_SPIN_DISABLE;
                    break;
               }
               mcu_comm_data_.supercap             = rx_data[7];
               break;
          case 0xAC: // 自瞄yaw包
               mcu_autoaim_data_.start_of_yaw_frame = rx_data[0];
               mcu_autoaim_data_.yaw[0]             = rx_data[1];
               mcu_autoaim_data_.yaw[1]             = rx_data[2];
               mcu_autoaim_data_.yaw[2]             = rx_data[3];
               mcu_autoaim_data_.yaw[3]             = rx_data[4];
               break;
          case 0xAD: // 自瞄pitch包
               mcu_autoaim_data_.start_of_pitch_frame  = rx_data[0];
               mcu_autoaim_data_.pitch[0]              = rx_data[1];
               mcu_autoaim_data_.pitch[1]              = rx_data[2];
               mcu_autoaim_data_.pitch[2]              = rx_data[3];
               mcu_autoaim_data_.pitch[3]              = rx_data[4];
               break;
          case 0xAE: // 云台IMU yaw包
               mcu_imu_data_.start_of_yaw_frame = rx_data[0];
               mcu_imu_data_.yaw_total_angle[0]             = rx_data[1];
               mcu_imu_data_.yaw_total_angle[1]             = rx_data[2];
               mcu_imu_data_.yaw_total_angle[2]             = rx_data[3];
               mcu_imu_data_.yaw_total_angle[3]             = rx_data[4];
               memcpy(&mcu_imu_data_.yaw_total_angle_f,mcu_imu_data_.yaw_total_angle,sizeof(float));
               break;
          case 0xAF: // 云台IMU pitch包
               mcu_imu_data_.start_of_pitch_frame = rx_data[0];
               mcu_imu_data_.pitch[0]             = rx_data[1];
               mcu_imu_data_.pitch[1]             = rx_data[2];
               mcu_imu_data_.pitch[2]             = rx_data[3];
               mcu_imu_data_.pitch[3]             = rx_data[4];
               memcpy(&mcu_imu_data_.pitch_f,mcu_imu_data_.pitch,sizeof(float));
               break;
          default:
               break;
     }

}
