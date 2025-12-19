#include "dvc_MCU_comm.h"
#include "cmsis_os2.h"
#include <cstdint>
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
     // static const osThreadAttr_t kMcuCommTaskAttr = {
     //      .name = "mcu_comm_task",
     //      .stack_size = 512,
     //      .priority = (osPriority_t) osPriorityNormal
     // };
     // //启动任务，将 this 传入
     // osThreadNew(McuComm::TaskEntry, this, &kMcuCommTaskAttr);
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void McuComm::TaskEntry(void *argument) {
     McuComm *self = static_cast<McuComm *>(argument);  // 还原 this 指针
     self->Task();  // 调用成员函数
}

// 实际任务逻辑
void McuComm::Task() {
     struct McuCommData mcu_comm_data_local;
     for (;;)
     {
          // 用临界区一次性复制，避免撕裂
          // __disable_irq();
          // mcu_comm_data__Local = *const_cast<const struct McuCommData*>(&(mcu_comm_data_));
          // __enable_irq();
          // osDelay(pdMS_TO_TICKS(10));
     }
}


void McuComm::CanSendCommand() {

     static uint8_t can_tx_frame[16];
     // 把 float 转换成字节
     union { float f; uint8_t b[4]; } conv;
     // ---- 第1帧：yaw_angle 的4个字节和 yaw_omega 的4个字节 ----
     conv.f = mcu_send_data_.yaw_angle;
     can_tx_frame[0] = conv.b[0];
     can_tx_frame[1] = conv.b[1];
     can_tx_frame[2] = conv.b[2];
     can_tx_frame[3] = conv.b[3];

     conv.f = mcu_send_data_.yaw_omega;
     can_tx_frame[4] = conv.b[0];
     can_tx_frame[5] = conv.b[1];
     can_tx_frame[6] = conv.b[2];
     can_tx_frame[7] = conv.b[3];

     // ---- 第2帧：pitch_angle 的4个字节和 pitch_omega 的4个字节 ----
     conv.f = mcu_send_data_.pitch_angle;
     can_tx_frame[8] = conv.b[0];
     can_tx_frame[9] = conv.b[1];
     can_tx_frame[10] = conv.b[2];
     can_tx_frame[11] = conv.b[3];

     conv.f = mcu_send_data_.pitch_omega;
     can_tx_frame[12] = conv.b[0];
     can_tx_frame[13] = conv.b[1];
     can_tx_frame[14] = conv.b[2];
     can_tx_frame[15] = conv.b[3];

     // 发送第2帧（8字节）
     fdcan_send_data(can_manage_object_->can_handler, GIMBAL_INFO_ID, can_tx_frame, 16);
}

void McuComm::CanRemoteControlRxCpltCallback(uint8_t* rx_data) {

     mcu_comm_data_.yaw                  = rx_data[0];
     mcu_comm_data_.pitch_angle          = rx_data[1];
     mcu_comm_data_.chassis_speed_x      = rx_data[2];
     mcu_comm_data_.chassis_speed_y      = rx_data[3];
     mcu_comm_data_.chassis_rotation     = rx_data[4];
     switch(rx_data[5])
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
     mcu_comm_data_.supercap             = rx_data[6];
}

void McuComm::CanAutoAimInfoRxCpltCallback(uint8_t* rx_data) {
     memcpy(&mcu_autoaim_data_.yaw_angle,&rx_data[0],4 * sizeof(uint8_t));
     memcpy(&mcu_autoaim_data_.pitch_angle,&rx_data[4],4 * sizeof(uint8_t));

     memcpy(&mcu_autoaim_data_.yaw_omega,&rx_data[8],4 * sizeof(uint8_t));
     memcpy(&mcu_autoaim_data_.pitch_omega,&rx_data[12],4 * sizeof(uint8_t));

     memcpy(&mcu_autoaim_data_.yaw_torque,&rx_data[16],4 * sizeof(uint8_t));
     memcpy(&mcu_autoaim_data_.pitch_torque,&rx_data[20],4 * sizeof(uint8_t));
}

void McuComm::CanImuInfoRxCpltCallback(uint8_t* rx_data) {
     memcpy(&mcu_imu_data_.yaw_total_angle_f,&rx_data[0],4 * sizeof(uint8_t));
     memcpy(&mcu_imu_data_.pitch_f,&rx_data[4],4 * sizeof(uint8_t));
     memcpy(&mcu_imu_data_.yaw_omega_f,&rx_data[8],4 * sizeof(uint8_t));
}
