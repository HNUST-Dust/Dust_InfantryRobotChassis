/**
 * @file imu.hpp
 * @brief IMU/INS 任务包装（遗留实现）：初始化 BMI088 + 启动 1kHz 循环调用 `INS_Task()`。
 *
 * 说明：
 * - 该类以“自建 RTOS 任务”的方式驱动 INS 更新，属于偏旧的耦合式写法（直接包含 BMI088/SPI/INS 相关头）。
 * - 当前工程其它模块多数采用 Topic 化与显式 Bind/Start 的方式，本文件暂时保留以兼容现有 bring-up。
 *
 * 注意：
 * - `Init()` 内部直接 `osThreadNew()`；调用方需确保 CMSIS-OS2 已启动。
 * - 任务循环固定 1kHz（`osDelay(1ms)`），若后续需要统一调度/观测，应考虑迁移到专用任务模块并 Topic 化输出。
 */

#ifndef IMU_H_
#define IMU_H_

#include "BMI088driver.h"
#include "spi.h"
#include "ins_task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"


class Imu {
private:
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *argument)
    {
        Imu *self = static_cast<Imu *>(argument);  // 还原 this 指针
        self->Task();  // 调用成员函数

    }
public:
    void Task()
    {
        for(;;)
        {
            INS_Task();
            osDelay(pdMS_TO_TICKS(1)); // 1kHz
        }
        
    }
    void Init()
    {
        // 陀螺仪初始化
        BMI088_Init(&hspi2, 0);// 不启用校准模式    
        INS_Init(); // 逆时针为+ ，-180 ~ 180
        static const osThreadAttr_t ImuTaskAttr = {
            .name = "ImuTask",
            .stack_size = 512,
            .priority = (osPriority_t) osPriorityNormal
        };
        // 启动任务，将 this 传入
        osThreadNew(TaskEntry, this, &ImuTaskAttr);
    }

    inline float GetYawAngleTotalAngle()
    {
        return INS.YawTotalAngle;
    }
};

#endif /* IMU_H_ */