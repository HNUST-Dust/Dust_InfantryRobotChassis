/**
 * @file bmi088.h
 * @brief BMI088 模块：采样 + 解算 + topic 发布（内部启动 RTOS 任务）
 *
 * 设计目标：
 * =========
 * - 将 BMI088 读周期集中到一个专用任务，其他模块只订阅 `orb::imu_data`。
 * - 模块内部直接完成 BMI088 初始化、姿态解算（QuaternionEKF/Mahony）以及 topic 发布。
 */

#pragma once

#include <array>
#include <cstdint>

#include "cmsis_os2.h"

#include "FreeRTOS.h"
#include "task.h"

#include "MahonyAHRS.h"
#include "control/dji_pid.h"
#include "estimation/QuaternionEKF.h"

class Bmi088Drv;

class Bmi088 {
public:
    Bmi088();

    bool Start();

    void Init() { (void)Start(); }

    float yaw_rad;

private:
    enum class Stage : uint8_t {
        BiasCalibrating = 0,
        WarmUp,
        Running,
    };

    static void TaskEntry(void* arg);
    void Task();

    void AlgoInit();
    void HwInit();
    void TemperatureCtrlStep();

    bool started_ = false;
    osThreadId_t thread_ = nullptr;

    // Driver and runtime states
    Bmi088Drv& drv_;

    Stage stage_ = Stage::BiasCalibrating;

    alg::DjiPid temperature_pid_{};

    std::array<float, 3> gyro_{};
    std::array<float, 3> accel_{};
    float temp_c_ = 0.0f;

    std::array<float, 3> gyro_bias_sum_{};
    std::array<float, 3> gyro_bias_{};
    std::uint32_t bias_samples_ = 0;

    std::uint32_t temp_stable_ticks_ = 0;
    std::uint32_t loop_count_ = 0;

    bool first_mahony_done_ = false;
    alg::QuaternionEkf quat_ekf_;
    alg::MahonyAhrs mahony_;

    StaticTask_t tcb_{};
    StackType_t stack_[4096]{};
};
