/**
 * @file QuaternionEKF.h
 * @brief 四元数姿态 EKF（带陀螺仪零偏估计与卡方检验）
 *
 * 设计思路：
 * =========
 * - 在嵌入式 IMU 姿态解算场景中，通过 EKF 同时估计四元数与陀螺仪 bias。
 * - 内部使用 `filter/kalman_filter.*` 作为矩阵与 KF 框架，并通过 User_Func 回调实现
 *   观测/线性化/融合等定制步骤。
 *
 * 线程模型：
 * =========
 * - 本实现暴露全局实例 `QEKF_INS`，默认按“单任务周期调用 update、其它处只读”的方式使用。
 * - 若多任务并发访问，应在调用方自行保证互斥或快照读取。
 *
 * 数据与单位：
 * =========
 * - 输入陀螺仪单位：rad/s；加速度单位：m/s^2。
 * - 输出欧拉角 `Yaw/Pitch/Roll` 为角度制（deg）。
 *
 * 注意事项：
 * =========
 * - 初始化会触发 `Kalman_Filter_Init()` 分配矩阵空间；应避免重复 init。
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include "filter/kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
