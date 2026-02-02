/**
 * @file Init.h
 * @brief 系统启动入口（Legacy glue，供 CubeMX/RTOS 生成代码调用）
 *
 * 设计思路：
 * =========
 * `Init()` 是工程对“外部生成代码”的兼容入口。
 * 典型用法是 FreeRTOS/CMSIS-RTOS 的初始化阶段调用 `Init()`，再由此进入
 * 项目的统一启动流程（当前为 `System_Boot()`）。
 *
 * 边界与约束：
 * ==========
 * - 该入口不应承载业务逻辑，只负责跳转到统一启动序列。
 * - 保持 C ABI（`extern "C"`），避免生成代码因 C++ 名字改编而找不到符号。
 */

#ifndef INIT_H
#define INIT_H

#ifdef __cplusplus
extern "C" {
#endif

// Legacy entry (kept for CubeMX generated freertos.c)
void Init(void);

#ifdef __cplusplus
}
#endif

#endif // INIT_H
