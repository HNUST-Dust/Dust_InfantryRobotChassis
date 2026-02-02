/**
 * @file Init.cpp
 * @brief 系统启动入口实现（Legacy glue）
 *
 * 核心逻辑：
 * =========
 * `Init()` 仅做一件事：调用 `System_Boot()`。
 *
 * 为什么要单独保留 Init():
 * =====================
 * - 兼容 CubeMX/RTOS 生成的启动代码。
 * - 将“生成代码世界”和“项目启动序列”解耦，后续迁移/重构只需要改这里。
 */

#include "Init.h"

#include "system_startup.h"

void Init(void)
{
    System_Boot();
}