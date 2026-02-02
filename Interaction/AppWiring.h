/**
 * @file AppWiring.h
 * @brief 平台 IO 装配（UART/CAN RX 回调 → 模块入口）
 *
 * 设计思路：
 * =========
 * 本文件定义“平台层 → 模块层”的装配入口 `App_WirePlatformIo()`。
 * 它的职责是注册 UART/CAN 的 RX 回调，并在回调里将帧快速分发到对应模块实例。
 *
 * 边界：
 * =====
 * - 只做接收侧装配，不做发送侧。
 * - 不做业务算法，不做耗时操作。
 *
 * 线程/中断约束：
 * ============
 * 回调可能运行在 IRQ 或专用接收任务上下文（取决于 BSP 实现）。
 * 因此回调内应保持：
 * - 不阻塞
 * - 不做重计算
 * - 只做转发/投递/轻量解析
 */

#pragma once

/**
 * @brief 注册 UART/CAN 的 RX 回调并完成扇出分发
 */
void App_WirePlatformIo(void);
