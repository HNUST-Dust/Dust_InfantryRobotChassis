#pragma once

#include "topic.hpp"

// Referee UI Tx Topic：用于异步发送裁判系统 UI 数据帧，避免在 UI 生成/业务线程里直接阻塞 UART。
// 说明：UI 帧长度不固定（常见几十~上百字节），这里用 RingTopic 缓冲若干帧。

namespace orb {

// 单帧 UI 原始字节流（小包即可覆盖现有 generated_ui 帧：1/2/5/7 图元 + string + delete）
struct UiTxFrame {
    uint16_t len = 0;
    uint8_t bytes[128] = {0};
};

inline RingTopic<UiTxFrame, 16> ui_tx;

}  // namespace orb
