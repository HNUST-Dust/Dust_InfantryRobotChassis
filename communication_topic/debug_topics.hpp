#pragma once

#include "topic.hpp"

// Debug/VOFA Tx 语义化 Topic：用于异步发送调试通道数据，避免在控制环里阻塞等待 UART。
// 约定：VOFA 侧每次发送 4 字节 float，帧尾也是 4 字节。

namespace orb {

struct VofaTx {
    uint8_t bytes[4] = {0, 0, 0, 0};
};

// RingTopic：允许多个调用点快速 publish，由独立 TxTask drain 并发送。
inline RingTopic<VofaTx, 64> vofa_tx;

}  // namespace orb
