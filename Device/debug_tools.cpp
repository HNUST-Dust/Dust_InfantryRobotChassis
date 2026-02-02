/**
 * @file debug_tools.cpp
 * @brief DebugTools 实现：VOFA 数据打包并发布到 `orb::uart_tx`，以及 RX 驱动的在线 feed。
 */

#include "debug_tools.h"

#include <cstdint>
#include <cstring>

#include "../communication_topic/uart_topics.hpp"

#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"

namespace {
inline uint32_t now_ms()
{
    return static_cast<uint32_t>(dwt_get_timeline_ms());
}

void vofa_daemon_fault(DaemonClient&) {}

DaemonClient* s_vofa_daemon = nullptr;
} // namespace

DebugTools& DebugTools_Instance()
{
    static DebugTools inst;
    return inst;
}

void DebugTools::Bind(BspUartHandle uart, orb::UartPort vofa_port)
{
    uart_ = uart;
    vofa_port_ = vofa_port;
    bound_ = true;
}

bool DebugTools::Start()
{
    if (started_) {
        return false;
    }
    // For VOFA we only need TX port configuration; RX is wired in App_WirePlatformIo().
    if (!bound_) {
        return false;
    }

    // Online criterion: receiving fresh VOFA RX frames from external debug tool.
    {
        static DaemonClient daemon(
            1000,
            vofa_daemon_fault,
            this,
            DaemonClient::Domain::COMM,
            DaemonClient::FaultLevel::WARN,
            DaemonClient::Priority::LOW);
        s_vofa_daemon = &daemon;
        (void)DaemonSupervisor::register_client(s_vofa_daemon);
        // Baseline timestamp; subsequent feed is driven by VofaReceiveCallback.
        s_vofa_daemon->feed(now_ms());
    }

    started_ = true;
    return true;
}

void DebugTools::VofaInit(){
    // VOFA 发送改为 Topic 化：由 UartTxTask 统一异步发送。
}

void DebugTools::VofaSendFloat(float data)
{
    orb::UartTxFrame pkt{};
    pkt.port = vofa_port_;
    pkt.len = 4;
    static_assert(sizeof(float) == 4, "VOFA float must be 4 bytes");
    std::memcpy(pkt.bytes, &data, 4);
    orb::uart_tx.publish(pkt);
}

void DebugTools::VofaSendTail()
{
    orb::UartTxFrame pkt{};
    pkt.port = vofa_port_;
    pkt.len = 4;
    pkt.bytes[0] = 0x00;
    pkt.bytes[1] = 0x00;
    pkt.bytes[2] = 0x80;
    pkt.bytes[3] = 0x7f;
    orb::uart_tx.publish(pkt);
}

void DebugTools::VofaReceiveCallback(uint8_t *buffer, uint16_t length)
{
    (void)buffer;
    (void)length;

    if (s_vofa_daemon) {
        s_vofa_daemon->feed(now_ms());
    }
}