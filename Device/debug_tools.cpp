#include "debug_tools.h"

#include <cstdint>
#include <cstring>

#include "../communication_topic/debug_topics.hpp"

void DebugTools::VofaInit(){
    // VOFA 发送改为 Topic 化：由 VofaTxTask 统一异步发送。
}

void DebugTools::VofaSendFloat(float data)
{
    orb::VofaTx pkt{};
    static_assert(sizeof(float) == 4, "VOFA float must be 4 bytes");
    std::memcpy(pkt.bytes, &data, sizeof(pkt.bytes));
    orb::vofa_tx.publish(pkt);
}

void DebugTools::VofaSendTail()
{
    orb::VofaTx pkt{};
    pkt.bytes[0] = 0x00;
    pkt.bytes[1] = 0x00;
    pkt.bytes[2] = 0x80;
    pkt.bytes[3] = 0x7f;
    orb::vofa_tx.publish(pkt);
}

void DebugTools::VofaReceiveCallback(uint8_t *buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
}