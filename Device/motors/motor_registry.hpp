#pragma once

#include <cstddef>
#include <cstdint>

#include "can_topics.hpp"

namespace actuator::drivers {

// 电机注册表 key：用 (bus + tx_id + rx_id) 描述一个电机实例。
// - bus: CAN 总线
// - tx_id: 该电机驱动发送使用的“基准/组帧”ID（不同协议含义不同）
// - rx_id: 该电机反馈/分发使用的 ID（不同协议含义不同）
struct MotorKey {
    orb::CanBus bus = orb::CanBus::CAN1;
    uint16_t tx_id = 0;
    uint16_t rx_id = 0;
};

inline bool operator==(const MotorKey& a, const MotorKey& b)
{
    return a.bus == b.bus && a.tx_id == b.tx_id && a.rx_id == b.rx_id;
}

// 小容量、零动态内存的电机注册表。
// 设计目标：减少每个驱动重复写的“数组 + 查找 + 替换注册”样板代码。
template <typename T, size_t N>
class MotorRegistry final {
public:
    // 注册或替换：若 key 已存在则替换 ptr，否则放到第一个空位。
    // 返回是否成功（满了会失败）。
    bool RegisterOrReplace(const MotorKey& key, T* motor)
    {
        // replace
        for (auto& e : entries_) {
            if (e.ptr && e.key == key) {
                e.ptr = motor;
                return true;
            }
        }

        // insert
        for (auto& e : entries_) {
            if (!e.ptr) {
                e.key = key;
                e.ptr = motor;
                return true;
            }
        }
        return false;
    }

    // 精确查找 (bus + tx + rx)
    T* FindExact(const MotorKey& key) const
    {
        for (const auto& e : entries_) {
            if (e.ptr && e.key == key) {
                return e.ptr;
            }
        }
        return nullptr;
    }

    // 常用便捷查找：按 (bus + rx) 定位实例。
    // 适用于 topic 命令本身只携带 rx_id 的场景。
    T* FindByBusRx(orb::CanBus bus, uint16_t rx_id) const
    {
        for (const auto& e : entries_) {
            if (!e.ptr) {
                continue;
            }
            if (e.key.bus == bus && e.key.rx_id == rx_id) {
                return e.ptr;
            }
        }
        return nullptr;
    }

    // 遍历所有已注册实例（用于需要“对所有电机做一次 Update/Publish”的驱动）。
    template <typename F>
    void ForEach(F&& fn) const
    {
        for (const auto& e : entries_) {
            if (!e.ptr) {
                continue;
            }
            fn(e.key, *e.ptr);
        }
    }

private:
    struct Entry {
        MotorKey key{};
        T* ptr = nullptr;
    };

    Entry entries_[N]{};
};

} // namespace actuator::drivers
