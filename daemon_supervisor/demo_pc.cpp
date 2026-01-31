#ifndef __ARM_ARCH
 // SITL / PC 模拟

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstdint>

using namespace std::chrono;

// -----------------------------
// DaemonClient
// -----------------------------
class DaemonClient {
public:
    enum class State { INIT, ONLINE, OFFLINE, RECOVERING };
    enum class Domain { SENSOR, CONTROL, COMM, POWER, GENERIC };
    enum class FaultLevel { WARN, DEGRADED, FATAL };
    enum class Priority { LOW, NORMAL, HIGH, CRITICAL };

    using Callback = void(*)(DaemonClient&);

    DaemonClient(uint32_t timeout_ms,
                 Callback cb,
                 void* owner,
                 Domain domain,
                 FaultLevel level,
                 Priority priority)
        : _timeout(timeout_ms),
          _last_seen(0),
          _callback(cb),
          _owner(owner),
          _domain(domain),
          _level(level),
          _priority(priority),
          _state(State::INIT)
    {}

    void feed(uint32_t now_ms) {
        _last_seen = now_ms;

        switch (_state) {
            case State::INIT: _state = State::ONLINE; break;
            case State::OFFLINE: _state = State::RECOVERING; break;
            case State::RECOVERING: _state = State::ONLINE; break;
            default: break;
        }
    }

    State state() const { return _state; }
    void* owner() const { return _owner; }

    // helper for timeout check (wrap-around safe)
    bool timed_out(uint32_t now) const {
        return (uint32_t)(now - _last_seen) > _timeout;
    }

    // public members for supervisor
    Callback _callback;
    Domain _domain;
    FaultLevel _level;
    Priority _priority;

private:
    uint32_t _timeout;
    uint32_t _last_seen;
    void* _owner;
    State _state;
};

// -----------------------------
// DaemonSupervisor
// -----------------------------
class DaemonSupervisor {
public:
    using SystemHook = void(*)(DaemonClient&);
    using HwFeed = void(*)();

    static constexpr int MAX_CLIENTS = 8;

    static void init() {
        std::lock_guard<std::mutex> lock(_mutex);
        _count = 0;
        _system_hook = nullptr;
        _hw_feed = nullptr;
    }

    static DaemonClient* register_client(DaemonClient* client) {
        if (!client) return nullptr;
        std::lock_guard<std::mutex> lock(_mutex);
        if (_count >= MAX_CLIENTS) return nullptr;
        _clients[_count++] = client;
        return client;
    }

    static void set_system_hook(SystemHook hook) {
        _system_hook = hook;
    }

    static void set_hw_feed(HwFeed f) {
        _hw_feed = f;
    }

    static void tick(uint32_t now_ms) {
        std::lock_guard<std::mutex> lock(_mutex);

        for (int i = 0; i < _count; i++) {
            auto* c = _clients[i];
            if (!c) continue;

            if (c->timed_out(now_ms)) {
                if (c->_callback && c->state() != DaemonClient::State::OFFLINE) {
                    std::cout << "[Daemon] Client timeout! Owner=" << c->owner() << "\n";
                    c->_callback(*c);
                }
                // set OFFLINE if not already
                if (c->state() != DaemonClient::State::OFFLINE)
                    c->feed(0); // feed(0) will move OFFLINE->RECOVERING next feed
            }
        }

        if (_hw_feed && critical_alive_()) _hw_feed();
    }

private:
    static bool critical_alive_() {
        for (int i = 0; i < _count; i++) {
            auto* c = _clients[i];
            if (!c) continue;
            if (c->_priority == DaemonClient::Priority::CRITICAL &&
                c->state() != DaemonClient::State::ONLINE)
                return false;
        }
        return true;
    }

    static std::mutex _mutex;
    static DaemonClient* _clients[MAX_CLIENTS];
    static int _count;
    static SystemHook _system_hook;
    static HwFeed _hw_feed;
};

// -----------------------------
// Static members
// -----------------------------
std::mutex DaemonSupervisor::_mutex;
DaemonClient* DaemonSupervisor::_clients[DaemonSupervisor::MAX_CLIENTS] = {nullptr};
int DaemonSupervisor::_count = 0;
DaemonSupervisor::SystemHook DaemonSupervisor::_system_hook = nullptr;
DaemonSupervisor::HwFeed DaemonSupervisor::_hw_feed = nullptr;

// -----------------------------
// Demo callback functions
// -----------------------------
void imu_fault(DaemonClient& c) {
    std::cout << "[Callback] IMU fault triggered! Owner=" << c.owner() << "\n";
}

void control_fault(DaemonClient& c) {
    std::cout << "[Callback] CONTROL fault triggered! Owner=" << c.owner() << "\n";
}

void system_fault(DaemonClient& c) {
    std::cout << "[SystemHook] FATAL client offline! Owner=" << c.owner() << " -> System reset!\n";
}

void hw_feed_demo() {
    std::cout << "[HW Feed] Hardware watchdog fed.\n";
}

// -----------------------------
// Main Demo
// -----------------------------
int main() {
    DaemonSupervisor::init();

    // create clients
    int imu_obj = 1;
    int ctrl_obj = 2;

    DaemonClient imu_client(
        500, imu_fault, &imu_obj,
        DaemonClient::Domain::SENSOR,
        DaemonClient::FaultLevel::DEGRADED,
        DaemonClient::Priority::NORMAL
    );

    DaemonClient ctrl_client(
        300, control_fault, &ctrl_obj,
        DaemonClient::Domain::CONTROL,
        DaemonClient::FaultLevel::FATAL,
        DaemonClient::Priority::CRITICAL
    );

    DaemonSupervisor::register_client(&imu_client);
    DaemonSupervisor::register_client(&ctrl_client);

    DaemonSupervisor::set_system_hook(system_fault);
    DaemonSupervisor::set_hw_feed(hw_feed_demo);

    uint32_t now_ms = 0;

    // simulate daemon task running at 100Hz
    for (int step = 0; step < 20; step++) {
        std::cout << "----- tick " << step << " -----\n";

        // feed imu every tick
        if (step != 5) imu_client.feed(now_ms);

        // feed control only first 3 ticks
        if (step < 3) ctrl_client.feed(now_ms);

        DaemonSupervisor::tick(now_ms);
        now_ms += 100; // simulate 100ms per tick

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
#endif