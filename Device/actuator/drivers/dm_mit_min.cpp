#include "dm_mit_min.hpp"

#include <cmath>
#include <cstring>

namespace actuator::drivers {

namespace {
inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
inline uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    const float span = x_max - x_min;
    const float offset = x - x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    const float scaled = (offset * max_int) / span;
    if (scaled <= 0.0f) return 0;
    if (scaled >= static_cast<float>(max_int)) return max_int;
    return static_cast<uint32_t>(scaled);
}
inline float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits) {
    const float span = x_max - x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    return (static_cast<float>(x_int) * span) / static_cast<float>(max_int) + x_min;
}
} // namespace

uint16_t DmMitMin::MitTxStdId(uint8_t master_id) {
    // minimal: use the same std_id composition as existing DM MIT normal.
    // Legacy code uses base_can_id + 0x100 for MIT (approx). Here we keep it simple:
    // std_id = (master_id << 4) | 0x00;  (user can override via cfg.base_std_id if needed)
    return static_cast<uint16_t>(master_id) << 4;
}

void DmMitMin::Init(BspCanHandle can, const Config& cfg) {
    can_ = can;
    cfg_ = cfg;
    if (cfg_.base_std_id == 0) {
        cfg_.base_std_id = MitTxStdId(cfg_.master_id);
    }
}

void DmMitMin::OnRx(const uint8_t data[8]) {
    // DM normal rx payload: [id(4)|status(4)], angle(16), omega(12), torque(12), mosT, rotT
    const uint8_t id4 = (data[0] & 0x0F);
    if (id4 != (cfg_.can_rx_id & 0x0F)) {
        return;
    }

    const uint16_t angle_u16 = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    const uint16_t omega_u12 = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    const uint16_t torque_u12 = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];

    now_angle_ = uint_to_float(angle_u16, -cfg_.angle_max, cfg_.angle_max, 16);
    now_omega_ = uint_to_float(omega_u12, -cfg_.omega_max, cfg_.omega_max, 12);
    now_torque_ = uint_to_float(torque_u12, -cfg_.torque_max, cfg_.torque_max, 12);
}

void DmMitMin::SetControl(float angle, float omega, float torque) {
    ctrl_angle_ = clampf(angle, -cfg_.angle_max, cfg_.angle_max);
    ctrl_omega_ = clampf(omega, -cfg_.omega_max, cfg_.omega_max);
    ctrl_torque_ = clampf(torque, -cfg_.torque_max, cfg_.torque_max);
}

void DmMitMin::PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                       float pmax, float vmax, float kpmax, float kdmax, float tmax) {
    std::memset(out, 0, 8);

    const uint16_t p_u16 = static_cast<uint16_t>(float_to_uint(p, -pmax, pmax, 16));
    const uint16_t v_u12 = static_cast<uint16_t>(float_to_uint(v, -vmax, vmax, 12));
    const uint16_t kp_u12 = static_cast<uint16_t>(float_to_uint(kp, 0, kpmax, 12));
    const uint16_t kd_u12 = static_cast<uint16_t>(float_to_uint(kd, 0, kdmax, 12));
    const uint16_t t_u12 = static_cast<uint16_t>(float_to_uint(t, -tmax, tmax, 12));

    out[0] = static_cast<uint8_t>((p_u16 >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(p_u16 & 0xFF);
    out[2] = static_cast<uint8_t>((v_u12 >> 4) & 0xFF);
    out[3] = static_cast<uint8_t>(((v_u12 & 0x0F) << 4) | ((kp_u12 >> 8) & 0x0F));
    out[4] = static_cast<uint8_t>(kp_u12 & 0xFF);
    out[5] = static_cast<uint8_t>((kd_u12 >> 4) & 0xFF);
    out[6] = static_cast<uint8_t>(((kd_u12 & 0x0F) << 4) | ((t_u12 >> 8) & 0x0F));
    out[7] = static_cast<uint8_t>(t_u12 & 0xFF);
}

void DmMitMin::PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len) {
    if (!can_) return;

    orb::CanTxFrame f{};
    f.bus = cfg_.bus;
    f.id = std_id;
    f.id_type = orb::CanIdType::Std;
    f.frame_type = orb::CanFrameType::Data;
    f.is_fd = false;
    f.brs = false;
    f.len = len;
    std::memset(f.data, 0, sizeof(f.data));
    if (len > 0) {
        const uint8_t n = (len <= 8) ? len : 8;
        std::memcpy(f.data, data, n);
    }
    orb::can_tx.publish(f);
}

void DmMitMin::PublishMitTx(float kp, float kd) {
    // kp/kd limit use typical DM ranges
    uint8_t data[8];
    PackMit(ctrl_angle_, ctrl_omega_, kp, kd, ctrl_torque_, data,
            cfg_.angle_max, cfg_.omega_max, 500.0f, 5.0f, cfg_.torque_max);

    const uint16_t std_id = static_cast<uint16_t>(cfg_.base_std_id) | static_cast<uint16_t>(cfg_.can_rx_id & 0x0F);
    PublishFrame(std_id, data, 8);
}

void DmMitMin::Enter() {
    const uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    PublishFrame(static_cast<uint16_t>(cfg_.base_std_id) | (cfg_.can_rx_id & 0x0F), data, 8);
}
void DmMitMin::Exit() {
    const uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    PublishFrame(static_cast<uint16_t>(cfg_.base_std_id) | (cfg_.can_rx_id & 0x0F), data, 8);
}
void DmMitMin::ClearError() {
    const uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};
    PublishFrame(static_cast<uint16_t>(cfg_.base_std_id) | (cfg_.can_rx_id & 0x0F), data, 8);
}
void DmMitMin::SaveZero() {
    const uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
    PublishFrame(static_cast<uint16_t>(cfg_.base_std_id) | (cfg_.can_rx_id & 0x0F), data, 8);
}

} // namespace actuator::drivers
