#pragma  once
#include "stdlib.h"
#include "stdint.h"
#include "ui.h"

class Referee{
public:
    void Init();
    void Task();
    void FreshDynamicUI();
    void FreshStaticUI();
    void RxCpltCallback(uint8_t *buffer, uint16_t length);

    // 只对外暴露需要的字段（按需添加）
    uint16_t GetCurrentHp() const { return status_.current_hp; }
    uint16_t GetMaxHp() const { return status_.max_hp; }
    uint16_t GetChassisPowerLimit() const { return status_.chassis_power_limit; }
    bool IsGimbalPowerOn() const { return status_.power_management_gimbal_output != 0; }

    uint8_t GetBulletType() const { return shoot_.bullet_type; }
    uint8_t GetLaunchingFrequency() const { return shoot_.launching_frequency; }
    float GetInitialSpeed() const { return shoot_.initial_speed; }

private:
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数

    // ...protocol definitions kept as private nested types to avoid对外暴露完整协议布局...
    struct StatusData {
        uint8_t id;                                     // 本机器人ID
        uint8_t level;                                  // 机器人等级
        uint16_t current_hp;                            // 当前血量
        uint16_t max_hp;                                // 最大血量
        uint16_t shooter_barrel_cooling_value;          // 射击热量每秒冷却值
        uint16_t shooter_barrel_heat_limit;             // 射击热量上限
        uint16_t chassis_power_limit;                   // 底盘功率上限
        // 电源管理模块的输出情况
        // bit0：gimbal口输出，0为无输出，1为24V输出
        // bit1：chassis口输出，0为无输出，1为24V输出
        // bit2：shooter口输出，0为无输出，1为24V输出
        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1; 
        uint8_t power_management_shooter_output : 1;
    } __attribute__((packed));
    static constexpr uint16_t kStatusDataId = 0x0201;

    struct ShootData {
        // 弹丸类型
        // 1：17mm 
        // 2：42mm
        uint8_t bullet_type;
        // 发射机构ID
        // 1：17mm发射机构
        // 2：保留位
        // 3：42mm发射机构            
        uint8_t shooter_number;         
        uint8_t launching_frequency;    // 发射频率（单位：Hz）
        float initial_speed;            // 发射初速度（单位：m/s）
    } __attribute__((packed));
    static constexpr uint16_t kShootDataId = 0x0207;

    // 保存解析后的数据（私有）
    StatusData status_{};
    ShootData shoot_{};
    // ISR 中只设置该标志，Task 中处理实际的 UI 更新
    volatile bool ui_update_requested_ = false;
};