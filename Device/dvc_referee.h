#pragma  once
#include "stdlib.h"
#include "stdint.h"
#include "ui.h"
#include <cstdint>

/**
 * @brief 裁判系统命令码类型
 *
 */
enum RefereeCommandID : uint16_t
{
    Referee_Command_ID_GAME_STATUS = 0x0001,
    Referee_Command_ID_GAME_RESULT = 0x0002,
    Referee_Command_ID_GAME_ROBOT_HP = 0x0003,
    Referee_Command_ID_EVENT_SELF_DATA = 0x0101,
    Referee_Command_ID_EVENT_SELF_SUPPLY,
    Referee_Command_ID_EVENT_SELF_REFEREE_WARNING = 0x0104,
    Referee_Command_ID_EVENT_SELF_DART_STATUS = 0x0105,
    Referee_Command_ID_ROBOT_STATUS = 0x0201,
    Referee_Command_ID_ROBOT_POWER_HEAT = 0x0202,
    Referee_Command_ID_ROBOT_POSITION = 0x0203,
    Referee_Command_ID_ROBOT_BUFF = 0x0204,
    Referee_Command_ID_ROBOT_AERIAL_STATUS,
    Referee_Command_ID_ROBOT_DAMAGE = 0x0206,
    Referee_Command_ID_ROBOT_BOOSTER = 0x0207,
    Referee_Command_ID_ROBOT_REMAINING_AMMO = 0x0208,
    Referee_Command_ID_ROBOT_RFID = 0x0209,
    Referee_Command_ID_ROBOT_DART_COMMAND = 0x020A,
    Referee_Command_ID_ROBOT_SENTRY_LOCATION = 0x020B,
    Referee_Command_ID_ROBOT_RADAR_MARK = 0x020C,
    Referee_Command_ID_ROBOT_SENTRY_DECISION = 0x020D,
    Referee_Command_ID_ROBOT_RADAR_DECISION = 0x020E,
    Referee_Command_ID_INTERACTION = 0x0301,
    Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CUSTOM_CONTROLLER = 0x0302,
    Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_MINIMAP = 0x0303,
    Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_REMOTE_CONTROL = 0x0304,
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_RADAR = 0x0305,
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_CUSTOM_CONTROLLER = 0x0306,
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_SENTRY_SEMIAUTOMATIC_MINIMAP = 0x0307,
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_ROBOT_MINIMAP = 0x0308,
};

struct RefereeUartData
{
    uint8_t frame_header = 0xa5;
    uint16_t data_length;
    uint8_t sequence;
    uint8_t crc_8;
    RefereeCommandID referee_command_id;
    uint8_t data[125];
} __attribute__((packed));

// ...protocol definitions kept as private nested types to avoid对外暴露完整协议布局...
struct StatusData {
    uint8_t id = 3;                                 // 本机器人ID
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

enum class GameType : uint8_t {
    RMUC = 0x1,
    RMUL_Engineer = 0x2,
    IRCA = 0x3,
    RMUL_3V3 = 0x4,
    RMUL_Infantry = 0x5,
};
struct GameStatus {
    GameType game_type : 4;            // 比赛类型
    uint8_t game_progress : 4;        // 比赛进程
    uint16_t stage_remain_time;   // 当前阶段剩余时间（单位：s）
    uint64_t sync_timestamp;        // 同步时间戳（单位：ms）
} __attribute__((packed));
static constexpr uint16_t kGameStatusId = 0x0001;

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

    bool spin_status_ = false; // 小陀螺状态
    bool booster_status_ = false; // 摩擦轮状态
    bool supercap_status_ = false; // 超级电容状态

    // 保存解析后的数据（私有）
    StatusData status_{0};
    ShootData shoot_{};
    GameStatus game_status_{};
private:
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数


};