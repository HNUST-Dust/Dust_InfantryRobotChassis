#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include <array>
#include <cstdint>

#ifndef __packed
#define __packed __attribute__((packed, aligned(1)))
#endif

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150


#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125

#if (defined(BMI088_ACCEL_RANGE_3G) + defined(BMI088_ACCEL_RANGE_6G) + defined(BMI088_ACCEL_RANGE_12G) + defined(BMI088_ACCEL_RANGE_24G)) != 1
#error "Select exactly one of BMI088_ACCEL_RANGE_3G/6G/12G/24G"
#endif

#if (defined(BMI088_GYRO_RANGE_2000) + defined(BMI088_GYRO_RANGE_1000) + defined(BMI088_GYRO_RANGE_500) + defined(BMI088_GYRO_RANGE_250) + defined(BMI088_GYRO_RANGE_125)) != 1
#error "Select exactly one of BMI088_GYRO_RANGE_2000/1000/500/250/125"
#endif


#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f


#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

enum BMI088Error {
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

class Bmi088Drv
{
public:
    struct __packed RawData {
        uint8_t status;
        int16_t accel[3];
        int16_t temp;
        int16_t gyro[3];
    };

    struct RealData {
        uint8_t status;
        float accel[3];
        float temp;
        float gyro[3];
        float time;
    };

    struct Sample {
        std::array<float, 3> gyro{};
        std::array<float, 3> accel{};
        float temperature = 0.0f;
    };

    static Bmi088Drv& Instance() noexcept
    {
        return instance_;
    }

    uint8_t Init();
    uint8_t InitAccel();
    uint8_t InitGyro();

    void Read(std::array<float, 3>& gyro, std::array<float, 3>& accel, float& temperature) noexcept;
    void Read(Sample& out) noexcept;
    [[nodiscard]] Sample Read() noexcept;

    void SetAccelSensitivity(float accel_sen) noexcept { accel_sen_ = accel_sen; }
    void SetGyroSensitivity(float gyro_sen) noexcept { gyro_sen_ = gyro_sen; }
    float GetAccelSensitivity() const noexcept { return accel_sen_; }
    float GetGyroSensitivity() const noexcept { return gyro_sen_; }

private:
    Bmi088Drv() = default;
    Bmi088Drv(const Bmi088Drv&) = delete;
    Bmi088Drv& operator=(const Bmi088Drv&) = delete;

    static Bmi088Drv instance_;

#if defined(BMI088_ACCEL_RANGE_24G)
    float accel_sen_ = BMI088_ACCEL_24G_SEN;
#elif defined(BMI088_ACCEL_RANGE_12G)
    float accel_sen_ = BMI088_ACCEL_12G_SEN;
#elif defined(BMI088_ACCEL_RANGE_6G)
    float accel_sen_ = BMI088_ACCEL_6G_SEN;
#else
    float accel_sen_ = BMI088_ACCEL_3G_SEN;
#endif

#if defined(BMI088_GYRO_RANGE_125)
    float gyro_sen_ = BMI088_GYRO_125_SEN;
#elif defined(BMI088_GYRO_RANGE_250)
    float gyro_sen_ = BMI088_GYRO_250_SEN;
#elif defined(BMI088_GYRO_RANGE_500)
    float gyro_sen_ = BMI088_GYRO_500_SEN;
#elif defined(BMI088_GYRO_RANGE_1000)
    float gyro_sen_ = BMI088_GYRO_1000_SEN;
#else
    float gyro_sen_ = BMI088_GYRO_2000_SEN;
#endif
};

#endif
