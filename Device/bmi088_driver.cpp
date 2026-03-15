#include "bmi088_driver.h"
#include "bmi088_reg.h"
#include "bmi088_middleware.h"


// 兼容：原驱动使用过全局灵敏度变量；现在由 BMI088 类持有。

Bmi088Drv Bmi088Drv::instance_{};

// 量程选择：跟随 bmi088_driver.h 里开启的宏，避免“写寄存器量程”和“软件换算灵敏度”不一致。
#if defined(BMI088_ACCEL_RANGE_24G)
#define BMI088_ACCEL_RANGE_REG_VALUE BMI088_ACC_RANGE_24G
#elif defined(BMI088_ACCEL_RANGE_12G)
#define BMI088_ACCEL_RANGE_REG_VALUE BMI088_ACC_RANGE_12G
#elif defined(BMI088_ACCEL_RANGE_6G)
#define BMI088_ACCEL_RANGE_REG_VALUE BMI088_ACC_RANGE_6G
#else
#define BMI088_ACCEL_RANGE_REG_VALUE BMI088_ACC_RANGE_3G
#endif

#if defined(BMI088_GYRO_RANGE_125)
#define BMI088_GYRO_RANGE_REG_VALUE BMI088_GYRO_125
#elif defined(BMI088_GYRO_RANGE_250)
#define BMI088_GYRO_RANGE_REG_VALUE BMI088_GYRO_250
#elif defined(BMI088_GYRO_RANGE_500)
#define BMI088_GYRO_RANGE_REG_VALUE BMI088_GYRO_500
#elif defined(BMI088_GYRO_RANGE_1000)
#define BMI088_GYRO_RANGE_REG_VALUE BMI088_GYRO_1000
#else
#define BMI088_GYRO_RANGE_REG_VALUE BMI088_GYRO_2000
#endif



#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:      \tBMI088_accel_write_single_reg(reg, data)
* @param:      reg - �Ĵ�����ַ
*               data - д�������
* @retval:     \tvoid
* @details:    \tͨ��BMI088���ٶȼƵ�SPI����д�뵥���Ĵ����ĺ궨��
************************************************************************
**/
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
/**
************************************************************************
* @brief:      \tBMI088_accel_read_single_reg(reg, data)
* @param:      reg - �Ĵ�����ַ
*               data - ��ȡ�ļĴ�������
* @retval:     \tvoid
* @details:    \tͨ��BMI088���ٶȼƵ�SPI���߶�ȡ�����Ĵ����ĺ궨��
************************************************************************
**/
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
/**
************************************************************************
* @brief:      \tBMI088_accel_read_muli_reg(reg, data, len)
* @param:      reg - ��ʼ�Ĵ�����ַ
*               data - �洢��ȡ���ݵĻ�����
*               len - Ҫ��ȡ���ֽ���
* @retval:     \tvoid
* @details:    \tͨ��BMI088���ٶȼƵ�SPI����������ȡ����Ĵ����ĺ궨��
************************************************************************
**/
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
/**
************************************************************************
* @brief:      \tBMI088_gyro_write_single_reg(reg, data)
* @param:      reg - �Ĵ�����ַ
*               data - д�������
* @retval:     \tvoid
* @details:    \tͨ��BMI088�����ǵ�SPI����д�뵥���Ĵ����ĺ궨��
************************************************************************
**/
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
************************************************************************
* @brief:      \tBMI088_gyro_read_single_reg(reg, data)
* @param:      reg - �Ĵ�����ַ
*               data - ��ȡ�ļĴ�������
* @retval:     \tvoid
* @details:    \tͨ��BMI088�����ǵ�SPI���߶�ȡ�����Ĵ����ĺ궨��
************************************************************************
**/
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
************************************************************************
* @brief:      \tBMI088_gyro_read_muli_reg(reg, data, len)
* @param:      reg - ��ʼ�Ĵ�����ַ
*               data - �洢��ȡ���ݵĻ�����
*               len - Ҫ��ȡ���ֽ���
* @retval:     \tvoid
* @details:    \tͨ��BMI088�����ǵ�SPI����������ȡ����Ĵ����ĺ궨��
************************************************************************
**/
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif

uint8_t Bmi088Drv::InitAccel()
{
    uint8_t res = 0;

    struct RegWriteCheck {
        uint8_t reg;
        uint8_t value;
        uint8_t error;
    };

    static constexpr RegWriteCheck kInitTable[] = {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, static_cast<uint8_t>(BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set), BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACCEL_RANGE_REG_VALUE, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, static_cast<uint8_t>(BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW), BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR},
    };

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (const auto& item : kInitTable)
    {
        BMI088_accel_write_single_reg(item.reg, item.value);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(item.reg, res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != item.value)
        {
            return item.error;
        }
    }
    return BMI088_NO_ERROR;
}
uint8_t Bmi088Drv::InitGyro()
{
    uint8_t res = 0;

    struct RegWriteCheck {
        uint8_t reg;
        uint8_t value;
        uint8_t error;
    };

    static constexpr RegWriteCheck kInitTable[] = {
        {BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_REG_VALUE, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, static_cast<uint8_t>(BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, static_cast<uint8_t>(BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
    };

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (const auto& item : kInitTable)
    {
        BMI088_gyro_write_single_reg(item.reg, item.value);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(item.reg, res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != item.value)
        {
            return item.error;
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t Bmi088Drv::Init()
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    error |= InitAccel();
    error |= InitGyro();

    return error;
}
void Bmi088Drv::Read(std::array<float, 3>& gyro, std::array<float, 3>& accel, float& temperate) noexcept
{
    uint8_t buf[8] = {};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * accel_sen_;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * accel_sen_;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * accel_sen_;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * gyro_sen_;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * gyro_sen_;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * gyro_sen_;
    }

    // temperature comes from accelerometer temperature registers
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void Bmi088Drv::Read(Sample& out) noexcept
{
    Read(out.gyro, out.accel, out.temperature);
}

Bmi088Drv::Sample Bmi088Drv::Read() noexcept
{
    Sample sample;
    Read(sample);
    return sample;
}

#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:      \tBMI088_write_single_reg(uint8_t reg, uint8_t data)
* @param:      reg - �Ĵ�����ַ
* @param:      data - д�������
* @retval:     \tvoid
* @details:    \t��BMI088������д�뵥���Ĵ���������
************************************************************************
**/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}
/**
************************************************************************
* @brief:      \tBMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
* @param:      reg - �Ĵ�����ַ
* @param:      return_data - ��ȡ�ļĴ�������
* @retval:     \tvoid
* @details:    \t��BMI088��������ȡ�����Ĵ���������
************************************************************************
**/
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}
/**
************************************************************************
* @brief:      \tBMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
* @param:      reg - ��ʼ�Ĵ�����ַ
*               buf - �洢��ȡ���ݵĻ�����
*               len - Ҫ��ȡ���ֽ���
* @retval:     \tvoid
* @details:    \t��BMI088������������ȡ����Ĵ���������
************************************************************************
**/
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
