/**
 * @file alg_math.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef MODULES_ALGORITHM_MATH_H_
#define MODULES_ALGORITHM_MATH_H_
/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <cfloat>

/* Exported macros -----------------------------------------------------------*/

// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
// deg换算到rad
#define DEG_TO_RAD (PI / 180.0f)
// rad -> deg
#define RAD_TO_DEG (180.0f / PI)
// 摄氏度换算到开氏度
#define CELSIUS_TO_KELVIN (273.15f)
// 圆周率PI
#define PI (3.14159265358979323846f)

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void math_boolean_logical_not(bool *value);

void math_endian_reverse_16(void *address);

uint16_t math_endian_reverse_16(void *source, void *destination);

void math_endian_reverse_32(void *address);

uint32_t math_endian_reverse_32(void *source, void *destination);

uint8_t math_sum_8(uint8_t *address, uint32_t length);

uint16_t math_sum_16(uint16_t *address, uint32_t length);

uint32_t math_sum_32(uint32_t *address, uint32_t length);

float math_sinc(float x);

int32_t math_float_to_int(float x, float float_min, float float_max, int32_t int_min, int32_t int_max);

float math_int_to_float(int32_t x, int32_t int_min, int32_t int_max, float float_min, float float_max);

float get_relative_angle_pm_pi(float now_angle_cum, float zero_angle);

float normalize_angle_diff(float target, float now);

float CalcYawError(float target, float now);

float normalize_angle_pm_pi(float angle);

float slew_limit(float cmd, float prev, float dt, float max_rate);

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
template<typename Type>
Type math_constrain(Type *x, Type min, Type max)
{
    if (*x < min)
    {
        *x = min;
    }
    else if (*x > max)
    {
        *x = max;
    }
    return (*x);
}

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template<typename Type>
Type math_abs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
 * @brief 求取模归化
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param modulus 模数
 * @return Type 返回的归化数, 介于 ±modulus / 2 之间
 */
template<typename Type>
Type math_modulus_normalization(Type x, Type modulus)
{
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f)
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}


#endif

/************************ COPYRIGHT(C) HNUST-DUST **************************/
