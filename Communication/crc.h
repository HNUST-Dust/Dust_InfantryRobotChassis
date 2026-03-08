/**
 * @file crc.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-12-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __CRC_H
#define __CRC_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include <stdio.h>               
#include <stdarg.h>  

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


unsigned char get_crc8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8); 
unsigned int verify_crc8_check_sum(unsigned char *pchMessage, unsigned int dwLength);
void append_crc8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

uint16_t get_crc16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t verify_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
void append_crc16_check_sum(uint8_t * pchMessage,uint32_t dwLength); 


#endif
