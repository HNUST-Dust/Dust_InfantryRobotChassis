/**
 * @file bsp_can.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

CanManageObject g_can1_manage_object = {0};
CanManageObject g_can2_manage_object = {0};
CanManageObject g_can3_manage_object = {0};

// CAN通信发送缓冲区
uint8_t g_can1_0x1ff_tx_data[8];
uint8_t g_can1_0x200_tx_data[8];
uint8_t g_can1_0x2ff_tx_data[8];
uint8_t g_can1_0x3fe_tx_data[8];
uint8_t g_can1_0x4fe_tx_data[8];

uint8_t g_can2_0x1ff_tx_data[8];
uint8_t g_can2_0x200_tx_data[8];
uint8_t g_can2_0x2ff_tx_data[8];
uint8_t g_can2_0x3fe_tx_data[8];
uint8_t g_can2_0x4fe_tx_data[8];

uint8_t g_can_supercap_tx_data[8];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param callback_function 处理回调函数
 */
void can_init(FDCAN_HandleTypeDef *hcan, CanCallback callback_function)
{
    if (hcan->Instance == FDCAN1)
    {
        g_can1_manage_object.can_handler = hcan;
        g_can1_manage_object.callback_function = callback_function;
        can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == FDCAN2)
    {
        g_can2_manage_object.can_handler = hcan;
        g_can2_manage_object.callback_function = callback_function;
        can_filter_mask_config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == FDCAN3)
    {
        g_can3_manage_object.can_handler = hcan;
        g_can3_manage_object.callback_function = callback_function;
        can_filter_mask_config(hcan, CAN_FILTER(28) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(29) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }

    HAL_FDCAN_Start(hcan);

    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
}

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param object_para 编号 | FIFOx | ID类型 | 帧类型
 * @param id ID
 * @param Mask_ID 屏蔽位(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(FDCAN_HandleTypeDef *hcan, uint8_t object_para, uint32_t id, uint32_t mask_id)
{
    FDCAN_FilterTypeDef filter = {0};

    assert_param(hcan != NULL);

    // 解析参数
    uint8_t filter_index = object_para >> 3;
    uint8_t fifo_select  = (object_para >> 2) & 0x01;
    uint8_t id_type_flag = (object_para >> 1) & 0x01;
    uint8_t frame_type   = object_para & 0x01;

    filter.FilterIndex   = filter_index;
    filter.IdType        = id_type_flag ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterType    = FDCAN_FILTER_MASK;
    filter.FilterConfig  = (fifo_select == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1     = id;
    filter.FilterID2     = mask_id;
    UNUSED(frame_type); // 帧类型在FDCAN中不需要配置

    HAL_FDCAN_ConfigFilter(hcan, &filter);
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t can_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.Identifier            = id;
    tx_header.IdType                = FDCAN_STANDARD_ID;            // 标准ID
    tx_header.TxFrameType           = FDCAN_DATA_FRAME;             // 数据帧
    tx_header.DataLength            = length;                       // 数据长度，注意有些HAL可能用DLC编码，需要确认
    tx_header.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;             // 错误指示，默认正常
    tx_header.BitRateSwitch         = FDCAN_BRS_OFF;                // 不启用比特率切换
    tx_header.FDFormat              = FDCAN_CLASSIC_CAN;            // 经典CAN格式
    tx_header.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;           // 不启用事件FIFO
    tx_header.MessageMarker         = 0;                            // 消息标记为0
    UNUSED(used_mailbox); // 避免未使用变量警告
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, data);
}

uint8_t fdcan_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.Identifier            = id;
    tx_header.IdType                = FDCAN_STANDARD_ID;            // 标准ID
    tx_header.TxFrameType           = FDCAN_DATA_FRAME;             // 数据帧
    if(length <= 8){
        tx_header.DataLength        = length;                       
    }
    if(length == 12){
        tx_header.DataLength        = FDCAN_DLC_BYTES_12;          
    }
    if(length == 16){
        tx_header.DataLength        = FDCAN_DLC_BYTES_16;          
    }
    if(length == 20){
        tx_header.DataLength        = FDCAN_DLC_BYTES_20;          
    }
    if(length == 24){
        tx_header.DataLength        = FDCAN_DLC_BYTES_24;          
    }
    if(length == 32){
        tx_header.DataLength        = FDCAN_DLC_BYTES_32;          
    }
    if(length == 48){
        tx_header.DataLength        = FDCAN_DLC_BYTES_48;          
    }
    if(length == 64){
        tx_header.DataLength        = FDCAN_DLC_BYTES_64;          
    }
    tx_header.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;             // 错误指示，默认正常
    tx_header.BitRateSwitch         = FDCAN_BRS_ON;                 // 启用比特率切换
    tx_header.FDFormat              = FDCAN_FD_CAN;                 // FD CAN格式
    tx_header.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;           // 不启用事件FIFO
    tx_header.MessageMarker         = 0;                            // 消息标记为0
    UNUSED(used_mailbox); // 避免未使用变量警告
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, data);
}

void can_period_elapsed_callback()
{
    // DJI电机专属
    // CAN1电机

}
/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &g_can1_manage_object.rx_buffer.header,
                                   g_can1_manage_object.rx_buffer.data);

            g_can1_manage_object.callback_function(&g_can1_manage_object.rx_buffer);
        }
        else if (hfdcan->Instance == FDCAN2)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &g_can2_manage_object.rx_buffer.header,
                                   g_can2_manage_object.rx_buffer.data);

            g_can2_manage_object.callback_function(&g_can2_manage_object.rx_buffer);
        }
        else if (hfdcan->Instance == FDCAN3)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &g_can3_manage_object.rx_buffer.header,
                                   g_can3_manage_object.rx_buffer.data);

            g_can3_manage_object.callback_function(&g_can3_manage_object.rx_buffer);
        }
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1,
                                   &g_can1_manage_object.rx_buffer.header,
                                   g_can1_manage_object.rx_buffer.data);

            g_can1_manage_object.callback_function(&g_can1_manage_object.rx_buffer);
        }
        else if (hfdcan->Instance == FDCAN2)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1,
                                   &g_can2_manage_object.rx_buffer.header,
                                   g_can2_manage_object.rx_buffer.data);

            g_can2_manage_object.callback_function(&g_can2_manage_object.rx_buffer);
        }
        else if (hfdcan->Instance == FDCAN3)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1,
                                   &g_can3_manage_object.rx_buffer.header,
                                   g_can3_manage_object.rx_buffer.data);

            g_can3_manage_object.callback_function(&g_can3_manage_object.rx_buffer);
        }
    }
}


/************************ COPYRIGHT(C) HNUST-DUST **************************/
