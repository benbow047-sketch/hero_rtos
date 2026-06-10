#include <stdio.h>
#include "bsp_fdcan.h"
#include "rm_module.h"
#include "supercap_task.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;




/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{

// 简化中断配置
    uint32_t FDCAN_RXActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_WATERMARK
                                 |FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_WATERMARK;

    can_filter_init();
    HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_RXActiveITs, 0);


    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_RXActiveITs, 0);


    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_RXActiveITs, 0);

}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter1;

    fdcan_filter1.IdType = FDCAN_STANDARD_ID;                       //标准ID
    fdcan_filter1.FilterIndex = 0;                                  //滤波器索引
    fdcan_filter1.FilterType = FDCAN_FILTER_MASK;                   //掩码模式
    fdcan_filter1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器1关联到FIFO0
    fdcan_filter1.FilterID1 = 0x000;                                // 匹配ID设为0
    fdcan_filter1.FilterID2 = 0x000;                                // 掩码设为0（所有位都不屏蔽）

    FDCAN_FilterTypeDef fdcan_filter2;

    fdcan_filter2.IdType = FDCAN_STANDARD_ID;                       //标准ID
    fdcan_filter2.FilterIndex = 1;                                  //滤波器索引
    fdcan_filter2.FilterType = FDCAN_FILTER_MASK;                   //掩码模式
    fdcan_filter2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器2关联到FIFO0
    fdcan_filter2.FilterID1 = 0x200;                // 基础ID（0x201~0x210的高位基准）
    fdcan_filter2.FilterID2 = 0xFF0;                // 掩码（屏蔽后4位，保留前7位）


    FDCAN_FilterTypeDef fdcan_filter3;

    fdcan_filter3.IdType = FDCAN_STANDARD_ID;                       //标准ID
    fdcan_filter3.FilterIndex = 2;                                  //滤波器索引
    fdcan_filter3.FilterType = FDCAN_FILTER_MASK;                   //掩码模式
    fdcan_filter3.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器3关联到FIFO00
    fdcan_filter3.FilterID1 = 0x000;                                // 匹配ID设为0
    fdcan_filter3.FilterID2 = 0x000;                                // 掩码设为0（所有位都不屏蔽）

    HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter1); 		 				  //接收ID2
    //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);

    HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter2); 		 				  //接收ID2
    //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);

    HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter3); 		 				  //接收ID2
    //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}


//static HAL_StatusTypeDef test;
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
static HAL_StatusTypeDef test;
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;

    if(len<=8)
        pTxHeader.DataLength = len;
    if(len==12)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
    if(len==16)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
    if(len==20)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
    if(len==24)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
    if(len==32)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
    if(len==48)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
    if(len==64)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_64;


    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;
    pTxHeader.FDFormat=FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;

    //test = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data);

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK)
        return 1;//发送
    return 0;

}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive_fifo0(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;
        if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
            len = pRxHeader.DataLength;
        else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_12)
            len = 12;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
            len = 16;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
            len = 20;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
            len = 24;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
            len = 32;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
            len = 48;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
            len = 64;

        return len;//接收数据
    }
    return 0;
}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive_fifo1(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1, &pRxHeader, buf)==HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;
        if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
            len = pRxHeader.DataLength;
        else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_12)
            len = 12;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
            len = 16;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
            len = 20;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
            len = 24;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
            len = 32;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
            len = 48;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
            len = 64;

        return len;//接收数据
    }
    return 0;
}




uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
    fdcanx_receive_fifo0(&hfdcan1, &rec_id1, rx_data1);
    //dji_motot_rx_callback(1,rec_id1, rx_data1);
    #ifdef BSP_USING_SUPERCAP_TASK
    supercap_can_rx_callback(rec_id1, rx_data1);
    #endif /* BSP_USING_SUPERCAP_TASK */
}
uint8_t rx_data2[8] = {0};
uint16_t rec_id2;
void fdcan2_rx_callback(void)
{
    fdcanx_receive_fifo0(&hfdcan2, &rec_id2, rx_data2);
    //dji_motot_rx_callback(2,rec_id2, rx_data2);
    #ifdef BSP_USING_SUPERCAP_TASK
    supercap_can_rx_callback(rec_id2, rx_data2);
    #endif /* BSP_USING_SUPERCAP_TASK */
}
uint8_t rx_data3[8] = {0};
uint16_t rec_id3;
void fdcan3_rx_callback(void)
{
    fdcanx_receive_fifo0(&hfdcan3, &rec_id3, rx_data3);
    //dji_motot_rx_callback(3,rec_id3, rx_data3);
    #ifdef BSP_USING_SUPERCAP_TASK
    supercap_can_rx_callback(rec_id3, rx_data3);
    #endif /* BSP_USING_SUPERCAP_TASK */
}


static int flag0 = 0, flag1 = 0, flag2 = 0, flag3 = 0, flag4 = 0;
static float cmd_dt;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static float cmd_start;
    cmd_start = dwt_get_time_us();
    flag0++;
    uint8_t max_loop = 8;  // 限制最大循环次数（建议为Rx FIFO深度的2倍）
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) && max_loop-- > 0)
    {
        if (hfdcan == &hfdcan1) {
            fdcan1_rx_callback();
            flag1++;
        }
        else if (hfdcan == &hfdcan2) {
            fdcan2_rx_callback();
            flag2++;
        }
        else if (hfdcan == &hfdcan3) {
            fdcan3_rx_callback();
            flag3++;
        } else {
            flag4++;
            // 异常情况：读取并丢弃数据，避免FIFO溢出
            FDCAN_RxHeaderTypeDef dummy_header;
            uint8_t dummy_data[8];
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &dummy_header, dummy_data);
        }
    }
    cmd_dt = dwt_get_time_us() - cmd_start;

}
static int flag10 = 0, flag11 = 0, flag12 = 0, flag13 = 0, flag14 = 0;
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo1ITs)
{

    flag10++;
    uint8_t max_loop = 8;  // 限制最大循环次数（建议为Rx FIFO深度的2倍）

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) && max_loop-- > 0)
    {
        if (hfdcan == &hfdcan1) {
            fdcan1_rx_callback();
            flag11++;
        }
        else if (hfdcan == &hfdcan2) {
            fdcan2_rx_callback();
            flag12++;
        }
        else if (hfdcan == &hfdcan3) {
            fdcan3_rx_callback();
            flag13++;
        } else {
            flag14++;
            // 异常情况：读取并丢弃数据，避免FIFO溢出
            FDCAN_RxHeaderTypeDef dummy_header;
            uint8_t dummy_data[8];
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &dummy_header, dummy_data);
        }
    }

}



void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if(ErrorStatusITs & FDCAN_IR_BO)
    {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
        // 所有CAN的Bus-Off都复位，不只是CAN3
        HAL_FDCAN_DeInit(hfdcan);
        if(hfdcan == &hfdcan1) MX_FDCAN1_Init();
        else if(hfdcan == &hfdcan2) MX_FDCAN2_Init();
        else if(hfdcan == &hfdcan3) MX_FDCAN3_Init();
        can_filter_init();
        bsp_can_init();
    }
    if(ErrorStatusITs & FDCAN_IR_EP)
    {
        if(hfdcan == &hfdcan1 )
        {
            MX_FDCAN1_Init();
            bsp_can_init();
        }
        else if(hfdcan == &hfdcan2)
        {
            MX_FDCAN2_Init();
            bsp_can_init();
        }
        else if(hfdcan == &hfdcan3 )
        {
            MX_FDCAN3_Init();
            bsp_can_init();
        }
    }
}



