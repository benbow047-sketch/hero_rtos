/**
 * @file usart_recive_task.c
 * @brief USART receive task - DBUS遥控器 + 裁判系统接收
 * @date 2026-06-07
 * @author Jasper
 */

#include "usart_recive_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "usart.h"
#include <string.h>

/* ---- 缓冲区大小 ---- */
#define CUSTOMER_CONTROLLER_BUF_SIZE  63   // 自定义控制器
#define REFEREE_RX_BUF_SIZE           209  // 裁判系统

/* ---- 自定义控制器（UART1） ---- */
static volatile uint8_t usart1_rx_buffer_index = 0;
static volatile uint16_t usart1_rx_size = 0;
static uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];

/* ---- DBUS 遥控器（UART5） ---- */
static volatile uint8_t usart5_rx_buffer_index = 0;
static volatile uint16_t usart5_rx_size = 0;
static uint8_t usart5_rx_buffer[2][DBUS_RX_BUF_SIZE];

/* ---- 裁判系统（UART10） ---- */
static volatile uint8_t usart10_rx_buffer_index = 0;
static volatile uint16_t usart10_rx_size = 0;
static uint8_t usart10_rx_buffer[2][REFEREE_RX_BUF_SIZE];

/* ---- 信号量和队列集 ---- */
static SemaphoreHandle_t xSemaphoreUART1 = NULL;
static SemaphoreHandle_t xSemaphoreUART5 = NULL;
static SemaphoreHandle_t xSemaphoreUART10 = NULL;
static QueueSetHandle_t xUartQueueSet = NULL;

/* ---- DMA 初始化 ---- */
static void USART1_DMA_Init(void)
{
    memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

static void USART5_DMA_Init(void)
{
    memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], DBUS_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

static void USART10_DMA_Init(void)
{
    memset(usart10_rx_buffer, 0, sizeof(usart10_rx_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_rx_buffer[usart10_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

/* ---- 数据处理函数 ---- */
static void process_uart1_data(void)
{
    if (xSemaphoreTake(xSemaphoreUART1, 0) == pdTRUE)
    {
        uint8_t finishedBuffer = usart1_rx_buffer_index ^ 1;
        // TODO: 自定义控制器数据解析
        // referee_data_unpack(usart1_rx_buffer[finishedBuffer], usart1_rx_size);
        memset(usart1_rx_buffer[finishedBuffer], 0, CUSTOMER_CONTROLLER_BUF_SIZE);
    }
}

static void process_uart5_data(void)
{
    if (xSemaphoreTake(xSemaphoreUART5, 0) == pdTRUE)
    {
        uint8_t finishedBuffer = usart5_rx_buffer_index ^ 1;
        /* DBUS 协议解析 */
        dbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);
        memset(usart5_rx_buffer[finishedBuffer], 0, DBUS_RX_BUF_SIZE);
    }
}

static void process_uart10_data(void)
{
    if (xSemaphoreTake(xSemaphoreUART10, 0) == pdTRUE)
    {
        uint8_t finishedBuffer = usart10_rx_buffer_index ^ 1;
        // TODO: 裁判系统数据解析
        // referee_data_unpack(usart10_rx_buffer[finishedBuffer], usart10_rx_size);
        memset(usart10_rx_buffer[finishedBuffer], 0, REFEREE_RX_BUF_SIZE);
    }
}

/* ---- 串口接收线程入口 ---- */
void USARTRecTask_Entry(void *argument)
{
    // 创建信号量
    xSemaphoreUART1 = xSemaphoreCreateBinary();
    xSemaphoreUART5 = xSemaphoreCreateBinary();
    xSemaphoreUART10 = xSemaphoreCreateBinary();

    // 创建队列集，统一管理多个信号量
    xUartQueueSet = xQueueCreateSet(3);
    xQueueAddToSet(xSemaphoreUART1, xUartQueueSet);
    xQueueAddToSet(xSemaphoreUART5, xUartQueueSet);
    xQueueAddToSet(xSemaphoreUART10, xUartQueueSet);

    // 初始化 DMA 接收
    USART1_DMA_Init();
    USART5_DMA_Init();
    USART10_DMA_Init();

    QueueSetMemberHandle_t xActivatedMember;

    for(;;)
    {
        // 阻塞等待任一信号量触发
        xActivatedMember = xQueueSelectFromSet(xUartQueueSet, portMAX_DELAY);

        if (xActivatedMember == xSemaphoreUART1)
            process_uart1_data();
        else if (xActivatedMember == xSemaphoreUART5)
            process_uart5_data();
        else if (xActivatedMember == xSemaphoreUART10)
            process_uart10_data();
    }
}

/* ---- DMA 接收完成回调 ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        if (Size > DBUS_RX_BUF_SIZE) return;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        usart5_rx_size = Size;
        usart5_rx_buffer_index ^= 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], DBUS_RX_BUF_SIZE);
        xSemaphoreGiveFromISR(xSemaphoreUART5, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART10)
    {
        if (Size > REFEREE_RX_BUF_SIZE) return;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        usart10_rx_size = Size;
        usart10_rx_buffer_index ^= 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_rx_buffer[usart10_rx_buffer_index], REFEREE_RX_BUF_SIZE);
        xSemaphoreGiveFromISR(xSemaphoreUART10, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART1)
    {
        if (Size > CUSTOMER_CONTROLLER_BUF_SIZE) return;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        usart1_rx_size = Size;
        usart1_rx_buffer_index ^= 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE);
        xSemaphoreGiveFromISR(xSemaphoreUART1, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ---- UART 错误回调 ---- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], DBUS_RX_BUF_SIZE);
        memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));
    }

    if (huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_rx_buffer[usart10_rx_buffer_index], REFEREE_RX_BUF_SIZE);
        memset(usart10_rx_buffer, 0, sizeof(usart10_rx_buffer));
    }

    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE);
        memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));
    }
}
