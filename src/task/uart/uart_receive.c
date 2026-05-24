//
// Created by ���ο� on 25-4-9.
//

#include "uart_receive.h"
#include "FreeRTOS.h"
#include "rm_module.h"
#include "esp8266.h"
#include "usart.h"
#include <string.h>
#include "cmsis_os.h"
#include "rc_sbus.h"
#include "unitree_motor.h"
// �Զ������������
static volatile uint8_t usart1_rx_buffer_index;  // ��ǰʹ�õĽ��ջ�����
static volatile uint16_t usart1_rx_size;
static uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];

#define DBUS_RX_BUF_SIZE 41
// ��˹ң����
static volatile uint8_t usart5_rx_buffer_index;  // ��ǰʹ�õĽ��ջ�����
static volatile uint16_t usart5_rx_size;
static uint8_t usart5_rx_buffer[2][DBUS_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART5;

// ����ϵͳ���� 10
extern SemaphoreHandle_t xSemaphoreUART10;

#define AIR_SENSOR_RX_BUF_SIZE 32
static volatile uint8_t uart7_rx_buffer_index;
static volatile uint16_t uart7_rx_size[2];
static uint8_t uart7_rx_buffer[2][AIR_SENSOR_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART7;

struct air_sensor_msg air_sensor_data;
volatile uint32_t uart7_error_code;
volatile uint32_t uart7_rx_callback_cnt;
volatile uint32_t uart7_proc_cnt;
volatile uint32_t uart7_checksum_fail_cnt;
volatile uint16_t uart7_last_size;

// �ⲿ����
extern QueueSetHandle_t xUartQueueSet; // ������м����,ͳһ���������ж��ź���
extern SemaphoreHandle_t xSemaphoreUSART2;
#define MOTOR_RX_QUEUE_LENGTH 16

 QueueHandle_t motor_rx_queue = NULL;

static uint8_t usart2_rx_buffer[sizeof(MotorData_t)];
static uint8_t usart3_rx_buffer[sizeof(MotorData_t)];



void USART5_DMA_Init(void) {
    memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));
    // �ر�DMA�Ĵ�������жϣ�����������ж�
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], DBUS_RX_BUF_SIZE); // ������Ϻ�����
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void USART7_DMA_Init(void) {
    memset(uart7_rx_buffer, 0, sizeof(uart7_rx_buffer));
    memset(uart7_rx_size, 0, sizeof(uart7_rx_size));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_buffer[uart7_rx_buffer_index], AIR_SENSOR_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}

void USART10_DMA_Init(void) {
    esp8266_dma_init();
}



void process_uart5_data(void) {
    uint8_t finishedBuffer;

    if (xSemaphoreTake(xSemaphoreUART5, 0) == pdTRUE) {
        finishedBuffer = usart5_rx_buffer_index ^ 1;
        /* SBUSЭ����� */
        sbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);

        memset(usart5_rx_buffer[finishedBuffer], 0, DBUS_RX_BUF_SIZE);


        // memset(usart5_rx_buffer[finishedBuffer], 0, DBUS_RX_BUF_SIZE);
    }
}

void process_uart10_data(void) {
    if (xSemaphoreTake(xSemaphoreUART10, 0) == pdTRUE) {
        esp8266_process_rx();
    }
}

void process_uart7_data(void) {
    uint8_t finishedBuffer;

    if (xSemaphoreTake(xSemaphoreUART7, 0) == pdTRUE) {
        uart7_proc_cnt++;
        finishedBuffer = uart7_rx_buffer_index ^ 1;
        uint16_t size = uart7_rx_size[finishedBuffer];
        uint8_t *data = uart7_rx_buffer[finishedBuffer];

        // 在接收缓冲中搜索 0x2C 帧头（兼容上电时的长初始化帧）
        int found = 0;
        int max_pos = (int)size - 12;
        if (max_pos < 0) max_pos = 0;
        for (int pos = 0; pos <= max_pos; pos++) {
            if (data[pos] == 0x2C) {
                uint8_t *frame = data + pos;
                uint8_t checksum = 0;
                for (int i = 0; i < 11; i++) {
                    checksum += frame[i];
                }
                checksum = ~checksum + 1;

                if (checksum == frame[11]) {
                    air_sensor_data.voc      = ((uint16_t)frame[1] << 8) | frame[2];
                    air_sensor_data.hcho     = ((uint16_t)frame[3] << 8) | frame[4];
                    air_sensor_data.eco2     = ((uint16_t)frame[5] << 8) | frame[6];
                    air_sensor_data.temp     = ((int16_t)((uint16_t)frame[7] << 8 | frame[8])) / 10.0f;
                    air_sensor_data.humidity = (((uint16_t)frame[9] << 8) | frame[10]) / 10.0f;
                    found = 1;
                    break;
                }
            }
        }

        if (!found) {
            uart7_checksum_fail_cnt++;
        }

        memset(uart7_rx_buffer[finishedBuffer], 0, AIR_SENSOR_RX_BUF_SIZE);
    }
}

__attribute__((noreturn)) void USARTRecTask_Entry(void const * argument)
{
    QueueSetMemberHandle_t xActivatedMember;


    USART5_DMA_Init();
    USART10_DMA_Init();
    USART7_DMA_Init();
    /* USER CODE BEGIN USARTRecTask_Entry */
    /* Infinite loop */
    for(;;)
    {
        // �����ȴ���һ�ź�������
        xActivatedMember = xQueueSelectFromSet(xUartQueueSet, portMAX_DELAY);

        // �жϴ���Դ���������ݣ����ݴ������ݵ���Ҫ�̶ȵ�������˳��

        if (xActivatedMember == xSemaphoreUART5) {
            process_uart5_data();  // ��˹ң����
        } else if (xActivatedMember == xSemaphoreUART10) {
            process_uart10_data();
        } else if (xActivatedMember == xSemaphoreUART7) {
            process_uart7_data(); // ����ϵͳ����ܣ�
        }

        vTaskDelay(1);
    }
    /* USER CODE END USARTRecTask_Entry */
}

void unitree_motor_rs485_init(void)
{
    if (motor_rx_queue == NULL)
    {
        motor_rx_queue = xQueueCreate(MOTOR_RX_QUEUE_LENGTH, sizeof(MotorRxMessage_t));
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_buffer, sizeof(usart3_rx_buffer));
}
void unitree_motor_rs485_reset(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_buffer, sizeof(usart3_rx_buffer));
}
int sizea=0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    sizea=Size;
    if (huart->Instance == USART2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        MotorRxMessage_t msg;
        if (Size == sizeof(MotorData_t))
        {
            msg.channel = usart2_485;
            memcpy(&msg.motor_data, usart2_rx_buffer, sizeof(MotorData_t));
            xQueueSendFromISR(motor_rx_queue, &msg, &xHigherPriorityTaskWoken);//�Ѿ�����Ϣ���͸������ˣ�����Ҫ�ٽ���������
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
        xSemaphoreGiveFromISR(xSemaphoreUSART2,NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if(huart->Instance == UART5)
    {
        sizea=Size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        usart5_rx_size = Size;
        usart5_rx_buffer_index = usart5_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], DBUS_RX_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART5, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART10)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        esp8266_rx_isr(Size);
        xSemaphoreGiveFromISR(xSemaphoreUART10, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == UART7)
    {
        uart7_rx_callback_cnt++;
        uart7_last_size = Size;

        if (Size > AIR_SENSOR_RX_BUF_SIZE)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_buffer[uart7_rx_buffer_index], AIR_SENSOR_RX_BUF_SIZE);
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        uart7_rx_size[uart7_rx_buffer_index] = Size;
        uart7_rx_buffer_index = uart7_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_buffer[uart7_rx_buffer_index], AIR_SENSOR_RX_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART7, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }


}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], DBUS_RX_BUF_SIZE); // ���շ������������
        memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));							   // ������ջ���
    }

    if(huart->Instance == USART10)
    {
        esp8266_error_recover();
    }

    if(huart->Instance == UART7)
    {
        uart7_error_code = huart->ErrorCode;
        HAL_UART_AbortReceive(&huart7);
        __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_PEF);
        memset(uart7_rx_buffer, 0, sizeof(uart7_rx_buffer));
        memset(uart7_rx_size, 0, sizeof(uart7_rx_size));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_buffer[uart7_rx_buffer_index], AIR_SENSOR_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }


}