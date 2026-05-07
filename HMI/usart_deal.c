#include "usart_deal.h"

#define len 128

extern volatile uint8_t task_measure;
extern volatile uint8_t task_sweep;
extern volatile uint8_t task_fault;
extern volatile uint8_t task_none;

typedef enum
{
    CMD_NUMB,
    TEST01,
    TEST02,
    TEST03,
} USART_RX_CMD_TYPE;
USART_RX_CMD_TYPE usart_rx_cmd_state;

uint8_t usart_rx_buffer[len] = {0};
volatile uint8_t usart_rx_buffer_index = 0;
volatile uint8_t usart_rx_proc_flag = 0;
char usart_rx_cmd[len] = {0};

void Usart_Send_Computer(UART_HandleTypeDef *huart, char *msg)
{
    HAL_UART_Transmit(huart, (uint8_t *)msg, strlen(msg), 1000);
}

void My_Usart_Init(void)
{
    printf("bkcmd=0\xff\xff\xff");
    HAL_Delay(20);

    memset(usart_rx_buffer, 0, sizeof(usart_rx_buffer));
    usart_rx_buffer_index = 0;
    usart_rx_proc_flag = 0;

    HAL_UART_Receive_IT(&huart1, &usart_rx_buffer[usart_rx_buffer_index], 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (usart_rx_buffer[usart_rx_buffer_index] == '\0')
        {
            /* Bug#2修复：收到帧结束符后立即重启IT。
             * 若不在此处重启，Sweep_Gain等阻塞任务执行期间（数十~数百ms）
             * Usart_Rx_Proc无法被调用，中断完全停止，后续字节全部丢失。
             * 重启IT写入位置为当前'\0'之后一格；Usart_Rx_Proc处理完成后
             * 会memset整个buffer并重置index，不会使用这一格的临时数据。 */
            usart_rx_proc_flag = 1;
            if (usart_rx_buffer_index < (uint8_t)(len - 1U))
            {
                HAL_UART_Receive_IT(&huart1,
                                    &usart_rx_buffer[usart_rx_buffer_index + 1U], 1);
            }
        }
        else
        {
            /* Bug#1修复：Nextion事件包（如0A FF FF FF）不含'\0'，
             * 若无边界检查，字节会持续累积直到usart_rx_buffer_index溢出
             * uint8_t范围（128），越界写入相邻内存。 */
            if (usart_rx_buffer_index < (uint8_t)(len - 1U))
            {
                usart_rx_buffer_index++;
                HAL_UART_Receive_IT(&huart1, &usart_rx_buffer[usart_rx_buffer_index], 1);
            }
            else
            {
                /* 溢出：丢弃当前帧，重新开始接收 */
                memset(usart_rx_buffer, 0, sizeof(usart_rx_buffer));
                usart_rx_buffer_index = 0;
                HAL_UART_Receive_IT(&huart1, &usart_rx_buffer[0], 1);
            }
        }
    }
}

void Usart_Rx_Proc(void)
{
    if (usart_rx_proc_flag)
    {

        usart_rx_proc_flag = 0;// 重置处理标志
        strcpy(usart_rx_cmd, (char *)usart_rx_buffer); // 将接收的命令复制到处理缓冲区
        if (strcmp(usart_rx_cmd, "test01") == 0)
        {
            usart_rx_cmd_state = TEST01;
        }
        else if (strcmp(usart_rx_cmd, "test02") == 0)
        {
            usart_rx_cmd_state = TEST02;
        }
        else if (strcmp(usart_rx_cmd, "test03") == 0)
        {
            usart_rx_cmd_state = TEST03;
        }
        else
        {
            usart_rx_cmd_state = CMD_NUMB; // 未识别的命令
        }

        switch (usart_rx_cmd_state)
        {
        case TEST01:
            task_measure = 1;
            task_sweep = 0;
            task_fault = 0;
            task_none = 0;
            break;

        case TEST02:
            task_measure = 0;
            task_sweep = 1;
            task_fault = 0;
            task_none = 0;
            break;

        case TEST03:
            task_measure = 0;
            task_sweep = 0;
            task_fault = 1;
            task_none = 0;
            break;

        default:
            task_none = 1;
            task_measure = 0;
            task_sweep = 0;
            task_fault = 0;
            break;
        }
        memset(usart_rx_buffer, 0, sizeof(usart_rx_buffer));
        usart_rx_buffer_index = 0;
        HAL_UART_Receive_IT(&huart1, &usart_rx_buffer[usart_rx_buffer_index], 1);
    }
}