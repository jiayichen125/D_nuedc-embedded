#include "usart_deal.h"

#define len 128
uint8_t usart_rx_buffer[len] = {0};
uint8_t usart_rx_buffer_index = 0;

void Usart_Send_Computer(UART_HandleTypeDef huart, char *msg)
{
    HAL_UART_Transmit(&huart, (uint8_t *)msg, strlen(msg), 1000);
}

void My_Usart_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &usart_rx_buffer[usart_rx_buffer_index], 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UART_Transmit(&huart1, &usart_rx_buffer[usart_rx_buffer_index], 1, 1000);
        HAL_UART_Receive_IT(&huart1, &usart_rx_buffer[usart_rx_buffer_index], 1);
    }
}
