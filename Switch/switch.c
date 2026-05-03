#include "gpio.h"

/*继电器闭合*/
void Relay2_On(void)
{
    HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);
}

/*继电器断开*/
void Relay2_Off(void)
{
    HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);
}

/*继电器闭合*/
void Relay1_On(void)
{
    HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_RESET);
}

/*继电器断开*/
void Relay1_Off(void)
{
    HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET);
}

