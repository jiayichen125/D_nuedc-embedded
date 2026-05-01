#include "gpio.h"

/*继电器闭合*/
void Relay_On(void)
{
    HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);
}

/*继电器断开*/
void Relay_Off(void)
{
    HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);
}
