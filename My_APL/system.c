#include "system.h"
void System_Init(void)
{
    ad9833_init(); // 初始化AD9833
    My_Usart_Init(); // 初始化USART
}