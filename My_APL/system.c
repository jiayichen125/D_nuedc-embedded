#include "system.h"
void System_Init(void)
{
    // 在这里进行系统初始化，例如时钟配置、GPIO初始化等
    // 这只是一个示例，具体的初始化代码需要根据你的硬件平台进行编写
    ad9833_init(); // 初始化AD9833
    My_Usart_Init(); // 初始化USART
}