#include "dds.h"
void waveset(uint32_t Freq, uint16_t type)
{
    ad9833_set_freq(Freq, type);
    // 根据输入的频率、波形类型和通道配置相应的寄存器值，并通过SPI发送给AD9833
}