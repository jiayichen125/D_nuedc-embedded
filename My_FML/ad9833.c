#include "ad9833.h"

void ad9833_write_reg(uint16_t value);

void ad9833_init(void)
{
    HAL_Delay(10);
    ad9833_write_reg(ad9833_Reg_control | ad9833_Reg_control_Reset); // 复位AD9833
    HAL_Delay(10);
}


void ad9833_write_reg(uint16_t value)
{
    // 通过SPI发送数据到AD9833的寄存器
    // 根据寄存器地址和要设置的值构造数据包，并发送给AD9833
    AD9833_SPI_CS_L;
    AD9833_SPI_16bits_Write(value);
    AD9833_SPI_CS_H;
}
void ad9833_set_waveform(uint16_t type)
{
    // 设置AD9833的输出波形类型
    // 根据输入的类型配置相应的寄存器值，并通过SPI发送给AD9833
}
void ad9833_set_freq(uint32_t freq)
{
    // 设置AD9833的输出频率
    // 根据输入的频率计算相应的寄存器值，并通过SPI发送给AD9833
}
