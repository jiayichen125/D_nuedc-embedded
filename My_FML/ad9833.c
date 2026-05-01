#include "ad9833.h"

void ad9833_write_reg(uint16_t value);

void ad9833_init(void)
{
    HAL_Delay(10);
    ad9833_write_reg(ad9833_Reg_control | ad9833_Reg_control_Reset); // 复位AD9833
    ad9833_write_reg(ad9833_Reg_control_B28 | ad9833_Reg_control_Reset);
    ad9833_set_freq(1000*10.73741824, ad9833_Sine); // 如果参数改成 Hz

    HAL_Delay(10);
}

void ad9833_write_reg(uint16_t value)
{
    // 通过SPI发送数据到AD9833的寄存器
    // 根据寄存器地址和要设置的值构造数据包，并发送给AD9833

    AD9833_SPI_16bits_Write(value);

}
void ad9833_set_waveform(uint16_t type)
{
    // 设置AD9833的输出波形类型
    // 根据输入的类型配置相应的寄存器值，并通过SPI发送给AD9833
    ad9833_write_reg(ad9833_Reg_control_B28 | type);

}
void ad9833_set_freq(uint32_t freq, uint16_t type)
{
    // 设置AD9833的输出频率
    // 根据输入的频率计算相应的寄存器值，并通过SPI发送给AD9833
    uint16_t Fre_L = (freq & 0x3FFF);
    uint16_t Fre_H = ((freq >> 14) & 0x3FFF);
    ad9833_write_reg(ad9833_Reg_control | ad9833_Reg_control_B28); // 设置B28位，允许连续写入频率寄存器
    ad9833_write_reg(ad9833_Reg_freq0 | Fre_L);
    ad9833_write_reg(ad9833_Reg_freq0 | Fre_H);
    ad9833_write_reg(ad9833_Reg_control | ad9833_Reg_control_B28 | type); // 设置波形类型
}
