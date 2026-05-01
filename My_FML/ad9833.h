#ifndef ad9833_H
#define ad9833_H
#include "system.h" 

// AD9833寄存器地址定义
#define ad9833_Reg_control (0<<14)
// AD9833控制寄存器位定义
#define ad9833_Reg_control_Reset (1<<8)
//控制字
#define ad9833_CH 1

void ad9833_init(void);
void ad9833_write_reg(uint16_t value);
void ad9833_set_waveform(uint16_t type);
void ad9833_set_freq(uint32_t freq);

#endif // ad9833_H