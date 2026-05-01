#ifndef ad9833_H
#define ad9833_H
#include "system.h" 


// AD9833寄存器
#define ad9833_Reg_control (0<<14)
#define ad9833_Reg_freq0 (1<<14)
#define ad9833_Reg_freq1 (2<<14)
#define ad9833_Reg_phase0 (6<<13)
#define ad9833_Reg_phase1 (7<<13)
// AD9833控制位
#define ad9833_Reg_control_Reset (1<<8)
#define ad9833_Reg_control_B28 (1<<13)
#define ad9833_Reg_control_HLB (1<<12)
#define ad9833_Reg_control_FSELECT (1<<11)
#define ad9833_Reg_control_PSELECT (1<<10)
#define ad9833_Reg_control_SLEEP1 (1<<7)
#define ad9833_Reg_control_SLEEP12 (1<<6)
#define ad9833_Reg_control_OPBITEN (1<<5)
#define ad9833_Reg_control_DIV2 (1<<3)
#define ad9833_Reg_control_MODE (1<<1)
//控制字
#define ad9833_CH 1
//频率控制字 计算公式：Freq = MCLK / 2^28 * Freq_control_word 
//所以 Freq_control_word = Freq * 2^28 / MCLK = Freq * 10.73741824
#define ad9833_Freq 1000
//波形类型
#define ad9833_Sine ((0<<5)|(0<<1)|(0<<3))
#define ad9833_Triangle ((0<<5)|(1<<1)|(0<<3))
#define ad9833_Square ((1<<5)|(0<<1)|(1<<3))
#define ad9833_Square_Div2 ((1<<5)|(0<<1)|(0<<3))


void ad9833_init(void);
void ad9833_set_waveform(uint16_t type);
void ad9833_set_freq(uint32_t freq, uint16_t type);

#endif // ad9833_H