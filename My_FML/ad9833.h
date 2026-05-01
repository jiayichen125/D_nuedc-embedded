#ifndef ad9833_H
#define ad9833_H

#include "system.h"

// AD9833 registers
#define ad9833_Reg_control (0<<14)
#define ad9833_Reg_freq0 (1<<14)
#define ad9833_Reg_freq1 (2<<14)
#define ad9833_Reg_phase0 (6<<13)
#define ad9833_Reg_phase1 (7<<13)

// AD9833 control bits
#define ad9833_Reg_control_B28 (1<<13)
#define ad9833_Reg_control_HLB (1<<12)
#define ad9833_Reg_control_FSELECT (1<<11)
#define ad9833_Reg_control_PSELECT (1<<10)
#define ad9833_Reg_control_Reset (1<<8)
#define ad9833_Reg_control_SLEEP1 (1<<7)
#define ad9833_Reg_control_SLEEP12 (1<<6)
#define ad9833_Reg_control_OPBITEN (1<<5)
#define ad9833_Reg_control_DIV2 (1<<3)
#define ad9833_Reg_control_MODE (1<<1)

#define ad9833_CH0 0
#define ad9833_CH1 1
#define ad9833_CH ad9833_CH0

#define AD9833_MCLK_HZ 25000000UL
#define AD9833_FREQ_BITS 28
#define AD9833_FREQ_DATA_BITS 14
#define AD9833_FREQ_WORD_SCALE (1ULL << AD9833_FREQ_BITS)
#define AD9833_FREQ_WORD_MAX ((1UL << AD9833_FREQ_BITS) - 1UL)
#define AD9833_FREQ_DATA_MASK ((1U << AD9833_FREQ_DATA_BITS) - 1U)
#define ad9833_Freq 1000

// Waveform control words
#define ad9833_Sine ((0<<5)|(0<<1)|(0<<3))
#define ad9833_Triangle ((0<<5)|(1<<1)|(0<<3))
#define ad9833_Square ((1<<5)|(0<<1)|(1<<3))
#define ad9833_Square_Div2 ((1<<5)|(0<<1)|(0<<3))

typedef struct
{
    uint32_t start_hz;
    uint32_t stop_hz;
    uint32_t step_hz;
    uint32_t current_hz;
    uint32_t dwell_ms;
    uint32_t last_tick;
    uint16_t type;
    uint8_t ch;
    uint8_t enable;
} ad9833_sweep_t;

void ad9833_init(void);
void ad9833_write_reg(uint16_t value);
void ad9833_set_waveform(uint16_t type);
void ad9833_set_freq(uint32_t freq, uint16_t type);
void ad9833_set_freq_ch(uint32_t freq, uint16_t type, uint8_t ch);
void ad9833_set_freq_word(uint32_t freq_word, uint16_t type, uint8_t ch);
uint32_t ad9833_freq_to_word(uint32_t freq_hz);
void ad9833_sweep_start(ad9833_sweep_t *sweep, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms, uint16_t type);
void ad9833_sweep_start_ch(ad9833_sweep_t *sweep, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms, uint16_t type, uint8_t ch);
uint8_t ad9833_sweep_process(ad9833_sweep_t *sweep);
void ad9833_sweep_stop(ad9833_sweep_t *sweep);

#endif // ad9833_H
