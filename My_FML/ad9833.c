#include "ad9833.h"

static uint16_t ad9833_control_word = (ad9833_Reg_control_B28 | ad9833_Sine);

static uint16_t ad9833_freq_reg(uint8_t ch)
{
    return (ch == ad9833_CH1) ? ad9833_Reg_freq1 : ad9833_Reg_freq0;
}

static uint16_t ad9833_fselect_bit(uint8_t ch)
{
    return (ch == ad9833_CH1) ? ad9833_Reg_control_FSELECT : 0;
}

void ad9833_init(void)
{
    AD9833_SPI_CS_H;
    AD9833_SPI_SCK_H;
    HAL_Delay(10);

    ad9833_control_word = ad9833_Reg_control_B28 | ad9833_Sine;
    ad9833_write_reg(ad9833_control_word | ad9833_Reg_control_Reset);
    ad9833_set_freq_ch(ad9833_Freq, ad9833_Sine, ad9833_CH0);
}

void ad9833_write_reg(uint16_t value)
{
    AD9833_SPI_CS_L;
    AD9833_SPI_16bits_Write(value);
    AD9833_SPI_CS_H;
}

uint32_t ad9833_freq_to_word(uint32_t freq_hz)
{
    uint64_t word = ((uint64_t)freq_hz * AD9833_FREQ_WORD_SCALE + (AD9833_MCLK_HZ / 2U)) / AD9833_MCLK_HZ;

    if (word > AD9833_FREQ_WORD_MAX)
    {
        word = AD9833_FREQ_WORD_MAX;
    }

    return (uint32_t)word;
}

void ad9833_set_waveform(uint16_t type)
{
    ad9833_control_word &= (uint16_t)(ad9833_Reg_control_B28 | ad9833_Reg_control_FSELECT | ad9833_Reg_control_PSELECT);
    ad9833_control_word |= type;
    ad9833_write_reg(ad9833_control_word);
}

void ad9833_set_freq(uint32_t freq, uint16_t type)
{
    ad9833_set_freq_ch(freq, type, ad9833_CH0);
}

void ad9833_set_freq_ch(uint32_t freq, uint16_t type, uint8_t ch)
{
    ad9833_set_freq_word(ad9833_freq_to_word(freq), type, ch);
}

void ad9833_set_freq_word(uint32_t freq_word, uint16_t type, uint8_t ch)
{
    uint16_t freq_reg = ad9833_freq_reg(ch);
    uint16_t fselect = ad9833_fselect_bit(ch);

    freq_word &= AD9833_FREQ_WORD_MAX;

    uint16_t fre_l = (uint16_t)(freq_word & AD9833_FREQ_DATA_MASK);
    uint16_t fre_h = (uint16_t)((freq_word >> AD9833_FREQ_DATA_BITS) & AD9833_FREQ_DATA_MASK);

    ad9833_control_word = ad9833_Reg_control_B28 | fselect | type;

    ad9833_write_reg(ad9833_control_word | ad9833_Reg_control_Reset);
    ad9833_write_reg(freq_reg | fre_l);
    ad9833_write_reg(freq_reg | fre_h);
    ad9833_write_reg(ad9833_control_word);
}

void ad9833_sweep_start(ad9833_sweep_t *sweep, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms, uint16_t type)
{
    ad9833_sweep_start_ch(sweep, start_hz, stop_hz, step_hz, dwell_ms, type, ad9833_CH0);
}

void ad9833_sweep_start_ch(ad9833_sweep_t *sweep, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms, uint16_t type, uint8_t ch)
{
    if (sweep == 0)
    {
        return;
    }

    if (step_hz == 0U)
    {
        step_hz = 1U;
    }

    sweep->start_hz = start_hz;
    sweep->stop_hz = stop_hz;
    sweep->step_hz = step_hz;
    sweep->current_hz = start_hz;
    sweep->dwell_ms = dwell_ms;
    sweep->last_tick = HAL_GetTick();
    sweep->type = type;
    sweep->ch = ch;
    sweep->enable = 1U;

    ad9833_set_freq_ch(start_hz, type, ch);
}

uint8_t ad9833_sweep_process(ad9833_sweep_t *sweep)
{
    uint32_t now;
    uint32_t next_hz;

    if ((sweep == 0) || (sweep->enable == 0U))
    {
        return 0U;
    }

    now = HAL_GetTick();
    if ((now - sweep->last_tick) < sweep->dwell_ms)
    {
        return 0U;
    }

    sweep->last_tick = now;

    if (sweep->start_hz <= sweep->stop_hz)
    {
        next_hz = sweep->current_hz + sweep->step_hz;
        if (next_hz > sweep->stop_hz)
        {
            next_hz = sweep->start_hz;
        }
    }
    else
    {
        if (sweep->current_hz <= (sweep->stop_hz + sweep->step_hz))
        {
            next_hz = sweep->start_hz;
        }
        else
        {
            next_hz = sweep->current_hz - sweep->step_hz;
        }
    }

    sweep->current_hz = next_hz;
    ad9833_set_freq_ch(next_hz, sweep->type, sweep->ch);

    return 1U;
}

void ad9833_sweep_stop(ad9833_sweep_t *sweep)
{
    if (sweep != 0)
    {
        sweep->enable = 0U;
    }
}
