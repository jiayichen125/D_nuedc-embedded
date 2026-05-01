#include "dds.h"

void waveset(uint32_t Freq, uint16_t type, uint16_t ch)
{
    ad9833_set_freq_ch(Freq, type, (uint8_t)ch);
}
