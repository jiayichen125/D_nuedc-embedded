#ifndef DDS_H
#define DDS_H
#include "system.h" 

typedef struct
{   
    uint16_t type;
    uint16_t ch;
    uint32_t freq;
    uint16_t phase;
} WaveFormConfig_t;
void waveset(uint32_t Freq, uint16_t type, uint16_t ch);
#endif // DDS_H