#include "My_SPI.h"

static void AD9833_SPI_Delay(void)
{
    __NOP();
    __NOP();
}

void AD9833_SPI_16bits_Write(uint16_t data)
{
    uint8_t i;

    AD9833_SPI_SCK_H;

    for (i = 0; i < 16; i++)
    {
        if ((data & 0x8000U) != 0U)
        {
            AD9833_SPI_SDA_H;
        }
        else
        {
            AD9833_SPI_SDA_L;
        }

        AD9833_SPI_Delay();
        AD9833_SPI_SCK_L;
        AD9833_SPI_Delay();
        AD9833_SPI_SCK_H;

        data <<= 1;
    }
}
