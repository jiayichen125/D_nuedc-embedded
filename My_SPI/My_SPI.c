#include "My_SPI.h"

void AD9833_SPI_16bits_Write(uint16_t data)
{
    uint16_t SendData = data;

    uint8_t i;
    for (i = 0; i < 16; i++)
    {
        if (SendData & 0x8000) // 判断最高位
        {
            AD9833_SPI_SDA_H;
            AD9833_SPI_SCK_L;
            HAL_Delay(1);     // 确保数据稳定
            SendData <<= 1;   // 数据左移一位，准备发送下一位
            AD9833_SPI_SCK_H; // 时钟拉高，数据被AD9833采样
        }
        else
        {
            AD9833_SPI_SDA_L;
            AD9833_SPI_SCK_L;
            HAL_Delay(1);     // 确保数据稳定
            SendData <<= 1;   // 数据左移一位，准备发送下一位
            AD9833_SPI_SCK_H; // 时钟拉高，数据被AD9833采样
        }
    }
    AD9833_SPI_CS_H; // 片选拉高，结束通信
}