#include "dds_apl.h"

WaveFormConfig_t WaveFormConfig;

void dds_process(void)
{
    // 在这里处理DDS相关的逻辑，例如根据WaveFormConfig的配置设置AD9833的寄存器值
    // 这只是一个示例，具体的处理逻辑需要根据你的应用需求进行编写
    WaveFormConfig.ch=ad9833_CH0;
    WaveFormConfig.freq=1000;
    WaveFormConfig.type=ad9833_Sine;
    waveset(WaveFormConfig.freq, WaveFormConfig.type, WaveFormConfig.ch);
}
