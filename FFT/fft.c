#include "fft.h"
#include "HMI.h"
#include <string.h>

/*外部引用*/
extern uint16_t ADC_Us[ADC_SIZE] = {0};             // 存放Us电压
extern uint16_t ADC_U0[ADC_SIZE] = {0};             // 存放U0电压
extern uint16_t ADC_Ui[ADC_SIZE] = {0};             // 存放Ui电压
extern uint16_t ADC_9833[ADC_SIZE] = {0};           // 存放AD9833电压

/* 变量 */
#define FFT_LEN 1024
#define ADC_LEN 1024

uint8_t ifftFlag = 0;
int BaseIdx = 0;     // 基波下标
float fs = 20000.0f; // 采样率
float FFT_Freq = 0;  // FFT计算得到频率
float FFT_Ampl1 = 0; // FFT计算得到的幅值
float FFT_Ampl2 = 0;
float DC = 0;            // 直流偏置
float FFT_mag_max = {0}; // 幅度谱最大值
uint32_t FFT_mag_max_index = 0;

/* 输入和输出缓冲 */

float FFT_Output[FFT_LEN];
float FFT_Input[FFT_LEN * 2];
float FFT_mag[FFT_LEN]; // 幅度谱
float IFFT_Output[FFT_LEN];

uint8_t EnableWindow = 1;
float Window_OutputBuffer[ADC_LEN];
static float window_power_correction = 1.0f;

void showdata(float *buffer, uint16_t n)
{
    for (int i = 0; i < n; i++)
    {
        printf("%.3f\n", buffer[i]);
    }
}

void window(void)
{
    if (EnableWindow)
    {
        for (int i = 0; i < ADC_LEN; i++)
        {
            float tempCos = cosf(2.0f * PI * i / (ADC_LEN - 1));
            Window_OutputBuffer[i] = 0.5f * (1.0f - tempCos);
        }
        /* Hann window attenuates energy, so compensate before reading amplitude. */
        window_power_correction = 1.5f;
    }
    else
    {
        for (int i = 0; i < ADC_LEN; i++)
        {
            Window_OutputBuffer[i] = 1.0f;
        }
        window_power_correction = 1.0f;
    }
}

void FFT_Process(uint16_t *ADC_Buffer, float *FFT_Ampl)
{
    uint32_t adc_sum = 0;
    float *ampl;
    ampl = FFT_Ampl;

    memset(FFT_Input, 0, sizeof(FFT_Input));
    memset(FFT_mag, 0, sizeof(FFT_mag));
    memset(FFT_Output, 0, sizeof(FFT_Output));

    for (int i = 0; i < ADC_LEN; i++)
    {
        adc_sum += ADC_Buffer[i];
    }
    DC = adc_sum / 1024.0f;

    window();

    for (int i = 0; i < ADC_LEN; i++)
    {
        FFT_Input[i * 2] = ((float)ADC_Buffer[i] - DC) * Window_OutputBuffer[i];
        FFT_Input[i * 2 + 1] = 0.0f;
    }

    /* FFT_Input is an interleaved complex buffer: real sample + zero imaginary part. */
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_Input, 0, 1);
    // showdata(FFT_Input, FFT_LEN);
    arm_cmplx_mag_f32(FFT_Input, FFT_mag, FFT_LEN);

    for (uint16_t i = 0; i < FFT_LEN; i++)
    {
        /* Keep DC single-sided; double non-DC bins for the single-sided spectrum. */
        if (i == 0)
            FFT_mag[i] = FFT_mag[i] / FFT_LEN * window_power_correction;
        else
            FFT_mag[i] = FFT_mag[i] * 2.0f / FFT_LEN * window_power_correction;
    }

    Process_FFT_mag(FFT_mag, &FFT_mag_max, &FFT_mag_max_index);
    ADC_FFT_Get_Wave_Mes(FFT_mag_max_index, fs, &ampl, &FFT_Freq, 2);
}

/**
 计算输入阻抗 Ri=Rs*UI/(US-UI)
 Rs=2kΩ
 */
void Calculate_Input_Impedance(int Rs)
{
    float Ri;
    FFT_Process(ADC_Ui,&FFT_Ampl1);
    FFT_Process(ADC_Us,&FFT_Ampl2);
    // 计算输入阻抗
    Ri = (float)Rs * FFT_Ampl1 / (FFT_Ampl2 - FFT_Ampl1);
    HMI_send_float("x0", Ri);
}

/**
 计算输出阻抗 R0=RL*（U∞-U0）/U0
 RL=2kΩ
 继电器断开时，U∞；继电器接通时，U0
 继电器
 */


 /**
 计算增益 Au=-U0/Ui
 继电器接通时，U0
 */

/*fft caculate */
// 从频谱中提取信号，找到主频，计算信号频率和幅度。

void Process_FFT_mag(float *FFT_mag, float *FFT_mag_max, uint32_t *FFT_mag_max_index)
{

    // 找幅度谱前一半数据，找到最大值和索引
    arm_max_f32(FFT_mag, FFT_LEN / 2, FFT_mag_max, FFT_mag_max_index);

    // 求频率：最大值结果*采样率/FFT长度
    FFT_Freq = (float)(*FFT_mag_max_index) * fs / (float)FFT_LEN;

    // 求幅值：最大值结果索引*2/FFT长度 前面已经进行过归一处理了，所以这里不需要再除以FFT_LEN了/*2
    FFT_Ampl = *FFT_mag_max;
}

void Find_BaseIndex(void)
{
    float max_val = 0.0f;
    BaseIdx = 0;

    for (int i = 2; i < FFT_LEN / 2; i++)
    {
        if (FFT_mag[i] > max_val)
        {
            max_val = FFT_mag[i];
            BaseIdx = i;
        }
    }
}

/*输入参数为FFT计算后的结果，输出矫正后的频率和幅度

FFT_mag_max_index				FFT结果中峰值的位置
fs				采样频率
FFT_Ampl	    矫正后的幅值
Freq[0]			矫正后的频率
correctNum		矫正的点数，一般取2即可，确保峰值左右的correctNum内没有其他信号
FFT_mag		FFT结果的幅值数组
FFT_mag_max_index				FFT结果中峰值的位置
fs				采样频率
FFT_Ampl	    矫正后的幅值
Freq[0]			矫正后的频率
correctNum		矫正的点数，一般取2即可，确保峰值左右的correctNum内没有其他信号
FFT_mag		FFT结果的幅值数组
*/

void ADC_FFT_Get_Wave_Mes(uint32_t FFT_mag_max_index, float fs, float *FFT_Ampl, float *Freq, int correctNum)
{
    int i;
    float DatePower1 = 0.0f;
    float DatePower2 = 0.0f;
    float f;

    for (i = -correctNum; i <= correctNum; i++)
    {
        /* Use local spectral energy centroid to refine the peak-bin frequency estimate. */
        DatePower1 += (FFT_mag_max_index + i) * FFT_mag[FFT_mag_max_index + i] * FFT_mag[FFT_mag_max_index + i];
        DatePower2 += FFT_mag[FFT_mag_max_index + i] * FFT_mag[FFT_mag_max_index + i];
    }

    f = DatePower1 / DatePower2;
    // Freq[0] = f * fs / FFT_LEN;
    *FFT_Ampl = sqrtf(DatePower2); // 对邻域内的能量（幅值的平方和）开根号，恢复有效值 (RMS) k=1, 去掉2倍, 直接出电压
    // HMI_send_float("x0", *FFT_Ampl);
    // HMI_send_float("x1", Freq[0]);
}
