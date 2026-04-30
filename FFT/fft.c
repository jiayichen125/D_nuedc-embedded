#include "fft.h"
#include "HMI.h"
#include <string.h>

extern uint16_t ADC_Buffer[1024];
extern TIM_HandleTypeDef htim3;// 定时器句柄

/* 变量 */
#define FFT_LEN 1024
#define ADC_LEN 1024
#define TIM3_CLK_HZ  20000000UL  /* 240MHz / (PSC+1=12) = 20MHz */

uint8_t ifftFlag = 0;
int BaseIdx = 0;         // 基波下标
int wave_type;           // 波形类别 1是正弦 2是方波 3是三角波
float fs = 100000.0f;    // 采样率
float FFT_Freq = 0;      // FFT计算得到频率
float FFT_Ampl = 0;      // FFT计算得到的幅值
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

/* ---------------------------------------------------------------
 * 自适应采样率
 * 采样率：20kHz / 200kHz / 2MHz
 * 带宽（±20%）防止边界频率混叠
 * --------------------------------------------------------------- */
static float select_fs(float freq)
{
    static float cur = 100000.0f;

    /* Add hysteresis so the sampling rate does not bounce near thresholds. */
    if (cur == 20000.0f && freq > 1200.0f)
        cur = 200000.0f;
    else if (cur == 200000.0f && freq < 800.0f)
        cur = 20000.0f;
    else if (cur == 200000.0f && freq > 12000.0f)
        cur = 2000000.0f;
    else if (cur == 2000000.0f && freq < 8000.0f)
        cur = 200000.0f;

    return cur;
}

/* 直接修改 TIM3->ARR，不停止定时器，下一帧 DMA 即生效
 * 注意：2MHz 档（ARR=9）要求 ADC 总转换时间 < 500ns，請確認 ADC 內核時鐘 */
static void apply_fs(float new_fs)
{
    /* Update ARR only; the timer keeps running and the new period takes effect next cycle. */
    uint32_t arr = (uint32_t)((float)TIM3_CLK_HZ / new_fs + 0.5f) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    fs = new_fs;
}

static int ClassifyWaveform(void)
{
    float sum_abs = 0.0f;
    float sum_sq = 0.0f;
    uint32_t peak_count = 0;
    uint16_t max_v = 0;
    uint16_t min_v = 65535;

    for (uint32_t i = 0; i < ADC_LEN; i++)
    {
        if (ADC_Buffer[i] > max_v)
            max_v = ADC_Buffer[i];
        if (ADC_Buffer[i] < min_v)
            min_v = ADC_Buffer[i];
    }

    {
        float vpp = (float)(max_v - min_v);
        float offset;
        float threshold;

        /* Ignore very small signals; shape classification is unstable near the noise floor. */
        if (vpp < 655.0f)
            return 0;

        offset = (float)min_v + vpp * 0.5f;
        threshold = vpp * 0.10f;

        for (uint32_t i = 0; i < ADC_LEN; i++)
        {
            float sample = (float)ADC_Buffer[i];
            float val = sample - offset;

            sum_abs += fabsf(val);
            sum_sq += val * val;

            /* Count how long the waveform stays near either peak. */
            if (sample >= (float)max_v - threshold ||
                sample <= (float)min_v + threshold)
            {
                peak_count++;
            }
        }
    }

    {
        float v_rms = sqrtf(sum_sq / (float)ADC_LEN);
        float v_avg = sum_abs / (float)ADC_LEN;
        float k_f;
        float r_peak;

        if (v_avg < 1e-6f)
            return 0;

        k_f = v_rms / v_avg;
        r_peak = (float)peak_count / (float)ADC_LEN;

        /* Square waves spend more time near peaks; triangle waves have a larger form factor. */
        if (r_peak > 0.80f && k_f < 1.05f)
            return 3;
        if (r_peak < 0.25f && k_f > 1.13f)
            return 2;
    }

    return 1;
}

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

void FFT_Process(void)
{
    uint32_t adc_sum = 0;

    memset(FFT_Input, 0, sizeof(FFT_Input));
    memset(FFT_mag, 0, sizeof(FFT_mag));
    memset(FFT_Output, 0, sizeof(FFT_Output));

    for (int i = 0; i < ADC_LEN; i++)
    {
        adc_sum += ADC_Buffer[i];
    }

    DC = adc_sum / 1024.0f;
    HMI_send_float("x_dc", DC * 3.3f / 65536.0f);

    window();

    for (int i = 0; i < ADC_LEN; i++)
    {
        FFT_Input[i * 2] = ((float)ADC_Buffer[i] - DC) * Window_OutputBuffer[i];
        FFT_Input[i * 2 + 1] = 0.0f;
    }

    /* FFT_Input is an interleaved complex buffer: real sample + zero imaginary part. */
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_Input, 0, 1);
    showdata(FFT_Input, FFT_LEN);
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
    ADC_FFT_Get_Wave_Mes(FFT_mag_max_index, fs, &FFT_Ampl, &FFT_Freq, 2);

    Find_BaseIndex();
    wave_type_detect();

    if (FFT_Freq > 50.0f)
    {
        float new_fs = select_fs(FFT_Freq);
        if (new_fs != fs)
            apply_fs(new_fs);
    }
}

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

void IFFT_Process(void)
{
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_Input, 1, 1);

    for (int i = 0; i < FFT_LEN; i++)
    {
        IFFT_Output[i] = FFT_Input[2 * i];
    }
}

void Find_BaseIndex(void)
{
    float max_val = 0.0f;
    BaseIdx = 0;

    /* Skip DC and low bins near DC, then search only the positive-frequency half. */
    for (int i = 2; i < FFT_LEN / 2; i++)
    {
        if (FFT_mag[i] > max_val)
        {
            max_val = FFT_mag[i];
            BaseIdx = i;
        }
    }
}

void wave_type_detect(void)
{
    int stat_type = 0;

    if (BaseIdx < 171)
    {
        /* Low-frequency region: 3rd harmonic is still reliable, so use the FFT ratio first. */
        float ratio = FFT_mag[3 * BaseIdx] / FFT_mag[BaseIdx];
        if (ratio < 0.05f)
            wave_type = 1;
        else if (ratio < 0.20f)
            wave_type = 2;
        else
            wave_type = 3;
    }
    else if (BaseIdx >= 205)
    {

        /* 高频段（基波 > 20kHz）：谐波超出奈奎斯特，完全依赖统计法 */
        stat_type = ClassifyWaveform();
        wave_type = (stat_type != 0) ? stat_type : 1;
    }
    else
    {
        float ratio;

        /* Transition region: combine FFT and time-domain results, prefer time-domain on conflict. */
        stat_type = ClassifyWaveform();
        ratio = FFT_mag[3 * BaseIdx] / FFT_mag[BaseIdx];

        if (ratio < 0.05f)
            wave_type = 1;
        else if (ratio < 0.20f)
            wave_type = 2;
        else
            wave_type = 3;

        if (stat_type == 0 || stat_type == wave_type)
        {
            wave_type = wave_type; /* 一致或统计法失效，信任 FFT */
        }
        else
        {
            wave_type = stat_type; /* 不一致，信任统计法 */
        }
    }

    switch (wave_type)
    {
    case 1:
        HMI_send_string("t0", "sine");
        break;
    case 2:
        HMI_send_string("t0", "triangle");
        break;
    case 3:
        HMI_send_string("t0", "square");
        break;
    default:
        HMI_send_string("t0", "unknown");
        break;
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
    Freq[0] = f * fs / FFT_LEN;
    *FFT_Ampl = sqrtf(DatePower2); // 对邻域内的能量（幅值的平方和）开根号，恢复有效值 (RMS) k=1, 去掉2倍, 直接出电压
    // HMI_send_float("x0", *FFT_Ampl);
    // HMI_send_float("x1", Freq[0]);
}
