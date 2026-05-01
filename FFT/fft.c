#include "fft.h"
#include "HMI.h"
#include "Switch.h"
#include <math.h>
#include <string.h>

extern uint16_t ADC_Us[1024];
extern uint16_t ADC_U0[1024];
extern uint16_t ADC_Ui[1024];
extern uint16_t ADC_AD8703[1024];
extern uint8_t Acquire_All_ADC_Samples_Blocking(uint32_t timeout_ms);

#define FFT_LEN 1024
#define ADC_LEN 1024
#define SWEEP_MAX_POINTS 256
#define ADC_VREF 3.3f
#define ADC_FULL_SCALE 65535.0f

uint8_t ifftFlag = 0;
int BaseIdx = 0;
float fs = 20000.0f;
float FFT_Freq = 0;
float FFT_Ampl1 = 0;
float FFT_Ampl2 = 0;
float DC = 0;
float FFT_mag_max = 0;
uint32_t FFT_mag_max_index = 0;

float FFT_Output[FFT_LEN];
float FFT_Input[FFT_LEN * 2];
float FFT_mag[FFT_LEN];
float IFFT_Output[FFT_LEN];

uint8_t EnableWindow = 1;
float Window_OutputBuffer[ADC_LEN];
static float window_power_correction = 1.0f;
static float sweep_ampl_buffer[SWEEP_MAX_POINTS];

__weak void AD9833_SetFrequency_Hz(float freq_hz)
{
    (void)freq_hz;
}

__weak float AD8703_ConvertVoltageToAmplitude(float detector_voltage, float freq_hz)
{
    (void)freq_hz;
    return detector_voltage;
}

static float Get_ADC_Mean(uint16_t *buffer)
{
    uint32_t sum = 0;

    for (uint16_t i = 0; i < ADC_LEN; i++)
    {
        sum += buffer[i];
    }

    return (float)sum / (float)ADC_LEN;
}

static float ADC_Code_To_Voltage(float adc_code)
{
    return adc_code * ADC_VREF / ADC_FULL_SCALE;
}

static uint8_t Normalize_To_Wave_Point(float value, float min_value, float max_value)
{
    float normalized;

    if (max_value <= min_value)
    {
        return 128;
    }

    normalized = (value - min_value) * 255.0f / (max_value - min_value);

    if (normalized < 0.0f)
    {
        normalized = 0.0f;
    }
    else if (normalized > 255.0f)
    {
        normalized = 255.0f;
    }

    return (uint8_t)(normalized + 0.5f);
}

static float Measure_FFT_Amplitude_From_U0(void)
{
    float amplitude = 0.0f;

    FFT_Process(ADC_U0, &amplitude);
    return amplitude;
}

static float Measure_DC_Level_From_U0(void)
{
    return ADC_Code_To_Voltage(Get_ADC_Mean(ADC_U0));
}

static float Measure_Detector_Amplitude_From_AD8703(float freq_hz)
{
    float detector_voltage = ADC_Code_To_Voltage(Get_ADC_Mean(ADC_AD8703));
    return AD8703_ConvertVoltageToAmplitude(detector_voltage, freq_hz);
}

void showdata(float *buffer, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++)
    {
        printf("%.3f\n", buffer[i]);
    }
}

void window(void)
{
    if (EnableWindow)
    {
        for (uint16_t i = 0; i < ADC_LEN; i++)
        {
            float tempCos = cosf(2.0f * PI * i / (ADC_LEN - 1U));
            Window_OutputBuffer[i] = 0.5f * (1.0f - tempCos);
        }
        window_power_correction = 1.5f;
    }
    else
    {
        for (uint16_t i = 0; i < ADC_LEN; i++)
        {
            Window_OutputBuffer[i] = 1.0f;
        }
        window_power_correction = 1.0f;
    }
}

void FFT_Process(uint16_t *ADC_Buffer, float *FFT_Ampl)
{
    uint32_t adc_sum = 0;

    memset(FFT_Input, 0, sizeof(FFT_Input));
    memset(FFT_mag, 0, sizeof(FFT_mag));
    memset(FFT_Output, 0, sizeof(FFT_Output));

    for (uint16_t i = 0; i < ADC_LEN; i++)
    {
        adc_sum += ADC_Buffer[i];
    }

    DC = adc_sum / (float)ADC_LEN;
    window();

    for (uint16_t i = 0; i < ADC_LEN; i++)
    {
        FFT_Input[i * 2U] = ((float)ADC_Buffer[i] - DC) * Window_OutputBuffer[i];
        FFT_Input[i * 2U + 1U] = 0.0f;
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_Input, 0, 1);
    arm_cmplx_mag_f32(FFT_Input, FFT_mag, FFT_LEN);

    for (uint16_t i = 0; i < FFT_LEN; i++)
    {
        if (i == 0U)
        {
            FFT_mag[i] = FFT_mag[i] / FFT_LEN * window_power_correction;
        }
        else
        {
            FFT_mag[i] = FFT_mag[i] * 2.0f / FFT_LEN * window_power_correction;
        }
    }

    Process_FFT_mag(FFT_mag, &FFT_mag_max, &FFT_mag_max_index);
    ADC_FFT_Get_Wave_Mes(FFT_mag_max_index, fs, FFT_Ampl, &FFT_Freq, 2);
}

void Calculate_Input_Impedance(int Rs)
{
    float Ri;

    FFT_Process(ADC_Ui, &FFT_Ampl1);
    FFT_Process(ADC_Us, &FFT_Ampl2);
    Ri = (float)Rs * FFT_Ampl1 / (FFT_Ampl2 - FFT_Ampl1);
    HMI_send_float("x0", Ri);
}

void Calculate_Output_Impedance(int RL)
{
    float R0;

    FFT_Process(ADC_U0, &FFT_Ampl1);
    Relay_Off();
    HAL_Delay(10);

    if (Acquire_All_ADC_Samples_Blocking(200U) == 1U)
    {
        FFT_Process(ADC_U0, &FFT_Ampl2);
        R0 = (float)RL * (FFT_Ampl2 - FFT_Ampl1) / FFT_Ampl1;
        HMI_send_float("x1", R0);
    }

    Relay_On();
}

void Calculate_Gain(void)
{
    float Au;

    FFT_Process(ADC_U0, &FFT_Ampl1);
    FFT_Process(ADC_Ui, &FFT_Ampl2);
    Au = -FFT_Ampl1 / FFT_Ampl2;
    HMI_send_float("x2", Au);
}

float Measure_Output_Amplitude(float current_freq_hz, float detector_switch_freq_hz)
{
    if (current_freq_hz <= 0.0f)
    {
        return Measure_DC_Level_From_U0();
    }

    if (current_freq_hz < detector_switch_freq_hz)
    {
        return Measure_FFT_Amplitude_From_U0();
    }

    return Measure_Detector_Amplitude_From_AD8703(current_freq_hz);
}

void Sweep_Amplitude_Response(char *wave_name,
                              int ch,
                              float start_freq_hz,
                              float stop_freq_hz,
                              uint16_t points,
                              float detector_switch_freq_hz,
                              uint32_t settle_ms)
{
    float log_start;
    float log_stop;
    float min_amplitude = 0.0f;
    float max_amplitude = 0.0f;
    uint16_t actual_points;

    if ((wave_name == NULL) || (points == 0U) || (stop_freq_hz < start_freq_hz))
    {
        return;
    }

    actual_points = points;
    if (actual_points > SWEEP_MAX_POINTS)
    {
        actual_points = SWEEP_MAX_POINTS;
    }

    if (detector_switch_freq_hz <= 0.0f)
    {
        detector_switch_freq_hz = 5000.0f;
    }

    if (detector_switch_freq_hz > 8000.0f)
    {
        detector_switch_freq_hz = 8000.0f;
    }

    log_start = log10f((start_freq_hz < 1.0f) ? 1.0f : start_freq_hz);
    log_stop = log10f(stop_freq_hz);

    Relay_On();
    HMI_Wave_Clear(wave_name, ch);

    for (uint16_t i = 0; i < actual_points; i++)
    {
        float freq_hz;

        if (i == 0U)
        {
            freq_hz = 0.0f;
        }
        else if (actual_points > 2U)
        {
            float position = (float)(i - 1U) / (float)(actual_points - 2U);
            freq_hz = powf(10.0f, log_start + (log_stop - log_start) * position);
        }
        else
        {
            freq_hz = (start_freq_hz < 1.0f) ? 1.0f : start_freq_hz;
        }

        AD9833_SetFrequency_Hz(freq_hz);
        HAL_Delay(settle_ms);

        if (Acquire_All_ADC_Samples_Blocking(200U) == 1U)
        {
            sweep_ampl_buffer[i] = Measure_Output_Amplitude(freq_hz, detector_switch_freq_hz);
        }
        else
        {
            sweep_ampl_buffer[i] = 0.0f;
        }

        if (i == 0U)
        {
            min_amplitude = sweep_ampl_buffer[i];
            max_amplitude = sweep_ampl_buffer[i];
        }
        else
        {
            if (sweep_ampl_buffer[i] < min_amplitude)
            {
                min_amplitude = sweep_ampl_buffer[i];
            }
            if (sweep_ampl_buffer[i] > max_amplitude)
            {
                max_amplitude = sweep_ampl_buffer[i];
            }
        }
    }

    for (uint16_t i = 0; i < actual_points; i++)
    {
        uint8_t wave_point = Normalize_To_Wave_Point(sweep_ampl_buffer[i], min_amplitude, max_amplitude);
        HMI_Wave(wave_name, ch, wave_point);
    }
}

void Sweep_Amplitude_Response_0Hz_To_12M5(char *wave_name,
                                          int ch,
                                          uint16_t points,
                                          float detector_switch_freq_hz,
                                          uint32_t settle_ms)
{
    Sweep_Amplitude_Response(wave_name,
                             ch,
                             1.0f,
                             12500000.0f,
                             points,
                             detector_switch_freq_hz,
                             settle_ms);
}

void Process_FFT_mag(float *FFT_mag, float *FFT_mag_max, uint32_t *FFT_mag_max_index)
{
    arm_max_f32(FFT_mag, FFT_LEN / 2U, FFT_mag_max, FFT_mag_max_index);
    FFT_Freq = (float)(*FFT_mag_max_index) * fs / (float)FFT_LEN;
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

void ADC_FFT_Get_Wave_Mes(uint32_t FFT_mag_max_index, float fs, float *FFT_Ampl, float *Freq, int correctNum)
{
    int i;
    float DatePower1 = 0.0f;
    float DatePower2 = 0.0f;
    float f;

    if (FFT_mag_max_index < (uint32_t)correctNum)
    {
        *FFT_Ampl = FFT_mag[FFT_mag_max_index];
        *Freq = (float)FFT_mag_max_index * fs / FFT_LEN;
        return;
    }

    if (FFT_mag_max_index > (FFT_LEN / 2U - 1U - (uint32_t)correctNum))
    {
        *FFT_Ampl = FFT_mag[FFT_mag_max_index];
        *Freq = (float)FFT_mag_max_index * fs / FFT_LEN;
        return;
    }

    for (i = -correctNum; i <= correctNum; i++)
    {
        DatePower1 += (FFT_mag_max_index + i) * FFT_mag[FFT_mag_max_index + i] * FFT_mag[FFT_mag_max_index + i];
        DatePower2 += FFT_mag[FFT_mag_max_index + i] * FFT_mag[FFT_mag_max_index + i];
    }

    f = DatePower1 / DatePower2;
    *Freq = f * fs / FFT_LEN;
    *FFT_Ampl = sqrtf(DatePower2);
}
