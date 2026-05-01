#ifndef __FFT_H__
#define __FFT_H__

#include "arm_const_structs.h"
#include "arm_math.h"
#include "main.h"
#include "usart.h"
#include "HMI.h"
#include <stdint.h>
#include "Switch.h"
#include "ad9833.h"

/* 扫频时 HMI 曲线页面编号，根据实际 Nextion 工程修改 */
#define SWEEP_HMI_PAGE  1


void FFT_Process(uint16_t *ADC_Buffer, float *FFT_Ampl);
void window(void);
void ADC_FFT_Get_Wave_Mes(uint32_t Row, float Fs, float *VPP, float *Freq, int correctNum);
void Find_BaseIndex(void);
void showdata(float *buffer, uint16_t n);
void Process_FFT_mag(float *FFT_mag, float *FFT_mag_max, uint32_t *FFT_mag_max_index,float *FFT_Ampl);

void Calculate_Input_Impedance(int Rs);
void Calculate_Output_Impedance(int RL);
void Calculate_Gain(void);
void Sweep_Gain(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz);


#endif
