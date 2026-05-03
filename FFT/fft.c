#include "fft.h"

/* -----------------------------------------------------------------------
 * 外部引用
 * 这四个数组由 main.c 中的 Split_ADC_Buffers() 填充，
 * 每次 ADC DMA 完成后更新，长度均为 ADC_LEN 点。
 * ----------------------------------------------------------------------- */
extern uint16_t ADC_Us[1024];   // IN16: 串联电阻 Rs 前端电压 (用于 Zi 计算)
extern uint16_t ADC_U0[1024];   // IN5:  放大器输出电压 (用于 Zo、Av 计算)
extern uint16_t ADC_Ui[1024];   // IN17: 串联电阻 Rs 后端电压 = 放大器输入 (用于 Zi、Av 计算)
extern uint16_t ADC_8307[1024]; // IN14: AD8307 对数检波器直流输出 (高频段幅值测量)

/* 外部函数：阻塞等待一次完整的 ADC DMA 帧采集完成
 * 用于需要在函数内部主动触发重采样的场合（如 Zo 测量时切换继电器后）
 * 返回 1=成功, 0=超时 */
extern uint8_t Acquire_All_ADC_Samples_Blocking(uint32_t timeout_ms);

/* -----------------------------------------------------------------------
 * 宏定义
 * ----------------------------------------------------------------------- */
#define FFT_LEN 1024 // FFT 点数，必须是2的幂
#define ADC_LEN 1024 // ADC 采样点数，与 FFT_LEN 保持一致
#define rank 2       // 每个 ADC 的扫描通道数（用于主缓冲区大小计算）

#define HMI_AU_MIN_DB (-80.0f) // hmi映射范围
#define HMI_AU_MAX_DB (80.0f)

#define AD8307_SLOPE_V_PER_DB (0.02575f) // 25 mV/dB，后续实测标定
#define AD8307_INTERCEPT_DBM (-84.27f)  // 典型截距，后续实测标定
#define AD8307_LOAD_OHM (50.0f)        // AD8307 输入等效负载/系统阻抗
#define AD8307_NOISE_FLOOR_VRMS (0.0f)

/* 目标 Ui 幅值。
 * 注意：这里的单位不是伏特，而是 FFT_Process() 输出的 ADC LSB 幅值。
 * 建议先固定 1kHz 输出，观察 FFT_Process(ADC_Ui, &Ui_Ampl) 得到的正常值，
 * 再把这个值填到 UI_TARGET_AMP_LSB。
 */
#define UI_TARGET_AMP_LSB        (2000.0f) 
#define UI_AMP_TOLERANCE         (0.03f) //允許誤差
#define UI_AMP_LOCK_MAX_ITER     (8U) //條幅次數
/* PGA/数字电位器幅值码范围。
 * 你的 ad9833_set_amplitude() 使用 0~255。
 * 最小值建议用 1，避免输出完全为 0 后闭环恢复困难。
 */
#define AD9833_AMP_CODE_MIN      (1U)
#define AD9833_AMP_CODE_MAX      (255U)
#define AD9833_AMP_CODE_INIT     (128U)
/* 幅值码调节方向。
 * 如果测试发现 amp_code 越大，Ui 幅值越大，保持 0。
 * 如果测试发现 amp_code 越大，Ui 幅值越小，改成 1。
 */
#define AD9833_AMP_REVERSE       (0U)

/* -----------------------------------------------------------------------
 * 全局变量
 * ----------------------------------------------------------------------- */
uint8_t ifftFlag = 0;
uint8_t adc_flag = 0;     // 用于重新采集 U∞ 的标志（Zo 测量时使用）
int BaseIdx = 0;          // 基波在 FFT 幅度谱中的下标
volatile uint8_t hmi_val; // 临时加上 用于查看变量值 原出于342行
static float ad8307_noise_watt = 0.0f;

/* 当前 PGA/数字电位器幅值码。
 * 用 static 保存，扫频下一个频点会沿用上一个频点的幅值码，
 * 这样调节更快。
 */
static uint8_t ad9833_amp_code = AD9833_AMP_CODE_INIT;

/* fs: 采样率，决定 FFT 频率分辨率 = fs/FFT_LEN
 * 例：fs=20kHz, FFT_LEN=1024 → 分辨率≈19.5Hz，最高可分析 10kHz 信号
 * 注意：测量高频时需提高 TIM3 触发频率并同步修改此值 */
float fs = 20000.0f;

float FFT_Freq = 0;  // 当前帧 FFT 计算得到的基波频率 (Hz)
float FFT_Ampl1 = 0; // FFT_Process 输出幅值暂存 (第1路)
float FFT_Ampl2 = 0; // FFT_Process 输出幅值暂存 (第2路)
float DC = 0;        // 直流偏置（各采样点均值），FFT 前去除以消除直流分量

float FFT_mag_max = 0;          // 幅度谱峰值（归一化后）
uint32_t FFT_mag_max_index = 0; // 幅度谱峰值所在 bin 下标

/* -----------------------------------------------------------------------
 * FFT 运算缓冲区
 * FFT_Input: 复数交错格式 [实部0, 虚部0, 实部1, 虚部1, ...]，长度2*FFT_LEN
 * FFT_mag:   幅度谱，长度 FFT_LEN，有效信息在前 FFT_LEN/2 个 bin
 * ----------------------------------------------------------------------- */
float FFT_Output[FFT_LEN];
float FFT_Input[FFT_LEN * 2];
float FFT_mag[FFT_LEN];
float IFFT_Output[FFT_LEN];

uint8_t EnableWindow = 1;                    // 1=使能 Flat Top 窗，0=矩形窗
float Window_OutputBuffer[ADC_LEN];          // 窗函数系数缓存
static float window_power_correction = 1.0f; // 窗函数幅值补偿系数（Flat Top≈4.63867）

/* 调试用：通过 printf/串口打印浮点数组，用于 PC 端验证 FFT 结果 */
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
        /* 窗函数增益系数*/
        window_power_correction = 1.55f;
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

/**
 * @brief 对指定 ADC 缓冲区执行 FFT，输出基波幅值
 *
 * @param ADC_Buffer  原始 ADC 采样数组（uint16_t, 长度 ADC_LEN）
 * @param FFT_Ampl    [OUT] 经重心插值矫正后的基波幅值（ADC LSB 为单位的峰值）
 *
 * 处理流程：
 *   1. 求均值 DC，作为直流偏置去除
 *   2. 加 Flat Top 窗（提高单频幅值测量精度）
 *   3. 填充复数输入缓冲（虚部为0）
 *   4. CMSIS arm_cfft_f32 执行 FFT
 *   5. arm_cmplx_mag_f32 计算各 bin 幅度
 *   6. 归一化：直流 bin /N, 其余 bin *2/N（对应单边谱峰值）
 *   7. 乘以窗函数幅值补偿系数（Flat Top≈4.63867）
 *   8. 找最大 bin → 重心插值精化频率和幅值 → 写入 *FFT_Ampl
 *
 * 注意：FFT_Ampl1/FFT_Ampl2 是全局暂存，连续调用时第一次结果
 *       已通过指针写出，第二次 memset 不会覆盖已写出的值。
 */
void FFT_Process(uint16_t *ADC_Buffer, float *FFT_Ampl)
{
    uint32_t adc_sum = 0;
    float *ampl;
    ampl = FFT_Ampl;

    /*清空缓存区*/
    memset(FFT_Input, 0, sizeof(FFT_Input));
    memset(FFT_mag, 0, sizeof(FFT_mag));
    memset(FFT_Output, 0, sizeof(FFT_Output));

    /* 步骤1: 计算均值（直流偏置），后续减去以消除 DC 分量 */
    for (int i = 0; i < ADC_LEN; i++)
    {
        adc_sum += ADC_Buffer[i];
    }
    DC = adc_sum / 1024.0f;

    /* 步骤2: 生成窗函数系数 */
    window();

    /* 步骤3: 填充复数输入，去直流 + 加窗，虚部置0 */
    for (int i = 0; i < ADC_LEN; i++)
    {
        FFT_Input[i * 2] = ((float)ADC_Buffer[i] - DC) * Window_OutputBuffer[i];
        FFT_Input[i * 2 + 1] = 0.0f;
    }

    /* 步骤4: CMSIS FFT，ifftFlag=0 正变换，bitReverseFlag=1 自动位反转 */
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_Input, 0, 1);

    /* 步骤5: 计算各 bin 复数模（幅度谱）*/
    arm_cmplx_mag_f32(FFT_Input, FFT_mag, FFT_LEN);

    /* 步骤6+7: 归一化 + 窗函数功率补偿 */
    for (uint16_t i = 0; i < FFT_LEN; i++)
    {
        if (i == 0)
            FFT_mag[i] = FFT_mag[i] / FFT_LEN * window_power_correction;
        else
            FFT_mag[i] = FFT_mag[i] * 2.0f / FFT_LEN * window_power_correction;
    }

    /* 步骤8: 找峰值 bin，再用重心插值精化频率和幅值 */
    Process_FFT_mag(FFT_mag, &FFT_mag_max, &FFT_mag_max_index, ampl);
    ADC_FFT_Get_Wave_Mes(FFT_mag_max_index, fs, ampl, &FFT_Freq, 2);
}

/**
 * @brief 计算被测放大器输入阻抗 Zi
 *
 * 原理：在信号源和放大器输入端之间串联已知电阻 Rs，
 *       通过 FFT 分别测量 Rs 前（Us）和 Rs 后（Ui）的幅值。
 *       由分压关系：Zi = Rs * Ui / (Us - Ui)
 *
 * 注意：Us 和 Ui 的幅值单位相同（ADC LSB），比值运算中单位自动消除，
 *       因此无需换算为实际电压即可直接计算阻抗（前提是两路前级增益相同）。
 *
 * @param Rs  串联已知电阻阻值 (Ω)
 */
void Calculate_Input_Impedance(int Rs)
{
    float Ri;
    FFT_Process(ADC_Ui, &FFT_Ampl1); // 先算 Ui，结果存 FFT_Ampl1
    FFT_Process(ADC_Us, &FFT_Ampl2); // 再算 Us，结果存 FFT_Ampl2
    Ri = (float)Rs * FFT_Ampl1 / (FFT_Ampl2 - FFT_Ampl1);
    HMI_send_float("x0", Ri);
}


/**
 * @brief 计算被测放大器输出阻抗 Zo
 *
 * 原理：等效输出模型为 Thevenin 源（Voc + Zo 串联）。
 *   - 继电器接通（接负载 RL）: 测 U0（带载输出电压）
 *   - 继电器断开（空载）:      测 U∞（开路输出电压）
 *   - 公式: Zo = RL * (U∞ - U0) / U0
 *
 * 时序：
 *   1. 先在继电器接通状态下（主循环已采好数据）取 U0 → FFT_Ampl1
 *   2. 断开继电器，等待 10ms 信号稳定
 *   3. 主动触发一次新采样（Acquire_All_ADC_Samples_Blocking）
 *   4. 取 U∞ → FFT_Ampl2
 *   5. 计算并发送结果，然后恢复继电器接通状态
 *
 * @param RL  外接负载电阻阻值 (Ω)
 */
void Calculate_Output_Impedance(int RL)
{
    float R0;
    /* 步骤1: 继电器当前接通，此帧数据即为带载测量值 */
    FFT_Process(ADC_U0, &FFT_Ampl1); // FFT_Ampl1 = U0 (带载)

    /* 步骤2: 断开负载，等待信号稳定 */
    Relay1_Off();
    HAL_Delay(10);

    /* 步骤3~4: 重采样，获取空载电压 */
    if (Acquire_All_ADC_Samples_Blocking(200U) == 1U)
    {
        FFT_Process(ADC_U0, &FFT_Ampl2); // FFT_Ampl2 = U∞ (空载)
        R0 = (float)RL * (FFT_Ampl2 - FFT_Ampl1) / FFT_Ampl1;
        HMI_send_float("x1", R0);
    }

    /* 步骤5: 恢复负载接通，保证后续测量环境一致 */
    Relay1_On();
}

/**
 * @brief 计算被测放大器电压增益 Av (dB)
 *
 * 公式：Av = 20 * lg(U0 / Ui)
 * 正值表示放大，负值表示衰减。
 *
 * 注意：此处使用 FFT 幅值比（峰值之比），由于是比值运算，
 *       不需要换算为实际电压，单位一致即可。
 */
void Calculate_Gain(void)
{
    float Au;

    FFT_Process(ADC_U0, &FFT_Ampl1); // FFT_Ampl1 = |U0| 幅值
    FFT_Process(ADC_Ui, &FFT_Ampl2); // FFT_Ampl2 = |Ui| 幅值

    /* Av(dB) = 20*log10(U0/Ui)，保护除零 */
    if (FFT_Ampl2 > 1e-6f)
        Au = 20.0f * log10f(FFT_Ampl1 / FFT_Ampl2);
    else
        Au = 0.0f;

    HMI_send_float("x2", Au);
}

/**
 * @brief 在幅度谱前半段（单边谱）中找峰值 bin
 *
 * 只搜索 [0, FFT_LEN/2) 范围，因为 FFT 输出后半段是前半段的镜像（共轭对称）。
 * 同时更新全局 FFT_Freq（粗略频率）和 FFT_Ampl1（峰值幅度）。
 */
void Process_FFT_mag(float *FFT_mag, float *FFT_mag_max, uint32_t *FFT_mag_max_index, float *FFT_Ampl)
{

    arm_max_f32(FFT_mag, FFT_LEN / 2, FFT_mag_max, FFT_mag_max_index);
    FFT_Freq = (float)(*FFT_mag_max_index) * fs / (float)FFT_LEN;
    *FFT_Ampl = *FFT_mag_max;
}


/**
 * @brief 手动搜索基波 bin（跳过 DC 附近的 bin 0、1）
 *
 * 与 Process_FFT_mag 的区别：从 i=2 开始，避免 DC 泄漏干扰，
 * 适用于低频信号（基波 bin 较小）时 arm_max_f32 可能被 DC 干扰的情况。
 * 结果写入全局 BaseIdx，可供其他函数使用。
 */
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


/**
 * @brief 重心插值法精化 FFT 频率和幅值
 *
 * 背景：FFT 频率分辨率 = fs/N，信号频率不一定恰好落在整数 bin 上，
 *       会造成幅值偏低、频率偏移（栅栏效应）。
 *       重心插值利用峰值 bin 周围 ±correctNum 个 bin 的能量分布，
 *       以功率加权重心估计真实频率，精度可提升约一个数量级。
 *
 * @param FFT_mag_max_index  峰值 bin 下标（由 Process_FFT_mag 得到）
 * @param fs                 采样率 (Hz)
 * @param FFT_Ampl           [OUT] 矫正后幅值（能量加权范数，近似峰值幅度）
 * @param Freq               [OUT] 矫正后频率 (Hz)
 * @param correctNum         参与插值的单侧 bin 数，通常取 2
 */
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

    if (FFT_mag_max_index > (FFT_LEN / 2 - 1U - (uint32_t)correctNum))
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

/**
 * @brief AD8307 底噪修正
 *
 * @param update_noise
 *        1 = 当前 ADC_8307[] 作为无信号底噪，更新 ad8307_noise_watt
 *        0 = 当前 ADC_8307[] 作为正式测量值，扣除底噪后返回 U0_vrms
 *
 * @return 扣底噪后的 U0_vrms。
 *         如果 update_noise=1，返回 0。
 *         如果信号功率低于底噪，返回 AD8307_NOISE_FLOOR_VRMS。
 *
 * 原理：
 *   AD8307 输出电压对应输入功率：
 *
 *       P_dBm = Vout / Slope + Intercept
 *
 *   但正式测量时：
 *
 *       P_total = P_signal + P_noise
 *
 *   所以必须先转成功率 W，再做：
 *
 *       P_signal = P_total - P_noise
 *
 *   不能直接用 dBm 相减，因为 dBm 是对数单位。
 */
float ad8307_noiseclean(uint8_t update_noise)
{
    uint32_t adc_sum = 0;

    for (int i = 0; i < ADC_LEN; i++)
    {
        adc_sum += ADC_8307[i];
    }

    float adc_avg = adc_sum / (float)ADC_LEN;
    float v_adc = adc_avg * 3.3f / 65536.0f;

    float p_dbm = v_adc / AD8307_SLOPE_V_PER_DB + AD8307_INTERCEPT_DBM;
    float p_watt = powf(10.0f, (p_dbm - 30.0f) / 10.0f);

    if (update_noise)
    {
        ad8307_noise_watt = p_watt;
        return 0.0f;
    }

    float p_signal_watt = p_watt - ad8307_noise_watt;

    if (p_signal_watt <= 0.0f)
    {
        return AD8307_NOISE_FLOOR_VRMS;
    }

    return sqrtf(AD8307_LOAD_OHM * p_signal_watt);
}

/**閉環條幅度鎖定函數
 * @brief 根据 ADC_Ui[] 闭环调节 AD9833 模块输出幅值
 *
 * 工作流程：
 *   1. 采集一帧 ADC 数据，更新 ADC_Ui[]
 *   2. 使用 FFT_Process(ADC_Ui, &Ui_Ampl) 得到输入端 Ui 的基波幅值
 *   3. 将 Ui_Ampl 与目标 UI_TARGET_AMP_LSB 比较
 *   4. 如果误差在允许范围内，认为锁定成功
 *   5. 如果 Ui 偏小，就增大 PGA 幅值码
 *      如果 Ui 偏大，就减小 PGA 幅值码
 *   6. 重复若干次，直到幅值接近目标值
 *
 * 注意：
 *   - 反馈点是 ADC_Ui[]，也就是放大器输入端。
 *   - 不要用 ADC_U0 或 AD8307 做这个闭环，否则会把被测放大器的频响补平。
 *   - 这个函数只负责“调源端幅值”，正式测量仍建议在函数返回后重新采一帧。
 *
 * @retval 1 幅值锁定成功
 * @retval 0 幅值未锁定或采样失败
 */
uint8_t AD9833_Lock_Ui_Amplitude(void)
{
    for (uint8_t iter = 0; iter < UI_AMP_LOCK_MAX_ITER; iter++)
    {
    
        if (Acquire_All_ADC_Samples_Blocking(200U) != 1U)
        {
            return 0U;
        }

        /* 用 FFT 提取 Ui 的基波幅值。
         * Ui_Ampl 的单位是 ADC LSB，不是实际电压。
         * 但闭环只关心比例，所以可以直接用。
         */
        float Ui_Ampl = 0.0f;
        FFT_Process(ADC_Ui, &Ui_Ampl);

        /* Ui 太小，说明信号源输出过低、采样异常或通道没接好。
         * 此时继续用 ratio 调节可能导致异常放大。
         */
        if (Ui_Ampl <= 1e-6f)
        {
            return 0U;
        }

        /* 计算目标幅值与当前幅值的比例。
         * ratio > 1：当前 Ui 偏小，需要增大输出
         * ratio < 1：当前 Ui 偏大，需要减小输出
         */
        float ratio = UI_TARGET_AMP_LSB / Ui_Ampl;

        /* 如果已经进入允许误差范围，认为恒幅完成。 */
        if ((ratio > (1.0f - UI_AMP_TOLERANCE)) &&
            (ratio < (1.0f + UI_AMP_TOLERANCE)))
        {
            return 1U;
        }

        /* 根据比例调整 PGA 幅值码。
         * 正向情况：amp_code 越大，输出越大
         * 反向情况：amp_code 越大，输出越小
         先自己測量在ad9833_set_amplitude(128/64)的情況下，Ui_Ampl的值，然後調整AD9833_AMP_REVERSE的值
				实测是amp_code越大输出越大,AD9833_AMP_REVERSE为0
         */
//#if AD9833_AMP_REVERSE
//        float next_code = (float)ad9833_amp_code / ratio;
//#else
        float next_code = (float)ad9833_amp_code * ratio;
//#endif

        /* 限制幅值码范围，避免超过 0~255。 */
        if (next_code < (float)AD9833_AMP_CODE_MIN)
        {
            next_code = (float)AD9833_AMP_CODE_MIN;
        }
        else if (next_code > (float)AD9833_AMP_CODE_MAX)
        {
            next_code = (float)AD9833_AMP_CODE_MAX;
        }

        /* 写入新的 PGA/数字电位器幅值码。 */
        ad9833_amp_code = (uint8_t)next_code;
        ad9833_set_amplitude(ad9833_amp_code);

        /* 等待 PGA 输出和后级滤波/跟随器稳定。 */
        HAL_Delay(10);
    }

    /* 达到最大迭代次数仍未进入误差范围，认为锁定失败。 */
    return 0U;
}



/* -----------------------------------------------------------------------
 * 扫频幅频特性测量
 *
 * 目标：在 start_hz ~ stop_hz 范围内步进扫频，逐点测量增益 Av(dB)，
 *       在 HMI 波形控件上绘制幅频特性曲线。
 *
 * 实现步骤（每个频率点）：
 *   ① ad9833_set_freq_ch(f, ad9833_Sine, ad9833_CH0) 设置当前频率
 *   ② HAL_Delay(5) 等待 AD9833 输出稳定（约 1~2 个信号周期）
 *   ③ Acquire_All_ADC_Samples_Blocking(200) 重新采样一帧
 *   ④ 判断频率范围：
 *      f <8000Hz → FFT 路径：FFT_Process(U0) / FFT_Process(Ui) → Av
 *      f >= 8000Hz → AD8307 路径：读 ADC_8307[] 均值 → 换算 Vrms → Av
 *   ⑤ 将 Av 映射到 HMI 波形控件坐标（0~255），调用 HMI_Wave()
 *   ⑥ f += step_hz，循环直到 f > stop_hz
 *
 * AD8307 换算公式（斜率 25mV/dB，截距 -84dBm，负载 50Ω）：
 *   v_adc  = ADC_8307_avg * 3.3f / 65535.0f
 *   p_dbm  = v_adc / 0.025f + (-84.0f)
 *   Vrms   = sqrtf(50.0f * powf(10.0f, (p_dbm - 30.0f) / 10.0f))
 *
 * 注意事项：
 *   - 扫频前调用 HMI_Wave_Clear() 清除上次曲线
 *   - 扫频前切换 HMI 页面到曲线页（page sweep_page_id）
 *   - 高频段 fs 需要对应提高（修改 TIM3 ARR），或提前完成高频率程序
 *   - 扫频结束后恢复 AD9833 到初始频率
 * ----------------------------------------------------------------------- */

/**
 * @brief 扫频幅频特性测量
 *
 * @param start_hz  起始频率 (Hz)
 * @param stop_hz   终止频率 (Hz)
 * @param step_hz   步进频率 (Hz)
 */
void Sweep_Gain(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz)
{
    /* 关闭信号源输出 */
    HAL_GPIO_WritePin(AD9833_Switch_GPIO_Port, AD9833_Switch_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    if (Acquire_All_ADC_Samples_Blocking(200U) == 1U)
    {
        ad8307_noiseclean(1U);
    }

    /* 恢复信号源输出 */
    HAL_GPIO_WritePin(AD9833_Switch_GPIO_Port, AD9833_Switch_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    /* 第②步~第⑤步：频率循环 */
    for (uint32_t f = start_hz; f <= stop_hz; f += step_hz)
    {
        /* 第②步：设置 AD9833 频率，等待输出稳定 */
        ad9833_set_freq_ch(f, ad9833_Sine, ad9833_CH0);
        HAL_Delay(5);

        /* 闭环调 AD9833/PGA 幅值 */
        AD9833_Lock_Ui_Amplitude();

        /* 第③步：触发一次新采样，若超时则跳过本频率点 */
        if (Acquire_All_ADC_Samples_Blocking(200U) != 1U)
        {
            continue;
        }

        float Au_dB = 0.0f;

        if (f < 8000U)
        {
	
            /* 第④步（低频路径）：FFT 计算 U0 和 Ui 幅值，求增益 */
            float U0_Ampl, Ui_Ampl;
            FFT_Process(ADC_U0, &U0_Ampl);
            FFT_Process(ADC_Ui, &Ui_Ampl);
            if (Ui_Ampl > 1e-6f)
                Au_dB = 20.0f * log10f(U0_Ampl / Ui_Ampl);
            else
                Au_dB = 0.0f;

        }
        else
        {
            /* 第④步（高频路径）：AD8307均值 → U0_vrms
             * ADC_Ui 去直流后 RMS → Ui_vrms，求增益
             * AD8307 参数：斜率 25mV/dB，截距 -84dBm，负载 50Ω */
            uint32_t adc_sum = 0;
            for (int i = 0; i < ADC_LEN; i++)
            {
                adc_sum += ADC_8307[i];
            }
            float adc_avg = adc_sum / (float)ADC_LEN;
            float v_adc = adc_avg * 3.3f / 65536.0f;
            float p_dbm = v_adc / (AD8307_SLOPE_V_PER_DB * 0.001f) + (AD8307_INTERCEPT_DBM);
            float U0_vrms = ad8307_noiseclean(0U);

            /* Ui：去直流后 RMS */
            float ui_sum = 0.0f;
            for (int i = 0; i < ADC_LEN; i++)
                ui_sum += (float)ADC_Ui[i];
            float ui_dc = ui_sum / (float)ADC_LEN;
            float ui_sq = 0.0f;
            for (int i = 0; i < ADC_LEN; i++)
            {
                float v = (float)ADC_Ui[i] - ui_dc;
                ui_sq += v * v;
            }
            float Ui_vrms = sqrtf(ui_sq / (float)ADC_LEN) * 3.3f / 65536.0f;

            if (Ui_vrms > 1e-6f)
                Au_dB = 20.0f * log10f(U0_vrms / Ui_vrms);
            else
                Au_dB = 0.0f;

            if ((U0_vrms > 0.0f) && (Ui_vrms > 1e-6f))
            {
                Au_dB = 20.0f * log10f(U0_vrms / Ui_vrms);
            }
            else
            {
                Au_dB = HMI_AU_MIN_DB; // hmi显示下限
            }
        }

        /* 第⑤步：将 Au_dB 映射到 HMI 坐标（0~255）并发送波形点
         * 当前映射范围：-80dB ~ +80dB → 0 ~ 255，可按实际量程调整 */
        hmi_val = (uint8_t)((Au_dB - HMI_AU_MIN_DB) * 255.0f / (HMI_AU_MAX_DB - HMI_AU_MIN_DB));
        HMI_Wave("s0.id", 0, hmi_val);
    }

    /* 扫频结束后恢复 AD9833 到初始测量频率 */
    ad9833_set_freq_ch(1000, ad9833_Sine, ad9833_CH0);
}

