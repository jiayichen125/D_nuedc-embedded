#include "HMI.h"
#include "usart.h"

// 发送字符串（用于文本显示）
void HMI_send_string(char* obj_name, char* showdata)
{
    // 格式: obj_name.txt="content" + 0xFF 0xFF 0xFF
    printf("%s.txt=\"%s\"\xff\xff\xff", obj_name, showdata);
}

// 发送整数
void HMI_send_number(char* obj_name, int num)
{
    // 格式: obj_name.val=num + 0xFF 0xFF 0xFF
    printf("%s.val=%d\xff\xff\xff", obj_name, num);
}

// 发送浮点数（转换为整数或特定精度）
void HMI_send_float(char* obj_name, float num)
{
    // 格式: obj_name.val=num (乘以1000保留3位小数) + 0xFF 0xFF 0xFF
    printf("%s.val=%d\xff\xff\xff", obj_name, (int)(num * 1000));
}

// 添加波形数据点（单个点）
void HMI_Wave(char* wf_name, int ch, int val)
{
    // 格式: add wf_name,ch,val + 0xFF 0xFF 0xFF
    printf("add %s,%d,%d\xff\xff\xff", wf_name, ch, val);
}

// 快速波形显示（连续数据）
void HMI_Wave_Fast(char* wf_name, int ch, int count, uint8_t* show_data)
{
    int i;
    
    // 第一步：发送命令头
    printf("addt %s,%d,%d\xff\xff\xff", wf_name, ch, count);
    
    // 第二步：延迟等待屏幕准备
    HAL_Delay(10);
    
    // 第三步：发送波形数据（字节流）
    for (i = 0; i < count; i++) {
        // 直接发送字节，不需要转换
        printf("%c", show_data[i]);
    }
    
    // 第四步：发送结束符
    printf("\xff\xff\xff");
    
    HAL_Delay(10);  // 等待屏幕处理
}

// 清空波形
void HMI_Wave_Clear(char* wf_name, int ch)
{
    // 格式: cle wf_name,ch + 0xFF 0xFF 0xFF
    printf("cle %s,%d\xff\xff\xff", wf_name, ch);
}

// 额外功能：设置组件属性
void HMI_set_property(char* obj_name, char* property, int value)
{
    // 格式: obj_name.property=value + 0xFF 0xFF 0xFF
    printf("%s.%s=%d\xff\xff\xff", obj_name, property, value);
}