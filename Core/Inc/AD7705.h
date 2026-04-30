#ifndef AD7705_H
#define AD7705_H

#include "main.h"
// AD7705.h 里加一行
#define ADC_BIPOLAR  1   // 1=双极模式，0=单极模式
// ─── 引脚定义 ───────────────────────────────
#define AD_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define AD_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define AD_DRDY()     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)

// ─── 传感器参数（LZ-WX2）────────────────────
#define SENSOR_MV_V        1.443f    // 灵敏度 mV/V
#define SENSOR_VCC         5.0f      // 激励电压 V
#define SENSOR_MAX_KG      5.0f      // 满量程 kg

// ─── ADC 参数 ────────────────────────────────
#define ADC_VREF_MV        2500.0f   // 参考电压 mV
#define ADC_GAIN           128.0f    // 增益
#define ADC_FULLSCALE      65535.0f  // 16bit

// ─── 滤波参数 ────────────────────────────────
#define FILTER_SIZE        16        // 滑动平均窗口
#define TARE_SAMPLES       20        // 去皮采样次数

// ─── 函数声明 ────────────────────────────────
void     AD7705_WriteByte(uint8_t data);
uint8_t  AD7705_ReadReg(uint8_t read_cmd);
uint8_t  AD7705_Init(void);
uint16_t AD7705_ReadData(void);
void     AD7705_Tare(void);
float    AD7705_RawToKg(uint16_t raw);
float    AD7705_ReadWeightKg(void);
void     AD7705_TensionTest(void);

#endif