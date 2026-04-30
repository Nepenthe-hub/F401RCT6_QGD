#include "spi.h"
#include "AD7705.h"
#include "stdio.h"
#include "string.h"

extern SPI_HandleTypeDef hspi1;

// ─── 内部变量 ────────────────────────────────
static float    g_tare_raw  = 0.0f;
static uint16_t filter_buf[FILTER_SIZE];
static uint8_t  filter_idx  = 0;
static uint8_t  filter_full = 0;

// ─── 理论比例系数 ─────────────────────────────
// 满量程 ADC span = (MV_V × VCC) / (VREF / GAIN) × 65535
// scale = MAX_KG / span_counts
static const float SCALE_KG_PER_COUNT =
    SENSOR_MAX_KG /
    ((SENSOR_MV_V * SENSOR_VCC) / (ADC_VREF_MV / ADC_GAIN) * ADC_FULLSCALE);

// ─── 底层通信 ────────────────────────────────
void AD7705_WriteByte(uint8_t data)
{
    AD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &data, 1, 100);
    AD_CS_HIGH();
}

uint8_t AD7705_ReadReg(uint8_t read_cmd)
{
    uint8_t val = 0;
    AD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &read_cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, &val, 1, 100);
    AD_CS_HIGH();
    return val;
}

// ─── 初始化 ──────────────────────────────────
uint8_t AD7705_Init(void)
{
    // 1. 复位
    uint8_t reset = 0xFF;
    AD_CS_LOW();
    for (int i = 0; i < 4; i++) {
        HAL_SPI_Transmit(&hspi1, &reset, 1, 100);
    }
    AD_CS_HIGH();
    HAL_Delay(10);

    // 2. 时钟寄存器：4.9152MHz, CLKDIV=1, 50Hz
    static const uint8_t clk_cmd[2] = {0x20, 0x0C};
    AD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, clk_cmd, 2, 100);
    AD_CS_HIGH();
    HAL_Delay(50);

    // 3. 验证时钟
    uint8_t clk_val = AD7705_ReadReg(0x28);
    if (clk_val != 0x0C) {
        printf("[ERR] Clock mismatch: 0x%02X\r\n", clk_val);
        return 1;
    }
    printf("[OK]  Clock OK\r\n");

    // 4. Setup：自校准 + Gain=128 + 单极
    static const uint8_t setup_cmd[2] = {0x10, 0x78};
    AD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, setup_cmd, 2, 100);
    AD_CS_HIGH();

    // 5. 等待自校准
    printf("[INFO] Self-cal...\r\n");
    uint32_t t0 = HAL_GetTick();
    while (AD_DRDY() == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 5000) {
            printf("[ERR] Self-cal timeout\r\n");
            return 1;
        }
    }
    printf("[OK]  Self-cal done %lu ms\r\n", HAL_GetTick() - t0);

    return 0;
}

// ─── 读取 ADC 原始值 ─────────────────────────
uint16_t AD7705_ReadData(void)
{
    uint32_t t0 = HAL_GetTick();
    while (AD_DRDY() == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 1000) {
            printf("[WARN] DRDY timeout\r\n");
            return 0xFFFF;
        }
    }
    uint8_t cmd   = 0x38;
    uint8_t rx[2] = {0};
    AD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, rx, 2, 100);
    AD_CS_HIGH();
    return (uint16_t)(rx[0] << 8 | rx[1]);
}

// ─── 滑动平均滤波 ────────────────────────────
static uint16_t AD7705_Filter(uint16_t new_val)
{
    filter_buf[filter_idx] = new_val;
    filter_idx = (filter_idx + 1) % FILTER_SIZE;
    if (filter_idx == 0) filter_full = 1;

    uint32_t sum = 0;
    uint8_t  cnt = filter_full ? FILTER_SIZE : filter_idx;
    for (uint8_t i = 0; i < cnt; i++) sum += filter_buf[i];
    return (uint16_t)(sum / cnt);
}

// ─── 去皮 ────────────────────────────────────
void AD7705_Tare(void)
{
    printf("[TARE] Collecting zero...\r\n");

    uint32_t sum  = 0;
    int      valid = 0;
    for (int i = 0; i < TARE_SAMPLES; i++) {
        uint16_t raw = AD7705_ReadData();
        if (raw != 0xFFFF) {
            sum += raw;
            valid++;
        }
        HAL_Delay(100);
    }

    if (valid == 0) {
        printf("[TARE ERR] No valid samples\r\n");
        return;
    }

    uint16_t tare_raw = (uint16_t)(sum / valid);
    g_tare_raw = (float)tare_raw;

    // 用真实零点预填充滤波缓冲，防止开始阶段数据偏低
    for (int i = 0; i < FILTER_SIZE; i++) filter_buf[i] = tare_raw;
    filter_idx  = 0;
    filter_full = 1;

    printf("[TARE] Zero RAW=%u\r\n", tare_raw);
}

// ─── RAW 转重量 ──────────────────────────────
float AD7705_RawToKg(uint16_t raw)
{
    float delta = (float)raw - g_tare_raw;
    float kg    = delta * SCALE_KG_PER_COUNT;
    return kg < 0.0f ? 0.0f : kg;
}

// ─── 对外接口：读取当前重量 ──────────────────
float AD7705_ReadWeightKg(void)
{
    uint16_t raw = AD7705_ReadData();
    if (raw == 0xFFFF) return -1.0f;
    uint16_t filtered = AD7705_Filter(raw);
    return AD7705_RawToKg(filtered);
}
// ─── 非阻塞读取 ADC 原始值 ──────────────────────────
// DRDY 未就绪时直接返回 0xFFFF，不等待
uint16_t AD7705_TryReadData(void)
{
    if (AD_DRDY() != GPIO_PIN_RESET) {
        return 0xFFFF;
    }

    uint8_t cmd   = 0x38;
    uint8_t rx[2] = {0};
    AD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, rx, 2, 100);
    AD_CS_HIGH();

    return (uint16_t)(rx[0] << 8 | rx[1]);
}

// ─── 非阻塞读取重量（kg）────────────────────────────
// 返回 -1.0f 表示本次 DRDY 未就绪，无新数据
float AD7705_TryReadWeightKg(void)
{
    uint16_t raw = AD7705_TryReadData();
    if (raw == 0xFFFF) return -1.0f;

    uint16_t filtered = AD7705_Filter(raw);
    return AD7705_RawToKg(filtered);
}
//// ─── 测试主函数 ──────────────────────────────
//void AD7705_TensionTest(void)
//{
//    printf("====== RAW DEBUG ======\r\n");

//    // 复位
//    uint8_t reset = 0xFF;
//    AD_CS_LOW();
//    for (int i = 0; i < 4; i++) HAL_SPI_Transmit(&hspi1, &reset, 1, 100);
//    AD_CS_HIGH();
//    HAL_Delay(10);

//    // 时钟
//    static uint8_t clk_cmd[2] = {0x20, 0x0C};
//    AD_CS_LOW();
//    HAL_SPI_Transmit(&hspi1, clk_cmd, 2, 100);
//    AD_CS_HIGH();
//    HAL_Delay(50);

//    // 验证时钟
//    uint8_t clk_val = AD7705_ReadReg(0x28);
//    printf("Clock reg = 0x%02X (expect 0x0C)\r\n", clk_val);

//    // ★ 先用 Gain=1 双极，最宽量程，排除截断问题
//    // 0x10 = 写CH1 Setup
//    // 0x40 = 自校准 + Gain=1 + 双极
//    static uint8_t setup_cmd[2] = {0x10, 0x40};
//    AD_CS_LOW();
//    HAL_SPI_Transmit(&hspi1, setup_cmd, 2, 100);
//    AD_CS_HIGH();

//    // 等自校准
//    printf("Waiting self-cal...\r\n");
//    uint32_t t0 = HAL_GetTick();
//    while (AD_DRDY() == GPIO_PIN_SET) {
//        if (HAL_GetTick() - t0 > 5000) {
//            printf("Self-cal TIMEOUT\r\n");
//            return;
//        }
//    }
//    printf("Self-cal OK %lu ms\r\n", HAL_GetTick() - t0);

//    // 读数
//    uint32_t cnt = 0;
//    while (1) {
//        // 等 DRDY
//        t0 = HAL_GetTick();
//        while (AD_DRDY() == GPIO_PIN_SET) {
//            if (HAL_GetTick() - t0 > 1000) {
//                printf("DRDY timeout\r\n");
//                goto end;
//            }
//        }

//        uint8_t cmd   = 0x38;
//        uint8_t rx[2] = {0};
//        AD_CS_LOW();
//        HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
//        HAL_SPI_Receive(&hspi1, rx, 2, 100);
//        AD_CS_HIGH();

//        uint16_t raw = (uint16_t)(rx[0] << 8 | rx[1]);

//        // 双极解读：0x8000=0V
//        int16_t signed_raw = (int16_t)(raw - 0x8000);
//        // Gain=1 双极：满量程 = VREF = 2500mV
//        float mv = (float)signed_raw / 32768.0f * 2500.0f;

//        printf("[%04lu] RAW=0x%04X (%5u)  Signed=%-6d  Vmv=%.2f\r\n",
//               cnt, raw, raw, signed_raw, mv);

//        cnt++;
//        HAL_Delay(200);
//    }
//    end:;
//}