#include "TMC5160.h"
#include "spi.h"
#include "gpio.h"
#include "usart.h"
#include <stdio.h> // printf 需要用到此头文件

// 内部显示用换算：微步→度 (0.00703125 度/微步)
#define U2D(x)  ((float)(x) * 360.0f / 51200.0f)

// 引入你在 CubeMX 里配置的 SPI6 句柄
extern SPI_HandleTypeDef hspi3;

// 宏定义映射：将片选操作映射到 PC4 
#define pTMC_CSN0(state) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, state)
#define pTMC_DRV_ENN(state) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, state)

// ======================================================
// 模块一：底层 SPI 通信协议层
// ======================================================

// 片选控制
void TMC_SPI1_CS(uint8_t motor, GPIO_PinState state)
{
    if (motor == 0) {
        pTMC_CSN0(state);
    }
}

// SPI 写寄存器
int8_t TMC_SPI1_Write(uint8_t motor, uint8_t reg, int32_t dat)
{   
    uint8_t txDat[5];
    uint8_t rxDat[5];
    
    txDat[4] = (int8_t) (dat);
    txDat[3] = (int8_t) (dat>>8);
    txDat[2] = (int8_t) (dat>>16);
    txDat[1] = (int8_t) (dat>>24);
    txDat[0] = (int8_t) (reg | 0x80); // 最高位置1，表示写操作
    
    TMC_SPI1_CS(motor, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, txDat, rxDat, 5, 100); 
    TMC_SPI1_CS(motor, GPIO_PIN_SET);   
    
    return rxDat[0];            
}

// SPI 读寄存器
int8_t TMC_SPI1_Read(uint8_t motor, uint8_t reg, uint32_t *dat)
{   
    volatile uint8_t i; 
    uint8_t txDat[5];
    uint8_t rxDat[5];

    txDat[4] = 0;
    txDat[3] = 0;
    txDat[2] = 0;
    txDat[1] = 0;
    txDat[0] = (reg & 0x7F); // 最高位置0，表示读操作
    
    // 第一次通信：发送读地址指令
    TMC_SPI1_CS(motor, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, txDat, rxDat, 5, 100); 
    TMC_SPI1_CS(motor, GPIO_PIN_SET);
    
    // 第二次通信：真正将数据读出
    TMC_SPI1_CS(motor, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, txDat, rxDat, 5, 100); 
    TMC_SPI1_CS(motor, GPIO_PIN_SET);   
    
    // 必须强转为 uint32_t，否则 uint8_t 移位前被提升为 int，bit7=1 时结果变负
    *dat = ((uint32_t)rxDat[1] << 24)
         | ((uint32_t)rxDat[2] << 16)
         | ((uint32_t)rxDat[3] <<  8)
         |  (uint32_t)rxDat[4];
    
    return rxDat[0];            
}
// 动态切换静音模式
void TMC_Enable_StealthChop(uint8_t enable)
{
    if (enable) {
        TMC_SPI1_Write(0, TMC5160_GCONF, 0x00000004); 
    } else {
        TMC_SPI1_Write(0, TMC5160_GCONF, 0x00000000); 
    }
}

// ======================================================
// 模块二：硬件初始化与核心 API 层
// ======================================================
//------------------------------------------------------
// TMC5160 基础硬件初始化 (修复起步参数版)
//------------------------------------------------------
void TMC_Init(void)
{
    pTMC_DRV_ENN(GPIO_PIN_SET);                                 
    HAL_Delay(10);      

    TMC_SPI1_Write(0, TMC5160_GSTAT, 0x00000007);
    
    // 关闭静音模式，强制使用 SpreadCycle (为了让 StallGuard 生效)
    TMC_Enable_StealthChop(0); 

    // 降低电流到 8 mA，方便调试观察 StallGuard 负载变化
    TMC_SPI1_Write(0, TMC5160_IHOLD_IRUN, 0x00070802);

    // 其他关键参数配置
    TMC_SPI1_Write(0, TMC5160_CHOPCONF, 0x000000C3); // bit16(VHIGHFS)必须为0，否则VHIGH=0时始终进入全步模式，StallGuard失效       
    TMC_SPI1_Write(0, TMC5160_PWMCONF,  0xC40C261E);    
    
    // TCOOLTHRS: StallGuard 激活条件为 TSTEP <= TCOOLTHRS
    // 手捏减速时 TSTEP 变大，若 TCOOLTHRS 太小(如0xFF=255)，电机稍微减速就退出检测区间
    // 设置极大值 0xFFFFF，保证从全速减速到几乎停止 StallGuard 始终有效
    TMC_SPI1_Write(0, TMC5160_TCOOLTH, 0x000FFFFF);  
    // COOLCONF: SGT=10 (更灵敏，原30太高导致负载变化时SG_RESULT压缩范围窄)
    TMC_SPI1_Write(0, TMC5160_COOLCONF, 0x000A0000); 

    //  补回起步关键参数 
    TMC_SPI1_Write(0, TMC5160_VSTART,   5);      // 起步初速度
    TMC_SPI1_Write(0, TMC5160_A1,       5000);   // 第一段加速度
    TMC_SPI1_Write(0, TMC5160_V1,       0);      // 阈值设为0，直接切入AMAX
    
    // AMAX/DMAX 设为 200000：从0加速到150000 usteps/s 仅需 0.75s（在800ms保护窗内完成）
    // 若仍设5000，需要30秒才能到位，速度模式5s测试期间电机始终在加速段，距离偏少
    TMC_SPI1_Write(0, TMC5160_AMAX,     200000);   
    TMC_SPI1_Write(0, TMC5160_DMAX,     200000);    
    TMC_SPI1_Write(0, TMC5160_D1,       5000);   // 位置模式必须设置，否则到达目标位置前无法正常减速停止
    TMC_SPI1_Write(0, TMC5160_VSTOP,    10);     
    TMC_SPI1_Write(0, TMC5160_TZEROWAIT, 1000);          
    
    TMC_SPI1_Write(0, TMC5160_VMAX, 0);                 
    
    pTMC_DRV_ENN(GPIO_PIN_RESET);                                   
}

// 进入速度控制模式并设定目标运行速度
void TMC_Run_Velocity(int32_t velocity)
{
    // 速度绝对值计算，并进行软件限幅保护
    uint32_t v_abs = (velocity >= 0) ? (uint32_t)velocity : (uint32_t)(-velocity);
    
    // 硬件限幅：VMAX 寄存器 23 位，超过 TMC5160_VMAX_LIMIT 相应幕携步难以检测
    if (v_abs > TMC5160_VMAX_LIMIT) {
        v_abs = TMC5160_VMAX_LIMIT;
    }
    
    TMC_SPI1_Write(0, TMC5160_VMAX, (int32_t)v_abs);
    
    if (velocity > 0) {
        TMC_SPI1_Write(0, TMC5160_RAMPMODE, 1);
    } else if (velocity < 0) {
        TMC_SPI1_Write(0, TMC5160_RAMPMODE, 2);
    } else {
        TMC_SPI1_Write(0, TMC5160_VMAX, 0); 
    }
}

// 读取当前实际速度 VACTUAL (24位有符号，微步/s)
// 正值=正方向，负值=反方向，0=静止
int32_t TMC_Get_Velocity(void)
{
    uint32_t vactual = 0;
    TMC_SPI1_Read(0, TMC5160_VACTUAL, &vactual);
    // VACTUAL 是 24 位有符号数，手动符号扩展到 32 位
    if (vactual & 0x00800000) {
        vactual |= 0xFF000000;
    }
    return (int32_t)vactual;
}

// 读取当前实际位置 XACTUAL (32位有符号，微步单位)
int32_t TMC_Get_Position(void)
{
    uint32_t xactual = 0;
    TMC_SPI1_Read(0, TMC5160_XACTUAL, &xactual);
    return (int32_t)xactual;
}

// 运动到绝对位置 (RAMPMODE=0)
void TMC_Move_Absolute(int32_t position)
{
    // 必须先设置 VMAX，否则之前 TMC_Stop() 将 VMAX 置 0，斗坡发生器不会动
    TMC_SPI1_Write(0, TMC5160_VMAX, (int32_t)TMC5160_POSITION_VMAX);
    TMC_SPI1_Write(0, TMC5160_RAMPMODE, 0); // 切换到位置模式
    TMC_SPI1_Write(0, TMC5160_XTARGET, position); // 写入目标坐标，立即开始运动
}

// 相对当前实际位置运动 delta 步 (正数正向，负数反向)
void TMC_Move_Relative(int32_t delta)
{
    int32_t current_pos = TMC_Get_Position();
    TMC_Move_Absolute(current_pos + delta);
}

// 执行平滑制动停止（速度模式和位置模式均适用）
void TMC_Stop(void)
{
    // 速度模式：将目标速度置 0，山坡发生器按 DMAX 自动减速
    TMC_SPI1_Write(0, TMC5160_VMAX, 0);
    // 位置模式：将 XTARGET 拉回 XACTUAL，防止山坡发生器继续向原目标驱动
    int32_t pos = TMC_Get_Position();
    TMC_SPI1_Write(0, TMC5160_XTARGET, pos);
}

// 获取电机当前机械负载 (StallGuard2 数值越小=负载越大)
uint16_t TMC_Get_Mechanical_Load(void)
{
    uint32_t drv_status = 0;
    
    int8_t status = TMC_SPI1_Read(0, TMC5160_DRV_STATUS, &drv_status);
    
    uint16_t sg_result = drv_status & 0x3FF; // 取低10位作为 SG_RESULT
    
    return sg_result;
}

// 位置模式等待运动完成，同时实时检测堵转
// target           : 目标位置 (微步)
// timeout_ms       : 整个运动最大允许时间 (ms)
// stall_threshold  : SG_RESULT 低于此値则判定堵转
// 返回: 0=正常到达, 1=堵转, 2=超时
uint8_t TMC_MoveAndWait(int32_t target, uint32_t timeout_ms, uint16_t stall_threshold)
{
    uint32_t start = HAL_GetTick();

    // 起步保护期：等待电机开始运动（速度 > 0）或最多等 800ms，防止短距离运动在延迟期内完成
    uint32_t boot_start = HAL_GetTick();
    while (HAL_GetTick() - boot_start < 800) {
        if (TMC_Get_Velocity() != 0) break;
        HAL_Delay(10);
    }
    // 再等 200ms 让 StallGuard 稳定
    HAL_Delay(200);
    // 进入循环，持续监测位置、速度和负载，直到到达目标、发生堵转或超时
    uint8_t stall_cnt = 0; // 连续堵转计数，防止 SG 单次偶发读值误判提前停车
    while (HAL_GetTick() - start < timeout_ms)
    {
        int32_t  pos  = TMC_Get_Position();
        int32_t  vel  = TMC_Get_Velocity();
        uint16_t load = TMC_Get_Mechanical_Load();

        // 到达判断：速度归零且坐标在目标 ±1.8° (±256 usteps) 范围内
        if (vel == 0 && pos >= target - 256 && pos <= target + 256) {
            return 0;
        }

        // 堵转判断：仅当 |速度| > 20000 usteps/s (140°/s) 时检测（正转和反转都生效）
        // 需连续 3 次 load < threshold 才判定堵转，避免 SG 单次抖动误判提前停车
        if (vel > 20000 || vel < -20000) {
            if (load < stall_threshold) {
                stall_cnt++;
                if (stall_cnt >= 3) {
                    TMC_Stop();
                    return 1;
                }
            } else {
                stall_cnt = 0;
            }
        } else {
            stall_cnt = 0; // 低速阶段重置计数，防止跨速度段累积误判
        }

        HAL_Delay(100);
    }

    TMC_Stop();
    return 2;
}

// ======================================================
// 模块三：业务逻辑层
// ======================================================
void TMC_Loop(void)
{

    // 目前由 UART_HMI 的中断/轮询解析函数直接驱动电机
    // 此处可以放置实时保护逻辑、状态上报逻辑等
    HAL_Delay(50); 
}
