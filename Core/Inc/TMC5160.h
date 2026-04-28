#ifndef __TMC5160_H
#define __TMC5160_H

#include "main.h"
//通用配置寄存器(0X00…0X0F)
#define TMC5160_GCONF       0x00// 全局配置寄存器，控制芯片的全局功能和模式
#define TMC5160_GSTAT       0x01// 全局状态寄存器，读写都可以，写入1会清除对应的状态位

//速度相关的驱动特性控制寄存器 (0X10…0X1F)
#define TMC5160_IHOLD_IRUN  0x10// 电流配置寄存器，控制电机的运行电流和静止电流
#define TMC5160_TCOOLTH     0x14// StallGuard 生效的速度阈值

//斜坡发生器运动寄存器集 (0X20…0X2D)
#define TMC5160_RAMPMODE    0x20// 运动模式寄存器，控制电机的运动模式（位置模式、速度模式等）
#define TMC5160_XACTUAL     0x21// 实际位置寄存器，读出当前电机的位置
#define TMC5160_VACTUAL     0x22// 实际速度寄存器，只读，24位有符号值 (unit: usteps/s)
#define TMC5160_VSTART      0x23// 启动速度寄存器，控制电机启动时的初始速度
#define TMC5160_A1          0x24// 第一段加速度寄存器，控制电机在加速阶段的加速度
#define TMC5160_V1          0x25// 第一段速度寄存器，控制电机在加速阶段的速度阈值
#define TMC5160_AMAX        0x26// 最大加速度寄存器，控制电机的最大加速度
#define TMC5160_VMAX        0x27// 最大速度寄存器，控制电机的最大速度
#define TMC5160_DMAX        0x28// 最大减速度寄存器，控制电机的最大减速度
#define TMC5160_D1          0x2A// 第一段减速度寄存器，控制电机在减速阶段的减速度
#define TMC5160_VSTOP       0x2B// 停止速度寄存器，控制电机停止时的速度阈值
#define TMC5160_TZEROWAIT   0x2C// 换向等待时间寄存器，控制电机换向时的等待时间
#define TMC5160_XTARGET     0x2D// 目标位置寄存器，写入目标位置后电机会自动运动到该位置

#define TMC5160_IOIN        0x04// 输入引脚状态寄存器 (bits[31:24]=VERSION, TMC5160应为0x30)

// ======================================================
// 运动控制限幅定义
// TMC5160 VMAX 寄存器为 23 位unsgined，硬件最大 8,388,607
// 软件限幅到 500,000：超过此速度电机易失步，且震动加剧无法检测
// ======================================================
#define TMC5160_VMAX_LIMIT      500000UL  // 速度上限，超过易失步
#define TMC5160_POSITION_VMAX  100000UL  // 位置模式默认运动速度

// 微步 ↔ 角度换算（1.8°步距电机 + 256细分 = 51200微步/圈）
#define TMC5160_USTEPS_PER_REV  51200UL
#define TMC5160_DEG_TO_USTEP(d) ((int32_t)((float)(d) * (float)TMC5160_USTEPS_PER_REV / 360.0f))
#define TMC5160_USTEP_TO_DEG(u) ((float)(u) * 360.0f / (float)TMC5160_USTEPS_PER_REV)

//微步控制寄存器(0X60…0X6B)
#define TMC5160_CHOPCONF    0x6C// 斩波器配置寄存器，控制微步细分、斩波频率等
#define TMC5160_COOLCONF    0x6D// StallGuard2 阈值和控制寄存器
#define TMC5160_DRV_STATUS  0x6F// 驱动状态寄存器，读取电机的状态信息，如过热、过载等
//驱动寄存器组(0X6C…0X7F)
#define TMC5160_PWMCONF     0x70// PWM 配置寄存器，控制 PWM 模式和频率

// ======================================================
// 硬件引脚宏定义
// PC5 是我们在 CubeMX 里配置的驱动使能引脚 (低电平使能)
// ======================================================
#define pTMC_DRV_ENN(state) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, state)

// 3. 电流与模式动态调节 (Power & Mode)


void TMC_Enable_StealthChop(uint8_t enable); // 动态切换静音模式

// 4. 运动控制 API
void    TMC_Run_Velocity(int32_t velocity);  // 速度模式：以指定速度持续运行(正负决定方向)
void    TMC_Move_Absolute(int32_t position); // 位置模式：运动到绝对坐标
void    TMC_Move_Relative(int32_t delta);    // 位置模式：相对当前位置运动指定步数
void    TMC_Stop(void);                      // 平滑制动停止电机

// 位置模式等待运动完成，同时检测堵转
// target: 目标位置, timeout_ms: 超时ms, stall_threshold: 堵转 SG 阈值
// 返回: 0=正常到达, 1=堵转, 2=超时
uint8_t TMC_MoveAndWait(int32_t target, uint32_t timeout_ms, uint16_t stall_threshold);

// 5. 状态读取 API
int32_t TMC_Get_Position(void);  // 读取当前实际位置 (XACTUAL)
int32_t TMC_Get_Velocity(void);  // 读取当前实际速度 (VACTUAL, usteps/s)
uint16_t TMC_Get_Mechanical_Load(void); // 读取 StallGuard 负载值

void  TMC_Init(void);
void  TMC_Loop(void);
#endif /* __TMC5160_H */
