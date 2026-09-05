/*
 * SPDX-FileCopyrightText: 2026 FryingCat <3551901875@qq.com>
 * SPDX-License-Identifier: MIT
 */

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include "foc_alg.h"
#include "foc_drv.h"
#include "mt6835.h"
#include "adc.h"
extern ADC_HandleTypeDef hadc1;
extern mt6835_t *mt6835;
void stm32_set_pwm_A(float Ua)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)(Ua*A_PWM_Period));
}

void stm32_set_pwm_B(float Ub)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)(Ub*B_PWM_Period));
}

void stm32_set_pwm_C(float Uc)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)(Uc*C_PWM_Period));
}

uint32_t stm32_update_Ia_raw(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
}

uint32_t stm32_update_Ib_raw(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

uint32_t stm32_update_Ic_raw(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
}

float stm32_cal_Ia(uint32_t raw,uint32_t offset)
{
    return (float)((int32_t)raw-(int32_t)(offset))*I_ADC_CONV; 
}

float stm32_cal_Ib(uint32_t raw,uint32_t offset)
{
    return (float)((int32_t)raw-(int32_t)(offset))*I_ADC_CONV; 
}

float stm32_cal_Ic(uint32_t raw,uint32_t offset)
{
    return (float)((int32_t)raw-(int32_t)(offset))*I_ADC_CONV; 
}

uint32_t stm32_update_angle_raw(void)
{
    return mt6835_get_raw_angle(mt6835, MT6835_READ_ANGLE_METHOD_BURST);
}

float stm32_cal_angle(uint32_t raw)
{
    return raw * (PI * 2.0f) / MT6835_ANGLE_RESOLUTION; // 0~2PI
}

void stm32_delayms(uint32_t ms)
{
    HAL_Delay(ms);
}

// float stm32_update_dt(Time_t* time)
// {
//     time->PastTime = time->ThisTime;
//     time->ThisTime =__HAL_TIM_GET_COUNTER(&htim3);
//     if(time->ThisTime > time->PastTime)
//     {
//         time->dt = (float)(time->ThisTime - time->PastTime)/(170000000.0f/300.0f);
//     }
//     else
//     {
//         time->dt = (float)(65535 - time->PastTime + time->ThisTime)/(170000000.0f/300.0f);
//     }
//     // if(time->dt == 0)
//     // {
//     //     time->dt = 0.0001f;
//     // }
// 	return  time->dt ;
// }

#define TIM3_FREQ_DIV (170000000.0f / 300.0f)  // 原分母，仅作为计算基准
#define DT_SCALE_FACTOR (1.0f / TIM3_FREQ_DIV) // 预计算倒数（乘法替代除法，更快）
#define TIM16_MAX_VAL   65536U                 // 16位计数器最大值（溢出后补1，修正原代码bug）

float stm32_update_dt(Time_t* time)//优化后的dt计算函数
{
    // 2. 保存上一次计数值，读取当前计数值（减少寄存器操作）
    const uint16_t past = time->PastTime;
    const uint16_t curr = __HAL_TIM_GET_COUNTER(&htim3);
    time->PastTime = curr; // 提前更新，减少内存访问
    time->ThisTime = curr;

    // 3. 无符号减法自动处理溢出（消除if-else分支，避免流水线停顿）
    const uint16_t diff = curr - past; // 16位无符号减法：溢出时自动计算正确差值

    // 4. 浮点运算优化：乘法替代除法，仅一次浮点运算
    const float dt = (float)diff * DT_SCALE_FACTOR;

    // 5. （可选）零值保护（若需保留原逻辑，简化判断）
    time->dt = (dt < 1e-9f) ? 0.000001f : dt;

    return time->dt;
}

void foc_init(Motor_HandleTypeDef *motor)
{
    // 初始化FOC算法相关参数和结构体
    memset(motor, 0, sizeof(Motor_HandleTypeDef));

    // 初始化驱动接口函数指针
    motor->MotorDrv.Set_PWM_A = stm32_set_pwm_A;      // 设置PWM函数指针
    motor->MotorDrv.Set_PWM_B = stm32_set_pwm_B;      // 设置PWM函数指针
    motor->MotorDrv.Set_PWM_C = stm32_set_pwm_C;      // 设置PWM函数指针

    motor->MotorDrv.Update_Ia_raw = stm32_update_Ia_raw;       // 获取IA电流函数指针
    motor->MotorDrv.Update_Ib_raw = stm32_update_Ib_raw;       // 获取IB电流函数指针
    motor->MotorDrv.Update_Ic_raw = stm32_update_Ic_raw;       // 获取IC电流函数指针
    
    motor->MotorDrv.Cal_Ia = stm32_cal_Ia;      // 电流转换函数指针
    motor->MotorDrv.Cal_Ib = stm32_cal_Ib;      // 电流转换函数指针
    motor->MotorDrv.Cal_Ic = stm32_cal_Ic;      // 电流转换函数指针

    motor->MotorDrv.Delayms = stm32_delayms;     // 延时函数指针
    motor->MotorDrv.Update_dt = stm32_update_dt;       // 获取时间差函数指针

    motor->MotorDrv.Update_Angle_raw = stm32_update_angle_raw;    // 获取角度函数指针
    motor->MotorDrv.Cal_Angle = stm32_cal_angle;   // 角度转换函数指针

    motor->motor_number = 1; // 设置电机编号

    // 初始化时间管理
    motor->time.ThisTime = 0.0f;
    motor->time.PastTime = 0.0f;
    motor->time.dt = 0.01f;

    //初始化电机配置
    motor->MotorConfig.Pole_pairs = 14; // 设置电机极对数
    motor->MotorConfig.DIR = 1 ;        // 设置电机转向
    motor->MotorConfig.PHASE = 3;      // 设置电机接线相序
    motor->MotorConfig.Mode_Sampling = 0X110 ;//AB相双电阻采样
    motor->MotorConfig.IMAX = 20.0f;   // 设置电流限幅
    motor->MotorConfig.UMAX = 24.0f;   // 设置电压限幅
    motor->MotorConfig.Ls = 0.001f;    // 设置定子电感
    motor->MotorConfig.Rs = 0.5f;      // 设置定子电阻
    motor->MotorConfig.angle_el_zero = 3.13263f; // 设置角度零点
    motor->MotorConfig.Kt = 0.806325918;           //设置转矩系数

    // 初始化PID参数(要用哪个初始化哪个)
    motor->MotorAlg.position_pid.KP = 20.3f;
    motor->MotorAlg.position_pid.KI = 0.0f;
    motor->MotorAlg.position_pid.KD = 0.0f;
    motor->MotorAlg.position_pid.integral_max = 100.0f;
    motor->MotorAlg.position_pid.integral_min = -100.0f;
    motor->MotorAlg.position_pid.output_max = 100.0f;
    motor->MotorAlg.position_pid.output_min = -100.0f;
    motor->MotorAlg.position_pid.integral_limit = 10.0f;
    motor->MotorAlg.position_pid.output_limit = 100.0f;

    motor->MotorAlg.velocity_pid.KP = 0.1f;
    motor->MotorAlg.velocity_pid.KI = 0.5f;
    motor->MotorAlg.velocity_pid.KD = 0.0f;
    motor->MotorAlg.velocity_pid.integral_max = 100.0f;
    motor->MotorAlg.velocity_pid.integral_min = -100.0f;
    motor->MotorAlg.velocity_pid.output_max = 100.0f;
    motor->MotorAlg.velocity_pid.output_min = -100.0f;
    motor->MotorAlg.velocity_pid.integral_limit = 10.0f;
    motor->MotorAlg.velocity_pid.output_limit = 100.0f;

    motor->MotorAlg.id_pid.KP = 0.058760f;
    motor->MotorAlg.id_pid.KI = 57.6113014f;
    motor->MotorAlg.id_pid.KD = 0.000f;
    motor->MotorAlg.id_pid.integral_max = motor->MotorConfig.UMAX/2;
    motor->MotorAlg.id_pid.integral_min = -motor->MotorConfig.UMAX/2;
    motor->MotorAlg.id_pid.output_max = motor->MotorConfig.UMAX/2;
    motor->MotorAlg.id_pid.output_min = -motor->MotorConfig.UMAX/2;
    motor->MotorAlg.id_pid.integral_limit = motor->MotorConfig.UMAX/2;
    motor->MotorAlg.id_pid.output_limit = motor->MotorConfig.UMAX/2;

    motor->MotorAlg.iq_pid.KP = 0.058760f;
    motor->MotorAlg.iq_pid.KI = 57.6113014f;
    motor->MotorAlg.iq_pid.KD = 0.000f;
    motor->MotorAlg.iq_pid.integral_max = motor->MotorConfig.UMAX/2;
    motor->MotorAlg.iq_pid.integral_min = -motor->MotorConfig.UMAX/2;
    motor->MotorAlg.iq_pid.output_max = motor->MotorConfig.UMAX/2;
    motor->MotorAlg.iq_pid.output_min = -motor->MotorConfig.UMAX/2;
    motor->MotorAlg.iq_pid.integral_limit = motor->MotorConfig.UMAX/2;
    motor->MotorAlg.iq_pid.output_limit = motor->MotorConfig.UMAX/2;

    motor->MotorAlg.mixed_pid.KP = 0.1f;
    motor->MotorAlg.mixed_pid.KI = 0.01f;
    motor->MotorAlg.mixed_pid.KD = 0.0f;
    motor->MotorAlg.mixed_pid.integral_limit = 10.0f;
    motor->MotorAlg.mixed_pid.output_limit = 100.0f;

    motor->MotorData.Velocity_LPF.last_output = 0.0f;
    motor->MotorData.Velocity_LPF.alpha = 0.5f; // 速度滤波系数
    motor->MotorData.IA_offset_raw = 0XFFFF;
    motor->MotorData.IB_offset_raw = 0XFFFF;
    motor->MotorData.IC_offset_raw = 0XFFFF;
    motor->MotorData.angle_all = 0.0f;

    motor->MotorData.Calibrate_2DIR_block__velocity_target = 0.03f;
    motor->MotorData.Calibrate_2DIR_nonblock__velocity_target = 10.0f;
    motor->MotorData.Calibrate_2DIR_nonblock__time_init = 0.5f;
    motor->MotorData.Calibrate_2DIR_nonblock__time_prep = 1.0f;
    motor->MotorData.Calibrate_2DIR_nonblock__time_process = 2.0f;

    motor->MotorData.Calibrate_pole_pairs_block__angle_step = 0.01f;
    motor->MotorData.Calibrate_pole_pairs_nonblock__velocity_target = 3.0f;
    motor->MotorData.Calibrate_pole_pairs_nonblock__time_init = 0.5f;
    motor->MotorData.Calibrate_pole_pairs_nonblock__time_prep = 1.0f;
    motor->MotorData.Calibrate_pole_pairs_nonblock__time_process = 5.0f;

    motor->MotorData.Calibrate_PHASE_nonblock__Ts = 1.0f;
    motor->MotorData.Calibrate_PHASE_nonblock__Duty = 0.05f;
    motor->MotorData.Calibrate_PHASE_nonblock__state = 1;

    motor->MotorData.Calibrate_angle_el_zero_sensor_block__sample_per = 20;
    motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__sample_per = 50;
    motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__state = 1;

    motor->MotorData.Calibrate_Ioffset_nonblock__sample_total = 1000;
}


