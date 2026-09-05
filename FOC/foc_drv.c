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
void stm32_set_pwm_a(float ua)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)(ua*A_PWM_Period));
}

void stm32_set_pwm_b(float ub)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)(ub*B_PWM_Period));
}

void stm32_set_pwm_c(float uc)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)(uc*C_PWM_Period));
}

uint32_t stm32_update_ia_raw(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
}

uint32_t stm32_update_ib_raw(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

uint32_t stm32_update_ic_raw(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
}

float stm32_cal_ia(uint32_t raw,uint32_t offset)
{
    return (float)((int32_t)raw-(int32_t)(offset))*I_ADC_CONV; 
}

float stm32_cal_ib(uint32_t raw,uint32_t offset)
{
    return (float)((int32_t)raw-(int32_t)(offset))*I_ADC_CONV; 
}

float stm32_cal_ic(uint32_t raw,uint32_t offset)
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

void stm32_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

// float stm32_update_dt(Time_t* time)
// {
//     time->past_time = time->this_time;
//     time->this_time =__HAL_TIM_GET_COUNTER(&htim3);
//     if(time->this_time > time->past_time)
//     {
//         time->dt = (float)(time->this_time - time->past_time)/(170000000.0f/300.0f);
//     }
//     else
//     {
//         time->dt = (float)(65535 - time->past_time + time->this_time)/(170000000.0f/300.0f);
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
    const uint16_t past = time->past_time;
    const uint16_t curr = __HAL_TIM_GET_COUNTER(&htim3);
    time->past_time = curr; // 提前更新，减少内存访问
    time->this_time = curr;

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
    motor->motor_drv.set_pwm_a = stm32_set_pwm_a;      // 设置PWM函数指针
    motor->motor_drv.set_pwm_b = stm32_set_pwm_b;      // 设置PWM函数指针
    motor->motor_drv.set_pwm_c = stm32_set_pwm_c;      // 设置PWM函数指针

    motor->motor_drv.update_ia_raw = stm32_update_ia_raw;       // 获取IA电流函数指针
    motor->motor_drv.update_ib_raw = stm32_update_ib_raw;       // 获取IB电流函数指针
    motor->motor_drv.update_ic_raw = stm32_update_ic_raw;       // 获取IC电流函数指针
    
    motor->motor_drv.cal_ia = stm32_cal_ia;      // 电流转换函数指针
    motor->motor_drv.cal_ib = stm32_cal_ib;      // 电流转换函数指针
    motor->motor_drv.cal_ic = stm32_cal_ic;      // 电流转换函数指针

    motor->motor_drv.delay_ms = stm32_delay_ms;     // 延时函数指针
    motor->motor_drv.update_dt = stm32_update_dt;       // 获取时间差函数指针

    motor->motor_drv.update_angle_raw = stm32_update_angle_raw;    // 获取角度函数指针
    motor->motor_drv.cal_angle = stm32_cal_angle;   // 角度转换函数指针

    motor->motor_number = 1; // 设置电机编号

    // 初始化时间管理
    motor->time.this_time = 0.0f;
    motor->time.past_time = 0.0f;
    motor->time.dt = 0.01f;

    //初始化电机配置
    motor->motor_config.pole_pairs = 14; // 设置电机极对数
    motor->motor_config.dir = 1 ;        // 设置电机转向
    motor->motor_config.phase = 3;      // 设置电机接线相序
    motor->motor_config.mode_sampling = 0X110 ;//AB相双电阻采样
    motor->motor_config.i_max = 20.0f;   // 设置电流限幅
    motor->motor_config.u_max = 24.0f;   // 设置电压限幅
    motor->motor_config.ls = 0.001f;    // 设置定子电感
    motor->motor_config.rs = 0.5f;      // 设置定子电阻
    motor->motor_config.angle_el_zero = 3.13263f; // 设置角度零点
    motor->motor_config.kt = 0.806325918;           //设置转矩系数

    // 初始化PID参数(要用哪个初始化哪个)
    motor->motor_alg.position_pid.kp = 20.3f;
    motor->motor_alg.position_pid.ki = 0.0f;
    motor->motor_alg.position_pid.kd = 0.0f;
    motor->motor_alg.position_pid.integral_max = 100.0f;
    motor->motor_alg.position_pid.integral_min = -100.0f;
    motor->motor_alg.position_pid.output_max = 100.0f;
    motor->motor_alg.position_pid.output_min = -100.0f;
    motor->motor_alg.position_pid.integral_limit = 10.0f;
    motor->motor_alg.position_pid.output_limit = 100.0f;

    motor->motor_alg.velocity_pid.kp = 0.1f;
    motor->motor_alg.velocity_pid.ki = 0.5f;
    motor->motor_alg.velocity_pid.kd = 0.0f;
    motor->motor_alg.velocity_pid.integral_max = 100.0f;
    motor->motor_alg.velocity_pid.integral_min = -100.0f;
    motor->motor_alg.velocity_pid.output_max = 100.0f;
    motor->motor_alg.velocity_pid.output_min = -100.0f;
    motor->motor_alg.velocity_pid.integral_limit = 10.0f;
    motor->motor_alg.velocity_pid.output_limit = 100.0f;

    motor->motor_alg.id_pid.kp = 0.058760f;
    motor->motor_alg.id_pid.ki = 57.6113014f;
    motor->motor_alg.id_pid.kd = 0.000f;
    motor->motor_alg.id_pid.integral_max = motor->motor_config.u_max/2;
    motor->motor_alg.id_pid.integral_min = -motor->motor_config.u_max/2;
    motor->motor_alg.id_pid.output_max = motor->motor_config.u_max/2;
    motor->motor_alg.id_pid.output_min = -motor->motor_config.u_max/2;
    motor->motor_alg.id_pid.integral_limit = motor->motor_config.u_max/2;
    motor->motor_alg.id_pid.output_limit = motor->motor_config.u_max/2;

    motor->motor_alg.iq_pid.kp = 0.058760f;
    motor->motor_alg.iq_pid.ki = 57.6113014f;
    motor->motor_alg.iq_pid.kd = 0.000f;
    motor->motor_alg.iq_pid.integral_max = motor->motor_config.u_max/2;
    motor->motor_alg.iq_pid.integral_min = -motor->motor_config.u_max/2;
    motor->motor_alg.iq_pid.output_max = motor->motor_config.u_max/2;
    motor->motor_alg.iq_pid.output_min = -motor->motor_config.u_max/2;
    motor->motor_alg.iq_pid.integral_limit = motor->motor_config.u_max/2;
    motor->motor_alg.iq_pid.output_limit = motor->motor_config.u_max/2;

    motor->motor_alg.mixed_pid.kp = 0.1f;
    motor->motor_alg.mixed_pid.ki = 0.01f;
    motor->motor_alg.mixed_pid.kd = 0.0f;
    motor->motor_alg.mixed_pid.integral_limit = 10.0f;
    motor->motor_alg.mixed_pid.output_limit = 100.0f;

    motor->motor_data.velocity_lpf.last_output = 0.0f;
    motor->motor_data.velocity_lpf.alpha = 0.5f; // 速度滤波系数
    motor->motor_data.ia_offset_raw = 0XFFFF;
    motor->motor_data.ib_offset_raw = 0XFFFF;
    motor->motor_data.ic_offset_raw = 0XFFFF;
    motor->motor_data.loopcount = 0;

    motor->motor_data.calibrate_2dir_block__velocity_target = 0.03f;
    motor->motor_data.calibrate_2dir_nonblock__velocity_target = 10.0f;
    motor->motor_data.calibrate_2dir_nonblock__time_init = 0.5f;
    motor->motor_data.calibrate_2dir_nonblock__time_prep = 1.0f;
    motor->motor_data.calibrate_2dir_nonblock__time_process = 2.0f;

    motor->motor_data.calibrate_pole_pairs_block__angle_step = 0.01f;
    motor->motor_data.calibrate_pole_pairs_nonblock__velocity_target = 3.0f;
    motor->motor_data.calibrate_pole_pairs_nonblock__time_init = 0.5f;
    motor->motor_data.calibrate_pole_pairs_nonblock__time_prep = 1.0f;
    motor->motor_data.calibrate_pole_pairs_nonblock__time_process = 5.0f;

    motor->motor_data.calibrate_phase_nonblock__ts = 1.0f;
    motor->motor_data.calibrate_phase_nonblock__duty = 0.05f;
    motor->motor_data.calibrate_phase_nonblock__state = 1;

    motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__sum = 0.0f;
    motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__count = 0;
    motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__state = 1;

    motor->motor_data.calibrate_i_offset_nonblock__sample_total = 1000;
}


