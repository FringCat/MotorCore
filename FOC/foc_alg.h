/*
 * SPDX-FileCopyrightText: 2026 FryingCat <3551901875@qq.com>
 * SPDX-License-Identifier: MIT
 */

#ifndef __FOC_ALG_H
#define __FOC_ALG_H

#include "stdint.h"

/**
 * @brief 时间管理结构体
 * @note 用于计算控制周期（dt）、速度（Δ角度/Δ时间），适配定时器计时逻辑
 */
typedef struct
{
    volatile uint32_t ThisTime;    // 当前时刻的时间戳（如定时器当前计数值/系统时间）
    volatile uint32_t PastTime;    // 上一时刻的时间戳（用于计算时间差）
    volatile double dt;            // 时间差（ThisTime - PastTime，单位：s/ms，需根据计时基准统一）
}Time_t;



typedef struct 
{
    float alpha;    // 滤波系数（0~1，值越大滤波越平滑，但响应越慢）
    float last_output;   // 滤波输出值（上一次滤波后的结果，用于下一次滤波计算）
}LPF_t;

/**
 * @brief PID控制器核心结构体
 * @note 通用PID结构体，适配位置环、速度环、电流环、混合环等所有PID控制需求
 */
typedef struct
{
    float KP;         // 比例系数（P）：快速响应误差
    float KI;         // 积分系数（I）：消除静态误差
    float This_I;     // 当前积分值（实时累加误差，需限幅避免积分饱和）
    float Last_I;     // 上一次积分值（用于积分分离、抗积分饱和等优化逻辑）
    float KD;         // 微分系数（D）：抑制超调，提升稳定性
    float error;      // 当前误差（目标值 - 反馈值）
    float Output;     // PID输出值（最终控制量，需结合限幅使用）
    float last_error; // 上一次误差（用于计算微分项：error - last_error）
    float last_feedback; // 上一次反馈值（用于微分项优化，减少噪声影响）

    float integral_limit; // 积分限幅（避免积分项过大，导致控制器饱和）
    float integral_max;   // 积分最大值（积分限幅上限）
    float integral_min;   // 积分最小值（积分限幅下限）
    float output_limit;   // 输出限幅（避免控制量超出执行器能力）
    float output_max;     // 输出最大值（输出限幅上限）
    float output_min;     // 输出最小值（输出限幅下限）
} PID_t;



/**
 * @brief 电机硬件参数配置结构体
 * @note 存储电机固有参数及控制限幅，FOC控制前需先初始化（如从电机手册获取）
 */
typedef struct
{
    uint32_t Pole_pairs;    // 电机极对数（关键参数：机械角度 → 电角度 = 机械角度 × 极对数）
    int DIR;           // 电机转向配置（1：正转，-1：反转，用于修正角度/电流方向）
    int PHASE;         // 电机接线相序（1：ABC，2：ACB，3：BAC，4：BCA，5：CAB，6：CBA，用于修正电流重构）
    float Vdc;         // 直流母线电压
    float IMAX;             // 电流输出限幅（单位：A，保护电机/功率管，避免过流）
    float UMAX;             // 电压输出限幅（单位：V，基于母线电压，避免PWM占空比超范围）
    float Ls;               // 电机定子电感（单位：H，FOC电流环PI参数设计的核心参数）
    float Rs;               // 电机定子电阻（单位：Ω，FOC电流环前馈补偿、铜损计算）
    float Kt;             // 转矩系数
    float angle_zero;       // 机械角度零点（编码器零位校准值，用于位置控制基准）
    float angle_el_zero;    // 电角度零点（FOC磁链定向基准，由机械零点×极对数计算）     
    int loopcount_rotor ;
    int Mode_Sampling;     //电流采样模式：0x100: AXX, 0x010: XBX , 0x001: XXC, 0x110: ABX, 0x101: AXC, 0x011: XBC, 0x111: ABC
} Motor_ConfigTypeDef;



/**
 * @brief 电机控制算法层结构体
 * @note 存储FOC算法、PID控制所需的中间变量，是算法层核心数据载体
 */
typedef struct
{
    float sin_theta,cos_theta;
    // -------------------------- 电流相关（FOC Clark/Park变换前后数据） --------------------------
    float IA, IB, IC;       // 三相定子电流（ADC采集后校准的值，单位：A）
    float Ialpha, Ibeta;    // αβ坐标系电流（Clark变换结果，FOC的中间坐标系）
    float Iq, Id;           // dq坐标系电流（Park变换结果，Id：磁通电流，Iq：转矩电流）
    
    // -------------------------- 电压相关（FOC Park逆变换/SVPWM输出数据） --------------------------
    float UA, UB, UC;       // 三相输出电压（SVPWM计算结果，最终对应PWM占空比）
    float Ualpha, Ubeta;    // αβ坐标系电压（Park逆变换结果，SVPWM的输入）
    float Uq, Ud;           // dq坐标系电压（注释：Ud：d轴电压，Uq：q轴电压，FOC电压环输出）
    
    // -------------------------- 位置/速度相关（控制反馈与目标计算） --------------------------
    float angle;            // 电机转子角度（编码器反馈后校准的值，单位：rad/deg）
    float last_angle;       // 上一时刻转子角度（用于计算速度：Velocity = (angle - LastAngle)/dt）
    float angle_el;         // 电机电角度（angle × Pole_pairs，FOC磁链定向的核心依据）
    float angle_flange;     // 电机输出轴角度(法兰)
    float Velocity;         // 电机转速（由Δangle/Δdt计算，单位：rad/s/RPM）
    float Velocity_flange;
    // -------------------------- PID控制器实例（对应不同控制环） --------------------------
    PID_t position_pid;     // 位置环PID（输入：位置误差（目标pos - 反馈pos），输出：目标速度）
    PID_t velocity_pid;     // 速度环PID（输入：速度误差（目标vel - 反馈vel），输出：目标Iq/Id）
    PID_t id_pid;           // 电流环d轴PID（输入：Id误差（目标Id - 反馈Id），输出：Ud电压）
    PID_t iq_pid;           // 电流环q轴PID（输入：Iq误差（目标Iq - 反馈Iq），输出：Uq电压）
    PID_t mixed_pid;        // 混合控制PID（输入：混合误差（如位置+力矩误差），输出：目标电流，适配机器人关节混合控制）

    int Sector;          // SVPWM扇区（1~6，对应不同的空间矢量位置，用于计算PWM占空比）
} Motor_AlgTypeDef;



/**
 * @brief 电机数据采集与校准结构体
 * @note 存储ADC原始数据、校准偏移量、历史反馈值，适配“单个采集”与“DMA批量采集”两种模式
 */
typedef struct
{
    float Ibus;            // 母线电流（注释：可选，用于功率计算与过流保护，单位：A）
    float Ubus;

    float IA_NoOrder;
    float IB_NoOrder;
    float IC_NoOrder;
    // -------------------------- 电流校准参数 --------------------------
    uint32_t IA_offset_raw;        // IA电流零点偏移（ADC零漂校准值，采集无电流时的ADC值转换而来）
    uint32_t IB_offset_raw;        // IB电流零点偏移（同上）
    uint32_t IC_offset_raw;        // IC电流零点偏移（同上，三相平衡时可省略，由IA+IB推导）
    
    // -------------------------- 速度数据 -------------------------------
    float Velocity_raw;     // 转速原始值（未滤波前的计算结果，用于后续平滑处理）
    LPF_t Velocity_LPF;     // 速度滤波器实例（用于平滑Velocity_raw，提升速度反馈质量）
    float angle_all;

    // -------------------------- 编码器角度原始数据（union适配两种读取方式） --------------------------
    union 
    {
        uint32_t Angle_raw;       // 单个角度原始值
        uint32_t Angle_raw_dma[10];// DMA批量采集的角度原始数组
    } AngleData __attribute__((packed));  // 定义联合体变量（如AngleData）

    // -------------------------- 电流ADC原始数据（union适配两种读取方式） --------------------------
    union 
    {
        struct
        {
            uint32_t IA_raw;       // IA电流ADC原始值
            uint32_t IB_raw;       // IB电流ADC原始值
            uint32_t IC_raw;       // IC电流ADC原始值
        }I_raw;                    // 结构化单个读取
       uint32_t I_raw_dma[3];     // DMA批量采集的电流原始数组
    } CurrentData __attribute__((packed));  // 定义联合体变量（如CurrentData）
} Motor_DataTypeDef;



/**
 * @brief 电机驱动层函数指针结构体
 * @note 驱动层与上层解耦的核心：通过函数指针定义硬件操作接口，不同MCU（如STM32/GD32）只需实现此接口，上层无需修改
 */
typedef struct
{
    // -------------------------- PWM输出接口 --------------------------
    void (*Set_PWM_A)(float Ua);    // 三相PWM-A相输出函数（参数：A相目标电压/占空比，由上层算法计算）
    void (*Set_PWM_B)(float Ub);    // 三相PWM-B相输出函数
    void (*Set_PWM_C)(float Uc);    // 三相PWM-C相输出函数

    // -------------------------- 电流采集接口 --------------------------
    uint32_t (*Update_Ia_raw)(void);       // 获取IA电流ADC原始值（返回：16/32位ADC值，无符号）
    uint32_t (*Update_Ib_raw)(void);       // 获取IB电流ADC原始值
    uint32_t (*Update_Ic_raw)(void);       // 获取IC电流ADC原始值
    
    float (*Cal_Ia)(uint32_t raw,uint32_t offset); // 电流ADC转换函数（参数：ADC原始值，返回：实际电流，单位A）
    float (*Cal_Ib)(uint32_t raw,uint32_t offset); // 电流ADC转换函数
    float (*Cal_Ic)(uint32_t raw,uint32_t offset); // 电流ADC转换函数

    // -------------------------- 角度采集接口 --------------------------
    uint32_t (*Update_Angle_raw)(void);    // 获取编码器角度原始值（返回：编码器输出的原始数据，如14位AS5047P数据）
    float (*Cal_Angle)(uint32_t raw); // 角度转换函数（参数：编码器原始值，返回：实际机械角度，单位rad/deg）

    // -------------------------- 通用硬件接口 --------------------------
    void (*Delayms)(uint32_t ms);  // 毫秒级延时函数（用于校准、初始化等待）
    float (*Update_dt)(Time_t* time);  // 计算时间差dt函数（参数：Time_t结构体，返回：当前dt值，用于速度计算）
} Motor_DrvTypeDef;



/**
 * @brief 电机控制总句柄结构体
 * @note 整合电机所有资源（配置、驱动、算法、数据、时间），支持多电机管理（通过motor_number区分）
 */
typedef struct
{
    uint32_t motor_number;              // 电机编号（多电机控制时用于区分不同电机，如关节1/关节2）
    Time_t time;                        // 时间管理实例（绑定当前电机的计时逻辑）    
    Motor_ConfigTypeDef MotorConfig;    // 电机硬件配置实例（存储当前电机的固有参数）
    Motor_DrvTypeDef MotorDrv;          // 电机驱动接口实例（绑定当前电机的硬件操作函数）
    Motor_AlgTypeDef MotorAlg;          // 电机算法层实例（存储当前电机的控制中间变量与PID）
    Motor_DataTypeDef MotorData;        // 电机数据处理实例（存储当前电机的原始数据与校准参数）
} Motor_HandleTypeDef;



// 数学常数
#define PI          3.14159265359f          // 圆周率
#define _2PI        6.28318530718f          // 2π
#define PI_2        1.57079632679f          // π/2
#define _3PI_2      4.71238898038f          // 3π/2
#define SQRT3       1.732050807568877293f   // 根号3
#define _1_SQRT3    0.57735026919f          // 1/根号3
#define _2_SQRT3    1.15470053838f          // 2/根号3
#define _1_3        (1.0f / 3.0f)           // 1/3
#define SIN_C3      (-0.16666502f)
#define SIN_C5      ( 0.00833026f)


// 通用数学工具
float my_sgn(float x);                                                              // 符号函数（返回1/0/-1）
float my_abs(float x);                                                              // 绝对值函数
float my_Limit(float x, float max, float min);                                      // 限幅（将x限制在[min,max]）
float my_map(float x, float in_min, float in_max, float out_min, float out_max);    // 数值映射（范围转换）
float my_round(float x);                                                            /* 四舍五入，等价 roundf */
float my_round_to_decimal(float x, int n);
float my_sin(float x);                                                              // 切比雪夫多项式近似的sin函数（输入x为弧度，输出为sin(x)的近似值）
float my_cos(float x);                                                              // 切比雪夫多项式近似的cos函数（输入x为弧度，输出为cos(x)的近似值）
float my_fmodf(float x, float y);
float my_floor(float x);                                                            /* 向下取整，等价 floorf */
float my_pow(float base, float exp);                                                /* 幂运算，等价 powf 常用场景 */
float my_sat(float e, float r);
int32_t my_fast_round(float x);

// 数据重置与初始化
void reset_data_angle(Motor_HandleTypeDef *motor);

// 电角度处理
float Limit_angle(float angle, float Low, float High);
float Limit_angle_el(float angle_el);                                               // 电角度限幅（约束在0~2π）
float Get_angle_el(Motor_HandleTypeDef *motor);                                     // 获取当前电角度
float update_angle_el(Motor_HandleTypeDef *motor);                                  // 更新电角度（机械角→电角度）
float Calculate_angle_el(float Pole_pairs,float angle,float angle_el_zero);         // 计算电角度（含极对数和零点）
float update_angle(Motor_HandleTypeDef *motor);

// FOC坐标变换
void update_sincos(Motor_HandleTypeDef *motor);
void Calculate_Park_N_theta(float Uq , float Ud , float angle_el, float *Ualpha, float *Ubeta); // Park逆变换（dq→αβ电压）
void Calculate_Park_N_sincos(float Uq , float Ud , float sin_theta,float cos_theta, float *Ualpha, float *Ubeta);
void update_Park_N(Motor_HandleTypeDef *motor);                                     // 更新Park逆变换结果到算法层
void Calculate_Clark_N(float Ualpha ,float Ubeta,float Upower, float *Ua, float *Ub, float *Uc); // Clark逆变换（αβ→三相电压）
void update_Clark_N(Motor_HandleTypeDef *motor);                                    // 更新Clark逆变换结果到算法层
void Calculate_Clark(float IA ,float IB ,float IC, float *Ialpha, float *Ibeta);    // Clark变换（三相电流→αβ电流）
void update_Clark(Motor_HandleTypeDef *motor);                                      // 更新Clark变换结果到算法层
void Calculate_Park_theta(float Ialpha ,float Ibeta ,float angle_el, float *Id, float *Iq); // Park变换（αβ电流→dq电流）
void Calculate_Park_sincos(float Ialpha ,float Ibeta ,float sin_theta,float cos_theta, float *Id, float *Iq); // Park变换（αβ电流→dq电流）
void update_Park(Motor_HandleTypeDef *motor);                                       // 更新Park变换结果到算法层

// PWM输出控制
void update_pwm(Motor_HandleTypeDef *motor);                                        // 更新PWM（用算法层三相电压输出）
void set_pwm(Motor_HandleTypeDef *motor,float Ta , float Tb ,float Tc);             // 设置三相PWM（考虑转向）
void set_pwm_nodir(Motor_HandleTypeDef *motor,float Ta , float Tb ,float Tc);       // 设置三相PWM（不考虑转向）

// SPWM相关
void update_spwm(Motor_HandleTypeDef *motor);                                       // 更新SPWM（用算法层三相电压输出）
void set_spwm(Motor_HandleTypeDef *motor,float Uq, float Ud ,float angle_el);       // 设置SPWM（考虑转向）
void set_svpwm_dir(Motor_HandleTypeDef *motor, float Uq , float Ud ,float angle_el);

// SVPWM相关
int Calculate_Sector( float Ualpha , float Ubeta );                                 // 计算电压矢量所在扇区（1~6）
int update_Sector(Motor_HandleTypeDef *motor);                                      // 更新扇区到算法层
int get_Sector(Motor_HandleTypeDef *motor);                                         // 获取当前扇区
void update_svpwm(Motor_HandleTypeDef *motor);                                      // 更新SVPWM（自动计算并输出PWM）
void set_svpwm(Motor_HandleTypeDef *motor, float Uq , float Ud ,float angle_el);    // 手动设置SVPWM参数

// 速度计算与滤波
float Calculate_LPF(float input, float last_output, float alpha);                   // 一阶低通滤波
float update_velocity_LPF(Motor_HandleTypeDef *motor);                              // 更新滤波后转速
float update_velocity_raw(Motor_HandleTypeDef *motor);                              // 更新原始转速
float get_velocity(Motor_HandleTypeDef *motor);                                     // 获取当前转速
float get_velocity_raw(Motor_HandleTypeDef *motor);                                 // 获取当前原始转速
float Calculate_velocity_LPF(float angle, float last_angle, float dt, float last_velocity, float alpha);  // 计算转速（含滤波）
float Calculate_velocity_raw(float angle, float last_angle, float dt);              // 计算转速（不含滤波）

// PID控制
float Calculate_PID(float target, float feedback, float dt ,PID_t* pid);  // PID计算（位置式+，支持积分处理）
float Calculate_PID_IS(float target, float feedback, float dt, PID_t* pid,float sep_err_upper, float sep_err_lower); //带有积分分离的PID计算
float Calculate_PID_IS_AIS(float target, float feedback, float dt, PID_t* pid,float n); //带有自适应积分分离的PID计算

// 时间相关
float update_dt(Motor_HandleTypeDef *motor); // 更新dt值
float get_dt(Motor_HandleTypeDef *motor); // 获取当前dt值

// 数据采样与处理
void Calculate_Order_int(float IA ,float IB ,float IC , int PHASE, int *IA_out, int *IB_out, int *IC_out); // 相序重构函数(int)
void Calculate_Order_float(float IA ,float IB ,float IC , int PHASE, float *IA_out, float *IB_out, float *IC_out); // 相序重构函数(float)
int update_IaIbIc(Motor_HandleTypeDef *motor,int Mode_Sampling,int PHASE);
void update_IalphaIbeta(Motor_HandleTypeDef *motor);
void update_IqId(Motor_HandleTypeDef *motor);
void update_Ioffset_block(Motor_HandleTypeDef *motor,int Mode_Sampling);  // 更新电流偏置-阻塞式（静止时多次采样平均）
int update_Ioffset_nonblock(Motor_HandleTypeDef *motor,int Mode_Sampling);// 更新电流偏置-非阻塞式（静止时单次采样累加，需多次调用）
float get_Ia(Motor_HandleTypeDef *motor);  // 获取IA电流
float get_Ib(Motor_HandleTypeDef *motor);  // 获取IB电流
float get_Ic(Motor_HandleTypeDef *motor);  // 获取IC电流
float get_Ia_offset(Motor_HandleTypeDef *motor);  // 获取IA电流偏置
float get_Ib_offset(Motor_HandleTypeDef *motor);  // 获取IB电流偏置
float get_Ic_offset(Motor_HandleTypeDef *motor);  // 获取IC电流偏置

//初始化相关
void update_2DIR_sensor_block(Motor_HandleTypeDef *motor);              //  传感器方向辨识（基于磁编，阻塞式更新）
void update_2DIR_sensor_nonblock(Motor_HandleTypeDef *motor);           //  传感器方向辨识（基于磁编，非阻塞式更新)

int *Calculate_PHASE(float IA, float IB, float IC,float UA, float UB, float UC);
int update_PHASE_nonblock(Motor_HandleTypeDef *motor,float IA_NoOrder, float IB_NoOrder, float IC_NoOrder); // 更新相序-非阻塞式
int update_PHASE_block(Motor_HandleTypeDef *motor); // 更新相序-阻塞式

void update_pole_pairs_sensor_block(Motor_HandleTypeDef *motor);        // 极对数辨识（有传感器，阻塞式更新）
void update_pole_pairs_sensor_nonblock(Motor_HandleTypeDef *motor);     // 极对数辨识（有传感器，非阻塞式更新）

void update_angle_el_zero_sensor_block(Motor_HandleTypeDef *motor);     //  电角度零点校准 （有传感器，阻塞式更新）
void update_angle_el_zero_sensor_nonblock(Motor_HandleTypeDef *motor);  //  电角度零点校准 （有传感器，非阻塞式更新）
void update_angle_el_zero_no_sensor_block(Motor_HandleTypeDef *motor);  //  电角度零点校准 （无传感器，阻塞式更新）

//控制相关
void ctrl_motor_openloop_velocity_el_nonblock(Motor_HandleTypeDef *motor,float velocity_el_target,float Uq,float Ud);
void ctrl_motor_openloop_velocity_nonblock(Motor_HandleTypeDef *motor,float velocity_target,float Uq,float Ud);
float ctrl_motor_openloop_angle_el_nonblock(Motor_HandleTypeDef *motor,float angle_el_target,float angle_el_start,float velocity_el_target ,float Uq,float Ud);
void ctrl_motor_openloop_angle_el_block(Motor_HandleTypeDef *motor,float angle_el_target,float angle_el_start,float velocity_el_target ,float Uq,float Ud);
float ctrl_motor_openloop_angle_nonblock(Motor_HandleTypeDef *motor,float angle_target,float angle_start,float velocity_target ,float Uq,float Ud);
void ctrl_motor_openloop_angle_block(Motor_HandleTypeDef *motor,float angle_target,float angle_start,float velocity_target ,float Uq,float Ud);

#endif
