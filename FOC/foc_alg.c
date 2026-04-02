#include "foc_alg.h"
#include <stdio.h>
// #include <math.h>
#include "arm_math.h"
#include <stdlib.h>
#include "SEGGER_RTT.h"
#include "foc_drv.h"
#include "flash.h"

// Flash 中存储 DIR 的专用扇区（SECTOR_1，与 MotorConfig 的 SECTOR_0 分开）
#define DIR_FLASH_ADDR  ADDR_FLASH_SECTOR_1
// 魔数：用于判断 Flash 中的数据是否有效
#define DIR_FLASH_MAGIC 0xD1A50000UL
float sgn(float x)
{
	if(x>0)
	{
		return 1 ;
	}
	else if(x<0)
	{
		return -1 ;
	}
	else
	{
		return 0 ;
	}
}

float Sat(float e, float r) 
{
    if (e > r) 
    {
        return r;
    } else if (e < -r) 
    {
        return -r;
    } else 
    {
        return e;
    }
}
//love11111
float myabs(float val)
{
	if(val>=0)
	{
		return val ;
	}
	else 
	{
		return -val;
	}
}

float round_to_decimal(float x, int n) 
{
    if (n < 0) return x;  // 处理无效输入（n不能为负数）

    float scale = pow(10.0f, n);  // 计算10^n（放大倍数）
    return round(x * scale) / scale;  // 四舍五入后还原
}

int32_t fast_round(float x) 
{
    if (x >= 0) 
    {
        return (int32_t)(x + 0.5f);
    } else 
    {
        return (int32_t)(x - 0.5f);
    }
}

float mymap( float Data ,float formLOW,float formHIGH, float toLOW,float toHIGH)
{
	return ((Data-formLOW)*((float)((toHIGH-toLOW)/(float)(formHIGH-formLOW))))+toLOW;
}

void reset_data_angle(Motor_HandleTypeDef *motor)
{
    motor->MotorData.angle_all = 0.0f;
    motor->MotorAlg.last_angle = 0.0f;
    motor->MotorAlg.angle = 0.0f;
}

float Limit_angle(float angle, float Low, float High)
{
    // 异常处理：若上下限差值过小（周期为0），返回NaN标识错误
    const float EPS = 1e-6f;
    float period = High - Low;
    if (fabs(period) < EPS)
    {
        return 0;  
    }

    if (period < 0.0f)
    {
        float temp = Low;
        Low = High;
        High = temp;
        period = -period; 
    }

    float offset = angle - Low;

    offset = fmod(offset, period);

    if (offset < 0.0f)
    {
        offset += period;
    }

    float angle_limited = Low + offset;

    return angle_limited;
}

float Limit_angle_el(float angle_el)
{
    // 先处理正角度，减去 2*PI 的整数倍
    while (angle_el >= 2 * PI)
    {
        angle_el -= 2 * PI;
    }
    // 再处理负角度，加上 2*PI 的整数倍
    while (angle_el < 0)
    {
        angle_el += 2 * PI;
    }
    return angle_el;
}

float Limit_angle_flange(float angle_all,float GR)
{
    float flange_angle = angle_all / GR;
    flange_angle = Limit_angle(flange_angle,-PI,PI);
    // flange_angle = Limit_angle(flange_angle,-12.5f,12.5f);
    // flange_angle = Limit_angle_el(flange_angle);
    return flange_angle;
}

float update_angle(Motor_HandleTypeDef *motor)//待更新:angle_all的更新是上一个周期的angle_all 不是这次的angle_all
{
    static float data_debug_0_ = 0.0f;
    float error_angle = motor->MotorAlg.angle-motor->MotorAlg.last_angle;
    if(fabs(error_angle) > (0.8f*2*PI))
    {
        if((error_angle)<0){motor->MotorData.angle_all += (2*PI - motor->MotorAlg.last_angle + motor->MotorAlg.angle) ;}//正转
        else if((error_angle)>=0){motor->MotorData.angle_all += -(2*PI - motor->MotorAlg.angle + motor->MotorAlg.last_angle) ;}//反转
    }
    else 
    {
        motor->MotorData.angle_all += error_angle ;
    }

    motor->MotorAlg.last_angle = motor->MotorAlg.angle;

    motor->MotorData.AngleData.Angle_raw = motor->MotorDrv.Update_Angle_raw();
    motor->MotorAlg.angle = motor->MotorDrv.Cal_Angle(motor->MotorData.AngleData.Angle_raw);
    // data_debug_0_ = Calculate_angle_NLLUT(motor->MotorAlg.angle,motor->MotorConfig.NLLUT_encoder,128);
    // motor->MotorAlg.angle_flange = Calculate_angle_flange(motor->MotorData.angle_all,motor->MotorConfig.GR,motor->MotorConfig.angle_zero);

    motor->MotorAlg.angle_el = Calculate_angle_el(motor->MotorConfig.Pole_pairs,motor->MotorAlg.angle, motor->MotorConfig.angle_el_zero);

    return motor->MotorAlg.angle;
}

float update_angle_NLLUT(Motor_HandleTypeDef *motor)
{
    static float data_debug_0_ = 0.0f;
    float error_angle = motor->MotorAlg.angle-motor->MotorAlg.last_angle;
    if(fabs(error_angle) > (0.8f*2*PI))
    {
        if((error_angle)<0){motor->MotorData.angle_all += (2*PI - motor->MotorAlg.last_angle + motor->MotorAlg.angle) ;}//正转
        else if((error_angle)>=0){motor->MotorData.angle_all += -(2*PI - motor->MotorAlg.angle + motor->MotorAlg.last_angle) ;}//反转
    }
    else 
    {
        motor->MotorData.angle_all += error_angle ;
    }

    motor->MotorAlg.last_angle = motor->MotorAlg.angle;

    motor->MotorData.AngleData.Angle_raw = motor->MotorDrv.Update_Angle_raw();
    motor->MotorAlg.angle = Calculate_angle_NLLUT(motor->MotorDrv.Cal_Angle(motor->MotorData.AngleData.Angle_raw),motor->MotorConfig.NLLUT_encoder,128);
    // motor->MotorAlg.angle_flange = Calculate_angle_flange(motor->MotorData.angle_all,motor->MotorConfig.GR,motor->MotorConfig.angle_zero);

    motor->MotorAlg.angle_el = Calculate_angle_el(motor->MotorConfig.Pole_pairs,motor->MotorAlg.angle, motor->MotorConfig.angle_el_zero);

    return motor->MotorAlg.angle;
}

float Get_angle_el(Motor_HandleTypeDef *motor) 
{
    return motor->MotorAlg.angle_el;
}

float Calculate_angle_el(float Pole_pairs,float angle,float angle_el_zero) 
{
    return Limit_angle_el(angle * Pole_pairs + angle_el_zero);
}

float Calculate_angle_flange(float angle ,float GR,float angle_zero)
{
    return Limit_angle_el(angle* (1/GR) + angle_zero);
}

float Calculate_angle_NLLUT(float angle ,float* NLLUT_encoder,uint32_t size_NLLUT)
{
    uint32_t sector_NLLUT = (uint32_t)fast_round((angle/(2*PI))*(float)size_NLLUT);
    float angle_1 = ((float)sector_NLLUT * (2*PI))/(float)size_NLLUT;
    float angle_2 = ((float)(sector_NLLUT+1) * (2*PI))/(float)size_NLLUT;
    float error_1 = NLLUT_encoder[sector_NLLUT];
    float error_2 = NLLUT_encoder[sector_NLLUT+1];
    if(sector_NLLUT >= (size_NLLUT-1))
    {
        return angle;
        // angle_2 = 0;
        // error_2 = NLLUT_encoder[0];
    }

    return angle+error_1+((error_2-error_1)*(angle-angle_1))/(angle_2-angle_1);
}

float update_angle_flange(Motor_HandleTypeDef *motor)//引出一个新问题：数据更新需要一个同步机制，同一个周期内只能有一次获取数据更新的操作，不能让其他函数重复发起数据更新的操作
{
    motor->MotorAlg.angle = motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());
    motor->MotorAlg.angle_flange = Calculate_angle_flange(motor->MotorData.angle_all,motor->MotorConfig.GR,motor->MotorConfig.angle_zero);
    return motor->MotorAlg.angle_flange;
}

float update_angle_el(Motor_HandleTypeDef *motor) 
{
    motor->MotorAlg.angle = motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());
    motor->MotorAlg.angle_el = Calculate_angle_el(motor->MotorConfig.Pole_pairs,motor->MotorAlg.angle, motor->MotorConfig.angle_el_zero);
    return motor->MotorAlg.angle_el;
}

float Limit(float value , float high , float low)
{
    return (value)<(low)?(low):((value)>(high)?(high):(value));//如果目标参数超出最大/最小值的范围，就把这个值锁死在最大/最小值
}

// float *Calculate_Park_N(float Uq , float Ud , float angle_el)
// {

// 	static float Upark_N[2];
	
// 	Upark_N[0] = Ud*cos(angle_el) - Uq*sin(angle_el); //Park逆变换 ①Ualpha = Ud * cosθ - Uq * sinθ
//  Upark_N[1] = Uq*cos(angle_el) + Ud*sin(angle_el); //           ②Ubeta  = Uq * cosθ + Ud * sinθ
// 	return Upark_N;
	
// }

float *Calculate_Park_N(float Uq , float Ud , float angle_el)
{
    static float Upark_N[2];  // 保留原静态数组以兼容接口
    float cos_theta, sin_theta;

    // angle_el = fmodf(angle_el, 2.0f * PI);  // 取模运算，限制角度范围
    // if(angle_el < 0.0f)  // 处理负角度
    // {
    //     angle_el += 2.0f * PI;
    // }
    cos_theta = arm_cos_f32(angle_el);
    sin_theta = arm_sin_f32(angle_el);

    Upark_N[0] = Ud * cos_theta - Uq * sin_theta;  // Ualpha
    Upark_N[1] = Uq * cos_theta + Ud * sin_theta;  // Ubeta

    return Upark_N;
}

float *update_Park_N(Motor_HandleTypeDef *motor)
{
    static float* Upark_N;
    Upark_N = Calculate_Park_N(motor->MotorAlg.Uq , motor->MotorAlg.Ud , Limit_angle_el(motor->MotorAlg.angle_el));
    motor->MotorAlg.Ualpha = Upark_N[0];
    motor->MotorAlg.Ubeta  = Upark_N[1];
    return Upark_N;
}

float *Calculate_Clark_N(float Ualpha ,float Ubeta,float Upower)
{	
    static float Uclark_N[3];
    
    //Clark逆变换:
    Uclark_N[0] = Ualpha + Upower/2;                 // ①Ua = Ualpha ;
    Uclark_N[1] = (sqrt_3*Ubeta-Ualpha)/2 + Upower/2; // ②Ub = (√3 * Ubeta - Ualpha)/2 ;
    Uclark_N[2] = -(Ualpha + sqrt_3*Ubeta)/2 + Upower/2;// ③Uc = ( -Ualpha - √3 * Ubeta )/2;
    
    return Uclark_N;
}

float *update_Clark_N(Motor_HandleTypeDef *motor)
{	
    static float* Uclark_N;
    Uclark_N = Calculate_Clark_N(motor->MotorAlg.Ualpha , motor->MotorAlg.Ubeta , motor->MotorConfig.UMAX);
    motor->MotorAlg.UA = Uclark_N[0];
    motor->MotorAlg.UB = Uclark_N[1];
    motor->MotorAlg.UC = Uclark_N[2];
    return Uclark_N;
}

#define INV_SQRT3_F   (1.0f / 1.7320508075688772f)  // ≈0.5773502691896257f
#define INV_3_F       (1.0f / 3.0f)                 // 1/3，Clark变换用

float *Calculate_Clark(float IA ,float IB ,float IC)
{	
    static float Iclark[2];
    // const float sqrt3 = sqrt(3.0); // 高精度√3
    
    // 幅值不变型Clark变换（对称三相电流）
    Iclark[0] = (2*IA - IB - IC)*INV_3_F;  // Iα
    Iclark[1] = (IB - IC)*INV_SQRT3_F;       // Iβ

    return Iclark;
}

float *update_Clark(Motor_HandleTypeDef *motor)
{	
    static float* Iclark;
    Iclark = Calculate_Clark(motor->MotorAlg.IA , motor->MotorAlg.IB , motor->MotorAlg.IC);
    motor->MotorAlg.Ialpha = Iclark[0];
    motor->MotorAlg.Ibeta  = Iclark[1];
    return Iclark;
}

float *Calculate_Park(float Ialpha ,float Ibeta ,float angle_el_rad)
{
    static float Ipark[2];
    
    // Park变换（输入电角度为弧度制）
    Ipark[0] = Ialpha * arm_cos_f32(angle_el_rad) + Ibeta * arm_sin_f32(angle_el_rad);  // Id
    Ipark[1] = -Ialpha * arm_sin_f32(angle_el_rad) + Ibeta * arm_cos_f32(angle_el_rad); // Iq
    
    return Ipark;
}
float *update_Park(Motor_HandleTypeDef *motor)
{
    static float* Ipark;
    Ipark = Calculate_Park(motor->MotorAlg.Ialpha , motor->MotorAlg.Ibeta , Limit_angle_el(motor->MotorAlg.angle_el));
    motor->MotorAlg.Id = Ipark[0];
    motor->MotorAlg.Iq = Ipark[1];
    return Ipark;
}

void update_pwm(Motor_HandleTypeDef *motor)
{
    float _Ua = Limit(motor->MotorAlg.UA/motor->MotorConfig.UMAX , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = Limit(motor->MotorAlg.UB/motor->MotorConfig.UMAX , 1 , 0 );
	float _Uc = Limit(motor->MotorAlg.UC/motor->MotorConfig.UMAX , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL | motor->MotorDrv.Set_PWM_B!=NULL | motor->MotorDrv.Set_PWM_C!=NULL)
    {
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case 2:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case 3:
            {
                motor->MotorDrv.Set_PWM_A(_Uc);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Ua);
            }break;
            case 4:
            {
                motor->MotorDrv.Set_PWM_A(_Uc);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Ub);
            }break;
            case 5:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Uc);
                motor->MotorDrv.Set_PWM_C(_Ua);
            }break;
            case 6:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Uc);
                motor->MotorDrv.Set_PWM_C(_Ub);
            }break;
            default:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
        }
    }
    else
    {
        /*打印报错信息*/
    }
}

void set_pwm(Motor_HandleTypeDef *motor,float Ta , float Tb ,float Tc)
{
    float _Ua = Limit(Ta , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = Limit(Tb , 1 , 0 );
	float _Uc = Limit(Tc , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL | motor->MotorDrv.Set_PWM_B!=NULL | motor->MotorDrv.Set_PWM_C!=NULL)
    {
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case 2:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case 3:
            {
                motor->MotorDrv.Set_PWM_A(_Uc);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Ua);
            }break;
            case 4:
            {
                motor->MotorDrv.Set_PWM_A(_Uc);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Ub);
            }break;
            case 5:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Uc);
                motor->MotorDrv.Set_PWM_C(_Ua);
            }break;
            case 6:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Uc);
                motor->MotorDrv.Set_PWM_C(_Ub);
            }break;
            default:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
        }
    }
    else
    {
        /*打印报错信息*/
    }
}

void set_pwm_nodir(Motor_HandleTypeDef *motor,float Ta , float Tb ,float Tc)
{
    float _Ua = Limit(Ta , 1 , 0 );//计算并限制ABC相所需的占空比
    float _Ub = Limit(Tb , 1 , 0 );
    float _Uc = Limit(Tc , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL | motor->MotorDrv.Set_PWM_B!=NULL | motor->MotorDrv.Set_PWM_C!=NULL)
    {
        motor->MotorDrv.Set_PWM_A(_Ua);
        motor->MotorDrv.Set_PWM_B(_Ub);
        motor->MotorDrv.Set_PWM_C(_Uc);
    }
    else
    {
        /*打印报错信息*/
    }
}

int Calculate_Sector( float Ualpha , float Ubeta )//存在较多边界条件问题
{
    if (Ubeta == 0.0f) 
    {
        return (Ualpha >= 0.0f) ? 1 : 4;  // α轴：正→1，负→4
    }
    if (Ualpha == 0.0f) {
        return (Ubeta >= 0.0f) ? 2 : 5;  // β轴：正→2，负→5
    }
	if((Ualpha>0.0f) && (Ubeta>0.0f) && (Ubeta/Ualpha < sqrt_3)){return 1 ;}
	else if((Ubeta>0.0f) && (Ubeta/myabs(Ualpha)>sqrt_3)){return 2 ;}
	else if((Ualpha<0.0f) && (Ubeta>0.0f) && (-Ubeta/Ualpha < sqrt_3)){return 3 ;}
	else if((Ualpha<0.0f) && (Ubeta<0.0f) && (Ubeta/Ualpha < sqrt_3)){return 4 ;}
	else if((Ubeta<0.0f) && (-Ubeta/myabs(Ualpha)>sqrt_3)){return 5 ;}
	else if((Ualpha>0.0f) && (Ubeta<0.0f) && (-Ubeta/Ualpha < sqrt_3)){return 6 ;}
	else {return 0;}
}

int update_Sector(Motor_HandleTypeDef *motor)
{
    motor->MotorAlg.Sector = Calculate_Sector(motor->MotorAlg.Ualpha , motor->MotorAlg.Ubeta);
    return motor->MotorAlg.Sector;
}

int get_Sector(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.Sector;
}

float Calculate_LPF(float input, float last_output, float alpha)
{
    return alpha * input + (1 - alpha) * last_output;
}

float update_velocity_LPF(Motor_HandleTypeDef *motor)  
{
    motor->MotorData.Velocity_raw = Calculate_velocity_raw(motor->MotorAlg.angle, motor->MotorAlg.last_angle, motor->time.dt);
    motor->MotorAlg.Velocity =  Calculate_LPF(motor->MotorData.Velocity_raw, motor->MotorData.Velocity_LPF.last_output, motor->MotorData.Velocity_LPF.alpha);
    motor->MotorData.Velocity_LPF.last_output = motor->MotorAlg.Velocity; // 更新滤波器的上次输出值
    // motor->MotorAlg.last_angle = motor->MotorAlg.angle; // （已注释）更新上一时刻角度,,update_angle_el函数中已进行此操作
    return motor->MotorAlg.Velocity;
}

float update_velocity_raw(Motor_HandleTypeDef *motor)
{
    motor->MotorData.Velocity_raw = Calculate_velocity_raw(motor->MotorAlg.angle, motor->MotorAlg.last_angle, motor->time.dt);
    // motor->MotorAlg.last_angle = motor->MotorAlg.angle; // （已注释）更新上一时刻角度,,update_angle_el函数中已进行此操作
    return motor->MotorData.Velocity_raw;
}

float Calculate_velocity_raw(float angle, float last_angle, float dt)
{
    if (dt <= 0) 
    {
        return 0; // 避免除以零
    }

    float velocity_raw;
    // 计算原始速度
    if(fabs(angle - last_angle) > (0.8f*2*PI))
    {
        if((angle - last_angle)<0){velocity_raw = (2*PI - last_angle + angle)/dt ;}//正转
        else if((angle - last_angle)>=0){velocity_raw = -(2*PI - angle + last_angle)/dt ;}//反转
    }
    else 
    {
        velocity_raw = (angle - last_angle)/dt ;
    }

    return velocity_raw;
}

float Calculate_velocity_LPF(float angle, float last_angle, float dt, float last_velocity, float alpha)
{
    float velocity_raw = Calculate_velocity_raw(angle, last_angle, dt);
    return Calculate_LPF(velocity_raw, last_velocity, alpha);
}

float get_velocity(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.Velocity;
}

float get_velocity_raw(Motor_HandleTypeDef *motor)
{
    return motor->MotorData.Velocity_raw;
}

/**
 * @brief 速度滑动平均滤波（SMA）更新函数
 * @note 原理：存储最近N个原始速度值，计算平均值作为滤波后速度，抑制随机噪声
 * @param motor：电机总句柄（含速度原始数据、滑动滤波配置）
 * @return 滤波后的电机转速（单位：rad/s/RPM，与原始速度单位一致）
 */
float update_velocity_SMA(Motor_HandleTypeDef *motor)
{
    // 1. 计算当前原始速度（复用现有原始速度计算函数，保证一致性）
    motor->MotorData.Velocity_raw = Calculate_velocity_raw(motor->MotorAlg.angle, 
                                                           motor->MotorAlg.last_angle, 
                                                           motor->time.dt);
    
    // 2. 滑动滤波核心逻辑：更新缓冲区、维护总和、计算平均值
    SMA_t *sma = &motor->MotorData.Velocity_SMA;
    
    // （1）移除即将被覆盖的旧值（避免总和累积错误）
    if (sma->sample_num >= sma->max_num)
    {
        // 缓冲区已满，先减去即将被覆盖的旧值
        sma->sum -= sma->buffer[sma->index];
    }
    else
    {
        // 缓冲区未满，计数+1（直到达到最大采样点数）
        sma->sample_num++;
    }
    
    // （2）存入新的原始速度值，更新总和
    sma->buffer[sma->index] = motor->MotorData.Velocity_raw;
    sma->sum += motor->MotorData.Velocity_raw;
    
    // （3）更新索引（循环覆盖，避免数组越界）
    sma->index = (sma->index + 1) % sma->max_num;
    
    // （4）计算滑动平均（滤波结果）
    motor->MotorAlg.Velocity = sma->sum / sma->sample_num;
    
    // 3. （可选）与低通滤波并行使用时，可在此添加二次滤波（如SMA+LPF组合）
    // motor->MotorAlg.Velocity = Calculate_LPF(motor->MotorAlg.Velocity, 
    //                                          motor->MotorData.Velocity_LPF.last_output, 
    //                                          0.05f);
    
    // 4. 无需手动更新last_angle（update_angle_el函数已处理，与原函数保持一致）
    return motor->MotorAlg.Velocity;
}

float Calculate_PID(float target, float feedback, float dt ,PID_t* pid)
{
    pid->error = target - feedback;
    
    // 计算积分项
    pid->This_I += pid->error * dt ;
    pid->This_I = Limit(pid->This_I, pid->integral_max, pid->integral_min); // 限制积分项防止积分饱和

    // 计算微分项
    float derivative = (pid->error - pid->last_error) / dt;

    // 计算PID输出
    pid->Output = pid->KP * pid->error + pid->KI * pid->This_I + pid->KD * derivative;
    // pid->Output = pid->KP * pid->error;
    pid->Output = Limit(pid->Output, pid->output_max, pid->output_min); // 限制输出范围

    // 更新历史误差
    pid->last_error = pid->error;

    return pid->Output;
}

float Calculate_PID_IS(float target, float feedback, float dt, PID_t* pid,float sep_err_upper, float sep_err_lower)   //积分分离
{
    pid->error = target - feedback;
    
    if ((pid->error >= sep_err_lower) && (pid->error <= sep_err_upper))
    {
        pid->This_I += pid->error * dt;
    }
    else
    {
        pid->This_I = 0.0f; // 超出范围时，积分项清零
    }
    pid->This_I = Limit(pid->This_I, pid->integral_max, pid->integral_min);

    float derivative = (pid->error - pid->last_error) / dt;
    pid->Output = pid->KP * pid->error + pid->KI * pid->This_I + pid->KD * derivative;
    pid->Output = Limit(pid->Output, pid->output_max, pid->output_min);

    pid->last_error = pid->error;
    return pid->Output;
}

float Calculate_PID_IS_AIS(float target, float feedback, float dt, PID_t* pid,float n)   //自适应积分分离
{
    static float target_sum = 0;
    float r = pid->KP;
    float D = target;

    float sum = r*D * ((1.0f-pow((1.0f-r),n))/r);

    pid->error = target - feedback;

    target_sum = (sum);
    if (fabs(pid->error) <= fabs(target-sum))
    {
        pid->This_I += pid->error * dt;
    }
    else
    {
        pid->This_I = 0.0f; // 超出范围时，积分项清零
    }
    pid->This_I = Limit(pid->This_I, pid->integral_max, pid->integral_min);

    float derivative = (pid->error - pid->last_error) / dt;
    pid->Output = pid->KP * pid->error + pid->KI * pid->This_I + pid->KD * derivative;
    pid->Output = Limit(pid->Output, pid->output_max, pid->output_min);

    pid->last_error = pid->error;
    return pid->Output;
}

void update_svpwm(Motor_HandleTypeDef *motor)
{
    static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;

    update_Park_N(motor);
    update_Sector(motor);

    K=(sqrt_3*1)/(motor->MotorConfig.UMAX/2);
	Ux = motor->MotorAlg.Ubeta;
	Uy = (sqrt_3/2.0f)*motor->MotorAlg.Ualpha - 0.5f*motor->MotorAlg.Ubeta;
	Uz = (sqrt_3/2.0f)*motor->MotorAlg.Ualpha + 0.5f*motor->MotorAlg.Ubeta;

    switch (motor->MotorAlg.Sector)
	{
	case 1:
		Tx = K*Uy ;
		Ty = K*Ux ;
		break;
	case 2:
		Tx = -K*Uy ;
		Ty = K*Uz ;
		break;
	case 3:
		Tx = K*Ux ;
		Ty = -K*Uz ;
		break;
	case 4:
		Tx = -K*Ux ;
		Ty = -K*Uy ;
		break;
	case 5:
		Tx = -K*Uz ;
		Ty = K*Uy ;
		break;
	case 6:
		Tx = K*Uz ;
		Ty = -K*Ux ;
		break;
	default:/*打印报错信息*/
		break;
	}
    	Tz = 0.5f*(1-Tx-Ty) ;
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}

	switch(motor->MotorAlg.Sector)
	{
		case 1 : 
			Tc = Tz ;
			Tb = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		case 2 : 
			Tc = Tz ;
			Ta = Tz + Ty ;
			Tb = Tz + Ty + Tx ;			
			break;
		case 3 : 
			Ta = Tz ;
			Tc = Tz + Ty ;
			Tb = Tz + Ty + Tx ;	
			break;
		case 4 : 
			Ta = Tz ;
			Tb = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 5 : 
			Tb = Tz ;
			Ta = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 6 : 
			Tb = Tz ;
			Tc = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		default:
			break;
	}

	float a = mymap(Ta,-1,1,0,1);
	float b = mymap(Tb,-1,1,0,1);
	float c = mymap(Tc,-1,1,0,1);

    set_pwm(motor,a,b,c);
}

void set_svpwm(Motor_HandleTypeDef *motor, float Uq , float Ud ,float angle_el)
{
    static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;
    float Ualpha = 0 , Ubeta = 0 ;
    
    float *Upark = Calculate_Park_N(Uq , Ud , angle_el);
    Ualpha = Upark[0];
    Ubeta  = Upark[1];
    
    int Sector = Calculate_Sector(Ualpha , Ubeta);
    
    K=(sqrt_3*1)/(motor->MotorConfig.UMAX/2);
    Ux = Ubeta;
    Uy = (sqrt_3/2.0f)*Ualpha - 0.5f*Ubeta;
    Uz = (sqrt_3/2.0f)*Ualpha + 0.5f*Ubeta;

    switch (Sector)
	{
        case 1:
            Tx = K*Uy ;
            Ty = K*Ux ;
            break;
        case 2:
            Tx = -K*Uy ;
            Ty = K*Uz ;
            break;
        case 3:
            Tx = K*Ux ;
            Ty = -K*Uz ;
            break;
        case 4:
            Tx = -K*Ux ;
            Ty = -K*Uy ;
            break;
        case 5:
            Tx = -K*Uz ;
            Ty = K*Uy ;
            break;
        case 6:
            Tx = K*Uz ;
            Ty = -K*Ux ;
            break;
        default:
            break;
	}
    	Tz = 0.5f*(1-Tx-Ty) ;
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}

	switch(Sector)
	{
		case 1 : 
			Tc = Tz ;
			Tb = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		case 2 : 
			Tc = Tz ;
			Ta = Tz + Ty ;
			Tb = Tz + Ty + Tx ;			
			break;
		case 3 : 
			Ta = Tz ;
			Tc = Tz + Ty ;
			Tb = Tz + Ty + Tx ;	
			break;
		case 4 : 
			Ta = Tz ;
			Tb = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 5 : 
			Tb = Tz ;
			Ta = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 6 : 
			Tb = Tz ;
			Tc = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		default:
			break;
	}

	float a = mymap(Ta,-1,1,0,1);
	float b = mymap(Tb,-1,1,0,1);
	float c = mymap(Tc,-1,1,0,1);
    set_pwm_nodir(motor,a,b,c);
}

void set_svpwm_dir(Motor_HandleTypeDef *motor, float Uq , float Ud ,float angle_el)
{
    static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;
    float Ualpha = 0 , Ubeta = 0 ;
    
    float *Upark = Calculate_Park_N(Uq , Ud , angle_el);
    Ualpha = Upark[0];
    Ubeta  = Upark[1];

    motor->MotorAlg.Ualpha = Upark[0];//打印
    motor->MotorAlg.Ubeta = Upark[1];
    
    int Sector = Calculate_Sector(Ualpha , Ubeta);
    
    K=(sqrt_3*1)/(motor->MotorConfig.UMAX/2);
    Ux = Ubeta;
    Uy = (sqrt_3/2.0f)*Ualpha - 0.5f*Ubeta;
    Uz = (sqrt_3/2.0f)*Ualpha + 0.5f*Ubeta;

    switch (Sector)
	{
        case 1:
            Tx = K*Uy ;
            Ty = K*Ux ;
            break;
        case 2:
            Tx = -K*Uy ;
            Ty = K*Uz ;
            break;
        case 3:
            Tx = K*Ux ;
            Ty = -K*Uz ;
            break;
        case 4:
            Tx = -K*Ux ;
            Ty = -K*Uy ;
            break;
        case 5:
            Tx = -K*Uz ;
            Ty = K*Uy ;
            break;
        case 6:
            Tx = K*Uz ;
            Ty = -K*Ux ;
            break;
        default:
            break;
	}
    	Tz = 0.5f*(1-Tx-Ty) ;
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}

	switch(Sector)
	{
		case 1 : 
			Tc = Tz ;
			Tb = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		case 2 : 
			Tc = Tz ;
			Ta = Tz + Ty ;
			Tb = Tz + Ty + Tx ;			
			break;
		case 3 : 
			Ta = Tz ;
			Tc = Tz + Ty ;
			Tb = Tz + Ty + Tx ;	
			break;
		case 4 : 
			Ta = Tz ;
			Tb = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 5 : 
			Tb = Tz ;
			Ta = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 6 : 
			Tb = Tz ;
			Tc = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		default:
			break;
	}

    motor->MotorAlg.UA = Ta*motor->MotorConfig.UMAX - motor->MotorConfig.UMAX/2;
    motor->MotorAlg.UB = Tb*motor->MotorConfig.UMAX - motor->MotorConfig.UMAX/2;
    motor->MotorAlg.UC = Tc*motor->MotorConfig.UMAX - motor->MotorConfig.UMAX/2;

	float a = mymap(Ta,-1,1,0,1);
	float b = mymap(Tb,-1,1,0,1);
	float c = mymap(Tc,-1,1,0,1);
    set_pwm(motor,a,b,c);
}

void update_spwm(Motor_HandleTypeDef *motor)
{
    update_Park_N(motor);
    update_Clark_N(motor);

    float TA = mymap(motor->MotorAlg.UA,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TB = mymap(motor->MotorAlg.UB,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TC = mymap(motor->MotorAlg.UC,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    
    set_pwm(motor,TA, TB, TC);
}

void set_spwm(Motor_HandleTypeDef *motor,float Uq, float Ud ,float angle_el)
{
    float *Upark = Calculate_Park_N(Uq , Ud , angle_el);
    float *Uclark = Calculate_Clark_N(Upark[0] , Upark[1] , motor->MotorConfig.UMAX);
    
    Uclark[0]= Uclark[0] - motor->MotorConfig.UMAX/2;
    Uclark[1]= Uclark[1] - motor->MotorConfig.UMAX/2;
    Uclark[2]= Uclark[2] - motor->MotorConfig.UMAX/2;

    // motor->MotorAlg.UA = Uclark[0];
    // motor->MotorAlg.UB = Uclark[1];
    // motor->MotorAlg.UC = Uclark[2];
    
    float TA = mymap(Uclark[0],-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TB = mymap(Uclark[1],-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TC = mymap(Uclark[2],-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);

    set_pwm(motor,TA, TB, TC);
    // set_pwm_nodir(motor,TA, TB, TC);
}

float get_dt(Motor_HandleTypeDef *motor)
{
    return motor->time.dt;
}

float update_dt(Motor_HandleTypeDef *motor)
{
    motor->MotorDrv.Update_dt(&motor->time);
    return motor->time.dt;
}

float update_IaIbIc(Motor_HandleTypeDef *motor)
{
    static float Ia_ = 0 , Ib_ = 0 , Ic_ = 0 ;
    if(motor->MotorDrv.Update_Ia_raw!=NULL | motor->MotorDrv.Update_Ib_raw!=NULL | motor->MotorDrv.Update_Ic_raw!=NULL)
    {
        motor->MotorData.CurrentData.I_raw.IA_raw = motor->MotorDrv.Update_Ia_raw();
        motor->MotorData.CurrentData.I_raw.IB_raw = motor->MotorDrv.Update_Ib_raw();
        motor->MotorData.CurrentData.I_raw.IC_raw = motor->MotorDrv.Update_Ic_raw();
        
        motor->MotorAlg.IA = motor->MotorDrv.Cal_Ia(motor->MotorData.CurrentData.I_raw.IA_raw , motor->MotorData.IA_offset_raw);
        motor->MotorAlg.IB = motor->MotorDrv.Cal_Ib(motor->MotorData.CurrentData.I_raw.IB_raw , motor->MotorData.IB_offset_raw);
        motor->MotorAlg.IC = motor->MotorDrv.Cal_Ic(motor->MotorData.CurrentData.I_raw.IC_raw , motor->MotorData.IC_offset_raw);
        
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                Ia_ = motor->MotorAlg.IA ;
                Ib_ = motor->MotorAlg.IB ;
                Ic_ = motor->MotorAlg.IC ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 2:
            {
                Ia_ = motor->MotorAlg.IB ;
                Ib_ = motor->MotorAlg.IA ;
                Ic_ = motor->MotorAlg.IC ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 3:
            {
                Ia_ = motor->MotorAlg.IC ;
                Ib_ = motor->MotorAlg.IB ;
                Ic_ = motor->MotorAlg.IA ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 4:
            {
                Ia_ = motor->MotorAlg.IC ;
                Ib_ = motor->MotorAlg.IA ;
                Ic_ = motor->MotorAlg.IB ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 5:
            {
                Ia_ = motor->MotorAlg.IB ;
                Ib_ = motor->MotorAlg.IC ;
                Ic_ = motor->MotorAlg.IA ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 6:
            {
                Ia_ = motor->MotorAlg.IA ;
                Ib_ = motor->MotorAlg.IC ;
                Ic_ = motor->MotorAlg.IB ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            default:
            {
                Ia_ = motor->MotorAlg.IA ;
                Ib_ = motor->MotorAlg.IB ;
                Ic_ = motor->MotorAlg.IC ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;

        }

        return 1 ;
    }
    else
    {
        /*打印报错信息*/
        return 0 ;
    }
}

float get_Ia(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.IA;
}

float get_Ib(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.IB;
}

float get_Ic(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.IC;
}

void update_Ioffset_block(Motor_HandleTypeDef *motor)
{
    for(int i=0 ; i<1000 ; i++)
    {
        motor->MotorData.IA_offset_raw += motor->MotorDrv.Update_Ia_raw();
        motor->MotorData.IB_offset_raw += motor->MotorDrv.Update_Ib_raw();
        motor->MotorData.IC_offset_raw += motor->MotorDrv.Update_Ic_raw();
    }
    motor->MotorData.IA_offset_raw = motor->MotorData.IA_offset_raw/1000 ;
    motor->MotorData.IB_offset_raw = motor->MotorData.IB_offset_raw/1000 ;
    motor->MotorData.IC_offset_raw = motor->MotorData.IC_offset_raw/1000 ;
}

int update_Ioffset_nonblock(Motor_HandleTypeDef *motor)
{
    static int count = 0 ;
    if(count < 1000)
    {
        motor->MotorData.IA_offset_raw += motor->MotorDrv.Update_Ia_raw();
        motor->MotorData.IB_offset_raw += motor->MotorDrv.Update_Ib_raw();
        motor->MotorData.IC_offset_raw += motor->MotorDrv.Update_Ib_raw();
        count++;
        return 0 ;
    }
    else if(count >= 1000)
    {
        motor->MotorData.IA_offset_raw = motor->MotorData.IA_offset_raw/1000 ;
        motor->MotorData.IB_offset_raw = motor->MotorData.IB_offset_raw/1000 ;
        motor->MotorData.IC_offset_raw = motor->MotorData.IC_offset_raw/1000 ;
        return 1 ;
    }
}

float get_Ia_offset(Motor_HandleTypeDef *motor)
{
    return motor->MotorData.IA_offset_raw;
}

float get_Ib_offset(Motor_HandleTypeDef *motor)
{
    return motor->MotorData.IB_offset_raw;
}

float get_Ic_offset(Motor_HandleTypeDef *motor)
{
    return motor->MotorData.IC_offset_raw;
}

void update_pole_pairs_sensor_block(Motor_HandleTypeDef *motor)
{
    float velocity_integral = 0.0f;
    set_svpwm(motor,motor->MotorConfig.UMAX*0.05f,0.0f, 0.0f);
    motor->MotorDrv.Delayms(2000);
    for(int i=0 ; i<1000 ; i++)
    {
        update_dt(motor);  //预热dt，防止因初次启动产生的极小dt干扰后面的速度计算
        update_angle(motor);
        update_velocity_raw(motor);
    }

    for(int i=0 ; i<1000 ; i++)
    {
        update_dt(motor);  //预热dt，防止因初次启动产生的极小dt干扰后面的速度计算
        update_angle(motor);
        update_velocity_raw(motor);
        velocity_integral += motor->MotorData.Velocity_raw*motor->time.dt;
        // printf("%f,%f,%f,%f\n",(angle_end - angle_start),velocity_integral,motor->MotorData.Velocity_raw,motor->MotorAlg.angle);
        set_svpwm(motor,motor->MotorConfig.UMAX*0.05f,0.0f,Limit_angle_el((float)i*0.01f));
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(2000);

    motor->MotorConfig.Pole_pairs = (uint32_t)round(myabs((float)(1000*0.01f)/(velocity_integral)));
    // printf("%d,%f\n",motor->MotorConfig.Pole_pairs,(myabs((float)(1000*0.01f)/(velocity_integral))));
    // printf("%f,%f,%f,%f\n",(angle_end - angle_start),velocity_integral,motor->MotorData.Velocity_raw,motor->MotorAlg.angle);
    set_svpwm(motor,0.0f, 0.0f , 0.0f); 
}

void update_pole_pairs_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    static int flag = 0;
    static float angle_start = 0;
    static float angle_end = 0;
    
    float velocity_target = 3.0f; 
    float time_init = 0.5f;
    float time_prep = 1.0f;
    float time_process = 5.0f;

    static uint8_t state = 0 ;
    static float total_time = 0 ;
    static float velocity_integral = 0.0f;

    if(flag == 0)
    {
        total_time += update_dt(motor);
        update_angle(motor);
        update_velocity_raw(motor);
        if(total_time < time_init)
        {
            state = 0;
        }
        else if(total_time >= time_init && total_time < (time_init+time_prep))
        {
            state = 1;
        }
        else if(total_time >= (time_init+time_prep) && total_time < (time_init+time_prep+time_process))
        {
            state = 2;
        }
        else if(total_time >= (time_init+time_prep+time_process))
        {
            state = 3;
        }
        
        switch (state)
        {
            case 0:
            {
                set_svpwm(motor,0.0f, 0.0f , 0.0f);
            }break;
            case 1:
            {
                set_svpwm(motor, motor->MotorConfig.UMAX*0.5f,0.0f,0.0f);
                angle_start = motor->MotorAlg.angle;
            }break;
            case 2:
            {
                velocity_integral += motor->MotorData.Velocity_raw*motor->time.dt;
                set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0.0f, Limit_angle_el((float)velocity_target*(total_time-time_init-time_prep)));
                // motor->MotorDrv.Delayms(1);
                angle_end = motor->MotorAlg.angle;
            }break;
            case 3:
            {
                angle_end = motor->MotorAlg.angle;
                motor->MotorConfig.Pole_pairs = (uint32_t)round(myabs((float)velocity_target*(total_time-time_init-time_prep)/(velocity_integral)));
                set_svpwm(motor,0.0f, 0.0f , 0.0f);
                // total_time = 0.0f;
                // velocity_integral = 0.0f;
                flag = 1;
                // printf("%d\n",motor->MotorConfig.Pole_pairs);
            }
            default:
            {
                /* 打印报错信息 */
            }break;
        }
    }
    printf("%d\n",motor->MotorConfig.Pole_pairs);

}

// ============================================================
// 相序辨识 —— 第一轮：传感器方向（编码器正/反转）
//
// 原理：用 set_pwm_nodir 以固定物理相序驱动，绕过 DIR 换相，
//       积分编码器速度判断方向。
// 结果：DIR=1（正序组候选 1/4/5）或 DIR=2（逆序组候选 2/3/6）
// ============================================================
void update_2DIR_sensor_block(Motor_HandleTypeDef *motor)
{
    const float velocity_target = 0.03f;
    float velocity_integral = 0.0f;

    // 用 set_pwm_nodir 做 d 轴励磁斜坡预热，绕过 DIR 换相，
    // 保证物理 A 相电流最大，转子对齐到固定位置
    for (int i = 0; i < 1000; i++)
    {
        update_dt(motor);
        update_angle(motor);
        update_velocity_raw(motor);
        float k = (float)(i + 1) / 1000.0f;
        // Clark 逆变换在 θ=0 时：Ua=2/3*Ud, Ub=Uc=-1/3*Ud
        // 直接用 set_svpwm 的底层计算，但通过 set_pwm_nodir 输出，绕过 DIR 换相
        float *up = Calculate_Park_N(0.0f, k * motor->MotorConfig.UMAX * 0.05f, 0.0f);
        float *uc = Calculate_Clark_N(up[0], up[1], motor->MotorConfig.UMAX);
        float ta = mymap(uc[0], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
        float tb = mymap(uc[1], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
        float tc = mymap(uc[2], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
        set_pwm_nodir(motor, ta, tb, tc);
        motor->MotorDrv.Delayms(1);
    }

    // 正向开环旋转，同样用 set_pwm_nodir 绕过 DIR 换相，
    // 保证驱动方向固定，通过编码器反馈速度积分判断传感器方向
    for (int i = 0; i < 2000; i++)
    {
        update_dt(motor);
        update_angle(motor);
        update_velocity_raw(motor);
        float angle_cmd = Limit_angle_el((float)i * velocity_target);
        float *up = Calculate_Park_N(motor->MotorConfig.UMAX * 0.05f, 0.0f, angle_cmd);
        float *uc = Calculate_Clark_N(up[0], up[1], motor->MotorConfig.UMAX);
        float ta = mymap(uc[0], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
        float tb = mymap(uc[1], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
        float tc = mymap(uc[2], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
        set_pwm_nodir(motor, ta, tb, tc);
        velocity_integral += motor->MotorData.Velocity_raw * motor->time.dt;
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(300);
    set_pwm_nodir(motor, 0.0f, 0.0f, 0.0f);

    if (velocity_integral > 0.0f)
    {
        motor->MotorConfig.DIR = 1;
        SEGGER_RTT_printf(0, "[DIR Round1] Sensor forward, DIR group=1/4/5\r\n");
    }
    else if (velocity_integral < 0.0f)
    {
        motor->MotorConfig.DIR = 2;
        SEGGER_RTT_printf(0, "[DIR Round1] Sensor reverse, DIR group=2/3/6\r\n");
    }
    else
    {
        SEGGER_RTT_printf(0, "[DIR Round1] Failed: no movement detected\r\n");
    }
}

// ============================================================
// 相序辨识 —— 第一轮（非阻塞版）：传感器方向
// 状态机：0-空闲  1-励磁预热  2-正向旋转采样  3-判断写结果
// ============================================================
void update_2DIR_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    const float velocity_target = 10.0f;
    const float time_init  = 0.2f;
    const float time_prep  = 0.8f;
    const float time_run   = 2.0f;

    static uint8_t state = 0;
    static float total_time = 0.0f;
    static float velocity_integral = 0.0f;

    total_time += update_dt(motor);
    update_angle(motor);
    update_velocity_raw(motor);

    if      (total_time < time_init)                        { state = 0; }
    else if (total_time < time_init + time_prep)            { state = 1; }
    else if (total_time < time_init + time_prep + time_run) { state = 2; }
    else                                                    { state = 3; }

    switch (state)
    {
        case 0:
            set_pwm_nodir(motor, 0.0f, 0.0f, 0.0f);
            break;

        case 1:
        {
            // d 轴励磁预热，用 set_pwm_nodir 绕过 DIR 换相
            float k = (total_time - time_init) / time_prep;
            float *up = Calculate_Park_N(0.0f, k * motor->MotorConfig.UMAX * 0.05f, 0.0f);
            float *uc = Calculate_Clark_N(up[0], up[1], motor->MotorConfig.UMAX);
            float ta = mymap(uc[0], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
            float tb = mymap(uc[1], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
            float tc = mymap(uc[2], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
            set_pwm_nodir(motor, ta, tb, tc);
        }
        break;

        case 2:
        {
            // 正向开环旋转，用 set_pwm_nodir 绕过 DIR 换相
            float angle_cmd = Limit_angle_el((total_time - time_init - time_prep) * velocity_target);
            float *up = Calculate_Park_N(motor->MotorConfig.UMAX * 0.05f, 0.0f, angle_cmd);
            float *uc = Calculate_Clark_N(up[0], up[1], motor->MotorConfig.UMAX);
            float ta = mymap(uc[0], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
            float tb = mymap(uc[1], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
            float tc = mymap(uc[2], -motor->MotorConfig.UMAX/2, motor->MotorConfig.UMAX/2, 0.0f, 1.0f);
            set_pwm_nodir(motor, ta, tb, tc);
            velocity_integral += motor->MotorData.Velocity_raw * motor->time.dt;
        }
        break;

        case 3:
        {
            set_pwm_nodir(motor, 0.0f, 0.0f, 0.0f);
            if (velocity_integral > 0.0f)
            {
                motor->MotorConfig.DIR = 1;
                SEGGER_RTT_printf(0, "[DIR Round1] Sensor forward, DIR group=1/4/5\r\n");
            }
            else if (velocity_integral < 0.0f)
            {
                motor->MotorConfig.DIR = 2;
                SEGGER_RTT_printf(0, "[DIR Round1] Sensor reverse, DIR group=2/3/6\r\n");
            }
            else
            {
                SEGGER_RTT_printf(0, "[DIR Round1] Failed: no movement detected\r\n");
            }
            total_time = 0.0f;
            velocity_integral = 0.0f;
            state = 0;
        }
        break;

        default:
            set_pwm_nodir(motor, 0.0f, 0.0f, 0.0f);
            break;
    }
}

// ============================================================
// 相序辨识 —— 第二轮（阻塞式）：电流注入法辨识接线顺序
//
// 原理（下桥臂采样注入法）：
//   依次激励物理 A、B、C 三相：
//     激励 X 相 = 令其他两相上桥臂全开（占空比=1），X 相下桥臂小占空比导通
//     电流路径：VCC → 其他两相 → 绕组 → X 相 → 采样电阻 → GND
//   读取三路 ADC 原始值，响应最大的那路就是 X 相的采样通道。
//   重复三次建立完整映射：ADC_RANK_1/2/3 各对应哪根物理线。
//   再查表得到正确的 DIR（同时包含 PWM 换相和电流换相）。
//
// 前提：
//   已完成第一轮传感器方向辨识，motor->MotorConfig.DIR 已为 1 或 2。
//   候选范围：DIR=1 → {1,4,5}；DIR=2 → {2,3,6}
//
// 注意：
//   - 注入占空比用小值（INJECT_DUTY），避免过流
//   - 完全不依赖 angle_el_zero，不需要电机转动
//   - 直接操作 Set_PWM_A/B/C，绕过所有换相逻辑
// ============================================================

// 注入占空比：其他两相上桥臂，X 相下桥臂
// 上桥臂接近 1，下桥臂用小值，电流 ≈ UMAX * INJECT_DUTY / Rs
#define DIR_INJECT_DUTY_HIGH  0.9f
#define DIR_INJECT_DUTY_LOW   0.1f
#define DIR_INJECT_SAMPLE_N   100
#define DIR_INJECT_SETTLE_MS  50

// 读取三路 ADC 原始值（不做换相，直接读硬件通道）
static void read_raw_iabc(Motor_HandleTypeDef *motor, uint32_t *ra, uint32_t *rb, uint32_t *rc)
{
    *ra = motor->MotorDrv.Update_Ia_raw();
    *rb = motor->MotorDrv.Update_Ib_raw();
    *rc = motor->MotorDrv.Update_Ic_raw();
}

void update_6DIR_current_block(Motor_HandleTypeDef *motor)
{
    // 候选组（由第一轮传感器方向决定）
    const int candidates_fwd[3] = {1, 4, 5};
    const int candidates_rev[3] = {2, 3, 6};
    const int *candidates = (motor->MotorConfig.DIR == 1) ? candidates_fwd : candidates_rev;

    // 三相注入配置：[激励相][A占空比, B占空比, C占空比]
    // 激励 A 相：B/C 上桥臂全开，A 下桥臂小占空比
    // 激励 B 相：A/C 上桥臂全开，B 下桥臂小占空比
    // 激励 C 相：A/B 上桥臂全开，C 下桥臂小占空比
    const float inject_pwm[3][3] = {
        {DIR_INJECT_DUTY_LOW,  DIR_INJECT_DUTY_HIGH, DIR_INJECT_DUTY_HIGH}, // 激励物理 A
        {DIR_INJECT_DUTY_HIGH, DIR_INJECT_DUTY_LOW,  DIR_INJECT_DUTY_HIGH}, // 激励物理 B
        {DIR_INJECT_DUTY_HIGH, DIR_INJECT_DUTY_HIGH, DIR_INJECT_DUTY_LOW},  // 激励物理 C
    };

    // 采样结果：inject_result[激励相][ADC通道] = 平均原始值
    float inject_result[3][3] = {{0}};

    SEGGER_RTT_printf(0, "[DIR Inject] Start current injection identification\r\n");

    for (int ph = 0; ph < 3; ph++)
    {
        // 施加注入 PWM（直接操作硬件，绕过 DIR 换相）
        motor->MotorDrv.Set_PWM_A(inject_pwm[ph][0]);
        motor->MotorDrv.Set_PWM_B(inject_pwm[ph][1]);
        motor->MotorDrv.Set_PWM_C(inject_pwm[ph][2]);
        motor->MotorDrv.Delayms(DIR_INJECT_SETTLE_MS);

        // 多次采样取平均，消除 ADC 噪声
        float sum_a = 0, sum_b = 0, sum_c = 0;
        for (int s = 0; s < DIR_INJECT_SAMPLE_N; s++)
        {
            uint32_t ra, rb, rc;
            read_raw_iabc(motor, &ra, &rb, &rc);
            // 取与零偏的差值绝对值（电流幅值）
            sum_a += (float)((ra > motor->MotorData.IA_offset_raw)
                             ? (ra - motor->MotorData.IA_offset_raw)
                             : (motor->MotorData.IA_offset_raw - ra));
            sum_b += (float)((rb > motor->MotorData.IB_offset_raw)
                             ? (rb - motor->MotorData.IB_offset_raw)
                             : (motor->MotorData.IB_offset_raw - rb));
            sum_c += (float)((rc > motor->MotorData.IC_offset_raw)
                             ? (rc - motor->MotorData.IC_offset_raw)
                             : (motor->MotorData.IC_offset_raw - rc));
        }
        inject_result[ph][0] = sum_a / DIR_INJECT_SAMPLE_N;
        inject_result[ph][1] = sum_b / DIR_INJECT_SAMPLE_N;
        inject_result[ph][2] = sum_c / DIR_INJECT_SAMPLE_N;

        SEGGER_RTT_printf(0, "[DIR Inject] Ph%d: ADC_A=%d ADC_B=%d ADC_C=%d\r\n",
                          ph,
                          (int)inject_result[ph][0],
                          (int)inject_result[ph][1],
                          (int)inject_result[ph][2]);

        // 关闭 PWM，等待电流衰减
        motor->MotorDrv.Set_PWM_A(0.0f);
        motor->MotorDrv.Set_PWM_B(0.0f);
        motor->MotorDrv.Set_PWM_C(0.0f);
        motor->MotorDrv.Delayms(50);
    }

    // 建立映射：对每个激励相，找响应最大的 ADC 通道
    // adc_to_phys[adc_ch] = 物理相（0=A,1=B,2=C）
    // phys_to_adc[phys_ph] = ADC 通道（0=RANK1,1=RANK2,2=RANK3）
    int phys_to_adc[3] = {-1, -1, -1};
    for (int ph = 0; ph < 3; ph++)
    {
        int best_ch = 0;
        float best_val = inject_result[ph][0];
        if (inject_result[ph][1] > best_val) { best_val = inject_result[ph][1]; best_ch = 1; }
        if (inject_result[ph][2] > best_val) { best_val = inject_result[ph][2]; best_ch = 2; }
        phys_to_adc[ph] = best_ch;
    }

    SEGGER_RTT_printf(0, "[DIR Inject] Mapping: PhysA->ADC%d, PhysB->ADC%d, PhysC->ADC%d\r\n",
                      phys_to_adc[0], phys_to_adc[1], phys_to_adc[2]);

    // 根据 phys_to_adc 映射和候选组，查找正确的 DIR
    // DIR 的 set_pwm 换相定义（Ua->物理相，即 ADC 通道对应关系）：
    //   DIR=1: Ua->A(adc0), Ub->B(adc1), Uc->C(adc2)  → phys_to_adc = {0,1,2}
    //   DIR=2: Ua->B(adc1), Ub->A(adc0), Uc->C(adc2)  → phys_to_adc = {1,0,2}
    //   DIR=3: Ua->C(adc2), Ub->B(adc1), Uc->A(adc0)  → phys_to_adc = {2,1,0}
    //   DIR=4: Ua->C(adc2), Ub->A(adc0), Uc->B(adc1)  → phys_to_adc = {2,0,1}
    //   DIR=5: Ua->B(adc1), Ub->C(adc2), Uc->A(adc0)  → phys_to_adc = {1,2,0}
    //   DIR=6: Ua->A(adc0), Ub->C(adc2), Uc->B(adc1)  → phys_to_adc = {0,2,1}
    // 其中 phys_to_adc[ph] 表示：激励物理 ph 相时，哪路 ADC 响应最大
    // 即 phys_to_adc[ph] = 物理 ph 相对应的 ADC 通道编号
    //
    // set_pwm 中 DIR=N 时：Set_PWM_A 输出 Ua，Set_PWM_B 输出 Ub，Set_PWM_C 输出 Uc
    // 所以物理 A 相（ADC_A 采样的那相）接收的是哪路 Ux，由 phys_to_adc[0] 决定：
    //   phys_to_adc[0]=0 → ADC_A 采物理 A → Ua 给 A → DIR=1 或 DIR=6
    //   phys_to_adc[0]=1 → ADC_B 采物理 A → Ub 给 A → DIR=2 或 DIR=5
    //   phys_to_adc[0]=2 → ADC_C 采物理 A → Uc 给 A → DIR=3 或 DIR=4
    // 再结合 phys_to_adc[1] 区分同组内的两个候选

    // DIR 对应的 phys_to_adc 期望值表
    // dir_map[DIR-1] = {phys_to_adc[0], phys_to_adc[1], phys_to_adc[2]}
    const int dir_map[6][3] = {
        {0, 1, 2}, // DIR=1: A->adc0, B->adc1, C->adc2
        {1, 0, 2}, // DIR=2: A->adc1, B->adc0, C->adc2
        {2, 1, 0}, // DIR=3: A->adc2, B->adc1, C->adc0
        {2, 0, 1}, // DIR=4: A->adc2, B->adc0, C->adc1
        {1, 2, 0}, // DIR=5: A->adc1, B->adc2, C->adc0
        {0, 2, 1}, // DIR=6: A->adc0, B->adc2, C->adc1
    };

    int best_dir = candidates[0];
    for (int ci = 0; ci < 3; ci++)
    {
        int dir_try = candidates[ci];
        const int *expected = dir_map[dir_try - 1];
        if (phys_to_adc[0] == expected[0] &&
            phys_to_adc[1] == expected[1] &&
            phys_to_adc[2] == expected[2])
        {
            best_dir = dir_try;
            break;
        }
    }

    motor->MotorConfig.DIR = best_dir;
    SEGGER_RTT_printf(0, "[DIR Inject] Done, DIR=%d\r\n", best_dir);
}

// ============================================================
// Flash 存读 DIR
//
// 格式（64-bit doubleword，Flash 最小写入单元）：
//   高 32 位：魔数 DIR_FLASH_MAGIC（用于有效性校验）
//   低 32 位：DIR 值（1~6）
//
// save_dir_to_flash：将当前 DIR 写入 DIR_FLASH_ADDR
// load_dir_from_flash：从 Flash 读取 DIR，魔数匹配则写入 motor，否则保留默认值
// ============================================================
void save_dir_to_flash(Motor_HandleTypeDef *motor)
{
    uint64_t data = ((uint64_t)DIR_FLASH_MAGIC << 32) | (uint32_t)motor->MotorConfig.DIR;
    // 先擦除该扇区（page 1，长度 1 页）
    flash_erase_address(1, 1);
    flash_write_single_address(DIR_FLASH_ADDR, &data, 1);
    SEGGER_RTT_printf(0, "[DIR Flash] Saved DIR=%d to Flash addr=0x%08X\r\n",
                      motor->MotorConfig.DIR, DIR_FLASH_ADDR);
}

void load_dir_from_flash(Motor_HandleTypeDef *motor)
{
    uint32_t buf[2] = {0};
    flash_read(DIR_FLASH_ADDR, buf, 2);
    // buf[0] = 低 32 位（DIR 值），buf[1] = 高 32 位（魔数）
    // 注意 STM32 小端：doubleword 低地址存低 32 位
    uint32_t magic = buf[1];
    uint32_t dir   = buf[0];

    if (magic == DIR_FLASH_MAGIC && dir >= 1 && dir <= 6)
    {
        motor->MotorConfig.DIR = (int)dir;
        SEGGER_RTT_printf(0, "[DIR Flash] Loaded DIR=%d from Flash\r\n", dir);
    }
    else
    {
        SEGGER_RTT_printf(0, "[DIR Flash] No valid data (magic=0x%08X), keep default DIR=%d\r\n",
                          magic, motor->MotorConfig.DIR);
    }
}

void update_angle_el_zero_no_sensor_block(Motor_HandleTypeDef *motor)
{
    motor->MotorDrv.Delayms(1000);
    set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.05f , 0.0f);
    motor->MotorDrv.Delayms(1000);
    update_angle(motor);
    motor->MotorConfig.angle_el_zero = -Calculate_angle_el(motor->MotorConfig.Pole_pairs,motor->MotorAlg.angle, 0.0f);
    motor->MotorDrv.Delayms(1000);
    set_svpwm(motor,0.0f, 0.0f , 0.0f);
    motor->MotorDrv.Delayms(1000);
}

void update_angle_el_zero_sensor_block(Motor_HandleTypeDef *motor)
{
    uint32_t sample_per = 20 ; //每极对采样20个
    uint32_t sample_total = motor->MotorConfig.Pole_pairs * sample_per; //总采样数
    float *angle_el_zero = (float*)calloc(sample_total,sizeof(float));  //按照采样数定义动态数组
    float angle_el_zero_all = 0;

    float angle_all_temp = motor->MotorData.angle_all;//保存现场
    float angle_last_temp = motor->MotorAlg.last_angle;
    float angle_temp = motor->MotorAlg.angle;

    reset_data_angle(motor);

    static float angle_debug_0 = 0;//调试用变量
    static float angle_debug_1 = 0;//调试用变量
    static float angle_debug_2 = 0;//调试用变量
    
    if(angle_el_zero == NULL)
    {   
        //打印报错信息
        SEGGER_RTT_printf(0, "Heap_Size is not enough!\n");
        free((void*)angle_el_zero);
        return;
    }

    set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.05f , 0.0f);
    motor->MotorDrv.Delayms(1000);
    // motor->MotorData.angle_all = update_angle(motor);
    // float angle_last = motor->MotorData.angle_all;
    float angle_now = 0.0f;
    do
    {
        update_dt(motor);
        update_angle(motor);
        angle_now = ctrl_motor_openloop_angle_nonblock(motor,2*PI,0.0f,0.6,0.0f,motor->MotorConfig.UMAX*0.05f);
        // angle_error = angle_now - motor->MotorData.angle_all;
        uint32_t i =(uint32_t)round((angle_now/(2*PI))*(float)sample_total);
        
        if(i>=sample_total|| i<0)
        {
            ctrl_motor_openloop_angle_nonblock(motor,0.0f,0.0f,1000.0f,0.0f,0.0f);//注销掉这个函数的angle_now，防止用到下一次开环执行程序中
            break;
        }
        angle_el_zero[i] = angle_now - motor->MotorData.angle_all;
        angle_debug_0 = angle_now;
        angle_debug_1 = motor->MotorData.angle_all;
        // printf("%d,%f,%f\n",i,angle_el_zero[i],angle_now);

    } while (angle_now);
    motor->MotorDrv.Delayms(500);
    do
    {
        update_dt(motor);
        update_angle(motor);
        angle_now = ctrl_motor_openloop_angle_nonblock(motor,0.0f,2*PI,-0.6,0.0f,motor->MotorConfig.UMAX*0.05f);
        // angle_error = angle_now - motor->MotorData.angle_all;
        uint32_t i =(uint32_t)round((angle_now/(2*PI))*(float)sample_total);
        if(i>=sample_total || i<0)
        {
            ctrl_motor_openloop_angle_nonblock(motor,0.0f,0.0f,1000.0f,0.0f,0.0f);//注销掉这个函数的angle_now，防止用到下一次开环执行程序中
            break;
        }
        angle_el_zero[i] += angle_now - motor->MotorData.angle_all;
        angle_el_zero[i] /= 2;

        angle_debug_0 = angle_now;
        angle_debug_1 = motor->MotorData.angle_all;
        // printf("%d,%f,%f\n",i,angle_el_zero[i],angle_now);
    } while (angle_now);
    // printf("%f\n",angle_now);
    for(int i = 0; i<sample_total ;i++)
    {
        angle_el_zero_all += angle_el_zero[i];
    }
    angle_debug_2 = angle_el_zero_all/(float)sample_total;
    // motor->MotorConfig.angle_el_zero = angle_el_zero_all/(float)sample_total;
    motor->MotorConfig.angle_el_zero = Calculate_angle_el(motor->MotorConfig.Pole_pairs,angle_el_zero_all/(float)sample_total, 0.0f);
    
    motor->MotorData.angle_all = angle_all_temp ;//返回现场
    motor->MotorAlg.last_angle = angle_last_temp;
    motor->MotorAlg.angle = angle_temp;

    set_svpwm(motor,0.0f, 0.0f , 0.0f);
    free((void*)angle_el_zero);

}

void update_angle_el_zero_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    static float flag = 1 ;

    uint32_t sample_per = 50 ; //每极对采样100个
    uint32_t sample_total = motor->MotorConfig.Pole_pairs * sample_per; //总采样数
    static float *angle_el_zero = NULL;  //按照采样数定义动态数组
    static float  angle_el_zero_all = 0;
    static float angle_all_temp;

    static uint8_t state = 1 ;
    static float total_time = 0.0f ;
    static float angle_now = 0.0f;

    total_time += update_dt(motor); //预热
    update_angle(motor);
    angle_all_temp = motor->MotorData.angle_all ;
    motor->MotorData.angle_all = 0;

    switch(state)
    {
        case 1:
        {
            angle_el_zero = (float*)calloc(sample_total,sizeof(float));
            if(angle_el_zero == NULL)
            {   
                //打印报错信息
                printf("Heap_Size is not enough!");
                free((void*)angle_el_zero);
                return;
            }
            else
            {
                state = 2;
                set_svpwm(motor,motor->MotorConfig.UMAX*0.5f, 0.0f , 0.0f);
            }
            
        }break;
        case 2:
        {
            angle_now = ctrl_motor_openloop_angle_nonblock(motor,2*PI,0.0f,0.3,motor->MotorConfig.UMAX*0.5f, 0.0f);
            uint32_t i =(uint32_t)round((angle_now/(2*PI))*(float)sample_total);
            // printf("%f\n",angle_el_zero[i]);

            if(i>=sample_total|| !angle_now)
            {
                ctrl_motor_openloop_angle_nonblock(motor,0.0f,0.0f,1000.0f,0.0f,0.0f);//注销掉这个函数的angle_now，防止用到下一次开环执行程序中
                state = 3;
                break;
            }
            angle_el_zero[i] = angle_now - motor->MotorData.angle_all;
        }break;
        case 3:
        {
            angle_now = ctrl_motor_openloop_angle_nonblock(motor,0.0f,2*PI,-0.3,motor->MotorConfig.UMAX*0.5f, 0.0f);
            uint32_t i =(uint32_t)round((angle_now/(2*PI))*(float)sample_total);
            // printf("%f\n",angle_el_zero[i]);
            if(i>=sample_total)
            {
                --i;
            }
            if(i<0 || !angle_now)
            {
                ctrl_motor_openloop_angle_nonblock(motor,0.0f,0.0f,1000.0f,0.0f,0.0f);//注销掉这个函数的angle_now，防止用到下一次开环执行程序中
                state = 4;
                break;
            }
            angle_el_zero[i] += angle_now - motor->MotorData.angle_all;
            angle_el_zero[i] /= 2;
        }break;
        case 4:
        {
            for(int i = 0; i<sample_total ;i++)
            {
                angle_el_zero_all += angle_el_zero[i];
            }
            motor->MotorConfig.angle_el_zero = angle_el_zero_all/(float)sample_total;
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            free((void*)angle_el_zero);   
            state = 1;
    
        }break;
        default:
        {
            //打印报错
        }break;
    }
    motor->MotorData.angle_all = angle_all_temp ;
}

void map_samples_to_lut(float *error_arr, int N_SAMPLES, float *lut_arr, int N_LUT)
{
    for (int j = 0; j < N_LUT; j++)  // 遍历 LUT 每个索引
    {
        // 步骤 1：计算当前 LUT 索引对应的机械角度（0~2π）
        float lut_angle = (2 * PI * j) / (float)N_LUT;

        // 步骤 2：定位对应的左采样点 i
        int i = (int)floor( (j * N_SAMPLES) / (float)N_LUT );
        // 处理边界：i 不能超过 N_SAMPLES-1
        i = (i >= N_SAMPLES) ? N_SAMPLES - 1 : i;
        // 右采样点（周期闭环）
        int i_next = (i + 1) % N_SAMPLES;

        // 步骤 3：计算左右采样点的角度
        float sample_angle_i = (2 * PI * i) / (float)N_SAMPLES;
        float sample_angle_next = (2 * PI * i_next) / (float)N_SAMPLES;

        // 步骤 4：计算插值比例（处理 i_next=0 时的周期跨越）
        float delta_angle;
        if (i_next == 0)
        {
            // 最后一个采样点到第一个采样点的角度差（跨越 2π 边界）
            delta_angle = (2 * PI - sample_angle_i) + sample_angle_next;
        }
        else
        {
            delta_angle = sample_angle_next - sample_angle_i;
        }
        float ratio = (lut_angle - sample_angle_i) / delta_angle;
        // 限制 ratio 在 0~1 范围内（避免浮点误差导致越界）
        ratio = (ratio < 0) ? 0 : (ratio > 1) ? 1 : ratio;

        // 步骤 5：线性插值得到 LUT 值
        lut_arr[j] = error_arr[i] * (1 - ratio) + error_arr[i_next] * ratio;
    }
}

void update_NLLUT_encoder_sensor_block(Motor_HandleTypeDef *motor)
{
    int sample_per = 50 ; //每极对采样10个
    int sample_total = motor->MotorConfig.Pole_pairs * sample_per ; //总采样数
    float voltage_calib_factor = 0.05f;
    float *angle_el_zero = (float*)calloc(sample_total,sizeof(float));  //按照采样数定义动态数组
    float angle_el_zero_all = 0;
    if(angle_el_zero == NULL)
    {   
        //打印报错信息
        SEGGER_RTT_printf(0,"into error!\n");
        free((void*)angle_el_zero);
        return;
    }

    set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*voltage_calib_factor , 0.0f);
    motor->MotorDrv.Delayms(2000);
    for(int i = 0; i<sample_total ;i++)
    {
        set_svpwm(motor,0.0f,motor->MotorConfig.UMAX*voltage_calib_factor,Limit_angle_el(i*(2*PI*motor->MotorConfig.Pole_pairs)/sample_total));
        motor->MotorDrv.Delayms(15);
        angle_el_zero[i] = ( ((float)i) / ((float)sample_total) ) * 2*PI - motor->MotorDrv.Cal_Angle( motor->MotorDrv.Update_Angle_raw() );
    }
    motor->MotorDrv.Delayms(500);
    for(int i = sample_total-1;i>=0;i--)
    {
        set_svpwm(motor,0.0f,motor->MotorConfig.UMAX*voltage_calib_factor,Limit_angle_el(i*(2*PI*motor->MotorConfig.Pole_pairs)/sample_total));
        motor->MotorDrv.Delayms(15);
        angle_el_zero[i] += ( ((float)i) / ((float)sample_total) ) * 2*PI - motor->MotorDrv.Cal_Angle( motor->MotorDrv.Update_Angle_raw() );
        angle_el_zero[i] /= 2 ;
    }
    for(int i = 0; i<sample_total ;i++)
    {
        angle_el_zero_all += angle_el_zero[i];
    }
    motor->MotorConfig.angle_el_zero = angle_el_zero_all/(float)sample_total;

    for(int i = 0; i<sample_total ;i++)
    {
        angle_el_zero[i] -= motor->MotorConfig.angle_el_zero; 
    }
    int LUT_total = sizeof(motor->MotorConfig.NLLUT_encoder)/sizeof(float);
    map_samples_to_lut(angle_el_zero,sample_total,motor->MotorConfig.NLLUT_encoder,LUT_total);
    free((void*)angle_el_zero);
    set_svpwm(motor,0,0,0);
}

void update_NLLUT_encoder_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    // 静态变量：保存校准上下文（跨调用不丢失）
    static uint8_t state = 0;                  // 状态机：0-初始化 1-励磁稳定 2-正向采样 3-反向采样 4-算零点/去零漂 5-映射LUT 6-收尾 7-错误
    static float total_time = 0.0f;            // 累计时间（替代Delayms）
    static int sample_count = 0;               // 采样计数器（正向递增，反向递减）
    static int sample_total = 0;               // 总采样数（极对数×10）
    static float *angle_el_zero = NULL;        // 误差采样数组（正向+反向平均）
    static float angle_el_zero_all = 0.0f;     // 零点偏移总和
    static int LUT_total = 0;                  // NLLUT数组长度（预存避免重复计算）

    // 1. 累加控制周期时间差（非阻塞计时核心）
    total_time += get_dt(motor);

    // 2. 状态切换判断（按时间/采样进度触发）
    if (state == 0) {}  // 初始化：首次调用触发，不依赖时间
    else if (state == 1)
    {
        // 励磁稳定：累计0.5s后切换
        if (total_time >= 0.5f)
        {
            total_time = 0.0f;
            sample_count = 0;
            state = 2;
        }
    }
    else if (state == 2)
    {
        // 正向采样：采样数达总采样数后切换
        if (sample_count >= sample_total)
        {
            total_time = 0.0f;
            sample_count = sample_total - 1;
            state = 3;
        }
    }
    else if (state == 3)
    {
        // 反向采样：采样数<0后切换
        if (sample_count < 0)
        {
            total_time = 0.0f;
            state = 4;
        }
    }
    else if (state == 4)
    {
        // 算零点/去零漂：执行一次后切换
        state = 5;
    }
    else if (state == 5)
    {
        // 映射LUT：执行一次后切换
        state = 6;
    }
    else if (state == 6)
    {
        // 收尾：执行一次后切换至结束
        state = 7;
    }

    // 3. 状态机逻辑执行
    switch (state)
    {
        // 状态0：初始化（分配内存、重置参数）
        case 0:
            // 重置上下文
            total_time = 0.0f;
            sample_count = 0;
            angle_el_zero_all = 0.0f;
            sample_total = motor->MotorConfig.Pole_pairs * 10;
            LUT_total = sizeof(motor->MotorConfig.NLLUT_encoder) / sizeof(float);

            // 分配采样数组
            angle_el_zero = (float*)calloc(sample_total, sizeof(float));
            if (angle_el_zero == NULL || LUT_total == 0)
            {
                state = 7;
                break;
            }

            // 启动d轴励磁
            set_svpwm(motor, 0.0f, motor->MotorConfig.UMAX * 0.5f, 0.0f);
            state = 1;
            break;

        // 状态1：d轴励磁稳定（等待转子停稳）
        case 1:
            set_svpwm(motor, 0.0f, motor->MotorConfig.UMAX * 0.5f, 0.0f);
            break;

        // 状态2：正向采样（按顺序采全周期误差）
        case 2:
        {
            // 计算当前目标电角度
            float target_el_angle = Limit_angle_el((float)sample_count * (2 * PI * motor->MotorConfig.Pole_pairs) / (float)sample_total);
            // 驱动至目标角度
            set_svpwm(motor, motor->MotorConfig.UMAX * 0.5f, 0.0f, target_el_angle);
            // 采样误差（理论电角度 - 实际编码器角度）
            angle_el_zero[sample_count] = ( (float)sample_count / (float)sample_total ) * 2 * PI- motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());      
            sample_count++;
            total_time = 0.0f;  // 确保采样间隔
        }
        break;

        // 状态3：反向采样（逆序采样，误差平均）
        case 3:
        {
            // 计算当前目标电角度（同正向对应点）
            float target_el_angle = Limit_angle_el((float)sample_count * (2 * PI * motor->MotorConfig.Pole_pairs) / (float)sample_total);
            // 驱动至目标角度
            set_svpwm(motor, motor->MotorConfig.UMAX * 0.5f, 0.0f, target_el_angle);
            // 采样反向误差并与正向平均
            float reverse_error = ( (float)sample_count / (float)sample_total ) * 2 * PI- motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());angle_el_zero[sample_count] = (angle_el_zero[sample_count] + reverse_error) / 2.0f;
            sample_count--;
            total_time = 0.0f;  // 确保采样间隔
        }
        break;

        // 状态4：计算零点偏移 + 误差去零漂
        case 4:
            // 算角度零点偏移（angle_el_zero）
            for (int i = 0; i < sample_total; i++)
            {
                angle_el_zero_all += angle_el_zero[i];
            }
            motor->MotorConfig.angle_el_zero = angle_el_zero_all / (float)sample_total;
            // 误差数组减去零点偏移（保留纯非线性误差）
            for (int i = 0; i < sample_total; i++)
            {
                angle_el_zero[i] -= motor->MotorConfig.angle_el_zero;
            }
            break;

        // 状态5：采样点映射到NLLUT数组
        case 5:
            map_samples_to_lut(angle_el_zero, sample_total, motor->MotorConfig.NLLUT_encoder, LUT_total);
            break;

        // 状态6：收尾（停止PWM、释放内存）
        case 6:
            set_svpwm(motor, 0.0f, 0.0f, 0.0f);
            free(angle_el_zero);
            angle_el_zero = NULL;
            break;

        // 状态7：结束/错误处理
        case 7:
            // 错误时释放残留内存
            if (angle_el_zero != NULL)
            {
                free(angle_el_zero);
                angle_el_zero = NULL;
            }
            break;

        // 默认：异常重置
        default:
            set_svpwm(motor, 0.0f, 0.0f, 0.0f);
            if (angle_el_zero != NULL)
            {
                free(angle_el_zero);
                angle_el_zero = NULL;
            }
            state = 0;
            break;
    }
}

void update_loopcount_rotor_block(Motor_HandleTypeDef *motor,float angle_encoder_B)
{
    static float angle_A =  0 ;
    static float angle_B =  0 ;
    static float angle_B_ = 0 ;
    update_angle(motor);
    // mt6816_update_angle(&mt6816);
    angle_A = Limit_angle_el(motor->MotorAlg.angle-motor->MotorConfig.angle_zero_gear_A);
    angle_B = Limit_angle_el(2*PI - angle_encoder_B +motor->MotorConfig.angle_zero_gear_B);

    for(int i = -8 ; i<9 ;i++)
    {
      angle_B_ = Limit_angle_el( fmodf( ((angle_A+(float)i*2*PI)*(motor->MotorConfig.GT_A/motor->MotorConfig.GT_B)) , (2*PI) ) ) ;
      if(fabs(angle_B_-angle_B)<0.05)
      {
        motor->MotorConfig.loopcount_rotor = i ;
        break;
      }
    }
}

void update_NLLUT_and_angle_el_zero_sensor_block(Motor_HandleTypeDef *motor)
{
    static float data_debug_0 = 0.0f;
    static uint32_t data_debug_1 = 0 ;
    static uint32_t data_debug_2 = 0 ;

    uint32_t sample_per = 20 ; //每极对采样20个
    uint32_t sample_total = motor->MotorConfig.Pole_pairs * sample_per; //总采样数
    uint32_t sample_total_NLLUT = sizeof(motor->MotorConfig.NLLUT_encoder)/sizeof(float);
    float *angle_el_zero = (float*)calloc(sample_total,sizeof(float));  //按照采样数定义动态数组
    float angle_el_zero_all = 0;

    float angle_all_temp = motor->MotorData.angle_all;//保存现场
    float angle_last_temp = motor->MotorAlg.last_angle;
    float angle_temp = motor->MotorAlg.angle;

    reset_data_angle(motor);
    
    if(angle_el_zero == NULL)
    {   
        //打印报错信息
        SEGGER_RTT_printf(0, "Heap_Size is not enough!\n");
        free((void*)angle_el_zero);
        return;
    }

    set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.05f , 0.0f);
    motor->MotorDrv.Delayms(1000);

    float angle_now = 0.0f;
    do
    {
        update_dt(motor);
        update_angle(motor);
        angle_now = ctrl_motor_openloop_angle_nonblock(motor,2*PI,0.0f,0.6,0.0f,motor->MotorConfig.UMAX*0.05f);
        uint32_t i = (uint32_t)round((angle_now/(2*PI))*(float)sample_total) ;
        uint32_t j = (uint32_t)round((angle_now/(2*PI))*(float)sample_total_NLLUT);
        if((i>=sample_total|| i<0)&&(j>=sample_total_NLLUT || j<0))
        {
            ctrl_motor_openloop_angle_nonblock(motor,0.0f,0.0f,1000.0f,0.0f,0.0f);//注销掉这个函数的angle_now，防止用到下一次开环执行程序中
            break;
        }
        angle_el_zero[i] = angle_now - motor->MotorData.angle_all;
        motor->MotorConfig.NLLUT_encoder[j] = angle_el_zero[i];

        data_debug_0 = motor->MotorConfig.NLLUT_encoder[j];
        data_debug_1 = j ;
        data_debug_2 = i ;
    } while (angle_now);
    motor->MotorDrv.Delayms(500);
    do
    {
        update_dt(motor);
        update_angle(motor);
        angle_now = ctrl_motor_openloop_angle_nonblock(motor,0.0f,2*PI,-0.6,0.0f,motor->MotorConfig.UMAX*0.05f);
        uint32_t i = (uint32_t)round((angle_now/(2*PI))*(float)sample_total);
        uint32_t j = (uint32_t)round((angle_now/(2*PI))*(float)sample_total_NLLUT);
        if((i>=sample_total|| i<0)&&(j>=sample_total_NLLUT || j<0))
        {
            ctrl_motor_openloop_angle_nonblock(motor,0.0f,0.0f,1000.0f,0.0f,0.0f);//注销掉这个函数的angle_now，防止用到下一次开环执行程序中
            break;
        }
        angle_el_zero[i] += angle_now - motor->MotorData.angle_all;
        angle_el_zero[i] /= 2;

        // motor->MotorConfig.NLLUT_encoder[j] += angle_now - motor->MotorData.angle_all;
        // motor->MotorConfig.NLLUT_encoder[j] /= 2;

        data_debug_0 = motor->MotorConfig.NLLUT_encoder[j];
        data_debug_1 = j ;
        data_debug_2 = i ;
    } while (angle_now);

    for(int i = 0; i<sample_total ; i++)
    {
        angle_el_zero_all += angle_el_zero[i];
    }
    motor->MotorConfig.angle_el_zero = Calculate_angle_el(motor->MotorConfig.Pole_pairs,angle_el_zero_all/(float)sample_total, 0.0f);
    motor->MotorData.angle_all = angle_all_temp ;//返回现场
    motor->MotorAlg.last_angle = angle_last_temp;
    motor->MotorAlg.angle = angle_temp;

    for(int i = 0 ; i<sample_total_NLLUT ; i++)
    {
        motor->MotorConfig.NLLUT_encoder[i] -= angle_el_zero_all/(float)sample_total;
    }

    set_svpwm(motor,0.0f, 0.0f , 0.0f);
    free((void*)angle_el_zero);    
}


// void update_loopcount_rotor_block(Motor_HandleTypeDef *motor,float angle_encoder_B)
// {
//     static float angle_A =  0 ;
//     static float angle_B =  0 ;
//     static float angle_B_ = 0 ;
//     static float angle_B__ = 0 ;
//     static float Kb = 0; // 定义Kb（若需长期保存，可改为static float Kb; 或赋值给motor结构体成员）
//     static float valid_Kb; // 可选：专门存储满足if条件的有效Kb（避免被循环覆盖）
//     update_angle(motor);
//     // mt6816_update_angle(&mt6816);J
//     angle_A = Limit_angle_el(motor->MotorAlg.angle-motor->MotorConfig.angle_zero_gear_A);
//     angle_B = Limit_angle_el(2*PI - angle_encoder_B +motor->MotorConfig.angle_zero_gear_B);

//     for(int i = 0 ; i<19 ;i++)
//     {
//         float X = ((angle_A + (float)i * 2 * PI) * (motor->MotorConfig.GT_A / motor->MotorConfig.GT_B));
//         float fmod_result = fmodf(X, 2 * PI);
//         Kb = (X - fmod_result) / (2 * PI);
//         angle_B_ = Limit_angle_el(fmod_result);
//         if(fabs(angle_B_-angle_B)<0.05)
//         {
//             // if(fabs(valid_Kb) - fabs(i) < 17)
//             // {
//                 angle_B__ = angle_B_;
//                 motor->MotorConfig.loopcount_rotor = i ;
//                 valid_Kb = Kb; 
//                 break; 
//             // }
//         }
//     }
// }

void ctrl_motor_openloop_velocity_el_nonblock(Motor_HandleTypeDef *motor,float velocity_el_target,float Uq,float Ud)
{
    static float velocity_integral = 0;
    velocity_integral += velocity_el_target*motor->time.dt;
    set_svpwm_dir(motor,Uq,Ud,Limit_angle_el(velocity_integral));
}

void ctrl_motor_openloop_velocity_nonblock(Motor_HandleTypeDef *motor,float velocity_target,float Uq,float Ud)
{
    static float velocity_integral = 0;
    velocity_integral += velocity_target*motor->time.dt;
    set_svpwm_dir(motor,Uq,Ud,Limit_angle_el(velocity_integral*(float)motor->MotorConfig.Pole_pairs));
    // set_spwm(motor,Uq,Ud,Limit_angle_el(velocity_integral*(float)motor->MotorConfig.Pole_pairs));
}

float ctrl_motor_openloop_angle_el_nonblock(Motor_HandleTypeDef *motor,float angle_el_target,float angle_el_start,float velocity_el_target ,float Uq,float Ud)
{
    static float angle_el_now = 0;
    static float velocity_integral = 0 ;
    velocity_integral += angle_el_target*motor->time.dt;
    angle_el_now = angle_el_start + velocity_integral;
    if(myabs(velocity_integral) < myabs(angle_el_target - angle_el_start))
    {
        set_svpwm_dir(motor,Uq,Ud,Limit_angle_el(velocity_integral));
        return angle_el_now ;
    }
    else
    {
        set_svpwm_dir(motor,0.0f,0.0f,0.0f);
        velocity_integral = 0.0f;
        angle_el_now = 0.0f;
        return 0 ;
    }
}

void ctrl_motor_openloop_angle_el_block(Motor_HandleTypeDef *motor,float angle_el_target,float angle_el_start,float velocity_el_target ,float Uq,float Ud)
{
    while(ctrl_motor_openloop_angle_el_nonblock(motor,angle_el_target,angle_el_start,velocity_el_target,Uq,Ud))
    {
        update_dt(motor);
    }
}

float ctrl_motor_openloop_angle_nonblock(Motor_HandleTypeDef *motor,float angle_target,float angle_start,float velocity_target ,float Uq,float Ud)
{
    static float angle_now = 0;
    static float velocity_integral = 0 ;
    static float angle = 0 ;
    velocity_integral += velocity_target*motor->time.dt;
    angle_now = angle_start + velocity_integral;
    angle = Limit_angle_el(velocity_integral*(float)motor->MotorConfig.Pole_pairs);
    if( fabs(velocity_integral) < fabs(angle_target-angle_start))
    {
        set_svpwm_dir(motor,Uq,Ud,angle);
        return angle_now ;
    }
    else
    {
        set_svpwm_dir(motor,0.0f,0.0f,0.0f);
        angle_now = 0.0f;
        velocity_integral = 0.0f;
        return 0;
    }
}
void Calculate_IdIq(float IA, float IB, float IC, float angle_el, float *IdIq_out)
{
    // --- 第一步: Clark 变换 (3相 -> 2相静止) ---
    // Ialpha = IA
    float Ialpha = IA;
    
    // Ibeta = (sqrt(3) * (IB - IC)) / 3
    float Ibeta = (sqrt_3 * (IB - IC)) / 3.0f;

    // --- 第二步: Park 变换 (2相静止 -> 2相同步旋转) ---
    float cos_theta = cosf(angle_el);
    float sin_theta = sinf(angle_el);

    // Id = Ialpha * cosθ + Ibeta * sinθ
    IdIq_out[0] = Ialpha * cos_theta + Ibeta * sin_theta;
    
    // Iq = -Ialpha * sinθ + Ibeta * cosθ
    IdIq_out[1] = -Ialpha * sin_theta + Ibeta * cos_theta;
}

void ctrl_motor_openloop_angle_block(Motor_HandleTypeDef *motor,float angle_target,float angle_start,float velocity_target ,float Uq,float Ud)
{
    while(ctrl_motor_openloop_angle_nonblock(motor,angle_target,angle_start,velocity_target,Uq,Ud))
    {
        static float iqid[2] = {0.0f, 0.0f};
        static float iq = 0;
        static float id = 0;
        update_dt(motor);
        update_angle(motor);
        Calculate_IdIq(motor->MotorAlg.IA,motor->MotorAlg.IB,motor->MotorAlg.IC,motor->MotorAlg.angle_el,iqid);
        iq = iqid[0];
        id = iqid[1];
    }
}

