/*
 * SPDX-FileCopyrightText: 2026 FryingCat <3551901875@qq.com>
 * SPDX-License-Identifier: MIT
 */

#include "foc_alg.h"
#include "SEGGER_RTT.h"
#include "foc_drv.h"
float my_sgn(float x)
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

float my_sat(float e, float r) 
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

float my_abs(float val)
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

/** 四舍五入到最近整数（等价 roundf，不依赖 libm） */
float my_round(float x)
{
    if (x >= 0.0f)
    {
        return (float)(int32_t)(x + 0.5f);
    }
    return (float)(int32_t)(x - 0.5f);
}

int32_t my_fast_round(float x)
{
    return (int32_t)my_round(x);
}

/** 向下取整（等价 floorf，不依赖 libm） */
float my_floor(float x)
{
    const int32_t i = (int32_t)x;
    if (x < 0.0f && (float)i > x)
    {
        return (float)(i - 1);
    }
    return (float)i;
}

static float my_powi_u(float base, uint32_t exp)
{
    float result = 1.0f;
    while (exp != 0u)
    {
        if ((exp & 1u) != 0u)
        {
            result *= base;
        }
        base *= base;
        exp >>= 1u;
    }
    return result;
}

static float my_powi(float base, int32_t exp)
{
    if (exp == 0)
    {
        return 1.0f;
    }
    if (exp < 0)
    {
        return 1.0f / my_powi_u(base, (uint32_t)(-exp));
    }
    return my_powi_u(base, (uint32_t)exp);
}

static float my_ln(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }

    int e = 0;
    float m = x;
    while (m >= 2.0f)
    {
        m *= 0.5f;
        e++;
    }
    while (m < 1.0f)
    {
        m *= 2.0f;
        e--;
    }

    const float t = m - 1.0f;
    const float ln_m =
        t * (1.0f + t * (-0.5f + t * (0.33333333f + t * (-0.25f))));
    return ln_m + (float)e * 0.69314718056f;
}

static float my_exp(float x)
{
    if (x <= -80.0f)
    {
        return 0.0f;
    }
    if (x >= 80.0f)
    {
        return 1e38f;
    }

    const float k_ln2 = 0.69314718056f;
    const int k = (int)(x / k_ln2 + (x >= 0.0f ? 0.5f : -0.5f));
    const float r = x - (float)k * k_ln2;
    const float er =
        1.0f + r * (1.0f + r * (0.5f + r * (0.16666667f + r * 0.04166667f)));

    if (k >= 0)
    {
        return er * my_powi_u(2.0f, (uint32_t)k);
    }
    return er / my_powi_u(2.0f, (uint32_t)(-k));
}

/** 幂运算（等价 powf 常用场景，不依赖 libm） */
float my_pow(float base, float exp)
{
    if (exp == 0.0f)
    {
        return 1.0f;
    }
    if (base == 0.0f)
    {
        return (exp > 0.0f) ? 0.0f : 1.0f;
    }

    const int32_t ei = (int32_t)exp;
    if (exp == (float)ei)
    {
        return my_powi(base, ei);
    }

    if (base < 0.0f)
    {
        return 0.0f;
    }

    return my_exp(exp * my_ln(base));
}

float my_round_to_decimal(float x, int n)
{
    if (n < 0)
    {
        return x;
    }

    const float scale = my_pow(10.0f, (float)n);
    return (float)my_fast_round(x * scale) / scale;
}

float my_map( float Data ,float formLOW,float formHIGH, float toLOW,float toHIGH)
{
	return ((Data-formLOW)*((float)((toHIGH-toLOW)/(float)(formHIGH-formLOW))))+toLOW;
}

float my_sin(float x)
{
    float sign = 1.0f;

    /* 输入应在 [0, 2π)；调用方先用 Limit_angle_el 归一化 */
    if (x < PI_2)
    {
        const float x2 = x * x;
        return x * (1.0f + x2 * (SIN_C3 + x2 * SIN_C5));
    }
    if (x <= PI)
    {
        x = PI - x;
    }
    else if (x <= _3PI_2)
    {
        x -= PI;
        sign = -1.0f;
    }
    else
    {
        x = _2PI - x;
        sign = -1.0f;
    }

    const float x2 = x * x;
    return sign * x * (1.0f + x2 * (SIN_C3 + x2 * SIN_C5));
}

float my_fmodf(float x, float y)
{
    if (y == 0.0f)
    {
        return 0.0f;
    }

    int n = (int)(x / y);
    return x - (float)n * y;
}

float my_cos(float x)
{
    x += PI_2;
    if (x >= _2PI)
    {
        x -= _2PI;
    }
    return my_sin(x);
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
    if (my_abs(period) < EPS)
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

    offset = my_fmodf(offset, period);

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

float update_angle(Motor_HandleTypeDef *motor)//待更新:angle_all的更新是上一个周期的angle_all 不是这次的angle_all
{
    float error_angle = motor->MotorAlg.angle-motor->MotorAlg.last_angle;
    if(my_abs(error_angle) > (0.8f*2*PI))
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

float update_angle_el(Motor_HandleTypeDef *motor) 
{
    motor->MotorAlg.angle = motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());
    motor->MotorAlg.angle_el = Calculate_angle_el(motor->MotorConfig.Pole_pairs,motor->MotorAlg.angle, motor->MotorConfig.angle_el_zero);
    return motor->MotorAlg.angle_el;
}

float my_Limit(float value , float high , float low)
{
    return (value)<(low)?(low):((value)>(high)?(high):(value));//如果目标参数超出最大/最小值的范围，就把这个值锁死在最大/最小值
}

void update_sincos(Motor_HandleTypeDef *motor)
{
    float angle_el = motor->MotorAlg.angle_el;
    motor->MotorAlg.sin_theta = my_sin(angle_el);
    motor->MotorAlg.cos_theta = my_cos(angle_el);
}

void Calculate_Park_N_theta(float Uq , float Ud , float angle_el, float *Ualpha, float *Ubeta)
{
    float cos_theta = my_cos(angle_el);
    float sin_theta = my_sin(angle_el);

    *Ualpha = Ud * cos_theta - Uq * sin_theta;
    *Ubeta  = Uq * cos_theta + Ud * sin_theta;
}

void Calculate_Park_N_sincos(float Uq , float Ud , float sin_theta ,float cos_theta, float *Ualpha, float *Ubeta)
{
    *Ualpha = Ud * cos_theta - Uq * sin_theta;
    *Ubeta  = Uq * cos_theta + Ud * sin_theta;
}

void update_Park_N(Motor_HandleTypeDef *motor)
{
    Calculate_Park_N_sincos(motor->MotorAlg.Uq , motor->MotorAlg.Ud , motor->MotorAlg.sin_theta,motor->MotorAlg.cos_theta,&motor->MotorAlg.Ualpha, &motor->MotorAlg.Ubeta);
}

void Calculate_Clark_N(float Ualpha ,float Ubeta,float Upower, float *Ua, float *Ub, float *Uc)
{
    //Clark逆变换:
    *Ua = Ualpha + Upower/2;                 // ①Ua = Ualpha ;
    *Ub = (SQRT3*Ubeta-Ualpha)/2 + Upower/2; // ②Ub = (√3 * Ubeta - Ualpha)/2 ;
    *Uc = -(Ualpha + SQRT3*Ubeta)/2 + Upower/2;// ③Uc = ( -Ualpha - √3 * Ubeta )/2;
}

void update_Clark_N(Motor_HandleTypeDef *motor)
{
    Calculate_Clark_N(motor->MotorAlg.Ualpha , motor->MotorAlg.Ubeta , motor->MotorConfig.UMAX,
                      &motor->MotorAlg.UA, &motor->MotorAlg.UB, &motor->MotorAlg.UC);
}

void Calculate_Clark(float IA ,float IB ,float IC, float *Ialpha, float *Ibeta)
{
    // 幅值不变型Clark变换（对称三相电流）
    *Ialpha = (2*IA - IB - IC)*_1_3;  // Iα
    *Ibeta  = (IB - IC)*_1_SQRT3;       // Iβ
}

void update_Clark(Motor_HandleTypeDef *motor)
{
    Calculate_Clark(motor->MotorAlg.IA , motor->MotorAlg.IB , motor->MotorAlg.IC,
                    &motor->MotorAlg.Ialpha, &motor->MotorAlg.Ibeta);
}

void Calculate_Park_theta(float Ialpha ,float Ibeta ,float angle_el_rad, float *Id, float *Iq)
{
    float cos_theta = my_cos(angle_el_rad);
    float sin_theta = my_sin(angle_el_rad);

    // Park变换（输入电角度为弧度制）
    *Id = Ialpha * cos_theta + Ibeta * sin_theta;
    *Iq = -Ialpha * sin_theta + Ibeta * cos_theta;
}

void Calculate_Park_sincos(float Ialpha ,float Ibeta ,float sin_theta,float cos_theta, float *Id, float *Iq)
{
    *Id = Ialpha * cos_theta + Ibeta * sin_theta;
    *Iq = -Ialpha * sin_theta + Ibeta * cos_theta;
}

void update_Park(Motor_HandleTypeDef *motor)
{
    Calculate_Park_sincos(motor->MotorAlg.Ialpha , motor->MotorAlg.Ibeta , motor->MotorAlg.sin_theta,motor->MotorAlg.cos_theta,&motor->MotorAlg.Id, &motor->MotorAlg.Iq);
}

void update_pwm(Motor_HandleTypeDef *motor)
{
    float _Ua = my_Limit(motor->MotorAlg.UA/motor->MotorConfig.UMAX , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = my_Limit(motor->MotorAlg.UB/motor->MotorConfig.UMAX , 1 , 0 );
	float _Uc = my_Limit(motor->MotorAlg.UC/motor->MotorConfig.UMAX , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL || motor->MotorDrv.Set_PWM_B!=NULL || motor->MotorDrv.Set_PWM_C!=NULL)
    {
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case -1:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Uc);
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
    float _Ua = my_Limit(Ta , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = my_Limit(Tb , 1 , 0 );
	float _Uc = my_Limit(Tc , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL ||motor->MotorDrv.Set_PWM_B!=NULL || motor->MotorDrv.Set_PWM_C!=NULL)
    {
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case -1:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Uc);
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
    float _Ua = my_Limit(Ta , 1 , 0 );//计算并限制ABC相所需的占空比
    float _Ub = my_Limit(Tb , 1 , 0 );
    float _Uc = my_Limit(Tc , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL || motor->MotorDrv.Set_PWM_B!=NULL ||motor->MotorDrv.Set_PWM_C!=NULL)
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

int Calculate_Sector( float Ualpha , float Ubeta )
{
    int A = (Ubeta > 0.0f) ? 1 : 0;
    int B = ((SQRT3 * Ualpha - Ubeta) > 0.0f) ? 1 : 0;
    int C = ((-SQRT3 * Ualpha - Ubeta) > 0.0f) ? 1 : 0;
    int N = A + 2 * B + 4 * C;

    switch (N)
    {
        case 1: return 2;
        case 2: return 6;
        case 3: return 1;
        case 4: return 4;
        case 5: return 3;
        case 6: return 5;
        default: return 1; /* Ualpha = Ubeta = 0 */
    }
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
    if(my_abs(angle - last_angle) > (0.8f*2*PI))
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

float Calculate_PID(float target, float feedback, float dt ,PID_t* pid)
{
    pid->error = target - feedback;
    
    // 计算积分项
    pid->This_I += pid->error * dt ;
    pid->This_I = my_Limit(pid->This_I, pid->integral_max, pid->integral_min); // 限制积分项防止积分饱和

    // 计算微分项
    float derivative = (pid->error - pid->last_error) / dt;

    // 计算PID输出
    pid->Output = pid->KP * pid->error + pid->KI * pid->This_I + pid->KD * derivative;
    // pid->Output = pid->KP * pid->error;
    pid->Output = my_Limit(pid->Output, pid->output_max, pid->output_min); // 限制输出范围

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
    pid->This_I = my_Limit(pid->This_I, pid->integral_max, pid->integral_min);

    float derivative = (pid->error - pid->last_error) / dt;
    pid->Output = pid->KP * pid->error + pid->KI * pid->This_I + pid->KD * derivative;
    pid->Output = my_Limit(pid->Output, pid->output_max, pid->output_min);

    pid->last_error = pid->error;
    return pid->Output;
}

float Calculate_PID_IS_AIS(float target, float feedback, float dt, PID_t* pid,float n)   //自适应积分分离
{
    float r = pid->KP;
    float D = target;

    float sum = r * D * ((1.0f - my_pow((1.0f - r), n)) / r);

    pid->error = target - feedback;

    if (my_abs(pid->error) <= my_abs(target-sum))
    {
        pid->This_I += pid->error * dt;
    }
    else
    {
        pid->This_I = 0.0f; // 超出范围时，积分项清零
    }
    pid->This_I = my_Limit(pid->This_I, pid->integral_max, pid->integral_min);

    float derivative = (pid->error - pid->last_error) / dt;
    pid->Output = pid->KP * pid->error + pid->KI * pid->This_I + pid->KD * derivative;
    pid->Output = my_Limit(pid->Output, pid->output_max, pid->output_min);

    pid->last_error = pid->error;
    return pid->Output;
}

void update_svpwm(Motor_HandleTypeDef *motor)
{
    float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	float Ta = 0 , Tb = 0 ,Tc = 0 ;

    update_Park_N(motor);
    update_Sector(motor);

    K=SQRT3/motor->MotorConfig.UMAX;
	Ux = motor->MotorAlg.Ubeta;
	Uy = (SQRT3/2.0f)*motor->MotorAlg.Ualpha - 0.5f*motor->MotorAlg.Ubeta;
	Uz = (SQRT3/2.0f)*motor->MotorAlg.Ualpha + 0.5f*motor->MotorAlg.Ubeta;

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
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}
	Tz = 0.5f*(1-Tx-Ty) ;

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

    motor->MotorAlg.UA = Ta*motor->MotorConfig.UMAX - motor->MotorConfig.UMAX/2;
    motor->MotorAlg.UB = Tb*motor->MotorConfig.UMAX - motor->MotorConfig.UMAX/2;
    motor->MotorAlg.UC = Tc*motor->MotorConfig.UMAX - motor->MotorConfig.UMAX/2;

    set_pwm(motor,Ta,Tb,Tc);
}

void set_svpwm(Motor_HandleTypeDef *motor, float Uq , float Ud ,float angle_el)
{
    float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	float Ta = 0 , Tb = 0 ,Tc = 0 ;
    float Ualpha = 0 , Ubeta = 0 ;

    Calculate_Park_N_theta(Uq , Ud , angle_el, &Ualpha, &Ubeta);

    int Sector = Calculate_Sector(Ualpha , Ubeta);
    
    K=SQRT3/motor->MotorConfig.UMAX;
    Ux = Ubeta;
    Uy = (SQRT3/2.0f)*Ualpha - 0.5f*Ubeta;
    Uz = (SQRT3/2.0f)*Ualpha + 0.5f*Ubeta;

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
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}
	Tz = 0.5f*(1-Tx-Ty) ;

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

    set_pwm_nodir(motor,Ta,Tb,Tc);
}

void set_svpwm_dir(Motor_HandleTypeDef *motor, float Uq , float Ud ,float angle_el)
{
    float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	float Ta = 0 , Tb = 0 ,Tc = 0 ;
    float Ualpha = 0 , Ubeta = 0 ;

    Calculate_Park_N_theta(Uq , Ud , angle_el, &Ualpha, &Ubeta);

    motor->MotorAlg.Ualpha = Ualpha;//打印
    motor->MotorAlg.Ubeta = Ubeta;
    
    int Sector = Calculate_Sector(Ualpha , Ubeta);
    
    K=SQRT3/motor->MotorConfig.UMAX;
    Ux = Ubeta;
    Uy = (SQRT3/2.0f)*Ualpha - 0.5f*Ubeta;
    Uz = (SQRT3/2.0f)*Ualpha + 0.5f*Ubeta;

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
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}
	Tz = 0.5f*(1-Tx-Ty) ;

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

    set_pwm(motor,Ta,Tb,Tc);
}

void update_spwm(Motor_HandleTypeDef *motor)
{
    update_Park_N(motor);
    update_Clark_N(motor);

    float TA = my_map(motor->MotorAlg.UA,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TB = my_map(motor->MotorAlg.UB,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TC = my_map(motor->MotorAlg.UC,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    
    set_pwm(motor,TA, TB, TC);
}

void set_spwm(Motor_HandleTypeDef *motor,float Uq, float Ud ,float angle_el)
{
    float Ualpha, Ubeta;
    float UA, UB, UC;

    Calculate_Park_N_theta(Uq , Ud , angle_el, &Ualpha, &Ubeta);
    Calculate_Clark_N(Ualpha , Ubeta , motor->MotorConfig.UMAX, &UA, &UB, &UC);

    UA = UA - motor->MotorConfig.UMAX/2;
    UB = UB - motor->MotorConfig.UMAX/2;
    UC = UC - motor->MotorConfig.UMAX/2;

    motor->MotorAlg.UA = UA;
    motor->MotorAlg.UB = UB;
    motor->MotorAlg.UC = UC;

    float TA = my_map(UA,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TB = my_map(UB,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TC = my_map(UC,-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);

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

void Calculate_Order_int(float IA, float IB, float IC, int PHASE, int *IA_out, int *IB_out, int *IC_out)
{
    int a = (int)IA;
    int b = (int)IB;
    int c = (int)IC;

    switch (PHASE)
    {
        case 1:
            *IA_out = a;
            *IB_out = b;
            *IC_out = c;
            break;
        case 2:
            *IA_out = b;
            *IB_out = a;
            *IC_out = c;
            break;
        case 3:
            *IA_out = c;
            *IB_out = b;
            *IC_out = a;
            break;
        case 4:
            *IA_out = c;
            *IB_out = a;
            *IC_out = b;
            break;
        case 5:
            *IA_out = b;
            *IB_out = c;
            *IC_out = a;
            break;
        case 6:
            *IA_out = a;
            *IB_out = c;
            *IC_out = b;
            break;
        default:
            *IA_out = a;
            *IB_out = b;
            *IC_out = c;
            break;
    }
}

void Calculate_Order_float(float IA, float IB, float IC, int PHASE, float *IA_out, float *IB_out, float *IC_out)
{
    float a = IA;
    float b = IB;
    float c = IC;

    switch (PHASE)
    {
        case 1:
            *IA_out = a;
            *IB_out = b;
            *IC_out = c;
            break;
        case 2:
            *IA_out = b;
            *IB_out = a;
            *IC_out = c;
            break;
        case 3:
            *IA_out = c;
            *IB_out = b;
            *IC_out = a;
            break;
        case 4:
            *IA_out = c;
            *IB_out = a;
            *IC_out = b;
            break;
        case 5:
            *IA_out = b;
            *IB_out = c;
            *IC_out = a;
            break;
        case 6:
            *IA_out = a;
            *IB_out = c;
            *IC_out = b;
            break;
        default:
            *IA_out = a;
            *IB_out = b;
            *IC_out = c;
            break;
    }
}

int update_IaIbIc(Motor_HandleTypeDef *motor,int Mode_Sampling,int PHASE)
{
    switch (Mode_Sampling)
    {
        case 0x111: /* ABC*/
        {
            if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL)
            {
                return 0;
            }
            motor->MotorData.CurrentData.I_raw.IA_raw = motor->MotorDrv.Update_Ia_raw();
            motor->MotorData.CurrentData.I_raw.IB_raw = motor->MotorDrv.Update_Ib_raw();
            motor->MotorData.CurrentData.I_raw.IC_raw = motor->MotorDrv.Update_Ic_raw();

            motor->MotorData.IA_NoOrder= motor->MotorDrv.Cal_Ia(motor->MotorData.CurrentData.I_raw.IA_raw, motor->MotorData.IA_offset_raw);
            motor->MotorData.IB_NoOrder = motor->MotorDrv.Cal_Ib(motor->MotorData.CurrentData.I_raw.IB_raw, motor->MotorData.IB_offset_raw);
            motor->MotorData.IC_NoOrder = motor->MotorDrv.Cal_Ic(motor->MotorData.CurrentData.I_raw.IC_raw, motor->MotorData.IC_offset_raw);
        }break;
        case 0x110: /* ABX */
        {
            if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ib_raw == NULL)
            {
                return 0;
            }
            motor->MotorData.CurrentData.I_raw.IA_raw = motor->MotorDrv.Update_Ia_raw();
            motor->MotorData.CurrentData.I_raw.IB_raw = motor->MotorDrv.Update_Ib_raw();
            motor->MotorData.CurrentData.I_raw.IC_raw = 0U;

            motor->MotorData.IA_NoOrder = motor->MotorDrv.Cal_Ia(motor->MotorData.CurrentData.I_raw.IA_raw, motor->MotorData.IA_offset_raw);
            motor->MotorData.IB_NoOrder = motor->MotorDrv.Cal_Ib(motor->MotorData.CurrentData.I_raw.IB_raw, motor->MotorData.IB_offset_raw);
            motor->MotorData.IC_NoOrder = -(motor->MotorData.IA_NoOrder + motor->MotorData.IB_NoOrder);
        }break;
        case 0x101: /* AXC*/
        {
            if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ia == NULL || motor->MotorDrv.Cal_Ic == NULL)
            {
                return 0;
            }
            motor->MotorData.CurrentData.I_raw.IA_raw = motor->MotorDrv.Update_Ia_raw();
            motor->MotorData.CurrentData.I_raw.IB_raw = 0U;
            motor->MotorData.CurrentData.I_raw.IC_raw = motor->MotorDrv.Update_Ic_raw();

            motor->MotorData.IA_NoOrder = motor->MotorDrv.Cal_Ia(motor->MotorData.CurrentData.I_raw.IA_raw, motor->MotorData.IA_offset_raw);
            motor->MotorData.IC_NoOrder = motor->MotorDrv.Cal_Ic(motor->MotorData.CurrentData.I_raw.IC_raw, motor->MotorData.IC_offset_raw);
            motor->MotorData.IB_NoOrder = -(motor->MotorData.IA_NoOrder + motor->MotorData.IC_NoOrder);
        }break;
        case 0x011: /* XBC*/
        {
            if (motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ib == NULL || motor->MotorDrv.Cal_Ic == NULL)
            {
                return 0;
            }
            motor->MotorData.CurrentData.I_raw.IA_raw = 0U;
            motor->MotorData.CurrentData.I_raw.IB_raw = motor->MotorDrv.Update_Ib_raw();
            motor->MotorData.CurrentData.I_raw.IC_raw = motor->MotorDrv.Update_Ic_raw();

            motor->MotorData.IB_NoOrder = motor->MotorDrv.Cal_Ib(motor->MotorData.CurrentData.I_raw.IB_raw, motor->MotorData.IB_offset_raw);
            motor->MotorData.IC_NoOrder = motor->MotorDrv.Cal_Ic(motor->MotorData.CurrentData.I_raw.IC_raw, motor->MotorData.IC_offset_raw);
            motor->MotorData.IA_NoOrder = -(motor->MotorData.IB_NoOrder + motor->MotorData.IC_NoOrder);
        }break;
        case 0x100: /* AXX */
        {
            if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Cal_Ia == NULL)
            {
                return 0;
            }
            motor->MotorData.CurrentData.I_raw.IA_raw = motor->MotorDrv.Update_Ia_raw();
            motor->MotorData.CurrentData.I_raw.IB_raw = 0U;
            motor->MotorData.CurrentData.I_raw.IC_raw = 0U;

            motor->MotorData.IA_NoOrder = motor->MotorDrv.Cal_Ia(motor->MotorData.CurrentData.I_raw.IA_raw, motor->MotorData.IA_offset_raw);
            motor->MotorData.IB_NoOrder = 0.0f;
            motor->MotorData.IC_NoOrder = 0.0f;
        }break;
        case 0x010: /* XBX*/
        {
            if (motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Cal_Ib == NULL)
            {
                return 0;
            }
            motor->MotorData.CurrentData.I_raw.IA_raw = 0U;
            motor->MotorData.CurrentData.I_raw.IB_raw = motor->MotorDrv.Update_Ib_raw();
            motor->MotorData.CurrentData.I_raw.IC_raw = 0U;

            motor->MotorData.IA_NoOrder = 0.0f;
            motor->MotorData.IB_NoOrder = motor->MotorDrv.Cal_Ib(motor->MotorData.CurrentData.I_raw.IB_raw, motor->MotorData.IB_offset_raw);
            motor->MotorData.IC_NoOrder = 0.0f;
        }break;
        case 0x001: /* XXC */
        {
            if (motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ic == NULL)
            {
                return 0;
            }
            motor->MotorData.CurrentData.I_raw.IA_raw = 0U;
            motor->MotorData.CurrentData.I_raw.IB_raw = 0U;
            motor->MotorData.CurrentData.I_raw.IC_raw = motor->MotorDrv.Update_Ic_raw();

            motor->MotorData.IA_NoOrder = 0.0f;
            motor->MotorData.IB_NoOrder = 0.0f;
            motor->MotorData.IC_NoOrder = motor->MotorDrv.Cal_Ic(motor->MotorData.CurrentData.I_raw.IC_raw, motor->MotorData.IC_offset_raw);
        }break;
        default:
            return 0;
    }

    Calculate_Order_float(motor->MotorData.IA_NoOrder, motor->MotorData.IB_NoOrder, motor->MotorData.IC_NoOrder, PHASE,
                          &motor->MotorAlg.IA, &motor->MotorAlg.IB, &motor->MotorAlg.IC);
    return 1 ;
}

void update_IalphaIbeta(Motor_HandleTypeDef *motor)
{
    Calculate_Clark(motor->MotorAlg.IA , motor->MotorAlg.IB , motor->MotorAlg.IC,&motor->MotorAlg.Ialpha, &motor->MotorAlg.Ibeta);
}

void update_IqId(Motor_HandleTypeDef *motor)
{
    Calculate_Park_sincos(motor->MotorAlg.Ialpha , motor->MotorAlg.Ibeta , motor->MotorAlg.sin_theta,motor->MotorAlg.cos_theta,&motor->MotorAlg.Id, &motor->MotorAlg.Iq);
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

int update_Ioffset_nonblock_(Motor_HandleTypeDef *motor,uint32_t this_IA_raw,uint32_t this_IB_raw,uint32_t this_IC_raw)
{
    if(motor->MotorData.Calibrate_Ioffset_nonblock__count < motor->MotorData.Calibrate_Ioffset_nonblock__sample_total)
    {
        motor->MotorData.Calibrate_Ioffset_nonblock__IA_offset_raw_all += this_IA_raw;
        motor->MotorData.Calibrate_Ioffset_nonblock__IB_offset_raw_all += this_IB_raw;
        motor->MotorData.Calibrate_Ioffset_nonblock__IC_offset_raw_all += this_IC_raw;
        motor->MotorData.Calibrate_Ioffset_nonblock__count++;
        return 0 ;
    }
    else if(motor->MotorData.Calibrate_Ioffset_nonblock__count >= motor->MotorData.Calibrate_Ioffset_nonblock__sample_total)
    {
        motor->MotorData.IA_offset_raw = motor->MotorData.Calibrate_Ioffset_nonblock__IA_offset_raw_all/motor->MotorData.Calibrate_Ioffset_nonblock__count;
        motor->MotorData.IB_offset_raw = motor->MotorData.Calibrate_Ioffset_nonblock__IB_offset_raw_all/motor->MotorData.Calibrate_Ioffset_nonblock__count;
        motor->MotorData.IC_offset_raw = motor->MotorData.Calibrate_Ioffset_nonblock__IC_offset_raw_all/motor->MotorData.Calibrate_Ioffset_nonblock__count; //因为IC没有偏置，所以这里直接用IB的平均值
        motor->MotorData.Calibrate_Ioffset_nonblock__IA_offset_raw_all = 0;
        motor->MotorData.Calibrate_Ioffset_nonblock__IB_offset_raw_all = 0;
        motor->MotorData.Calibrate_Ioffset_nonblock__IC_offset_raw_all = 0;
        motor->MotorData.Calibrate_Ioffset_nonblock__count = 0;
        return 1 ;
    }
    return 0 ;
}

int update_Ioffset_nonblock(Motor_HandleTypeDef *motor,int Mode_Sampling)
{
    uint32_t this_IA_raw = 0U;
    uint32_t this_IB_raw = 0U;
    uint32_t this_IC_raw = 0U;

    if(motor->MotorData.Calibrate_Ioffset_nonblock__count < motor->MotorData.Calibrate_Ioffset_nonblock__sample_total)
    {
        switch(Mode_Sampling)
        {
            case 0x111: /* ABC*/
            {
                if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL)
                {
                    return 0;
                }
                this_IA_raw = motor->MotorDrv.Update_Ia_raw();
                this_IB_raw = motor->MotorDrv.Update_Ib_raw();
                this_IC_raw = motor->MotorDrv.Update_Ic_raw();
            }break;
            case 0x110: /* ABX */
            {
                if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ib_raw == NULL)
                {
                    return 0;
                }
                this_IA_raw = motor->MotorDrv.Update_Ia_raw();
                this_IB_raw = motor->MotorDrv.Update_Ib_raw();
                this_IC_raw = 0U;
            }break;
            case 0x101: /* AXC*/
            {
                if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ia == NULL || motor->MotorDrv.Cal_Ic == NULL)
                {
                    return 0;
                }
                this_IA_raw = motor->MotorDrv.Update_Ia_raw();
                this_IB_raw = 0U;
                this_IC_raw = motor->MotorDrv.Update_Ic_raw();

            }break;
            case 0x011: /* XBC*/
            {
                if (motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ib == NULL || motor->MotorDrv.Cal_Ic == NULL)
                {
                    return 0;
                }
                this_IA_raw = 0U;
                this_IB_raw = motor->MotorDrv.Update_Ib_raw();
                this_IC_raw = motor->MotorDrv.Update_Ic_raw();

            }break;
            case 0x100: /* AXX */
            {
                if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Cal_Ia == NULL)
                {
                    return 0;
                }
                this_IA_raw = motor->MotorDrv.Update_Ia_raw();
                this_IB_raw = 0U;
                this_IC_raw = 0U;
            }break;
            case 0x010: /* XBX*/
            {
                if (motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Cal_Ib == NULL)
                {
                    return 0;
                }
                this_IA_raw = 0U;
                this_IB_raw = motor->MotorDrv.Update_Ib_raw();
                this_IC_raw = 0U;

            }break;
            case 0x001: /* XXC */
            {
                if (motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ic == NULL)
                {
                    return 0;
                }
                this_IA_raw = 0U;
                this_IB_raw = 0U;
                this_IC_raw = motor->MotorDrv.Update_Ic_raw();
            }break;
            default:
                return 0;            
        }
    }
    return update_Ioffset_nonblock_(motor,this_IA_raw,this_IB_raw,this_IC_raw);
}

void update_Ioffset_block(Motor_HandleTypeDef *motor,int Mode_Sampling)
{
    while(!update_Ioffset_nonblock(motor,Mode_Sampling));
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

        set_svpwm(motor,motor->MotorConfig.UMAX*0.05f,0.0f,Limit_angle_el((float)i*motor->MotorData.Calibrate_pole_pairs_block__angle_step));
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(2000);
    motor->MotorConfig.Pole_pairs = (uint32_t)my_round(my_abs((float)(1000*motor->MotorData.Calibrate_pole_pairs_block__angle_step)/velocity_integral));

    set_svpwm(motor,0.0f, 0.0f , 0.0f); 
}

void update_pole_pairs_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    float this_dt = update_dt(motor);
    update_angle(motor);
    update_velocity_raw(motor);
    update_pole_pairs_sensor_nonblock_(motor,this_dt,motor->MotorData.Velocity_raw);
}

void update_pole_pairs_sensor_nonblock_(Motor_HandleTypeDef *motor,float this_dt,float this_velocity_raw)
{
    motor->MotorData.Calibrate_pole_pairs_nonblock__total_time += this_dt;
    if(motor->MotorData.Calibrate_pole_pairs_nonblock__total_time < motor->MotorData.Calibrate_pole_pairs_nonblock__time_init)
    {
        motor->MotorData.Calibrate_pole_pairs_nonblock__state = 0;
    }
    else if(motor->MotorData.Calibrate_pole_pairs_nonblock__total_time >= motor->MotorData.Calibrate_pole_pairs_nonblock__time_init && motor->MotorData.Calibrate_pole_pairs_nonblock__total_time < (motor->MotorData.Calibrate_pole_pairs_nonblock__time_init+motor->MotorData.Calibrate_pole_pairs_nonblock__time_prep))
    {
        motor->MotorData.Calibrate_pole_pairs_nonblock__state = 1;
    }
    else if(motor->MotorData.Calibrate_pole_pairs_nonblock__total_time >= (motor->MotorData.Calibrate_pole_pairs_nonblock__time_init+motor->MotorData.Calibrate_pole_pairs_nonblock__time_prep) && motor->MotorData.Calibrate_pole_pairs_nonblock__total_time < (motor->MotorData.Calibrate_pole_pairs_nonblock__time_init+motor->MotorData.Calibrate_pole_pairs_nonblock__time_prep+motor->MotorData.Calibrate_pole_pairs_nonblock__time_process))
    {
        motor->MotorData.Calibrate_pole_pairs_nonblock__state = 2;
    }
    else if(motor->MotorData.Calibrate_pole_pairs_nonblock__total_time >= (motor->MotorData.Calibrate_pole_pairs_nonblock__time_init+motor->MotorData.Calibrate_pole_pairs_nonblock__time_prep+motor->MotorData.Calibrate_pole_pairs_nonblock__time_process))
    {
        motor->MotorData.Calibrate_pole_pairs_nonblock__state = 3;
    }
    
    switch (motor->MotorData.Calibrate_pole_pairs_nonblock__state)
    {
        case 0:
        {
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
        }break;
        case 1:
        {
            set_svpwm(motor, motor->MotorConfig.UMAX*0.05f,0.0f,0.0f);
        }break;
        case 2:
        {
            motor->MotorData.Calibrate_pole_pairs_nonblock__velocity_integral += this_velocity_raw*this_dt;
            set_svpwm(motor,motor->MotorConfig.UMAX*0.05f,0.0f, Limit_angle_el((float)motor->MotorData.Calibrate_pole_pairs_nonblock__velocity_target*(motor->MotorData.Calibrate_pole_pairs_nonblock__total_time-motor->MotorData.Calibrate_pole_pairs_nonblock__time_init-motor->MotorData.Calibrate_pole_pairs_nonblock__time_prep)));
            // motor->MotorDrv.Delayms(1);
        }break;
        case 3:
        {
            motor->MotorConfig.Pole_pairs = (uint32_t)my_round(my_abs((float)motor->MotorData.Calibrate_pole_pairs_nonblock__velocity_target*(motor->MotorData.Calibrate_pole_pairs_nonblock__total_time-motor->MotorData.Calibrate_pole_pairs_nonblock__time_init-motor->MotorData.Calibrate_pole_pairs_nonblock__time_prep)/(motor->MotorData.Calibrate_pole_pairs_nonblock__velocity_integral)));
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            motor->MotorData.Calibrate_pole_pairs_nonblock__total_time = 0.0f;
            motor->MotorData.Calibrate_pole_pairs_nonblock__velocity_integral = 0.0f;
            motor->MotorData.Calibrate_pole_pairs_nonblock__state = 0;
        }
        default:
        {
            /* 打印报错信息 */
        }break;
    }
    
}

void update_2DIR_sensor_block(Motor_HandleTypeDef *motor)
{
    float velocity_integral = 0.0f;
    for(int i=0 ; i<1000 ; i++)
    {
        update_dt(motor);  //预热dt，防止因初次启动产生的极小dt干扰后面的速度计算
        update_angle(motor);
        update_velocity_raw(motor);
        set_svpwm(motor,0.001f*i*motor->MotorConfig.UMAX*0.05f, motor->MotorConfig.UMAX*0.05f - 0.001f*i*motor->MotorConfig.UMAX*0.05f , 0.0f);
        motor->MotorDrv.Delayms(1);
    }

    for(int i=0 ; i<2000 ; i++)
    {
        update_dt(motor);
        update_angle(motor);
        update_velocity_raw(motor);
        // if(myabs(motor->MotorData.Velocity_raw) > 2*motor->MotorData.Calibrate_2DIR_block__velocity_target)
        // {
        //     motor->MotorData.Velocity_raw = 0.0f ;
        // }
        set_svpwm(motor,motor->MotorConfig.UMAX*0.05f,0.0f,Limit_angle_el((float)i*motor->MotorData.Calibrate_2DIR_block__velocity_target));
        velocity_integral += motor->MotorData.Velocity_raw;
        // printf("%f,%f\n",motor->MotorData.Velocity_raw,velocity_integral);
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(500);

    if(velocity_integral>0)
    {
        motor->MotorConfig.DIR = 1;
    }
    else if(velocity_integral<0)
    {
        motor->MotorConfig.DIR = -1;
    }
    else
    {
        /*传感器异常报错*/
    }
    set_svpwm(motor,0.0f,0.0f,0);
}

void update_2DIR_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    float this_dt = update_dt(motor); //预热
    update_angle(motor);
    update_velocity_raw(motor);
    update_2DIR_sensor_nonblock_(motor,this_dt,motor->MotorData.Velocity_raw);
}

void update_2DIR_sensor_nonblock_(Motor_HandleTypeDef *motor,float this_dt,float this_velocity_raw)
{
    motor->MotorData.Calibrate_2DIR_nonblock__total_time += this_dt; //预热

    if(motor->MotorData.Calibrate_2DIR_nonblock__total_time < motor->MotorData.Calibrate_2DIR_nonblock__time_init)
    {
        motor->MotorData.Calibrate_2DIR_nonblock__state = 0;
    }
    else if(motor->MotorData.Calibrate_2DIR_nonblock__total_time >= motor->MotorData.Calibrate_2DIR_nonblock__time_init && motor->MotorData.Calibrate_2DIR_nonblock__total_time < (motor->MotorData.Calibrate_2DIR_nonblock__time_init+motor->MotorData.Calibrate_2DIR_nonblock__time_prep))
    {
        motor->MotorData.Calibrate_2DIR_nonblock__state = 1;
    }
    else if(motor->MotorData.Calibrate_2DIR_nonblock__total_time >= (motor->MotorData.Calibrate_2DIR_nonblock__time_init+motor->MotorData.Calibrate_2DIR_nonblock__time_prep) && motor->MotorData.Calibrate_2DIR_nonblock__total_time < (motor->MotorData.Calibrate_2DIR_nonblock__time_init+motor->MotorData.Calibrate_2DIR_nonblock__time_prep+motor->MotorData.Calibrate_2DIR_nonblock__time_process))
    {
        motor->MotorData.Calibrate_2DIR_nonblock__state = 2;
    }
    else if(motor->MotorData.Calibrate_2DIR_nonblock__total_time >= (motor->MotorData.Calibrate_2DIR_nonblock__time_init+motor->MotorData.Calibrate_2DIR_nonblock__time_prep+motor->MotorData.Calibrate_2DIR_nonblock__time_process))
    {
        motor->MotorData.Calibrate_2DIR_nonblock__state = 3;
    }

    switch (motor->MotorData.Calibrate_2DIR_nonblock__state)
    {
        case 0:
        {
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
        }break;
        case 1:
        {
            float K =  (motor->MotorData.Calibrate_2DIR_nonblock__total_time-motor->MotorData.Calibrate_2DIR_nonblock__time_init)/motor->MotorData.Calibrate_2DIR_nonblock__time_process;
            set_svpwm(motor,0.0f,K*motor->MotorConfig.UMAX*0.05f, 0.0f);
        }break;
        case 2:
        {
            set_svpwm(motor,motor->MotorConfig.UMAX*0.05f,3.0f, Limit_angle_el((float)(motor->MotorData.Calibrate_2DIR_nonblock__total_time-motor->MotorData.Calibrate_2DIR_nonblock__time_init-motor->MotorData.Calibrate_2DIR_nonblock__time_prep)*motor->MotorData.Calibrate_2DIR_nonblock__velocity_target));
            motor->MotorData.Calibrate_2DIR_nonblock__velocity_integral += this_velocity_raw;
        }break;
        case 3:
        {
            if(motor->MotorData.Calibrate_2DIR_nonblock__velocity_integral>0)
            {
                motor->MotorConfig.DIR = 1;
            }
            else if(motor->MotorData.Calibrate_2DIR_nonblock__velocity_integral<0)
            {
                motor->MotorConfig.DIR = -1;
            }
            else
            {
                /*传感器异常报错*/
            }
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            motor->MotorData.Calibrate_2DIR_nonblock__total_time = 0.0f;
            motor->MotorData.Calibrate_2DIR_nonblock__velocity_integral = 0.0f;
        }
        default:
        {
            /* 打印报错信息 */
        }break;
    }
    // printf("%f,%d,%f,%f,%f\n",motor->MotorData.Calibrate_2DIR_nonblock__velocity_integral,motor->MotorConfig.DIR,motor->MotorData.Velocity_raw,motor->MotorAlg.angle,motor->time.dt);
}

void Calculate_PHASE(float IA, float IB, float IC,float UA, float UB, float UC, int *phase_a, int *phase_b, int *phase_c)//同时兼容直接向三相注入IqId时的工况，也就是在电机运行的情况下进行相序辨识
{
    int PHASE[3] = {0,0,0};
    float U[3] = {UA, UB, UC};
    float I[3] = {IA, IB, IC};

    for(int i = 0 ; i<3 ; i++)
    {
        for(int j = 0 ; j<3 ; j++)
        {
            if( U[i]/I[j] > 0 )
            {
                PHASE[i] = j+1; //相序标识最小也为1
            }
        }
    }

    *phase_a = PHASE[0];
    *phase_b = PHASE[1];
    *phase_c = PHASE[2];
}
int update_PHASE_nonblock(Motor_HandleTypeDef *motor,float IA_NoOrder, float IB_NoOrder, float IC_NoOrder)
{
    float this_dt = update_dt(motor);
    return update_PHASE_nonblock_(motor,IA_NoOrder,IB_NoOrder,IC_NoOrder,this_dt);
}

int update_PHASE_nonblock_(Motor_HandleTypeDef *motor,float IA_NoOrder, float IB_NoOrder, float IC_NoOrder,float this_dt)
{
    motor->MotorData.Calibrate_PHASE_nonblock__time += this_dt;
    if(motor->MotorData.Calibrate_PHASE_nonblock__time>(float)motor->MotorData.Calibrate_PHASE_nonblock__state * motor->MotorData.Calibrate_PHASE_nonblock__Ts)
    {
        motor->MotorData.Calibrate_PHASE_nonblock__state ++ ;
    }

    switch(motor->MotorData.Calibrate_PHASE_nonblock__state)
    {
        case 1 :
        {
            motor->MotorConfig.PHASE = -1; //先将相序设置为-1，表示正在检测相序
            set_pwm(motor,0.0f,0.0f,0.0f);
        }break;
        case 2 :
        {
            motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral = 0.0f;
            set_pwm(motor,motor->MotorData.Calibrate_PHASE_nonblock__Duty,0.0f,0.0f);
        }break;
        case 3 :
        {
            motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral += IA_NoOrder * this_dt;
            motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral += IB_NoOrder * this_dt;
            motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral += IC_NoOrder * this_dt;
            int phase_a, phase_b, phase_c;
            Calculate_PHASE(motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral , motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral , motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral , motor->MotorConfig.UMAX/2*motor->MotorData.Calibrate_PHASE_nonblock__Duty , 0.0f , 0.0f, &phase_a, &phase_b, &phase_c);
            motor->MotorData.Calibrate_PHASE_nonblock__phase_a = phase_a;
        }break;
        case 4:
        {
            motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral = 0.0f;
            set_pwm(motor,0.0f,motor->MotorData.Calibrate_PHASE_nonblock__Duty,0.0f);
        }break;
        case 5:
        {
            motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral += IA_NoOrder * this_dt;
            motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral += IB_NoOrder * this_dt;
            motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral += IC_NoOrder * this_dt;
            int phase_a, phase_b, phase_c;
            Calculate_PHASE(motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral , motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral , motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral , 0.0f , motor->MotorConfig.UMAX/2*motor->MotorData.Calibrate_PHASE_nonblock__Duty , 0.0f, &phase_a, &phase_b, &phase_c);
            motor->MotorData.Calibrate_PHASE_nonblock__phase_b = phase_b;            
        }break;
        case 6:
        {
            motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral = 0.0f;
            set_pwm(motor,0.0f,0.0f,motor->MotorData.Calibrate_PHASE_nonblock__Duty);
        }break;
        case 7:
        {
            motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral += IA_NoOrder * this_dt;
            motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral += IB_NoOrder * this_dt;
            motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral += IC_NoOrder * this_dt;
            int phase_a, phase_b, phase_c;
            Calculate_PHASE(motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral , motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral , motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral , 0.0f , 0.0f , motor->MotorConfig.UMAX/2*motor->MotorData.Calibrate_PHASE_nonblock__Duty, &phase_a, &phase_b, &phase_c);
            motor->MotorData.Calibrate_PHASE_nonblock__phase_c = phase_c;   
        }break;
        case 8:
        {
            if( motor->MotorData.Calibrate_PHASE_nonblock__phase_a == 1 && motor->MotorData.Calibrate_PHASE_nonblock__phase_b == 2 && motor->MotorData.Calibrate_PHASE_nonblock__phase_c == 3 )
            {
                motor->MotorConfig.PHASE = 1;
            }
            else if ( motor->MotorData.Calibrate_PHASE_nonblock__phase_a == 2 && motor->MotorData.Calibrate_PHASE_nonblock__phase_b == 1 && motor->MotorData.Calibrate_PHASE_nonblock__phase_c == 3 )
            {
                motor->MotorConfig.PHASE = 2;
            }
            else if ( motor->MotorData.Calibrate_PHASE_nonblock__phase_a == 3 && motor->MotorData.Calibrate_PHASE_nonblock__phase_b == 2 && motor->MotorData.Calibrate_PHASE_nonblock__phase_c == 1 )
            {
                motor->MotorConfig.PHASE = 3;
            }
            else if ( motor->MotorData.Calibrate_PHASE_nonblock__phase_a == 3 && motor->MotorData.Calibrate_PHASE_nonblock__phase_b == 1 && motor->MotorData.Calibrate_PHASE_nonblock__phase_c == 2 )
            {
                motor->MotorConfig.PHASE = 4;
            }
            else if ( motor->MotorData.Calibrate_PHASE_nonblock__phase_a == 2 && motor->MotorData.Calibrate_PHASE_nonblock__phase_b == 3 && motor->MotorData.Calibrate_PHASE_nonblock__phase_c == 1 )
            {
                motor->MotorConfig.PHASE = 5;
            }
            else if ( motor->MotorData.Calibrate_PHASE_nonblock__phase_a == 1 && motor->MotorData.Calibrate_PHASE_nonblock__phase_b == 3 && motor->MotorData.Calibrate_PHASE_nonblock__phase_c == 2 )
            {
                motor->MotorConfig.PHASE = 6;
            }
            else
            {
                motor->MotorConfig.PHASE = -1;
            }
        }break;
        case 9:
        {
            set_pwm(motor,0.0f,0.0f,0.0f);
            motor->MotorData.Calibrate_PHASE_nonblock__IA_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__IB_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__IC_Integral = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__time = 0.0f;
            motor->MotorData.Calibrate_PHASE_nonblock__state = 1 ;
            return motor->MotorConfig.PHASE ;
        }break;
    }
    return 0 ;
}

int update_PHASE_block(Motor_HandleTypeDef *motor)
{
    update_IaIbIc(motor,0x111,1);
    while(!update_PHASE_nonblock(motor,motor->MotorData.IA_NoOrder, motor->MotorData.IB_NoOrder, motor->MotorData.IC_NoOrder));
    if(motor->MotorConfig.PHASE == -1)
    {
        //打印报错信息
    }
    return motor->MotorConfig.PHASE ;
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
    uint16_t sample_total = motor->MotorConfig.Pole_pairs * motor->MotorData.Calibrate_angle_el_zero_sensor_block__sample_per;
    float *angle_el_zero = (float*)calloc(sample_total,sizeof(float));
    float angle_el_zero_all = 0.0f;
    float angle_all_temp = motor->MotorData.angle_all;
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
    // motor->MotorData.angle_all = update_angle(motor);
    // float angle_last = motor->MotorData.angle_all;
    float angle_now = 0.0f;
    int running;
    do
    {
        update_dt(motor);
        update_angle(motor);
        running = ctrl_motor_openloop_angle_nonblock(motor,motor->time.dt,2*PI,0.0f,0.6,0.0f,motor->MotorConfig.UMAX*0.05f);
        angle_now = 0.0f + motor->MotorData.Openloop__progress/(float)motor->MotorConfig.Pole_pairs;
        // angle_error = angle_now - motor->MotorData.angle_all;
        uint32_t i =(uint32_t)my_round((angle_now/(2*PI))*(float)sample_total);
        int32_t i_int = (int32_t)my_round((angle_now/(2*PI))*(float)sample_total);
        if(i>=sample_total|| i_int<0)
        {
            ctrl_motor_openloop_reset(motor);
            break;
        }
        angle_el_zero[i] = angle_now - motor->MotorData.angle_all;

        // printf("%d,%f,%f\n",i,angle_el_zero[i],angle_now);

    } while (running);
    ctrl_motor_openloop_reset(motor);
    motor->MotorDrv.Delayms(500);
    do
    {
        update_dt(motor);
        update_angle(motor);
        running = ctrl_motor_openloop_angle_nonblock(motor,motor->time.dt,0.0f,2*PI,-0.6,0.0f,motor->MotorConfig.UMAX*0.05f);
        angle_now = 2*PI + motor->MotorData.Openloop__progress/(float)motor->MotorConfig.Pole_pairs;
        // angle_error = angle_now - motor->MotorData.angle_all;
        uint32_t i =(uint32_t)my_round((angle_now/(2*PI))*(float)sample_total);
        int32_t i_int = (int32_t)my_round((angle_now/(2*PI))*(float)sample_total);
        if(i>=sample_total || i_int<0)
        {
            ctrl_motor_openloop_reset(motor);
            break;
        }
        angle_el_zero[i] += angle_now - motor->MotorData.angle_all;
        angle_el_zero[i] /= 2;

        // printf("%d,%f,%f\n",i,angle_el_zero[i],angle_now);
    } while (running);
    ctrl_motor_openloop_reset(motor);
    // printf("%f\n",angle_now);
    for(int i = 0; i<(int)sample_total ;i++)
    {
        angle_el_zero_all += angle_el_zero[i];
    }
    // motor->MotorConfig.angle_el_zero = angle_el_zero_all/(float)sample_total;
    motor->MotorConfig.angle_el_zero = Calculate_angle_el(motor->MotorConfig.Pole_pairs,angle_el_zero_all/(float)sample_total, 0.0f);
    
    motor->MotorData.angle_all = angle_all_temp ;
    motor->MotorAlg.last_angle = angle_last_temp;
    motor->MotorAlg.angle = angle_temp;

    set_svpwm(motor,0.0f, 0.0f , 0.0f);
    free((void*)angle_el_zero);
}

void update_angle_el_zero_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    float this_dt = update_dt(motor); //预热
    update_angle(motor);
    update_angle_el_zero_sensor_nonblock_(motor,this_dt,motor->MotorData.angle_all);
}

void update_angle_el_zero_sensor_nonblock_(Motor_HandleTypeDef *motor,float this_dt,float this_angle_all)
{
    uint16_t sample_total = motor->MotorConfig.Pole_pairs * motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__sample_per;
    float angle_all_temp = this_angle_all;
    motor->MotorData.angle_all = 0;

    switch(motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__state)
    {
        case 1:
        {
            motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero = (float*)calloc(sample_total,sizeof(float));
            if(motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero == NULL)
            {   
                //打印报错信息
                // printf("Heap_Size is not enough!");
                SEGGER_RTT_printf(0, "Heap_Size is not enough!\n");
                motor->MotorData.angle_all = angle_all_temp;
                free((void*)motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero);
                return;
            }
            else
            {
                motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__state = 2;
                set_svpwm(motor,motor->MotorConfig.UMAX*0.05f, 0.0f , 0.0f);
            }
            
        }break;
        case 2:
        {
            int running = ctrl_motor_openloop_angle_nonblock(motor,this_dt,2*PI,0.0f,0.3,motor->MotorConfig.UMAX*0.05f, 0.0f);
            float angle_now = 0.0f + motor->MotorData.Openloop__progress/(float)motor->MotorConfig.Pole_pairs;
            uint32_t i =(uint32_t)my_round((angle_now/(2*PI))*(float)sample_total);
            if(i>=sample_total|| !running)
            {
                ctrl_motor_openloop_reset(motor);
                motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__state = 3;
                break;
            }
            motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero[i] = angle_now - motor->MotorData.angle_all;
        }break;
        case 3:
        {
            int running = ctrl_motor_openloop_angle_nonblock(motor,this_dt,0.0f,2*PI,-0.3,motor->MotorConfig.UMAX*0.05f, 0.0f);
            float angle_now = 2*PI + motor->MotorData.Openloop__progress/(float)motor->MotorConfig.Pole_pairs;
            uint32_t i =(uint32_t)my_round((angle_now/(2*PI))*(float)sample_total);
            int32_t i_int =(int32_t)my_round((angle_now/(2*PI))*(float)sample_total);
            // printf("%f\n",angle_el_zero[i]);
            if(i>=sample_total)
            {
                --i;
            }
            if(i_int<0 || !running)
            {
                ctrl_motor_openloop_reset(motor);
                motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__state = 4;
                break;
            }
            motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero[i] += angle_now - motor->MotorData.angle_all;
            motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero[i] /= 2;
        }break;
        case 4:
        {
            float angle_el_zero_all = 0.0f;
            for(int i = 0; i<(int)sample_total ;i++)
            {
                angle_el_zero_all += motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero[i];
            }
            motor->MotorConfig.angle_el_zero = angle_el_zero_all/(float)sample_total;
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            free((void*)motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero);
            motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__angle_el_zero = NULL;
            motor->MotorData.Calibrate_angle_el_zero_sensor_nonblock__state = 1;
    
        }break;
        default:
        {
            //打印报错
        }break;
    }
    motor->MotorData.angle_all = angle_all_temp;
}

void ctrl_motor_openloop_reset(Motor_HandleTypeDef *motor)
{
    motor->MotorData.Openloop__angle_el = 0.0f;
    motor->MotorData.Openloop__progress = 0.0f;
    motor->MotorData.Openloop__state = 0;
}

void ctrl_motor_openloop_velocity_el_nonblock(Motor_HandleTypeDef *motor,float this_dt,float velocity_el_target,float Uq,float Ud)
{
    motor->MotorData.Openloop__angle_el = Limit_angle_el(motor->MotorData.Openloop__angle_el + velocity_el_target*this_dt);
    set_svpwm_dir(motor,Uq,Ud,motor->MotorData.Openloop__angle_el);
}

void ctrl_motor_openloop_velocity_nonblock(Motor_HandleTypeDef *motor,float this_dt,float velocity_target,float Uq,float Ud)
{
    ctrl_motor_openloop_velocity_el_nonblock(motor,this_dt,velocity_target*(float)motor->MotorConfig.Pole_pairs,Uq,Ud);
}

int ctrl_motor_openloop_angle_el_nonblock(Motor_HandleTypeDef *motor,float this_dt,float angle_el_target,float angle_el_start,float velocity_el_target ,float Uq,float Ud)
{
    if(motor->MotorData.Openloop__state == 0)
    {
        motor->MotorData.Openloop__state = 1;
        motor->MotorData.Openloop__progress = 0.0f;
        motor->MotorData.Openloop__angle_el = Limit_angle_el(angle_el_start);
    }
    if(motor->MotorData.Openloop__state == 2)
    {
        set_svpwm_dir(motor,Uq,Ud,motor->MotorData.Openloop__angle_el);
        return 0;
    }

    motor->MotorData.Openloop__progress += velocity_el_target*this_dt;
    if(my_abs(motor->MotorData.Openloop__progress) < my_abs(angle_el_target - angle_el_start))
    {
        motor->MotorData.Openloop__angle_el = Limit_angle_el(angle_el_start + motor->MotorData.Openloop__progress);
        set_svpwm_dir(motor,Uq,Ud,motor->MotorData.Openloop__angle_el);
        return 1;
    }

    motor->MotorData.Openloop__progress = angle_el_target - angle_el_start;
    motor->MotorData.Openloop__angle_el = Limit_angle_el(angle_el_target);
    motor->MotorData.Openloop__state = 2;
    set_svpwm_dir(motor,Uq,Ud,motor->MotorData.Openloop__angle_el);
    return 0;
}

int ctrl_motor_openloop_angle_el_block(Motor_HandleTypeDef *motor,float angle_el_target,float angle_el_start,float velocity_el_target ,float Uq,float Ud)
{
    int running;
    while((running = ctrl_motor_openloop_angle_el_nonblock(motor,motor->time.dt,angle_el_target,angle_el_start,velocity_el_target,Uq,Ud)))
    {
        update_dt(motor);
    }
    return running;
}

int ctrl_motor_openloop_angle_nonblock(Motor_HandleTypeDef *motor,float this_dt,float angle_target,float angle_start,float velocity_target ,float Uq,float Ud)
{
    float pole_pairs = (float)motor->MotorConfig.Pole_pairs;
    return ctrl_motor_openloop_angle_el_nonblock(motor,this_dt,angle_target*pole_pairs,angle_start*pole_pairs,velocity_target*pole_pairs,Uq,Ud);
}

int ctrl_motor_openloop_angle_block(Motor_HandleTypeDef *motor,float angle_target,float angle_start,float velocity_target ,float Uq,float Ud)
{
    int running;
    while((running = ctrl_motor_openloop_angle_nonblock(motor,motor->time.dt,angle_target,angle_start,velocity_target,Uq,Ud)))
    {
        update_dt(motor);
        update_angle(motor);
    }
    return running;
}

