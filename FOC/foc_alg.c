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

float update_angle_NLLUT(Motor_HandleTypeDef *motor)
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
    uint32_t sector_NLLUT = (uint32_t)my_fast_round((angle/(2*PI))*(float)size_NLLUT);
    float angle_1 = ((float)sector_NLLUT * (2*PI))/(float)size_NLLUT;
    float angle_2 = ((float)(sector_NLLUT+1) * (2*PI))/(float)size_NLLUT;
    float error_1 = NLLUT_encoder[sector_NLLUT];
    float error_2 = NLLUT_encoder[sector_NLLUT+1];
    if(sector_NLLUT >= (size_NLLUT-1))
    {
        return angle;
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

float my_Limit(float value , float high , float low)
{
    return (value)<(low)?(low):((value)>(high)?(high):(value));//如果目标参数超出最大/最小值的范围，就把这个值锁死在最大/最小值
}

float *Calculate_Park_N(float Uq , float Ud , float angle_el)
{
    static float Upark_N[2];  // 保留原静态数组以兼容接口
    float cos_theta, sin_theta;

    cos_theta = my_cos(angle_el);
    sin_theta = my_sin(angle_el);

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
    Uclark_N[1] = (SQRT3*Ubeta-Ualpha)/2 + Upower/2; // ②Ub = (√3 * Ubeta - Ualpha)/2 ;
    Uclark_N[2] = -(Ualpha + SQRT3*Ubeta)/2 + Upower/2;// ③Uc = ( -Ualpha - √3 * Ubeta )/2;
    
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

float *Calculate_Clark(float IA ,float IB ,float IC)
{	
    static float Iclark[2];
    // const float sqrt3 = sqrt(3.0); // 高精度√3
    
    // 幅值不变型Clark变换（对称三相电流）
    Iclark[0] = (2*IA - IB - IC)*_1_3;  // Iα
    Iclark[1] = (IB - IC)*_1_SQRT3;       // Iβ

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
    float cos_theta = my_cos(angle_el_rad);
    float sin_theta = my_sin(angle_el_rad);

    // Park变换（输入电角度为弧度制）
    Ipark[0] = Ialpha * cos_theta + Ibeta * sin_theta;  // Id
    Ipark[1] = -Ialpha * sin_theta + Ibeta * cos_theta; // Iq
    
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

int Calculate_Sector( float Ualpha , float Ubeta )//存在较多边界条件问题
{
    if (Ubeta == 0.0f) 
    {
        return (Ualpha >= 0.0f) ? 1 : 4;  // α轴：正→1，负→4
    }
    if (Ualpha == 0.0f) {
        return (Ubeta >= 0.0f) ? 2 : 5;  // β轴：正→2，负→5
    }
	if((Ualpha>0.0f) && (Ubeta>0.0f) && (Ubeta/Ualpha < SQRT3)){return 1 ;}
	else if((Ubeta>0.0f) && (Ubeta/my_abs(Ualpha)>SQRT3)){return 2 ;}
	else if((Ualpha<0.0f) && (Ubeta>0.0f) && (-Ubeta/Ualpha < SQRT3)){return 3 ;}
	else if((Ualpha<0.0f) && (Ubeta<0.0f) && (Ubeta/Ualpha < SQRT3)){return 4 ;}
	else if((Ubeta<0.0f) && (-Ubeta/my_abs(Ualpha)>SQRT3)){return 5 ;}
	else if((Ualpha>0.0f) && (Ubeta<0.0f) && (-Ubeta/Ualpha < SQRT3)){return 6 ;}
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
    static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;

    update_Park_N(motor);
    update_Sector(motor);

    K=(SQRT3*1)/(motor->MotorConfig.UMAX/2);
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

	float a = my_map(Ta,-1,1,0,1);
	float b = my_map(Tb,-1,1,0,1);
	float c = my_map(Tc,-1,1,0,1);

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
    
    K=(SQRT3*1)/(motor->MotorConfig.UMAX/2);
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

	float a = my_map(Ta,-1,1,0,1);
	float b = my_map(Tb,-1,1,0,1);
	float c = my_map(Tc,-1,1,0,1);
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
    
    K=(SQRT3*1)/(motor->MotorConfig.UMAX/2);
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

	float a = my_map(Ta,-1,1,0,1);
	float b = my_map(Tb,-1,1,0,1);
	float c = my_map(Tc,-1,1,0,1);
    set_pwm(motor,a,b,c);
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
    float *Upark = Calculate_Park_N(Uq , Ud , angle_el);
    float *Uclark = Calculate_Clark_N(Upark[0] , Upark[1] , motor->MotorConfig.UMAX);
    
    Uclark[0]= Uclark[0] - motor->MotorConfig.UMAX/2;
    Uclark[1]= Uclark[1] - motor->MotorConfig.UMAX/2;
    Uclark[2]= Uclark[2] - motor->MotorConfig.UMAX/2;

    motor->MotorAlg.UA = Uclark[0];
    motor->MotorAlg.UB = Uclark[1];
    motor->MotorAlg.UC = Uclark[2];
    
    float TA = my_map(Uclark[0],-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TB = my_map(Uclark[1],-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);
    float TC = my_map(Uclark[2],-motor->MotorConfig.UMAX/2,motor->MotorConfig.UMAX/2,0.0f,1.0f);

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

int* Calculate_Order_int(float IA, float IB, float IC, int PHASE)
{
    static int data_[3];
    int a = (int)IA;
    int b = (int)IB;
    int c = (int)IC;

    switch (PHASE)
    {
        case 1:
            data_[0] = a;
            data_[1] = b;
            data_[2] = c;
            break;
        case 2:
            data_[0] = b;
            data_[1] = a;
            data_[2] = c;
            break;
        case 3:
            data_[0] = c;
            data_[1] = b;
            data_[2] = a;
            break;
        case 4:
            data_[0] = c;
            data_[1] = a;
            data_[2] = b;
            break;
        case 5:
            data_[0] = b;
            data_[1] = c;
            data_[2] = a;
            break;
        case 6:
            data_[0] = a;
            data_[1] = c;
            data_[2] = b;
            break;
        default:
            data_[0] = a;
            data_[1] = b;
            data_[2] = c;
            break;
    }
    return data_;
}

float* Calculate_Order_float(float IA, float IB, float IC, int PHASE)
{
    static float data_[3];
    float a = IA;
    float b = IB;
    float c = IC;

    switch (PHASE)
    {
        case 1:
            data_[0] = a;
            data_[1] = b;
            data_[2] = c;
            break;
        case 2:
            data_[0] = b;
            data_[1] = a;
            data_[2] = c;
            break;
        case 3:
            data_[0] = c;
            data_[1] = b;
            data_[2] = a;
            break;
        case 4:
            data_[0] = c;
            data_[1] = a;
            data_[2] = b;
            break;
        case 5:
            data_[0] = b;
            data_[1] = c;
            data_[2] = a;
            break;
        case 6:
            data_[0] = a;
            data_[1] = c;
            data_[2] = b;
            break;
        default:
            data_[0] = a;
            data_[1] = b;
            data_[2] = c;
            break;
    }
    return data_;
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

    float *ph = Calculate_Order_float(motor->MotorData.IA_NoOrder, motor->MotorData.IB_NoOrder, motor->MotorData.IC_NoOrder, PHASE);
    motor->MotorAlg.IA = ph[0];
    motor->MotorAlg.IB = ph[1];
    motor->MotorAlg.IC = ph[2];
    return 1 ;
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

int update_Ioffset_nonblock(Motor_HandleTypeDef *motor,int Mode_Sampling)
{
    static uint32_t count = 0 ;
    static uint32_t IA_offset_raw_all = 0,IB_offset_raw_all = 0,IC_offset_raw_all = 0;
    if(count < 1000)
    {
        switch(Mode_Sampling)
        {
            case 0x111: /* ABC*/
            {
                if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL)
                {
                    return 0;
                }
                IA_offset_raw_all += motor->MotorDrv.Update_Ia_raw();
                IB_offset_raw_all += motor->MotorDrv.Update_Ib_raw();
                IC_offset_raw_all += motor->MotorDrv.Update_Ic_raw();
            }break;
            case 0x110: /* ABX */
            {
                if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ib_raw == NULL)
                {
                    return 0;
                }
                IA_offset_raw_all += motor->MotorDrv.Update_Ia_raw();
                IB_offset_raw_all += motor->MotorDrv.Update_Ib_raw();
                IC_offset_raw_all += 0U;
            }break;
            case 0x101: /* AXC*/
            {
                if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ia == NULL || motor->MotorDrv.Cal_Ic == NULL)
                {
                    return 0;
                }
                IA_offset_raw_all += motor->MotorDrv.Update_Ia_raw();
                IB_offset_raw_all += 0U;
                IC_offset_raw_all += motor->MotorDrv.Update_Ic_raw();

            }break;
            case 0x011: /* XBC*/
            {
                if (motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ib == NULL || motor->MotorDrv.Cal_Ic == NULL)
                {
                    return 0;
                }
                IA_offset_raw_all += 0U;
                IB_offset_raw_all += motor->MotorDrv.Update_Ib_raw();
                IC_offset_raw_all += motor->MotorDrv.Update_Ic_raw();

            }break;
            case 0x100: /* AXX */
            {
                if (motor->MotorDrv.Update_Ia_raw == NULL || motor->MotorDrv.Cal_Ia == NULL)
                {
                    return 0;
                }
                IA_offset_raw_all += motor->MotorDrv.Update_Ia_raw();
                IB_offset_raw_all += 0U;
                IC_offset_raw_all += 0U;
            }break;
            case 0x010: /* XBX*/
            {
                if (motor->MotorDrv.Update_Ib_raw == NULL || motor->MotorDrv.Cal_Ib == NULL)
                {
                    return 0;
                }
                IA_offset_raw_all += 0U;
                IB_offset_raw_all += motor->MotorDrv.Update_Ib_raw();
                IC_offset_raw_all += 0U;

            }break;
            case 0x001: /* XXC */
            {
                if (motor->MotorDrv.Update_Ic_raw == NULL || motor->MotorDrv.Cal_Ic == NULL)
                {
                    return 0;
                }
                IA_offset_raw_all += 0U;
                IB_offset_raw_all += 0U;
                IC_offset_raw_all += motor->MotorDrv.Update_Ic_raw();
            }break;
            default:
                return 0;            
        }
        count++;
        return 0 ;
    }
    else if(count >= 1000)
    {
        motor->MotorData.IA_offset_raw = IA_offset_raw_all/count;
        motor->MotorData.IB_offset_raw = IB_offset_raw_all/count;
        motor->MotorData.IC_offset_raw = IC_offset_raw_all/count; //因为IC没有偏置，所以这里直接用IB的平均值
        IA_offset_raw_all = 0;
        IB_offset_raw_all = 0;
        IC_offset_raw_all = 0;
        count = 0;
        return 1 ;
    }
    return 0 ;
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
        // printf("%f,%f,%f,%f\n",(angle_end - angle_start),velocity_integral,motor->MotorData.Velocity_raw,motor->MotorAlg.angle);
        set_svpwm(motor,motor->MotorConfig.UMAX*0.05f,0.0f,Limit_angle_el((float)i*0.01f));
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(2000);

    motor->MotorConfig.Pole_pairs = (uint32_t)my_round(my_abs((float)(1000*0.01f)/(velocity_integral)));
    // printf("%d,%f\n",motor->MotorConfig.Pole_pairs,(myabs((float)(1000*0.01f)/(velocity_integral))));
    // printf("%f,%f,%f,%f\n",(angle_end - angle_start),velocity_integral,motor->MotorData.Velocity_raw,motor->MotorAlg.angle);
    set_svpwm(motor,0.0f, 0.0f , 0.0f); 
}

void update_pole_pairs_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    static int flag = 0;
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
            }break;
            case 2:
            {
                velocity_integral += motor->MotorData.Velocity_raw*motor->time.dt;
                set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0.0f, Limit_angle_el((float)velocity_target*(total_time-time_init-time_prep)));
                // motor->MotorDrv.Delayms(1);
            }break;
            case 3:
            {
                motor->MotorConfig.Pole_pairs = (uint32_t)my_round(my_abs((float)velocity_target*(total_time-time_init-time_prep)/(velocity_integral)));
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
}

void update_2DIR_sensor_block(Motor_HandleTypeDef *motor)
{
    float velocity_target = 0.03f; 
    static float velocity_integral = 0.0f;

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
        // if(myabs(motor->MotorData.Velocity_raw) > 2*velocity_target)
        // {
        //     motor->MotorData.Velocity_raw = 0.0f ;
        // }
        set_svpwm(motor,motor->MotorConfig.UMAX*0.05f,0.0f,Limit_angle_el((float)i*velocity_target));
        velocity_integral += motor->MotorData.Velocity_raw;
        // printf("%f,%f\n",motor->MotorData.Velocity_raw,velocity_integral);
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(500);

    if(velocity_integral>0)
    {
        motor->MotorConfig.DIR = 1;
        velocity_integral = 0.0f;
    }
    else if(velocity_integral<0)
    {
        motor->MotorConfig.DIR = 2;
        velocity_integral = 0.0f;
    }
    else
    {
        /*传感器异常报错*/
    }
    set_svpwm(motor,0.0f,0.0f,0);
}

void update_2DIR_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    float velocity_target = 10.0f; 
    float time_init = 0.5f;
    float time_prep = 1.0f;
    float time_process = 2.0f;
    // float time_finish = 0.5f;

    static uint8_t state = 0 ;
    static float total_time = 0.0f ;
    static float velocity_integral = 0.0f;

    total_time += update_dt(motor); //预热
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
            float K =  (total_time-time_init)/time_process;
            set_svpwm(motor,0.0f,K*motor->MotorConfig.UMAX*0.5f, 0.0f);
        }break;
        case 2:
        {
            set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,3.0f, Limit_angle_el((float)(total_time-time_init-time_prep)*velocity_target));
            velocity_integral += motor->MotorData.Velocity_raw;
        }break;
        case 3:
        {
            if(velocity_integral>0)
            {
                motor->MotorConfig.DIR = 1;
            }
            else if(velocity_integral<0)
            {
                motor->MotorConfig.DIR = 2;
            }
            else
            {
                /*传感器异常报错*/
            }
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            total_time = 0.0f;
            velocity_integral = 0.0f;
        }
        default:
        {
            /* 打印报错信息 */
        }break;
    }
    // printf("%f,%d,%f,%f,%f\n",velocity_integral,motor->MotorConfig.DIR,motor->MotorData.Velocity_raw,motor->MotorAlg.angle,motor->time.dt);
}

int *Calculate_PHASE(float IA, float IB, float IC,float UA, float UB, float UC)//同时兼容直接向三相注入IqId时的工况，也就是在电机运行的情况下进行相序辨识
{   
    static int PHASE[3] = {0,0,0};
    float U[3] = {0,0,0};
    float I[3] = {0,0,0};
    
    U[0] = UA;
    U[1] = UB;
    U[2] = UC;

    I[0] = IA;
    I[1] = IB;
    I[2] = IC;

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

    return PHASE;
}
int update_PHASE_nonblock(Motor_HandleTypeDef *motor,float IA_NoOrder, float IB_NoOrder, float IC_NoOrder)
{
    static float Ts = 1.0f;
    static float Duty = 0.05f;

    static float time = 0.0f ;//中间变量
    static int state = 1 ;
    static int PHASE[3] = {0,0,0};
    static float IA_Integral , IB_Integral , IC_Integral;
    static Time_t time_phase;

    time += motor->MotorDrv.Update_dt(&time_phase);
    if(time>(float)state * Ts)
    {
        state ++ ;
    }

    switch(state)
    {
        case 1 :
        {
            motor->MotorConfig.PHASE = -1; //先将相序设置为-1，表示正在检测相序
            set_pwm(motor,0.0f,0.0f,0.0f);
        }break;
        case 2 :
        {
            IA_Integral = 0.0f;
            IB_Integral = 0.0f;
            IC_Integral = 0.0f;
            set_pwm(motor,Duty,0.0f,0.0f);
        }break;
        case 3 :
        {
            IA_Integral += IA_NoOrder * time_phase.dt;
            IB_Integral += IB_NoOrder * time_phase.dt;
            IC_Integral += IC_NoOrder * time_phase.dt;
            int *PHASE_ = Calculate_PHASE(IA_Integral , IB_Integral , IC_Integral , motor->MotorConfig.UMAX/2*Duty , 0.0f , 0.0f);
            PHASE[0] = PHASE_[0];
        }break;
        case 4:
        {
            IA_Integral = 0.0f;
            IB_Integral = 0.0f;
            IC_Integral = 0.0f;
            set_pwm(motor,0.0f,Duty,0.0f);
        }break;
        case 5:
        {
            IA_Integral += IA_NoOrder * time_phase.dt;
            IB_Integral += IB_NoOrder * time_phase.dt;
            IC_Integral += IC_NoOrder * time_phase.dt;
            int *PHASE_ = Calculate_PHASE(IA_Integral , IB_Integral , IC_Integral , 0.0f , motor->MotorConfig.UMAX/2*Duty , 0.0f);
            PHASE[1] = PHASE_[1];            
        }break;
        case 6:
        {
            IA_Integral = 0.0f;
            IB_Integral = 0.0f;
            IC_Integral = 0.0f;
            set_pwm(motor,0.0f,0.0f,Duty);
        }break;
        case 7:
        {
            IA_Integral += IA_NoOrder * time_phase.dt;
            IB_Integral += IB_NoOrder * time_phase.dt;
            IC_Integral += IC_NoOrder * time_phase.dt;
            int *PHASE_ = Calculate_PHASE(IA_Integral , IB_Integral , IC_Integral , 0.0f , 0.0f , motor->MotorConfig.UMAX/2*Duty);
            PHASE[2] = PHASE_[2];   
        }break;
        case 8:
        {
            if( PHASE[0] == 1 && PHASE[1] == 2 && PHASE[2] == 3 )
            {
                motor->MotorConfig.PHASE = 1;
            }
            else if ( PHASE[0] == 2 && PHASE[1] == 1 && PHASE[2] == 3 )
            {
                motor->MotorConfig.PHASE = 2;
            }
            else if ( PHASE[0] == 3 && PHASE[1] == 2 && PHASE[2] == 1 )
            {
                motor->MotorConfig.PHASE = 3;
            }
            else if ( PHASE[0] == 3 && PHASE[1] == 1 && PHASE[2] == 2 )
            {
                motor->MotorConfig.PHASE = 4;
            }
            else if ( PHASE[0] == 2 && PHASE[1] == 3 && PHASE[2] == 1 )
            {
                motor->MotorConfig.PHASE = 5;
            }
            else if ( PHASE[0] == 1 && PHASE[1] == 3 && PHASE[2] == 2 )
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
            IA_Integral = 0.0f;
            IB_Integral = 0.0f;
            IC_Integral = 0.0f;
            time = 0.0f;
            state = 1 ;
            return motor->MotorConfig.PHASE ;
        }break;
    }
    return 0 ;
}

int update_PHASE_block(Motor_HandleTypeDef *motor)
{
    update_dt(motor);
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
    uint32_t sample_per = 20 ; //每极对采样20个
    uint32_t sample_total = motor->MotorConfig.Pole_pairs * sample_per; //总采样数
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
    // motor->MotorData.angle_all = update_angle(motor);
    // float angle_last = motor->MotorData.angle_all;
    float angle_now = 0.0f;
    do
    {
        update_dt(motor);
        update_angle(motor);
        angle_now = ctrl_motor_openloop_angle_nonblock(motor,2*PI,0.0f,0.6,0.0f,motor->MotorConfig.UMAX*0.05f);
        // angle_error = angle_now - motor->MotorData.angle_all;
        uint32_t i =(uint32_t)my_round((angle_now/(2*PI))*(float)sample_total);
        int32_t i_int = (int32_t)my_round((angle_now/(2*PI))*(float)sample_total);
        if(i>=sample_total|| i_int<0)
        {
            ctrl_motor_openloop_angle_nonblock(motor,0.0f,0.0f,1000.0f,0.0f,0.0f);//注销掉这个函数的angle_now，防止用到下一次开环执行程序中
            break;
        }
        angle_el_zero[i] = angle_now - motor->MotorData.angle_all;

        // printf("%d,%f,%f\n",i,angle_el_zero[i],angle_now);

    } while (angle_now);
    motor->MotorDrv.Delayms(500);
    do
    {
        update_dt(motor);
        update_angle(motor);
        angle_now = ctrl_motor_openloop_angle_nonblock(motor,0.0f,2*PI,-0.6,0.0f,motor->MotorConfig.UMAX*0.05f);
        // angle_error = angle_now - motor->MotorData.angle_all;
        uint32_t i =(uint32_t)my_round((angle_now/(2*PI))*(float)sample_total);
        int32_t i_int = (int32_t)my_round((angle_now/(2*PI))*(float)sample_total);
        if(i>=sample_total || i_int<0)
        {
            ctrl_motor_openloop_angle_nonblock(motor,0.0f,0.0f,1000.0f,0.0f,0.0f);//注销掉这个函数的angle_now，防止用到下一次开环执行程序中
            break;
        }
        angle_el_zero[i] += angle_now - motor->MotorData.angle_all;
        angle_el_zero[i] /= 2;

        // printf("%d,%f,%f\n",i,angle_el_zero[i],angle_now);
    } while (angle_now);
    // printf("%f\n",angle_now);
    for(int i = 0; i<sample_total ;i++)
    {
        angle_el_zero_all += angle_el_zero[i];
    }
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
                // printf("Heap_Size is not enough!");
                SEGGER_RTT_printf(0, "Heap_Size is not enough!\n");
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
            uint32_t i =(uint32_t)my_round((angle_now/(2*PI))*(float)sample_total);
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
            uint32_t i =(uint32_t)my_round((angle_now/(2*PI))*(float)sample_total);
            int32_t i_int =(int32_t)my_round((angle_now/(2*PI))*(float)sample_total);
            // printf("%f\n",angle_el_zero[i]);
            if(i>=sample_total)
            {
                --i;
            }
            if(i_int<0 || !angle_now)
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
    if(my_abs(velocity_integral) < my_abs(angle_el_target - angle_el_start))
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
    if( my_abs(velocity_integral) < my_abs(angle_target-angle_start))
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
    float Ibeta = (SQRT3 * (IB - IC)) / 3.0f;

    // --- 第二步: Park 变换 (2相静止 -> 2相同步旋转) ---
    float cos_theta = my_cos(angle_el);
    float sin_theta = my_sin(angle_el);

    // Id = Ialpha * cosθ + Ibeta * sinθ
    IdIq_out[0] = Ialpha * cos_theta + Ibeta * sin_theta;
    
    // Iq = -Ialpha * sinθ + Ibeta * cosθ
    IdIq_out[1] = -Ialpha * sin_theta + Ibeta * cos_theta;
}

void ctrl_motor_openloop_angle_block(Motor_HandleTypeDef *motor,float angle_target,float angle_start,float velocity_target ,float Uq,float Ud)
{
    while(ctrl_motor_openloop_angle_nonblock(motor,angle_target,angle_start,velocity_target,Uq,Ud))
    {
        static float iqid[2] = {0.0f,0.0f} ;
        update_dt(motor);
        update_angle(motor);
        Calculate_IdIq(motor->MotorAlg.IA,motor->MotorAlg.IB,motor->MotorAlg.IC,motor->MotorAlg.angle_el,iqid);
    }
}

