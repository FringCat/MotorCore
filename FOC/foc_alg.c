/*
 * SPDX-FileCopyrightText: 2026 FryingCat <3551901875@qq.com>
 * SPDX-License-Identifier: MIT
 */

#include "foc_alg.h"
#include "foc_drv.h"

void log_motor(Motor_HandleTypeDef *motor, const char *msg)
{
    if (motor->motor_drv.log && msg)
    {
        motor->motor_drv.log(msg);
    }
}

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

float my_map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return ((x-in_min)*((float)((out_max-out_min)/(float)(in_max-in_min))))+out_min;
}

float my_sin(float x)
{
    float sign = 1.0f;

    /* 输入应在 [0, 2π)；调用方先用 limit_angle_el 归一化 */
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
    motor->motor_data.loopcount = 0;
    motor->motor_alg.last_angle = 0.0f;
    motor->motor_alg.angle = 0.0f;
}

float limit_angle(float angle, float low, float high)
{
    // 异常处理：若上下限差值过小（周期为0），返回NaN标识错误
    const float EPS = 1e-6f;
    float period = high - low;
    if (my_abs(period) < EPS)
    {
        return 0;  
    }

    if (period < 0.0f)
    {
        float temp = low;
        low = high;
        high = temp;
        period = -period; 
    }

    float offset = angle - low;

    offset = my_fmodf(offset, period);

    if (offset < 0.0f)
    {
        offset += period;
    }

    float angle_limited = low + offset;

    return angle_limited;
}

float limit_angle_el(float angle_el)
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

float update_angle(Motor_HandleTypeDef *motor)
{
    float last = motor->motor_alg.angle;

    motor->motor_data.angle_data.angle_raw = motor->motor_drv.update_angle_raw();
    motor->motor_alg.angle = motor->motor_drv.cal_angle(motor->motor_data.angle_data.angle_raw);
    motor->motor_alg.angle_el = calculate_angle_el(motor->motor_config.pole_pairs,motor->motor_alg.angle, motor->motor_config.angle_el_zero);

    float error_angle = motor->motor_alg.angle - last;
    if(my_abs(error_angle) > (0.8f*2*PI))
    {
        if(error_angle < 0){motor->motor_data.loopcount++;}//正转过零
        else {motor->motor_data.loopcount--;}//反转过零
    }

    motor->motor_alg.last_angle = last;
    return motor->motor_alg.angle;
}

float calculate_angle_all(int32_t loopcount, float angle)
{
    return (float)loopcount * (2.0f * PI) + angle;
}

float get_angle_all(Motor_HandleTypeDef *motor)
{
    return calculate_angle_all(motor->motor_data.loopcount, motor->motor_alg.angle);
}

float get_angle_el(Motor_HandleTypeDef *motor) 
{
    return motor->motor_alg.angle_el;
}

float calculate_angle_el(float pole_pairs,float angle,float angle_el_zero) 
{
    return limit_angle_el(angle * pole_pairs + angle_el_zero);
}

float update_angle_el(Motor_HandleTypeDef *motor) 
{
    motor->motor_alg.angle = motor->motor_drv.cal_angle(motor->motor_drv.update_angle_raw());
    motor->motor_alg.angle_el = calculate_angle_el(motor->motor_config.pole_pairs,motor->motor_alg.angle, motor->motor_config.angle_el_zero);
    return motor->motor_alg.angle_el;
}

float my_limit(float value , float high , float low)
{
    return (value)<(low)?(low):((value)>(high)?(high):(value));//如果目标参数超出最大/最小值的范围，就把这个值锁死在最大/最小值
}

void update_sincos(Motor_HandleTypeDef *motor)
{
    float angle_el = motor->motor_alg.angle_el;
    motor->motor_alg.sin_theta = my_sin(angle_el);
    motor->motor_alg.cos_theta = my_cos(angle_el);
}

void calculate_park_n_theta(float uq , float ud , float angle_el, float *u_alpha, float *u_beta)
{
    float cos_theta = my_cos(angle_el);
    float sin_theta = my_sin(angle_el);

    *u_alpha = ud * cos_theta - uq * sin_theta;
    *u_beta  = uq * cos_theta + ud * sin_theta;
}

void calculate_park_n_sincos(float uq , float ud , float sin_theta ,float cos_theta, float *u_alpha, float *u_beta)
{
    *u_alpha = ud * cos_theta - uq * sin_theta;
    *u_beta  = uq * cos_theta + ud * sin_theta;
}

void update_park_n(Motor_HandleTypeDef *motor)
{
    calculate_park_n_sincos(motor->motor_alg.uq , motor->motor_alg.ud , motor->motor_alg.sin_theta,motor->motor_alg.cos_theta,&motor->motor_alg.u_alpha, &motor->motor_alg.u_beta);
}

void calculate_clark_n(float u_alpha ,float u_beta,float upower, float *ua, float *ub, float *uc)
{
    //Clark逆变换:
    *ua = u_alpha + upower/2;                 // ①Ua = u_alpha ;
    *ub = (SQRT3*u_beta-u_alpha)/2 + upower/2; // ②Ub = (√3 * u_beta - u_alpha)/2 ;
    *uc = -(u_alpha + SQRT3*u_beta)/2 + upower/2;// ③Uc = ( -u_alpha - √3 * u_beta )/2;
}

void update_clark_n(Motor_HandleTypeDef *motor)
{
    calculate_clark_n(motor->motor_alg.u_alpha , motor->motor_alg.u_beta , motor->motor_config.u_max,
                      &motor->motor_alg.ua, &motor->motor_alg.ub, &motor->motor_alg.uc);
}

void calculate_clark(float ia ,float ib ,float ic, float *i_alpha, float *i_beta)
{
    // 幅值不变型Clark变换（对称三相电流）
    *i_alpha = (2*ia - ib - ic)*_1_3;  // Iα
    *i_beta  = (ib - ic)*_1_SQRT3;       // Iβ
}

void update_clark(Motor_HandleTypeDef *motor)
{
    calculate_clark(motor->motor_alg.ia , motor->motor_alg.ib , motor->motor_alg.ic,
                    &motor->motor_alg.i_alpha, &motor->motor_alg.i_beta);
}

void calculate_park_theta(float i_alpha ,float i_beta ,float angle_el_rad, float *id, float *iq)
{
    float cos_theta = my_cos(angle_el_rad);
    float sin_theta = my_sin(angle_el_rad);

    // Park变换（输入电角度为弧度制）
    *id = i_alpha * cos_theta + i_beta * sin_theta;
    *iq = -i_alpha * sin_theta + i_beta * cos_theta;
}

void calculate_park_sincos(float i_alpha ,float i_beta ,float sin_theta,float cos_theta, float *id, float *iq)
{
    *id = i_alpha * cos_theta + i_beta * sin_theta;
    *iq = -i_alpha * sin_theta + i_beta * cos_theta;
}

void update_park(Motor_HandleTypeDef *motor)
{
    calculate_park_sincos(motor->motor_alg.i_alpha , motor->motor_alg.i_beta , motor->motor_alg.sin_theta,motor->motor_alg.cos_theta,&motor->motor_alg.id, &motor->motor_alg.iq);
}

void update_pwm(Motor_HandleTypeDef *motor)
{
    float _ua = my_limit(motor->motor_alg.ua/motor->motor_config.u_max , 1 , 0 );//计算并限制ABC相所需的占空比
	float _ub = my_limit(motor->motor_alg.ub/motor->motor_config.u_max , 1 , 0 );
	float _uc = my_limit(motor->motor_alg.uc/motor->motor_config.u_max , 1 , 0 );

    if(motor->motor_drv.set_pwm_a || motor->motor_drv.set_pwm_b || motor->motor_drv.set_pwm_c)
    {
        switch (motor->motor_config.dir)
        {
            case 1:
            {
                motor->motor_drv.set_pwm_a(_ua);
                motor->motor_drv.set_pwm_b(_ub);
                motor->motor_drv.set_pwm_c(_uc);
            }break;
            case -1:
            {
                motor->motor_drv.set_pwm_a(_ub);
                motor->motor_drv.set_pwm_b(_ua);
                motor->motor_drv.set_pwm_c(_uc);
            }break;
            default:
            {
                motor->motor_drv.set_pwm_a(_ua);
                motor->motor_drv.set_pwm_b(_ub);
                motor->motor_drv.set_pwm_c(_uc);
            }break;
        }
    }
    else
    {
        log_motor(motor, "pwm callbacks not bound");
    }
}

void set_pwm(Motor_HandleTypeDef *motor,float ta , float tb ,float tc)
{
    float _ua = my_limit(ta , 1 , 0 );//计算并限制ABC相所需的占空比
	float _ub = my_limit(tb , 1 , 0 );
	float _uc = my_limit(tc , 1 , 0 );

    if(motor->motor_drv.set_pwm_a ||motor->motor_drv.set_pwm_b || motor->motor_drv.set_pwm_c)
    {
        switch (motor->motor_config.dir)
        {
            case 1:
            {
                motor->motor_drv.set_pwm_a(_ua);
                motor->motor_drv.set_pwm_b(_ub);
                motor->motor_drv.set_pwm_c(_uc);
            }break;
            case -1:
            {
                motor->motor_drv.set_pwm_a(_ub);
                motor->motor_drv.set_pwm_b(_ua);
                motor->motor_drv.set_pwm_c(_uc);
            }break;
            default:
            {
                motor->motor_drv.set_pwm_a(_ua);
                motor->motor_drv.set_pwm_b(_ub);
                motor->motor_drv.set_pwm_c(_uc);
            }break;
        }
    }
    else
    {
        log_motor(motor, "pwm callbacks not bound");
    }
}

void set_pwm_nodir(Motor_HandleTypeDef *motor,float ta , float tb ,float tc)
{
    float _ua = my_limit(ta , 1 , 0 );//计算并限制ABC相所需的占空比
    float _ub = my_limit(tb , 1 , 0 );
    float _uc = my_limit(tc , 1 , 0 );

    if(motor->motor_drv.set_pwm_a || motor->motor_drv.set_pwm_b ||motor->motor_drv.set_pwm_c)
    {
        motor->motor_drv.set_pwm_a(_ua);
        motor->motor_drv.set_pwm_b(_ub);
        motor->motor_drv.set_pwm_c(_uc);
    }
    else
    {
        log_motor(motor, "pwm callbacks not bound");
    }
}

int calculate_svpwm_sector( float u_alpha , float u_beta )
{
    int A = (u_beta > 0.0f) ? 1 : 0;
    int B = ((SQRT3 * u_alpha - u_beta) > 0.0f) ? 1 : 0;
    int C = ((-SQRT3 * u_alpha - u_beta) > 0.0f) ? 1 : 0;
    int N = A + 2 * B + 4 * C;

    switch (N)
    {
        case 1: return 2;
        case 2: return 6;
        case 3: return 1;
        case 4: return 4;
        case 5: return 3;
        case 6: return 5;
        default: return 1; /* u_alpha = u_beta = 0 */
    }
}

int update_svpwm_sector(Motor_HandleTypeDef *motor)
{
    motor->motor_alg.sector = calculate_svpwm_sector(motor->motor_alg.u_alpha , motor->motor_alg.u_beta);
    return motor->motor_alg.sector;
}

int get_svpwm_sector(Motor_HandleTypeDef *motor)
{
    return motor->motor_alg.sector;
}

float calculate_lpf(float input, float last_output, float alpha)
{
    return alpha * input + (1 - alpha) * last_output;
}

float update_velocity_lpf(Motor_HandleTypeDef *motor)  
{
    motor->motor_data.velocity_raw = calculate_velocity_raw(motor->motor_alg.angle, motor->motor_alg.last_angle, motor->time.dt);
    motor->motor_alg.velocity =  calculate_lpf(motor->motor_data.velocity_raw, motor->motor_data.velocity_lpf.last_output, motor->motor_data.velocity_lpf.alpha);
    motor->motor_data.velocity_lpf.last_output = motor->motor_alg.velocity; // 更新滤波器的上次输出值
    return motor->motor_alg.velocity;
}

float update_velocity_raw(Motor_HandleTypeDef *motor)
{
    motor->motor_data.velocity_raw = calculate_velocity_raw(motor->motor_alg.angle, motor->motor_alg.last_angle, motor->time.dt);
    return motor->motor_data.velocity_raw;
}

float calculate_velocity_raw(float angle, float last_angle, float dt)
{
    if (dt <= 0) 
    {
        return 0; // 避免除以零
    }

    float velocity_raw = 0.0f;
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

float calculate_velocity_lpf(float angle, float last_angle, float dt, float last_velocity, float alpha)
{
    float velocity_raw = calculate_velocity_raw(angle, last_angle, dt);
    return calculate_lpf(velocity_raw, last_velocity, alpha);
}

float get_velocity(Motor_HandleTypeDef *motor)
{
    return motor->motor_alg.velocity;
}

float get_velocity_raw(Motor_HandleTypeDef *motor)
{
    return motor->motor_data.velocity_raw;
}

float calculate_pid(float target, float feedback, float dt ,PID_t* pid)
{
    pid->error = target - feedback;
    
    // 计算积分项
    pid->this_i += pid->error * dt ;
    pid->this_i = my_limit(pid->this_i, pid->integral_max, pid->integral_min); // 限制积分项防止积分饱和

    // 计算微分项
    float derivative = (pid->error - pid->last_error) / dt;

    // 计算PID输出
    pid->output = pid->kp * pid->error + pid->ki * pid->this_i + pid->kd * derivative;
    // pid->output = pid->kp * pid->error;
    pid->output = my_limit(pid->output, pid->output_max, pid->output_min); // 限制输出范围

    // 更新历史误差
    pid->last_error = pid->error;

    return pid->output;
}

float calculate_pid_is(float target, float feedback, float dt, PID_t* pid,float sep_err_upper, float sep_err_lower)   //积分分离
{
    pid->error = target - feedback;
    
    if ((pid->error >= sep_err_lower) && (pid->error <= sep_err_upper))
    {
        pid->this_i += pid->error * dt;
    }
    else
    {
        pid->this_i = 0.0f; // 超出范围时，积分项清零
    }
    pid->this_i = my_limit(pid->this_i, pid->integral_max, pid->integral_min);

    float derivative = (pid->error - pid->last_error) / dt;
    pid->output = pid->kp * pid->error + pid->ki * pid->this_i + pid->kd * derivative;
    pid->output = my_limit(pid->output, pid->output_max, pid->output_min);

    pid->last_error = pid->error;
    return pid->output;
}

float calculate_pid_is_ais(float target, float feedback, float dt, PID_t* pid,float n)   //自适应积分分离
{
    float r = pid->kp;
    float D = target;

    float sum = r * D * ((1.0f - my_pow((1.0f - r), n)) / r);

    pid->error = target - feedback;

    if (my_abs(pid->error) <= my_abs(target-sum))
    {
        pid->this_i += pid->error * dt;
    }
    else
    {
        pid->this_i = 0.0f; // 超出范围时，积分项清零
    }
    pid->this_i = my_limit(pid->this_i, pid->integral_max, pid->integral_min);

    float derivative = (pid->error - pid->last_error) / dt;
    pid->output = pid->kp * pid->error + pid->ki * pid->this_i + pid->kd * derivative;
    pid->output = my_limit(pid->output, pid->output_max, pid->output_min);

    pid->last_error = pid->error;
    return pid->output;
}

void update_svpwm(Motor_HandleTypeDef *motor)
{
    float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	float ta = 0 , tb = 0 ,tc = 0 ;

    update_park_n(motor);
    update_svpwm_sector(motor);

    K=SQRT3/motor->motor_config.u_max;
	Ux = motor->motor_alg.u_beta;
	Uy = (SQRT3/2.0f)*motor->motor_alg.u_alpha - 0.5f*motor->motor_alg.u_beta;
	Uz = (SQRT3/2.0f)*motor->motor_alg.u_alpha + 0.5f*motor->motor_alg.u_beta;

    switch (motor->motor_alg.sector)
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
		log_motor(motor, "svpwm sector invalid");
		break;
	}
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}
	Tz = 0.5f*(1-Tx-Ty) ;

	switch(motor->motor_alg.sector)
	{
		case 1 : 
			tc = Tz ;
			tb = Tz + Ty ;
			ta = Tz + Ty + Tx ;
			break;
		case 2 : 
			tc = Tz ;
			ta = Tz + Ty ;
			tb = Tz + Ty + Tx ;			
			break;
		case 3 : 
			ta = Tz ;
			tc = Tz + Ty ;
			tb = Tz + Ty + Tx ;	
			break;
		case 4 : 
			ta = Tz ;
			tb = Tz + Ty ;
			tc = Tz + Ty + Tx ;	
			break;
		case 5 : 
			tb = Tz ;
			ta = Tz + Ty ;
			tc = Tz + Ty + Tx ;	
			break;
		case 6 : 
			tb = Tz ;
			tc = Tz + Ty ;
			ta = Tz + Ty + Tx ;
			break;
		default:
			break;
	}

    motor->motor_alg.ua = ta*motor->motor_config.u_max - motor->motor_config.u_max/2;
    motor->motor_alg.ub = tb*motor->motor_config.u_max - motor->motor_config.u_max/2;
    motor->motor_alg.uc = tc*motor->motor_config.u_max - motor->motor_config.u_max/2;

    set_pwm(motor,ta,tb,tc);
}

void set_svpwm(Motor_HandleTypeDef *motor, float uq , float ud ,float angle_el)
{
    float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	float ta = 0 , tb = 0 ,tc = 0 ;
    float u_alpha = 0 , u_beta = 0 ;

    calculate_park_n_theta(uq , ud , angle_el, &u_alpha, &u_beta);

    int sector = calculate_svpwm_sector(u_alpha , u_beta);
    
    K=SQRT3/motor->motor_config.u_max;
    Ux = u_beta;
    Uy = (SQRT3/2.0f)*u_alpha - 0.5f*u_beta;
    Uz = (SQRT3/2.0f)*u_alpha + 0.5f*u_beta;

    switch (sector)
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

	switch(sector)
	{
		case 1 : 
			tc = Tz ;
			tb = Tz + Ty ;
			ta = Tz + Ty + Tx ;
			break;
		case 2 : 
			tc = Tz ;
			ta = Tz + Ty ;
			tb = Tz + Ty + Tx ;			
			break;
		case 3 : 
			ta = Tz ;
			tc = Tz + Ty ;
			tb = Tz + Ty + Tx ;	
			break;
		case 4 : 
			ta = Tz ;
			tb = Tz + Ty ;
			tc = Tz + Ty + Tx ;	
			break;
		case 5 : 
			tb = Tz ;
			ta = Tz + Ty ;
			tc = Tz + Ty + Tx ;	
			break;
		case 6 : 
			tb = Tz ;
			tc = Tz + Ty ;
			ta = Tz + Ty + Tx ;
			break;
		default:
			break;
	}

    motor->motor_alg.ua = ta*motor->motor_config.u_max - motor->motor_config.u_max/2;
    motor->motor_alg.ub = tb*motor->motor_config.u_max - motor->motor_config.u_max/2;
    motor->motor_alg.uc = tc*motor->motor_config.u_max - motor->motor_config.u_max/2;

    set_pwm_nodir(motor,ta,tb,tc);
}

void set_svpwm_dir(Motor_HandleTypeDef *motor, float uq , float ud ,float angle_el)
{
    float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	float ta = 0 , tb = 0 ,tc = 0 ;
    float u_alpha = 0 , u_beta = 0 ;

    calculate_park_n_theta(uq , ud , angle_el, &u_alpha, &u_beta);

    motor->motor_alg.u_alpha = u_alpha;//打印
    motor->motor_alg.u_beta = u_beta;
    
    int sector = calculate_svpwm_sector(u_alpha , u_beta);
    
    K=SQRT3/motor->motor_config.u_max;
    Ux = u_beta;
    Uy = (SQRT3/2.0f)*u_alpha - 0.5f*u_beta;
    Uz = (SQRT3/2.0f)*u_alpha + 0.5f*u_beta;

    switch (sector)
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

	switch(sector)
	{
		case 1 : 
			tc = Tz ;
			tb = Tz + Ty ;
			ta = Tz + Ty + Tx ;
			break;
		case 2 : 
			tc = Tz ;
			ta = Tz + Ty ;
			tb = Tz + Ty + Tx ;			
			break;
		case 3 : 
			ta = Tz ;
			tc = Tz + Ty ;
			tb = Tz + Ty + Tx ;	
			break;
		case 4 : 
			ta = Tz ;
			tb = Tz + Ty ;
			tc = Tz + Ty + Tx ;	
			break;
		case 5 : 
			tb = Tz ;
			ta = Tz + Ty ;
			tc = Tz + Ty + Tx ;	
			break;
		case 6 : 
			tb = Tz ;
			tc = Tz + Ty ;
			ta = Tz + Ty + Tx ;
			break;
		default:
			break;
	}

    motor->motor_alg.ua = ta*motor->motor_config.u_max - motor->motor_config.u_max/2;
    motor->motor_alg.ub = tb*motor->motor_config.u_max - motor->motor_config.u_max/2;
    motor->motor_alg.uc = tc*motor->motor_config.u_max - motor->motor_config.u_max/2;

    set_pwm(motor,ta,tb,tc);
}

void update_spwm(Motor_HandleTypeDef *motor)
{
    update_park_n(motor);
    update_clark_n(motor);

    float ta = my_map(motor->motor_alg.ua,-motor->motor_config.u_max/2,motor->motor_config.u_max/2,0.0f,1.0f);
    float tb = my_map(motor->motor_alg.ub,-motor->motor_config.u_max/2,motor->motor_config.u_max/2,0.0f,1.0f);
    float tc = my_map(motor->motor_alg.uc,-motor->motor_config.u_max/2,motor->motor_config.u_max/2,0.0f,1.0f);
    
    set_pwm(motor,ta, tb, tc);
}

void set_spwm(Motor_HandleTypeDef *motor,float uq, float ud ,float angle_el)
{
    float u_alpha, u_beta;
    float ua, ub, uc;

    calculate_park_n_theta(uq , ud , angle_el, &u_alpha, &u_beta);
    calculate_clark_n(u_alpha , u_beta , motor->motor_config.u_max, &ua, &ub, &uc);

    ua = ua - motor->motor_config.u_max/2;
    ub = ub - motor->motor_config.u_max/2;
    uc = uc - motor->motor_config.u_max/2;

    motor->motor_alg.ua = ua;
    motor->motor_alg.ub = ub;
    motor->motor_alg.uc = uc;

    float ta = my_map(ua,-motor->motor_config.u_max/2,motor->motor_config.u_max/2,0.0f,1.0f);
    float tb = my_map(ub,-motor->motor_config.u_max/2,motor->motor_config.u_max/2,0.0f,1.0f);
    float tc = my_map(uc,-motor->motor_config.u_max/2,motor->motor_config.u_max/2,0.0f,1.0f);

    set_pwm(motor,ta, tb, tc);
}

float get_dt(Motor_HandleTypeDef *motor)
{
    return motor->time.dt;
}

float update_dt(Motor_HandleTypeDef *motor)
{
    motor->motor_drv.update_dt(&motor->time);
    return motor->time.dt;
}

void calculate_order_int(float ia, float ib, float ic, int phase, int *IA_out, int *IB_out, int *IC_out)
{
    int a = (int)ia;
    int b = (int)ib;
    int c = (int)ic;

    switch (phase)
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

void calculate_order_float(float ia, float ib, float ic, int phase, float *IA_out, float *IB_out, float *IC_out)
{
    float a = ia;
    float b = ib;
    float c = ic;

    switch (phase)
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

int update_iaibic(Motor_HandleTypeDef *motor,int mode_sampling,int phase)
{
    switch (mode_sampling)
    {
        case 0x111: /* ABC*/
        {
            if (!motor->motor_drv.update_ia_raw || !motor->motor_drv.update_ib_raw || !motor->motor_drv.update_ic_raw)
            {
                return 0;
            }
            motor->motor_data.current_data.i_raw.ia_raw = motor->motor_drv.update_ia_raw();
            motor->motor_data.current_data.i_raw.ib_raw = motor->motor_drv.update_ib_raw();
            motor->motor_data.current_data.i_raw.ic_raw = motor->motor_drv.update_ic_raw();

            motor->motor_data.ia_no_order= motor->motor_drv.cal_ia(motor->motor_data.current_data.i_raw.ia_raw, motor->motor_data.ia_offset_raw);
            motor->motor_data.ib_no_order = motor->motor_drv.cal_ib(motor->motor_data.current_data.i_raw.ib_raw, motor->motor_data.ib_offset_raw);
            motor->motor_data.ic_no_order = motor->motor_drv.cal_ic(motor->motor_data.current_data.i_raw.ic_raw, motor->motor_data.ic_offset_raw);
        }break;
        case 0x110: /* ABX */
        {
            if (!motor->motor_drv.update_ia_raw || !motor->motor_drv.update_ib_raw)
            {
                return 0;
            }
            motor->motor_data.current_data.i_raw.ia_raw = motor->motor_drv.update_ia_raw();
            motor->motor_data.current_data.i_raw.ib_raw = motor->motor_drv.update_ib_raw();
            motor->motor_data.current_data.i_raw.ic_raw = 0U;

            motor->motor_data.ia_no_order = motor->motor_drv.cal_ia(motor->motor_data.current_data.i_raw.ia_raw, motor->motor_data.ia_offset_raw);
            motor->motor_data.ib_no_order = motor->motor_drv.cal_ib(motor->motor_data.current_data.i_raw.ib_raw, motor->motor_data.ib_offset_raw);
            motor->motor_data.ic_no_order = -(motor->motor_data.ia_no_order + motor->motor_data.ib_no_order);
        }break;
        case 0x101: /* AXC*/
        {
            if (!motor->motor_drv.update_ia_raw || !motor->motor_drv.update_ic_raw || !motor->motor_drv.cal_ia || !motor->motor_drv.cal_ic)
            {
                return 0;
            }
            motor->motor_data.current_data.i_raw.ia_raw = motor->motor_drv.update_ia_raw();
            motor->motor_data.current_data.i_raw.ib_raw = 0U;
            motor->motor_data.current_data.i_raw.ic_raw = motor->motor_drv.update_ic_raw();

            motor->motor_data.ia_no_order = motor->motor_drv.cal_ia(motor->motor_data.current_data.i_raw.ia_raw, motor->motor_data.ia_offset_raw);
            motor->motor_data.ic_no_order = motor->motor_drv.cal_ic(motor->motor_data.current_data.i_raw.ic_raw, motor->motor_data.ic_offset_raw);
            motor->motor_data.ib_no_order = -(motor->motor_data.ia_no_order + motor->motor_data.ic_no_order);
        }break;
        case 0x011: /* XBC*/
        {
            if (!motor->motor_drv.update_ib_raw || !motor->motor_drv.update_ic_raw || !motor->motor_drv.cal_ib || !motor->motor_drv.cal_ic)
            {
                return 0;
            }
            motor->motor_data.current_data.i_raw.ia_raw = 0U;
            motor->motor_data.current_data.i_raw.ib_raw = motor->motor_drv.update_ib_raw();
            motor->motor_data.current_data.i_raw.ic_raw = motor->motor_drv.update_ic_raw();

            motor->motor_data.ib_no_order = motor->motor_drv.cal_ib(motor->motor_data.current_data.i_raw.ib_raw, motor->motor_data.ib_offset_raw);
            motor->motor_data.ic_no_order = motor->motor_drv.cal_ic(motor->motor_data.current_data.i_raw.ic_raw, motor->motor_data.ic_offset_raw);
            motor->motor_data.ia_no_order = -(motor->motor_data.ib_no_order + motor->motor_data.ic_no_order);
        }break;
        case 0x100: /* AXX */
        {
            if (!motor->motor_drv.update_ia_raw || !motor->motor_drv.cal_ia)
            {
                return 0;
            }
            motor->motor_data.current_data.i_raw.ia_raw = motor->motor_drv.update_ia_raw();
            motor->motor_data.current_data.i_raw.ib_raw = 0U;
            motor->motor_data.current_data.i_raw.ic_raw = 0U;

            motor->motor_data.ia_no_order = motor->motor_drv.cal_ia(motor->motor_data.current_data.i_raw.ia_raw, motor->motor_data.ia_offset_raw);
            motor->motor_data.ib_no_order = 0.0f;
            motor->motor_data.ic_no_order = 0.0f;
        }break;
        case 0x010: /* XBX*/
        {
            if (!motor->motor_drv.update_ib_raw || !motor->motor_drv.cal_ib)
            {
                return 0;
            }
            motor->motor_data.current_data.i_raw.ia_raw = 0U;
            motor->motor_data.current_data.i_raw.ib_raw = motor->motor_drv.update_ib_raw();
            motor->motor_data.current_data.i_raw.ic_raw = 0U;

            motor->motor_data.ia_no_order = 0.0f;
            motor->motor_data.ib_no_order = motor->motor_drv.cal_ib(motor->motor_data.current_data.i_raw.ib_raw, motor->motor_data.ib_offset_raw);
            motor->motor_data.ic_no_order = 0.0f;
        }break;
        case 0x001: /* XXC */
        {
            if (!motor->motor_drv.update_ic_raw || !motor->motor_drv.cal_ic)
            {
                return 0;
            }
            motor->motor_data.current_data.i_raw.ia_raw = 0U;
            motor->motor_data.current_data.i_raw.ib_raw = 0U;
            motor->motor_data.current_data.i_raw.ic_raw = motor->motor_drv.update_ic_raw();

            motor->motor_data.ia_no_order = 0.0f;
            motor->motor_data.ib_no_order = 0.0f;
            motor->motor_data.ic_no_order = motor->motor_drv.cal_ic(motor->motor_data.current_data.i_raw.ic_raw, motor->motor_data.ic_offset_raw);
        }break;
        default:
            return 0;
    }

    calculate_order_float(motor->motor_data.ia_no_order, motor->motor_data.ib_no_order, motor->motor_data.ic_no_order, phase,
                          &motor->motor_alg.ia, &motor->motor_alg.ib, &motor->motor_alg.ic);
    return 1 ;
}

void update_ialpha_ibeta(Motor_HandleTypeDef *motor)
{
    calculate_clark(motor->motor_alg.ia , motor->motor_alg.ib , motor->motor_alg.ic,&motor->motor_alg.i_alpha, &motor->motor_alg.i_beta);
}

void update_iqid(Motor_HandleTypeDef *motor)
{
    calculate_park_sincos(motor->motor_alg.i_alpha , motor->motor_alg.i_beta , motor->motor_alg.sin_theta,motor->motor_alg.cos_theta,&motor->motor_alg.id, &motor->motor_alg.iq);
}

float get_ia(Motor_HandleTypeDef *motor)
{
    return motor->motor_alg.ia;
}

float get_ib(Motor_HandleTypeDef *motor)
{
    return motor->motor_alg.ib;
}

float get_ic(Motor_HandleTypeDef *motor)
{
    return motor->motor_alg.ic;
}

int update_i_offset_nonblock_(Motor_HandleTypeDef *motor,uint32_t this_ia_raw,uint32_t this_ib_raw,uint32_t this_ic_raw)
{
    if(motor->motor_data.calibrate_i_offset_nonblock__count < motor->motor_data.calibrate_i_offset_nonblock__sample_total)
    {
        motor->motor_data.calibrate_i_offset_nonblock__ia_offset_raw_all += this_ia_raw;
        motor->motor_data.calibrate_i_offset_nonblock__ib_offset_raw_all += this_ib_raw;
        motor->motor_data.calibrate_i_offset_nonblock__ic_offset_raw_all += this_ic_raw;
        motor->motor_data.calibrate_i_offset_nonblock__count++;
        return 0 ;
    }
    else if(motor->motor_data.calibrate_i_offset_nonblock__count >= motor->motor_data.calibrate_i_offset_nonblock__sample_total)
    {
        motor->motor_data.ia_offset_raw = motor->motor_data.calibrate_i_offset_nonblock__ia_offset_raw_all/motor->motor_data.calibrate_i_offset_nonblock__count;
        motor->motor_data.ib_offset_raw = motor->motor_data.calibrate_i_offset_nonblock__ib_offset_raw_all/motor->motor_data.calibrate_i_offset_nonblock__count;
        motor->motor_data.ic_offset_raw = motor->motor_data.calibrate_i_offset_nonblock__ic_offset_raw_all/motor->motor_data.calibrate_i_offset_nonblock__count;
        motor->motor_data.calibrate_i_offset_nonblock__ia_offset_raw_all = 0;
        motor->motor_data.calibrate_i_offset_nonblock__ib_offset_raw_all = 0;
        motor->motor_data.calibrate_i_offset_nonblock__ic_offset_raw_all = 0;
        motor->motor_data.calibrate_i_offset_nonblock__count = 0;
        return 1 ;
    }
    return 0 ;
}

int update_i_offset_nonblock(Motor_HandleTypeDef *motor,int mode_sampling)
{
    uint32_t this_ia_raw = 0U;
    uint32_t this_ib_raw = 0U;
    uint32_t this_ic_raw = 0U;

    if(motor->motor_data.ia_offset_raw!= 0 && motor->motor_data.ib_offset_raw!= 0 && motor->motor_data.ic_offset_raw!= 0)
    {
        return 1 ;
    }

    if(motor->motor_data.calibrate_i_offset_nonblock__count < motor->motor_data.calibrate_i_offset_nonblock__sample_total)
    {
        switch(mode_sampling)
        {
            case 0x111: /* ABC*/
            {
                if (!motor->motor_drv.update_ia_raw || !motor->motor_drv.update_ib_raw || !motor->motor_drv.update_ic_raw)
                {
                    return 0;
                }
                this_ia_raw = motor->motor_drv.update_ia_raw();
                this_ib_raw = motor->motor_drv.update_ib_raw();
                this_ic_raw = motor->motor_drv.update_ic_raw();
            }break;
            case 0x110: /* ABX */
            {
                if (!motor->motor_drv.update_ia_raw || !motor->motor_drv.update_ib_raw)
                {
                    return 0;
                }
                this_ia_raw = motor->motor_drv.update_ia_raw();
                this_ib_raw = motor->motor_drv.update_ib_raw();
                this_ic_raw = 0U;
            }break;
            case 0x101: /* AXC*/
            {
                if (!motor->motor_drv.update_ia_raw || !motor->motor_drv.update_ic_raw || !motor->motor_drv.cal_ia || !motor->motor_drv.cal_ic)
                {
                    return 0;
                }
                this_ia_raw = motor->motor_drv.update_ia_raw();
                this_ib_raw = 0U;
                this_ic_raw = motor->motor_drv.update_ic_raw();

            }break;
            case 0x011: /* XBC*/
            {
                if (!motor->motor_drv.update_ib_raw || !motor->motor_drv.update_ic_raw || !motor->motor_drv.cal_ib || !motor->motor_drv.cal_ic)
                {
                    return 0;
                }
                this_ia_raw = 0U;
                this_ib_raw = motor->motor_drv.update_ib_raw();
                this_ic_raw = motor->motor_drv.update_ic_raw();

            }break;
            case 0x100: /* AXX */
            {
                if (!motor->motor_drv.update_ia_raw || !motor->motor_drv.cal_ia)
                {
                    return 0;
                }
                this_ia_raw = motor->motor_drv.update_ia_raw();
                this_ib_raw = 0U;
                this_ic_raw = 0U;
            }break;
            case 0x010: /* XBX*/
            {
                if (!motor->motor_drv.update_ib_raw || !motor->motor_drv.cal_ib)
                {
                    return 0;
                }
                this_ia_raw = 0U;
                this_ib_raw = motor->motor_drv.update_ib_raw();
                this_ic_raw = 0U;

            }break;
            case 0x001: /* XXC */
            {
                if (!motor->motor_drv.update_ic_raw || !motor->motor_drv.cal_ic)
                {
                    return 0;
                }
                this_ia_raw = 0U;
                this_ib_raw = 0U;
                this_ic_raw = motor->motor_drv.update_ic_raw();
            }break;
            default:
                return 0;            
        }
    }
    return update_i_offset_nonblock_(motor,this_ia_raw,this_ib_raw,this_ic_raw);
}

void update_i_offset_block(Motor_HandleTypeDef *motor,int mode_sampling)
{
    while(!update_i_offset_nonblock(motor,mode_sampling));
}

float get_ia_offset(Motor_HandleTypeDef *motor)
{
    return motor->motor_data.ia_offset_raw;
}

float get_ib_offset(Motor_HandleTypeDef *motor)
{
    return motor->motor_data.ib_offset_raw;
}

float get_ic_offset(Motor_HandleTypeDef *motor)
{
    return motor->motor_data.ic_offset_raw;
}

void update_pole_pairs_sensor_block(Motor_HandleTypeDef *motor)
{
    float velocity_integral = 0.0f;
    set_svpwm(motor,motor->motor_config.u_max*0.05f,0.0f, 0.0f);
    motor->motor_drv.delay_ms(2000);
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
        velocity_integral += motor->motor_data.velocity_raw*motor->time.dt;

        set_svpwm(motor,motor->motor_config.u_max*0.05f,0.0f,limit_angle_el((float)i*motor->motor_data.calibrate_pole_pairs_block__angle_step));
        motor->motor_drv.delay_ms(1);
    }
    motor->motor_drv.delay_ms(2000);
    motor->motor_config.pole_pairs = (uint32_t)my_round(my_abs((float)(1000*motor->motor_data.calibrate_pole_pairs_block__angle_step)/velocity_integral));

    set_svpwm(motor,0.0f, 0.0f , 0.0f); 
}

void update_pole_pairs_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    float this_dt = update_dt(motor);
    update_angle(motor);
    update_velocity_raw(motor);
    update_pole_pairs_sensor_nonblock_(motor,this_dt,motor->motor_data.velocity_raw);
}

void update_pole_pairs_sensor_nonblock_(Motor_HandleTypeDef *motor,float this_dt,float this_velocity_raw)
{
    motor->motor_data.calibrate_pole_pairs_nonblock__total_time += this_dt;
    if(motor->motor_data.calibrate_pole_pairs_nonblock__total_time < motor->motor_data.calibrate_pole_pairs_nonblock__time_init)
    {
        motor->motor_data.calibrate_pole_pairs_nonblock__state = 0;
    }
    else if(motor->motor_data.calibrate_pole_pairs_nonblock__total_time >= motor->motor_data.calibrate_pole_pairs_nonblock__time_init && motor->motor_data.calibrate_pole_pairs_nonblock__total_time < (motor->motor_data.calibrate_pole_pairs_nonblock__time_init+motor->motor_data.calibrate_pole_pairs_nonblock__time_prep))
    {
        motor->motor_data.calibrate_pole_pairs_nonblock__state = 1;
    }
    else if(motor->motor_data.calibrate_pole_pairs_nonblock__total_time >= (motor->motor_data.calibrate_pole_pairs_nonblock__time_init+motor->motor_data.calibrate_pole_pairs_nonblock__time_prep) && motor->motor_data.calibrate_pole_pairs_nonblock__total_time < (motor->motor_data.calibrate_pole_pairs_nonblock__time_init+motor->motor_data.calibrate_pole_pairs_nonblock__time_prep+motor->motor_data.calibrate_pole_pairs_nonblock__time_process))
    {
        motor->motor_data.calibrate_pole_pairs_nonblock__state = 2;
    }
    else if(motor->motor_data.calibrate_pole_pairs_nonblock__total_time >= (motor->motor_data.calibrate_pole_pairs_nonblock__time_init+motor->motor_data.calibrate_pole_pairs_nonblock__time_prep+motor->motor_data.calibrate_pole_pairs_nonblock__time_process))
    {
        motor->motor_data.calibrate_pole_pairs_nonblock__state = 3;
    }
    
    switch (motor->motor_data.calibrate_pole_pairs_nonblock__state)
    {
        case 0:
        {
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
        }break;
        case 1:
        {
            set_svpwm(motor, motor->motor_config.u_max*0.05f,0.0f,0.0f);
        }break;
        case 2:
        {
            motor->motor_data.calibrate_pole_pairs_nonblock__velocity_integral += this_velocity_raw*this_dt;
            set_svpwm(motor,motor->motor_config.u_max*0.05f,0.0f, limit_angle_el((float)motor->motor_data.calibrate_pole_pairs_nonblock__velocity_target*(motor->motor_data.calibrate_pole_pairs_nonblock__total_time-motor->motor_data.calibrate_pole_pairs_nonblock__time_init-motor->motor_data.calibrate_pole_pairs_nonblock__time_prep)));
        }break;
        case 3:
        {
            motor->motor_config.pole_pairs = (uint32_t)my_round(my_abs((float)motor->motor_data.calibrate_pole_pairs_nonblock__velocity_target*(motor->motor_data.calibrate_pole_pairs_nonblock__total_time-motor->motor_data.calibrate_pole_pairs_nonblock__time_init-motor->motor_data.calibrate_pole_pairs_nonblock__time_prep)/(motor->motor_data.calibrate_pole_pairs_nonblock__velocity_integral)));
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            motor->motor_data.calibrate_pole_pairs_nonblock__total_time = 0.0f;
            motor->motor_data.calibrate_pole_pairs_nonblock__velocity_integral = 0.0f;
            motor->motor_data.calibrate_pole_pairs_nonblock__state = 0;
        }break;
        default:
        {
            log_motor(motor, "pole_pairs cal state invalid");
        }break;
    }
    
}

void update_2dir_sensor_block(Motor_HandleTypeDef *motor)
{
    float velocity_integral = 0.0f;
    for(int i=0 ; i<1000 ; i++)
    {
        update_dt(motor);  //预热dt，防止因初次启动产生的极小dt干扰后面的速度计算
        update_angle(motor);
        update_velocity_raw(motor);
        set_svpwm(motor,0.001f*i*motor->motor_config.u_max*0.05f, motor->motor_config.u_max*0.05f - 0.001f*i*motor->motor_config.u_max*0.05f , 0.0f);
        motor->motor_drv.delay_ms(1);
    }

    for(int i=0 ; i<2000 ; i++)
    {
        update_dt(motor);
        update_angle(motor);
        update_velocity_raw(motor);
        set_svpwm(motor,motor->motor_config.u_max*0.05f,0.0f,limit_angle_el((float)i*motor->motor_data.calibrate_2dir_block__velocity_target));
        velocity_integral += motor->motor_data.velocity_raw;

        motor->motor_drv.delay_ms(1);
    }
    motor->motor_drv.delay_ms(500);

    if(velocity_integral>0)
    {
        motor->motor_config.dir = 1;
    }
    else if(velocity_integral<0)
    {
        motor->motor_config.dir = -1;
    }
    else
    {
        log_motor(motor, "2dir sensor no motion");
    }
    set_svpwm(motor,0.0f,0.0f,0);
}

void update_2dir_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    float this_dt = update_dt(motor); //预热
    update_angle(motor);
    update_velocity_raw(motor);
    update_2dir_sensor_nonblock_(motor,this_dt,motor->motor_data.velocity_raw);
}

void update_2dir_sensor_nonblock_(Motor_HandleTypeDef *motor,float this_dt,float this_velocity_raw)
{
    motor->motor_data.calibrate_2dir_nonblock__total_time += this_dt; //预热

    if(motor->motor_data.calibrate_2dir_nonblock__total_time < motor->motor_data.calibrate_2dir_nonblock__time_init)
    {
        motor->motor_data.calibrate_2dir_nonblock__state = 0;
    }
    else if(motor->motor_data.calibrate_2dir_nonblock__total_time >= motor->motor_data.calibrate_2dir_nonblock__time_init && motor->motor_data.calibrate_2dir_nonblock__total_time < (motor->motor_data.calibrate_2dir_nonblock__time_init+motor->motor_data.calibrate_2dir_nonblock__time_prep))
    {
        motor->motor_data.calibrate_2dir_nonblock__state = 1;
    }
    else if(motor->motor_data.calibrate_2dir_nonblock__total_time >= (motor->motor_data.calibrate_2dir_nonblock__time_init+motor->motor_data.calibrate_2dir_nonblock__time_prep) && motor->motor_data.calibrate_2dir_nonblock__total_time < (motor->motor_data.calibrate_2dir_nonblock__time_init+motor->motor_data.calibrate_2dir_nonblock__time_prep+motor->motor_data.calibrate_2dir_nonblock__time_process))
    {
        motor->motor_data.calibrate_2dir_nonblock__state = 2;
    }
    else if(motor->motor_data.calibrate_2dir_nonblock__total_time >= (motor->motor_data.calibrate_2dir_nonblock__time_init+motor->motor_data.calibrate_2dir_nonblock__time_prep+motor->motor_data.calibrate_2dir_nonblock__time_process))
    {
        motor->motor_data.calibrate_2dir_nonblock__state = 3;
    }

    switch (motor->motor_data.calibrate_2dir_nonblock__state)
    {
        case 0:
        {
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
        }break;
        case 1:
        {
            float K =  (motor->motor_data.calibrate_2dir_nonblock__total_time-motor->motor_data.calibrate_2dir_nonblock__time_init)/motor->motor_data.calibrate_2dir_nonblock__time_process;
            set_svpwm(motor,0.0f,K*motor->motor_config.u_max*0.05f, 0.0f);
        }break;
        case 2:
        {
            set_svpwm(motor,motor->motor_config.u_max*0.05f,3.0f, limit_angle_el((float)(motor->motor_data.calibrate_2dir_nonblock__total_time-motor->motor_data.calibrate_2dir_nonblock__time_init-motor->motor_data.calibrate_2dir_nonblock__time_prep)*motor->motor_data.calibrate_2dir_nonblock__velocity_target));
            motor->motor_data.calibrate_2dir_nonblock__velocity_integral += this_velocity_raw;
        }break;
        case 3:
        {
            if(motor->motor_data.calibrate_2dir_nonblock__velocity_integral>0)
            {
                motor->motor_config.dir = 1;
            }
            else if(motor->motor_data.calibrate_2dir_nonblock__velocity_integral<0)
            {
                motor->motor_config.dir = -1;
            }
            else
            {
                log_motor(motor, "2dir sensor no motion");
            }
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            motor->motor_data.calibrate_2dir_nonblock__total_time = 0.0f;
            motor->motor_data.calibrate_2dir_nonblock__velocity_integral = 0.0f;
        }break;
        default:
        {
            log_motor(motor, "2dir cal state invalid");
        }break;
    }
}

void calculate_phase(float ia, float ib, float ic,float ua, float ub, float uc, int *phase_a, int *phase_b, int *phase_c)//同时兼容直接向三相注入IqId时的工况，也就是在电机运行的情况下进行相序辨识
{
    int phase[3] = {0,0,0};
    float U[3] = {ua, ub, uc};
    float I[3] = {ia, ib, ic};

    for(int i = 0 ; i<3 ; i++)
    {
        for(int j = 0 ; j<3 ; j++)
        {
            if( U[i]/I[j] > 0 )
            {
                phase[i] = j+1; //相序标识最小也为1
            }
        }
    }

    *phase_a = phase[0];
    *phase_b = phase[1];
    *phase_c = phase[2];
}
int update_phase_nonblock(Motor_HandleTypeDef *motor,float ia_no_order, float ib_no_order, float ic_no_order)
{
    float this_dt = update_dt(motor);
    return update_phase_nonblock_(motor,ia_no_order,ib_no_order,ic_no_order,this_dt);
}

int update_phase_nonblock_(Motor_HandleTypeDef *motor,float ia_no_order, float ib_no_order, float ic_no_order,float this_dt)
{
    motor->motor_data.calibrate_phase_nonblock__time += this_dt;
    if(motor->motor_data.calibrate_phase_nonblock__time>(float)motor->motor_data.calibrate_phase_nonblock__state * motor->motor_data.calibrate_phase_nonblock__ts)
    {
        motor->motor_data.calibrate_phase_nonblock__state ++ ;
    }

    switch(motor->motor_data.calibrate_phase_nonblock__state)
    {
        case 1 :
        {
            motor->motor_config.phase = -1; //先将相序设置为-1，表示正在检测相序
            set_pwm(motor,0.0f,0.0f,0.0f);
        }break;
        case 2 :
        {
            motor->motor_data.calibrate_phase_nonblock__ia_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__ib_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__ic_integral = 0.0f;
            set_pwm(motor,motor->motor_data.calibrate_phase_nonblock__duty,0.0f,0.0f);
        }break;
        case 3 :
        {
            motor->motor_data.calibrate_phase_nonblock__ia_integral += ia_no_order * this_dt;
            motor->motor_data.calibrate_phase_nonblock__ib_integral += ib_no_order * this_dt;
            motor->motor_data.calibrate_phase_nonblock__ic_integral += ic_no_order * this_dt;
            int phase_a, phase_b, phase_c;
            calculate_phase(motor->motor_data.calibrate_phase_nonblock__ia_integral , motor->motor_data.calibrate_phase_nonblock__ib_integral , motor->motor_data.calibrate_phase_nonblock__ic_integral , motor->motor_config.u_max/2*motor->motor_data.calibrate_phase_nonblock__duty , 0.0f , 0.0f, &phase_a, &phase_b, &phase_c);
            motor->motor_data.calibrate_phase_nonblock__phase_a = phase_a;
        }break;
        case 4:
        {
            motor->motor_data.calibrate_phase_nonblock__ia_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__ib_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__ic_integral = 0.0f;
            set_pwm(motor,0.0f,motor->motor_data.calibrate_phase_nonblock__duty,0.0f);
        }break;
        case 5:
        {
            motor->motor_data.calibrate_phase_nonblock__ia_integral += ia_no_order * this_dt;
            motor->motor_data.calibrate_phase_nonblock__ib_integral += ib_no_order * this_dt;
            motor->motor_data.calibrate_phase_nonblock__ic_integral += ic_no_order * this_dt;
            int phase_a, phase_b, phase_c;
            calculate_phase(motor->motor_data.calibrate_phase_nonblock__ia_integral , motor->motor_data.calibrate_phase_nonblock__ib_integral , motor->motor_data.calibrate_phase_nonblock__ic_integral , 0.0f , motor->motor_config.u_max/2*motor->motor_data.calibrate_phase_nonblock__duty , 0.0f, &phase_a, &phase_b, &phase_c);
            motor->motor_data.calibrate_phase_nonblock__phase_b = phase_b;            
        }break;
        case 6:
        {
            motor->motor_data.calibrate_phase_nonblock__ia_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__ib_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__ic_integral = 0.0f;
            set_pwm(motor,0.0f,0.0f,motor->motor_data.calibrate_phase_nonblock__duty);
        }break;
        case 7:
        {
            motor->motor_data.calibrate_phase_nonblock__ia_integral += ia_no_order * this_dt;
            motor->motor_data.calibrate_phase_nonblock__ib_integral += ib_no_order * this_dt;
            motor->motor_data.calibrate_phase_nonblock__ic_integral += ic_no_order * this_dt;
            int phase_a, phase_b, phase_c;
            calculate_phase(motor->motor_data.calibrate_phase_nonblock__ia_integral , motor->motor_data.calibrate_phase_nonblock__ib_integral , motor->motor_data.calibrate_phase_nonblock__ic_integral , 0.0f , 0.0f , motor->motor_config.u_max/2*motor->motor_data.calibrate_phase_nonblock__duty, &phase_a, &phase_b, &phase_c);
            motor->motor_data.calibrate_phase_nonblock__phase_c = phase_c;   
        }break;
        case 8:
        {
            if( motor->motor_data.calibrate_phase_nonblock__phase_a == 1 && motor->motor_data.calibrate_phase_nonblock__phase_b == 2 && motor->motor_data.calibrate_phase_nonblock__phase_c == 3 )
            {
                motor->motor_config.phase = 1;
            }
            else if ( motor->motor_data.calibrate_phase_nonblock__phase_a == 2 && motor->motor_data.calibrate_phase_nonblock__phase_b == 1 && motor->motor_data.calibrate_phase_nonblock__phase_c == 3 )
            {
                motor->motor_config.phase = 2;
            }
            else if ( motor->motor_data.calibrate_phase_nonblock__phase_a == 3 && motor->motor_data.calibrate_phase_nonblock__phase_b == 2 && motor->motor_data.calibrate_phase_nonblock__phase_c == 1 )
            {
                motor->motor_config.phase = 3;
            }
            else if ( motor->motor_data.calibrate_phase_nonblock__phase_a == 3 && motor->motor_data.calibrate_phase_nonblock__phase_b == 1 && motor->motor_data.calibrate_phase_nonblock__phase_c == 2 )
            {
                motor->motor_config.phase = 4;
            }
            else if ( motor->motor_data.calibrate_phase_nonblock__phase_a == 2 && motor->motor_data.calibrate_phase_nonblock__phase_b == 3 && motor->motor_data.calibrate_phase_nonblock__phase_c == 1 )
            {
                motor->motor_config.phase = 5;
            }
            else if ( motor->motor_data.calibrate_phase_nonblock__phase_a == 1 && motor->motor_data.calibrate_phase_nonblock__phase_b == 3 && motor->motor_data.calibrate_phase_nonblock__phase_c == 2 )
            {
                motor->motor_config.phase = 6;
            }
            else
            {
                motor->motor_config.phase = -1;
            }
        }break;
        case 9:
        {
            set_pwm(motor,0.0f,0.0f,0.0f);
            motor->motor_data.calibrate_phase_nonblock__ia_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__ib_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__ic_integral = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__time = 0.0f;
            motor->motor_data.calibrate_phase_nonblock__state = 1 ;
            return motor->motor_config.phase ;
        }break;
    }
    return 0 ;
}

int update_phase_block(Motor_HandleTypeDef *motor)
{
    update_ia_ib_ic(motor,0x111,1);
    while(!update_phase_nonblock(motor,motor->motor_data.ia_no_order, motor->motor_data.ib_no_order, motor->motor_data.ic_no_order));
    if(motor->motor_config.phase == -1)
    {
        log_motor(motor, "phase identify failed");
    }
    return motor->motor_config.phase ;
}

void update_angle_el_zero_no_sensor_block(Motor_HandleTypeDef *motor)
{
    motor->motor_drv.delay_ms(1000);
    set_svpwm(motor,0.0f, motor->motor_config.u_max*0.05f , 0.0f);
    motor->motor_drv.delay_ms(1000);
    update_angle(motor);
    motor->motor_config.angle_el_zero = -calculate_angle_el(motor->motor_config.pole_pairs,motor->motor_alg.angle, 0.0f);
    motor->motor_drv.delay_ms(1000);
    set_svpwm(motor,0.0f, 0.0f , 0.0f);
    motor->motor_drv.delay_ms(1000);
}

void update_angle_el_zero_sensor_block(Motor_HandleTypeDef *motor)
{
    float angle_el_zero_sum = 0.0f;
    uint32_t angle_el_zero_n = 0;
    int32_t loopcount_temp = motor->motor_data.loopcount;
    float angle_last_temp = motor->motor_alg.last_angle;
    float angle_temp = motor->motor_alg.angle;

    reset_data_angle(motor);

    set_svpwm(motor,0.0f, motor->motor_config.u_max*0.05f , 0.0f);
    motor->motor_drv.delay_ms(1000);
    int running;
    do
    {
        update_dt(motor);
        update_angle(motor);
        running = ctrl_motor_openloop_angle_nonblock(motor,motor->time.dt,2*PI,0.0f,0.6,0.0f,motor->motor_config.u_max*0.05f);
        if(!running)
        {
            break;
        }
        float angle_now = motor->motor_data.openloop__progress/(float)motor->motor_config.pole_pairs;
        angle_el_zero_sum += angle_now - get_angle_all(motor);
        angle_el_zero_n++;
    } while (running);
    ctrl_motor_openloop_reset(motor);
    motor->motor_drv.delay_ms(500);
    do
    {
        update_dt(motor);
        update_angle(motor);
        running = ctrl_motor_openloop_angle_nonblock(motor,motor->time.dt,0.0f,2*PI,-0.6,0.0f,motor->motor_config.u_max*0.05f);
        if(!running)
        {
            break;
        }
        float angle_now = 2*PI + motor->motor_data.openloop__progress/(float)motor->motor_config.pole_pairs;
        angle_el_zero_sum += angle_now - get_angle_all(motor);
        angle_el_zero_n++;
    } while (running);
    ctrl_motor_openloop_reset(motor);
    if(angle_el_zero_n > 0)
    {
        motor->motor_config.angle_el_zero = calculate_angle_el(motor->motor_config.pole_pairs,angle_el_zero_sum/(float)angle_el_zero_n, 0.0f);
    }

    motor->motor_data.loopcount = loopcount_temp;
    motor->motor_alg.last_angle = angle_last_temp;
    motor->motor_alg.angle = angle_temp;

    set_svpwm(motor,0.0f, 0.0f , 0.0f);
}

void update_angle_el_zero_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    float this_dt = update_dt(motor); //预热
    update_angle(motor);
    update_angle_el_zero_sensor_nonblock_(motor,this_dt,get_angle_all(motor));
}

void update_angle_el_zero_sensor_nonblock_(Motor_HandleTypeDef *motor,float this_dt,float this_angle_all)
{
    switch(motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__state)
    {
        case 1:
        {
            reset_data_angle(motor);
            motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__sum = 0.0f;
            motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__count = 0;
            motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__state = 2;
            set_svpwm(motor,motor->motor_config.u_max*0.05f, 0.0f , 0.0f);
        }break;
        case 2:
        {
            int running = ctrl_motor_openloop_angle_nonblock(motor,this_dt,2*PI,0.0f,0.3,motor->motor_config.u_max*0.05f, 0.0f);
            if(!running)
            {
                ctrl_motor_openloop_reset(motor);
                motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__state = 3;
                break;
            }
            float angle_now = motor->motor_data.openloop__progress/(float)motor->motor_config.pole_pairs;
            motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__sum += angle_now - this_angle_all;
            motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__count++;
        }break;
        case 3:
        {
            int running = ctrl_motor_openloop_angle_nonblock(motor,this_dt,0.0f,2*PI,-0.3,motor->motor_config.u_max*0.05f, 0.0f);
            if(!running)
            {
                ctrl_motor_openloop_reset(motor);
                motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__state = 4;
                break;
            }
            float angle_now = 2*PI + motor->motor_data.openloop__progress/(float)motor->motor_config.pole_pairs;
            motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__sum += angle_now - this_angle_all;
            motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__count++;
        }break;
        case 4:
        {
            if(motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__count > 0)
            {
                motor->motor_config.angle_el_zero = motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__sum/(float)motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__count;
            }
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            motor->motor_data.calibrate_angle_el_zero_sensor_nonblock__state = 1;
        }break;
        default:
        {
            log_motor(motor, "angle_el_zero cal state invalid");
        }break;
    }
}

void ctrl_motor_openloop_reset(Motor_HandleTypeDef *motor)
{
    motor->motor_data.openloop__angle_el = 0.0f;
    motor->motor_data.openloop__progress = 0.0f;
    motor->motor_data.openloop__state = 0;
}

void ctrl_motor_openloop_velocity_el_nonblock(Motor_HandleTypeDef *motor,float this_dt,float velocity_el_target,float uq,float ud)
{
    motor->motor_data.openloop__angle_el = limit_angle_el(motor->motor_data.openloop__angle_el + velocity_el_target*this_dt);
    set_svpwm_dir(motor,uq,ud,motor->motor_data.openloop__angle_el);
}

void ctrl_motor_openloop_velocity_nonblock(Motor_HandleTypeDef *motor,float this_dt,float velocity_target,float uq,float ud)
{
    ctrl_motor_openloop_velocity_el_nonblock(motor,this_dt,velocity_target*(float)motor->motor_config.pole_pairs,uq,ud);
}

int ctrl_motor_openloop_angle_el_nonblock(Motor_HandleTypeDef *motor,float this_dt,float angle_el_target,float angle_el_start,float velocity_el_target ,float uq,float ud)
{
    if(motor->motor_data.openloop__state == 0)
    {
        motor->motor_data.openloop__state = 1;
        motor->motor_data.openloop__progress = 0.0f;
        motor->motor_data.openloop__angle_el = limit_angle_el(angle_el_start);
    }
    if(motor->motor_data.openloop__state == 2)
    {
        set_svpwm_dir(motor,uq,ud,motor->motor_data.openloop__angle_el);
        return 0;
    }

    motor->motor_data.openloop__progress += velocity_el_target*this_dt;
    if(my_abs(motor->motor_data.openloop__progress) < my_abs(angle_el_target - angle_el_start))
    {
        motor->motor_data.openloop__angle_el = limit_angle_el(angle_el_start + motor->motor_data.openloop__progress);
        set_svpwm_dir(motor,uq,ud,motor->motor_data.openloop__angle_el);
        return 1;
    }

    motor->motor_data.openloop__progress = angle_el_target - angle_el_start;
    motor->motor_data.openloop__angle_el = limit_angle_el(angle_el_target);
    motor->motor_data.openloop__state = 2;
    set_svpwm_dir(motor,uq,ud,motor->motor_data.openloop__angle_el);
    return 0;
}

int ctrl_motor_openloop_angle_el_block(Motor_HandleTypeDef *motor,float angle_el_target,float angle_el_start,float velocity_el_target ,float uq,float ud)
{
    int running;
    while((running = ctrl_motor_openloop_angle_el_nonblock(motor,motor->time.dt,angle_el_target,angle_el_start,velocity_el_target,uq,ud)))
    {
        update_dt(motor);
    }
    return running;
}

int ctrl_motor_openloop_angle_nonblock(Motor_HandleTypeDef *motor,float this_dt,float angle_target,float angle_start,float velocity_target ,float uq,float ud)
{
    float pole_pairs = (float)motor->motor_config.pole_pairs;
    return ctrl_motor_openloop_angle_el_nonblock(motor,this_dt,angle_target*pole_pairs,angle_start*pole_pairs,velocity_target*pole_pairs,uq,ud);
}

int ctrl_motor_openloop_angle_block(Motor_HandleTypeDef *motor,float angle_target,float angle_start,float velocity_target ,float uq,float ud)
{
    int running;
    while((running = ctrl_motor_openloop_angle_nonblock(motor,motor->time.dt,angle_target,angle_start,velocity_target,uq,ud)))
    {
        update_dt(motor);
        update_angle(motor);
    }
    return running;
}

