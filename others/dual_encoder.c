#include <math.h>
#include "dual_encoder.h"

float stm32_update_encoder_A(void)
{
    // 在这里添加实际的编码器A读取逻辑
    // 例如，从硬件接口获取编码器A的角度值(0--2*PI)
    float angle_A = 0.0f; // 替换为实际读取值
    return angle_A;
}

float stm32_update_encoder_B(void)
{
    // 在这里添加实际的编码器B读取逻辑
    // 例如，从硬件接口获取编码器B的角度值(0--2*PI)
    float angle_B = 0.0f; // 替换为实际读取值
    return angle_B;
}

void stm32_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void dual_encoder_init(DualEncoder_HandleTypeDef *dual_encoder_ptr)
{
    dual_encoder_ptr->angle_zero_gear_A = 0.0f;
    dual_encoder_ptr->angle_zero_gear_B = 0.0f;

    dual_encoder_ptr->angle_A = 0.0f;
    dual_encoder_ptr->angle_B = 0.0f;

    dual_encoder_ptr->GT_A = 0.0f;
    dual_encoder_ptr->GT_B = 0.0f;

    dual_encoder_ptr->loopcount_rotor = 0.0f;
    dual_encoder_ptr->angle_flange = 0.0f;
    dual_encoder_ptr->update_encoder_A = stm32_update_encoder_A;
    dual_encoder_ptr->update_encoder_B = stm32_update_encoder_B;
    dual_encoder_ptr->delay_ms = stm32_delay_ms;
}

float dual_encoder_limit_angle_rotor(float angle)
{
    // 先处理正角度，减去 2*PI 的整数倍
    while (angle >= 2 * DualEncoder_PI)
    {
        angle -= 2 * DualEncoder_PI;
    }
    // 再处理负角度，加上 2*PI 的整数倍
    while (angle < 0)
    {
        angle += 2 * DualEncoder_PI;
    }
    return angle;
}

float dual_encoder_map_angle(float angle, float Low, float High)
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

float dual_encoder_limit_angle_flange(float angle_all,float GR)
{
    float flange_angle = angle_all / GR;
    flange_angle = dual_encoder_map_angle(flange_angle,-DualEncoder_PI,DualEncoder_PI);
    return flange_angle;
}

float dual_encoder_update_angle_flange(DualEncoder_HandleTypeDef *dual_encoder_ptr)
{
    static float angle_B_;

    for(int count =0 ; count<30 ; count++)//尝试多次读取以确保稳定
    {
        dual_encoder_ptr->angle_A = dual_encoder_limit_angle_rotor(dual_encoder_ptr->update_encoder_A());
        dual_encoder_ptr->angle_B = dual_encoder_limit_angle_rotor(dual_encoder_ptr->update_encoder_B());
        for(int i = -(dual_encoder_ptr->GT_B/2 - 1) ; i<(dual_encoder_ptr->GT_B/2) ;i++) //溢出圈数为GT_B , 取中间值为0圈
        {
            angle_B_ = dual_encoder_limit_angle_rotor( fmodf( ((dual_encoder_ptr->angle_A+(float)i*2*DualEncoder_PI)*(dual_encoder_ptr->GT_A/dual_encoder_ptr->GT_B)) , (2*DualEncoder_PI) ) ) ;
            if(fabs(angle_B_-dual_encoder_ptr->angle_B)<0.05)
            {
                dual_encoder_ptr->loopcount_rotor = i ;
                break;
            }
        }
        dual_encoder_ptr->angle_A_all = (dual_encoder_ptr->loopcount_rotor * 2 * DualEncoder_PI + dual_encoder_limit_angle_rotor(dual_encoder_ptr->angle_A - dual_encoder_ptr->angle_zero_gear_A) );
        dual_encoder_ptr->angle_flange = dual_encoder_limit_angle_flange(dual_encoder_ptr->angle_A_all,dual_encoder_ptr->GR);
        dual_encoder_ptr->delay_ms(1);
    } 
    return dual_encoder_ptr->angle_flange;
}
