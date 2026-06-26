#include "main.h"

/*
使用示例：
#include "dual_encoder.h"
DualEncoder_HandleTypeDef dual_encoder;
int main(void)
{
    dual_encoder_init(&dual_encoder);   //在初始化代码内填写编码器参数，如零点、齿轮比等

    while (1)
    {
        float flange_angle = dual_encoder_update_angle_flange(&dual_encoder);
    }
} 
*/

#define DualEncoder_PI 3.14159265359f

typedef struct 
{
    float angle_zero_gear_A;
    float angle_zero_gear_B;

    float angle_A;
    float angle_B;

    float GT_A;
    float GT_B;
    float GR;

    float loopcount_rotor;
    float angle_flange; 
    float angle_A_all;

    float (*update_encoder_A)(void); // 更新编码器主轴角度的函数
    float (*update_encoder_B)(void); // 更新编码器从轴角度的函数
    void (*delay_ms)(uint16_t ms);  // 毫秒延时
}DualEncoder_HandleTypeDef;

void  dual_encoder_init(DualEncoder_HandleTypeDef *dual_encoder_ptr);
float dual_encoder_limit_angle_rotor(float angle);
float dual_encoder_map_angle(float angle, float Low, float High);
float dual_encoder_limit_angle_flange(float angle_all,float GR);
float dual_encoder_update_angle_flange(DualEncoder_HandleTypeDef *dual_encoder_ptr);


