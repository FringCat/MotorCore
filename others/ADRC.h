#ifndef ADRC_H__
#define ADRC_H__

#include "main.h"
#include "arm_math.h"
typedef struct 
{
    float r;      // 速度因子 (跟踪速度，电机控制常用10~100)
    float h;      // 滤波因子/采样周期 (与系统采样周期一致，如1ms=0.001f)
    float R;      // 整体增益 (通常取1.0f，强化跟踪时取2~5)

    float x1;     // TD输出：跟踪指令
    float x2;     // TD输出：跟踪指令的微分
}TD_HandleTypeDef;

typedef struct 
{
    float beta01; // 状态x1观测增益 (主导收敛速度，常用10~100)
    float beta02; // 状态x2观测增益 (通常=beta01^2)
    float beta03; // 总扰动x3观测增益 (通常=beta01^3)
    float b0;     // 标称控制增益 (电机控制：速度环b0=1/J，J为转动惯量)
    float h;      // 采样周期

    float alpha_1,delta_1;
    float alpha_2,delta_2;
    float alpha_3,delta_3;

    float z1;     // ESO输出：观测的电机输出（位置/速度）
    float z2;     // ESO输出：观测的电机输出微分（速度/加速度）
    float z3;     // ESO输出：观测的总扰动

}ESO_HandleTypeDef;

typedef struct 
{
    float beta1;  // e1误差反馈增益 (位置/速度误差，常用5~50)
    float beta2;  // e2误差反馈增益 (微分误差，常用1~10)

    float alpha_1,delta_1;
    float alpha_2,delta_2;

    float e1;     // 状态误差1：TD.x1 - ESO.z1
    float e2;     // 状态误差2：TD.x2 - ESO.z2
    float u0;     // 基础控制量（非线性反馈输出）
    float u;      // 最终控制量（补偿扰动后，限幅前）
}NLSEF_HandleTypeDef;

typedef struct 
{
    int ID; 
    TD_HandleTypeDef TD;
    ESO_HandleTypeDef ESO;
    NLSEF_HandleTypeDef NLSEF;
}ADRC_HandleTypeDef;

void ADRC_Init(ADRC_HandleTypeDef *adrc, int id, float h, float b0);
float update_ADRC(ADRC_HandleTypeDef *adrc,float target ,float now);

float fhan(float x1, float x2, float r, float h);
float fal(float e, float alpha, float delta);

void update_ADRC_TD(TD_HandleTypeDef *td, float v_ref);
void update_ADRC_ESO(ESO_HandleTypeDef *eso, float y_meas, float u);
void update_ADRC_NLSEF(NLSEF_HandleTypeDef *nlsef,TD_HandleTypeDef *td, ESO_HandleTypeDef *eso,float b0);

#endif // !ADRC_H__
