#ifndef SMO_H_
#define SMO_H_

#include "main.h"
#include "arm_math.h"
#include "foc_alg.h"
#include "ADRC.h"

// 宏定义：符号函数（工程常用，也可替换为tanh减小抖振）
#define SIGN(x) ((x) >= 0.0f ? 1.0f : -1.0f)

// 饱和函数宏：x为输入，threshold为饱和阈值
// 效果：|x| ≤ threshold时，输出x/threshold（线性段）；|x| > threshold时，输出±1（饱和段）
#define SAT(x, threshold) \
    ((threshold) <= 0.0f ? SIGN(x) : \
    ((x) > (threshold)) ? 1.0f : \
    ((x) < -(threshold)) ? -1.0f : \
    ((x) / (threshold)))

// 宏定义：角度归一化（限制在[0, 2π]，适配PLL逻辑）
#define NORMALIZE_ANGLE(angle) \
    do { \
        while (angle > (float)(2*PI)) angle -= 2.0f * (float)PI; \
        while (angle < 0.0f) angle += 2.0f * (float)PI; \
    } while (0)

typedef struct 
{
    /********************* 1. 电机参数（需根据实际电机配置） *********************/
    float Rs;               // 定子电阻 (Ω)
    float Ls;               // 定子电感 (H)
    float PolePairs;        // 电机极对数

    /********************* 2. 滑模观测器核心配置参数 *********************/
    float SmoGain;          // 滑模增益 K
    float Ts;               // 控制周期（采样周期） (s)

    /********************* 3. 输入变量（来自FOC主程序） *********************/
    float Ualpha;           // α轴电压输入 (V)
    float Ubeta;            // β轴电压输入 (V)
    float Ialpha;           // α轴电流反馈 (A)
    float Ibeta;            // β轴电流反馈 (A)

    /********************* 4. 中间状态变量（观测器内部计算） *********************/
    float IalphaHat;        // α轴观测电流 (A)
    float IbetaHat;         // β轴观测电流 (A)
    float EalphaHat;        // α轴观测反电动势 (V)
    float EbetaHat;         // β轴观测反电动势 (V)
    float Salpha;           // α轴滑模面（电流误差）
    float Sbeta;            // β轴滑模面（电流误差）

    /********************* 5. 输出变量（给FOC用） *********************/
    float ThetaEst;         // 观测到的电角度 (rad)
    float OmegaEst;         // 观测到的电角速度 (rad/s)

    /********************* 6. PLL锁相环参数（核心新增） *********************/
    float PlKp;             // PLL比例增益
    float PlKi;             // PLL积分增益
    float PlIntegral;       // PLL积分项缓存
    float PlErr;            // PLL相位误差
    float SinTheta;         // 观测角度的正弦值
    float CosTheta;         // 观测角度的余弦值

    /********************* 7. 内部缓存变量 *********************/
    float LastTheta;        // 上一时刻电角度

    float angle_el_output;
}SMO_HandleTypeDef;

void SMO_Init(SMO_HandleTypeDef *hsmo);
float update_SMO(SMO_HandleTypeDef *hsmo,float T,float Ualpha,float Ubeta,float Ialpha,float Ibeta);
float update_angle_SMO(SMO_HandleTypeDef *hsmo,Motor_HandleTypeDef *motor,float Ualpha,float Ubeta,float Ialpha,float Ibeta);
#endif