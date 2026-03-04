#ifndef RLS_H_
#define RLS_H_

#include "main.h"
#include "arm_math.h"
#include "foc_alg.h"

#define RLS_PARAM_NUM        2U

// typedef struct
// {
//     uint16_t numRows;     
//     uint16_t numCols;     
//     float32_t *pData;     
// }martix_HandleTypeDef;

// typedef struct
// {
//     martix_HandleTypeDef P;
//     martix_HandleTypeDef K;
//     martix_HandleTypeDef theta;
//     martix_HandleTypeDef phi;

//     martix_HandleTypeDef *(*martix_cal_mult)(martix_HandleTypeDef *A, martix_HandleTypeDef *B); // 矩阵乘法函数指针
//     martix_HandleTypeDef *(*martix_cal_sub)(martix_HandleTypeDef *A, martix_HandleTypeDef *B); // 矩阵相减函数指针
//     martix_HandleTypeDef *(*martix_cal_trans)(martix_HandleTypeDef *A); // 矩阵转置函数指针
//     martix_HandleTypeDef *(*martix_cal_scale)(martix_HandleTypeDef *A, float32_t scale); // 矩阵缩放函数指针(点乘)

// }math_HandleTypeDef;

typedef struct
{
    float32_t P[RLS_PARAM_NUM][RLS_PARAM_NUM]; // 协方差矩阵 P (N×N)
    float32_t K[RLS_PARAM_NUM];          // 修正增益数组 K (N×1)
    
    float32_t theta[RLS_PARAM_NUM];      // 待辨识参数数组 [R, L, ...]
    float32_t phi[RLS_PARAM_NUM];        // 回归向量 φ [i, di/dt, ...]
    float32_t y;                         // 实际输出值（如电机端电压u）
    float32_t lambda;                    // 遗忘因子 λ (0 < λ ≤ 1，值越小遗忘越快)

    arm_matrix_instance_f32 P_mat;      // 协方差矩阵 P 的 arm_matrix_instance_f32 结构体
    arm_matrix_instance_f32 K_mat;      // 修正增益数组 K 的 arm_matrix_instance_f32 结构体
    arm_matrix_instance_f32 theta_mat;  // 待辨识参数数组 theta 的 arm_matrix_instance_f32 结构体
    arm_matrix_instance_f32 phi_mat;    // 回归向量 phi 的 arm_matrix_instance_f32 结构体

    // math_HandleTypeDef math;             // 包含矩阵运算函数指针的结构体
} RLS_HandleTypeDef;

void RLS_Init(RLS_HandleTypeDef *RLS, float32_t lambda);
void RLS_update_theta(RLS_HandleTypeDef *RLS, float32_t y, float32_t *phi);
void RLS_update(RLS_HandleTypeDef *RLS, Motor_HandleTypeDef *motor);

#endif // !RLS_H_