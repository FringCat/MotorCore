#include "RLS.h"

/**
 * @brief  RLS算法初始化函数
 * @param  RLS: 指向RLS句柄的指针
 * @param  lambda: 遗忘因子（建议取值0.95~0.995）
 * @retval 无
 * @note   1. 初始化协方差矩阵P为对角阵（初始值1000），代表初始参数不确定性高
 *         2. 初始化待辨识参数theta为0（可根据需求修改初始值）
 *         3. 初始化arm_math矩阵结构体，绑定数据缓冲区
 */
void RLS_Init(RLS_HandleTypeDef *RLS, float32_t lambda)
{
    if (RLS == NULL)
    {
        return;
    }

    if (lambda <= 0.0f || lambda > 1.0f)
    {
        RLS->lambda = 0.98f;  // 默认值
    }
    else
    {
        RLS->lambda = lambda;
    }

    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        for (uint8_t j = 0; j < RLS_PARAM_NUM; j++)
        {
            if (i == j)
            {
                RLS->P[i][j] = 1000.0f;  // 对角线元素为1000
            }
            else
            {
                RLS->P[i][j] = 0.0f;     // 非对角线元素为0
            }
        }
    }

    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        RLS->K[i] = 0.0f;
        RLS->theta[i] = 0.0f;
        RLS->phi[i] = 0.0f;
    }
    RLS->y = 0.0f;

    arm_mat_init_f32(&RLS->P_mat, RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)RLS->P);
    arm_mat_init_f32(&RLS->K_mat, RLS_PARAM_NUM, 1, RLS->K);
    arm_mat_init_f32(&RLS->theta_mat, RLS_PARAM_NUM, 1, RLS->theta);
    arm_mat_init_f32(&RLS->phi_mat, RLS_PARAM_NUM, 1, RLS->phi);
}

/**
 * @brief  RLS算法单次递推更新函数（核心）
 * @param  RLS: 指向RLS句柄的指针
 * @param  y: 当前实际输出值（如电机端电压u）
 * @param  phi: 回归向量数组指针（如[i, di/dt]）
 * @retval 无
 * @note   1. 严格按照RLS递推公式实现，基于arm_math矩阵函数
 *         2. 包含除0保护、维度检查，保证运行稳定性
 *         3. 局部缓冲区静态分配，避免栈溢出
 */
void RLS_update_theta(RLS_HandleTypeDef *RLS, float32_t y, float32_t *phi)
{
    // 局部静态缓冲区（避免栈溢出，复用内存）
    static float32_t P_phi_buf[RLS_PARAM_NUM];        // P×φ的结果缓冲区 (N×1)
    static float32_t phi_T_buf[RLS_PARAM_NUM];        // φ转置的缓冲区 (1×N)
    static float32_t phi_T_P_phi_buf[1];              // φᵀ×P×φ的结果缓冲区 (1×1)
    static float32_t K_phi_T_buf[RLS_PARAM_NUM][RLS_PARAM_NUM]; // K×φᵀ的缓冲区 (N×N)
    static float32_t I_K_phi_T_buf[RLS_PARAM_NUM][RLS_PARAM_NUM]; // I - K×φᵀ的缓冲区 (N×N)
    static float32_t I_mat_buf[RLS_PARAM_NUM][RLS_PARAM_NUM];     // 单位矩阵缓冲区 (N×N)

    // 局部矩阵结构体（复用，避免重复初始化）
    static arm_matrix_instance_f32 P_phi_mat;
    static arm_matrix_instance_f32 phi_T_mat;
    static arm_matrix_instance_f32 phi_T_P_phi_mat;
    static arm_matrix_instance_f32 K_phi_T_mat;
    static arm_matrix_instance_f32 I_K_phi_T_mat;
    static arm_matrix_instance_f32 I_mat;
    static arm_matrix_instance_f32 temp_mat;          // 临时矩阵缓冲区 (N×N)
    static float32_t temp_buf[RLS_PARAM_NUM][RLS_PARAM_NUM]; // 临时数据缓冲区

    // 1. 安全检查
    if (RLS == NULL || phi == NULL)
    {
        return;
    }

    // 2. 更新输入输出缓存
    RLS->y = y;
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        RLS->phi[i] = phi[i];
    }

    // 3. 初始化局部矩阵结构体（仅首次调用初始化，后续复用）
    static uint8_t first_init = 0;
    if (first_init == 0)
    {
        // 初始化单位矩阵I
        for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
        {
            for (uint8_t j = 0; j < RLS_PARAM_NUM; j++)
            {
                I_mat_buf[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }

        // 初始化局部矩阵结构体
        arm_mat_init_f32(&P_phi_mat, RLS_PARAM_NUM, 1, P_phi_buf);
        arm_mat_init_f32(&phi_T_mat, 1, RLS_PARAM_NUM, phi_T_buf);
        arm_mat_init_f32(&phi_T_P_phi_mat, 1, 1, phi_T_P_phi_buf);
        arm_mat_init_f32(&K_phi_T_mat, RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)K_phi_T_buf);
        arm_mat_init_f32(&I_K_phi_T_mat, RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)I_K_phi_T_buf);
        arm_mat_init_f32(&I_mat, RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)I_mat_buf);
        arm_mat_init_f32(&temp_mat, RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)temp_buf);

        first_init = 1;
    }

    // -------------------------- 步骤1：计算 P×φ --------------------------
    if (arm_mat_mult_f32(&RLS->P_mat, &RLS->phi_mat, &P_phi_mat) != ARM_MATH_SUCCESS)
    {
        return; // 维度不匹配，直接退出
    }

    // -------------------------- 步骤2：计算 φᵀ×P×φ --------------------------
    // 2.1 转置φ得到φᵀ
    arm_mat_trans_f32(&RLS->phi_mat, &phi_T_mat);

    // 2.2 计算 φᵀ×P
    arm_mat_mult_f32(&phi_T_mat, &RLS->P_mat, &temp_mat);

    // 2.3 计算 (φᵀ×P)×φ = φᵀ×P×φ
    arm_mat_mult_f32(&temp_mat, &RLS->phi_mat, &phi_T_P_phi_mat);

    // -------------------------- 步骤3：计算修正增益 K --------------------------
    // 3.1 计算分母：λ + φᵀ×P×φ
    float32_t denominator = RLS->lambda + phi_T_P_phi_buf[0];

    // 3.2 除0保护
    if (fabs(denominator) < 1e-6f)
    {
        denominator = 1e-6f;
    }

    // 3.3 K = (P×φ) / denominator → 标量缩放
    arm_mat_scale_f32(&P_phi_mat, 1.0f / denominator, &RLS->K_mat);

    // -------------------------- 步骤4：计算 I - K×φᵀ --------------------------
    // 4.1 计算 K×φᵀ
    arm_mat_mult_f32(&RLS->K_mat, &phi_T_mat, &K_phi_T_mat);

    // 4.2 计算 I - K×φᵀ
    arm_mat_sub_f32(&I_mat, &K_phi_T_mat, &I_K_phi_T_mat);

    // -------------------------- 步骤5：更新协方差矩阵 P --------------------------
    // 5.1 计算 (I - K×φᵀ) × P
    arm_mat_mult_f32(&I_K_phi_T_mat, &RLS->P_mat, &temp_mat);

    // 5.2 计算 P = (1/λ) × (I - K×φᵀ)×P
    arm_mat_scale_f32(&temp_mat, 1.0f / RLS->lambda, &RLS->P_mat);

    // -------------------------- 步骤6：更新待辨识参数 θ --------------------------
    // 6.1 计算预测误差：error = y - φᵀ×θ
    float32_t error = y;
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        error -= phi[i] * RLS->theta[i];
    }

    // 6.2 更新θ：θ = θ + K×error
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        RLS->theta[i] += RLS->K[i] * error;
    }
}

void RLS_update(RLS_HandleTypeDef *RLS, Motor_HandleTypeDef *motor)
{
    static float last_IA = 0.0f;
    static float phi[2] = {0.0f,0.0f};

    phi[0] = motor->MotorAlg.IA;
    phi[1] = (motor->MotorAlg.IA - last_IA)/motor->time.dt;

    RLS_update_theta(RLS,motor->MotorAlg.UA,phi);
    last_IA = motor->MotorAlg.IA;

    //需要摸清楚设置相电压的函数
}