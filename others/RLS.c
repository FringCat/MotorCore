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
    if (RLS == NULL) return;

    // 1. 初始化基础参数
    RLS->lambda = (lambda <= 0 || lambda > 1) ? 0.999f : lambda;
    
    // 2. 初始化P矩阵（对角阵，10.0f）
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        for (uint8_t j = 0; j < RLS_PARAM_NUM; j++)
        {
            RLS->P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 3. 初始化K/theta/phi/y为0
    memset(RLS->K, 0, sizeof(RLS->K));
    memset(RLS->theta, 0, sizeof(RLS->theta));
    memset(RLS->phi, 0, sizeof(RLS->phi));
    RLS->y = 0.0f;

    arm_mat_init_f32(&RLS->P_mat,  RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)RLS->P);
    arm_mat_init_f32(&RLS->K_mat,  RLS_PARAM_NUM, 1, RLS->K);
    arm_mat_init_f32(&RLS->theta_mat,  RLS_PARAM_NUM, 1, RLS->theta);
    arm_mat_init_f32(&RLS->phi_mat,  RLS_PARAM_NUM, 1, RLS->phi);
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
    static float32_t phi_T_P_row[RLS_PARAM_NUM];      // φᵀ×P 的行向量缓冲区 (1×N)，供 MATRIX_CHECK 下合法乘法链
    static float32_t phi_T_P_phi_buf[1];              // φᵀ×P×φ的结果缓冲区 (1×1)
    static float32_t K_phi_T_buf[RLS_PARAM_NUM][RLS_PARAM_NUM]; // K×φᵀ的缓冲区 (N×N)
    static float32_t I_K_phi_T_buf[RLS_PARAM_NUM][RLS_PARAM_NUM]; // I - K×φᵀ的缓冲区 (N×N)
    static float32_t I_mat_buf[RLS_PARAM_NUM][RLS_PARAM_NUM];     // 单位矩阵缓冲区 (N×N)

    // 局部矩阵结构体（复用，避免重复初始化）
    static arm_matrix_instance_f32 P_phi_mat;
    static arm_matrix_instance_f32 phi_T_mat;
    static arm_matrix_instance_f32 phi_T_P_mat;       // 1×N：φᵀ×P（不可写入 N×N 的 temp，否则 MATRIX_CHECK / 越界）
    static arm_matrix_instance_f32 phi_T_P_phi_mat;
    static arm_matrix_instance_f32 K_phi_T_mat;
    static arm_matrix_instance_f32 I_K_phi_T_mat;
    static arm_matrix_instance_f32 I_mat;
    static arm_matrix_instance_f32 temp_mat;          // 临时矩阵缓冲区 (N×N)
    static float32_t temp_buf[RLS_PARAM_NUM][RLS_PARAM_NUM]; // 临时数据缓冲区

    // 1. 安全检查：空指针防护
    if (RLS == NULL || phi == NULL)
    {
        return;
    }

    // 2. 更新输入输出缓存 + 输入数据清洗
    RLS->y = (isnan(y) || isinf(y)) ? 0.0f : y; // 清洗y值
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        RLS->phi[i] = (isnan(phi[i]) || isinf(phi[i])) ? 0.0f : phi[i]; // 清洗phi值
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

        // 初始化局部矩阵结构体（绑定维度和缓冲区）
        arm_mat_init_f32(&P_phi_mat,     RLS_PARAM_NUM, 1,              P_phi_buf);
        arm_mat_init_f32(&phi_T_mat,     1,              RLS_PARAM_NUM, phi_T_buf);
        arm_mat_init_f32(&phi_T_P_mat,   1,              RLS_PARAM_NUM, phi_T_P_row);
        arm_mat_init_f32(&phi_T_P_phi_mat, 1,           1,              phi_T_P_phi_buf);
        arm_mat_init_f32(&K_phi_T_mat,   RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)K_phi_T_buf);
        arm_mat_init_f32(&I_K_phi_T_mat, RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)I_K_phi_T_buf);
        arm_mat_init_f32(&I_mat,         RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)I_mat_buf);
        arm_mat_init_f32(&temp_mat,      RLS_PARAM_NUM, RLS_PARAM_NUM, (float32_t *)temp_buf);

        first_init = 1;
    }

    // -------------------------- 步骤1：计算 P×φ --------------------------
    if (arm_mat_mult_f32(&RLS->P_mat, &RLS->phi_mat, &P_phi_mat) != ARM_MATH_SUCCESS)
    {
        return; // 维度不匹配，直接退出
    }

    // -------------------------- 步骤2：计算 φᵀ×P×φ --------------------------
    // 2.1 转置φ得到φᵀ（N×1 → 1×N）
    if (arm_mat_trans_f32(&RLS->phi_mat, &phi_T_mat) != ARM_MATH_SUCCESS)
    {
        return;
    }

    // 2.2 计算 φᵀ×P → 结果必须为 1×N，写入 phi_T_P_mat（原先用 N×N temp 违反 MATRIX_CHECK 且可能越界）
    if (arm_mat_mult_f32(&phi_T_mat, &RLS->P_mat, &phi_T_P_mat) != ARM_MATH_SUCCESS)
    {
        return;
    }

    // 2.3 计算 (φᵀ×P)×φ = φᵀ×P×φ → 1×1
    if (arm_mat_mult_f32(&phi_T_P_mat, &RLS->phi_mat, &phi_T_P_phi_mat) != ARM_MATH_SUCCESS)
    {
        return;
    }

    // -------------------------- 步骤3：计算修正增益 K --------------------------
    // 3.1 计算分母：λ + φᵀ×P×φ
    float32_t denominator = RLS->lambda + phi_T_P_phi_buf[0];

    // 3.2 除0保护 + 分母清洗
    if (fabs(denominator) < 1e-6f || isnan(denominator) || isinf(denominator))
    {
        denominator = 1e-6f;
    }

    // 3.3 K = (P×φ) / denominator → 标量缩放
    if (arm_mat_scale_f32(&P_phi_mat, 1.0f / denominator, &RLS->K_mat) != ARM_MATH_SUCCESS)
    {
        return;
    }

    // 3.4 清洗K值，避免NAN/INF
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        if (isnan(RLS->K[i]) || isinf(RLS->K[i]))
        {
            RLS->K[i] = 0.0f;
        }
    }

    // -------------------------- 步骤4：计算 I - K×φᵀ --------------------------
    // 4.1 计算 K×φᵀ（N×1 乘 1×N → N×N）
    if (arm_mat_mult_f32(&RLS->K_mat, &phi_T_mat, &K_phi_T_mat) != ARM_MATH_SUCCESS)
    {
        return;
    }

    // 4.2 计算 I - K×φᵀ
    if (arm_mat_sub_f32(&I_mat, &K_phi_T_mat, &I_K_phi_T_mat) != ARM_MATH_SUCCESS)
    {
        return;
    }

    // -------------------------- 步骤5：更新协方差矩阵 P --------------------------
    // 5.1 计算 (I - K×φᵀ) × P（N×N）
    if (arm_mat_mult_f32(&I_K_phi_T_mat, &RLS->P_mat, &temp_mat) != ARM_MATH_SUCCESS)
    {
        return;
    }

    // 5.2 计算 P = (1/λ) × (I - K×φᵀ)×P + P矩阵清洗
    if (arm_mat_scale_f32(&temp_mat, 1.0f / RLS->lambda, &RLS->P_mat) != ARM_MATH_SUCCESS)
    {
        return;
    }
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        for (uint8_t j = 0; j < RLS_PARAM_NUM; j++)
        {
            if (isnan(RLS->P[i][j]) || isinf(RLS->P[i][j]))
            {
                RLS->P[i][j] = (i == j) ? 10.0f : 0.0f; // 异常时重置为初始对角阵
            }
        }
    }

    // -------------------------- 步骤6：更新待辨识参数 θ --------------------------
    // 6.1 计算预测误差：error = y - φᵀ×θ
    float32_t error = RLS->y; // 用清洗后的y值
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        // 逐个项清洗，避免单个项污染error
        float32_t term = RLS->phi[i] * RLS->theta[i];
        term = (isnan(term) || isinf(term)) ? 0.0f : term;
        error -= term;
    }

    // 6.2 提前清洗error（核心：更新theta前完成）
    if (isnan(error) || isinf(error))
    {
        error = 0.0f;
    }

    // 6.3 JScope调试值（已清洗，无NAN）
    static float rls_debug_data_0 = 0.0f;
    rls_debug_data_0 = error;

    // 6.4 更新θ：θ = θ + K×error + 最终清洗
    for (uint8_t i = 0; i < RLS_PARAM_NUM; i++)
    {
        float32_t delta = RLS->K[i] * error;
        delta = (isnan(delta) || isinf(delta)) ? 0.0f : delta;
        
        RLS->theta[i] += delta;

        // 兜底防护：确保theta绝对无NAN
        if (isnan(RLS->theta[i]) || isinf(RLS->theta[i]))
        {
            RLS->theta[i] = 0.0f;
        }
    }
}

void RLS_Ls_update_theta_scalar(RLS_HandleTypeDef *RLS, float32_t int_u, float32_t int_i, float32_t i, float32_t R_fixed)
{
    // 1. 安全检查
    if (RLS == NULL || isnan(int_u) || isnan(int_i) || isnan(i) || isnan(R_fixed))
    {
        return;
    }

    // 2. 计算一阶RLS的输入y（标量）：y = ∫u dt - R×∫i dt
    float32_t y = int_u - R_fixed * int_i;
    // 清洗y值
    y = (isnan(y) || isinf(y)) ? 0.0f : y;
    RLS->y = y; // 存入结构体（兼容原有逻辑）

    // 3. 回归项phi（标量）：phi = i
    float32_t phi = (isnan(i) || isinf(i)) ? 0.0f : i;
    RLS->phi[0] = phi; // 存入结构体

    // 4. 提取协方差P（标量，仅用P[0][0]）
    float32_t P = RLS->P[0][0];
    // 清洗P值
    P = (isnan(P) || isinf(P)) ? 10.0f : P;

    // -------------------------- 一阶RLS标量核心运算 --------------------------
    // 步骤1：计算分母 λ + P×φ²
    float32_t denominator = RLS->lambda + P * phi * phi;
    // 除0保护
    if (fabs(denominator) < 1e-6f)
    {
        denominator = 1e-6f;
    }

    // 步骤2：计算增益K（标量）
    float32_t K = (P * phi) / denominator;
    // 清洗K值
    K = (isnan(K) || isinf(K)) ? 0.0f : K;
    RLS->K[0] = K; // 存入结构体

    // 步骤3：计算预测误差（标量）
    float32_t error = y - phi * RLS->theta[0];
    error = (isnan(error) || isinf(error)) ? 0.0f : error;

    // 步骤4：更新Ls（标量，theta[0] = Ls）
    float32_t delta = K * error;
    delta = (isnan(delta) || isinf(delta)) ? 0.0f : delta;
    RLS->theta[0] += delta;

    // 步骤5：更新协方差P（标量）
    P = (1.0f / RLS->lambda) * (P - K * phi * P);
    // 清洗P值（避免溢出/NaN）
    P = (isnan(P) || isinf(P)) ? 10.0f : P;
    P = (fabs(P) > 1e6f) ? 1e6f : P; // 限幅，防止P过大
    RLS->P[0][0] = P;

    // // -------------------------- 物理约束（关键） --------------------------
    // // Ls非负 + 合理范围（根据你的电机调整，比如0~0.1H=100mH）
    // if (isnan(RLS->theta[0]) || isinf(RLS->theta[0]))
    // {
    //     RLS->theta[0] = 0.0f;
    // }
    // RLS->theta[0] = (RLS->theta[0] < 0.0f) ? 0.0f : RLS->theta[0];
    // RLS->theta[0] = (RLS->theta[0] > 0.1f) ? 0.1f : RLS->theta[0];

    // JScope调试：Ls值（theta[0]）
    static float rls_ls_debug = 0.0f;
    rls_ls_debug = RLS->theta[0];
}


// void RLS_update(RLS_HandleTypeDef *RLS, Motor_HandleTypeDef *motor)
// {
//     static float last_I = 0.0f;
//     static float I = 0.0f;
//     static float U = 0.0f;
//     static float phi[2] = {0.0f,0.0f};

//     // arm_sqrt_f32(motor->MotorAlg.Iq*motor->MotorAlg.Iq + motor->MotorAlg.Id*motor->MotorAlg.Id,&I);
//     // arm_sqrt_f32(motor->MotorAlg.Uq*motor->MotorAlg.Uq + motor->MotorAlg.Ud*motor->MotorAlg.Ud,&U);

//     // motor->MotorData.Ibus = I;
//     // motor->MotorData.Ubus = U;

//     I = motor->MotorAlg.Id;
//     U = motor->MotorAlg.Ud;

//     phi[0] = I ;
//     phi[1] = (I - last_I);

//     RLS_update_theta(RLS,U,phi);
//     last_I = I;

//     // set_pwm(motor,Ua/motor->MotorConfig.UMAX,0.0f,0.0f);
// }

void RLS_update(RLS_HandleTypeDef *RLS, Motor_HandleTypeDef *motor)//根据积分形式电压方程修改
{
    static float last_I = 0.0f;
    static float I = 0.0f;
    static float U = 0.0f;
    static float phi[2] = {0.0f,0.0f};

    // static float time = 0.0f;
    // time += motor->time.dt;
    // I = motor->MotorAlg.Id * time;
    // U = motor->MotorAlg.Ud * time;

    I += motor->MotorAlg.Id * motor->time.dt;
    U += motor->MotorAlg.Ud * motor->time.dt;

    phi[0] = I ;
    phi[1] = motor->MotorAlg.Id;

    RLS_update_theta(RLS,U,phi);
    // RLS_Ls_update_theta_scalar(RLS,U,I,motor->MotorAlg.Id,0.2f);

}