#include "ADRC.h"

/**
 * @brief  ADRC控制器整体初始化函数
 * @note   电机上电/复位时调用，初始化TD/ESO/NLSEF所有参数和状态
 * @param  adrc: ADRC控制器结构体指针
 * @param  id: 电机ID（区分多电机场景）
 * @param  h: 采样周期（如1ms=0.001f，所有模块共用）
 * @param  b0: 标称控制增益（速度环b0=1/J，J为电机转动惯量）
 * @retval 无
 */
void ADRC_Init(ADRC_HandleTypeDef *adrc, int id, float h, float b0)
{
    // 1. 基础参数初始化
    adrc->ID = id;  // 电机ID赋值

    /********************* TD模块初始化 *********************/
    // 速度因子r：电机控制常用50（平衡响应速度与抗噪）
    adrc->TD.r = 6940.01f;
    // 采样周期h：与系统采样周期一致（外部传入）
    adrc->TD.h = h;
    // 整体增益R：默认1.0f（强化跟踪时可改2~5）
    adrc->TD.R = 160.5f;
    // TD状态初始化（避免初始突变）
    adrc->TD.x1 = 0.0f;
    adrc->TD.x2 = 0.0f;

    /********************* ESO模块初始化 *********************/
    // 观测增益：按极点配置法（beta01为核心，beta02=beta01?，beta03=beta01?）
    adrc->ESO.beta01 = 150.0f;        // 观测带宽核心参数（电机速度环常用40）
    adrc->ESO.beta02 = adrc->ESO.beta01 * adrc->ESO.beta01; // 1600.0f
    adrc->ESO.beta03 = adrc->ESO.beta02 * adrc->ESO.beta01; // 64000.0f
    // 标称控制增益：外部传入（与电机惯量匹配）
    adrc->ESO.b0 = b0;
    // 采样周期：与TD一致
    adrc->ESO.h = h;

    // ESO的fal非线性参数（工程经验值，速度环适配）
    adrc->ESO.alpha_1 = 0.9f;  adrc->ESO.delta_1 = 0.01f;
    adrc->ESO.alpha_2 = 0.9f;  adrc->ESO.delta_2 = 0.01f;
    adrc->ESO.alpha_3 = 0.9f;  adrc->ESO.delta_3 = 0.01f;

    // ESO状态初始化
    adrc->ESO.z1 = 0.0f;
    adrc->ESO.z2 = 0.0f;
    adrc->ESO.z3 = 0.0f;

    /********************* NLSEF模块初始化 *********************/
    // 反馈增益（速度环经验值）
    adrc->NLSEF.beta1 = 1600.5f;   // 位置/速度误差反馈增益
    adrc->NLSEF.beta2 = 3.1f;    // 微分误差反馈增益

    // NLSEF的fal非线性参数
    adrc->NLSEF.alpha_1 = 0.85f; adrc->NLSEF.delta_1 = 0.00000000000005f;
    adrc->NLSEF.alpha_2 = 0.9f;  adrc->NLSEF.delta_2 = 0.001f;

    // NLSEF状态初始化
    adrc->NLSEF.e1 = 0.0f;
    adrc->NLSEF.e2 = 0.0f;
    adrc->NLSEF.u0 = 0.0f;
    adrc->NLSEF.u = 0.0f;
}

float sign(float x)
{
    if (x > 0.0){ return 1.0;}
    else if(x < 0.0){ return -1.0; }
    else if (x == 0.0){ return 0.0;}
    else
    { 

    }
}

float fsg(float x ,float d)
{
    return (sign(x + d) - sign(x - d)) / 2;
}

/**
 * 
 * @brief  最速控制综合函数fhan()（ADRC-TD核心）
 * @note   适配电机控制的嵌入式场景，无除法溢出/浮点异常，运算效率高
 * @param  x1: TD跟踪误差（参考指令 - 跟踪值）
 * @param  x2: TD微分信号
 * @param  r:  速度因子（跟踪速度，电机控制常用10~100）
 * @param  h:  滤波因子/采样周期（与系统采样周期一致，如1ms=0.001f）
 * @retval fhan函数输出值（用于更新TD的微分状态x2）
 */
float fhan(float x1, float x2, float r, float h)
{
    // 1. 基础参数计算（保留原有逻辑）
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;

    // 2. 替换fabs(y)为arm_abs_f32（硬件加速绝对值）
    float abs_y;
    arm_abs_f32(&y, &abs_y, 1); // 单元素绝对值计算（硬件加速）

    // 3. 开方输入计算 + 保护（避免负数输入）
    float sqrt_input = d * (d + 8 * abs_y);
    sqrt_input = (sqrt_input < 0.0f) ? 0.0f : sqrt_input; // 负数置0，防止NaN

    // 4. 替换sqrt()为arm_sqrt_f32（硬件加速平方根）
    float a1;
    arm_status sqrt_status = arm_sqrt_f32(sqrt_input, &a1);
    // 异常处理：若开方失败（理论上不会触发，因已做输入保护）
    if (sqrt_status != ARM_MATH_SUCCESS)
    {
        a1 = 0.0f;
    }

    // 5. 后续逻辑保留，仅优化计算效率
    float a2 = a0 + sign(y) * (a1 - d) / 2.0f;
    float Sy = fsg(y, d);
    float a = (a0 + y - a2) * Sy + a2;
    float Sa = fsg(a, d);

    // 6. 最终输出计算（精简运算步骤）
    float sign_a = sign(a);
    float result = r * (a / d - sign_a) * Sa + r * sign_a;

    return result;
}

// fal非线性函数
float fal(float e, float alpha, float delta)
{
    float abs_e = (e >= 0) ? e : -e;
    float fal_out;

    if(abs_e > delta)
    {
        fal_out = powf(abs_e, alpha) * ((e > 0) ? 1.0f : -1.0f);
    }
    else
    {
        fal_out = e / powf(delta, 1 - alpha);
    }

    return fal_out;
}

/**
 * @brief  ESO模块状态更新函数（二阶非线性ESO，适配电机控制）
 * @note   电机控制主循环中调用（每采样周期1次），观测电机状态+总扰动
 * @param  eso: ESO参数结构体指针（包含观测增益、fal参数、状态变量）
 * @param  y_meas: 电机输出实测值（如速度/位置采样值）
 * @param  u: 前一周期的控制量（NLSEF输出的u，补偿后的值）
 * @retval 无（更新eso->z1/eso->z2/eso->z3）
 */
void update_ADRC_ESO(ESO_HandleTypeDef *eso, float y_meas, float u)
{
    static float I_e = 0.0f;
    // 1. 计算观测误差：实测值y_meas - ESO观测值z1（核心反馈项）
    float e = y_meas - eso->z1 ;
    I_e += e*eso->h;
    // 2. 计算各状态的非线性fal反馈项（与结构体参数一一对应）
    
    static float fal_e1 = 0.0f;
    static float fal_e2 = 0.0f;
    static float fal_e3 = 0.0f;

    fal_e1 = fal(e, eso->alpha_1, eso->delta_1);
    fal_e2 = fal(e, eso->alpha_2, eso->delta_2);
    fal_e3 = fal(e, eso->alpha_3, eso->delta_3);

    eso->z3 = eso->h * (eso->beta03 * fal_e3);
    eso->z2 += eso->h * (eso->z3 + eso->beta02 * fal_e2 + eso->b0 * u);
    eso->z1 += eso->h * (eso->z2 + eso->beta01 * fal_e1);
    // if(fabs(I_e)<1.3f)
    // {
    //     // eso->z1 = eso->z1 + eso->h * (eso->z2 + eso->beta01 * fal_e1);
    //     // eso->z2 = eso->z2 + eso->h * (eso->z3 + eso->beta02 * fal_e2 + eso->b0 * u);
    //     // eso->z3 = eso->z3 + eso->h * (eso->beta03 * fal_e3);

    //     eso->z3 = eso->h * (eso->beta03 * fal_e3);
    //     eso->z2 += eso->h * (eso->z3 + eso->beta02 * fal_e2 + eso->b0 * u);
    //     eso->z1 += eso->h * (eso->z2 + eso->beta01 * fal_e1);
        
    // }
    // else
    // {
    //     eso->z1 = 0 ;
    //     eso->z2 = 0 ;
    //     eso->z3 = 0 ;
    //     I_e = 0;
    // }

}

/**
 * @brief  TD模块状态更新函数（核心）
 * @note   电机控制主循环中调用（每采样周期1次），基于fhan函数更新跟踪状态
 * @param  td: TD参数结构体指针
 * @param  v_ref: 参考输入（电机目标速度/位置，如100rad/s）
 * @retval 无（更新td->x1和td->x2）
 */
void update_ADRC_TD(TD_HandleTypeDef *td, float v_ref)
{
    float x1_err = td->x1 - v_ref ;  

    float fh = -fhan(x1_err, td->x2, td->r, td->h);
    
    td->x1 = td->x1 + td->h * td->x2;

    td->x2 = td->x2 + td->h * td->R * fh;
}

/**
 * @brief  NLSEF模块状态更新函数（二阶非线性状态误差反馈，适配电机控制）
 * @note   电机控制主循环中调用（每采样周期1次），生成补偿后的控制量
 * @param  nlsef: NLSEF参数结构体指针（包含反馈增益、fal参数、状态变量）
 * @param  td: TD模块结构体指针（提供参考状态x1/x2）
 * @param  eso: ESO模块结构体指针（提供观测状态z1/z2和总扰动z3）
 * @param  b0: 标称控制增益（与ESO的b0一致，用于扰动补偿）
 * @retval 无（更新nlsef->e1/e2/u0/u）
 */
void update_ADRC_NLSEF(NLSEF_HandleTypeDef *nlsef, 
                       TD_HandleTypeDef *td, 
                       ESO_HandleTypeDef *eso,
                       float b0)
{
    // 1. 计算状态误差（匹配你TD/ESO的误差逻辑：参考-观测）
    nlsef->e1 = td->x1 - eso->z1;  // 速度/位置误差：TD平滑指令 - ESO观测值
    nlsef->e2 = td->x2 - eso->z2;  // 微分误差：TD指令微分 - ESO观测微分

    // 2. 计算非线性误差反馈项（与结构体参数一一对应）
    float fal_e1 = fal(nlsef->e1, nlsef->alpha_1, nlsef->delta_1);
    float fal_e2 = fal(nlsef->e2, nlsef->alpha_2, nlsef->delta_2);

    // 3. 计算基础控制量u0（非线性反馈叠加）
    nlsef->u0 = nlsef->beta1 * fal_e1 + nlsef->beta2 * fal_e2;
    // nlsef->u0 = nlsef->beta1 * fal_e1;

    // 4. 扰动补偿：生成最终控制量u（核心抗扰逻辑）
    // 公式：u = (u0 - 总扰动)/标称增益b0
    nlsef->u = (nlsef->u0 - eso->z3) / b0;
    // nlsef->u = nlsef->u0;
}

/**
 * @brief  ADRC控制器总更新函数（一键调用，整合TD/ESO/NLSEF）
 * @note   电机控制主循环中单次调用即可完成ADRC全流程计算
 * @param  adrc: ADRC控制器结构体指针
 * @param  target: 电机目标值（如目标速度/位置）
 * @param  now: 电机当前实测值（如编码器采样的速度/位置）
 * @retval float: 最终输出的控制量（已限幅，可直接给到电机驱动）
 */
float update_ADRC(ADRC_HandleTypeDef *adrc, float target, float now)
{
    // 1. 保存上一周期的控制量（ESO需要前一周期的u来观测扰动）
    float u_prev = adrc->NLSEF.u;

    // 2. 更新TD模块：平滑目标值，生成跟踪指令x1和微分x2
    update_ADRC_TD(&adrc->TD, target);

    // 3. 更新ESO模块：观测当前状态z1/z2和总扰动z3（传入实测值+上一周期控制量）
    update_ADRC_ESO(&adrc->ESO, now, u_prev);

    // 4. 更新NLSEF模块：生成补偿后的控制量（传入ESO的标称增益b0）
    update_ADRC_NLSEF(&adrc->NLSEF, &adrc->TD, &adrc->ESO, adrc->ESO.b0);

    // 6. 返回最终限幅后的控制量
    return adrc->NLSEF.u;
}
