#include "SMO.h"

void SMO_Init(SMO_HandleTypeDef *hsmo)
{
  /* #################### 1. 电机参数初始化 #################### */
  hsmo->Rs = 0.1423715f;        // 实际定子电阻
  hsmo->Ls = 0.000108832f;      // 实际定子电感
  hsmo->PolePairs = 14;         // 实际极对数

  /* #################### 2. SMO核心配置 #################### */
  hsmo->SmoGain = 75.1f;        // 滑模增益（根据调试调整）
  hsmo->Ts = 1.0e-4f;           // 10kHz控制周期

  /* #################### 3. 状态变量清零 #################### */
  hsmo->Ualpha = 0.0f;
  hsmo->Ubeta = 0.0f;
  hsmo->Ialpha = 0.0f;
  hsmo->Ibeta = 0.0f;
  hsmo->IalphaHat = 0.0f;
  hsmo->IbetaHat = 0.0f;
  hsmo->EalphaHat = 0.0f;
  hsmo->EbetaHat = 0.0f;
  hsmo->Salpha = 0.0f;
  hsmo->Sbeta = 0.0f;
  hsmo->ThetaEst = 0.0f;
  hsmo->OmegaEst = 0.0f;
  hsmo->LastTheta = 0.0f;

  /* #################### 4. PLL参数初始化（核心新增） #################### */
  hsmo->PlKp = 3000.0f;          // PLL比例增益（初始值，调试范围50~200）
  hsmo->PlKi = 0.0f;           // PLL积分增益（初始值，调试范围5~50）
  hsmo->PlIntegral = 0.0f;      // 积分项初始化为0
  hsmo->PlErr = 0.0f;           // 相位误差初始化为0
  hsmo->SinTheta = 0.0f;        // 初始正弦值
  hsmo->CosTheta = 1.0f;        // 初始余弦值
}


/**
  * @brief  SMO+PLL核心更新函数（替换反正切，用PLL跟踪角度）
  * @param  hsmo: SMO句柄
  * @param  Ualpha/Ubeta: α/β轴电压输入
  * @param  Ialpha/Ibeta: α/β轴电流反馈
  */
float update_SMO(SMO_HandleTypeDef *hsmo,float T,float Ualpha,float Ubeta,float Ialpha,float Ibeta)
{
  /* #################### 步骤1：更新输入变量 #################### */
  hsmo->Ualpha = Ualpha;
  hsmo->Ubeta = Ubeta;
  hsmo->Ialpha = Ialpha;
  hsmo->Ibeta = Ibeta;
  hsmo->Ts = T;  

  /* #################### 步骤2：计算滑模面（电流误差） #################### */
  hsmo->Salpha = hsmo->IalphaHat - Ialpha;
  hsmo->Sbeta = hsmo->IbetaHat - Ibeta;

  /* #################### 步骤3：SMO观测电流和反电动势 #################### */
  float invLs = 1.0f / hsmo->Ls;          
  float Rs_over_Ls = hsmo->Rs * invLs;    

  static float Ealpha_fal = 0.0f;
  static float Ebeta_fal = 0.0f;
  static float Ealpha_Hat = 0.0f;
  static float Ebeta_Hat = 0.0f;

  // Ealpha_fal = fal(hsmo->Salpha,0.9f,0.01f);
  // Ebeta_fal = fal(hsmo->Sbeta,0.9f,0.01f);
  Ealpha_Hat = hsmo->SmoGain * SAT(hsmo->Salpha,45.0f);
  Ebeta_Hat = hsmo->SmoGain * SAT(hsmo->Sbeta,45.0f);

  // α轴观测电流更新
  hsmo->IalphaHat += hsmo->Ts * ( -Rs_over_Ls * hsmo->IalphaHat  + invLs * (Ualpha - Ealpha_Hat));

  // β轴观测电流更新
  hsmo->IbetaHat += hsmo->Ts * ( -Rs_over_Ls * hsmo->IbetaHat  + invLs * (Ubeta - Ebeta_Hat));

  // 观测反电动势（等效控制）
  hsmo->EalphaHat = Ealpha_Hat;
  hsmo->EbetaHat = Ebeta_Hat;

  /* #################### 步骤4：PLL锁相环跟踪角度（核心替换部分） #################### */
  // ------------ 4.1 计算PLL相位误差 ------------
  // 原理：反电动势矢量 E = [Eα; Eβ] = Ke*ω*[-sinθ; cosθ]
  // 相位误差 = Eα*cosθ + Eβ*sinθ （误差为0时，PLL锁定角度）
  hsmo->PlErr = hsmo->EalphaHat * hsmo->CosTheta + hsmo->EbetaHat * hsmo->SinTheta;

  // ------------ 4.2 PLL PI控制器（输出电角速度） ------------
  // 比例项 + 积分项（积分限幅避免饱和）
  hsmo->OmegaEst = hsmo->PlKp * hsmo->PlErr + hsmo->PlIntegral;
  // 积分项更新（带限幅，防止积分饱和）
  hsmo->PlIntegral += hsmo->PlKi * hsmo->PlErr * hsmo->Ts;
  // 积分限幅（根据电机最大电角速度调整，示例：14对极电机，最高3000rpm → ω=2π*3000/60*14=4398rad/s）
  if (hsmo->PlIntegral > 5000.0f) hsmo->PlIntegral = 5000.0f;
  if (hsmo->PlIntegral < -5000.0f) hsmo->PlIntegral = -5000.0f;

  // ------------ 4.3 积分角速度得到电角度 ------------
  hsmo->ThetaEst += hsmo->OmegaEst * hsmo->Ts;
  hsmo->angle_el_output = hsmo->ThetaEst + PI;
  NORMALIZE_ANGLE(hsmo->ThetaEst);  // 角度归一化
  hsmo->angle_el_output = Limit_angle_el(hsmo->angle_el_output);  // 角度归一化

  // ------------ 4.4 更新正弦/余弦值（供下一周期计算误差） ------------
  hsmo->SinTheta = arm_sin_f32(hsmo->ThetaEst);  // CMSIS-DSP快速正弦
  hsmo->CosTheta = arm_cos_f32(hsmo->ThetaEst);  // CMSIS-DSP快速余弦

  /* #################### 步骤5：更新历史角度 #################### */
  hsmo->LastTheta = hsmo->ThetaEst;

  return hsmo->angle_el_output;
}

float update_angle_SMO(SMO_HandleTypeDef *hsmo,Motor_HandleTypeDef *motor,float Ualpha,float Ubeta,float Ialpha,float Ibeta)
{
  static float angle_el_SMO = 0.0f;
  static float angle_el_all_SMO = 0.0f;
  static float angle_SMO = 0.0f;
  static float Velocity_SMO = 0.0f;
  static float Velocity_raw_SMO = 0.0f;
  static float last_Velocity_SMO = 0.0f;
  static float last_angle_SMO = 0.0f;
  static float last_angle_el_SMO = 0.0f;

  float error_angle_el = angle_el_SMO - last_angle_el_SMO;
  if(fabs(error_angle_el) > (0.8f*2*PI))
  {
    if((error_angle_el)<0){angle_el_all_SMO += (2*PI - last_angle_el_SMO + angle_el_SMO) ;}//正转
    else if((error_angle_el)>=0){angle_el_all_SMO += -(2*PI - angle_el_SMO + last_angle_el_SMO) ;}//反转
  }
  else 
  {
    angle_el_all_SMO += error_angle_el ;
  }

  last_angle_el_SMO = angle_el_SMO;

  angle_el_SMO = update_SMO(hsmo, motor->time.dt, Ualpha, Ubeta, Ialpha, Ibeta);

  angle_SMO =  Limit_angle_el(angle_el_all_SMO/(float)motor->MotorConfig.Pole_pairs);

  Velocity_raw_SMO = Calculate_velocity_raw(angle_SMO, last_angle_SMO, motor->time.dt);
  Velocity_SMO =  Calculate_LPF(Velocity_raw_SMO, last_Velocity_SMO,0.015f);
  
  last_Velocity_SMO = Velocity_SMO; 
  last_angle_SMO = angle_SMO;

  motor->MotorAlg.angle = angle_SMO;
  motor->MotorAlg.last_angle = last_angle_SMO;
  motor->MotorAlg.angle_el = angle_el_SMO;
  motor->MotorAlg.Velocity = Velocity_SMO;

  return angle_SMO;
}