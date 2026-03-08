/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_alg.h"
#include "can_handler.h"
#include "fdcan.h"
#include "arm_math.h"  
#include "fsm.h"
#include "ADRC.h"
#include "SMO.h"
#include "RLS.h"

#define INV_SQRT3_F   (1.0f / 1.7320508075688772f)  // ≈0.5773502691896257f
#define INV_3_F       (1.0f / 3.0f)                 // 1/3，Clark变换用
#define PI_F          3.141592653589793f
#define TWO_PI_F      (2.0f * PI_F)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */
extern Motor_HandleTypeDef motor;
extern CAN_Handler_t can_handler;
extern fsm_HandleTypeDef fsm_motor;
extern int isoffset_done;
extern ADRC_HandleTypeDef ADRC;
extern SMO_HandleTypeDef SMO;
extern RLS_HandleTypeDef RLS;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);
  if(isoffset_done==0)
  {
    isoffset_done = update_Ioffset_nonblock(&motor);
  }
  else
  {
    motor.MotorData.CurrentData.I_raw.IA_raw = motor.MotorDrv.Update_Ia_raw();
    motor.MotorData.CurrentData.I_raw.IB_raw = motor.MotorDrv.Update_Ib_raw();

    motor.MotorAlg.IC = motor.MotorDrv.Cal_Ia(motor.MotorData.CurrentData.I_raw.IA_raw , motor.MotorData.IA_offset_raw);
    motor.MotorAlg.IB = motor.MotorDrv.Cal_Ib(motor.MotorData.CurrentData.I_raw.IB_raw , motor.MotorData.IB_offset_raw);
    motor.MotorAlg.IA = -(motor.MotorAlg.IC+motor.MotorAlg.IB);
  }

  update_dt(&motor);
  update_angle(&motor);
  update_Clark(&motor);
  update_Park(&motor);
  // update_velocity_LPF(&motor);
  // update_angle_SMO(&SMO, &motor, motor.MotorAlg.Ualpha, motor.MotorAlg.Ubeta, motor.MotorAlg.Ialpha, motor.MotorAlg.Ibeta); //打开SMO时记得关闭 update_angle 跟 update_velocity_LPF
  // RLS_update(&RLS, &motor);
  HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
  
  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief  FDCAN RX FIFO0回调函数（收到新数据时自动触发）
  * @param  hfdcan: FDCAN句柄
  * @retval None
  */
// float time_out = 0.0f;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_handler.can_rxheader, can_handler.cmd_buf);
  switch (can_handler.can_rxheader.Identifier)
  {
    case CAN_ID:
    {
      can_update_cmd(&can_handler, can_handler.cmd_buf);
      can_update_data(&can_handler, motor.MotorAlg.angle_flange, motor.MotorAlg.Velocity_flange, motor.MotorAlg.Iq*(1/motor.MotorConfig.Kt), 0.0f, 0.0f);
      fsm_motor.state = RUN;
      fsm_motor.timeout = 0.1f;//100MS内没有新命令下来，超时，取消控制
      // time_out = 0.1f;
    }; break;
    case Require_Status_ID:
    {
      // can_update_data(&can_handler, motor.MotorAlg.angle_flange, motor.MotorAlg.Velocity_flange, motor.MotorAlg.Iq*(1/motor.MotorConfig.Kt), 0.0f, 0.0f);
    };break;
    case DISABLE_ID:
    {
      fsm_motor.state = STOP;
      can_reset_cmd(&can_handler);
    };break;
    case CALIBRATION_ID:
    {
      fsm_motor.state = CALIBRATION;
    };break;
    case En_MIT_MODE_ID:
    {

    };break;
    case ZERO_POSITION_ID:
    {

    };break;
    default:
    {
      
    };break;
  }
  can_printf_cmd(&can_handler);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint16_t timer_cnt = TIM1->CNT; 
  static float Kp = 0.0f;
  static float Kd = 0.3f;
  static float target_Velocity = 30.0f;
  static float target_Position =  0.0f;
  static float forward_torque_flange =  0.0f;
  static float forward_torque =  0.0f;
  static float output = 0;

  if (htim->Instance == TIM1)
  {
    if(timer_cnt <500)
    {
      HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin);
      if(isoffset_done)
      {
        
        //三角波注入例程
        // {
        //   static float time = 0.0f;
        //   static int flag = 0;
          
        //   if(flag == 0){time += motor.time.dt;}
        //   else{time -= motor.time.dt;}

        //   if(flag == 0 && time > 0.1f)
        //   {
        //     flag = 1;
        //   }
        //   else if(flag == 1 && time < 0.0f)
        //   {
        //     flag = 0;
        //   }
        //   motor.MotorAlg.Uq = 0.0f;
        //   motor.MotorAlg.Ud = motor.MotorConfig.UMAX*0.2f*time;
        //   update_svpwm(&motor);
        // }

        //正弦注入例程
        {
          static float time = 0.0f;
          static float output_sin = 0.0f;
          time += motor.time.dt;
          output_sin = arm_sin_f32(Limit_angle_el(time*500.0f))*5.0f;
          motor.MotorAlg.Uq = Calculate_PID(0.0f, motor.MotorAlg.Iq , motor.time.dt , &motor.MotorAlg.iq_pid);
          motor.MotorAlg.Ud = Calculate_PID(output_sin, motor.MotorAlg.Id , motor.time.dt , &motor.MotorAlg.id_pid);
          update_svpwm(&motor);//输出SVPWM
        }

        // RLS例程
        { 
          RLS_update(&RLS, &motor);
        }

        // 无减速箱的MIT例程
        // {  
        //   output = (forward_torque + Kp * (target_Position - motor.MotorAlg.angle)  + Kd * (target_Velocity - motor.MotorAlg.Velocity)); //无减速箱
        //   motor.MotorAlg.Uq = Calculate_PID(output, motor.MotorAlg.Iq , motor.time.dt , &motor.MotorAlg.iq_pid);
        //   motor.MotorAlg.Ud = Calculate_PID(0.0f, motor.MotorAlg.Id , motor.time.dt , &motor.MotorAlg.id_pid);
        //   update_svpwm(&motor);//输出SVPWM
        // }

        // 带减速箱的MIT例程
        // {  
        //   motor.MotorAlg.angle_flange = Limit_angle_flange(motor.MotorData.angle_all,motor.MotorConfig.GR);
        //   motor.MotorAlg.Velocity_flange = motor.MotorAlg.Velocity/motor.MotorConfig.GR; //更新法兰速度
        //   output = (1/motor.MotorConfig.Kt)*(forward_torque_flange + Kp * (target_Position - motor.MotorAlg.angle_flange)  + Kd * (target_Velocity - motor.MotorAlg.Velocity_flange));//带减速箱
        //   motor.MotorAlg.Uq = Calculate_PID(output, motor.MotorAlg.Iq , motor.time.dt , &motor.MotorAlg.iq_pid);
        //   motor.MotorAlg.Ud = Calculate_PID(0.0f, motor.MotorAlg.Id , motor.time.dt , &motor.MotorAlg.id_pid);
        //   update_svpwm(&motor);//输出SVPWM
        // }

        // ADRC例程
        // {
        //   motor.MotorAlg.Uq = Limit(update_ADRC(&ADRC,adrc_input,motor.MotorAlg.Velocity),12.0f,-12.0f);
        //   update_svpwm(&motor);//输出SVPWM
        // }

        // 速度环例程
        // {
        //   motor.MotorAlg.Uq = Calculate_PID(30.0f, motor.MotorAlg.Velocity, motor.time.dt , &motor.MotorAlg.velocity_pid);
        //   update_svpwm(&motor);//输出SVPWM
        // }

        // 单电流环例程
        // {
        //   motor.MotorAlg.Uq = Calculate_PID(1.0f, motor.MotorAlg.Iq , motor.time.dt , &motor.MotorAlg.iq_pid);
        //   motor.MotorAlg.Ud = Calculate_PID(0.0f, motor.MotorAlg.Id , motor.time.dt , &motor.MotorAlg.id_pid);
        //   update_svpwm(&motor);//输出SVPWM
        // }

        // 速度电流环例程
        // {
        //   float output = Calculate_PID(22.0f, motor.MotorAlg.Velocity, motor.time.dt , &motor.MotorAlg.velocity_pid);
        //   motor.MotorAlg.Uq = Calculate_PID(output, motor.MotorAlg.Iq , motor.time.dt , &motor.MotorAlg.iq_pid);
        //   motor.MotorAlg.Ud = Calculate_PID(0.0f, motor.MotorAlg.Id , motor.time.dt , &motor.MotorAlg.id_pid);
        //   update_svpwm(&motor);//输出SVPWM
        // }
        
        // SMO例程
        // {
        //   float output = Calculate_PID(20.0f, motor.MotorAlg.Velocity, motor.time.dt , &motor.MotorAlg.velocity_pid);
        //   motor.MotorAlg.Uq = Calculate_PID(output, motor.MotorAlg.Iq , motor.time.dt , &motor.MotorAlg.iq_pid);
        //   motor.MotorAlg.Ud = Calculate_PID(0.0f, motor.MotorAlg.Id , motor.time.dt , &motor.MotorAlg.id_pid);
        //   update_svpwm(&motor);//输出SVPWM
        //   // update_angle_SMO(&SMO, &motor, motor.MotorAlg.Ualpha, motor.MotorAlg.Ubeta, motor.MotorAlg.Ialpha, motor.MotorAlg.Ibeta); //这个要放在ADC中断里跑
        // }

        //状态机例程
        // {
        //   fsm_run();
        // }
        
      }
      HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin);
    }
  }
}
/* USER CODE END 1 */
