/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_drv.h"
#include "foc_alg.h"
#include "mt6835.h"
#include "MT6816.h"
#include "SEGGER_RTT.h"
#include "arm_math.h"
#include "flash.h"
#include "can_handler.h"
#include "fsm.h"
#include "drv_DRV835X.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
fsm_HandleTypeDef fsm_motor;
Motor_HandleTypeDef motor;
mt6835_t *mt6835 = NULL;
mt6816_HandleTypeDef mt6816;
Motor_ConfigTypeDef motorconfig;
CAN_Handler_t can_handler;

int isoffset_done = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_RTT_Init();                                //JLINK RTT 初始化
  SEGGER_RTT_SetTerminal(0);
  
  fsm_init(&fsm_motor);                             //状态机初始化
  foc_init(&motor);                                 //foc初始化
  mt6816_init(&mt6816);                             //副磁编初始化
  mt6835 = mt6835_stm32_spi_port_init();            //主磁编初始化
  DRV835X_Init();                                   //电驱芯片初始化

  HAL_Delay(1000);
  for(int i = 0; i<30 && motor.MotorConfig.loopcount_rotor == 0XFFFF ; i++)//双编码判定圈数（flange范围±Π）
  {
    mt6816_update_angle(&mt6816);
    update_loopcount_rotor_block(&motor,mt6816.angle);
    motor.MotorData.angle_all = (motor.MotorConfig.loopcount_rotor * 2 * PI + Limit_angle_el(motor.MotorAlg.angle-motor.MotorConfig.angle_zero_gear_A) );
    motor.MotorAlg.angle_flange = Limit_angle_flange(motor.MotorData.angle_all,motor.MotorConfig.GR);
    motor.MotorDrv.Delayms(1);
  }
  update_flash(ADDR_FLASH_SECTOR_0,(uint64_t*)&motor.MotorConfig,sizeof(motor.MotorConfig)/4);
  flash_read(ADDR_FLASH_SECTOR_0,(uint32_t*)&motorconfig,sizeof(motor.MotorConfig)/4);
  SEGGER_RTT_printf(0,"LWJ666 has inited!\n");

  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);         //adc采样中断(PWM通道4触发)
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);       //定时中断(20Khz 兼为PWM定时器) 
  // __HAL_FDCAN_ENABLE_IT(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); //使能CAN中断
  
  // __HAL_FDCAN_DISABLE_IT(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); 
  // __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);      
  // __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);   

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) 
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // if(isoffset_done)
    // {
    //   update_PHASE_nonblock(&motor);
    // }

    // 双编码实时读取法兰角度例程
    // {
    // mt6816_update_angle(&mt6816);
    // update_loopcount_rotor_block(&motor,mt6816.angle);
    // motor.MotorData.angle_all = (motor.MotorConfig.loopcount_rotor * 2 * PI + Limit_angle_el(motor.MotorAlg.angle-motor.MotorConfig.angle_zero_gear_A) );
    // motor.MotorAlg.angle_flange = Limit_angle_flange(motor.MotorData.angle_all,motor.MotorConfig.GR);
    // }

    // 各类校准例程
    // {
    // update_2DIR_sensor_block(&motor);
    // update_angle_el_zero_sensor_block(&motor);
    // update_angle_el_zero_no_sensor_block(&motor);
    // update_NLLUT_and_angle_el_zero_sensor_block(&motor);
    // }

    // 关节电机产品例程
    // switch(fsm_motor.state)
    // {
    //   case CALIBRATION:
    //   {
    //     __HAL_FDCAN_DISABLE_IT(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); 
    //     __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);      
    //     __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);

    //     SEGGER_RTT_printf(0,"Start calibration!\n");
    //     // update_2DIR_sensor_block(&motor);
    //     // update_angle_el_zero_sensor_block(&motor);
    //     // HAL_Delay(1000); 
    //     update_NLLUT_and_angle_el_zero_sensor_block(&motor);
    //     HAL_Delay(1000);    
    //     SEGGER_RTT_printf(0,"end calibration!\n");

    //     __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);         
    //     __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);      
    //     __HAL_FDCAN_ENABLE_IT(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); 
    //     fsm_motor.state = SLEEP;
    //   };break;
    //   case SET_ZERO:
    //   {
        
    //   };break;
    //   default:
    //   {

    //   };break;
    // }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    SEGGER_RTT_printf(0,"error!\n");
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
