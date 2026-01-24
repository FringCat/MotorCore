/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          :  drv_DRV835X.c
 * Description        :  DRV835X driver 
 ******************************************************************************
 * @attention
 *
* COPYRIGHT:    Copyright (c) 2025

* CREATED BY:   ming fei.tang
* DATE:         January 04th, 2025

 ******************************************************************************
 */
/* USER CODE END Header */
#include "drv_DRV835X.h"
#include "SEGGER_RTT.h"
/* Private macro -------------------------------------------------------------*/
extern SPI_HandleTypeDef        hspi1;

/* Private define ------------------------------------------------------------*/
#define TIME_OUT                0XFFFF
#define DEFAULT_GAIN            10
#define DRV835X_SPI_Handle      hspi1

// DRV8353 SPI CS PIN 
#define DRV835X_CS_EN           HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,GPIO_PIN_RESET)
#define DRV835X_CS_DIS          HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,GPIO_PIN_SET)

// DRV8353 ENABLE PIN 
/*
    Gate driver enable. When this pin is logic low the device goes to a low power sleep mode. 
    An 8 to 40-µs low pulse can be used to reset fault conditions.
*/
#define DRV835X_ENABLE_LOW      HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,GPIO_PIN_RESET)
#define DRV835X_ENABLE_HIGH     HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,GPIO_PIN_SET)

// DRV8353 PWML PIN: INLA INLB INLC 
/*
   Low-side gate driver control input. This pin controls the output of the low-side gate driver.
*/
#define DRV835X_PWML_LOW        HAL_GPIO_WritePin(PWML_GPIO_Port,PWML_Pin,GPIO_PIN_RESET)
#define DRV835X_PWML_HIGH       HAL_GPIO_WritePin(PWML_GPIO_Port,PWML_Pin,GPIO_PIN_SET)

/* Private variables ---------------------------------------------------------*/
Stru_DRV835X_Status stru_DRV835X_Status;
Stru_DRV835X stru_DRV8353Obj;

StruDRV835XCfgPara stru_config = 
{
    // Driver Control Register (address = 0x02h)
   .PWM_MODE = PWM_MODE_3X,
    
    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
   .SEN_LVL = SEN_LVL_1_0,   //  00b = Sense OCP 0.25 V
   .CSA_GAIN = CSA_GAIN_20,   //  01b = 10-V/V shunt amplifier gain
   .VREF_DIV = VREF_DIV_2,    //  1b = Sense amplifier reference voltage is VREF divided by 2
    
    // OCP Control Register (address = 0x05h)
   .VDS_LVL =  VDS_LVL_1_88,
   .OCP_DEG =  OCP_DEG_6US,
   .OCP_MODE = OCP_REPORT,
   .DEAD_TIME = DEADTIME_200NS,
    
   //Gate Drive HS Register (address = 0x03h)
   .IDRIVEP_HS = IDRIVEP_HS_1000MA,
   .IDRIVEN_HS = IDRIVEN_HS_2000MA,
   .LOCK = LOCK_OFF,
    
   // Gate Drive LS Register (address = 0x04h) 
   .IDRIVEN_LS = IDRIVEN_LS_2000MA,
   .IDRIVEP_LS = IDRIVEP_LS_1000MA,
   .TDRIVE = TDRIVE_4000NS,
   .CBC = PWM_GIVER_ENABLE,  // 1b = For VDS_OCP and SEN_OCP, the fault is cleared when
                             // a new PWM input is given or after tRETRY
};

/* Private function prototypes -----------------------------------------------*/
static uint16_t read_reg(uint16_t address);
static uint16_t write_reg(uint16_t address, uint16_t data);


void DRV835X_updateCfgPara( void )
{
    uint16_t data;

    stru_DRV8353Obj.drvCtrl_obj.data = read_reg( DCR );
    stru_DRV8353Obj.drvCsa_obj.data = read_reg( CSACR );
    stru_DRV8353Obj.drvCfg_obj.data = read_reg( DFGCR );

    stru_DRV8353Obj.drvGateHS_obj.data = read_reg( HSR );
    stru_DRV8353Obj.drvGateLS_obj.data = read_reg( LSR );
    stru_DRV8353Obj.drvOcp_obj.data = read_reg( OCPCR );

    stru_DRV8353Obj.faultStatusReg1_obj.data = read_reg( FSR1 );
    stru_DRV8353Obj.faultStatusReg2_obj.data = read_reg( FSR2 );
    // SEGGER_RTT_printf(0, "stru_DRV8353Obj.drvCsa_obj.data = 0x%04X\n\r", stru_DRV8353Obj.drvCsa_obj.data);
    // Driver Control Register (address = 0x02h)
    stru_DRV8353Obj.drvCtrl_obj.ctrlRegObj.PWM_MODE  = stru_config.PWM_MODE;
    data = stru_DRV8353Obj.drvCtrl_obj.data;
    write_reg( DCR, data);

    //Gate Drive HS Register (address = 0x03h)
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.IDRIVEP_HS = stru_config.IDRIVEP_HS;
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.IDRIVEN_HS = stru_config.IDRIVEN_HS;
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.LOCK = stru_config.LOCK;
    data = stru_DRV8353Obj.drvGateHS_obj.data;
    write_reg( HSR, data);

    // Gate Drive LS Register (address = 0x04h) 
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.IDRIVEN_LS = stru_config.IDRIVEN_LS;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.IDRIVEP_LS = stru_config.IDRIVEP_LS;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.TDRIVE = stru_config.TDRIVE;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.CBC = stru_config.CBC;
    data = stru_DRV8353Obj.drvGateLS_obj.data;
    write_reg( LSR, data);

    // OCP Control Register (address = 0x05h)
    stru_DRV8353Obj.drvOcp_obj.ocpObj.VDS_LVL =  stru_config.VDS_LVL;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.OCP_DEG =  stru_config.OCP_DEG;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.OCP_MODE = stru_config.OCP_MODE;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.DEAD_TIME = stru_config.DEAD_TIME;
    data = stru_DRV8353Obj.drvOcp_obj.data;
    write_reg( OCPCR, data);

    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
    stru_DRV8353Obj.drvCsa_obj.csaObj.SEN_LVL  = stru_config.SEN_LVL;
    stru_DRV8353Obj.drvCsa_obj.csaObj.CSA_GAIN = stru_config.CSA_GAIN;
    stru_DRV8353Obj.drvCsa_obj.csaObj.VREF_DIV = stru_config.VREF_DIV;
    data = stru_DRV8353Obj.drvCsa_obj.data;
    write_reg( CSACR, data);
    // SEGGER_RTT_printf(0, "data = 0x%04X\n\r", data); 
    // SEGGER_RTT_printf(0, "stru_DRV8353Obj.drvCsa_obj.data = 0x%04X\n\r", read_reg( CSACR ));
}


void DRV835X_Init( void )
{
    DRV835X_ENABLE_LOW;
    HAL_Delay(100);
    DRV835X_ENABLE_HIGH;
    HAL_Delay(100);
    
    // SET PWML to low
    DRV835X_PWML_LOW;
    HAL_Delay(200);
    
    // SEGGER_RTT_printf(0, "DRV835X Init: Updating configuration parameters...\n\r");
    DRV835X_updateCfgPara();
    // SEGGER_RTT_printf(0, "DRV835X Init: Configuration parameters updated.\n\r");
    DRV835X_PWML_HIGH;
    DRV835X_ENABLE_HIGH;
    HAL_Delay(200);
    // DRV835X_PWML_LOW;
    // DRV835X_ENABLE_LOW;
}



void DRV835X_read_FaultStatusReg1(void)
{
    stru_DRV8353Obj.faultStatusReg1_obj.data = read_reg( FSR1 );
}

void DRV835X_read_FaultStatusReg2(void)
{
    stru_DRV8353Obj.faultStatusReg2_obj.data = read_reg( FSR2 );
}


// static uint16_t read_reg(uint16_t address)
// {
//     uint16_t data;
//     Input_WrReg stru_Input_WrRegObj;
    
//     stru_Input_WrRegObj.inputRegObj.WR =  R_MODE;
//     stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
//     data = stru_Input_WrRegObj.data;
    
//     DRV835X_CS_EN;
//     HAL_SPI_Transmit(&DRV835X_SPI_Handle, (uint8_t *)&data, 1,TIME_OUT);
//     DRV835X_CS_DIS;
//     HAL_Delay(1);
    
//     DRV835X_CS_EN;
//     HAL_SPI_Receive(&DRV835X_SPI_Handle, (uint8_t *)&data, 1, TIME_OUT);
//     DRV835X_CS_DIS;
//     HAL_Delay(1);

//     return (data & 0x7FF);
// }

// static uint16_t write_reg(uint16_t address, uint16_t data)
// {
//     Input_WrReg stru_Input_WrRegObj;
    
//     stru_Input_WrRegObj.inputRegObj.WR =  W_MODE;
//     stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
//     stru_Input_WrRegObj.inputRegObj.DATA = data;
    
//     data = stru_Input_WrRegObj.data;
//     do
//     {
//         DRV835X_CS_EN;
//         HAL_SPI_Transmit(&DRV835X_SPI_Handle, (uint8_t *)&data, 1, TIME_OUT);
//         DRV835X_CS_DIS;
//         HAL_Delay(1);
//     }while (read_reg(address) != (data & 0x7FF));
    
//     return 0;
// }
static uint16_t write_reg(uint16_t address, uint16_t data)
{
    uint16_t tx_data;  // 发送缓冲区（存储写命令+地址+数据）
    uint16_t rx_data;  // 接收缓冲区（写操作可能无需使用，但函数要求必须提供）
    Input_WrReg stru_Input_WrRegObj;
    
    // 配置写命令、地址和要写入的数据
    stru_Input_WrRegObj.inputRegObj.WR = W_MODE;
    stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
    stru_Input_WrRegObj.inputRegObj.DATA = data;  // data为传入的要写入的寄存器值
    tx_data = stru_Input_WrRegObj.data;  // 整合为16位发送数据
    
    do
    {
        DRV835X_CS_EN;  // 使能片选
        // 一次SPI事务完成"发送写命令+地址+数据"，同时接收从机应答（rx_data可忽略）
        HAL_SPI_TransmitReceive(&DRV835X_SPI_Handle,
                               (uint8_t *)&tx_data,
                               (uint8_t *)&rx_data,
                               1,  // 保持原逻辑的传输长度（1个16位数据）
                               TIME_OUT);
        DRV835X_CS_DIS;  // 禁用片选
        HAL_Delay(1);    // 保留原延时，确保写入时序稳定
        
        // 循环校验：读取刚写入的地址，确认数据写入成功（read_reg已改为TransmitReceive版本）
    } while (read_reg(address) != (data));  // 对比写入的DATA部分（&0x7FF与原逻辑一致）
    
    return 0;
}

static uint16_t read_reg(uint16_t address)
{
    uint16_t tx_data;  // 发送缓冲区
    uint16_t rx_data;  // 接收缓冲区
    Input_WrReg stru_Input_WrRegObj;
    
    // 配置发送数据（包含读命令和地址）
    stru_Input_WrRegObj.inputRegObj.WR = R_MODE;
    stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
    tx_data = stru_Input_WrRegObj.data;  // 获取要发送的16位数据
    
    // 一次SPI事务中同时完成发送（命令+地址）和接收（数据）
    DRV835X_CS_EN;  // 使能片选
    // 发送tx_data，同时接收数据到rx_data，长度为1（保持原逻辑的长度定义）
    HAL_SPI_TransmitReceive(&DRV835X_SPI_Handle, 
                           (uint8_t *)&tx_data, 
                           (uint8_t *)&rx_data, 
                           1, 
                           TIME_OUT);
    DRV835X_CS_DIS;  // 禁用片选
    HAL_Delay(1);    // 保留延时，确保时序稳定

    return (rx_data & 0x7FF);  // 返回处理后的接收数据
}

/* End of this file */