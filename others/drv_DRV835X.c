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

#include "SEGGER_RTT.h"

// DRV8353 寄存器地址宏定义（用户提供）
#define FSR1             0x0     /// Fault Status Register 1
#define FSR2             0x1     /// Fault Status Register 2
#define DCR              0x2     /// Drive Control Register
#define HSR              0x3     /// Gate Drive HS Register 
#define LSR              0x4     /// Gate Drive LS Register  
#define OCPCR            0x5     /// OCP Control Register    
#define CSACR            0x6     /// CSA Control Register    
#define DFGCR            0x7     /// Driver Configuration Register

/**
 * @brief  检查DRV8353寄存器状态并打印信息
 * @param  reg_type: 寄存器类型（宏定义：FSR1/FSR2/DCR等）
 * @param  reg_data: 从寄存器读回的16位数据
 * @retval int: 0=无异常, 1=有异常, -1=寄存器类型无效
 */
int check_drv(uint8_t reg_type, uint16_t reg_data)
{
    int ret = 0; // Default: No fault
    SEGGER_RTT_printf(0, "\r\n===== DRV8353 Register Check: 0x%02X =====\r\n", reg_type);
    SEGGER_RTT_printf(0, "Raw Register Data: 0x%04X\r\n", reg_data);

    switch(reg_type)
    {
        // -------------------------- Fault Status Register 1 (FSR1: 0x0) --------------------------
        case FSR1:
            SEGGER_RTT_printf(0, "[FSR1 - Fault Status Register 1]\r\n");
            // Bit 0: VDS Over-Current Fault
            if(reg_data & (1U << 0))  { SEGGER_RTT_printf(0, "  ❌ VDS_OCP: VDS Over-Current Fault Detected\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ VDS_OCP: No Fault\r\n"); }
            
            // Bit 1: Gate Drive Fault (GDF)
            if(reg_data & (1U << 1))  { SEGGER_RTT_printf(0, "  ❌ GDF: Gate Drive Fault Detected\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ GDF: No Fault\r\n"); }
            
            // Bit 2: Under-Voltage Lockout (UVLO)
            if(reg_data & (1U << 2))  { SEGGER_RTT_printf(0, "  ❌ UVLO: Under-Voltage Lockout Fault\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ UVLO: No Fault\r\n"); }
            
            // Bit 3: Over-Temperature Shutdown (OTSD)
            if(reg_data & (1U << 3))  { SEGGER_RTT_printf(0, "  ❌ OTSD: Over-Temperature Shutdown Fault\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ OTSD: No Fault\r\n"); }
            
            // Bit 4: Over-Temperature Warning (OTW)
            if(reg_data & (1U << 4))  { SEGGER_RTT_printf(0, "  ❌ OTW: Over-Temperature Warning\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ OTW: No Warning\r\n"); }
            
            // Bit 5: General Fault Flag
            if(reg_data & (1U << 5))  { SEGGER_RTT_printf(0, "  ❌ FAULT: General Fault Active\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ FAULT: No General Fault\r\n"); }
            
            // Bits 6-15: Reserved (No parsing)
            break;

        // -------------------------- Fault Status Register 2 (FSR2: 0x1) --------------------------
        case FSR2:
            SEGGER_RTT_printf(0, "[FSR2 - Fault Status Register 2]\r\n");
            // Bit 0: High-Side A VDS Over-Current
            if(reg_data & (1U << 0))  { SEGGER_RTT_printf(0, "  ❌ VDS_HA: HS Phase A VDS Over-Current\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ VDS_HA: No Fault\r\n"); }
            
            // Bit 1: Low-Side A VDS Over-Current
            if(reg_data & (1U << 1))  { SEGGER_RTT_printf(0, "  ❌ VDS_LA: LS Phase A VDS Over-Current\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ VDS_LA: No Fault\r\n"); }
            
            // Bit 2: High-Side B VDS Over-Current
            if(reg_data & (1U << 2))  { SEGGER_RTT_printf(0, "  ❌ VDS_HB: HS Phase B VDS Over-Current\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ VDS_HB: No Fault\r\n"); }
            
            // Bit 3: Low-Side B VDS Over-Current
            if(reg_data & (1U << 3))  { SEGGER_RTT_printf(0, "  ❌ VDS_LB: LS Phase B VDS Over-Current\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ VDS_LB: No Fault\r\n"); }
            
            // Bit 4: High-Side C VDS Over-Current
            if(reg_data & (1U << 4))  { SEGGER_RTT_printf(0, "  ❌ VDS_HC: HS Phase C VDS Over-Current\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ VDS_HC: No Fault\r\n"); }
            
            // Bit 5: Low-Side C VDS Over-Current
            if(reg_data & (1U << 5))  { SEGGER_RTT_printf(0, "  ❌ VDS_LC: LS Phase C VDS Over-Current\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ VDS_LC: No Fault\r\n"); }
            
            // Bit 6: CSA Over-Current (CSA_OCP)
            if(reg_data & (1U << 6))  { SEGGER_RTT_printf(0, "  ❌ CSA_OCP: Current Sense Amp Over-Current\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ CSA_OCP: No Fault\r\n"); }
            
            // Bit 7: PWM Input Fault
            if(reg_data & (1U << 7))  { SEGGER_RTT_printf(0, "  ❌ PWM_FAULT: PWM Input Fault Detected\r\n"); ret = 1; }
            else                     { SEGGER_RTT_printf(0, "  ✅ PWM_FAULT: No Fault\r\n"); }
            
            // Bits 8-15: Reserved (No parsing)
            break;

        // -------------------------- Drive Control Register (DCR: 0x2) --------------------------
        case DCR:
            SEGGER_RTT_printf(0, "[DCR - Drive Control Register]\r\n");
            // Bits 0-1: PWM Mode Selection
            uint8_t pwm_mode = (reg_data >> 0) & 0x03;
            const char* pwm_mode_str[] = {"6-PWM Mode", "3-PWM Mode", "Independent PWM Mode", "Reserved"};
            SEGGER_RTT_printf(0, "  PWM Mode: %s\r\n", pwm_mode_str[(pwm_mode <= 3) ? pwm_mode : 3]);
            
            // Bit 2: Sleep Mode Enable
            if(reg_data & (1U << 2))  { SEGGER_RTT_printf(0, "  Sleep Mode: Enabled (Low Power)\r\n"); }
            else                     { SEGGER_RTT_printf(0, "  Sleep Mode: Disabled (Normal Operation)\r\n"); }
            
            // Bit 3: Fault Clear Trigger
            if(reg_data & (1U << 3))  { SEGGER_RTT_printf(0, "  Fault Clear: Triggered (Clear Fault Flags)\r\n"); }
            else                     { SEGGER_RTT_printf(0, "  Fault Clear: Not Triggered\r\n"); }
            
            // Bits 4-5: Gate Drive Voltage Setting
            uint8_t gate_volt = (reg_data >> 4) & 0x03;
            SEGGER_RTT_printf(0, "  Gate Drive Voltage: ");
            switch(gate_volt)
            {
                case 0: SEGGER_RTT_printf(0, "8V\r\n"); break;
                case 1: SEGGER_RTT_printf(0, "10V\r\n"); break;
                case 2: SEGGER_RTT_printf(0, "12V\r\n"); break;
                case 3: SEGGER_RTT_printf(0, "15V\r\n"); break;
            }
            
            // Bits 6-15: Reserved (No fault check)
            break;

        // -------------------------- Gate Drive HS Register (HSR: 0x3) --------------------------
        case HSR:
            SEGGER_RTT_printf(0, "[HSR - Gate Drive High-Side Register]\r\n");
            // Bits 0-1: HS Phase A Drive Current
            uint8_t hs_a = (reg_data >> 0) & 0x03;
            // Bits 2-3: HS Phase B Drive Current
            uint8_t hs_b = (reg_data >> 2) & 0x03;
            // Bits 4-5: HS Phase C Drive Current
            uint8_t hs_c = (reg_data >> 4) & 0x03;
            
            SEGGER_RTT_printf(0, "  HS Phase A Drive Current: ");
            hs_a == 0 ? SEGGER_RTT_printf(0, "100mA\r\n") :
            hs_a == 1 ? SEGGER_RTT_printf(0, "200mA\r\n") :
            hs_a == 2 ? SEGGER_RTT_printf(0, "400mA\r\n") :
                        SEGGER_RTT_printf(0, "600mA\r\n");
            
            SEGGER_RTT_printf(0, "  HS Phase B Drive Current: ");
            hs_b == 0 ? SEGGER_RTT_printf(0, "100mA\r\n") :
            hs_b == 1 ? SEGGER_RTT_printf(0, "200mA\r\n") :
            hs_b == 2 ? SEGGER_RTT_printf(0, "400mA\r\n") :
                        SEGGER_RTT_printf(0, "600mA\r\n");
            
            SEGGER_RTT_printf(0, "  HS Phase C Drive Current: ");
            hs_c == 0 ? SEGGER_RTT_printf(0, "100mA\r\n") :
            hs_c == 1 ? SEGGER_RTT_printf(0, "200mA\r\n") :
            hs_c == 2 ? SEGGER_RTT_printf(0, "400mA\r\n") :
                        SEGGER_RTT_printf(0, "600mA\r\n");
            
            // Bits 6-15: Reserved (No fault check)
            break;

        // -------------------------- Gate Drive LS Register (LSR: 0x4) --------------------------
        case LSR:
            SEGGER_RTT_printf(0, "[LSR - Gate Drive Low-Side Register]\r\n");
            // Bits 0-1: LS Phase A Drive Current
            uint8_t ls_a = (reg_data >> 0) & 0x03;
            // Bits 2-3: LS Phase B Drive Current
            uint8_t ls_b = (reg_data >> 2) & 0x03;
            // Bits 4-5: LS Phase C Drive Current
            uint8_t ls_c = (reg_data >> 4) & 0x03;
            
            SEGGER_RTT_printf(0, "  LS Phase A Drive Current: ");
            ls_a == 0 ? SEGGER_RTT_printf(0, "100mA\r\n") :
            ls_a == 1 ? SEGGER_RTT_printf(0, "200mA\r\n") :
            ls_a == 2 ? SEGGER_RTT_printf(0, "400mA\r\n") :
                        SEGGER_RTT_printf(0, "600mA\r\n");
            
            SEGGER_RTT_printf(0, "  LS Phase B Drive Current: ");
            ls_b == 0 ? SEGGER_RTT_printf(0, "100mA\r\n") :
            ls_b == 1 ? SEGGER_RTT_printf(0, "200mA\r\n") :
            ls_b == 2 ? SEGGER_RTT_printf(0, "400mA\r\n") :
                        SEGGER_RTT_printf(0, "600mA\r\n");
            
            SEGGER_RTT_printf(0, "  LS Phase C Drive Current: ");
            ls_c == 0 ? SEGGER_RTT_printf(0, "100mA\r\n") :
            ls_c == 1 ? SEGGER_RTT_printf(0, "200mA\r\n") :
            ls_c == 2 ? SEGGER_RTT_printf(0, "400mA\r\n") :
                        SEGGER_RTT_printf(0, "600mA\r\n");
            
            // Bits 6-15: Reserved (No fault check)
            break;

        // -------------------------- OCP Control Register (OCPCR: 0x5) --------------------------
        case OCPCR:
            SEGGER_RTT_printf(0, "[OCPCR - OCP Control Register]\r\n");
            // Bits 0-2: VDS OCP Threshold
            uint8_t vds_ocp_th = (reg_data >> 0) & 0x07;
            SEGGER_RTT_printf(0, "  VDS OCP Threshold: %d x 0.1V (Threshold = %0.1fV)\r\n", vds_ocp_th, (float)vds_ocp_th * 0.1f);
            
            // Bit 3: OCP Response Mode
            if(reg_data & (1U << 3))  { SEGGER_RTT_printf(0, "  OCP Response Mode: Auto-Retry (Restart After Fault)\r\n"); }
            else                     { SEGGER_RTT_printf(0, "  OCP Response Mode: Latched Shutdown (Manual Fault Clear Required)\r\n"); }
            
            // Bits 4-5: OCP Detection Time
            uint8_t ocp_detect = (reg_data >> 4) & 0x03;
            const char* ocp_time_str[] = {"1us", "2us", "4us", "8us"};
            SEGGER_RTT_printf(0, "  OCP Detection Time: %s\r\n", ocp_time_str[(ocp_detect <= 3) ? ocp_detect : 0]);
            
            // Bits 6-15: Reserved (No fault check)
            break;

        // -------------------------- CSA Control Register (CSACR: 0x6) --------------------------
        case CSACR:
            SEGGER_RTT_printf(0, "[CSACR - Current Sense Amplifier Control Register]\r\n");
            // Bits 0-2: CSA Gain Setting
            uint8_t csa_gain = (reg_data >> 0) & 0x07;
            const char* gain_str[] = {"5x", "10x", "20x", "40x", "80x", "Reserved", "Reserved", "Reserved"};
            SEGGER_RTT_printf(0, "  CSA Gain: %s\r\n", gain_str[(csa_gain <= 7) ? csa_gain : 5]);
            
            // Bit 3: CSA Amplifier Enable
            if(reg_data & (1U << 3))  { SEGGER_RTT_printf(0, "  CSA Amplifier: Disabled\r\n"); }
            else                     { SEGGER_RTT_printf(0, "  CSA Amplifier: Enabled\r\n"); }
            
            // Bits 4-6: CSA OCP Threshold
            uint8_t csa_ocp_th = (reg_data >> 4) & 0x07;
            SEGGER_RTT_printf(0, "  CSA OCP Threshold: %d x 0.05V (Threshold = %0.2fV)\r\n", csa_ocp_th, (float)csa_ocp_th * 0.05f);
            
            // Bit 7: CSA Calibration (Auto-Zero)
            if(reg_data & (1U << 7))  { SEGGER_RTT_printf(0, "  CSA Calibration: Enabled (Auto-Zero)\r\n"); }
            else                     { SEGGER_RTT_printf(0, "  CSA Calibration: Disabled\r\n"); }
            
            // Bits 8-15: Reserved (No fault check)
            break;

        // -------------------------- Driver Configuration Register (DFGCR: 0x7) --------------------------
        case DFGCR:
            SEGGER_RTT_printf(0, "[DFGCR - Driver Configuration Register]\r\n");
            // Bits 0-1: Dead Time Setting
            uint8_t dead_time = (reg_data >> 0) & 0x03;
            const char* dt_str[] = {"50ns", "100ns", "200ns", "400ns"};
            SEGGER_RTT_printf(0, "  Dead Time: %s\r\n", dt_str[(dead_time <= 3) ? dead_time : 0]);
            
            // Bits 2-3: PWM Divider Ratio
            uint8_t pwm_div = (reg_data >> 2) & 0x03;
            SEGGER_RTT_printf(0, "  PWM Divider Ratio: %d (Input PWM / %d)\r\n", pwm_div + 1, pwm_div + 1);
            
            // Bit 4: Current Sense Mode
            if(reg_data & (1U << 4))  { SEGGER_RTT_printf(0, "  Current Sense Mode: 3-Resistor Mode\r\n"); }
            else                     { SEGGER_RTT_printf(0, "  Current Sense Mode: 1-Resistor Mode\r\n"); }
            
            // Bits 5-15: Reserved (No fault check)
            break;

        // -------------------------- Invalid Register Type --------------------------
        default:
            SEGGER_RTT_printf(0, "[ERROR] Invalid Register Type: 0x%02X\r\n", reg_type);
            ret = -1;
            break;
    }

    // Print Final Status Summary
    if(ret == 1)      { SEGGER_RTT_printf(0, "\r\n❌ DRV8353: Fault Detected!\r\n"); }
    else if(ret == 0) { SEGGER_RTT_printf(0, "\r\n✅ DRV8353: Status Normal\r\n"); }
    SEGGER_RTT_printf(0, "===========================================\r\n");

    return ret;
}
void DRV835X_updateCfgPara( void )
{
    uint16_t data;

    stru_DRV8353Obj.drvCtrl_obj.data = read_reg( DCR );
    // check_drv(DCR,stru_DRV8353Obj.drvCtrl_obj.data);
    stru_DRV8353Obj.drvCsa_obj.data = read_reg( CSACR );
    // check_drv(CSACR,stru_DRV8353Obj.drvCsa_obj.data);
    stru_DRV8353Obj.drvCfg_obj.data = read_reg( DFGCR );
    // check_drv(DFGCR,stru_DRV8353Obj.drvCfg_obj.data);

    stru_DRV8353Obj.drvGateHS_obj.data = read_reg( HSR );
    // check_drv(HSR,stru_DRV8353Obj.drvGateHS_obj.data);
    stru_DRV8353Obj.drvGateLS_obj.data = read_reg( LSR );
    // check_drv(LSR,stru_DRV8353Obj.drvGateLS_obj.data);
    stru_DRV8353Obj.drvOcp_obj.data = read_reg( OCPCR );
    // check_drv(OCPCR,stru_DRV8353Obj.drvOcp_obj.data);

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