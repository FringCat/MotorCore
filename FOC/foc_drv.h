#ifndef __FOC_DRV_H
#define __FOC_DRV_H

#include "foc_alg.h"

#define A_PWM_Period 4250
#define B_PWM_Period 4250
#define C_PWM_Period 4250
#define I_ADC_CONV    0.00080586f/20.0f/0.002f
#define U_ADC_CONV    0.00080586

void foc_init(Motor_HandleTypeDef *motor);
#endif
