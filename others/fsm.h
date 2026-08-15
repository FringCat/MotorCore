#ifndef FSM_H_
#define FSM_H_
#include "main.h"
#include "foc_alg.h"
#include "can_handler.h"
#include "fdcan.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#define RUN 1
#define STOP 2
#define CALIBRATION 3
#define SET_ZERO 4
#define ERROR 5
#define SLEEP 6

typedef struct
{
    uint32_t state;
    uint32_t flag_block;
    float timeout ;
}fsm_HandleTypeDef;

extern Motor_HandleTypeDef motor;
extern CAN_Handler_t can_handler;
extern fsm_HandleTypeDef fsm_motor;
extern int isoffset_done;

void fsm_init(fsm_HandleTypeDef *fsm);
void fsm_run(void);

#endif 
