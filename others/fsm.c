#include "fsm.h"

void fsm_init(fsm_HandleTypeDef *fsm)
{
    fsm->state = SLEEP;
    fsm->flag_block = 0;
    fsm->timeout = 0.0f;
}
void fsm_run(void)
{
    static float Kp = 0.0f;
    static float Kd = 0.0f;
    static float target_Velocity = 0.0f;
    static float target_Position =  0.0f;
    static float forward_torque_flange =  0.0f;
    static float output = 0;
    switch (fsm_motor.state)
    {
        case STOP:
        {

        }; break;
        case SLEEP:
        {
            can_reset_cmd(&can_handler);
            Kp = 0.0f;
            Kd = 0.0f;
            target_Velocity = 0.0f;
            target_Position =  0.0f;
            forward_torque_flange =  0.0f;
            fsm_motor.timeout = 0.0f;

            // if(motor.MotorAlg.Velocity_flange > 0.1f || motor.MotorAlg.Velocity_flange < -0.1f)
            // {
            //     Kd+= 0.000001f;  
            // }
            // else
            // {
            //     Kd = 0.1f;
            // }   
            motor.MotorAlg.angle_flange = Limit_angle_flange(motor.MotorData.angle_all,motor.MotorConfig.GR);
            motor.MotorAlg.Velocity_flange = motor.MotorAlg.Velocity/motor.MotorConfig.GR; //更新法兰速度

            output = (1/motor.MotorConfig.Kt)*(forward_torque_flange + Kp * (target_Position - motor.MotorAlg.angle_flange)  + Kd * (target_Velocity - motor.MotorAlg.Velocity_flange));//带减速箱
            motor.MotorAlg.Uq = Calculate_PID(output, motor.MotorAlg.Iq , motor.time.dt , &motor.MotorAlg.iq_pid);
            motor.MotorAlg.Ud = Calculate_PID(0.0f, motor.MotorAlg.Id , motor.time.dt , &motor.MotorAlg.id_pid);
            update_svpwm(&motor);  
        };break;
        case RUN:
        {
            if(fsm_motor.timeout > 0)
            {
                Kp = can_handler.cmd_kp;
                Kd = can_handler.cmd_kd;
                target_Velocity = can_handler.cmd_v_target;
                target_Position =  can_handler.cmd_p_target;
                forward_torque_flange =  can_handler.cmd_t_target;
                fsm_motor.timeout = fsm_motor.timeout - motor.time.dt ;

                motor.MotorAlg.angle_flange = Limit_angle_flange(motor.MotorData.angle_all,motor.MotorConfig.GR);
                motor.MotorAlg.Velocity_flange = motor.MotorAlg.Velocity/motor.MotorConfig.GR; //更新法兰速度
                // output = (forward_torque + Kp * (target_Position - motor.MotorAlg.angle)  + Kd * (target_Velocity - motor.MotorAlg.Velocity)); //无减速箱
                output = (1/motor.MotorConfig.Kt)*(forward_torque_flange + Kp * (target_Position - motor.MotorAlg.angle_flange)  + Kd * (target_Velocity - motor.MotorAlg.Velocity_flange));//带减速箱
                motor.MotorAlg.Uq = Calculate_PID(output, motor.MotorAlg.Iq , motor.time.dt , &motor.MotorAlg.iq_pid);
                motor.MotorAlg.Ud = Calculate_PID(0.0f, motor.MotorAlg.Id , motor.time.dt , &motor.MotorAlg.id_pid);
                update_svpwm(&motor);//输出SVPWM
            }
            else
            {
                fsm_motor.state = SLEEP;
            }
        }; break;
        case CALIBRATION:
        {
            // __HAL_FDCAN_DISABLE_IT(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); 
            // __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);      
            // __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);

            // SEGGER_RTT_printf(0,"Start calibration!\n");
            // // update_angle_el_zero_sensor_block(&motor);
            // HAL_Delay(1000);    
            // SEGGER_RTT_printf(0,"end calibration!\n");
            // __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);         
            // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);      
            // __HAL_FDCAN_ENABLE_IT(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); 
        };break;
        case SET_ZERO:
        {

        };break;
        case ERROR:
        {

        };break;
        default:
        {

        };break;
    }
}