#include "pid.h"
#include "stm32f3xx_hal.h"   
#include <math.h>           

void PID_Init(PID_Controller *pid)
{
    pid->kp = PID_KP;
    pid->ki = PID_KI;
    pid->kd = PID_KD;

    pid->setpoint    = PID_SETPOINT;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;

    pid->output_max   =  PID_OUTPUT_MAX;
    pid->output_min   =  PID_OUTPUT_MIN;
    pid->integral_max =  PID_INTEGRAL_MAX;
    pid->integral_min =  PID_INTEGRAL_MIN;

    pid->dt = PID_DT;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Compute(PID_Controller *pid, float measured_angle)
{
    float error = pid->setpoint - measured_angle;

    float p_term = pid->kp * error;

    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    float i_term = pid->ki * pid->integral;

    float derivative = (error - pid->prev_error) / pid->dt;
    float d_term     = pid->kd * derivative;
    pid->prev_error  = error;

    float output = p_term + i_term + d_term;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

static void _set_motor(TIM_HandleTypeDef *htim, uint32_t channel,
                       GPIO_TypeDef *fwd_port, uint16_t fwd_pin,
                       GPIO_TypeDef *bwd_port, uint16_t bwd_pin,
                       float duty)          /* 0 – PID_OUTPUT_MAX */
{
    uint32_t pwm_val = (uint32_t)fabsf(duty);

    if (duty > 0.0f) {
        HAL_GPIO_WritePin(fwd_port, fwd_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(bwd_port, bwd_pin, GPIO_PIN_RESET);
    } else if (duty < 0.0f) {
        HAL_GPIO_WritePin(fwd_port, fwd_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(bwd_port, bwd_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(fwd_port, fwd_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(bwd_port, bwd_pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(htim, channel, pwm_val);
}

void PID_ApplyMotors(float output,
                     TIM_HandleTypeDef *htim,
                     uint32_t channelA, uint32_t channelB,
                     GPIO_TypeDef *in1_port, uint16_t in1_pin,
                     GPIO_TypeDef *in2_port, uint16_t in2_pin,
                     GPIO_TypeDef *in3_port, uint16_t in3_pin,
                     GPIO_TypeDef *in4_port, uint16_t in4_pin)
{

    _set_motor(htim, channelA,
               in1_port, in1_pin, in2_port, in2_pin,  output);

    _set_motor(htim, channelB,
               in3_port, in3_pin, in4_port, in4_pin,  output);
}

void PID_StopMotors(TIM_HandleTypeDef *htim,
                    uint32_t channelA, uint32_t channelB,
                    GPIO_TypeDef *in1_port, uint16_t in1_pin,
                    GPIO_TypeDef *in2_port, uint16_t in2_pin,
                    GPIO_TypeDef *in3_port, uint16_t in3_pin,
                    GPIO_TypeDef *in4_port, uint16_t in4_pin)
{
    _set_motor(htim, channelA, in1_port, in1_pin, in2_port, in2_pin, 0.0f);
    _set_motor(htim, channelB, in3_port, in3_pin, in4_port, in4_pin, 0.0f);
}
