/* ---------------------------------------------------------------
 *  pid.c  –  PID Controller for Self-Balancing Robot (STM32 HAL)
 * --------------------------------------------------------------- */

#include "pid.h"
#include "stm32f3xx_hal.h"   /* replace xxxx with your STM32 series  */
#include <math.h>            /* fabsf()                               */
#include "main.h"

/* ---------------------------------------------------------------
 *  PID_Init
 * --------------------------------------------------------------- */
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

    pid->dt = DT_S;
}

/* ---------------------------------------------------------------
 *  PID_Reset
 * --------------------------------------------------------------- */
void PID_Reset(PID_Controller *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

/* ---------------------------------------------------------------
 *  PID_Compute
 * --------------------------------------------------------------- */
float PID_Compute(PID_Controller *pid, float measured_angle)
{
    /* 1. Error */
    float error = pid->setpoint - measured_angle;

    /* 2. Proportional */
    float p_term = pid->kp * error;

    /* 3. Integral with anti-windup clamp */
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    float i_term = pid->ki * pid->integral;

    /* 4. Derivative (on measurement to avoid derivative kick on setpoint change) */
    float derivative = (error - pid->prev_error) / pid->dt;
    float d_term     = pid->kd * derivative;
    pid->prev_error  = error;

    /* 5. Sum & clamp output */
    float output = p_term + i_term + d_term;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

/* ---------------------------------------------------------------
 *  Internal helper – set one motor's direction + PWM duty
 *
 *  forward_port/pin  → HIGH when spinning "forward"
 *  backward_port/pin → HIGH when spinning "backward"
 * --------------------------------------------------------------- */
static void _set_motor(TIM_HandleTypeDef *htim, uint32_t channel,
                       GPIO_TypeDef *fwd_port, uint16_t fwd_pin,
                       GPIO_TypeDef *bwd_port, uint16_t bwd_pin,
                       float duty)          /* 0 – PID_OUTPUT_MAX */
{
    uint32_t pwm_val = (uint32_t)fabsf(duty);

    if (duty > 0.0f) {
        /* Forward */
        HAL_GPIO_WritePin(fwd_port, fwd_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(bwd_port, bwd_pin, GPIO_PIN_RESET);
    } else if (duty < 0.0f) {
        /* Backward */
        HAL_GPIO_WritePin(fwd_port, fwd_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(bwd_port, bwd_pin, GPIO_PIN_SET);
    } else {
        /* Brake (both LOW → coast; both HIGH → brake depending on driver) */
        HAL_GPIO_WritePin(fwd_port, fwd_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(bwd_port, bwd_pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(htim, channel, pwm_val);
}

/* ---------------------------------------------------------------
 *  PID_ApplyMotors
 * --------------------------------------------------------------- */
void PID_ApplyMotors(float output,
                     TIM_HandleTypeDef *htim,
                     uint32_t channelA, uint32_t channelB,
                     GPIO_TypeDef *in1_port, uint16_t in1_pin,
                     GPIO_TypeDef *in2_port, uint16_t in2_pin,
                     GPIO_TypeDef *in3_port, uint16_t in3_pin,
                     GPIO_TypeDef *in4_port, uint16_t in4_pin)
{
    /* Both motors receive the same output (tank-style drive).
       Flip the sign for one motor if they are physically mirrored. */
    _set_motor(htim, channelA,
               in1_port, in1_pin, in2_port, in2_pin,  output);

    _set_motor(htim, channelB,
               in3_port, in3_pin, in4_port, in4_pin,  output);
               /* use -output here if motor B is mirrored */
}

/* ---------------------------------------------------------------
 *  PID_StopMotors
 * --------------------------------------------------------------- */
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