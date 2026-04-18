#ifndef PID_H
#define PID_H
#include "main.h"
#include <stdint.h>

/* ---------------------------------------------------------------
 *  PID Controller for Self-Balancing Robot
 *  Target: STM32 (HAL-based PWM output)
 * --------------------------------------------------------------- */

/* Tune these to your robot */
#define PID_KP              50.0f
#define PID_KI               0.8f
#define PID_KD               1.2f

#define PID_SETPOINT         0.0f   /* desired balance angle (degrees) */

#define PID_OUTPUT_MAX     999.0f   /* match your TIM ARR (100% duty)  */
#define PID_OUTPUT_MIN    -999.0f
#define PID_INTEGRAL_MAX   400.0f   /* anti-windup clamp               */
#define PID_INTEGRAL_MIN  -400.0f
#define DT_S               0.005f

/* ---------------------------------------------------------------
 *  PID state structure  –  one instance per controller
 * --------------------------------------------------------------- */
typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;         /* target angle                  */
    float integral;         /* accumulated integral term     */
    float prev_error;       /* for derivative calculation    */

    float output_max;
    float output_min;
    float integral_max;
    float integral_min;

    float dt;               /* sample time in seconds        */
} PID_Controller;

/* ---------------------------------------------------------------
 *  Public API
 * --------------------------------------------------------------- */

/**
 * @brief  Initialise (or re-initialise) the PID controller.
 * @param  pid       Pointer to PID_Controller instance
 * @param  kp/ki/kd  Gains
 * @param  setpoint  Desired balance angle (degrees)
 * @param  dt        Sample period in seconds (e.g. 0.005 for 200 Hz)
 */
void PID_Init(PID_Controller *pid);

/**
 * @brief  Compute one PID step.
 * @param  pid            Pointer to PID_Controller instance
 * @param  measured_angle Current angle from IMU (degrees)
 * @return Signed output value in [-OUTPUT_MAX, +OUTPUT_MAX].
 *         Positive → tilt forward  |  Negative → tilt backward
 */
float PID_Compute(PID_Controller *pid, float measured_angle);

/**
 * @brief  Reset integral and derivative history (call on enable / re-arm).
 */
void PID_Reset(PID_Controller *pid);

/**
 * @brief  Apply PID output to both motors via STM32 HAL PWM.
 *
 *         Motor direction is controlled by two GPIO pins per motor
 *         (IN1/IN2 style driver such as L298N / TB6612).
 *
 * @param  output       Value from PID_Compute()
 * @param  htim         Pointer to TIM handle used for PWM
 * @param  channelA     TIM_CHANNEL_x for motor A
 * @param  channelB     TIM_CHANNEL_x for motor B
 * @param  in1_port / in1_pin   Direction GPIO for motor A forward
 * @param  in2_port / in2_pin   Direction GPIO for motor A backward
 * @param  in3_port / in3_pin   Direction GPIO for motor B forward
 * @param  in4_port / in4_pin   Direction GPIO for motor B backward
 */
void PID_ApplyMotors(float output,
                     TIM_HandleTypeDef *htim,
                     uint32_t channelA, uint32_t channelB,
                     GPIO_TypeDef *in1_port, uint16_t in1_pin,
                     GPIO_TypeDef *in2_port, uint16_t in2_pin,
                     GPIO_TypeDef *in3_port, uint16_t in3_pin,
                     GPIO_TypeDef *in4_port, uint16_t in4_pin);

/**
 * @brief  Stop both motors immediately (brake / coast).
 */
void PID_StopMotors(TIM_HandleTypeDef *htim,
                    uint32_t channelA, uint32_t channelB,
                    GPIO_TypeDef *in1_port, uint16_t in1_pin,
                    GPIO_TypeDef *in2_port, uint16_t in2_pin,
                    GPIO_TypeDef *in3_port, uint16_t in3_pin,
                    GPIO_TypeDef *in4_port, uint16_t in4_pin);

#endif /* PID_H */