#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <main.h>

#define PID_KP              50.0f
#define PID_KI               0.0f
#define PID_KD               0.2f

#define PID_SETPOINT         0.0f 

#define PID_OUTPUT_MAX     999.0f 
#define PID_OUTPUT_MIN    -999.0f
#define PID_INTEGRAL_MAX   400.0f 
#define PID_INTEGRAL_MIN  -400.0f
#define PID_DT              0.005f

typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;
    float integral;
    float prev_error;

    float output_max;
    float output_min;
    float integral_max;
    float integral_min;

    float dt;
} PID_Controller;

void PID_Init(PID_Controller *pid);
float PID_Compute(PID_Controller *pid, float measured_angle);
void PID_Reset(PID_Controller *pid);

void PID_ApplyMotors(float output,
                     TIM_HandleTypeDef *htim,
                     uint32_t channelA, uint32_t channelB,
                     GPIO_TypeDef *in1_port, uint16_t in1_pin,
                     GPIO_TypeDef *in2_port, uint16_t in2_pin,
                     GPIO_TypeDef *in3_port, uint16_t in3_pin,
                     GPIO_TypeDef *in4_port, uint16_t in4_pin);

void PID_StopMotors(TIM_HandleTypeDef *htim,
                    uint32_t channelA, uint32_t channelB,
                    GPIO_TypeDef *in1_port, uint16_t in1_pin,
                    GPIO_TypeDef *in2_port, uint16_t in2_pin,
                    GPIO_TypeDef *in3_port, uint16_t in3_pin,
                    GPIO_TypeDef *in4_port, uint16_t in4_pin);


#endif