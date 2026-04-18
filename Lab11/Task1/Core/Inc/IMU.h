#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float angle;
    float accel;
    float gyro;
} IMU_Angles_t;

int IMU_Init(void);
int IMU_Update(IMU_Angles_t *angles);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
