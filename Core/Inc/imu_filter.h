#ifndef __IMU_FILTER_H
#define __IMU_FILTER_H

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

void Mahony_Init(float sample_freq_hz);
void Mahony_Update(Attitude_t *att,
                   float ax, float ay, float az,
                   float gx, float gy, float gz);

#endif
