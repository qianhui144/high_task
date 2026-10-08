#include "imu_filter.h"
#include <math.h>

static float twoKp = 2.0f * 2.0f;
static float twoKi = 2.0f * 0.005f;
static float integralFBx = 0, integralFBy = 0, integralFBz = 0;
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float invSampleFreq = 0.005f;

void Mahony_Init(float sample_freq_hz)
{
    invSampleFreq = 1.0f / sample_freq_hz;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
}

void Mahony_Update(Attitude_t *att,
                   float ax, float ay, float az,
                   float gx, float gy, float gz)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 - q2*q3;
        halfvz = q0*q0 - 0.5f + q3*q3;

        halfex = (ay*halfvz - az*halfvy);
        halfey = (az*halfvx - ax*halfvz);
        halfez = (ax*halfvy - ay*halfvx);

        integralFBx += twoKi * halfex * invSampleFreq;
        integralFBy += twoKi * halfey * invSampleFreq;
        integralFBz += twoKi * halfez * invSampleFreq;

        gx += twoKp * halfex + integralFBx;
        gy += twoKp * halfey + integralFBy;
        gz += twoKp * halfez + integralFBz;
    }

    gx *= 0.5f * invSampleFreq;
    gy *= 0.5f * invSampleFreq;
    gz *= 0.5f * invSampleFreq;
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    float roll_r  = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2));
    float pitch_r = asinf(2.0f*(q0*q2-q3*q1));
    float yaw_r   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3));

    att->roll  = roll_r  * 57.29578f;
    att->pitch = pitch_r * 57.29578f;
    att->yaw   = yaw_r   * 57.29578f;
}
