/* Analytic ZYX motion profile shared by host tests and the upstream comparison. */
#ifndef IMU_TEST_MOTION_H
#define IMU_TEST_MOTION_H
#include <math.h>

static lib_quatf_t test_quat(double roll, double pitch, double yaw)
{
    double cr = cos(roll/2), sr = sin(roll/2), cp = cos(pitch/2), sp = sin(pitch/2);
    double cy = cos(yaw/2), sy = sin(yaw/2);
    lib_quatf_t q = {(float)(cr*cp*cy+sr*sp*sy), (float)(sr*cp*cy-cr*sp*sy),
                        (float)(cr*sp*cy+sr*cp*sy), (float)(cr*cp*sy-sr*sp*cy)};
    return q;
}

static void test_gravity(lib_quatf_t q, float out[3])
{
    out[0] = 2.0f*(q.q1*q.q3-q.q0*q.q2);
    out[1] = 2.0f*(q.q0*q.q1+q.q2*q.q3);
    out[2] = 1.0f-2.0f*(q.q1*q.q1+q.q2*q.q2);
}

static double test_quat_error(lib_quatf_t a, lib_quatf_t b)
{
    double minus = (double)(a.q0-b.q0)*(a.q0-b.q0)+(double)(a.q1-b.q1)*(a.q1-b.q1)
                 +(double)(a.q2-b.q2)*(a.q2-b.q2)+(double)(a.q3-b.q3)*(a.q3-b.q3);
    double plus = (double)(a.q0+b.q0)*(a.q0+b.q0)+(double)(a.q1+b.q1)*(a.q1+b.q1)
                +(double)(a.q2+b.q2)*(a.q2+b.q2)+(double)(a.q3+b.q3)*(a.q3+b.q3);
    return 4.0*asin(fmin(1.0, 0.5*sqrt(fmin(minus, plus))));
}

static lib_quatf_t test_motion(double t, float rate[3], float acc[3])
{
    double roll = 0.35*sin(0.7*t), pitch = 0.25*sin(0.5*t), yaw = 0.3*t+0.1*sin(0.3*t);
    double rd = 0.245*cos(0.7*t), pd = 0.125*cos(0.5*t), yd = 0.3+0.03*cos(0.3*t);
    lib_quatf_t q = test_quat(roll, pitch, yaw);
    rate[0] = (float)(rd-yd*sin(pitch));
    rate[1] = (float)(pd*cos(roll)+yd*sin(roll)*cos(pitch));
    rate[2] = (float)(-pd*sin(roll)+yd*cos(roll)*cos(pitch));
    test_gravity(q, acc);
    return q;
}
#endif
