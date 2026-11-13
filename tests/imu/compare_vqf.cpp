/* Differential test against the author's MIT-licensed VQF implementation. */
extern "C" {
#include "drv_imu.h"
}
#include "vqf.hpp"
#include "imu_test_motion.h"
#include <cassert>
#include <cstdio>

static void compare(float dt, int variant)
{
    VQFParams params;
    drv_imu_t imu;
    drv_imu_init(&imu, nullptr);
    drv_imu_vqf_config_t config = imu.vqf_config;
    params.magDistRejectionEnabled = false;
    if (variant == 1) {
        params.motionBiasEstEnabled = config.motion_bias_enabled = false;
        params.restBiasEstEnabled = config.rest_bias_enabled = false;
    } else if (variant == 2) {
        params.motionBiasEstEnabled = config.motion_bias_enabled = false;
    } else if (variant == 3) {
        params.restBiasEstEnabled = config.rest_bias_enabled = false;
    } else if (variant == 4) {
        params.tauAcc = config.tau_acc = 0.15f;
        params.restFilterTau = config.rest_filter_tau = 0.2f;
        params.biasSigmaMotion = config.bias_sigma_motion = 0.2f;
    }
    assert(drv_imu_vqf_configure(&imu, &config));
    drv_imu_set_algorithm(&imu, DRV_IMU_ALGO_VQF);
    VQF reference(params, dt);
    double max_angle = 0, max_bias = 0;
    int rest_mismatch = 0;
    for (int i = 1; i <= static_cast<int>(30.0f/dt); ++i) {
        float t = i*dt;
        float gyr[3] = {0}, acc[3] = {0, 0, 1};
        float unused[3];
        if (t > 2.0f && t < 17.0f) test_motion(t-2.0, gyr, acc);
        else if (t >= 17.0f) test_motion(15.0, unused, acc);
        for (int k = 0; k < 3; ++k) {
            gyr[k] += (k+1)*0.002f+0.0002f*sinf(i*(k+1)*0.17f);
            acc[k] += 0.001f*sinf(i*(k+1)*0.23f);
        }
        imu.gyro = (lib_vec3f_t){gyr[0], gyr[1], gyr[2]};
        imu.acc = (lib_vec3f_t){acc[0], acc[1], acc[2]};
        float acc_si[3] = {acc[0]*9.80665f, acc[1]*9.80665f, acc[2]*9.80665f};
        reference.update(gyr, acc_si);
        drv_imu_update(&imu, dt);
        float q[4], b[3];
        reference.getQuat6D(q);
        reference.getBiasEstimate(b);
        lib_quatf_t expected = {q[0], q[1], q[2], q[3]};
        max_angle = fmax(max_angle, test_quat_error(imu.quat, expected));
        for (int k = 0; k < 3; ++k) max_bias = fmax(max_bias, fabs(imu.filter.vqf.bias[k]-b[k]));
        if (imu.filter.vqf.rest_detected != reference.getRestDetected()) ++rest_mismatch;
    }
    std::printf("VQF reference dt=%.4f variant=%d max angle=%.6f deg, bias=%.8f rad/s, rest mismatches=%d\n",
                dt, variant, max_angle*180.0/acos(-1.0), max_bias, rest_mismatch);
    assert(max_angle < 0.000524); // < 0.03 degrees, including float rounding over 30 seconds.
    assert(max_bias < 0.00002);
    assert(rest_mismatch <= 1);
}

int main()
{
    for (int variant = 0; variant <= 4; ++variant) {
        compare(0.001f, variant);
        compare(0.005f, variant);
    }
    std::puts("Official VQF differential comparison: PASS");
}
