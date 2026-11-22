/** Host-side behavioral tests: no HAL, SPI access, or hardware flashing. */
#include "drv_imu.h"
#include "imu_test_motion.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const float dt = 0.001f;

static void set_sample(drv_imu_t *imu, const float gyr[3], const float acc[3])
{
    imu->gyro.x = gyr[0] + imu->gyro_offset.x;
    imu->gyro.y = gyr[1] + imu->gyro_offset.y;
    imu->gyro.z = gyr[2] + imu->gyro_offset.z;
    imu->acc.x = acc[0] + imu->acc_offset.x;
    imu->acc.y = acc[1] + imu->acc_offset.y;
    imu->acc.z = acc[2] + imu->acc_offset.z;
}

static void check_unit(const drv_imu_t *imu)
{
    const lib_quatf_t q = imu->quat;
    float n = q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3;
    assert(isfinite(n) && fabsf(n - 1.0f) < 3e-6f);
}

static void check_covariance(const drv_imu_t *imu)
{
    double l[6][6] = {{0}};
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            double x = imu->filter.ekf.covariance[i][j];
            assert(isfinite(x));
            assert(fabs(x - imu->filter.ekf.covariance[j][i]) < 1e-6);
            for (int k = 0; k < j; ++k)
            {
                x -= l[i][k] * l[j][k];
            }
            if (i == j)
            {
                assert(x > 0.0);
                l[i][j] = sqrt(x);
            }
            else
            {
                l[i][j] = x / l[j][j];
            }
        }
    }
}

static void test_rotations(void)
{
    for (int algo = 0; algo < 3; ++algo)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            drv_imu_t imu;
            drv_imu_init(&imu, NULL);
            assert(drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)algo));
            for (int i = 1; i <= 8000; ++i)
            {
                double angle = 0.7 * (double)i * (double)dt;
                lib_quatf_t expected = test_quat(axis == 0 ? angle : 0.0, axis == 1 ? angle : 0.0,
                                                 axis == 2 ? angle : 0.0);
                float gyr[3] = {0}, acc[3];
                gyr[axis] = 0.7f;
                test_gravity(expected, acc);
                set_sample(&imu, gyr, acc);
                drv_imu_update(&imu, dt);
                check_unit(&imu);
                assert(test_quat_error(imu.quat, expected) < 0.003);
                if (algo == DRV_IMU_ALGO_EKF && i % 200 == 0)
                {
                    check_covariance(&imu);
                }
            }
        }
    }
    puts("constant rotations on all axes: PASS");
}

static void test_dynamic_motion(void)
{
    for (int algo = 1; algo < 3; ++algo)
    {
        drv_imu_t imu;
        double max_error = 0.0;
        drv_imu_init(&imu, NULL);
        drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)algo);
        for (int i = 1; i <= 30000; ++i)
        {
            float gyr[3], acc[3], unused[3];
            lib_quatf_t expected = test_motion((double)i * dt, unused, acc);
            test_motion(((double)i - 0.5) * dt, gyr, unused);
            /* Deterministic vibration and angular noise, not tied to filter internals. */
            for (int k = 0; k < 3; ++k)
            {
                gyr[k] += 0.0008f * sinf((float)(i * (k + 1)) * 0.17f);
                acc[k] += 0.003f * sinf((float)(i * (k + 1)) * 0.23f);
            }
            set_sample(&imu, gyr, acc);
            drv_imu_update(&imu, dt);
            check_unit(&imu);
            max_error = fmax(max_error, test_quat_error(imu.quat, expected));
            if (algo == DRV_IMU_ALGO_EKF && i % 200 == 0)
            {
                check_covariance(&imu);
            }
        }
        printf("dynamic motion algo=%d max error=%.6f deg\n", algo, max_error * 180.0 / acos(-1.0));
        assert(max_error < 0.0174533); /* < 1 degree */
    }
}

static void test_static_bias(void)
{
    const float acc[3] = {0, 0, 1};
    for (int algo = 1; algo < 3; ++algo)
    {
        drv_imu_t imu;
        float gyr[3] = {0.015f, -0.010f, algo == DRV_IMU_ALGO_VQF ? 0.008f : 0.0f};
        drv_imu_init(&imu, NULL);
        imu.gyro_offset = (lib_vec3f_t){0.02f, 0.01f, -0.03f};
        imu.acc_offset = (lib_vec3f_t){0.02f, -0.03f, 0.01f};
        drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)algo);
        set_sample(&imu, gyr, acc);
        for (int i = 0; i < 40000; ++i)
        {
            drv_imu_update(&imu, dt);
            if (algo == DRV_IMU_ALGO_EKF && i % 500 == 0)
            {
                check_covariance(&imu);
            }
        }
        const float *bias = algo == DRV_IMU_ALGO_EKF ? imu.filter.ekf.bias : imu.filter.vqf.bias;
        printf("static bias algo=%d estimate=(%.7f, %.7f, %.7f) rad/s\n", algo, bias[0], bias[1],
               bias[2]);
        for (int k = 0; k < (algo == DRV_IMU_ALGO_EKF ? 2 : 3); ++k)
        {
            assert(fabsf(bias[k] - gyr[k]) < 0.0004f);
        }
        if (algo == DRV_IMU_ALGO_VQF)
        {
            assert(imu.filter.vqf.rest_detected);
        }
        assert(fabsf(imu.gyro_offset.z + 0.03f) < 1e-7f);
        float h[3];
        test_gravity(imu.quat, h);
        assert(fabsf(h[0]) < 0.002f && fabsf(h[1]) < 0.002f);
        check_unit(&imu);
    }
}

static void test_gates_and_invalid_samples(void)
{
    const float invalid_dt[] = {0, -1, NAN, INFINITY, 0.101f};
    for (int algo = 0; algo < 3; ++algo)
    {
        drv_imu_t imu, before;
        drv_imu_init(&imu, NULL);
        drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)algo);
        imu.acc.z = 1.0f;
        for (unsigned int i = 0; i < sizeof(invalid_dt) / sizeof(invalid_dt[0]); ++i)
        {
            before = imu;
            drv_imu_update(&imu, invalid_dt[i]);
            assert(memcmp(&before, &imu, sizeof(imu)) == 0);
        }
        imu.gyro.x = NAN;
        before = imu;
        drv_imu_update(&imu, dt);
        assert(memcmp(&before, &imu, sizeof(imu)) == 0);
        imu.gyro.x = 0.0f;
        imu.gyro.z = 1.0f;
        imu.acc.z = NAN;
        drv_imu_update(&imu, 0.01f);
        assert(test_quat_error(imu.quat, test_quat(0, 0, 0.01)) < 1e-5);
        imu.acc.z = 0.0f;
        drv_imu_update(&imu, 0.01f);
        assert(test_quat_error(imu.quat, test_quat(0, 0, 0.02)) < 1e-5);
        check_unit(&imu);
    }
    drv_imu_t imu;
    drv_imu_init(&imu, NULL);
    drv_imu_set_algorithm(&imu, DRV_IMU_ALGO_EKF);
    imu.acc.z = 1;
    for (int i = 0; i < 1000; ++i)
    {
        drv_imu_update(&imu, dt);
    }
    imu.acc.x = 1;
    drv_imu_update(&imu, dt);
    assert(!imu.filter.ekf.acc_used); /* Acceleration norm gate. */
    imu.acc.z = 0;
    drv_imu_update(&imu, dt);
    assert(!imu.filter.ekf.acc_used); /* Unit-norm acceleration, innovation gate. */
    assert(imu.filter.ekf.innovation_squared > imu.ekf_config.innovation_gate);
    check_covariance(&imu);
    puts("invalid samples, gyro-only propagation, EKF gates: PASS");
}

static void test_switch_reset_alignment(void)
{
    drv_imu_t imu, before;
    drv_imu_init(&imu, NULL);
    lib_quatf_t original = test_quat(0.4, -0.2, 1.2);
    imu.quat = original;
    for (int algo = 1; algo < 3; ++algo)
    {
        assert(drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)algo));
        assert(test_quat_error(imu.quat, original) < 1e-6);
        drv_imu_reset_filter(&imu);
        assert(test_quat_error(imu.quat, original) < 1e-6);
    }
    before = imu;
    assert(!drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)-1));
    assert(!drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)3));
    assert(memcmp(&before, &imu, sizeof(imu)) == 0);
    drv_imu_ekf_config_t ekf = imu.ekf_config;
    ekf.acc_noise = NAN;
    assert(!drv_imu_ekf_configure(&imu, &ekf));
    drv_imu_vqf_config_t vqf = imu.vqf_config;
    vqf.bias_forgetting_time = 0;
    assert(!drv_imu_vqf_configure(&imu, &vqf));
    assert(memcmp(&before, &imu, sizeof(imu)) == 0);
    assert(drv_imu_ekf_configure(&imu, NULL));
    assert(drv_imu_vqf_configure(&imu, NULL));
    drv_imu_set_algorithm(&imu, DRV_IMU_ALGO_EKF);
    imu.filter.ekf.bias[0] = 0.01f;
    before = imu;
    drv_imu_set_algorithm(&imu, DRV_IMU_ALGO_EKF);
    assert(memcmp(&before, &imu, sizeof(imu)) == 0);
    float gravity[3], zero[3] = {0};
    test_gravity(original, gravity);
    set_sample(&imu, zero, gravity);
    drv_imu_initial_alignment(&imu);
    float estimated[3];
    test_gravity(imu.quat, estimated);
    for (int k = 0; k < 3; ++k)
    {
        assert(fabsf(gravity[k] - estimated[k]) < 1e-6f);
    }
    assert(imu.filter.ekf.bias[0] == 0.0f);
    drv_imu_vqf_update(&imu, dt);
    assert(imu.algorithm == DRV_IMU_ALGO_VQF);
    drv_imu_ekf_update(&imu, dt);
    assert(imu.algorithm == DRV_IMU_ALGO_EKF);
    drv_imu_mahony_update(&imu, dt);
    assert(imu.algorithm == DRV_IMU_ALGO_MAHONY);
    puts("configuration, switch continuity, alignment, reset: PASS");
}

static void test_vqf_rest_and_variable_dt(void)
{
    drv_imu_t imu;
    drv_imu_init(&imu, NULL);
    drv_imu_set_algorithm(&imu, DRV_IMU_ALGO_VQF);
    imu.acc.z = 1;
    for (int i = 0; i < 1000; ++i)
    {
        drv_imu_update(&imu, dt);
    }
    assert(!imu.filter.vqf.rest_detected);
    for (int i = 0; i < 1000; ++i)
    {
        drv_imu_update(&imu, dt);
    }
    assert(imu.filter.vqf.rest_detected);
    imu.gyro.z = 0.7f;
    drv_imu_update(&imu, dt);
    assert(!imu.filter.vqf.rest_detected);
    drv_imu_reset_filter(&imu);
    lib_quatf_t start = imu.quat;
    const float periods[] = {0.001f, 0.002f, 0.0005f, 0.005f};
    double t = 0;
    for (int i = 0; i < 12000; ++i)
    {
        float step = periods[(i / 1000) % 4];
        t += step;
        drv_imu_update(&imu, step);
        check_unit(&imu);
    }
    double initial_yaw = 2.0 * atan2(start.q3, start.q0);
    assert(test_quat_error(imu.quat, test_quat(0, 0, initial_yaw + 0.7 * t)) < 0.005);
    /* Exact upside-down initial inclination correction. */
    drv_imu_init(&imu, NULL);
    imu.acc.z = -1;
    drv_imu_vqf_update(&imu, dt);
    check_unit(&imu);
    float h[3];
    test_gravity(imu.quat, h);
    assert(h[2] < -0.9999f);
    puts("VQF rest detection, changing sample period, inverted pose: PASS");
}

/* Physical fixture: sensor is mounted +90 degrees about the reference x axis.
 * q_sensor = q_body * qx(90deg), independent of the production mounting API. */
static lib_quatf_t vertical_sensor_pose(lib_quatf_t body)
{
    const float c = 0.7071067811865475f;
    lib_quatf_t sensor = {c * (body.q0 - body.q1), c * (body.q0 + body.q1), c * (body.q2 + body.q3),
                          c * (body.q3 - body.q2)};
    return sensor;
}

static void test_mounting_output(void)
{
    const float deg = (float)(acos(-1.0) / 180.0);
    drv_imu_t imu;
    lib_quatf_t body;
    drv_imu_init(&imu, NULL);
    assert(imu.mounting_quat.q0 == 1.0f);
    drv_imu_set_euler_mode(&imu, DRV_IMU_EULER_ZYX);
    imu.quat = test_quat(0.2, -0.3, 1.1);
    drv_imu_quat_to_euler(&imu);
    assert(fabsf(imu.euler.roll * deg - 0.2f) < 1e-6f);
    assert(fabsf(imu.euler.pitch * deg + 0.3f) < 1e-6f);
    assert(fabsf(imu.euler.yaw * deg - 1.1f) < 1e-6f);

    assert(drv_imu_set_mounting(&imu, 90, 0, 0));
    /* Both sides of the uncorrected +90deg singularity, in all display modes. */
    const float rolls[] = {-20.0f, -0.1f, 0.0f, 0.1f, 20.0f};
    for (int mode = 0; mode < 3; ++mode)
    {
        drv_imu_set_euler_mode(&imu, (drv_imu_euler_mode_t)mode);
        for (unsigned int i = 0; i < sizeof(rolls) / sizeof(rolls[0]); ++i)
        {
            lib_quatf_t expected = test_quat(rolls[i] * deg, 0, 0.7);
            imu.quat = vertical_sensor_pose(expected);
            lib_quatf_t sensor = imu.quat;
            drv_imu_quat_to_euler(&imu);
            drv_imu_get_body_quat(&imu, &body);
            assert(test_quat_error(body, expected) < 1e-6);
            assert(fabsf(imu.euler.roll - rolls[i]) < 0.0001f);
            assert(fabsf(imu.euler.pitch) < 0.0001f);
            assert(fabsf(imu.euler.yaw * deg - 0.7f) < 1e-6f);
            assert(memcmp(&sensor, &imu.quat, sizeof(sensor)) == 0);
        }
    }
    /* Mixed tilt and heading must also remove the mount on the correct side. */
    drv_imu_set_euler_mode(&imu, DRV_IMU_EULER_ZYX);
    imu.quat = vertical_sensor_pose(test_quat(-0.25, 0.35, -2.4));
    drv_imu_quat_to_euler(&imu);
    assert(fabsf(imu.euler.roll * deg + 0.25f) < 1e-6f);
    assert(fabsf(imu.euler.pitch * deg - 0.35f) < 1e-6f);
    assert(fabsf(imu.euler.yaw * deg + 2.4f) < 1e-6f);

    /* qz(90)qx(90) maps sensor x/y/z to reference y/z/x. */
    assert(drv_imu_set_mounting(&imu, 90, 0, 90));
    imu.quat = (lib_quatf_t){0.5f, 0.5f, 0.5f, 0.5f};
    drv_imu_get_body_quat(&imu, &body);
    assert(test_quat_error(body, test_quat(0, 0, 0)) < 1e-6);
    imu.gyro = (lib_vec3f_t){1, 2, 3};
    lib_vec3f_t gyro;
    drv_imu_get_body_gyro(&imu, &gyro);
    assert(fabsf(gyro.x - 3) < 1e-6f);
    assert(fabsf(gyro.y - 1) < 1e-6f);
    assert(fabsf(gyro.z - 2) < 1e-6f);

    drv_imu_t before = imu;
    assert(!drv_imu_set_mounting(&imu, NAN, 0, 0));
    assert(!drv_imu_set_mounting(&imu, 0, INFINITY, 0));
    assert(!drv_imu_set_mounting(&imu, 0, 0, NAN));
    assert(!drv_imu_set_mounting(NULL, 0, 0, 0));
    assert(memcmp(&before, &imu, sizeof(imu)) == 0);
    for (int algo = 0; algo < 3; ++algo)
    {
        drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)algo);
        drv_imu_reset_filter(&imu);
        assert(memcmp(&before.mounting_quat, &imu.mounting_quat, sizeof(imu.mounting_quat)) == 0);
    }
    puts("mounting: identity, vertical zero, crossing 90deg, mixed pose, gyro axes, reset: PASS");
}

static void test_vertical_mount_algorithms(void)
{
    for (int algo = 0; algo < 3; ++algo)
    {
        drv_imu_t imu;
        drv_imu_init(&imu, NULL);
        drv_imu_set_mounting(&imu, 90, 0, 0);
        imu.acc.y = 1.0f; /* Upright board, horizontal reference body. */
        drv_imu_initial_alignment(&imu);
        drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)algo);
        for (int i = 0; i < 2000; ++i)
        {
            drv_imu_update(&imu, dt);
        }
        drv_imu_quat_to_euler(&imu);
        assert(fabsf(imu.euler.roll) < 0.001f);
        assert(fabsf(imu.euler.pitch) < 0.001f);
        assert(fabsf(imu.euler.yaw) < 0.001f);
        /* Turning about vertical excites sensor y, not sensor z. */
        imu.gyro.y = 0.7f;
        for (int i = 0; i < 1000; ++i)
        {
            drv_imu_update(&imu, dt);
        }
        lib_quatf_t body;
        drv_imu_get_body_quat(&imu, &body);
        assert(test_quat_error(body, test_quat(0, 0, 0.7)) < 0.003);
        lib_vec3f_t gyro;
        drv_imu_get_body_gyro(&imu, &gyro);
        assert(fabsf(gyro.z - 0.7f) < 1e-6f);
    }
    puts("Mahony/EKF/VQF: upright-board alignment and physical yaw rotation: PASS");
}

static void test_body_rate_bias_compensation(void)
{
    for (int algo = 0; algo < 3; ++algo)
    {
        drv_imu_t imu;
        drv_imu_init(&imu, NULL);
        drv_imu_set_mounting(&imu, 90, 0, 0);
        drv_imu_set_algorithm(&imu, (drv_imu_algorithm_t)algo);
        imu.gyro_offset = (lib_vec3f_t){0.11f, -0.22f, 0.33f};
        imu.gyro = (lib_vec3f_t){0.14f, -0.28f, 0.42f};
        const float bias[3] = {0.01f, -0.02f, 0.03f};
        for (int i = 0; i < 3; ++i)
        {
            if (algo == DRV_IMU_ALGO_EKF)
            {
                imu.filter.ekf.bias[i] = bias[i];
            }
            else if (algo == DRV_IMU_ALGO_VQF)
            {
                imu.filter.vqf.bias[i] = bias[i];
            }
            else
            {
                imu.integral_fb[i] = -bias[i];
            }
        }
        drv_imu_t before = imu;
        lib_vec3f_t rate;
        drv_imu_get_body_gyro(&imu, &rate);
        /* sensor true rate=(.02,-.04,.06), Rx(90) maps it to (.02,-.06,-.04). */
        assert(fabsf(rate.x - 0.02f) < 1e-6f);
        assert(fabsf(rate.y + 0.06f) < 1e-6f);
        assert(fabsf(rate.z + 0.04f) < 1e-6f);
        assert(memcmp(&before, &imu, sizeof(imu)) == 0);
        imu.gyro = (lib_vec3f_t){0.12f, -0.24f, 0.36f};
        drv_imu_get_body_gyro(&imu, &rate);
        assert(fabsf(rate.x) + fabsf(rate.y) + fabsf(rate.z) < 1e-6f);
    }
    puts("body gyro: static/online bias compensation and vertical sensor axes: PASS");
}

static void check_finite_lpf(const drv_imu_vqf_state_t *s)
{
    const double *state[] = {s->acc_lp_state, s->rest_gyr_state, s->rest_acc_state,
                             s->motion_r_state, s->motion_bias_state};
    const unsigned int count[] = {6, 6, 6, 18, 4};
    for (int k = 0; k < 5; ++k)
    {
        assert(isfinite(s->init_time[k]));
        for (unsigned int i = 0; i < count[k]; ++i)
        {
            assert(isfinite(state[k][i]));
        }
    }
}

static void test_vqf_finite_initialization(void)
{
    drv_imu_t imu;
    drv_imu_init(&imu, NULL);
    drv_imu_set_algorithm(&imu, DRV_IMU_ALGO_VQF);
    for (int k = 0; k < 5; ++k)
    {
        assert(!imu.filter.vqf.lp_initialized[k]);
    }
    check_finite_lpf(&imu.filter.vqf);
    imu.acc.y = 1;
    drv_imu_initial_alignment(&imu);
    const float periods[] = {0.0005f, 0.001f, 0.002f, 0.005f};
    for (int i = 0; i < 4000; ++i)
    {
        drv_imu_update(&imu, periods[i % 4]);
        check_finite_lpf(&imu.filter.vqf);
        check_unit(&imu);
    }
    for (int k = 0; k < 5; ++k)
    {
        assert(imu.filter.vqf.lp_initialized[k]);
    }
    assert(imu.filter.vqf.rest_detected);
    assert(drv_imu_filter_is_ready(&imu));
    drv_imu_reset_filter(&imu);
    assert(!drv_imu_filter_is_ready(&imu));
    for (int k = 0; k < 5; ++k)
    {
        assert(!imu.filter.vqf.lp_initialized[k]);
    }
    check_finite_lpf(&imu.filter.vqf);
    puts("VQF finite LP initialization, variable-dt warmup, reset (no NaN sentinel): PASS");
}

int main(void)
{
    drv_imu_init(NULL, NULL);
    drv_imu_update(NULL, dt);
    drv_imu_ekf_update(NULL, dt);
    drv_imu_vqf_update(NULL, dt);
    test_mounting_output();
    test_vertical_mount_algorithms();
    test_body_rate_bias_compensation();
    test_vqf_finite_initialization();
    test_switch_reset_alignment();
    test_gates_and_invalid_samples();
    test_rotations();
    test_dynamic_motion();
    test_static_bias();
    test_vqf_rest_and_variable_dt();
    printf("IMU behavioral tests: PASS (handle size=%zu bytes)\n", sizeof(drv_imu_t));
    return 0;
}
