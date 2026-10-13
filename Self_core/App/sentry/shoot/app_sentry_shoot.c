#include "app_sentry_shoot.h"
#include "lib_pid.h"
#include "lib_math.h"
#include <string.h>
enum { MOTOR_FRIC_L=3, MOTOR_DISC=4, MOTOR_FRIC_R=5 };
static struct { drv_motor_data_t feedback[6]; int16_t current[6]; } s_motor_state;
static struct { struct { uint8_t fire; } data; } s_command_state;
static lib_pid_t s_pid_disc, s_pid_fric_l, s_pid_fric_r;
static void launcher_reset(void)
{
    s_motor_state.current[MOTOR_FRIC_L] = 0;
    s_motor_state.current[MOTOR_FRIC_R] = 0;
    s_motor_state.current[MOTOR_DISC] = 0;
    lib_pid_reset(&s_pid_disc);
    lib_pid_reset(&s_pid_fric_l);
    lib_pid_reset(&s_pid_fric_r);
}
static void launch_control(void)
{
    switch (s_command_state.data.fire) {
    case APP_SENTRY_FIRE_OFF:
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, 0.0f,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, 0.0f,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);

        break;
    default:
        launcher_reset();

        break;

    case APP_SENTRY_FIRE_ON:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_calc(
            &s_pid_disc, SENTRY_DISC_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed), 0.005f);
        break;

    case APP_SENTRY_FIRE_FIR:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        break;

    case APP_SENTRY_FIRE_REVERSE:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_calc(
            &s_pid_disc, -SENTRY_DISC_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed), 0.005f);
        break;

    }
}
void app_sentry_shoot_init(void)
{
    memset(&s_motor_state,0,sizeof(s_motor_state));
    lib_pid_init(&s_pid_disc,100,0,0,0,0,0,0,DRV_MOTOR_M2006_CURRENT_MIN,DRV_MOTOR_M2006_CURRENT_MAX,2000);
    lib_pid_init(&s_pid_fric_l,15,1,0,0,0,0,0,DRV_MOTOR_M3508_CURRENT_MIN,DRV_MOTOR_M3508_CURRENT_MAX,1000);
    lib_pid_init(&s_pid_fric_r,15,1,0,0,0,0,0,DRV_MOTOR_M3508_CURRENT_MIN,DRV_MOTOR_M3508_CURRENT_MAX,1000);
}
void app_sentry_launcher_ctrl_200hz(const app_shoot_input_t *input, app_shoot_output_t *output)
{
    memcpy(s_motor_state.feedback,input->feedback.motor,sizeof(s_motor_state.feedback));
    s_command_state.data.fire=input->command.fire;
    if (!input->command.ready || input->command.emergency_stop) launcher_reset(); else launch_control();
    memcpy(output->current,&s_motor_state.current[3],sizeof(output->current));
    output->vofa[0]=SENTRY_DISC_TAR_SPEED;
    output->vofa[1]=lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed);
    output->vofa[2]=SENTRY_FRICTION_TAR_SPEED;
    output->vofa[3]=lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed);
    output->vofa[4]=-SENTRY_FRICTION_TAR_SPEED;
    output->vofa[5]=lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed);
    output->vofa[6]=s_motor_state.feedback[MOTOR_FRIC_R].current;
}
