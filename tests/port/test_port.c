/** @file test_port.c
 * @brief 裸机功能迁移回归：输入映射、控制单位、协议及导航仲裁。
 */
#include <assert.h>
#include <stdlib.h>
#undef assert
#define assert(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "project_cfg.h"
#include "app_control.h"
#include "app_sentry_gimbal.h"
#include "app_sentry_shoot.h"
#include "app_sentry_common.h"
#include "app_sentry_can.h"
#include "navigation_ctrl.h"
#include "drv_motor.h"
#include "drv_referee.h"
#include "drv_vofa.h"
#include "lib_pid.h"
static void near(float a,float b,float tolerance) { if (!(isfinite(a)&&fabsf(a-b)<=tolerance)) { fprintf(stderr,"near: %g != %g tol %g\n",a,b,tolerance); exit(1); } }
static void test_input(void)
{
    drv_dbus_data_t d={0}; app_gimbal_dbus_input_t o;
    for(unsigned i=0;i<4;i++) { d.rc.ch[i]=1024; }
    d.rc.rolling_wheel=1024; d.rc.s1=1;
    d.rc.s2=1; assert(app_gimbal_comm_dbus_rx(&d,&o)); assert(o.source==APP_GIMBAL_INPUT_DBUS);
    d.rc.s2=3; assert(app_gimbal_comm_dbus_rx(&d,&o)); assert(o.source==APP_GIMBAL_INPUT_CAN);
    d.rc.s2=2; assert(app_gimbal_comm_dbus_rx(&d,&o)); assert(o.source==APP_GIMBAL_INPUT_ESTOP);
}
static void test_gimbal(void)
{
    app_gimbal_input_t in={0}; app_gimbal_output_t out={0};
    in.imu.valid=1; in.dbus.valid=1;in.dbus.input.source=APP_GIMBAL_INPUT_DBUS;
    in.dbus.input.launcher_mode=APP_SENTRY_FIRE_OFF;
    in.feedback.motor[0].angle_rad=SENTRY_GIMBAL_L_YAW_ZERO*LIB_RAD_PER_ENC13;
    in.feedback.motor[1].angle_rad=SENTRY_GIMBAL_S_YAW_ZERO*LIB_RAD_PER_ENC13;
    in.feedback.motor[2].angle_rad=SENTRY_GIMBAL_PITCH_ZERO*LIB_RAD_PER_ENC13;
    app_sentry_gimbal_init(); app_sentry_gimbal_ctrl_1khz(&in,&out);
    near(out.quat.q0,1,1e-5f);near(out.pitch,0,1e-5f);
    in.dbus.input.chassis_vx_norm=.25f;in.dbus.input.chassis_vy_norm=.5f;
    app_sentry_gimbal_ctrl_1khz(&in,&out);
    near(out.chassis.vx,SENTRY_CHASSIS_VX_TAR_SPEED_MAX*.25f*.001f,.0011f);
    near(out.chassis.vy,-SENTRY_CHASSIS_VY_TAR_SPEED_MAX*.5f*.001f,.0011f);
    in.dbus.input.source=APP_GIMBAL_INPUT_CAN;in.can.radar_speed.vx=1.123f;in.can.radar_speed.vy=-.456f;
    app_sentry_gimbal_ctrl_1khz(&in,&out);near(out.chassis.vx,1.123f,.0011f);near(out.chassis.vy,.456f,.0011f);
    in.imu.euler.yaw=.3f;in.imu.euler.roll=.1f;in.imu.euler.pitch=-.2f;
    in.feedback.motor[2].angle_rad=(SENTRY_GIMBAL_PITCH_ZERO+100)*LIB_RAD_PER_ENC13;
    app_sentry_gimbal_ctrl_1khz(&in,&out);
    near(out.yaw,.3f,1e-5f);near(out.pitch,.1f+100*LIB_RAD_PER_ENC13,1e-5f);near(out.roll,.2f,1e-5f);
    float norm=out.quat.q0*out.quat.q0+out.quat.q1*out.quat.q1+out.quat.q2*out.quat.q2+out.quat.q3*out.quat.q3;
    near(norm,1,1e-5f);
    in.dbus.input.source=APP_GIMBAL_INPUT_ESTOP;app_sentry_gimbal_ctrl_1khz(&in,&out);
    assert(out.current[0]==0&&out.current[1]==0&&out.current[2]==0&&out.shoot.emergency_stop);
}
static void test_shoot(void)
{
    app_shoot_input_t in={0};app_shoot_output_t out;
    in.command.ready=1;in.command.fire=APP_SENTRY_FIRE_ON;
    app_sentry_shoot_init();app_sentry_launcher_ctrl_200hz(&in,&out);
    assert(out.current[0]>0&&out.current[1]>0&&out.current[2]<0);
    lib_pid_t pid;lib_pid_init(&pid,85,150,0,1,0,3000,0,-10000,10000,7000);
    assert(out.current[1]==(int16_t)lib_pid_ff_calc(&pid,SENTRY_DISC_TAR_SPEED,0,3000,0,.005f));
    near(out.disc_integral,pid.ki*pid.integral,.01f);
    in.command.fire=APP_SENTRY_FIRE_FIR;app_sentry_launcher_ctrl_200hz(&in,&out);assert(out.current[1]==0);
    in.command.fire=APP_SENTRY_FIRE_REVERSE;app_sentry_launcher_ctrl_200hz(&in,&out);assert(out.current[1]<0);
    in.command.emergency_stop=1;app_sentry_launcher_ctrl_200hz(&in,&out);
    assert(out.current[0]==0&&out.current[1]==0&&out.current[2]==0);near(out.disc_integral,0,0);
}
static void test_wire(void)
{
    app_ctrl_can_msg_t m={.type=APP_CAN_GIM_FEEDBACK};app_can_frame_t f[APP_SENTRY_CAN_MAX_FRAMES];
    m.value.gimbal.yaw=LIB_RAD_PER_DEG*90;m.value.gimbal.pitch=LIB_RAD_PER_DEG*-30;m.value.gimbal.quat.q0=1;
    assert(app_sentry_can_encode(&m,f)==4);assert(f[0].std_id==0x122&&f[1].std_id==0x124&&f[2].std_id==0x233&&f[3].std_id==0x139);
    float x,y;memcpy(&x,f[3].data,4);memcpy(&y,f[3].data+4,4);near(x,90,.0001f);near(y,-30,.0001f);assert(f[3].bus==1);
    memcpy(&x,f[1].data,4);near(x,LIB_RAD_PER_DEG*90,.0001f);
    float data[9]={1,2,3,4,5,6,7,8,9};uint8_t frame[DRV_VOFA_FRAME_SIZE];
    assert(sizeof(frame)==40);assert(drv_vofa_pack(data,frame)==DRV_VOFA_OK);assert(memcmp(data,frame,36)==0);
    const uint8_t tail[]={0,0,0x80,0x7f};assert(memcmp(frame+36,tail,4)==0);
#if CURRENT_ROBOT == ROBOT_NAVIGATION
    m.type=APP_CAN_CHAS_CURRENT;for(unsigned i=0;i<4;i++) {m.value.current[i]=100+i;}
    assert(app_sentry_can_encode(&m,f)==1);assert(f[0].std_id==0x200&&f[0].data[7]==103);
#endif
    uint8_t command[8];memset(command,0x55,8);assert(!drv_motor_dm_command_frame(command,0));assert(command[0]==0x55);
    assert(drv_motor_dm_command_frame(command,DRV_MOTOR_DM_CMD_ENABLE));for(unsigned i=0;i<7;i++){assert(command[i]==255);}assert(command[7]==0xfc);
    uint8_t packet[128];drv_referee_context_t parser;drv_referee_init(&parser);
    const uint8_t payload[]={1,2,3};uint16_t n=drv_referee_build_frame(packet,sizeof(packet),0x0301,payload,3,7);
    assert(n==12);(void)drv_referee_process(&parser,packet,4);(void)drv_referee_process(&parser,packet+4,n-4);assert(parser.s_frames==1);
}
static void test_navigation(void)
{
    app_chassis_input_t in={0};app_dbus_snapshot_t dbus={0};app_imu_snapshot_t imu={0};app_chassis_output_t out;
    navigation_ctrl_init();dbus.valid=1;dbus.switch_s1=3;dbus.switch_s2=3;dbus.channel_norm[3]=.2f;
    navigation_ctrl_step(&in,&dbus,&imu,100,&out);near(out.target.vx_speed,1.2f,1e-5f);
    assert(out.current[0]<0&&out.current[1]>0&&out.current[2]<0&&out.current[3]>0);
    dbus.switch_s2=1;navigation_ctrl_step(&in,&dbus,&imu,100,&out);for(unsigned i=0;i<4;i++){assert(out.current[i]==0);}
    imu.valid=1;imu.sampled=1;imu.tick=100;imu.euler.yaw=LIB_RAD_PER_DEG*90;
    navigation_ctrl_step(&in,&dbus,&imu,100,&out);assert(out.current[0]>0&&out.current[1]<0&&out.current[2]<0&&out.current[3]>0);
    navigation_ctrl_step(&in,&dbus,&imu,121,&out);for(unsigned i=0;i<4;i++){assert(out.current[i]==0);}
    dbus.switch_s1=1;navigation_ctrl_step(&in,&dbus,&imu,121,&out);for(unsigned i=0;i<4;i++){assert(out.current[i]==0);}
    dbus.switch_s1=2;in.can.speed.vx=.7f;navigation_ctrl_step(&in,&dbus,&imu,121,&out);near(out.target.vx_speed,.7f,1e-6f);
    dbus.valid=0;navigation_ctrl_step(&in,&dbus,&imu,121,&out);for(unsigned i=0;i<4;i++){assert(out.current[i]==0);}
}
int main(void){test_input();test_gimbal();test_shoot();test_wire();test_navigation();puts("PASS: input, gimbal, launcher, CAN, VOFA, referee, DM and navigation");return 0;}
