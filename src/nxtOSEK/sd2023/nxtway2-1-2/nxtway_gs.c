/* Linetrace program
  ON-OFF control
  with light sensor calibration sequence
  */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"
#include "nxt_config.h"

// プロトタイプ宣言
void wait_touch(int);
void start_sequence(void);

typedef enum{
  WAIT_MODE,
  INIT_MODE,          /* system initialize mode */
  CAL_MODE,           /* gyro sensor offset calibration mode */
  CONTROL_MODE /* balance and RC control mode */
} MODE_ENUM;

MODE_ENUM nxtway_gs_mode = WAIT_MODE; /* NXTway-GS mode at initial state */

void ecrobot_device_initialize(void)
{
  ecrobot_set_light_sensor_active(PORT_LIGHT);
  ecrobot_init_sonar_sensor(PORT_SONAR);
}

void ecrobot_device_terminate(void)
{
  nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
  nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
  ecrobot_term_sonar_sensor(PORT_SONAR);
}

DeclareCounter(SysTimerCnt);

void user_1ms_isr_type2(void)
{
  (void)SignalCounter(SysTimerCnt);
}

S8  cmd_forward, cmd_turn;

TASK(Task_4ms)
{
  static U32 gyro_offset;
  static U32 avg_cnt;        
  static U32 cal_start_time;        
  S8        pwm_l, pwm_r;
  switch(nxtway_gs_mode){
  case(WAIT_MODE):
    break;
  case(INIT_MODE):
    gyro_offset = 0;
    avg_cnt = 0;
    balance_init();
    nxt_motor_set_count(PORT_MOTOR_L, 0); /* reset left motor count */
    nxt_motor_set_count(PORT_MOTOR_R, 0); /* reset right motor count */
    cal_start_time = ecrobot_get_systick_ms();
    nxtway_gs_mode = CAL_MODE;
    break;
  case(CAL_MODE):
    gyro_offset += (U32)ecrobot_get_gyro_sensor(PORT_GYRO);
    avg_cnt++;
    if ((ecrobot_get_systick_ms() - cal_start_time) >= 1000U) {
      gyro_offset /= avg_cnt;
      ecrobot_sound_tone(440U, 500U, 30U); /* beep a tone */
      nxtway_gs_mode = CONTROL_MODE;
    }
    break;
  case(CONTROL_MODE):
    balance_control(
      (F32)cmd_forward,
      (F32)cmd_turn,
      (F32)ecrobot_get_gyro_sensor(PORT_GYRO),
      (F32)gyro_offset,
      (F32)nxt_motor_get_count(PORT_MOTOR_L),
      (F32)nxt_motor_get_count(PORT_MOTOR_R),
      (F32)ecrobot_get_battery_voltage(),
      &pwm_l,
      &pwm_r);
    nxt_motor_set_speed(PORT_MOTOR_L, pwm_l, 1);
    nxt_motor_set_speed(PORT_MOTOR_R, pwm_r, 1);
    break;
  default:
    nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
    nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
    break;
  }
  TerminateTask(); 
}

int trace_target = 550;

TASK(Task_20ms)
{
  int bright;
  cmd_forward = 30;
  bright = ecrobot_get_light_sensor(PORT_LIGHT);
  if(bright < trace_target){
    cmd_turn = - 60;
  } else {
    cmd_turn = 60;
  }
  TerminateTask(); 
}

TASK(Task_100ms)
{
  if(nxtway_gs_mode == CONTROL_MODE){
    if(ecrobot_get_touch_sensor(PORT_TOUCH)){
      nxtway_gs_mode = WAIT_MODE;
    }
  }
  TerminateTask(); 
}

TASK(Task_Background)
{
  display_clear(1);
  start_sequence();
  nxtway_gs_mode = INIT_MODE;
  while(1){
    systick_wait_ms(500U); /* 500msec wait */
  }
}

void wait_touch(int freq){
  while(ecrobot_get_touch_sensor(PORT_TOUCH)==0){}  //wait for pushing switch
  while(ecrobot_get_touch_sensor(PORT_TOUCH)==1){}  //wait for releasing switch
  ecrobot_sound_tone(freq,100,50);
}

void start_sequence(){
  int bright_white, bright_black;
  wait_touch(800);
  bright_white = ecrobot_get_light_sensor(PORT_LIGHT);
  display_goto_xy(0,0); display_int(bright_white, 6); display_update();
  wait_touch(bright_white);
  bright_black = ecrobot_get_light_sensor(PORT_LIGHT);
  display_goto_xy(0,1); display_int(bright_black, 6); display_update();
  wait_touch(bright_black);
  trace_target = (int)((bright_white + bright_black) / 2.0);
}