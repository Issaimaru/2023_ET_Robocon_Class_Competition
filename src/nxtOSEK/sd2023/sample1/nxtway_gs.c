// NxtWay_GS sample program

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"
#include "nxt_config.h"

typedef enum{
  INIT_MODE, 	 /* system initialize mode */
  CAL_MODE,  	 /* gyro sensor offset calibration mode */
  CONTROL_MODE /* balance and RC control mode */
} MODE_ENUM;

MODE_ENUM nxtway_gs_mode = INIT_MODE; /* NXTway-GS mode at initial state */

void ecrobot_device_initialize(void)
{
    ecrobot_init_sonar_sensor(PORT_SONAR);
}

void ecrobot_device_terminate(void)
{
  nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
  nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
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
  S8	pwm_l, pwm_r;
  switch(nxtway_gs_mode){
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

TASK(Task_20ms)
{
  TerminateTask(); 
}

TASK(Task_100ms)
{
  TerminateTask(); 
}

TASK(Task_Background)
{
  int freq;
  cmd_forward = 0;
  cmd_turn = 0;
  systick_wait_ms(5000U);
  while(1){
    cmd_forward = 0;
    cmd_turn = 30;
    while(70 > (freq = ecrobot_get_sonar_sensor(PORT_SONAR))){
      ecrobot_sound_tone((800-(5*freq)),100,50);
      systick_wait_ms(500U); 
    }
    cmd_turn = 0;
    cmd_forward = 40;
    while(70 < ecrobot_get_sonar_sensor(PORT_SONAR)){
      systick_wait_ms(500U); /* 500msec wait */
    }
  }
}
