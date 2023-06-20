#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "nxt_config.h"

void ecrobot_device_initialize(void)
{
  ecrobot_init_sonar_sensor(NXT_PORT_S2);
}
void ecrobot_device_terminate(void)
{
}
DeclareCounter(SysTimerCnt);
void user_1ms_isr_type2(void)
{
  (void)SignalCounter(SysTimerCnt);
}
TASK(Task_4ms)
{
  U8 sw;
  sw = ecrobot_get_touch_sensor(NXT_PORT_S4);
  if(sw == 1){
    ecrobot_set_light_sensor_active(NXT_PORT_S3);
  } else {
    ecrobot_set_light_sensor_inactive(NXT_PORT_S3);
  }
  TerminateTask(); 
}
TASK(Task_20ms)
{
  static int cnt=0;
  cnt++;
  if(cnt > 100){
    ecrobot_sound_tone(1000, 20, 20);
    cnt = 0;
  }
  TerminateTask(); 
}
TASK(Task_100ms)
{
  S32 dist;
  dist = ecrobot_get_sonar_sensor(NXT_PORT_S2);
  if(dist < 30){
    nxt_motor_set_speed(NXT_PORT_B, 50, 0);
  } else {
    nxt_motor_set_speed(NXT_PORT_B, 00, 0);
  }
  TerminateTask(); 
}
TASK(Task_Background)
{
  while(1){
    systick_wait_ms(500U); /* 500msec wait */
  }
}
