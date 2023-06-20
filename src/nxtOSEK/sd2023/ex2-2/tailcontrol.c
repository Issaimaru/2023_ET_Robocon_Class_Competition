#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "nxt_config.h"

void ecrobot_device_initialize(void)
{

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
  TerminateTask(); 
}

int tail_target = 100;
TASK(Task_20ms)
{
  int angle,err;
  float kp =2.0;
  angle = nxt_motor_get_count(PORT_MOTOR_TAIL);
  err = (int)((tail_target - angle) * kp);
  if(err > 100) err = 100;
  if(err < -100) err = -100;
  nxt_motor_set_speed(PORT_MOTOR_TAIL, err, 0);
  TerminateTask(); 
}

TASK(Task_100ms)
{
  TerminateTask(); 
}

TASK(Task_Background)
{
  while(1){
    systick_wait_ms(500U); /* 500msec wait */
  }
}
