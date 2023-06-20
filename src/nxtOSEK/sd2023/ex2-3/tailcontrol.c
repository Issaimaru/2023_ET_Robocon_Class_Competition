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

int tail_target = 106;
TASK(Task_20ms)
{
  int angle,err;
  float kp = 0.5;
  float ki = 0.01;
  float err_p;
  static float err_int;
  angle = nxt_motor_get_count(PORT_MOTOR_TAIL);
  err_p = tail_target - angle;
  err_int += err_p;
  err = (int)((err_p * kp) + (err_int * ki)); 
  if(err > 100) err =100;
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
