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

TASK(Task_Background)
{
   nxt_motor_set_speed(NXT_PORT_B, 80,0);
   systick_wait_ms(5000U);
   nxt_motor_set_speed(NXT_PORT_B, 0,0);
   while(1){
    systick_wait_ms(500U); /* 500msec wait */
   }
}
