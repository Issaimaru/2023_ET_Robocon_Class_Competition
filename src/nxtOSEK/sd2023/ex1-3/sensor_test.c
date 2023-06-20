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

TASK(Task_Background)
{
   while(1){
     S32 range = ecrobot_get_sonar_sensor(NXT_PORT_S2);
     U32 freq = 1500 - (range * 4);
     ecrobot_sound_tone(freq, 20, 20);

     U8 sw = ecrobot_get_touch_sensor(NXT_PORT_S4);
     if(sw == 1){
       ecrobot_set_light_sensor_active(NXT_PORT_S3);
     } else {
       ecrobot_set_light_sensor_inactive(NXT_PORT_S3);
     }
     systick_wait_ms(1000U);
   }
}
