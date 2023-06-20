/* soundtest.c */
#include	"math.h"

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
  StatusType ercd;
  ercd = SignalCounter(SysTimerCnt); 
  if(ercd != E_OK)
  {
    ShutdownOS(ercd);
  }
}

/* Task1 executed every 10ms */

TASK(OSEK_Task_Background)
{
  ecrobot_sound_tone(440, 100, 50);systick_wait_ms(1000);
  ecrobot_sound_tone(440, 100, 50);systick_wait_ms(1000);
  ecrobot_sound_tone(440, 100, 50);systick_wait_ms(1000);
  ecrobot_sound_tone(880, 1000, 50);
  while(1){}
}  
