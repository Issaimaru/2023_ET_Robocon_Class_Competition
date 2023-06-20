/* Linetrace program
  PID control
  stanging start with tail bar
  */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"
#include "nxt_config.h"
#include "math.h"

// プロトタイプ宣言
void wait_touch(int);
void light_calibration(void);
void trace_control(void);
int limit_value(int, int , int);
int get_light_sensor(void);
void tail_control(void);
void sonar_agent(void);

typedef enum{
  WAIT_MODE,
  INIT_MODE,          /* system initialize mode */
  CAL_MODE,           /* gyro sensor offset calibration mode */
  CONTROL_MODE /* balance and RC control mode */
} MODE_ENUM;

MODE_ENUM nxtway_gs_mode = WAIT_MODE; /* NXTway-GS mode at initial state */

typedef enum{
  TRACE_ON,
  TRACE_OFF,
} TRACE_MODE;

TRACE_MODE trace_mode = TRACE_OFF;

void ecrobot_device_initialize(void)
{
  ecrobot_set_light_sensor_active(PORT_LIGHT);
  ecrobot_init_sonar_sensor(PORT_SONAR);
  //ecrobot_set_bt_device_name("ZAKU-I"); // <- set NXT NAME
  ecrobot_init_bt_slave("1234");
}

void ecrobot_device_terminate(void)
{
  nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
  nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
  ecrobot_term_sonar_sensor(PORT_SONAR);
  ecrobot_term_bt_connection();
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
const float wheel_dia = 85.0;
const float tread = 163;
float odd_distance;
float robot_a;
void
oddmetry(void){
  int theta_r = nxt_motor_get_count(PORT_MOTOR_R);
  int theta_l = nxt_motor_get_count(PORT_MOTOR_L);
  odd_distance =((theta_r + theta_l) * 0.5 * 0.008726 * wheel_dia);
  robot_a = (theta_r - theta_l) * wheel_dia / tread;
}

float position_x = 0.0;
float position_y = 0.0;
float odd_distance_old = 0.0;
void
localization(void){
  float ds = odd_distance - odd_distance_old;
  float rad = robot_a * 0.01745 ; // pi / 180
  position_x = ds * cos(rad) + position_x;
  position_y = ds * sin(rad) + position_y;
  odd_distance_old = odd_distance;
}


TASK(Task_20ms)
{
  trace_control();
  tail_control();
  oddmetry();
  TerminateTask(); 
}

int trace_target;
void
trace_control(){
  int bright;
  float kp = 2.0;
  float ki = 0.00007;
  float kd = 5.000;
  int err_p,err_d;
  static int err_int = 0;
  static int err_old = 0;
  int steering;
  if(trace_mode == TRACE_ON){
    bright = get_light_sensor();
    err_p = trace_target - bright;
    err_int += err_p;
    err_d = err_p - err_old;
    err_old = err_p;
    steering = (int)((kp * err_p) + (ki * err_int) + (kd * err_d));
    steering = limit_value(steering, 100, -100);
    cmd_turn = steering;
  } else {
    cmd_turn = 0;
  }
}
int tail_target;
void
tail_control(){
  int angle;
  angle = nxt_motor_get_count(PORT_MOTOR_TAIL);
  if(angle < tail_target){
    nxt_motor_set_speed(PORT_MOTOR_TAIL, 57, 0);
  } else {
    nxt_motor_set_speed(PORT_MOTOR_TAIL, -50,0);
  }                 
}
int get_light_sensor(void){
  int bright;
  bright = 1023 - ecrobot_get_light_sensor(PORT_LIGHT);
  return(bright);
}
int
limit_value(int a, int max, int min){
  if(a > max) a = max;
  if(a < min) a = min;
  return(a);
}
int sonar_distance ;

void sonar_agent(void){
  sonar_distance = ecrobot_get_sonar_sensor(PORT_SONAR);
}
TASK(Task_100ms)
{
  sonar_agent();
  localization();
  ecrobot_bt_data_logger(cmd_forward, cmd_turn);
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

  trace_mode = TRACE_OFF;
  tail_target = 0;
  systick_wait_ms(5000U);
  cmd_forward = 0;

  light_calibration();
  tail_target = 109;

  //wait_touch(500);
  while(sonar_distance > 30){
    systick_wait_ms(300U);
  } 
  nxtway_gs_mode = INIT_MODE;
  systick_wait_ms(1000U);

  tail_target = 0;

  trace_mode = TRACE_ON;

  cmd_forward = 20;
  systick_wait_ms(2000U); 

  cmd_forward = 50;

  while(odd_distance < 3000){
    systick_wait_ms(500U);
  }
  cmd_forward = 50;

  while(1){
    systick_wait_ms(1000U);
  }
}

void wait_touch(int freq){
  while(ecrobot_get_touch_sensor(PORT_TOUCH)==0){systick_wait_ms(100U);}  //wait for pushing switch
  while(ecrobot_get_touch_sensor(PORT_TOUCH)==1){systick_wait_ms(100U);}  //wait for releasing switch
  ecrobot_sound_tone(freq,100,50);
}

void light_calibration(){
  int bright_white, bright_black;
  wait_touch(800);
  bright_white = get_light_sensor();
  display_goto_xy(0,0); display_int(bright_white, 6); display_update();
  wait_touch(bright_white);
  bright_black = get_light_sensor();
  display_goto_xy(0,1); display_int(bright_black, 6); display_update();
  trace_target = (int)((bright_white + bright_black) / 2.0);
}