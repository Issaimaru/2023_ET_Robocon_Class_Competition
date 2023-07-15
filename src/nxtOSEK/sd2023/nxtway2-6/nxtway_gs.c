 /* Linetrace program
  PID control
  stanging start with tail bar
  */
  /* User:Issaimaru*/
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
  INIT_MODE, 	 /* system initialize mode */
  CAL_MODE,  	 /* gyro sensor offset calibration mode */
  CONTROL_MODE /* balance and RC control mode */
} MODE_ENUM;

MODE_ENUM nxtway_gs_mode = WAIT_MODE; /* NXTway-GS mode at initial state */

typedef enum{
  TRACE_ON,
  TRACE_OFF,
} TRACE_MODE;

TRACE_MODE trace_mode = TRACE_OFF;

typedef enum{
  DIRECTION_ON,
  DIRECTION_OFF,
} DIRECTION_MODE;
DIRECTION_MODE direction_mode = DIRECTION_OFF;

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
  S8	pwm_l, pwm_r;
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
  display_goto_xy(0,0); display_int((int)position_x, 6);
  display_goto_xy(0,1); display_int((int)position_y, 6);
  display_goto_xy(0,2); display_int((int)robot_a, 6);
  display_update();
}



TASK(Task_20ms)
{
  trace_control();
  tail_control();
	oddmetry();
  TerminateTask(); 
}

int trace_target;
float kp,ki,kd;

void
trace_control(){
  int bright;
  
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
    steering = limit_value(steering, 120, -120);
    cmd_turn = steering;
  } else {
    cmd_turn = 0;
  }
}
int tail_target,n;
void
tail_control(){
  int angle;
  angle = nxt_motor_get_count(PORT_MOTOR_TAIL);
  if(angle < tail_target){
    nxt_motor_set_speed(PORT_MOTOR_TAIL, n, 0);
  } else {
    nxt_motor_set_speed(PORT_MOTOR_TAIL, -55,0);
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

float direction_target = 0.0;

void go_direction(void){
   kp = 2.0;
   ki = 0.0;
   kd = 0.0;
  int err_p,err_d;
  static int err_int = 0;
  static int err_old = 0;
  int steering;
  if(direction_mode == DIRECTION_ON){
    err_p = robot_a - direction_target;
    err_int += err_p;
    err_d = err_p - err_old;
    err_old = err_p;
    steering = (int)((kp * err_p) + (ki * err_int) + (kd * err_d));
    steering = limit_value(steering, 120, -120);
    cmd_turn = steering;
  } 
}
TASK(Task_100ms)
{
  sonar_agent();

  ecrobot_bt_data_logger(cmd_forward, cmd_turn);
  if(nxtway_gs_mode == CONTROL_MODE){
    if(ecrobot_get_touch_sensor(PORT_TOUCH)){
      nxtway_gs_mode = WAIT_MODE;
    }
  }
  TerminateTask(); 
}

void sound(int freq,int duration,int volume){//�͈� 31-2100[Hz]�C256(2.56[sec])�C0-100
    ecrobot_sound_tone(freq,duration,volume); //�����o����
}

void LineTrace(){//status:動作確認済み
  	cmd_forward = 30;
	kp = 1.35;
    ki = 0.00001;
    kd = 14.000;
	
	systick_wait_ms(2000U);

  cmd_forward = 60;
	kp = 0.65;
    ki = 0.00001;
    kd = 14.000;
	
  systick_wait_ms(9000U);//9000
	kp = 0.90;//0.90
    ki = 0.00001;
    kd = 14.000;
	 cmd_forward = 120;//122
 	systick_wait_ms(11000U);
	
	//trace_target = 480;
	 cmd_forward = 42;//42
	kp = 1.30;
    ki = 0.00013;//10
    kd = 13.000;
	systick_wait_ms(5000U);
}

void seesaw(){//status:動作確認済み
    int distance=ecrobot_get_sonar_sensor(PORT_SONAR);//�����g�Z���T���狗�����擾
    
    while(distance>30){//�Q�[�g�̒��O�܂Ői�ނ�
        systick_wait_ms(100U);
    	display_goto_xy(0,2); display_int(distance, 6); display_update();
        distance=ecrobot_get_sonar_sensor(PORT_SONAR);//�����g�Z���T���狗�����擾
    }
    sound(500,100,100);//�����o���Ēm�点���
    int gate_distance=odd_distance;//�Q�[�g�̈ʒu��c��
    
	kp = 0.65;
    ki = 0.00001;
    kd = 14.000;
	
    int seesaw_distance=650;//�Q�[�g����V�[�\�[�܂ł̋���[cm]
    cmd_forward = 85; //���x�̕ύX
    while((odd_distance-gate_distance) < seesaw_distance){//�V�[�\�[�̒��O�܂Ői��
    	display_goto_xy(0,2); display_int(((odd_distance-gate_distance)), 6); display_update();
        systick_wait_ms(100U);
    }
  
    sound(800,100,100);//�����o���Ēm�点���
    seesaw_distance=odd_distance;//�V�[�\�[�̈ʒu��c��
    
    while((odd_distance-seesaw_distance)<200){//�V�[�\�[�̒��S�܂Ői��
        systick_wait_ms(100U);  
    }
    cmd_forward=30;
    sound(1600,100,100);//�����o���Ēm�点���
    
    int centor_distance=odd_distance;//���S�̈ʒu��c��
    while((odd_distance-centor_distance)<400){//�V�[�\�[�������܂Ői��
        systick_wait_ms(100U); 
    }
    sound(2000,100,100);//�����o���Ēm�点���
    cmd_forward=70;
    int edge_distance=odd_distance;//�V�[�\�[�̒[�̈ʒu��c��
	
    while((odd_distance-edge_distance)<600){
      systick_wait_ms(100U);
    }
	kp = 0.65;
    ki = 0.00001;
    kd = 14.000;
	cmd_forward=30;
}

void Groping(){//status:動作確認未完了
	systick_wait_ms(1000U);
	
	cmd_forward=15;
	
	for(int i=0;i<5;i++){
		cmd_forward*=-1;
		systick_wait_ms(1000U);
	}
	sound(1600,100,100);
	
	trace_mode = TRACE_OFF;
	
	cmd_forward=120;
	
	int group_distance=odd_distance;
	while((odd_distance-group_distance)<700){//�����������Œ���
		systick_wait_ms(30U);
	}
	sound(1600,100,100);
	
	kp = 0.65;
    ki = 0.00001;
    kd = 14.000;
	cmd_forward=30;
	
	trace_mode = TRACE_ON;
}

void Limbo(){
  cmd_forward = 30;
	systick_wait_ms(500U);
  tail_target = 109;
	while(sonar_distance > 30){
		systick_wait_ms(500U);
	}
	
	cmd_forward = 0;
	systick_wait_ms(500U);
	trace_mode = TRACE_OFF;
	
	nxtway_gs_mode = WAIT_MODE;
	for(int i=109;i>=58;i-=3){
		tail_target = i; systick_wait_ms(900U);
	}//機体を傾ける

  int limbo_distance=odd_distance;

	trace_mode = TRACE_ON;
	cmd_forward = 20;
	
	sound(900,100,50);
	
  while((odd_distance-limbo_distance)<600){
    systick_wait_ms(30U);
  }

  sound(1600,100,100);

  cmd_forward = 0;
	systick_wait_ms(500U);
	trace_mode = TRACE_OFF;

  nxtway_gs_mode = WAIT_MODE;
	for(int i=55;i<108;i+=5){
		tail_target = i; systick_wait_ms(900U);
	}//機体を元の角度に戻す

  sound(900,100,50);

  trace_mode = TRACE_ON;
	cmd_forward = 0;
}

TASK(Task_Background)
{
  display_clear(1);

  trace_mode = TRACE_OFF;
  tail_target = 0;
  systick_wait_ms(2000U);
  cmd_forward = 0;

  light_calibration();
  tail_target = 109; n=62;

  //wait_touch(500);
  while(sonar_distance > 30){
    systick_wait_ms(1000U);
  }
	
  nxtway_gs_mode = INIT_MODE;
  systick_wait_ms(1000U);
	
  tail_target = 0;
	
  trace_mode = TRACE_ON;
	
	//ここからロボコン用の動作

  LineTrace();//担当者:鈴木
	 	
	seesaw();//担当者:竹内
	
	Groping();//担当者:勝田，竹内
	
  Limbo();//担当者:竹内，赤祖父

	while(true){
		systick_wait_ms(30);
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