#ifndef FakeSign_H_
#define FakeSign_H_

#include "CEncoder.h"   // 엔코더모터(L, R, Ball screw)
#include "CRobotArm.h"  // 로봇팔
#include "CServo.h"     // 그리퍼 서보모터+압력센서

#include "turtlebot3_sensor.h"
#include <iostream>  
#include <sstream>    

/* Instances */
CEncoder* CEncoder::instances[3] = {NULL, NULL, NULL};
CEncoder CMotor_L, CMotor_R, CMotor_B;
CRobotArm robot;  // 로봇팔 
CServo cservo;  // 그리퍼(서보+압력)

/* functions */
void update_imu();  // 처음 init_imu 부분에서 imu 한 번 업데이트 후 시작  

/* Parameters */
#define ODOM_INTERVAL 10.5 //ms단위. 현재 100HZ인데, 10ms대신 10.1ms인 이유는 odom.py에서 10ms로 송신하는데, 수신부가 빠르게되면 버퍼가 쌓일 수 있으므로, 송신부의 속도보다 조금 느리게 수신. 만약 이렇게해도 버퍼(씹히는문제)발생시 odom 수신코드를 통신 인터럽트로 바꾸기.
#define PI 3.141592

/* pinSetting */
#define LED_PIN 13
#define motorDirL 12
#define motorDirR 11
#define motorPwmL 6
#define motorPwmR 9

#define motorPwmB 10
#define motorDirB A1
#define swPin0 72
#define swPin1 75

#define ENC_L_2 2
#define ENC_L_4 4
#define ENC_R_7 7
#define ENC_R_8 8
#define ENC_B_45 45
#define ENC_B_72 72  // 오! 치명적 오류

/* instances */
static Turtlebot3Sensor sensors;

/* Logic */
String s_pulse_R;  // convert wheel pulse to string operator
String s_pulse_L;
String s_delta_theta;
String cmd;
String logic_heading_angle;
String rotate_done;
String Last_signal="0";
int count=0;  //임시변수. 테스트용.

int logic_warn_sound;
int heading_dir;

/* MOTOR */
float linx;  // cmd_vel linear.x
float angz;  // cmd_vel angular.z 
float theta;  // IMU angle (Yaw)
float last_theta;
float delta_theta;
float theta_sum;
float heading_angle;

/* IMU */
bool initIMU_ret; 
float init_theta;

float gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z;
float mag_x, mag_y, mag_z;
float ori_x, ori_y, ori_z, ori_w;

/* odom */
uint32_t prev_time;
uint32_t reset_time;   // odom이 들어오는지 확인. 100ms 동안 안 들어오면 모터 속도 0으로 

int ball_screw;   //임시로 만든 변수. ball_screw==true면 창민이쪽에서 실행하면 됨. 이름은 창민이랑 통일하기. 이 변수 창민이쪽에 선언되게 되면 여기선 삭제해서 중복선언 피하기.

/* RobotArm */
void serialEvent1();
void serialEvent();
float data[8];
float jnt_ang[9]; // float형태

unsigned long now = 0;
unsigned long before = 0;

bool packet = false, sent = false;
bool robot_op = false, grab_begin = false; // 그리핑 처음에 성공하고, 나중에도 성공하는지 확인 
bool st_matlab = false, st_matlab2 = false, st_matlab3 = false;

int servo_ang_max = 120, servo_ang_min = 100; // max : 서보 최대 회전, min : 커질수록 오므라듦
bool grabed = false; // 통신을 통한 수동 그리핑 테스트
int ball_mm = 0, depth = 0; 
int depth_desire = 260; // 내가 원하는 만큼의 이격거리
bool forward = false;
bool ball_screw_only = false;
bool servo_only_open = false;



/* functions */
void setPinMode(){
  pinMode(motorDirL, OUTPUT);
  pinMode(motorDirR, OUTPUT);
  pinMode(motorDirB, OUTPUT);
  pinMode(motorPwmL, OUTPUT);
  pinMode(motorPwmR, OUTPUT);
  pinMode(motorPwmB, OUTPUT);

  pinMode(45, INPUT);
  pinMode(72, INPUT);

  pinMode(3, INPUT_PULLUP);
  pinMode(75, INPUT);  // external pullup
}

void init_imu(){
  initIMU_ret = sensors.init();
  sensors.initIMU();
  sensors.calibrationGyro();
  update_imu();
}

void update_imu()
{
  sensors.updateIMU();
  float* p_imu_data;

  p_imu_data = sensors.getImuAngularVelocity();
  gyro_x = p_imu_data[0];
  gyro_y = p_imu_data[1];
  gyro_z = p_imu_data[2];

  p_imu_data = sensors.getImuLinearAcc();
  acc_x = p_imu_data[0];
  acc_y = p_imu_data[1];
  acc_z = p_imu_data[2];

  p_imu_data = sensors.getImuMagnetic();
  mag_x = p_imu_data[0];
  mag_y = p_imu_data[1];
  mag_z = p_imu_data[2];

  /* ori_x, ori_y가 0이 안나와서 일단 0으로 고정시킴. 
  calibration문제이거나 실제로 opencr보드를 좀 기울게 설치해서 그러거나. */
  p_imu_data = sensors.getOrientation();
  ori_w = p_imu_data[0];
  ori_x = p_imu_data[1];
  ori_y = p_imu_data[2];
  ori_z = p_imu_data[3];
}

void setInitAngle(){
  init_theta = atan2f(ori_x * ori_y + ori_w * ori_z, 0.5f - ori_y * ori_y - ori_z  * ori_z);  //0.9-> 1.0이 아니라 0.9->0.10이 되어버린다. 아두이노 float->string 변환 안됨. 값이상해짐. 이부분 파이썬으로 옮기기.
  last_theta = init_theta;
  delta_theta = 0.;
  rotate_done = "0";
}

void init_serial(){
  cmd.reserve(200);
  Last_signal="0";
  count=0;
}

void update_theta(){
    theta = atan2f(ori_w * ori_z, 0.5f - ori_z  * ori_z); 
    delta_theta = theta - last_theta;
    last_theta = theta;
    
    rotate_done= "0";

    if(delta_theta>PI){  //정확히 -3.13xx에서 3.13xx로 회전할때.
      delta_theta = -1*((2*PI)-delta_theta);  
    }
    else if(delta_theta<-PI){
      delta_theta = (2*PI)-abs(delta_theta);
    }
}

void init_ballscrew(){
  if (digitalRead(3)==LOW)
  {
    CMotor_B.dir = 0;
    CMotor_B.initEncoder = 2;
  }
  else {
    while (CMotor_B.initEncoder != 2)
    {
      CMotor_B.InitPosition();
      digitalWrite(motorDirB, CMotor_B.dir);
      analogWrite(motorPwmB, CMotor_B.pwm);
    }
    CMotor_B.dir = 0;
  }
}

void rotate_heading_angle(){
  theta_sum += abs(delta_theta);
  if(theta_sum >= abs(heading_angle)-0.001){ //+-0.286도
    heading_angle=0.;
    angz = 0.;
    theta_sum=0;
    
    rotate_done= "1";
    count++;   //임시변수. 테스트용.
    
    CMotor_L.CMDVELtoTarget(linx, angz);
    CMotor_R.CMDVELtoTarget(linx, angz);
  }
}

void send_message_to_odom(){
    s_pulse_R = String(int(CMotor_R.pulse*1000));
    s_pulse_L = String(int(CMotor_L.pulse*1000));
    s_delta_theta = String(int(delta_theta*10000));

    Serial.println("s"+s_delta_theta+"a"+s_pulse_L+"b"+s_pulse_R+"c"+rotate_done+"d"+Last_signal+"e"); 
    //Last_signal변수 한번 보내고 초기화시켜주는거 잊지말기. 창민이측에서 해야함.
    Last_signal="0";  // 창민이쪽으로 확인패킷 보내기.

    // Serial.println("s"+s_delta_theta+"a"+s_pulse_L+"b"+s_pulse_R+"c"+String(int(gyro_x*1000)) \
    // +","+String(int(gyro_y*1000))+","+String(int(gyro_z*1000))+","+String(int(acc_x*1000))+"," \
    // +String(int(acc_y*1000))+","+String(int(acc_z*1000))+","+String(int(ori_x*1000))+","+ \
    // String(int(ori_y*1000))+","+String(int(ori_z*1000))+","+String(int(ori_w*1000))+"e");
}

void motor_write(){
  if (millis() - reset_time > 100){  
    // 통신이 100ms 이상 끊기면 모터 출력 0으로
    analogWrite(motorPwmL, 0);
    analogWrite(motorPwmR, 0);
  }
  else {
    /* (DC Motor) motor output */
    digitalWrite(motorDirL, !CMotor_L.dir);
    digitalWrite(motorDirR, !CMotor_R.dir);
    analogWrite(motorPwmL, CMotor_L.pwm);
    analogWrite(motorPwmR, CMotor_R.pwm); 
  }

  /* (Ball screw) motor output */
  digitalWrite(motorDirB, CMotor_B.dir);
  analogWrite(motorPwmB, CMotor_B.pwm);
}

void get_last_signal(){
  /* 유석이 마지막 자리 패킷 */
  if(robot.status == end_rob){ // end_rob =2 로봇팔 작업 끝났을 때
    Last_signal = String(end_rob);
    // Serial.println("Ylast = end_rob");
  }
  else if(robot.status == read_fail){ // read_fail = 1 못 읽었을때 
    Last_signal = String(read_fail);
    // Serial.println("Ylast= NRead");
  }
  else{ // 보통의 경우 
    Last_signal = String(0); 
    // Serial.print("Ylast=");Serial.println(Ylast);
  }
}


#endif