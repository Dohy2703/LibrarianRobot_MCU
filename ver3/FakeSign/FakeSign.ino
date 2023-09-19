#include "FakeSign.h"   // 기본 헤더파일

void setup() {
  /* Serial setting */
  Serial1.begin(115200);
  Serial1.setTimeout(1);
  Serial.end();
  Serial.begin(9600);
  init_serial();

  /* CServo setting */
  jnt_ang[0] = 0; // 초기 status =0 - 대기
  cservo.attach(5);
  cservo.setAngleLimit(0, servo_ang_max);
  cservo.setInitAngle(servo_ang_min, true);  // init angle, inc(+:1, -:0)  // 120 : 오므림
  cservo.setGrabPeriod(3000);  // set between 0~65535, recommended : 1000
  cservo.attachFSR(A0); 

  /* RobotArm setting */
  robot.robot_init();

  /* pinMode setting */
  setPinMode();

  /* IMU setting */
  init_imu();
  update_imu();
  setInitAngle();
  
  /* DC Motor pin setting */
  CMotor_L.begin(ENC_L_2, ENC_L_4, 1);   // int pin1, int pin2, motor_num
  CMotor_R.begin(ENC_R_7, ENC_R_8, 0);
  CMotor_B.begin(ENC_B_45, 72, 2);
  
  /* Ball screw position initialize */
  init_ballscrew();
}


void loop() {
  if(millis() - prev_time >= ODOM_INTERVAL){
    CMotor_L.CurrentVel(millis() - prev_time);
    CMotor_R.CurrentVel(millis() - prev_time);
    
    update_imu();
    update_theta();

    if(heading_angle!=0.){  // 서가 진입 전 IMU로 차체 회전
      rotate_heading_angle();
    }
    
    send_message_to_odom();

    prev_time = millis();
  }
    
  /* (DC Motor) Encoder pid */
  CMotor_L.EncoderPID();
  CMotor_R.EncoderPID();

  /* (DC Motor) convert pid result to PWM */
  CMotor_L.PIDtoPWM();
  CMotor_R.PIDtoPWM();

  /* (Ball screw) position control */
  CMotor_B.MoveEncoder_mm(CMotor_B.input_B);

  /* motor output (CMotor_L, CMotor_R, CMotor_B) */
  motor_write(false); // 인자 : 볼 스크류 사용 여부
}


/* Serial Communication Read */
void serialEvent() {
  // s{linx},{angz}a{sound_logic}b{heading_angle}c{ball_screw}e
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    // initialize OpenCR if Nav2 would be restarted
    if (inChar=='S'){  
      rotate_done = "0";
      Last_signal="0";
      count=0;   //임시변수. 테스트용.
      return;
    }

    // add it to the inputString:
    cmd += inChar;

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      if(cmd.startsWith("s") && cmd.endsWith("e\n")){   //창민이꺼 로봇팔 동작할때는 시작문자를 다른거써서 프로토콜 바꿔사용하기.
        // Serial.println(cmd);
        linx = cmd.substring(1,cmd.indexOf(',')).toFloat();
        
        if(linx < 0.04 and linx > 0.0){  //모터 데드존 고려.
          linx=0.04;
        }
        else if(linx > -0.04 and linx < 0.0){
          linx=-0.04;
        }

        angz = cmd.substring(cmd.indexOf(',')+1,cmd.indexOf('a')).toFloat();
        logic_warn_sound = cmd.substring(cmd.indexOf('a')+1,cmd.indexOf('b')).toInt();
        heading_angle = cmd.substring(cmd.indexOf('b')+1,cmd.indexOf('c')).toFloat();
        ball_screw= cmd.substring(cmd.indexOf('c')+1,cmd.length()-1).toInt();  //이제 추가로 if ball_screw일때 로봇팔 동작하도록 코드짜기.
        // Serial.print(String(int(heading_angle*1000)));
        if (logic_warn_sound==1){
          sensors.makeMelody(1);
          sensors.onMelody();
          CMotor_R.pulse=0;
          CMotor_L.pulse=0;
          cmd = "";
          return;
        }
        if (heading_angle!=0.){
          if(heading_angle>0.) angz=0.3;
          else angz = -0.3;
          theta = atan2f(ori_w * ori_z, 0.5f - ori_z  * ori_z);  //0.9-> 1.0이 아니라 0.9->0.10이 되어버린다. 아두이노 float->string 변환 안됨. 값이상해짐. 이부분 파이썬으로 옮기기.
          last_theta = theta;
          theta_sum = 0;
        }

        if (count==2 && ball_screw==1){   //임시변수. 테스트용. 헤딩액션 2번하면 피드백한번. 
          Last_signal="1";
          }
        if (count==3){
          Last_signal="2";
          count++;
        }
        CMotor_L.CMDVELtoTarget(linx, angz);
        CMotor_R.CMDVELtoTarget(linx, angz);
      }
      cmd = "";
    }
  }

  odom_time = millis();
}