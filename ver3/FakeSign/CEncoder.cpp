#include "CEncoder.h"

/* 
  External Interrupt in openCR
  EXTI    Arduino
  0    ->    2
  1    ->    3
  2    ->    4    
  3    ->    7
  4    ->    8

  5    ->    42  // if you want to use this pin, you should not use int pin 0,2
  6    ->    45  // Do not work
  7    ->    72
  8    ->    75
*/

/* ISR Definition */
void CEncoder::DecodeISR0 ()
{
  if (CEncoder::instances[0] != NULL)
    CEncoder::instances[0]->Decode();
} 

void CEncoder::DecodeISR1 ()
{
  if (CEncoder::instances[1] != NULL)
    CEncoder::instances[1]->Decode();
} 

void CEncoder::DecodeISR2 ()
{
  if (CEncoder::instances[2] != NULL)
    CEncoder::instances[2]->DecodeB();
} 

void CEncoder::SW0_ISR()
{
  if (CEncoder::instances[2] != NULL)
    CEncoder::instances[2]->sw0_isr();
}

void CEncoder::SW1_ISR()
{
  if (CEncoder::instances[2] != NULL)
    CEncoder::instances[2]->sw1_isr();
}


/* ISR Execution */
void CEncoder::sw1_isr()  // 눌렸을 때, 처음 시작할 때 스위치가 눌려있어서 이동 안하는 문제가 있음 -> 플래그로 초기화시켜줘야될듯!!
{
  if (digitalRead(75)==HIGH){
    sw1_flag = true;
  }
  if (digitalRead(75)==LOW && sw1_flag){
    pwm = 0;
    // pulse_count = 5200;
    // input_B = 5200;
    pulse_count = 5600;
    input_B = 148;
    sw1_flag = false;
    dir = 1;
    // input_B = 5200;
  }
}

void CEncoder::sw0_isr()
{
  if (digitalRead(3)==HIGH){
    sw0_flag = true;
  }
  if (digitalRead(3)==LOW && sw0_flag){
    pwm = 0;
    pulse_count = 0;
    input_B = 0;  // 순서 주의
    sw0_flag = false;
    // input_B = 0;

    if (initEncoder==0) initEncoder = 1;
    dir = 0;
  }
}


/* Motor || motor_num = {0:right, 1:left, 2:ball_screw} */
void CEncoder::begin(uint8_t PinA_, uint8_t PinB_, int motor_num)
{  
  // motor_num = {0:right, 1:left, 2:ball_screw}

  pinA = PinA_;
  pinB = PinB_;

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  switch (pinA)
  {
  case 2: 
    attachInterrupt (0, DecodeISR0, CHANGE);
    attachInterrupt (2, DecodeISR0, CHANGE);
    instances [0] = this;
    break;

  case 4:
    attachInterrupt (0, DecodeISR0, CHANGE);
    attachInterrupt (2, DecodeISR0, CHANGE);
    instances [0] = this;
    break;
  
  case 7: 
    attachInterrupt (3, DecodeISR1, CHANGE);
    attachInterrupt (4, DecodeISR1, CHANGE);
    instances [1] = this;
    break;

  case 8: 
    attachInterrupt (3, DecodeISR1, CHANGE);
    attachInterrupt (4, DecodeISR1, CHANGE);
    instances [1] = this;
    break;

  case 45: 
    attachInterrupt (6, DecodeISR2, CHANGE);
    attachInterrupt (7, DecodeISR2, CHANGE);

    attachInterrupt (1, SW0_ISR, CHANGE);
    attachInterrupt (8, SW1_ISR, CHANGE);
    instances[2] = this;
  break;  

  case 72: 
    attachInterrupt (6, DecodeISR2, CHANGE);
    attachInterrupt (7, DecodeISR2, CHANGE);
    instances[2] = this;
  break;  

  case 75: 
    attachInterrupt (7, DecodeISR2, CHANGE);
    attachInterrupt (8, DecodeISR2, CHANGE);
    instances[2] = this;
  break;  
  }

  if (motor_num == 1) // left
  {
    left_motor_dir = true;
    left_motor_PID = true;
    P_GAIN_ = P_GAIN_L;
    I_GAIN_ = I_GAIN_L;
    D_GAIN_ = D_GAIN_L;
  }
  else if (motor_num == 0) // right
  {
    left_motor_dir = false;  // setting miss
    left_motor_PID = false;
    P_GAIN_ = P_GAIN_R;
    I_GAIN_ = I_GAIN_R;
    D_GAIN_ = D_GAIN_R;
  }
}

void CEncoder::Decode()
{  
  uint8_t pinA_state, pinB_state;

  pinA_state= digitalRead(pinA);
  pinB_state= digitalRead(pinB);
  uint8_t newState=(pinA_state<<1|pinB_state);
  switch((_previousState <<2)|newState){
    case 0b0001: // 0x01 PREV|CURR
    case 0b0111: // 0x07
    case 0b1110: // 0x0E
    case 0b1000: // 0x08
            {pulse_count--; break;}
    case 0b0010: // 0x02
    case 0b1011: // 0x0B
    case 0b1101: // 0x0D
    case 0b0100: // 0x04
            {pulse_count++; break;}
  default:
      break;
  }
  _previousState = newState;
}

void CEncoder::DecodeB()
{  
  uint8_t pinB_state;

  pinB_state = digitalRead(pinB);

  if (pinB_state != prev_pinB_state){
    if (dir == 0)
    {
      pulse_count++;
    }
    else
    {
      pulse_count--;
    }
  }
  prev_pinB_state = pinB_state;
}

void CEncoder::PrintPulse()
{
  Serial.print(pulse_count);
  Serial.print('\t');
  Serial.println(dir);
}

void CEncoder::CurrentVel(float time_interval)
{ 
  rpm = 60./((double)ENCODER_INTERVAL/1000.) * (double)pulse_count / (13. * 4.) / MOTOR_RATIO;  // calculate RPM
  ang_vel = rpm * RPMtoRAD;  // calculate rad/s

  current = abs(ang_vel);  // current rad/s
  
  pulse = pulse_count;
  pulse_count = 0;  // reset pulse count
}

double CEncoder::CMDVELtoTarget(double lin_x_, double ang_z_)
{
  double angVel_rotate = constrain((ang_z_*WHEEL_SEPARATION/2),-ANG_SPEED_LIMIT,ANG_SPEED_LIMIT);    
  double dir_term=left_motor_dir==1?1.:-1.;
  double angVel_linear = constrain(lin_x_, -LIN_SPEED_LIMIT, LIN_SPEED_LIMIT);  
  double angVel_merge = (angVel_rotate*dir_term + angVel_linear)/WHEEL_RADIUS;  // angVel_merge = angular velocity rotate term + angular velocity linear term
  
  dir=angVel_merge>0?0:1;
  target = abs(angVel_merge);
}

void CEncoder::EncoderPID()
{
  if (millis() - prev_pid_millis >= PID_INTERVAL)
  {
    double realError, errorGap;
    double pControl, iControl, dControl;
      
    realError = target - current;             //for P
    accError += realError;                    //for I
    if (current <= 0.07) accError = 0; 
  
    errorGap = target - current - realError;  //for D  
    pControl = P_GAIN_ * realError;
    iControl = I_GAIN_ * (accError * PID_INTERVAL / 1000);
    dControl = D_GAIN_ * (errorGap / PID_INTERVAL);

    iControl = constrain(iControl, -0.3, 0.3);

    dControl = prev_dControl*0.8 + dControl*0.2;
    prev_dControl = dControl;
    pidControl = current + pControl + iControl + dControl;

    prev_pid_millis = millis();
  }
}

uint8_t CEncoder::PIDtoPWM()
{
  int pidControl_to_pwm;
  
  pidControl_to_pwm = int((pidControl+0.58)*25./1.52);  //PID끈상태
  
  if (pidControl_to_pwm <= 15) pidControl_to_pwm = 0;
  pwm = constrain(pidControl_to_pwm, 0, 255);
}


/* ball screw */
void CEncoder::CountPulse_B()
{ 
  rpm = 60./((double)ENCODER_INTERVAL/1000.) * (double)pulse_count / (13. * 4.) / MOTOR_RATIO;  // calculate RPM
  ang_vel = rpm * RPMtoRAD;  // calculate rad/s

  current = abs(ang_vel);  // current rad/s
  
  pulse = pulse_count;
}

void CEncoder::InitPosition()
{
  dir = 1;
  pwm = 50;

  if (initEncoder==1)
  {
    dir = 0;
    pwm = 0;
    initEncoder = 2;
  }
}

void CEncoder::MoveEncoder_mm(uint16_t distance)
{
  int cur_pulse_count = 0;
  cur_pulse_count = abs(pulse_count);
  if (distance < 0) distance = 0;
  if (distance >= 148) distance = 148;

  target_position = int((float)distance*1000/27);

  if (target_position > cur_pulse_count)
  {
    dir = 0;
    pwm = map(abs(target_position - cur_pulse_count), 0, 5300, 75, 175);
  }
  else 
  {
    dir = 1;
    pwm = map(abs(target_position - cur_pulse_count), 0, 5300, 75, 175);
  }
  
  if (target_position-5<cur_pulse_count && target_position+5>cur_pulse_count)
  {
    pwm = 0;
  }

  mm_position = distance;

  if (pulse_count<0)
  {
    pwm = 0;
    input_B = 0;
    dir = 0;
    pulse_count = 0;
  }
}






