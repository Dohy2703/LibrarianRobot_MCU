#include "CServo.h"

/* FSR sensor */
void CServo::attachFSR(uint8_t fsrPin_)
{
  fsrPin = fsrPin_;
}

void CServo::checkFSR()
{
  if (millis() - prev_fsr_millis > 10)
  {
    fsrRead = map(analogRead(fsrPin), 0, 1024, 0, 255);  

    fsrRead = 0.3*fsrRead + 0.7*prev_fsrRead;

    /* check FSR sensor detection and stop servo motor */
    // if (grab_mode == 2 && fsrRead > 50 && grab_flag1 == true && grab_ON)
    // {
    //   grab_flag1 = false;   // 노이즈로 인한 압력센서의 최고값을 방지하기 위해 두 번 확인 (두 번째) 
    //   isGrab = true;

    //   grab_mode = 3;
    // }
    if (grab_mode == 2 && fsrRead > 50 && grab_ON)
    {
      // grab_flag1 = true;  // 노이즈로 인한 압력센서의 최고값을 방지하기 위해 두 번 확인 (첫 번째)
      isGrab = true;
      grab_mode = 3;
    }      
    else if (isGrab==true && fsrRead < 50 && !grab_ON)
    {
      isGrab=false;
    }

    prev_fsrRead = fsrRead;
    // prev_fsr_millis = millis();
  }
}

void CServo::printFSR()
{
  // Serial.print(0);
  Serial.println(fsrRead);
}


/* servo motor */
void CServo::setAngleLimit(uint8_t min_angle_, uint8_t max_angle_)
{
  min_angle = min_angle_;
  max_angle = max_angle_;
}

void CServo::setInitAngle(uint8_t init_angle_, bool inc_)
{
  /* you should use CServo::attach() and CServo::setAngleLimit()
     before using this member function */
  inc = inc_;  // increase or decrease angle when grab
  init_angle = constrain(init_angle_, min_angle, max_angle);
  this->write(init_angle);
}

void CServo::setGrabPeriod(uint16_t period_)
{
  /* you should use CServo::setAngleLimit()
     before using this member function */
  if (period_ == 0) ang_diff_period = 1000;
  else  
    ang_diff_period = period_;
}

void CServo::gripperOpen()
{
  grab_ON = true;  
  grab_mode = 0;
}


void CServo::gripperClose()
{
  grab_ON = true;  
  grab_mode = 2;
  inc = true;
  prev_millis = millis();
}


void CServo::grab() // 루프를 돌면서 계속 갱신되는 부분
{
  if (grab_ON == false)
  {
    return;
  }

  unsigned long time_gap = millis() - prev_millis;
  int inc_sign = inc?1:-1;  // default : 1

  switch (grab_mode)
  {
    case 0:   // set init time and ready for grab
      grab_mode = 1;
      this->write(init_angle);  // 90 : 오므리고 있음
      prev_millis = millis();
      inc = false;  // 벌리는 방향
      break;

    case 1:   // opening  90 -> 0
    
      inc_angle = constrain(int( (max_angle - min_angle) * time_gap / ang_diff_period ), 0, 180); // *2 부분은 겉멋용
      angle = constrain(init_angle + inc_angle*inc_sign, min_angle, max_angle);  

      this->write(angle);
  
      prev_angle = angle;

      if ( (inc == true && angle >= target) || (inc == false && angle <= target) )
      {
        // grab_mode = 2;
        // prev_millis = millis();
        // inc = true;  // 오므리는 방향
        grab_ON = false;
      }
      break;

    case 2:   // grab   0 -> 90
      inc_angle = constrain(int( (max_angle - min_angle) * time_gap / ang_diff_period ), 0, 180);
      angle = constrain(target + inc_angle*inc_sign, min_angle, max_angle);  

      this->write(angle);
  
      prev_angle = angle;

      if ( (inc == true && angle >= init_angle))
      {  // failed to catch anything
        grab_mode = 0;
        grab_ON = false;
        prev_millis = millis();
      }
      break;

    case 3:   // succeeded to grab something
      this->write(prev_angle);

      grab_ON = false;
      isGrab = true;
      break;

    case 4:   // succeeded to grab something
      inc_angle = constrain(int( (max_angle - min_angle) * time_gap / ang_diff_period ), 0, 180);
      angle = constrain(prev_angle_save + inc_angle*inc_sign, min_angle, max_angle);

      this->write(angle);
  
      prev_angle = angle;

      if ((inc == false && angle <= target) )
      {
        grab_mode = 2;
        prev_millis = millis();
        inc = true;  // 오므리는 방향
      }

      break;

  }
}

void CServo::grabReset(){
  grab_mode = 4;
  inc = false;
  prev_angle_save = prev_angle;
  grab_ON = true;
  grab_flag1 = false;
  prev_millis = millis();
  isGrab=false;
}

