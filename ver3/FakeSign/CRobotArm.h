#ifndef Capstone_Robot_Arm_H_
#define Capstone_Robot_Arm_H_

#include <DynamixelWorkbench.h>
#include <math.h>
#include <string.h> 
#include <string>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""     
#endif   
#define BAUDRATE  1000000
#define pi  3.14159265359
#define DXL_ID    0
#define DXL_ID2   1
#define DXL_ID3   2
#define DXL_ID4   3
#define DXL_ID5   4
#define DXL_ID6   5
#define DXL_ID7   6
#define fail_grip 20 // const char형태임
#define succ_grip 21
#define end_rob 2
#define grabb 23
#define NRead 1 // 책을 못읽었을때 


class CRobotArm : public DynamixelWorkbench
{
  private:
    uint8_t dxl_id[7]  = {DXL_ID,DXL_ID2,DXL_ID3,DXL_ID4,DXL_ID5,DXL_ID6,DXL_ID7};
    const uint8_t handler_index = 0;
    const char *log;      
    uint16_t model_number0  = 0;
    bool result = false;

  public: 
    int status = 0;
    int32_t goal_position[7]; 
    void robot_init();
    uint32_t ang2pwm(float x);
    uint32_t ang2pwm_ext(float x);
    float pwm2ang(float x);
    void robot_run(bool& packet_,float *jnt_ang_);
    void robot_read();
};
#endif