#ifndef Capstone_Encoder_H_
#define Capstone_Encoder_H_

#include <stdint.h>
#include <Arduino.h>

/* Parameters */
#define MOTOR_RATIO 49.
#define RPMtoRAD 0.10471975512
#define PULSEtoRAD 6.283185307/(13*4*49)
#define ENCODER_INTERVAL 10

#define WHEEL_RADIUS 0.0525  // 0.0525m = 52.5mm
#define WHEEL_SEPARATION 0.42    // distance from wheel to another wheel
#define LIN_SPEED_LIMIT 0.6  // set limitation
#define ANG_SPEED_LIMIT 1.0

/* Information -> 실제로 코드에서 사용하진 않음 */
#define WHEEL_MAX_ANG_VEL 15.4   // maximum angular velocity of wheels
#define MAX_LIN_VEL WHEEL_MAX_ANG_VEL * WHEEL_RADIUS  // maximum linear velocity (x-axis)
#define MAX_ANG_VEL WHEEL_RADIUS * WHEEL_MAX_ANG_VEL / (WHEEL_SEPARATION/2)  // maximum angular velocity (z-axis)

/* PID GAIN */
#define P_GAIN_L 1.
#define I_GAIN_L 3.
#define D_GAIN_L 0.
#define P_GAIN_R 1.
#define I_GAIN_R 3.
#define D_GAIN_R 0.
#define P_GAIN_B 1.
#define I_GAIN_B 3.
#define D_GAIN_B 0.
#define PID_INTERVAL 3  // PID control time

class CEncoder
{
  
  private:
    uint8_t pinA, pinB;
    uint8_t pinSW0, pinSW1;
    uint8_t pinA_state, pinB_state;
    
    double pidControl;
    bool left_motor_dir = true;
    bool left_motor_PID = true;
    double P_GAIN_, I_GAIN_, D_GAIN_;

    uint32_t prevMillis;
    int32_t prev_pulse_count;
    uint8_t _previousState;
    double prev_dControl = 0;
    double accError = 0;
    uint32_t prev_pid_millis = 0;

    static void DecodeISR0();  // left or right
    static void DecodeISR1();  // left or right
    static void DecodeISR2();  // ball screw
    static void SW0_ISR();
    static void SW1_ISR();

    bool prev_pinSW0_state, prev_pinSW1_state;
    uint16_t target_position = 0;
    byte cnt_ = 0;

    bool sw0_flag = false;
    bool sw1_flag = false;

    bool prev_pinB_state;

  public:

    bool check = false;
    byte initEncoder = 0;

    double rpm, ang_vel;

    volatile int32_t pulse_count = 0;  // real time encoder pulse
    
    int32_t pulse = 0;  // 우리가 실제로 사용할 펄스
    uint8_t pwm = 0;
    bool dir = 0;

    int16_t input_B = 0;

    double current, target;
    
    static CEncoder *instances[3];

    uint16_t mm_position = 0;

    /* Motor(wheel) functions */
    /*
      0. begin : attach instance with initial settings
      0. Decode : use Interrupt to generate pulses
      0. PrintPulse : for checking pulses  

    ` 1. CurrentVel : convert 'pulse' to 'angular velocity(rad/s)'
      2. CMDVELtoTarget : convert 'cmd_vel' to 'target angular velocity of each motor'
      3. EncoderPID : PID using 'current angular velocity' and 'target angular velocity'
      4. PIDtoPWM : convert 'PID controled angular velocity' to 'target PWM' 
    */

    void begin(uint8_t PinA_, uint8_t PinB_, int motor_num);  // set pins, dir
    void Decode();  // decode pulse count 
    void PrintPulse();  // check pulse
    
    void CurrentVel(float time_interval);  // for our system, I made another version of CurrentVel
    double CMDVELtoTarget(double lin_x_, double ang_z_);
    void EncoderPID();
    uint8_t PIDtoPWM();

    /* Ball screw functions */
    void sw0_isr();
    void sw1_isr();
    void DecodeB();
    void CountPulse_B();
    void InitPosition();
    void MoveEncoder_mm(uint16_t distance);
};


#endif