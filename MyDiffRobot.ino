// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS CONTROLLED WITH YOUR SMARTPHONE
// JJROBOTS BROBOT KIT: (Arduino Leonardo + BROBOT ELECTRONIC BRAIN SHIELD + STEPPER MOTOR drivers)
// This code is prepared for new BROBOT shield  with ESP8266 Wifi module
// Author: JJROBOTS.COM
// Date: 02/09/2014
// Updated: 25/06/2017
// Version: 2.82
// License: GPL v2
// Compiled and tested with Arduino 1.6.8. This new version of code does not need external libraries (only Arduino standard libraries)
// Project URL: http://jjrobots.com/b-robot-evo-2-much-more-than-a-self-balancing-robot (Features,documentation,build instructions,how it works, SHOP,...)
// New updates:
//   - New default parameters specially tuned for BROBOT EVO 2 version (More agile, more stable...)
//   - New Move mode with position control (for externally programming the robot with a Blockly or pyhton programming interfaces)
//   - New telemtry packets send to TELEMETRY IP for monitoring Battery, Angle, ... (old battery packets for touch osc not working now)
//   - Default telemetry server is 192.168.4.2 (first client connected to the robot)
//  Thanks to our users on the forum for the new ideas. Specially sasa999, KomX, ...

// The board needs at least 10-15 seconds with no motion (robot steady) at beginning to give good values... Robot move slightly when it´s ready!
// MPU6050 IMU connected via I2C bus. Angle estimation using complementary filter (fusion between gyro and accel)
// Angle calculations and control part is running at 100Hz

// The robot is OFF when the angle is high (robot is horizontal). When you start raising the robot it
// automatically switch ON and start a RAISE UP procedure.
// You could RAISE UP the robot also with the robot arm servo (Servo button on the interface)
// To switch OFF the robot you could manually put the robot down on the floor (horizontal)

// We use a standard PID controllers (Proportional, Integral derivative controller) for robot stability
// More info on the project page: How it works page at jjrobots.com
// We have a PI controller for speed control and a PD controller for stability (robot angle)
// The output of the control (motors speed) is integrated so it´s really an acceleration not an speed.

// Join the module Wifi Access Point (by default: JJROBOTS_XX) with your Smartphone/Tablet...
//   Wifi password: 87654321
//    PRO mode (PRO button). On PRO mode steering and throttle are more aggressive
//    PAGE2: PID adjustements [optional][dont touch if you dont know what you are doing...;-) ]

#include <Wire.h>

// Uncomment this lines to connect to an external Wifi router (join an existing Wifi network)
//#define EXTERNAL_WIFI
//#define WIFI_SSID "YOUR_WIFI"
//#define WIFI_PASSWORD "YOUR_PASSWORD"
//#define WIFI_IP "192.168.1.101"  // Force ROBOT IP
//#define TELEMETRY "192.168.1.38" // Tlemetry server port 2223

#define TELEMETRY "192.168.4.2" // Default telemetry server (first client) port 2223

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_ANGLE 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 780   // Max recommended value: 860
#define MAX_STEERING_PRO 260   // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26   // Max recommended value: 32

// Default control terms for EVO 2
#define KP 0.32
#define KD 0.050
#define KP_THROTTLE 0.080
#define KI_THROTTLE 0.1
#define KP_POSITION 0.06
#define KD_POSITION 0.45
//#define KI_POSITION 0.02

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1
#define KD_RAISEUP 0.16
#define KP_THROTTLE_RAISEUP 0   // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30   // Iterm windup constants for PI control 
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0  // Offset angle for balance (to compensate robot own weight distribution)

// Servo definitions
#define SERVO_AUX_NEUTRO 1500  // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 700
#define SERVO_MAX_PULSEWIDTH 2500

#define SERVO2_NEUTRO 1500
#define SERVO2_RANGE 1400

// Telemetry
#define TELEMETRY_BATTERY 1
#define TELEMETRY_ANGLE 1
//#define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!

#define ZERO_SPEED 65535
#define MAX_ACCEL 14      // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

#define MICROSTEPPING 16   // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

#define DEBUG 0   // 0 = No debug info (default) DEBUG 1 for console output

// AUX definitions
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

String MAC;  // MAC address of Wifi module

uint8_t cascade_control_loop_counter = 0;
uint8_t loop_counter;       // To generate a medium loop 40Hz
uint8_t slow_loop_counter;  // slow loop 2Hz
uint8_t sendBattery_counter; // To send battery status
int16_t BatteryValue;

long timer_old;
long timer_value;
float debugVariable;
float dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
float angle_adjusted_filtered = 0.0;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
bool newControlParameters = false;
bool modifing_control_parameters = false;
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float angle_offset = ANGLE_OFFSET;

boolean positionControlMode = false;
uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)
int prescalerM2;
int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2;        // Actual speed of motors
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed
volatile int8_t M2_int_flag = 0; 
#define ENABLE_MOTORS_PIN 8
#define DEBUG 0
// INITIALIZATION
void setup()
{
  // STEPPER PINS ON JJROBOTS BROBOT BRAIN BOARD
  pinMode(ENABLE_MOTORS_PIN, OUTPUT); // ENABLE MOTORS
  pinMode(2, OUTPUT); // STEP MOTOR 1 PORTE,6
  pinMode(5, OUTPUT); // DIR MOTOR 1  PORTB,4
  pinMode(3, OUTPUT); // STEP MOTOR 2 PORTD,6
  pinMode(6, OUTPUT); // DIR MOTOR 2  PORTC,6
  digitalWrite(ENABLE_MOTORS_PIN, HIGH);  // Disbale motors
//  pinMode(10, OUTPUT);  // Servo1 (arm)
//  pinMode(13, OUTPUT);  // Servo2

  Serial.begin(115200); // Serial output to console


  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.begin();

#if DEBUG > 0
  delay(9000);
#else
  delay(1000);
#endif
  Serial.println("JJROBOTS");
  delay(200);
  Serial.println("Don't move for 10 sec...");
  MPU6050_setup();  // setup MPU6050 IMU
  delay(500);

  // With the new ESP8266 WIFI MODULE WE NEED TO MAKE AN INITIALIZATION PROCESS
  Serial.println("WIFI init");
  delay(100);

#ifdef EXTERNAL_WIFI
  ESPsendCommand("AT+CWQAP", "OK", 3);
  ESPsendCommand("AT+CWMODE=1", "OK", 3);
  //String auxCommand = (String)"AT+CWJAP="+WIFI_SSID+","+WIFI_PASSWORD;
  char auxCommand[90] = "AT+CWJAP=\"";
  strcat(auxCommand, WIFI_SSID);
  strcat(auxCommand, "\",\"");
  strcat(auxCommand, WIFI_PASSWORD);
  strcat(auxCommand, "\"");
  ESPsendCommand(auxCommand, "OK", 14);
#ifdef WIFI_IP
  strcpy(auxCommand, "AT+CIPSTA=\"");
  strcat(auxCommand, WIFI_IP);
  strcat(auxCommand, "\"");
  ESPsendCommand(auxCommand, "OK", 4);
#endif
  ESPsendCommand("AT+CIPSTA?", "OK", 4);
#else  // Deafault : we generate a wifi network

  // Generate Soft AP. SSID=JJROBOTS, PASS=87654321
  char *cmd =  "AT+CWSAP=\"JJROBOTS_XX\",\"87654321\",5,3";
  // Update XX characters with MAC address (last 2 characters)
  cmd[19] = MAC[10];
  cmd[20] = MAC[11];
#endif
  // Start UDP SERVER on port 2222, telemetry port 2223


  // Calibrate gyros
  MPU6050_calibrate();

  // Init servos

  // STEPPER MOTORS INITIALIZATION
  // MOTOR1 => TIMER1
  TCCR1A = 0;                       // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8=> 1MHz //, => 2Mhz
  OCR1A = ZERO_SPEED;               // Motor stopped
  dir_M1 = 0;
  TCNT1 = 0; // 16-bit register

  // MOTOR2 => TIMER3
  TCCR2A = (1<<WGM21);// Timer0 CTC mode 2, OCxA,B outputs disconnected
  TCCR2B = (1<<CS20)| (1<<CS21) | (1<<CS22) ; // Prescaler=1024, => 15.625Khz
  prescalerM2 = 1024;
  OCR2A =ZERO_SPEED;//  0xFF;   // Motor stopped
  //  TCCR3A = 0;                       // Timer3 CTC mode 4, OCxA,B outputs disconnected
  //  TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
  //  OCR3A = ZERO_SPEED;   // Motor stopped

  dir_M2 = 0;
  TCNT2 = 0; // 8-bit register
  delay(200);

  // Enable stepper drivers and TIMER interrupts
  digitalWrite(ENABLE_MOTORS_PIN, LOW);   // Enable stepper drivers
  // Enable TIMERs interrupts
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
  //  TIMSK3 |= (1 << OCIE1A); // Enable Timer1 interrupt
  TIMSK2 |= (1<< OCIE2A); // Enable timer0 interrupt
  Serial.println("interrupts set");

  // Little motor vibration and servo move to indicate that robot is ready
  for (uint8_t k = 0; k < 5; k++)
  {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
    delay(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
    delay(200);
  }
  setMotorSpeedM1(0);
  setMotorSpeedM2(0);
  Serial.println("BROBOT by JJROBOTS v2.82");
  Serial.println("Start...");
  
  timer_old = micros();
}

// MAIN LOOP
void loop()
{
  timer_value = micros();

  //OCR1A = ZERO_SPEED/10000;
//  int top = 100;
//  setMotorSpeedM1(top);
//  setMotorSpeedM2(top);
//  delay(1000);
//  setMotorSpeedM1(top);
//  setMotorSpeedM2(top);
//  Serial.println(OCR2A);
//  
 
  // New IMU data?
  if (MPU6050_newData())
  {
    MPU6050_read_3axis();
    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value - timer_old) * 0.000001; // dt in seconds
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    float MPU_sensor_angle = MPU6050_getAngle(dt);
    angle_adjusted = MPU_sensor_angle + angle_offset;
    if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
      angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;

#if DEBUG==1
    Serial.print(dt);
    Serial.print(" ");
    Serial.print(angle_offset);
    Serial.print(" ");
    Serial.print(angle_adjusted);
    Serial.print(",");
    Serial.println(angle_adjusted_filtered);
#endif
    //Serial.print("\t");

    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed

#if DEBUG==2
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(estimated_speed_filtered);
#endif

    if (true)//positionControlMode)
    {
      // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // Convert from motor position control to throttle / steering commands
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);
    }

    // ROBOT SPEED CONTROL: This is a PI controller.
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output


#if DEBUG==3
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.print(estimated_speed_filtered);
    Serial.print(" ");
    Serial.println(target_angle);
#endif

    // Stability control (100Hz loop): This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly to the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    int angle_ready;
    angle_ready = 82;
    // angle_ready = 74;  // Default angle
    if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
    {
      // NORMAL MODE
      digitalWrite(4, LOW);  // Motors enable
      // NOW we send the commands to the motors
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
    }
    else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
    {
      digitalWrite(4, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // RESET steps
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
      throttle = 0;
      steering = 0;
    }


    // Servo2

    // Normal condition?
    if ((angle_adjusted < 56) && (angle_adjusted > -56))
    {
      Kp = Kp_user;            // Default user control gains
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
    }
    else    // We are in the raise up procedure => we use special control parameters
    {
      Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
    }

  } // End of new IMU data

  // Medium loop 7.5Hz
  if (loop_counter >= 15)
  {
    loop_counter = 0;
    // Telemetry here?
#if TELEMETRY_ANGLE==1
    char auxS[25];
    int ang_out = constrain(int(angle_adjusted * 10), -900, 900);
    sprintf(auxS, "$tA,%+04d", ang_out);
#endif
#if TELEMETRY_DEBUG==1
    char auxS[50];
    sprintf(auxS, "$tD,%d,%d,%ld", int(angle_adjusted * 10), int(estimated_speed_filtered), steps1);
#endif

  } // End of medium loop
  else if (slow_loop_counter >= 100) // 1Hz
  {
    slow_loop_counter = 0;
    // Read  status
    Serial.print(speed_M1);
    Serial.print("\t");
    Serial.println(speed_M2);
  }  // End of slow loop
}
