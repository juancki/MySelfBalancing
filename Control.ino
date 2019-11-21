// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// Control functions (PID controls, Steppers control...)

// PD controller implementation(Proportional, derivative). DT in seconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}


// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
  return (output);
}


float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * float(speedM);
  return (output);
}

// TIMER 1 : STEPPER MOTOR1 SPEED CONTROL
ISR(TIMER1_COMPA_vect)
{
  if (dir_M1 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTD,2);// SET(PORTE, 6); // STEP MOTOR 1
  //delay_1us();
  if (dir_M1 > 0)
    steps1--;
  else
    steps1++;
  CLR(PORTD,2);//CLR(PORTE, 6);
  // Serial.println(dir_M1);
}
// TIMER 0 : STEPPER MOTOR2 SPEED CONTROL
ISR(TIMER2_COMPA_vect)
{

  if (dir_M2 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTD,3);//SET(PORTD, 6); // STEP MOTOR 2
  //delay_1us();
  if (dir_M2 > 0)
    steps2--;
  else
    steps2++;
  CLR(PORTD, 3);
  //Serial.println(dir_M2);
}


// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse)
void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  long speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M1 * 16; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M1 * 8; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 1000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    SET(PORTD,5);//SET(PORTB, 4); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 1000000 / -speed;
    dir_M1 = -1;
    CLR(PORTD,5);//CLR(PORTB, 4); // Dir Motor 1
  }
//  Serial.print("M1: ");
//  Serial.print(timer_period);
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR1A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT1 > OCR1A)
    TCNT1 = 0;
  
//  Serial.print("M1: ");
//  Serial.println(OCR1A,HEX);
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)
void setMotorSpeedM2(int16_t tspeed)
{

  long timer_period;
  long speed;
  tspeed = -tspeed;
  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL/3)
    speed_M2 -= MAX_ACCEL/3;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL/3)
    speed_M2 += MAX_ACCEL/3;
  else
    speed_M2 = tspeed;
    
#if MICROSTEPPING==16
  speed = speed_M2 * 16; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M2 * 8; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 1000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    CLR(PORTD, 6);   // Dir Motor2 (Forward)
  }
  else
  {
    timer_period = 1000000 / -speed;
    dir_M2 = -1;
    SET(PORTD, 6);  // DIR Motor 2
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;
// We use both the prescaler and the TCT OCR2A (8bits) to set the timer_period.
  long q = 8*(timer_period+1);
//  if ((q/prescalerM2)>256 && (q/prescalerM2)>256){ // If the
//    int q2 = q/256;
//    if (q2<8){
//      prescaler =
//    }
//    OCR2A = (q/prescalerM2)-1;
//  }
  
  OCR2A = (q/prescalerM2)-1;

//  Serial.print("M2: ");
//  Serial.print(timer_period);
  // Check  if we need to reset the timer...
  if (TCNT2 > OCR2A)
    TCNT2 = 0;
//  Serial.print("M2: ");
//  Serial.println(OCR2A,HEX);
}
