#include "flipperservo.h"
#include <Wire.h>
#define PID_P 1.0
#define PID_I 0.0
#define PID_D 0.0
#define MAX_SPEED 63

void FlipperServo::initialize(byte motor_num)
{
  frequency = 0;
  zero_direction = 0;
  calibrated_zero = calibrated_zeros[motor_num];
  reversed = reversed_rotation[motor_num];
  reversed_sense = reversed_sensing[motor_num];
  amplitude = 0;
  phase_offset = 0;
  hbridge.initialize(hbridge_i2c_addresses[motor_num]);
#ifdef SIMULATION
  servo_position = 0;
#else
  feedback.initialize(feedback_chipselect_pins[motor_num]);
  feedback.begin();
  servo_position = reversed_sense*feedback.singleRead();
  Serial.print(motor_num);
  Serial.print(": ");
  Serial.print(feedback_chipselect_pins[motor_num]);
  Serial.print(" ");
  Serial.println(feedback.singleRead());

#endif

#ifdef CALIBRATE_MOTORS
  calibrated_zero = reversed_sense*servo_position;
#endif
}

void FlipperServo::update()
{
#ifndef SIMULATION
  // Non-simulated operation will use the feedback.singleRead() method to get current servo_position feedback
  servo_position = reversed_sense*feedback.singleRead();

#endif

  // here you will write code that calculates the desired position for each motor 
  // (for the shortest path to achieve it, you don't want the motor to perform unnecessary turns)
  
  // calculate phase based on amplitude, frequency and phase_offset 
  // you can use the function millis() to get current time in milliseconds
  float temp_time = fmod( millis()/1000.0 , 1.0/frequency ); // calculate time step for phase
  float phase = (amplitude/2) * sin(2*PI*frequency*temp_time + phase_offset*RADIANS_PER_TICK);

  // calculate the motor's desired_position given phase, amplitude and calibrated_zeros
  // use the reversed_rotation matrix to correctly orient all motors
  int desired_position = reversed*(zero_direction + phase) - calibrated_zero; 
  
  // avoid extra turns of the actuator
  desired_position %= 1024;

  // calculate shortest_path as the shortest distance between desired_position and servo_position. 
  int shortest_path = (desired_position - servo_position) % 1024;
  
  // shortest path is always in the -512...511 range. ensures correct direction of rotation
  // avoid multiple turns of the motor
  while(shortest_path > 511)  
    shortest_path -= 1024;
  while(shortest_path < -512)
    shortest_path += 1024;
    
#ifdef SIMULATION
  servo_position = desired_position;  // current servo position feedback is set equal to desired position
#else
  move(shortest_path);
#endif
}

void FlipperServo::move(int movement)
{
  // saturate the movement(shortest_path) command between -MAX_SERVO_SPEED and MAX_SERVO_SPEED
  movement = min(movement,MAX_SERVO_SPEED);
  movement = max(movement,-MAX_SERVO_SPEED);
    
  hbridge.drive(movement);
}
