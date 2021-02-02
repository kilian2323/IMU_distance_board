#include "navigation.h"
#include <helper_3dmath.h>

#define MAX_AMPLITUDE 1.6

extern bool batteryLow;
extern bool leftBeacon;
extern bool rightBeacon;

int desired_depth = 30;
extern byte depth;
byte depth_1, depth_2;

extern FlipperServo motor[];

// motor coordinate frame difference in radians, only around Z axis
// motors are at 30 degree angles
float sin_motor_angles[] = {0.5, -0.5, 0.5, -0.5};
float cos_motor_angles = 0.86602540378;

// angular multiplier array for roll, pitch and yaw configurations as columns 0, 1, and 2, respectively
// rows represent motors 0:3
// columns represent angular movement directions: roll, pitch, yaw
int angular_multiplier[NUM_OF_MOTORS][3] = { -1, -1, 1,
                                             -1, 1, 1,
                                             1, 1, -1,
                                             1, -1, -1
                                           };

//////////////////////////////////////////////////////////////////////////////////////////////////////
// PID controller settings
int8_t ek = 0, ek_1 = 0, ek_2 = 0;
float uk_1 = 0; // previous control signal

//////////////////////////////////////////////////////////////////////////////////////////////////////
// FOR FUZZY DIRECTION CONTROL
//float MU1[5] = {0, 0, 0, 0, 0}; // Membership degree for Fx (VN, N, Z, P, VP)
//float MU2[5] = {0, 0, 0, 0, 0}; // Membership degree for Fz (VN, N, Z, P, VP)
//float MU_R[25] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // RULES TABLE (See image RULES_TABLE.png)
float MU_CONFIG[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // OUTPUT Membership functions for defuzzification (9 possible directions. See image ZERO_DIRECTION.png)

// Initialisation of Universes of Discourse for Fx
float N_Fx = -1.0;
float P_Fx = -N_Fx;
float UOD_Fx[3] = {N_Fx, 0, P_Fx};

// Initialisation of Universes of Discourse for Fz
float VN_Fz = -20;
float N_Fz = -10;
float VP_Fz = -VN_Fz;
float P_Fz = -N_Fz;
float UOD_Fz[5] = {VN_Fz, N_Fz, 0, P_Fz, VP_Fz};

// Initialisation of Universes of Discourse for the output (direction)
float *UoDconfig = new float[9] { -M_PI / 2, -M_PI / 3, -M_PI / 6, -M_PI / 12, 0, M_PI / 12, M_PI / 6, M_PI / 3, M_PI / 2};
//////////////////////////////////////////////////////////////////////////////////////////////////////

float InitialOffset[NUM_OF_MOTORS] = { 0.0, 0.0, 0.0, 0.0 };
static const float PhaseOffsets[NUM_OF_MOTORS] = { M_PI, 0.0, 0.0, M_PI };

//////////////////////////////////////////////////////////////////////////////////////////////////////
void fuzzifyZ(float FF, float* UniverseOfDiscourse, float* MU) //UniverseOfDiscourse=[VN,N,Z,P,VP]
{
  if (FF <= UniverseOfDiscourse[0])
  { MU[0] = 1;
    MU[1] = 0;
    MU[2] = 0;
    MU[3] = 0;
    MU[4] = 0;
  }
  else if (FF > UniverseOfDiscourse[0] && FF <= UniverseOfDiscourse[1])
  {
    MU[0] = (FF - UniverseOfDiscourse[1]) / (UniverseOfDiscourse[0] - UniverseOfDiscourse[1]);
    MU[1] = (FF - UniverseOfDiscourse[0]) / (UniverseOfDiscourse[1] - UniverseOfDiscourse[0]);
    MU[2] = 0;
    MU[3] = 0;
    MU[4] = 0;
  }
  else if (FF > UniverseOfDiscourse[1] && FF <= UniverseOfDiscourse[2])
  {
    MU[0] = 0;
    MU[1] = (FF) / (UniverseOfDiscourse[1]);
    MU[2] = (FF - UniverseOfDiscourse[1]) / (-UniverseOfDiscourse[1]);
    MU[3] = 0;
    MU[4] = 0;
  }
  else if (FF > UniverseOfDiscourse[2] && FF <= UniverseOfDiscourse[3])
  {
    MU[0] = 0;
    MU[1] = 0;
    MU[2] = (FF - UniverseOfDiscourse[3]) / (-UniverseOfDiscourse[3]);
    MU[3] = (FF) / (UniverseOfDiscourse[3]);
    MU[4] = 0;
  }
  else if (FF > UniverseOfDiscourse[3] && FF <= UniverseOfDiscourse[4])
  {
    MU[0] = 0;
    MU[1] = 0;
    MU[2] = 0;
    MU[3] = (FF - UniverseOfDiscourse[4]) / (UniverseOfDiscourse[3] - UniverseOfDiscourse[4]);
    MU[4] = (FF - UniverseOfDiscourse[3]) / (UniverseOfDiscourse[4] - UniverseOfDiscourse[3]);
  }
  else if (FF > UniverseOfDiscourse[4])
  {
    MU[0] = 0;
    MU[1] = 0;
    MU[2] = 0;
    MU[3] = 0;
    MU[4] = 1;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void fuzzifyX(float FF, float* UniverseOfDiscourse, float* MU) //UniverseOfDiscourse=[VN,N,Z,P,VP]
{
  if (FF <= UniverseOfDiscourse[0]) { 
    MU[0] = 1;
    MU[1] = 0;
    MU[2] = 0;
  }
  else if (FF > UniverseOfDiscourse[0] && FF <= UniverseOfDiscourse[1]) {
    MU[0] = (FF - UniverseOfDiscourse[1]) / (UniverseOfDiscourse[0] - UniverseOfDiscourse[1]);
    MU[1] = (FF - UniverseOfDiscourse[0]) / (UniverseOfDiscourse[1] - UniverseOfDiscourse[0]);
    MU[2] = 0;
  }
  else if (FF > UniverseOfDiscourse[1] && FF <= UniverseOfDiscourse[2]) {
    MU[0] = 0;
    MU[1] = (-FF + UniverseOfDiscourse[2]) / (UniverseOfDiscourse[2] - UniverseOfDiscourse[1]);
    MU[2] = (FF - UniverseOfDiscourse[1]) / (UniverseOfDiscourse[2] - UniverseOfDiscourse[1]);
  }
  else if (FF > UniverseOfDiscourse[2]) {
    MU[0] = 0;
    MU[1] = 0;
    MU[2] = 1;
  }
}

/*
   Computes the value of the rules in the table (25 rules)
*/
void computeRulesSlow(float* MU_Fx, float* MU_Fz)
{
  float MU_R[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // RULES TABLE (See image RULES_TABLE.png)
  //RULES COMPUTING
  MU_R[0] = fmin(MU_Fx[0], MU_Fz[0]);  
  MU_R[1] = fmin(MU_Fx[1], MU_Fz[0]);  
  MU_R[2] = fmin(MU_Fx[2], MU_Fz[0]);
  MU_R[3] = fmin(MU_Fx[0], MU_Fz[1]);  
  MU_R[4] = fmin(MU_Fx[1], MU_Fz[1]);  
  MU_R[5] = fmin(MU_Fx[2], MU_Fz[1]);
  MU_R[6] = fmin(MU_Fx[0], MU_Fz[2]);  
  MU_R[7] = fmin(MU_Fx[1], MU_Fz[2]);  
  MU_R[8] = fmin(MU_Fx[2], MU_Fz[2]);
  MU_R[9] = fmin(MU_Fx[0], MU_Fz[3]);  
  MU_R[10] = fmin(MU_Fx[1], MU_Fz[3]);  
  MU_R[11] = fmin(MU_Fx[2], MU_Fz[3]);
  MU_R[12] = fmin(MU_Fx[0], MU_Fz[4]);  
  MU_R[13] = fmin(MU_Fx[1], MU_Fz[4]);  
  MU_R[14] = fmin(MU_Fx[2], MU_Fz[4]);



  //INFERENCE MECHANISM
  MU_CONFIG[0] = MU_R[13];
  MU_CONFIG[1] = fmax(MU_R[12], MU_R[14]);
  MU_CONFIG[2] = MU_R[10];
  MU_CONFIG[3] = fmax(MU_R[9], MU_R[11]);
  MU_CONFIG[4] = fmax(MU_R[6], fmax(MU_R[7], MU_R[8]));
  MU_CONFIG[5] = MU_R[4];
  MU_CONFIG[6] = fmax(MU_R[3], MU_R[5]);
  MU_CONFIG[7] = fmax(MU_R[0], MU_R[2]);
  MU_CONFIG[8] = MU_R[1];
}

/*
   This function computes the value of the output based on the firing rules (defuzzification process using Center of Gravity method)
*/
float defuzzify(float* MU_CONFIG, float* UoDconfig, int n)
{
  float sum1 = 0;
  float sum2 = 0;
  for (int i = 0; i < n; i++)
  { sum1 += MU_CONFIG[i];
    sum2 += MU_CONFIG[i] * UoDconfig[i];
  }

  if (sum1 == 0) return 0;
  else return (sum2 / sum1);
}

void calc_motor_commands(float x, float y, float z, float roll, float pitch, float yaw) {
  float motor_speeds[NUM_OF_MOTORS] = {0.0, 0.0, 0.0, 0.0};  // will be used to calculate the desired amplitude of oscillation (motor.amplitude)
  int new_directions[NUM_OF_MOTORS] = {0, 0, 0, 0};  // will be used to calculate the desired reference of oscillation (motor.zero_direction)

  float MU1[3] = {0, 0, 0}; // Membership degree for Fx (N, Z, P)
  float MU2[5] = {0, 0, 0, 0, 0}; // Membership degree for Fz (VN, N, Z, P, VP)

  fuzzifyX(x, UOD_Fx, &MU1[0]);
  fuzzifyZ(z, UOD_Fz, &MU2[0]);
  computeRulesSlow(&MU1[0], &MU2[0]);
  float fins_direction = defuzzify(&MU_CONFIG[0], &UoDconfig[0], 9);


  // all fins will move in reference to the "slow mode" configuration, where all fins' reference is horizontal towards the middle of the robot.
  new_directions[0] = ((- fins_direction) ) / RADIANS_PER_TICK;
  new_directions[1] = ((M_PI + fins_direction) ) / RADIANS_PER_TICK;
  new_directions[2] = ((M_PI + fins_direction) ) / RADIANS_PER_TICK;
  new_directions[3] = ((- fins_direction) ) / RADIANS_PER_TICK;


  // make sure that only the forward fins contribute to forward motion and only the back ones to backward motion
  if (x > 0) {
    motor_speeds[0] = abs(x);
    motor_speeds[1] = 0;
    motor_speeds[2] = 0;
    motor_speeds[3] = abs(x);
  }
  else if (x < 0) {
    motor_speeds[0] = 0;
    motor_speeds[1] = abs(x);
    motor_speeds[2] = abs(x);
    motor_speeds[3] = 0;
  }

  for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
    // additional force is applied to all fins to contribute to z motion
    motor_speeds[motornum] += abs(z) / 4;
    motor[motornum].phase_offset = PhaseOffsets[motornum] / (RADIANS_PER_TICK);
  }

  float max_speed = 0;
  // find the fastest motor and store its speed to the variable max_speed.
  for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
    if (motor_speeds[motornum] > max_speed) {
      max_speed = motor_speeds[motornum];
    }
  }

  // if the fastest motor is faster than physically possible (i.e. has a value greater than 1), scale all motor speeds down
  if (max_speed > 1.0) {
    for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
      motor_speeds[motornum] /= max_speed;
    }
  }

  // apply speeds to all motors. Remember that "motor_speeds" is an array of normalized values. Use the "MAX_AMPLITUDE" macro to scale the result
  for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
    motor[motornum].amplitude = motor_speeds[motornum] * MAX_AMPLITUDE / RADIANS_PER_TICK;
  }

  // slow down the change of direction to make stabilization easier
  for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
    if (motor[motornum].amplitude < 0.3 / RADIANS_PER_TICK) {
      continue;
    }
    int shortest_path = (new_directions[motornum] - motor[motornum].zero_direction) % 1024;
    // shortest path is always in the -512...511 range
    while (shortest_path > 511) {
      shortest_path -= 1024;
    }
    while (shortest_path < -512) {
      shortest_path += 1024;
    }
#define MAX_DIR_CHANGE 80
    if (shortest_path > MAX_DIR_CHANGE) {
      shortest_path = MAX_DIR_CHANGE;
    }
    if (shortest_path < -MAX_DIR_CHANGE) {
      shortest_path = -MAX_DIR_CHANGE;
    }

    // calculate the zero_direction attribute using the "shortest_path" variable
    motor[motornum].zero_direction += shortest_path;

    motor[motornum].frequency = 1.3; // set frequency to a constant 1.3Hz
  }

}

void dive() {
  calc_motor_commands(0, 0, -1, 0, 0, 0);
}

void swim_up() {
  calc_motor_commands(0, 0, 1, 0, 0, 0);
}

void swim_forward() {
  calc_motor_commands(1, 0, 0, 0, 0, 0);
}

void swim_backward() {
  calc_motor_commands(-1, 0, 0, 0, 0, 0);
}

void roll_left() {
  calc_motor_commands(0, 0, 0, -1, 0, 0);
}

void roll_right() {
  calc_motor_commands(0, 0, 0, 1, 0, 0);
}

void swim_up_fwd() {
  calc_motor_commands(1, 0, 1, 0, 0, 0);
}

void swim_down_fwd() {
  calc_motor_commands(1, 0, -1, 0, 0, 0);
}

void yaw_left() {
  calc_motor_commands(0, 0, 0, 0, 0, 1);
}

void yaw_right() {
  calc_motor_commands(0, 0, 0, 0, 0, -1);
}

void depth_bangbang(int8_t depth_error) {
  if (depth_error <  -DEPTH_DEADZONE) {         // bang-bang depth contoller
    calc_motor_commands(0, 0, 1, 0, 0, 0);      // if deeper than desired: swim up
  }
  else if (depth_error >  DEPTH_DEADZONE) {
    calc_motor_commands(0, 0, -1, 0, 0, 0);     // if shallower than desired: dive
  }
}

float depth_pid(int8_t depth_error) {
  // this runs every 100ms

  ek = depth_error;  // current error

  // calculate control signal
#if defined(CONT_PID) || defined(CONT_PD)
  float uk = uk_1 + (ek * PID_A) + (ek_1 * PID_B) + (ek_2 * PID_C);

  ek_2 = ek_1;  // assign previous error to more previous error
  ek_1 = ek;    // assign current error to previous error
  uk_1 = uk;    // keep previous output of controller
#endif

#ifdef CONT_PI_D
  float uk = uk_1 + (ek * PID_A) + ( ek_1 * PID_B) - (PID_C * (depth - 2 * depth_1 + depth_2) );

  ek_2 = ek_1;  // keep ek-2 error
  ek_1 = ek;    // keep previous error
  uk_1 = uk;    // keep previous output of controller
  depth_2 = depth_1;
  depth_1 = depth; // keep previous depth
#endif

  return uk;
}

void depth_control(int8_t depth_error) {
  float depth_pid_output = -depth_pid(depth_error);
  calc_motor_commands(0, 0, depth_pid_output, 0, 0, 0);
}

void navigate() {
  if (batteryLow) {
    // this stops the motors in case battery has dropped bellow acceptable levels
    motor[FRONT_RIGHT].zero_direction = 1.57 / RADIANS_PER_TICK;
    motor[FRONT_RIGHT].amplitude = 0;
    motor[FRONT_RIGHT].frequency = 0;
    motor[BACK_RIGHT].zero_direction = 1.57 / RADIANS_PER_TICK;
    motor[BACK_RIGHT].amplitude = 0;
    motor[BACK_RIGHT].frequency = 0;
    motor[BACK_LEFT].zero_direction = 1.57 / RADIANS_PER_TICK;
    motor[BACK_LEFT].amplitude = 0;
    motor[BACK_LEFT].frequency = 0;
    motor[FRONT_LEFT].zero_direction = 1.57 / RADIANS_PER_TICK;
    motor[FRONT_LEFT].amplitude = 0;
    motor[FRONT_LEFT].frequency = 0;
  }
  else {
    desired_depth = 40;
        if (leftBeacon || rightBeacon) {
          desired_depth = 40;
        }
        else {
          desired_depth = 20;
        }


    int8_t depth_error = desired_depth - depth;
    depth_control(depth_error);

  }
}
