#ifndef CONFIG_H
#define CONFIG_H

#define SIMULATION  // uncomment for simulations, comment out for real robot
#define ROBOT_NUM 2 // this particular robot will be used for IAS0060 (2020)
#define CALIBRATE_PRESSURE
#define CALIBRATE_MOTORS
//#define USE_IMU

#define AIR_PRESSURE 100400 // Pa

#define NUM_OF_MOTORS 4
#define FRONT_RIGHT 0
#define BACK_RIGHT 1
#define BACK_LEFT 2
#define FRONT_LEFT 3

#define MOTOR_POWER_EN A3
#define BATTERY A2
#define PRESSURE_CS A0 //chip select for pressure sensor
#define LED A1
#define LEFT_BEACON 5
#define RIGHT_BEACON 4

#define PLACEHOLDER 1

#define LOW_BATTERY_LEVEL 620  // e-02 Volts
#define BATTERY_ALARM_DELAY 20 // seconds of low voltage before alarm

#define RADIANS_PER_TICK 0.006135923f // 2pi/1024

#define DESIRED_DEPTH_ERROR 5 // cm, +-

#if ROBOT_NUM == 2
#define DESIRED_DEPTH 105 // cm
#define DEPTH_DEADZONE 5  // cm
#define DESIRED_DEPTH_ERROR 5 // cm, +-
#define PRESSURE_SLOPE 0.106145
#define PRESSURE_OFFSET 12492.1
const int calibrated_zeros[NUM_OF_MOTORS] = {188, 161, 726, 356};
#else
#error "Robot number out of range. Check config.h"
#endif // ROBOT_NUM == ?



const int reversed_rotation[NUM_OF_MOTORS] = {1, 1, -1, -1};
const int reversed_sensing[NUM_OF_MOTORS] = {-1, -1, -1, -1};
const byte hbridge_i2c_addresses[NUM_OF_MOTORS] = {0x64, 0x60, 0x61, 0x63};
const byte feedback_chipselect_pins[NUM_OF_MOTORS] = {7, 8, 9, 10};
#define IMU_I2C_ADDRESS 0x69
#define SERIAL_SPEED 115200
#define MAX_SERVO_SPEED 40


// loop & sampling timings
#define MAIN_LOOP_FREQ      100 // Hz
#define MOTOR_LOOP_FREQ     10 // Hz
#define ROS_PUBLISH_FREQ    10 // Hz
#define HEARTBEAT_FREQ      1 // Hz
#define PRESSURE_READ_FREQ  10 // Hz
#define BATTERY_READ_FREQ   10 // Hz
#define BEACON_READ_FREQ    10 // Hz



#endif //CONFIG_H
