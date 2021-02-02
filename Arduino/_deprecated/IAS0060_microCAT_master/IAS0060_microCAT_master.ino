#include <SPI.h>
#include "minimoto.h"
#include "as5040.h"
#include "flipperservo.h"
#include "config.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include "navigation.h"
#include "pressure.h"
#include <avr/wdt.h>

volatile bool beaconFlag = false;

#ifdef SIMULATION
#else
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
#endif
volatile bool wait = 0; // loop timing variable

float temperature = 20;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

byte depth = 0; // cm

bool leftBeacon = false, rightBeacon = false;  // light sensor values
bool batteryLow = false;

int main_loop_counter = 0;
//int main_loop_time = 0;

FlipperServo motor[NUM_OF_MOTORS];
Pressure pressure;

void initMotorValues() {
  // give kinematic values to the motors. Will work with simulation and real robot.

  for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
    // fill out kinematic parameter values in 'motor' object for each fin
    motor[motornum].frequency = 1.0;    // Hz
    motor[motornum].zero_direction = 0 * 1.57 / RADIANS_PER_TICK;
    motor[motornum].amplitude = 1.0 / RADIANS_PER_TICK;
    motor[motornum].phase_offset = 0;
  }
}



#ifdef SIMULATION
// dmpGetGravity and gmpGetYawPitchRoll are defined in MPU6050_6Axis_MotionApps20.h, may not be necessary here.
void dmpGetGravity(VectorFloat *v, Quaternion *q) {
  v -> x = 2 * (q -> x * q -> z - q -> w * q -> y);
  v -> y = 2 * (q -> w * q -> x + q -> y * q -> z);
  v -> z = q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z;
}

void dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
  // yaw: (about Z axis)
  data[0] = atan2(2 * q -> x * q -> y - 2 * q -> w * q -> z, 2 * q -> w * q -> w + 2 * q -> x * q -> x - 1);
  // pitch: (nose up/down, about Y axis)
  data[1] = atan(gravity -> x / sqrt(gravity -> y * gravity -> y + gravity -> z * gravity -> z));
  // roll: (tilt left/right, about X axis)
  data[2] = atan(gravity -> y / sqrt(gravity -> x * gravity -> x + gravity -> z * gravity -> z));
}

void updateAngles() {
  // for SIMULATION use gravity and q containers on above functions for Gravity and YawPitchRoll
  dmpGetGravity(&gravity, &q);
  dmpGetYawPitchRoll(ypr, &q, &gravity);
}

void sendMotorsToSim() {
  Serial.print("SYNCSYNC");
  for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
    // fill out kinematic parameter values in 'motor' object for each fin
    Serial.write((byte*) & (motor[motornum].frequency), 4);
    Serial.write((byte*) & (motor[motornum].zero_direction), 2);
    Serial.write((byte*) & (motor[motornum].amplitude), 2);
    Serial.write((byte*) & (motor[motornum].phase_offset), 2);
  }
  Serial.write('\n');
}

#else // not for sim, for normal operation

void update_receivers() {
  leftBeacon = (digitalRead(LEFT_BEACON) == LOW);
  rightBeacon = (digitalRead(RIGHT_BEACON) == LOW);

  if (leftBeacon || rightBeacon) {
    beaconFlag = 1;     // if any of the sensors detects a beacon, flip the flag up
  }
  else {
    beaconFlag = 0;
  }
}

void updateAngles() {
#ifdef USE_IMU
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();  // get interrupt status bits, these become 0 after reading the register.

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();     // read current FIFO buffer size (the number of bytes stored (and can be read) in the FIFO buffer)

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    temperature = ( (float) mpu.getTemperature() + 12412.0) / 340.0;
    Quaternion uwsim_coordinates(0, 1, 0, 0);
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //    q = uwsim_coordinates.getProduct(q);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
#endif
}

// for filtering, require 5 consecutive low battery readings before triggering low battery mode
int batterylevel = 700; // start with full battery for averaging
int num_of_warnings = 0;
void checkBattery() {
  batterylevel = int(float(batterylevel) * 0.975 + float(analogRead(BATTERY)) * 0.025);
  if (batterylevel < LOW_BATTERY_LEVEL) {
    num_of_warnings++;
  }
  else {
    num_of_warnings = 0;
  }
  if (num_of_warnings >= BATTERY_ALARM_DELAY * BATTERY_READ_FREQ) {
    batteryLow = true;
  }
}
#endif

void updateFlippers() {
  for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
    motor[motornum].update();
  }
}

void setup()
{
  randomSeed(analogRead(6));            // sets a random number based on reading the analog 6 port
  pinMode(LED, OUTPUT);                 // init pins for LED, motor power measurement (*config.h) etc
  pinMode(MOTOR_POWER_EN, OUTPUT);
  pinMode(LEFT_BEACON, INPUT);
  pinMode(RIGHT_BEACON, INPUT);
  digitalWrite(MOTOR_POWER_EN, HIGH);
  digitalWrite(LED, HIGH);

  analogReference(INTERNAL);            // 1.1V reference for battery reading
  delay(5000);
  Wire.begin();                         // init wire lib, join the i2c bus as master
  Serial.begin(SERIAL_SPEED);           // init serial data transmision at set baud rate (*config.h)

#ifdef SIMULATION
#else                                 // ifndef SIMULATION--> init pressure module: init SPI com, setup data packet structures etc
  pressure.init();
#endif

  // motors will run both with simulated and real sensors
  for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++) {
    motor[motornum].initialize(motornum);
  }

  // =========================================================================
  // setup 16-bit timer 1 to control the loop timing at 100Hz:
  cli();            // stop interrupts
  TCCR1A = 0;       // set entire TCCR1A register to 0
  TCCR1B = 0;       // same for TCCR1B
  TCNT1  = 0;       // initialize counter value to 0
  // set compare match register for 100 Hz increments
  OCR1A = 19999;    // = 16000000 / (8 * 100) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for prescaler: 8
  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();            // allow interrupts
  // =========================================================================

  // Enable watchdog timer
  wdt_enable (WDTO_1S);
}

void loop()
{
  wdt_reset ();  // reset the watchdog timer

  wait = 1; // loop timing switch

  main_loop_counter++;

  if (main_loop_counter % (MAIN_LOOP_FREQ / MOTOR_LOOP_FREQ) == 0) {
    // these will run every 10 times the main_loop_counter gets updated, ie every 100msec
    updateAngles();  // get orientation values
    navigate();
    // calc desired position for each motor (for shortest path to achieve it), constrain within range, write to i2c des motor speed
    updateFlippers(); // uncomment for closed-loop motor control
  }

#ifdef SIMULATION
  if (main_loop_counter % (MAIN_LOOP_FREQ / ROS_PUBLISH_FREQ) == 0) {
    // every 10 main loop iterations
    sendMotorsToSim();
  }
#else

  if (main_loop_counter % (MAIN_LOOP_FREQ / BATTERY_READ_FREQ) == 0) {
    checkBattery();
  }
  if (main_loop_counter % (MAIN_LOOP_FREQ / PRESSURE_READ_FREQ) == 0) {
    // every 10 main loop iterations
    depth = pressure.cm();
  }

  if (main_loop_counter % (MAIN_LOOP_FREQ / BEACON_READ_FREQ) == 0) {
    update_receivers();
  }

#endif

  if ((main_loop_counter % (MAIN_LOOP_FREQ / HEARTBEAT_FREQ)) / 10 == 0) {
    // every 100 main loop iterations (once every second)
    digitalWrite(LED, !batteryLow);
  }
  else {
    digitalWrite(LED, batteryLow);
  }

  while (wait); // if there is time left over after everything else is done, it will wait here until the preallocated loop time elapses
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  // this interrupt hits every 100ms. it is used for loop timing
  wait = 0;
}

#ifdef SIMULATION
#define SERIAL_BUFFER_LENGTH 32
#define SERIAL_MSG_LENGTH 2*4+4*4+1+1+1
byte serial_buffer[SERIAL_BUFFER_LENGTH];
byte buffer_index = 0;
String sync = "syncsync";
byte syncstage = 0;
byte messagestart = 0;


void serialEvent() {
  // arduino is reading a message comming in its serial port containing a sync string ("syncsync"), orientation(quaternions), depth, left and right beacon states
  while (Serial.available()) {
    serial_buffer[buffer_index] = Serial.read();
    if (syncstage < 8) {
      if (serial_buffer[buffer_index] == sync[syncstage]) {
        syncstage++;
      }
      if (syncstage == 8)
        messagestart = buffer_index + 1;
    }
    // after reading the whole sync string it will receive simulated values for depth, left and right beacon states
    else {
      byte messagelength = messagestart < buffer_index + 1 ? buffer_index + 1 - messagestart : buffer_index + 1 + SERIAL_BUFFER_LENGTH - messagestart;
      if (messagelength + 2 * 4 == SERIAL_MSG_LENGTH) {
        if (serial_buffer[buffer_index] == '\n') {
          byte * qpointer = (byte*)&q;
          for (int i = 0; i < 4 * 4; i++) {
            *(qpointer + i) = serial_buffer[(messagestart + i) % SERIAL_BUFFER_LENGTH];
          }
          depth = serial_buffer[(messagestart + 16) % SERIAL_BUFFER_LENGTH];
          leftBeacon = serial_buffer[(messagestart + 17) % SERIAL_BUFFER_LENGTH] & 0x01;
          rightBeacon = serial_buffer[(messagestart + 17) % SERIAL_BUFFER_LENGTH] & 0x02;
        }
        syncstage = 0;
      }
    }
    buffer_index++;
    if (buffer_index >= SERIAL_BUFFER_LENGTH) {
      buffer_index = 0;
    }
  }
}
#endif
