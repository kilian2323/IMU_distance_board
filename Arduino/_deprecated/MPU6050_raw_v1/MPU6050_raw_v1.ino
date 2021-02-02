// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

#include "Wire.h"


unsigned long lastUpdate = 0;
const unsigned int updateDelay = 20;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t ax, ay, az;
int16_t gx, gy, gz;
float temperature = 20;

VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
float euler[3];         // [psi, theta, phi]    Euler angle container
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool wait = false;






#define LED_PIN LED_BUILTIN
bool blinkState = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void updateAngles() {
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();  // get interrupt status bits, these become 0 after reading the register.

  // Serial.print("mpuIntStatus: ");Serial.println(mpuIntStatus);
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();     // read current FIFO buffer size (the number of bytes stored (and can be read) in the FIFO buffer)
  // Serial.print("bytes in FIFO: ");Serial.println(fifoCount);
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    // Serial.println("Overflow");
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
    Serial.println(ypr[0]);
  }
  
  
}

void setup() {
    
    Wire.begin();  

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;                
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    
    // =========================================================================
    // setup 16-bit timer 1 to control the loop timing at 100Hz:
    cli();                // disable interrupts
    TCCR1A = 0;           // set entire TCCR1A register to 0
    TCCR1B = 0;           // same for TCCR1B 
    TCNT1  = 0;           // initialize counter value to 0 // counting
    // set compare match register for 100 Hz increments
    OCR1A = 19999;        // = 16000000 / (8 * 100) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12, CS11 and CS10 bits for prescaler: 8
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();                // enable interrupts
    // =========================================================================
    
    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    if(millis() - lastUpdate > updateDelay) {
      updateAngles();
      lastUpdate = millis();
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
    
}
