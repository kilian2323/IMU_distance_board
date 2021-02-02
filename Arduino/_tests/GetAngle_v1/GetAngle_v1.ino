/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;
volatile bool wait = false;

void setup() {
  
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
  
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  Serial.println(F("Calculating gyro offset, do not move MPU6050"));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("Done!\n");

  
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
	Serial.print("X : ");
	Serial.print(mpu.getAngleX());
	Serial.print("\tY : ");
	Serial.print(mpu.getAngleY());
	Serial.print("\tZ : ");
	Serial.println(mpu.getAngleZ());
	timer = millis();  
  }
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  // this interrupt hits every 100ms. it is used for loop timing
  wait = 0;
}
