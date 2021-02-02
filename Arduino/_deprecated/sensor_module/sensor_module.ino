/*
     This software interfaces an group of sensors and relays sensor data through a serial port for use with a ROS-based simulation


*/
#include "config.h"
#include <avr/wdt.h>

#include <NewPing.h>


volatile bool wait = 0;
uint16_t main_loop_counter = 0;

// ultrasonic sensor
const int trigPin = 7;
const int echoPin = 6;
long duration;
int distance;
//
#define TRIGGER_PIN 7
#define ECHO_PIN 2
#define MAX_DISTANCE 50
// NewPing setup of pins and maximum distance
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {

  Serial.begin(SERIAL_BAUD);     // setup serial communication

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(9, INPUT); // Sets the echoPin as an Input

  attachInterrupt(2, EchoPinISR, CHANGE);  // Pin 2 interrupt on any change


  // =========================================================================
  // setup 16-bit timer 1 to control the loop timing at 100Hz:
  cli();                // disable interrupts
  TCCR1A = 0;           // set entire TCCR1A register to 0
  TCCR1B = 0;           // same for TCCR1B 
  TCNT1  = 0;           // initialize counter value to 0
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

  wdt_enable (WDTO_1S); // enable watchdog timer with 1 sec timeout


}

void loop() {
  digitalWrite(9, HIGH);
  wait = 1;             // loop timing switch

  wdt_reset ();         // reset the watchdog timer

  main_loop_counter++;  // count loops

  unsigned int distance = sonar.ping_cm();
  Serial.print(distance);
  Serial.println("cm");

  //  digitalWrite(trigPin, LOW);
  //  delayMicroseconds(2);
  //  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  //  digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(10);
  //  digitalWrite(trigPin, LOW);
  //  // Reads the echoPin, returns the sound wave travel time in microseconds
  //  duration = pulseIn(echoPin, HIGH);
  //  // Calculating the distance
  //  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  //  // Displays the distance on the Serial Monitor
  //  Serial.print("Distance: ");
  //  Serial.print(distance);
  //  Serial.println(" cm");





  digitalWrite(9, LOW);
  
  while (wait); // if there is time left over in the after everything else is done, it will wait here until the preallocated loop time elapses

}

void EchoPinISR() {
    static unsigned long startTime;
    if (digitalRead(2)) // Gone HIGH
        startTime = micros();
    else  // Gone LOW
        LastPulseTime = micros() - startTime;
}

ISR(TIMER1_COMPA_vect)     // timer compare interrupt service routine
{
  // this interrupt hits every 100ms. it is used for loop timing
  wait = 0;
}
