// ---------------------------------------------------------------------------
// This example shows how to use NewPing's ping_timer method which uses the Timer2 interrupt to get the
// ping time. The advantage of using this method over the standard ping method is that it permits a more
// event-driven sketch which allows you to appear to do two things at once. An example would be to ping
// an ultrasonic sensor for a possible collision while at the same time navigating. This allows a
// properly developed sketch to multitask. Be aware that because the ping_timer method uses Timer2,
// other features or libraries that also use Timer2 would be effected. For example, the PWM function on
// pins 3 & 11 on Arduino Uno (pins 9 and 11 on Arduino Mega) and the Tone library. Note, only the PWM
// functionality of the pins is lost (as they use Timer2 to do PWM), the pins are still available to use.
// NOTE: For Teensy/Leonardo (ATmega32U4) the library uses Timer4 instead of Timer2.
// ---------------------------------------------------------------------------
#include <NewPing.h>
#include "Wire.h"
#include <MPU6050_light.h>

#define TRIGGER_PIN    7 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN       2 // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define PROBE_PIN      9 // Oscilloscope pin for probing speed of main loop and length of ping value retrieval.

const String messageStart = "SYNCSYNC";
const byte messageEnd = '\n';
const byte delim = ';';

const bool debug = false;             // If true, human-readable results will be printed instead of ROS messages.
const unsigned long maxDropped = 10; // Adjust this number: If dropped gets higher than this number, we will use -1 as the result.
const unsigned int pingSpeed = 50;   // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
const unsigned int imuSpeed = 10;    // How frequently are we going to interface the IMU. 10ms would be 100 times a second.

unsigned long pingTimer;             // Holds the next ping time.
volatile bool wait = false;          // Flag to control the loop frequency using Timer 1.
volatile unsigned long dropped = 0;  // Counts how many calls to sonar.check_timer() returned false.
int distanceCm = -1;                 // Stores the resulting distance in cm.

unsigned long imuTime = 0;           // Holds the time when the IMU was last interfaced.
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MPU6050 mpu(Wire);

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
  pingTimer = millis(); // Start ping sensor now.
  pinMode(PROBE_PIN,OUTPUT);
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcGyroOffsets();// Reset IMU to 0.

  
}

void loop() {  
  wait = 1;             // loop timing switch
  getDistance();        // will interface the ping sensor only when it's time to do so
  getIMU();             // will interface the IMU sensor only when it's time to do so
  if(!debug) {
    sendMessage();      // sends a message to ROS node
  } else {
    printResults();
  }
  while(wait) {};       // waits for flag to restart loop (keep constant loop frequency, as per settings of Timer 1)
}

void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // <---- Don't do anything here!
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    dropped = 0;
    distanceCm = sonar.ping_result / US_ROUNDTRIP_CM;    
  } else {
    dropped++;
  }
  // <---- Don't do anything here!
}

// This interrupt hits every 100ms. It is used for loop timing.
ISR(TIMER1_COMPA_vect)     // timer compare interrupt service routine
{  
  wait = 0;
}

// Gets the distance from the ping sensor.
// To be called in each loop cycle.
void getDistance() {
  digitalWrite(PROBE_PIN,HIGH);
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer += pingSpeed;      // Set the next ping time.
    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  }
  digitalWrite(PROBE_PIN,LOW);
  if(dropped > maxDropped) {
    dropped = 0;
    distanceCm = -1;    
  }
}

// Gets data from the IMU sensor.
// To be called in each loop cycle.
void getIMU() {
  if(millis() - imuTime > imuSpeed) { // If the loop frequency is slower than the frequency defined in imuSpeed, the IMU will be interfaced more slowly.
    mpu.update();
    yaw = mpu.getAngleZ();
    imuTime = millis();
  }
}

// Sends the message to the ROS node.
void sendMessage() {
  //Serial.print(distanceCm);Serial.println("cm");
  Serial.print(messageStart);
  /*
  Serial.write((byte*) &distanceCm, 2);
  Serial.write((byte*) &yaw, 4);
  */
  Serial.print(distanceCm);
  Serial.print((char)delim);
  Serial.print(yaw);
  Serial.print((char)messageEnd);
}

void printResults() {
  Serial.print(distanceCm);Serial.println("cm");
  Serial.print(yaw);Serial.println("°");
  Serial.println("-------------");
}
