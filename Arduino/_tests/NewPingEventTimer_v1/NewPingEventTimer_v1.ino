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

#define TRIGGER_PIN   7 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN      2 // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.

bool state = true;
volatile bool wait = 0;
volatile unsigned long dropped = 0;
const unsigned long maxDropped = 10; // If dropped gets higher than this number, we will use -1 as the result
int distanceCm = -1;
int distanceRaw = -1;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pingTimer = millis(); // Start now.
  pinMode(9,OUTPUT);

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
}

void loop() {
  digitalWrite(9,HIGH);
  //state = !state;
  wait = 1;             // loop timing switch
  
  // Notice how there's no delays in this sketch to allow you to do other processing in-line while doing distance pings.
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer += pingSpeed;      // Set the next ping time.
    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  }
  digitalWrite(9,LOW);
  // Do other stuff here, really. Think of it as multi-tasking.
  if(dropped > maxDropped) {
    dropped = 0;
    distanceCm = -1;    
  }
  Serial.print(distanceCm);Serial.println("cm");
  
  while(wait) {};
}

void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    // Here's where you can add code.
    dropped = 0;
    distanceCm = sonar.ping_result / US_ROUNDTRIP_CM;
    /*
    Serial.print("Ping: ");
    Serial.print(sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    Serial.println("cm");
    */
  } else {
    dropped++;
  }
  // Don't do anything here!
}

ISR(TIMER1_COMPA_vect)     // timer compare interrupt service routine
{
  // this interrupt hits every 100ms. it is used for loop timing
  wait = 0;
}
