// 1servo.ino: Example sketch for tinyServo84
// tinySero84 version 1.0.0
// Author: D.Dubins
// Date: 23-Dec-24
// Controlling 1 servo on any pin in BANKA on ATtiny84 (clock frequency=8MHz)
// The following pin numbers are used:
// servo #0..7: PA0..PA7 (8 servos)
// servo #8..10: PB0..PB2 (3 servos)

#include "tinyServo84.h"

// Change these values to suit your application:
#define SVOMAXANGLE 179   // maximum angle for servo.
#define SVOMINPULSE 500   // minimum pulse width in microseconds for servo signal (0 degrees)
#define SVOMAXPULSE 2500  // maximum pulse width in microseconds for servo signal (for maximum angle)
#define SVOTIMEOUT 500    // timeout in ms to disable servos.

tinyServo84 myServos;     // declare object called myServos of class tinyServo84

void setup() {
  myServos.setCTC();
  myServos.attachServo(0); // attach servo0 (PA0)
  myServos.homeServos(); // home servo0
}

void loop() {
  // Uncomment for disabling Timer1 if the servos don't receive a different signal, after
  // SVOTIMEOUT milliseconds.
  // This servo_timeout_check() is optional. Temporarily turning off Timer1 will free the mcu to do other things.
  // The function argument is the change in pulse width signal that will result in servos being re-enabled.
  // (Default=0, which means any change in signal will wake up servos).
  // The only reason you might need a number tolerance is if you would like to disable the timer, and you
  // are reading noisy potentiometer readings (suggested: ~10 per servo enabled).
  //myServos.servo_timeout_check(0);  // if servos are inactive, stop Timer1 (less trouble for other routines)
  
  //Uncomment for rocking servo 0 at full speed.
  /*myServos.setServo(0, 0);
  delay(1000);
  myServos.setServo(0, SVOMAXANGLE);
  delay(1000);*/

  // Uncomment to rock servo 0 using moveTo(), with a 10ms delay between steps:
  moveTo(0, 10);
  moveTo(SVOMAXANGLE, 10);

  // Uncomment to move servo 0 slowly (0.5 second steps)
  /*for (int i = 0; i < SVOMAXANGLE; i++) {
    myServos.setServo(0, i);
    delay(500); // 0.5s delay should let you see each angle
  }
  for (int i = SVOMAXANGLE; i >=0; i--) {
    myServos.setServo(0, i);
    delay(500);
  }*/

  // Uncomment for potentiometer control:
  //int location = map(analogRead(A7), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A7 & remap to angle.
  //myServos.setServo(0, location);  // write new location to servo 0
  //delay(50);              // wait a bit to reduce jittering

  // Uncomment for potentiometer control of servo0 with slower movement:
  //int location = map(analogRead(A7), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A7 & remap to angle.
  //moveTo(location, 5);  // move to new location, delay=4 ms between steps
}

void moveTo(int s0, int wait) { // routine to move servo slower (more smoothly)
  int loc = s0; // store s0 to loc
  static int pos; // keep track of current position
  if (wait == 0) { // option to move as fast as possible
    myServos.setServo(0, loc); // send new location to servo
  } else {
    int dev = 0; // to keep track of deviation from s0
    do {
      dev = 0;
      if (loc > pos) pos++;
      if (loc < pos) pos--;
      dev += abs(pos - loc);
      myServos.setServo(0, pos);
      delay(wait); // wait a bit (slows down movement)
    } while (dev > 0);
  }
}
