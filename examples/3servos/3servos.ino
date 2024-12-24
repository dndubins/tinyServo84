// 1servo.ino: Example sketch for tinyServo84
// tinyServo84.h version 1.0.0
// Author: D. Dubins
// Date: 23-Dec-24
// Controlling 3 servos on any digital pins in BANK A and/or B from PA0 to PB2 on ATtiny84 (clock frequency=8MHz)
// The library maps specific servo numbers to the following pins:
// servo 0: PA0   servo 3: PA3   servo 6: PA6   servo 9: PB1
// servo 1: PA1   servo 4: PA4   servo 7: PA7   servo 10: PB2
// servo 2: PA2   servo 5: PA5   servo 8: PB0

#include "tinyServo84.h"

// Change these values to suit your application:
#define NSERVO 3          // number of servos to control (up to 11)
#define SVOMAXANGLE 179   // maximum angle for servo.
#define SVOMINPULSE 500   // minimum pulse width in microseconds for servo signal (0 degrees)
#define SVOMAXPULSE 2500  // maximum pulse width in microseconds for servo signal (for maximum angle)
#define SVOTIMEOUT 500    // timeout in ms to disable servos.

tinyServo84 myServos;  // declare object called myServos of class tinyServo84

void setup() {
  myServos.setCTC();
  for (int i = 0; i < NSERVO; i++) {  // Attach servo 0 on PA0, servo 1 on PA1 ... up to NSERVO,
    myServos.attachServo(i);          // following the guide in the sketch header.
  } 
  myServos.homeServos();      // home any attached servos
}

void loop() {
  // Uncomment the timeout check below for disabling Timer1, if the servos don't receive a command after
  // SVOTIMEOUT msec. This servo_timeout_check() is optional. Temporarily turning off Timer1 will free
  // the mcu to do other things. You can also manually suspend Timer1 with the command "myServos.disableTimerInterrupt();".
  myServos.servo_timeout_check();  // if servos are inactive, stop Timer1 (less trouble for other routines)

  // Uncomment to rock all servos simultaneously, at full speed.
  for (int i = 0; i < NSERVO; i++) {
    myServos.setServo(i, 0);
  }
  delay(1000);
  for (int i = 0; i < NSERVO; i++) {
    myServos.setServo(i, SVOMAXANGLE);
  }
  delay(1000);

  // Uncomment to rock all servos smoothly using moveTo(), with a 10ms delay between steps:
  //moveTo(0, 0, 0, 10);
  //moveTo(SVOMAXANGLE, SVOMAXANGLE, SVOMAXANGLE, 10);
  
  // Uncomment to rock servo 1 slowly
  /*for (int i = 0; i < SVOMAXANGLE; i++) {
    myServos.setServo(1, i);
    delay(500); // 0.5s delay should let you see each angle
  }
  for (int i = SVOMAXANGLE; i >=0; i--) {
    myServos.setServo(1, i);
    delay(500);
  }*/

  // Uncomment to rock all servos at full speed through 0-SVOMAXANGLE, sequentially.
  /*for (int i = 0; i < NSERVO; i++) {
    myServos.setServo(i, 0);
    delay(1000);
    myServos.setServo(i, SVOMAXANGLE);
    delay(1000);
  }*/

  // Uncomment for potentiometer control:
  /*int location = map(analogRead(A7), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A7 & remap to angle.
  myServos.setServo(0, location);  // write new location to servo 0
  myServos.setServo(1, location);  // write new location to servo 1
  myServos.setServo(2, location);  // write new location to servo 2
  delay(50);                      // wait a bit to reduce jittering
  */
  
  // Uncomment for potentiometer control of all servos with smoother movement:
  //int location = map(analogRead(A7), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A7 & remap to angle.
  //myServos.moveTo(location, location, location, 5);  // move to new location, delay=4 ms between steps
}

void moveTo(int s0, int s1, int s2, int wait) {  // routine for controlling all servos slowly, simultaneously.
  // wait=0: as fast as possible.
  // Note: Change structure of moveTo arguments to match # servos (add coordinates s3, s4 ... as needed).
  int loc[NSERVO] = { s0, s1, s2 };  // create array for loc’ns
  static int pos[NSERVO];            // remembers last value of pos
  if (wait == 0) {                   // if wait=0, move as fast as possible
    for (int i = 0; i < NSERVO; i++) {
      myServos.setServo(i, loc[i]);  // write new position to servos
    }
  } else {
    int dev = 0;  // to track deviation
    do {
      dev = 0;
      for (int i = 0; i < NSERVO; i++) {  // moves servos one step
        if (loc[i] > pos[i]) pos[i]++;    // add 1 to pos[i]
        if (loc[i] < pos[i]) pos[i]--;    // subtr 1 from pos[i]
        dev += abs(pos[i] - loc[i]);      // calculate deviation
      }
      for (int i = 0; i < NSERVO; i++) {
        myServos.setServo(i, pos[i]);  // write new position to servos
      }
      delay(wait);      // slow down movement
    } while (dev > 0);  // stop when location attained
  }                     // end if
}
