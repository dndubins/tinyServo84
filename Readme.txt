Readme file for Arduino tinyServo84 Library

tinyServo84 is a library that can control up to 11 servos on pin banks A and B of the ATtiny84 microprocessor.

This library was originally created as a sketch because I ignored PWM capabilities when I ordered a PCB. Using PA2, PA3, and PA4
to control servos, I quickly realized the regular servo library wouldn't do the job. So that's why I wrote this one.

* The example sketch "1servo.ino" illustrates a few basic moves controlling 1 servo.
* The example sketch "3servos.ino" illustrates a few basic moves controlling 3 servos.
* For those of you who don't like libraries, I included a library-free version in a self-contained sketch, "tinyServo84_nolib.ino".
* The example sketch "bruteForceServo.ino" should drive a servo on most digital pins regardless of the mcu. It's the sketch you know you could have written in 5 minutes.

The functions available in the library include:

attachServo(byte servo_num); // to attach the servo
detachServo(byte servo_num); // to detach the servo, which also sets the corresponding signal pin to INPUT mode
setServo(byte servo_num, int angle); // to set the servo to a specific angle
homeServos(); // routine to home the servos
moveTo(int s0, int s1, int s2, int s3, int s4, int s5, int s6, int wait); // enter location of each servo, and wait in msec for smoother movement
servo_timeout_check(in tol); // a timer for servo inactivity to temporarily disable the timer
enableTimerInterrupt(); // re-enable the timer
disableTimerInterrupt(); // disable the timer
setCTC(); // sets the timer to CTC mode at 50Hz rollover

To use the library, copy the download to the Library directory.
 
Technical notes:
- tinyServo84 only currently works on the ATTiny84, although the example sketch bruteForceServo.ino should work on other platforms.
- The library assumes a clock speed of 8MHz.
