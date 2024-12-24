// tinyServo84.ino
// Control up to 8 servos from pins PA0...PA7 or up to 3 servos from PB0...PB2
// using CTC mode of Timer1 (ATtiny84)
// Clock speed = 8MHz
// Transferability: This is a very specific sketch! Will only work on the ATtiny84.
// However, if you have a good understanding of timers and CTC mode, you can adapt it to
// non-PWM pins of other MCUs as well.
// Author: D.Dubins
// Date: 19-Dec-24
// Last Updated: 23-Dec-24
//
// Notes: I have a potentiometer wired as a voltage divider on pin A7. This would conflict with the sketch if 8 servos
// are attached.
// This sketch could be modified to accommodate up to 11 servos by setting the starting servo at PA0 and sending signals
// to both banks (A and B) instead of just one.

//#define SERIALDEBUG       // comment out for no serial debugging

#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef SERIALDEBUG
#include <SoftwareSerial.h>
SoftwareSerial mySerial(PA5, PA6);  // Start the software serial monitor on PA5(RX) and PA6(TX)
#endif

#define NSVO 3            // number of servos to control (up to 8)
#define SVOMAXANGLE 179   // maximum angle for servo.
#define SVOMINPULSE 500   // minimum pulse width in microseconds for servo signal (0 degrees). Default: 500
#define SVOMAXPULSE 2500  // maximum pulse width in microseconds for servo signal (for maximum angle). Default: 2500
#define SVOTIMEOUT 500    // timeout in ms to disable servos. Should be long enough to attain setpoint.
#define SVO1 PA0          // starting pin in bank for servo 0. PA0 for first pin in BANKA, PB0 for first pin in BANKB (default:PA0).
#define DATAREG DDRA      // DDRA for BANKA, DDRB for BANKB (default: DDRA)
#define PORTREG PORTA     // PORTA for BANKA, PORTB for BANKB (default: DDRA)

//This sketch allows you to make servo0 what you want.
//Make sure you declare your servo pins sequentially.
//If SVO1 is PA2, and NSVO is 3, then PA2 will be servo0, PA3 will be servo1, and PA4 will be servo2.
//If SVO1 is PA0 and NSVO is 7, then servo0=PA0, servo1=PA1, ... servo7=PA7.
//If SVO1 is PB0 and NSVO is 3, then servo0=PB0, servo1=PB1, servo2=PB2 (do not exceed 3 servos if starting from PB0).
//Warning: if you change SVO1, then make sure you check/change the breakpoints for the registers in the rest of the program.

//change the dimensions of the following arrays to match NSVO
unsigned int servo_PWs[NSVO] = { 1500, 1500, 1500 };  // Pulse widths in microseconds (default to center position)
bool servo_attached[NSVO] = { 0, 0, 0 };              // Servo attachment status
bool timer1_enabled = false;                          // to keep track if Timer1 is enabled
unsigned long servo_tLast = 0UL;                      // To store the time the last servo was used (timeout function)

void setup() {
#ifdef SERIALDEBUG
  mySerial.begin(9600);  // Start the software serial monitor (for debugging)
#endif
  setCTC();  // set CTC mode to start a 50Hz timer for the servo signals
  // attach servos here
  attachServo(0);  // pin PA0 is servo0
  attachServo(1);  // pin PA1 is servo1
  attachServo(2);  // pin PA2 is servo2

  // if you would like to detach a specific servo at any time:
  //detachServo(0); // detach servo0
  //detachServo(1); // detach servo1
  //detachServo(2); // detach servo2
  //detachServo(3); // etc ...

  homeServos();  // start sketch by sending attached servos to home position
}

void loop() {
  // Uncomment the timeout check below for disabling Timer1, if the servos don't receive a command after
  // SVOTIMEOUT msec. This servo_timeout_check() is optional. Temporarily turning off Timer1 will free
  // the mcu to do other things. You can also manually suspend Timer1 with the command "disableTimerInterrupt();".
  servo_timeout_check();  // if servos are inactive, stop Timer1 (less trouble for other routines)
  // Uncomment to rock all servos simultaneously, at full speed.
  for (int i = 0; i < NSVO; i++) {
    setServo(i, 0);
  }
  delay(1000);
  for (int i = 0; i < NSVO; i++) {
    setServo(i, SVOMAXANGLE);
  }
  delay(1000);

  // Uncomment to rock all servos smoothly using moveTo(), with a 10ms delay between steps:
  //moveTo(0, 0, 0, 10);
  //moveTo(SVOMAXANGLE, SVOMAXANGLE, SVOMAXANGLE, 10);

  // Uncomment to rock servo1 very slowly
  /*for (int i = 0; i < SVOMAXANGLE; i++) {
    setServo(1, i);
    delay(500); // 0.5s delay should let you see each angle
  }
  for (int i = SVOMAXANGLE; i >=0; i--) {
    setServo(1, i);
    delay(500);
  }*/

  // Uncomment to rock all servos at full speed through 0-SVOMAXANGLE, sequentially.
  /*for (int i = 0; i < NSVO; i++) {
    setServo(i, 0);
    delay(1000);
    setServo(i, SVOMAXANGLE);
    delay(1000);
  }*/

  // Uncomment for quick potentiometer control:
  //int location = map(analogRead(A7), 1023, 0, 0, SVOMAXANGLE); // take pot reading on pin A7 & remap to angle.
  //setServo(0, location);  // write new location to servo0
  //setServo(1, location);  // write new location to servo1
  //setServo(2, location);  // write new location to servo2
  //delay(50);              // wait a bit to reduce jittering

  // Uncomment for potentiometer control of all servos with slower movement:
  //int location = map(analogRead(A7), 1023, 0, 0, SVOMAXANGLE); // take pot reading on pin A7 & remap to angle.
  //moveTo(location, location, location, 5);  // move to new location, delay=4 ms between steps
}

void attachServo(byte servo_num) {  // function to attach servo
  if (servo_num < NSVO) {
    servo_attached[servo_num] = true;      // Set servo_attached to true
    DATAREG |= (1 << (SVO1 + servo_num));  // Set servo pin to OUTPUT mode
  }
}

void detachServo(byte servo_num) {  // function to detach servo
  if (servo_num < NSVO) {
    servo_attached[servo_num] = false;      // Set servo_attached to false
    PORTREG &= ~(1 << (SVO1 + servo_num));  // Set servo pin low
    DATAREG &= ~(1 << (SVO1 + servo_num));  // Set servo pin to INPUT mode (less chatter when not doing anything)
  }
}

void setServo(byte servo_num, int angle) {                                 // function to set servo position
  if (!timer1_enabled) enableTimerInterrupt();                             // enable Timer1 in case it timed out
  int pulse_width = map(angle, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // convert angle to pulse width in microseconds
  pulse_width = constrain(pulse_width, SVOMINPULSE, SVOMAXPULSE);          // constrain pulse width to min and max
  if (pulse_width != servo_PWs[servo_num] && servo_attached[servo_num]) {  // Disable interrupts only if signal changes and servo is attached
    cli();                                                                 // Disable interrupts. It's best to update volatile global variables with interrupts diabled.
    servo_PWs[servo_num] = pulse_width;                                    // Store new pulse_width in servo_PWs
    sei();                                                                 // Enable interrupts. Spend as little time in "disabled interrupt land" as possible.
  }
  servo_tLast = millis();  // record time servo was last used for timeout function
}

void homeServos() {  // function to home servos
  // Note: servo will not home unless it is first enabled.
  for (byte i = 0; i < NSVO; i++) {
    setServo(i, 90);  // send all servos to middle position
  }
  delay(1000);  // wait for servos to home
}

void moveTo(int s0, int s1, int s2, int wait) {  // function for controlling all servos slowly, simultaneously.
  // wait=0: as fast as possible.
  // Note: Change structure of moveTo arguments to match # servos (add coordinates s3, s4 ... as needed).
  int loc[NSVO] = { s0, s1, s2 };  // create array for loc’ns
  static int pos[NSVO];            // remembers last value of pos
  if (wait == 0) {                 // if wait=0, move as fast as possible
    for (int i = 0; i < NSVO; i++) {
      setServo(i, loc[i]);  // write new position to servos
    }
  } else {
    int dev = 0;  // to track deviation
    do {
      dev = 0;
      for (int i = 0; i < NSVO; i++) {  // moves servos one step
        if (loc[i] > pos[i]) pos[i]++;  // add 1 to pos[i]
        if (loc[i] < pos[i]) pos[i]--;  // subtr 1 from pos[i]
        dev += abs(pos[i] - loc[i]);    // calculate deviation
      }
      for (int i = 0; i < NSVO; i++) {
        setServo(i, pos[i]);  // write new position to servos
      }
      delay(wait);      // slow down movement
    } while (dev > 0);  // stop when location attained
  }                     // end if
}

void setCTC() {  // function to set the registers of the ATtiny84 for Timer 1 CTC mode
  // Setting up Timer1 for 1µs ticks (assuming 8MHz clock)
  cli();                // stop interrupts
  TCCR1A = 0;           // clear timer control register A
  TCCR1B = 0;           // clear timer control register B
  TCNT1 = 0;            // set counter to 0
  TCCR1B = _BV(WGM12);  // CTC mode (Table 12-5 on ATtiny84 datasheet)
  //TCCR1B = _BV(WGM13) | _BV(WGM12);
  //TCCR1B |= _BV(CS10);  // prescaler=1
  //TCCR1B |= _BV(CS11);  // prescaler=8
  TCCR1B |= _BV(CS11) | _BV(CS10);  // prescaler=64
  //TCCR1B |= _BV(CS12); // prescaler=256
  //TCCR1B |= _BV(CS12) |  _BV(CS10); // prescaler=1024
  OCR1A = 2499;  //OCR1A=(fclk/(N*frequency))-1 (where N is prescaler).
  //N=64, OCR1A=2499: 50Hz cycle, 8us per tick.
  //N=8, OCR1A=19999: 50Hz cycle, 1us per tick.
  TIMSK1 |= _BV(OCIE1A);  // enable timer compare
  sei();                  // enable interrupts
  timer1_enabled = true;  // timer1 is now enabled
}

ISR(TIM1_COMPA_vect) {  // This is the ISR that will turn off the pins at the correct widths
  //The function micros() does not advance inside the ISR.
  //TCNT1 starts at 0 and counts up. Each increment lasts 8 microseconds. We are going to use this as a timer.
  //8 microseconds gives us (2500-500us)/8us =250 steps. This is fine for a 180 degree servo. For a 360 degree servo,
  //if more steps are needed, you can set the timer to N=8, OCR1A=19999 and this will give 2000 steps, but require
  //more attention by the ISR.
  for (byte i = 0; i < NSVO; i++) {
    // Turn on the servo pin
    if (servo_attached[i]) {         // only turn on pin if servo attached
      PORTREG |= (1 << (SVO1 + i));  // set correct servo pin high
    }
  }

  while ((TCNT1 * 8) < (SVOMAXPULSE + 100)) {  // multiply TCNT1 by microseconds/step
    // a 50 Hz pulse has a period of 20,000 us. We just need to make it past SVOMAXPULSE with a small buffer.
    for (byte i = 0; i < NSVO; i++) {
      if (servo_attached[i] && (TCNT1 * 8) > servo_PWs[i]) {
        // Turn off the servo pin if the timer exceeds the pulse width
        PORTREG &= ~(1 << (SVO1 + i));  // Set correct servo pin low
      }
    }
  }
}

void enableTimerInterrupt() {  // run this if you'd like to (re)enable CTC timer interrupt
#ifdef SERIALDEBUG
  mySerial.prinln("Timer1 enabled.");
#endif
  TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 Compare Match A interrupt
  timer1_enabled = true;
}

void disableTimerInterrupt() {  // run this if you'd like to disable CTC timer interrupt. This will disable all servos.
#ifdef SERIALDEBUG
  mySerial.prinln("Timer1 disabled.");
#endif
  TIMSK1 &= ~(1 << OCIE1A);  // Disable Timer1 Compare Match A interrupt
  timer1_enabled = false;
}

void servo_timeout_check() {  // tol is added for potentiometer control. Default should be zero.
  if (((millis() - servo_tLast) > SVOTIMEOUT) && timer1_enabled) {
    disableTimerInterrupt();  // disable Timer1
  }
}
