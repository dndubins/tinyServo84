// tinyServo84.cpp file for tinyServo84.h library.
// Control up to 11 servos from pins PA0...PA7, and PB0...PB2
// using CTC mode of Timer1 (ATtiny84)
// Clock speed = 8MHz
// Transferability: This is a very specific lebrary. It will only work on the ATtiny84.
// Author: D.Dubins
// Date: 19-Dec-24
// Last Updated: 23-Dec-24
//
// The following pin numbers are used:
// servo #0..7: PA0..PA7 (8 servos)
// servo #8..10: PB0..PB2 (3 servos)

#include "tinyServo84.h"
#include <avr/io.h>
#include <avr/interrupt.h>

unsigned int tinyServo84::servo_PWs[NSVO] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };  // pulse-width of mid-points of each servo
bool tinyServo84::servo_attached[NSVO] = { false, false, false, false, false, false, false, false, false, false, false }; // each pin starts out unattached

tinyServo84::tinyServo84() {
}

void tinyServo84::attachServo(byte servo_num) { // function to attach servo
  if (servo_num < 8) {
    tinyServo84::servo_attached[servo_num] = true;   // Set servo_attached to true
    DDRA |= (1 << (PA0 + servo_num));  // Set servo pin in DDRA to OUTPUT mode
  }else if (servo_num < NSVO){
	tinyServo84::servo_attached[servo_num] = true;  // Set servo_attached to true
    DDRB |= (1 << (PB0 + (servo_num - 8)));  // Set servo pin in DDRB to OUTPUT mode
  }
}

void tinyServo84::detachServo(byte servo_num) { // function to detach servo
  if (servo_num < 8) {
    tinyServo84::servo_attached[servo_num] = false;   // Set servo_attached to false
    PORTA &= ~(1 << (PA0 + servo_num)); // Set servo pin low
    DDRA &= ~(1 << (PA0 + servo_num));  // Set servo pin to INPUT mode (less chatter when not doing anything)
 } else {
    tinyServo84::servo_attached[servo_num] = false;
    PORTB &= ~(1 << (PB0 + (servo_num - 8))); // Set servo pin low
    DDRB &= ~(1 << (PB0 + (servo_num - 8)));  // Set servo pin to INPUT mode (less chatter when not doing anything)
 }
} 

void tinyServo84::setServo(byte servo_num, int angle) {
  int pulse_width = map(angle, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // convert angle to pulse width in microseconds
  pulse_width = constrain(pulse_width, SVOMINPULSE, SVOMAXPULSE);          // constrain pulse width to min and max
  if (pulse_width != tinyServo84::servo_PWs[servo_num] && tinyServo84::servo_attached[servo_num]) { // Disable interrupts only if signal changes and servo is attached
    cli();                                                                 // Disable interrupts. It's best to update volatile global variables with interrupts diabled.
    tinyServo84::servo_PWs[servo_num] = pulse_width;                       // Store new pulse_width in servo_PWs
    sei();                                                                 // Enable interrupts. Spend as little time in "disabled interrupt land" as possible.
  }
}

void tinyServo84::homeServos() {  // function to home servos
  // Note: servo will not home unless it is first enabled.
  for (byte i = 0; i < NSVO; i++) {
    tinyServo84::setServo(i, 90);  // send all servos to middle position
  }
  delay(1000);  // wait for servos to home
}

void tinyServo84::servo_timeout_check(int tol) { // tol is added for potentiometer control. Default should be zero.
  static bool timer1_enabled = false;     // keep track of whether timer1 is enabled
  static int totalLast;                   // keep track of the last total
  static unsigned long servo_timer;       // keep track of the time duration since the servo last moved
  int total = 0;
  for (int i = 0; i < NSVO; i++) {
    if (tinyServo84::servo_attached[i]) {
      total += tinyServo84::servo_PWs[i];  // add up the total pulse widths for the enabled servos. We are using this as a marker.
    }
  }
  if (abs(total - totalLast) > tol) {  // if total pulse width changed outside the defined tolerance (new setpoint requested)
    servo_timer = millis();            // reset servo_timer
    if (!timer1_enabled) {             // if Timer1 is disabled
      enableTimerInterrupt();	       // restart Timer1
      timer1_enabled = true;           // set Timer1 enabled flag to true (reduces needles switching)
    }
  }
  totalLast = total;                    // remember the total for next time

  if (((millis() - servo_timer) > SVOTIMEOUT) && timer1_enabled) {
    disableTimerInterrupt(); // disable Timer1
    timer1_enabled = false;  // set Timer1 enabled flag to false
  }
}

void tinyServo84::enableTimerInterrupt() {
  TIMSK1 |= (1 << OCIE1A);
}

void tinyServo84::disableTimerInterrupt() {
  TIMSK1 &= ~(1 << OCIE1A);
}

void tinyServo84::setCTC() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B = _BV(WGM12);
  TCCR1B |= _BV(CS11) | _BV(CS10);
  OCR1A = 2499;
  TIMSK1 |= _BV(OCIE1A);
  sei();
}

ISR(TIM1_COMPA_vect) {
  for (byte i = 0; i < NSVO; i++) {
    if (tinyServo84::servo_attached[i]) {
	  if (i < 8) {
        PORTA |= (1 << (PA0 + i));
	  } else {
		PORTB |= (1 << (PB0 + (i - 8)));
	  }
    }
  }
  while ((TCNT1 * 8) < (SVOMAXPULSE + 100)) {
    for (byte i = 0; i < NSVO; i++) {
      if (tinyServo84::servo_attached[i] && (TCNT1 * 8) > tinyServo84::servo_PWs[i]) {
	    if (i < 8) {
          PORTA &= ~(1 << (PA0 + i));
	    } else {
		  PORTB &= ~(1 << (PB0 + (i - 8)));
	    }
      }
    }
  }
}

