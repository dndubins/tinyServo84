// tinyServo84.cpp file for tinyServo84.h library.
// The following pin numbers are used:
// servo #0..7: PA0..PA7 (8 servos)
// servo #8..10: PB0..PB2 (3 servos)

#include "tinyServo84.h"
#include <avr/io.h>
#include <avr/interrupt.h>

unsigned int tinyServo84::servo_PWs[NSVO] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
bool tinyServo84::servo_attached[NSVO] = { false, false, false, false, false, false, false, false, false, false, false };

tinyServo84::tinyServo84() {
  // Constructor - any initialization code you need
}

void tinyServo84::attachServo(byte servo_num) {
  if (servo_num < 8) {
    tinyServo84::servo_attached[servo_num] = true;
    DDRA |= (1 << (PA0 + servo_num));  // Set servo pin in DDRA to OUTPUT mode
  }else if (servo_num < NSVO){
	tinyServo84::servo_attached[servo_num] = true;
    DDRB |= (1 << (PB0 + (servo_num - 8)));  // Set servo pin in DDRB to OUTPUT mode
  }
}

void tinyServo84::detachServo(byte servo_num) {
  if (servo_num < 8) {
    tinyServo84::servo_attached[servo_num] = false;
    PORTA &= ~(1 << (PA0 + servo_num)); // Set servo pin low
    DDRA &= ~(1 << (PA0 + servo_num));  // Set servo pin to INPUT mode
 } else {
    tinyServo84::servo_attached[servo_num] = false;
    PORTB &= ~(1 << (PB0 + (servo_num - 8))); // Set servo pin low
    DDRB &= ~(1 << (PB0 + (servo_num - 8)));  // Set servo pin to INPUT mode
 }
} 

void tinyServo84::setServo(byte servo_num, int angle) {
  int pulse_width = map(angle, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE); // convert angle to pulse width in microseconds
  pulse_width = constrain(pulse_width, SVOMINPULSE, SVOMAXPULSE);  // constrain pulse width to min and max
  if (pulse_width != tinyServo84::servo_PWs[servo_num] && tinyServo84::servo_attached[servo_num]) { // Disable interrupts only if signal changes and servo is attached
    cli();  // Disable interrupts. It's best to update volatile global variables with interrupts diabled.
    tinyServo84::servo_PWs[servo_num] = pulse_width; // Store new pulse_width in servo_PWs
    sei();
  } // Enable interrupts. Spend as little time in "disabled interrupt land" as possible.
}

void tinyServo84::homeServos() {
  for (byte i = 0; i < NSVO; i++) {
    tinyServo84::setServo(i, 90);
  }
  delay(1000);
}

void tinyServo84::servo_timeout_check(int tol) { // tol is added for potentiometer control. Default should be zero.
  static bool timer1_enabled = false;
  static int totalLast;
  static unsigned long servo_timer;
  int total = 0;
  for (int i = 0; i < NSVO; i++) {
    if (tinyServo84::servo_attached[i]) {
      total += tinyServo84::servo_PWs[i];
    }
  }
  if (abs(total - totalLast) > tol) {  // if changes outside tolerance
    servo_timer = millis();
    if (!timer1_enabled) {
      enableTimerInterrupt();	       // restart Timer1
      timer1_enabled = true;           // set enabled flag to true
    }
  }
  totalLast = total;

  if (((millis() - servo_timer) > SVOTIMEOUT) && timer1_enabled) {
    disableTimerInterrupt(); // disable Timer1
    timer1_enabled = false;  // set enabled flag to false
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

