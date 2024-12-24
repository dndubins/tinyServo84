// tinyServo84.cpp file for tinyServo84.h library.
// Control up to 11 servos from pins PA0...PA7, and PB0...PB2
// using CTC mode of Timer1 (ATtiny84)
// Clock speed = 8MHz
// Transferability: This library is only designed to work on the ATtiny84.
// Author: D.Dubins
// Date: 19-Dec-24
// Last Updated: 23-Dec-24
//
// The library maps specific servo numbers to the following pins:
// servo 0: PA0   servo 3: PA3   servo 6: PA6   servo 9: PB1
// servo 1: PA1   servo 4: PA4   servo 7: PA7   servo 10: PB2
// servo 2: PA2   servo 5: PA5   servo 8: PB0

#include "tinyServo84.h"
#include <avr/io.h>
#include <avr/interrupt.h>

unsigned int tinyServo84::servo_PWs[NSVO] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };  // pulse-width of mid-points of each servo
bool tinyServo84::servo_attached[NSVO] = { false, false, false, false, false, false, false, false, false, false, false }; // each pin starts out unattached
bool tinyServo84::timer1_enabled = false; // to keep track if Timer1 is enabled
unsigned long tinyServo84::servo_tLast = 0UL; // To store the time the last servo was used (timeout function)

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
  if(!tinyServo84::timer1_enabled)tinyServo84::enableTimerInterrupt();     // enable Timer1 in case it timed out
  int pulse_width = map(angle, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // convert angle to pulse width in microseconds
  pulse_width = constrain(pulse_width, SVOMINPULSE, SVOMAXPULSE);          // constrain pulse width to min and max
  if (pulse_width != tinyServo84::servo_PWs[servo_num] && tinyServo84::servo_attached[servo_num]) { // Disable interrupts only if signal changes and servo is attached
    cli();                                                                 // Disable interrupts. It's best to update volatile global variables with interrupts diabled.
    tinyServo84::servo_PWs[servo_num] = pulse_width;                       // Store new pulse_width in servo_PWs
    sei();                                                                 // Enable interrupts. Spend as little time in "disabled interrupt land" as possible.
    tinyServo84::servo_tLast = millis();                                   // Record time servo was last used for timeout function
  }
}

void tinyServo84::homeServos() {  // function to home servos
  // Note: servo will not home unless it is first enabled.
  for (byte i = 0; i < NSVO; i++) {
    tinyServo84::setServo(i, 90);  // send all servos to middle position
  }
  delay(1000);  // wait for servos to home
}

void tinyServo84::setCTC() {  // function to set the registers of the ATtiny84 for Timer 1 CTC mode
  // Setting up Timer1 for 1µs ticks (assuming 8MHz clock)
  cli();		  // stop interrupts
  TCCR1A = 0;		  // clear timer control register A
  TCCR1B = 0;		  // clear timer control register B
  TCNT1 = 0;		  // set counter to 0
  TCCR1B = _BV(WGM12);    // CTC mode (Table 12-5 on ATtiny84 datasheet)
  TCCR1B |= _BV(CS11) | _BV(CS10);  // prescaler=64
  OCR1A = 2499;  //OCR1A=(fclk/(N*frequency))-1 (where N is prescaler).
  TIMSK1 |= _BV(OCIE1A);  // enable timer compare
  sei();                  // enable interrupts
  timer1_enabled=true;    // timer1 is now enabled
}

ISR(TIM1_COMPA_vect) {  // This is the ISR that will turn off the pins at the correct widths
  //The function micros() does not advance inside the ISR.
  //TCNT1 starts at 0 and counts up. Each increment lasts 8 microseconds. We are going to use this as a timer.
  //8 microseconds gives us (2500-500us)/8us =250 steps. This is fine for a 180 degree servo. For a 360 degree servo,
  //if more steps are needed, you can set the timer to N=8, OCR1A=19999 and this will give 2000 steps, but require
  //more attention by the ISR.
  for (byte i = 0; i < NSVO; i++) {    
    if (tinyServo84::servo_attached[i]) { // only turn on pin if servo attached
	  if (i < 8) {
        PORTA |= (1 << (PA0 + i));  // turn on pins in PORTA
	  } else {
		PORTB |= (1 << (PB0 + (i - 8))); // turn on pins in PORTB
	  }
    }
  }
  while ((TCNT1 * 8) < (SVOMAXPULSE + 100)) {  // multiply TCNT1 by microseconds/step
    // a 50 Hz pulse has a period of 20,000 us. We just need to make it past SVOMAXPULSE with a small buffer (100 microseconds).
    for (byte i = 0; i < NSVO; i++) {
      if (tinyServo84::servo_attached[i] && (TCNT1 * 8) > tinyServo84::servo_PWs[i]) {   // Turn off the servo pin if the timer exceeds the pulse width
	    if (i < 8) {
          PORTA &= ~(1 << (PA0 + i));  // turn off pins in PORTA
	    } else {
		  PORTB &= ~(1 << (PB0 + (i - 8)));  // turn off pins in PORTB
	    }
      }
    }
  }
}

void tinyServo84::enableTimerInterrupt() {
  TIMSK1 |= (1 << OCIE1A);   // enable Timer1
  tinyServo84::timer1_enabled = true; 
}

void tinyServo84::disableTimerInterrupt() {
  TIMSK1 &= ~(1 << OCIE1A);  // disable Timer1
  tinyServo84::timer1_enabled = false;
}

void servo_timeout_check() {  // check to shut down Timer1 after SVOTIMEOUT msec have elapsed since last servo move
  if (((millis() - tinyServo84::servo_tLast) > SVOTIMEOUT) && tinyServo84::timer1_enabled) {
    tinyServo84::disableTimerInterrupt();  // disable Timer1
  }
}
