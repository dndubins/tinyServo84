/* servoSlave84.ino
   ATtiny84 as an I2C slave. Master sketch: servoMasterUno.ino
   tinyServo84.h version 1.0.4
   Author: David Dubins
   Date: 08-May-26
   Last Updated: 20-May-26
   Written to work with TinyWireS.h available here: https://github.com/rambo/TinyWire
   Adapted from: https://pwbotics.wordpress.com/2021/05/05/programming-ATtiny84-and-i2c-communication-using-ATtiny84/

   The following are the ATtiny84 pins by function:
   -----------------------------------------------
   Pin 1: Vcc (1.8-5.5V)
   Pin 2: 10/XTAL1/PCINT8/PB0
   Pin 3: 9/XTAL2/PCINT9/PB1
   Pin 4: dW/RESET/PCINT11/PB3
   Pin 5: PWM/OC0A/CKOUT/8/INT0/PCINT10/PB2
   Pin 6: PWM/ICP/OC0B/7/A7/ADC7/PCINT7/PA7
   Pin 7: PWM/MOSI/SDA/OC1A/6/A6/ADC6/PCINT6/PA6

   Pin 8: PWM/D0/OC1B/MISO/5/A5/ADC5/PCINT5/PA5
   Pin 9: T1/SCL/SCK/4/A4/ADC4/PCINT4/PA4
   Pin 10: 3/A3/ADC3/PCINT3/PA3
   Pin 11: 2/A2/ADC2/PCINT2/PA2
   Pin 12: 1/A1/ADC1/PCINT1/PA1
   Pin 13: AREF/0/A0/ADC0/PCINT0/PA0
   Pin 14: GND

   Wiring:
   -------
   ATtiny84 - Uno

   Pin 14 - GND
   Pin 7 - SDA (use 10K pullup)
   Pin 9 - SCL (use 10K pullup)
   Pin 1 - 5V

   The TinyServo84.h library maps specific servo numbers to the following pins:
   servo 0: PA0   servo 3: PA3   servo 6: PA6   servo 9: PB1
   servo 1: PA1   servo 4: PA4   servo 7: PA7   servo 10: PB2
   servo 2: PA2   servo 5: PA5   servo 8: PB0
 */

// I2C parameters
//#define CALIBRATE  // if defined, returns the whole posArr[] to the master
#include <TinyWireS.h>
#define I2C_ADDR 0x08  // ATtiny84 I2C Address

// Servo parameters
#include <tinyServo84.h>
// Change these values to suit your application:
#define NSERVO 6          // number of servos to control (up to 11)
#define SVOMAXANGLE 179   // maximum angle for servo.
#define SVOMINPULSE 500   // minimum pulse width in microseconds for servo signal (0 degrees)
#define SVOMAXPULSE 2500  // maximum pulse width in microseconds for servo signal (for maximum angle)
#define SVOTIMEOUT 10000  // timeout in ms to disable servos.
#define ARMDELAY 20       // delay for servo commands (relates to speed). Default: 20
#define PACKET_LEN 7      // length of posArr[] including checkSum at end

// Map the servos you need here:
//                        0   1   2   3   4   5   6   7   8   9  10
//                       PA0 PA1 PA2 PA3 PA4 PA5 PA6 PA7 PB0 PB1 PB2
byte s_index[6] = { 1, 2, 3, 5, 7, 10 };  // servo numbers for PCB board (6 of them)
bool s_active[6] = { 0, 1, 1, 0, 0, 0 };  // active servos for this project (subset)

byte potPin = A0;  // analog reading of pot

tinyServo84 myServos;              // declare object called myServos of class tinyServo84

uint8_t RXdata[PACKET_LEN];   //position array (6 elements + checkSum)
volatile uint8_t RXtemp[PACKET_LEN]; // for copying from ISR (safer)
volatile bool frameReady = false;  // new location ready flag for ISR

void setup() {
  TinyWireS.begin(I2C_ADDR);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
  myServos.setCTC();  // set CTC mode for Timer 1
  // Attach the active servos here:
  for (int i = 0; i < NSERVO; i++) {
    if (s_active[i]) {
      myServos.attachServo(s_index[i]);
    }
  }
}

void loop() {  // The slave will continuously wait for requests or data from the master.
  // Uncomment the following to test servo limits using the onboard potentiometer:
#ifdef CALIBRATE
  static int lastDiv = -1;
  int div = map(analogRead(potPin), 0, 1023, 0, 179);
  if (abs(div - lastDiv) >= 2) {  // 2-degree deadband
    lastDiv = div;
    for (int i = 0; i < NSERVO; i++) {
      if (s_active[i]) {
        RXdata[i] = div;
      }
    }
    calculate_checkSum(RXdata, PACKET_LEN);
    moveTo(RXdata);
  }
#else
  if (frameReady) {
    noInterrupts();
    for(int i=0;i<PACKET_LEN;i++){ // copy back RXtemp[] to RXdata[]
      RXdata[i] = RXtemp[i];
    }
    frameReady = false;
    interrupts();
    if(validate_checkSum(RXdata, PACKET_LEN)){
      // don't nag the servos:
      /* static unsigned long lastServoUpdate = 0;
      if(millis() - lastServoUpdate >= 20){
        lastServoUpdate = millis();
        moveTo(RXdata);
      }*/
      moveTo(RXdata);
    }
  }
#endif
  TinyWireS_stop_check();  // detect I2C STOP and reset USI state (needs to be in the loop)
  //comment the next line out to disable servo timeouts:
  myServos.servo_timeout_check(SVOTIMEOUT);  // if servos are inactive, stop servos
}

// Function to handle data received from the master
void receiveEvent() {
  uint8_t temp[PACKET_LEN];
  int i = 0;
  while (TinyWireS.available() && i < PACKET_LEN) {
    temp[i++] = TinyWireS.receive();  // Receive the byte from the master
  }
  //check data: RXdata should be the correct length. Checksum should be done in the loop.
  if (i == PACKET_LEN) {
    for(int j=0;j<PACKET_LEN;j++){ // copy back temp[] to RXtemp[]
      RXtemp[j] = temp[j];
    }
    frameReady = true;                                             // good data received, flag loop to move servos
  }
}

// Function to send data to the master when requested
void requestEvent() {
#ifdef CALIBRATE
  //sendArr(localRX, PACKET_LEN);  // send frame back to the Master to see it.
  sendArr(RXdata, PACKET_LEN);  // send frame back to the Master to see it.
#else
  sendCheckSum(RXdata[PACKET_LEN - 1]);  // send the last checkSum back as vrification that data was received
#endif
}

void sendArr(uint8_t* arr, int len) {  // send a fixed length
  for (int i = 0; i < len; i++) {
    TinyWireS.send(arr[i]);
  }
}

void sendCheckSum(uint8_t c) {
  TinyWireS.send(c);
}

// moveTo is as fast as possible (control speed and constrain using the master)
void moveTo(uint8_t a[]) {  // manually move to a spot
  for (int i = 0; i < NSERVO; i++) {
    if (s_active[i]) {
      byte pos = constrain(a[i], 0, SVOMAXANGLE);  // prevents unreasonable asks
      myServos.setServo(s_index[i], pos);
    }
  }
}

// To calculate the checksum for data to be transmitted
void calculate_checkSum(uint8_t a[], uint8_t size) {  // last element of array will hold checksum value
  uint16_t sum = 0;
  for (int i = 0; i < (size - 1); i++) {
    sum += a[i];
  }
  a[size - 1] = (sum % 256);
}

// To validate the checksum for received data
bool validate_checkSum(uint8_t a[], uint8_t size) {
  uint16_t sum = 0;
  for (int16_t i = 0; i < (size - 1); i++) {
    sum += a[i];
  }
  return (a[size - 1] == (sum % 256));  // return result of checksum
}
