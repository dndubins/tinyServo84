// servoMasterUno.ino
// Uno as an I2C Master, with Attiny84 as slave. Slave sketch: servoSlave84.ino.
// tinyServo84.h version 1.0.5
// Author: David Dubins
// Date: 10-Feb-25
// Last Updated: 21-May-26
// Libraries:
//   Written to work with TinyWireS.h available here: https://github.com/rambo/TinyWire
//   Adapted from: https://pwbotics.wordpress.com/2021/05/05/programming-attiny85-and-i2c-communication-using-attiny85/

#define SERIALDEBUG
//#define CALIBRATE     // asks for and expects entire position array from slave
#define PACKET_LEN 7  // length of posArr[] including checkSum at end
// Servo Parameters
#define STEPDELAY 10  // delay for servo commands (relates to speed). Default: 50

// ATtiny84 Slave Parameters
#include <Wire.h>        // start Wire.h as master (no address needed)
#define I2C_ADDR1 0x08   // I2C address of the ATtiny84 slave (0x08)
bool rcv = false;        // flag for new received data
int16_t txCheckSum = 0;  // checksum for communications validation

// Structure to hold servo data
struct ServoStruct {
  uint8_t Pin;   // not used in this sketch, just to keep track
  uint8_t MIN;   // minimum servo position
  uint8_t MAX;   // maximum servo position
  uint8_t HOME;  // home servo position
  uint8_t POS;   // keep track of current servo position
} sData[6];      //declare strutured array with 6 servos

// Servo data to send to ATtiny84 over I2C
uint8_t TXdata[PACKET_LEN];
uint8_t RXdata[PACKET_LEN];
enum ServoId { LX, LY, LB, RX, RY, RB, CHECKSUM }; // for more intelligible indexing

int checktot = 0;     // to check if motion is required
int checklast = 0;

void setup() {
  Wire.begin();  // Initialize I2C as master
  //Wire.setClock(50000);           // slower clock speed
  Serial.begin(115200);  // Start serial communication
  setServos();           // initialize sData[] with correct values
  Serial.println("Master ready.");  // send welcome msg
  homeServos();
}

void loop() {
#ifdef CALIBRATE
  // Receive data from slave
  if (receiveFromSlave()) {  // Receive RXdata from slave
    printLoc(RXdata);        // Print the received struct
  } else {
    Serial.println("Data communications error.");
  }
  delay(500);  // Small delay to avoid overloading the slave
#else          // Regular routine
  Serial.println("MIN:");
  moveTo(sData[0].MIN, sData[1].HOME, sData[2].HOME, sData[3].HOME, sData[4].HOME, sData[5].HOME, STEPDELAY);  // move to first location
  printLoc(TXdata);                                          // Print the sent coordinates
  delay(1000);                                               // wait between receiving and sending

  Serial.println("MAX:");
  moveTo(sData[0].MAX, sData[1].HOME, sData[2].HOME, sData[3].HOME, sData[4].HOME, sData[5].HOME, STEPDELAY);  // move to first location
  printLoc(TXdata);                                          // Print the sent coordinates
  delay(1000);                                               // wait between receiving and sending
#endif
}

bool receiveFromSlave() {  // receive the whole payload
#ifdef CALIBRATE
  int i = 0;                                    // keep track of # bytes received
  Wire.requestFrom(I2C_ADDR1, sizeof(RXdata));  // Request of size of integer (just need the checkSum)
  while (Wire.available() && i < PACKET_LEN) {
    RXdata[i++] = Wire.read();  // Read the next byte from the slave
  }
  return (i == sizeof(RXdata) && validate_checkSum(RXdata, PACKET_LEN));  // new data has been received of the correct size, and matches txCheckSum
#else                                                                     // only expecting one byte back
  int i = 0;                                     // keep track of # bytes received
  Wire.requestFrom(I2C_ADDR1, sizeof(uint8_t));  // Request of size of byte (just need the rxCheckSum)
  while (Wire.available()) {
    RXdata[i++] = Wire.read();  // Read the next byte from the slave
  }
  return (i == sizeof(uint8_t) && RXdata[0] == txCheckSum);  // new data has been received of the correct size, and rxCheckSum matches txCheckSum
#endif
}

bool sendToSlave(uint8_t* a, size_t len) {  // size_t is the variable type returned by sizeof().
  Wire.beginTransmission(I2C_ADDR1);        // Start I2C transmission to slave
  Wire.write(a, len);                       // Send the data as the char array myCharArr (size of ServoStruct)
  return (Wire.endTransmission() == 0);     // endTransmission() returns a 0 on success
}

// fill values of sData
void setServos() {   //set up the physical parameters for all servos
                     //sData[].Pin is digital pin # for Servo (placeholder). Not used in this sketch.
  sData[0].Pin = 1;  //digital pin for Servo (PA1)
  sData[0].MIN = 5;
  sData[0].MAX = 174;
  sData[0].HOME = 69;

  sData[1].Pin = 2;  //digital pin for Servo (PA2)
  sData[1].MIN = 2;
  sData[1].MAX = 172;
  sData[1].HOME = 76;

  sData[2].Pin = 3;  //digital pin for Servo (PA3)
  sData[2].MIN = 2;
  sData[2].MAX = 178;
  sData[2].HOME = 90;

  sData[3].Pin = 5;  //digital pin for Servo (PA5)
  sData[3].MIN = 5;
  sData[3].MAX = 171;
  sData[3].HOME = 74;

  sData[4].Pin = 7;  //digital pin for Servo (PA7)
  sData[4].MIN = 5;
  sData[4].MAX = 170;
  sData[4].HOME = 45;

  sData[5].Pin = 8;  //digital pin for Servo (PB2)
  sData[5].MIN = 119; // 118 abolute min (claw open)
  sData[5].MAX = 170; // 172 absolute max (claw closed)
  sData[5].HOME = 120; // use min

  // set inital position of servos
  for (int i = 0; i < 6; i++) {
    sData[i].POS = sData[i].HOME;
  }
}

void printLoc(uint8_t* pos) {  // print location to serial monitor
  for (int i = 0; i < 6; i++) {
    Serial.print(pos[i]);
    (i < 5) ? Serial.print(", ") : Serial.println("");
  }
}


// linear moveTo() routine
void moveTo(byte SV0, byte SV1, byte SV2, byte SV3, byte SV4, byte SV5, int sSpeed) {  // manually move to a spot
  byte S[6] = { SV0, SV1, SV2, SV3, SV4, SV5 };
  int delta[6];
  int dev;
  for (int i = 0; i < 6; i++) {
    S[i] = constrain(S[i], sData[i].MIN, sData[i].MAX);
  }
  do {
    dev = 0;
    for (int i = 0; i < 6; i++) {
      delta[i] = 0;
      if (S[i] > sData[i].POS) delta[i] += 1;  // Can't use STEPSIZE here or dev can't be zero
      if (S[i] < sData[i].POS) delta[i] -= 1;
      sData[i].POS = constrain(sData[i].POS + delta[i], sData[i].MIN, sData[i].MAX);
      TXdata[i] = sData[i].POS;
      dev += abs(sData[i].POS - S[i]);
    }
    calculate_checkSum(TXdata, PACKET_LEN);      // calculate the checksum. Change payload manually here (7 int numbers in payload)
    txCheckSum = TXdata[PACKET_LEN - 1];         // store txCheckSum before sending
    if (!sendToSlave(TXdata, sizeof(TXdata))) {  // Send TXdata to slave
#ifdef SERIALDEBUG
      Serial.println("Communications error sending data to slave.");
#endif
    }
    delay(sSpeed);  //give a chance to get to setpoint
  } while (dev != 0);
#ifdef SERIALDEBUG
  Serial.println("moveTo(" + (String)sData[0].POS + "," + (String)sData[1].POS + "," + (String)sData[2].POS + "," + (String)sData[3].POS + "," + (String)sData[4].POS + "," + (String)sData[5].POS + "," + (String)sSpeed + ");");  // replace last # with desired speed
#endif
}

// smooth moveTo() routine
void moveTo_smooth(byte SV0, byte SV1, byte SV2, byte SV3, byte SV4, byte SV5, int sSpeed) {
  byte target[6] = { SV0, SV1, SV2, SV3, SV4, SV5 };
  int start[6];
  int delta[6];
  int maxDelta = 0;
  // Hard sinc at start
  for (int i = 0; i < 6; i++) {
    TXdata[i] = sData[i].POS;
  }
  calculate_checkSum(TXdata, PACKET_LEN);
  sendToSlave(TXdata, sizeof(TXdata));

  for (int i = 0; i < 6; i++) {
    target[i] = constrain(target[i], sData[i].MIN, sData[i].MAX);
    start[i] = sData[i].POS;
    delta[i] = target[i] - start[i];
    maxDelta = max(maxDelta, abs(delta[i]));
  }
  // number of interpolation steps
  int steps = maxDelta;
  if (steps == 0) return;  // leave if not moving anywhere
  for (int step = 0; step <= steps; step++) {
    // normalized 0→1
    float t = (float)step / steps;
    // smoothstep easing
    //float ease = t * t * (3.0 - 2.0 * t);
    // cosine easing
    float ease = 0.5 - 0.5 * cos(t * PI);

    for (int i = 0; i < 6; i++) {
      sData[i].POS = start[i] + delta[i] * ease;
      TXdata[i] = sData[i].POS;
    }
    calculate_checkSum(TXdata, PACKET_LEN);
    txCheckSum = TXdata[PACKET_LEN - 1];         // store txCheckSum before sending
    if (!sendToSlave(TXdata, sizeof(TXdata))) {  // Send TXdata to slave
#ifdef SERIALDEBUG
      Serial.println("Communications error sending data to slave.");
#endif
    }
    delay(sSpeed);
  }
  for (int i = 0; i < 6; i++) {
    sData[i].POS = target[i];  // snap to exact target
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

void homeServos() {  //home the servos
  Serial.println("***** Homing servos. *****");
  moveTo(sData[0].HOME, sData[1].HOME, sData[2].HOME, sData[3].HOME, sData[4].HOME, sData[5].HOME, 50);  //Home slowly
  Serial.println("Finished homing.");
}
