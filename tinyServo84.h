#ifndef tinyServo84_h
#define tinyServo84_h

#include <Arduino.h>

#define NSVO 11           // number of servos to control (up to 11)
#define SVOMAXANGLE 179   // maximum angle for servo.
#define SVOMINPULSE 500   // minimum pulse width in microseconds for servo signal (0 degrees)
#define SVOMAXPULSE 2500  // maximum pulse width in microseconds for servo signal (for maximum angle)
#define SVOTIMEOUT 500    // timeout in ms to disable servos.

class tinyServo84 {
  public:
    tinyServo84();
    void attachServo(byte servo_num);
    void detachServo(byte servo_num);
    void setServo(byte servo_num, int angle);
    void homeServos();
    void setCTC();
    void enableTimerInterrupt();
    void disableTimerInterrupt();
    void servo_timeout_check();
    static unsigned int servo_PWs[NSVO];    // Pulse widths in microseconds
    static bool servo_attached[NSVO];       // Servo attachment status
    static bool timer1_enabled;		    // To keep track if Timer1 is enabled
    static unsigned long servo_tLast;	    // To store the time the last servo was used (timeout function)
};

#endif
