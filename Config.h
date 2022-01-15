#define TOPIC_NAME "motor_pid"

#define Kp 3.0
#define Ki 0
#define Kd 0
#define Ti 60

#define ENC_A PB3  // PA4
#define ENC_B PA15 // PA5

#define MOTOR_PWM PB0 // PA0
#define MOTOR_DIR PB1 // PA1

#define ENC_REV_COUNT 540
#define INPUT_MAX_RANGE 600
#define PWM_MAX_RANGE 65535
#define MIN_IN_MS 60000 // one minute in milli seconds

// A simple way to toggle debugging and changing the used serial
#define DEBUG

#ifdef DEBUG
#define debug(x) Serial3.print(x)
#define debugT(x, y) \
  Serial3.print(x);  \
  Serial3.print(y);  \
  Serial3.print(",  ");
#define debugln(x) Serial3.println(x)
#else
#define debug(x)
#define debugT(x, y)
#define debugln(x)
#endifm
