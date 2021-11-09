// https://github.com/imax9000/Arduino-PID-Library
#include <PID_v2.h>

#define Kp 0.5
#define Ki 2
#define Kd 1

#define ENC_REV_COUNT 2500
#define ENC_A PB12
#define ENC_B PB13

#define MOTOR_PWM PB1
#define MOTOR_DIR PB0

#define MIN_IN_SECS 60
#define PWM_RANGE 65535

// A simple way to toggle debugging and changing the used serial
#define DEBUG 0

#if DEBUG == 1
#define debug(x) Serial1.print(x)
#define debugln(x) Serial1.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

const int interval = 1000; // 1000 ms

volatile long long enc_counter = 0;

int rpm = 0, motorPwm = 0;

long previousMillis = 0, currentMillis = 0;

long setPoint = 0;

PID_v2 pid(Kp, Ki, Kd, PID::Direct);

void setup()
{
  Serial1.begin(9600);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_B, CHANGE);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  previousMillis = millis();

  pid.SetOutputLimits(0, PWM_RANGE);
  pid.Start(rpm, 0, setPoint); // input, outpt, setpoint
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // Calculate RPM
    rpm = (float)(enc_counter * MIN_IN_SECS / ENC_REV_COUNT);

    // Get pid output
    motorPwm = pid.Run(rpm);

    analogWrite(MOTOR_PWM, motorPwm);

    // Only update display when there is a reading
    if (motorPwm > 0 || rpm > 0) {
      debug("PWM VALUE: ");
      debug(motorPwm);
      debug('\t');
      debug(" PULSES: ");
      debug(enc_counter);
      debug('\t');
      debug(" SPEED: ");
      debug(rpm);
      debugln(" RPM");
    }

    enc_counter = 0;
  }
}

void ISR_A()
{
  if (digitalRead(ENC_A) != digitalRead(ENC_B))
    enc_counter++;
  else
    enc_counter--;
}

void ISR_B()
{
  if (digitalRead(ENC_A) == digitalRead(ENC_B))
    enc_counter++;
  else
    enc_counter--;
}
