#include <ros.h>
#include <std_msgs/Int16.h>
#include "Timer.h"

HardwareSerial Serial3(PB11, PB10);

#define TOPIC_NAME "motor_pid"

#define Kp 0.07
#define Ki 0.03
#define Kd 0.02

#define ENC_REV_COUNT 540
#define ENC_A PA4
#define ENC_B PA5

#define MOTOR_PWM PA0
#define MOTOR_DIR PA1

#define INPUT_MAX_RANGE 300

#define MIN_IN_SECS 60
#define PWM_RANGE 65535

// A simple way to toggle debugging and changing the used serial
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial3.print(x)
#define debugln(x) Serial3.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

const int interval = 10; // 10 ms

volatile long long enc_counter = 0, last_count;

int rpm = 0;

unsigned long  deltaTime = 0;

int setPoint = 0;
double error, lastError, proportional, integral, derivative;

void messageCb(const std_msgs::Int16 &sp)
{
  setPoint = constrain(sp.data, 0, INPUT_MAX_RANGE);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> sub(TOPIC_NAME, &messageCb);

Timer t;

void setup()
{
  Serial3.begin(115200);

//  (nh.getHardware())->setPort(&Serial1);
//  (nh.getHardware())->setBaud(115200);
  // nh.initNode();
  // nh.subscribe(sub);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_B, CHANGE);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  t.every(interval, calc_pid);
}

void loop()
{
  // nh.spinOnce();
  
  if (Serial3.available()) {
    int input = Serial3.readString().toInt();
    setPoint = constrain(input, -1* INPUT_MAX_RANGE, INPUT_MAX_RANGE);
  }
  

  t.update();
}

void calc_pid() {
  deltaTime = interval;
  int deltaCount = enc_counter - last_count;
  rpm = (( (float) deltaCount / ENC_REV_COUNT)) * (MIN_IN_SECS * 1000.0 / deltaTime);

  error = setPoint - rpm;
  integral += (error * deltaTime);
  derivative = (error - lastError) / deltaTime;
  lastError = error;
  int output = Kp * error + Ki * integral + Kd * derivative;

  if (output < 0) {
    digitalWrite(MOTOR_DIR, LOW);
  } else {
    digitalWrite(MOTOR_DIR, HIGH);
  }

  output = abs(output);
  output = constrain(output, 0, INPUT_MAX_RANGE);
  output = map(output, 0, INPUT_MAX_RANGE, 0, PWM_RANGE);

  debug("PWM VALUE: ");
  debug(output);
  debug('\t');
  debug("Encoder Count: ");
  debug(enc_counter);
  debug('\t');
  debug(" SPEED: ");
  debug(rpm);
  debug('\t');
  debug("SP: ");
  debug(setPoint);
  debugln(" ");

  last_count =enc_counter;
//  enc_counter = 0;

  analogWrite(MOTOR_PWM, output);
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
