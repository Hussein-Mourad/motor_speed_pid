// #include <ros.h>
// #include <std_msgs/Int16.h>
#include "Timer.h"

// HardwareSerial Serial(PB11, PB10);

#define TOPIC_NAME "motor_pid"

#define Kp 0.03
#define Ki 0.005
#define Kd 0.02

#define ENC_REV_COUNT 540
#define ENC_A 4 // PA4
#define ENC_B 5 // PA5

#define MOTOR_PWM 2 //PA0
#define MOTOR_DIR 3 //PA1

#define INPUT_MAX_RANGE 300

#define MIN_IN_MS 60000 // one minute in milli seconds 
#define PWM_MAX_RANGE 65535

// A simple way to toggle debugging and changing the used serial
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugT(x, y) Serial.print(x); Serial.print(y); Serial.print("\t")
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugT(x, y)
#define debugln(x)
#endif

long enc_count = 0;

float rpm = 0;
long motorPwm = 0;

const int deltaTime = 10; // 10ms interval

int setPoint = 0;

float error, lastError, integral, derivative;

// void messageCb(const std_msgs::Int16 &sp)
// {
//   setPoint = constrain(sp.data, 0, INPUT_MAX_RANGE);
// }

// ros::NodeHandle nh;
// ros::Subscriber<std_msgs::Int16> sub(TOPIC_NAME, &messageCb);

Timer t;

void setup()
{
#if DEBUG
  Serial.begin(9600);
#endif

  // (nh.getHardware())->setPort(&Serial1);
  // (nh.getHardware())->setBaud(115200);
  // nh.initNode();
  // nh.subscribe(sub);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_B, CHANGE);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  t.every(deltaTime, calc_pid);
}



void loop()
{
  // nh.spinOnce();

  if (Serial.available())
  {
    int input = Serial.readString().toInt();
    setPoint = constrain(input, -1 * INPUT_MAX_RANGE, INPUT_MAX_RANGE);
  }

  t.update();
}

void calc_pid()
{
  enc_count += 27;

  rpm = ((float)enc_count / ENC_REV_COUNT) * (MIN_IN_MS / deltaTime);

  error = setPoint - rpm;
  integral += (error * deltaTime);
  derivative = (error - lastError) / deltaTime;
  lastError = error;
  motorPwm = Kp * error + Ki * integral + Kd * derivative;
  motorPwm -= (2 * error * deltaTime);

  if (motorPwm < 0)
    digitalWrite(MOTOR_DIR, LOW);
  else
    digitalWrite(MOTOR_DIR, HIGH);

  motorPwm = abs(motorPwm);
  motorPwm = constrain(motorPwm, 0, PWM_MAX_RANGE);
  //  motorPwm = constrain(motorPwm, 0, INPUT_MAX_RANGE);
  //  motorPwm = map(motorPwm, 0, INPUT_MAX_RANGE, 0, PWM_MAX_RANGE);

  debugT("deltaCount: ", enc_count);
  debugT("e: ", error);
  debugT("p: ", error);
  debugT("i: ", integral);
  debugT("d: ", derivative);

  debugT("PWM: ", motorPwm);
  debugT("rpm: ", rpm);
  debugT("sp: ", setPoint);
  debugln();

  enc_count = 0;

  analogWrite(MOTOR_PWM, motorPwm);
}


void ISR_A()
{
  if (digitalRead(ENC_A) != digitalRead(ENC_B))
    enc_count++;
  else
    enc_count--;
}

void ISR_B()
{
  if (digitalRead(ENC_A) == digitalRead(ENC_B))
    enc_count++;
  else
    enc_count--;
}
