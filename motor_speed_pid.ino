// #include <ros.h>
// #include <std_msgs/Int16.h>
#include <Timer.h>

HardwareSerial Serial3(PB11, PB10);

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
#endif

volatile long encoderCount = 0, lastCount = 0;

float rpm = 0;
long motorPwm = 0;

const float deltaTime = 20; // 10ms interval

int setPoint = 0;

float error, lastError, integral, derivative;

long long currentMillis = 0, previousMillis = 0;

// void messageCb(const std_msgs::Int16 &sp)
// {
//   setPoint = constrain(sp.data, 0, INPUT_MAX_RANGE);
// }

// ros::NodeHandle nh;
// ros::Subscriber<std_msgs::Int16> sub(TOPIC_NAME, &messageCb);

Timer t;

void setup()
{

#ifdef DEBUG
  Serial3.begin(115200);
#endif

  // (nh.getHardware())->setPort(&Serial1);
  // (nh.getHardware())->setBaud(115200);
  // nh.initNode();
  // nh.subscribe(sub);

  analogWriteFrequency(2000); 
  analogWriteResolution(16); 


  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_B, CHANGE);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  t.every(deltaTime, calc_pid);
  // analogWrite(MOTOR_PWM, 20000);
}

void loop()
{
  // nh.spinOnce();

   if (Serial3.available()) {
     int input = Serial3.readString().toInt();
    setPoint = constrain(input, -1 * INPUT_MAX_RANGE, INPUT_MAX_RANGE);
   }

  // setPoint = 400;

  // currentMillis = millis();
  // if (currentMillis - previousMillis >= deltaTime)
  // {
    // previousMillis = currentMillis;
    // calc_pid();
  // }
  t.update();
  // analogWrite(MOTOR_PWM, 2000);
}

void calc_pid()
{
  rpm = ((float)(encoderCount - lastCount) / ENC_REV_COUNT) * (MIN_IN_MS / deltaTime);

  error = setPoint - rpm;
  integral += (error * deltaTime);
  derivative = (error - lastError) / deltaTime;
  lastError = error;
  float Kc = 1 / Kp;
  // motorPwm = Kc * error + (Kc/Ti) * integral + Kd * derivative;
  motorPwm = Kc * error;

  if (motorPwm < 0)
    digitalWrite(MOTOR_DIR, LOW);
  else
    digitalWrite(MOTOR_DIR, HIGH);

  motorPwm = abs(motorPwm);
  if (motorPwm > INPUT_MAX_RANGE)
    integral -= (2 * error * deltaTime);

  motorPwm = constrain(motorPwm, 0, INPUT_MAX_RANGE);
  // motorPwm = constrain(motorPwm, 0, PWM_MAX_RANGE);

  motorPwm = map(motorPwm, 0, INPUT_MAX_RANGE, 0, PWM_MAX_RANGE);
  motorPwm = constrain(motorPwm, 0, PWM_MAX_RANGE);

  debugT("encoder ", encoderCount);
  debugT("lastCount ", lastCount);

  debugT("deltaCount: ", encoderCount - lastCount);
  //  debugT("e: ", error);
  debugT("p: ", Kc * error);
  //  debugT("i: ", integral);

  debugT("PWM: ", motorPwm);
  debugT("rpm: ", rpm);
  debugT("sp: ", setPoint);
  debugln();

  lastCount = encoderCount;

  analogWrite(MOTOR_PWM, motorPwm);
}

void ISR_A()
{
  digitalRead(ENC_A) != digitalRead(ENC_B) ? encoderCount++ : encoderCount--;
}

void ISR_B()
{
  digitalRead(ENC_A) == digitalRead(ENC_B) ? encoderCount++ : encoderCount--;
}
