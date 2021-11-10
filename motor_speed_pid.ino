#include <ros.h>
#include <std_msgs/Int16.h>

HardwareSerial Serial3(PB11, PB10);

#define TOPIC_NAME "motor_pid"

#define Kp 0.5
#define Ki 2
#define Kd 1

#define ENC_REV_COUNT 2500
#define ENC_A PB12
#define ENC_B PB13

#define MOTOR_PWM PB1
#define MOTOR_DIR PB0

#define motorDir HIGH

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

const int interval = 1000; // 1000 ms

volatile long long enc_counter = 0;

int rpm = 0, motorPwm = 0;

unsigned long previousMillis = 0, currentMillis = 0, deltaTime=0;

int16_t setPoint = 0;
double error,lastError, proportional, integral, derivative;

void messageCb(const std_msgs::Int16 &sp)
{
  setPoint = sp.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> sub(TOPIC_NAME, &messageCb);

void setup()
{
  Serial3.begin(115200);

  (nh.getHardware())->setPort(&Serial1);
  (nh.getHardware())->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_B, CHANGE);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  previousMillis = millis();
}

void loop()
{
  currentMillis = millis();
  deltaTime = currentMillis - previousMillis;

  if (deltaTime > interval)
  {
    previousMillis = currentMillis;

    // Calculate RPM
    rpm = (float)(enc_counter * MIN_IN_SECS / ENC_REV_COUNT);

    // get motor pid value
    motorPwm = calc_pid(rpm, deltaTime);

    // set motor output
    analogWrite(MOTOR_PWM, motorPwm);
    digitalWrite(MOTOR_DIR, motorDir);

    // Only update display when there is a reading
    if (motorPwm > 0 || rpm > 0)
    {
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
  nh.spinOnce();
}

int calc_pid(int input, unsigned long deltaTime){
  error = setPoint - input;
  integral += (error * deltaTime);
  derivative = (error - lastError) / deltaTime;
  lastError = error;
  int output = Kp * error + Ki * integral + Kd * derivative;
  output = min(0,max(output, PWM_RANGE));
  return output;
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
