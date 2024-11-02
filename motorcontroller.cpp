#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int PWM_MIN = 0;
const int PWM_MAX = 4096;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(1000);
}

void loop() {
  if(Serial.available() > 0) {

  }
}

void setSpeed(int motor_pin, int motor_power) {
  int pwm_power = map(motor_power, -128, 127, PWM_MIN, PWM_MAX)//value converted, og low high, to 
  pwm.setPWM(motor_pin, pwm_power, PWM_MAX - pwm_power);
}  