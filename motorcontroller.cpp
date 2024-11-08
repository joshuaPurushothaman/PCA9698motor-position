#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int PWM_MIN = 0;
const int PWM_MAX = 4096;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// Btw, as a primary test, you might wanna just do regular PWM on a Pico pin (aka without PCA9685)
// There's an example under Examples -> Sweep or something?
// https://wiki-content.arduino.cc/en/Tutorial/LibraryExamples/Sweep

// WIRING: 
// VBUS to motor controller power of all sorts! Tis probably 
// I2C: gpio0 and 1 i think??? I2C is available on many pairs of pins on the pico...
// https://forum.arduino.cc/t/raspberry-pi-pico-i2c/918824/6
// GND to PCA9685

// VEX motor controller into 0'th slot on PCA9685
const int test_motor = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(1000);
}

void loop() {
  if(Serial.available() > 0) {
    // Enter power as a float from -1.0 to 1.0
    float power = Serial.parseFloat();
    setSpeed(test_motor, constrain(power, -1.0, 1.0));
  }

  delay(10);
}

void setSpeed(int motor_pin, int motor_power) {
  int pwm_power = map(motor_power, -128, 127, PWM_MIN, PWM_MAX)//value converted, og low high, to 
  pwm.setPWM(motor_pin, pwm_power, PWM_MAX - pwm_power);
}  