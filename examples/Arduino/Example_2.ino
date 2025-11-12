#include <Arduino.h>

// LED pins
#define LED1 D6
#define LED2 D7

// SERVO PWM pins
#define SERVO1 A2
#define SERVO2 A3
#define SERVO3 D3
#define SERVO1 D4

// LEDC PWM
const int pwmServoFreq = 50;
const int pwmServoResolution = 14;
const int pwmServo1_Ch = 0;
const int pwmServo2_Ch = 1;
const int pwmServo3_Ch = 2;
const int pwmServo4_Ch = 3;

// Servo pulse range
const int servoMin = 1000;          // 1 ms
const int servoMax = 2000;          // 2 ms

uint32_t angleToDuty(int angle) {
  int pulse = map(angle, 0, 180, servoMin, servoMax);
  // return (uint32_t)((pulse * 65535UL) / 20000UL);
  int duty = (pulse * ((1 << pwmServoResolution) - 1)) / 20000UL;
  return duty;
}

//

void setup() {
  ledcSetup(pwmServo3_Ch, pwmServoFreq, pwmServoResolution);
  ledcAttachPin(SERVO3, pwmServo3_Ch);


}

void loop() {

  uint32_t duty = angleToDuty(90);
  ledcWrite(pwmServo3_Ch, duty);
  // put your main code here, to run repeatedly:
  // for (int angle = 0; angle <= 180; angle += 5) {
  //   uint32_t duty = angleToDuty(angle);
  //   ledcWrite(pwmServo3_Ch, duty);
  //   delay(20);
  // }

}
