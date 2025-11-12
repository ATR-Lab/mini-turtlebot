#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- I2C ----------
#define SDA_PIN A4
#define SCL_PIN A5

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- MOTOR DRIVER ----------
#define MTA1 A6
#define MTA2 A7
#define MTB1 D8
#define MTB2 D9

// ---------- ENCODER PINS ----------
#define MEA1 D10
#define MEA2 D11
#define MEB1 D12
#define MEB2 D13

// ---------- PWM CONFIG ----------
const int pwmFreq = 500;
const int pwmResolution = 10;
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

// ---------- Encoder & Motion ----------
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

// ---------- Encoder Settings ----------
const int ticksPerRev = 12 * 4;  // 12 ticks per channel, 4x decoding
const float gearRatio = 30.0;     // Update if gear reduction is used

#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

// ---------- ISR ----------
void IRAM_ATTR handleEncoderA() {
  uint8_t state = READ_ENC_A();
  uint8_t combo = (prevStateA << 2) | state;
  switch (combo) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCountA++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCountA--; break;
  }
  prevStateA = state;
}

void IRAM_ATTR handleEncoderB() {
  uint8_t state = READ_ENC_B();
  uint8_t combo = (prevStateB << 2) | state;
  switch (combo) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCountB++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCountB--; break;
  }
  prevStateB = state;
}

// ---------- Motor PWM ----------
int motorPowerA = 500;  // Range -1023 to 1023; Initial Min +- 165, Dynamic Min +-90
int motorPowerB = 500;  // Range -1023 to 1023; Initial Min +- 165, Dynamic Min +-90

void setMotorA(int pwm) {
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    ledcWrite(pwmA1_Ch, pwm);
    ledcWrite(pwmA2_Ch, 0);
  } else {
    ledcWrite(pwmA1_Ch, 0);
    ledcWrite(pwmA2_Ch, -pwm);
  }
}

void setMotorB(int pwm) {
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    ledcWrite(pwmB1_Ch, 0);
    ledcWrite(pwmB2_Ch, pwm);
  } else {
    ledcWrite(pwmB1_Ch, -pwm);
    ledcWrite(pwmB2_Ch, 0);
  }
}

void setupEncoderInterrupt(){
  pinMode(MEA1, INPUT); pinMode(MEA2, INPUT);
  pinMode(MEB1, INPUT); pinMode(MEB2, INPUT);
  prevStateA = READ_ENC_A();
  prevStateB = READ_ENC_B();
  attachInterrupt(digitalPinToInterrupt(MEA1), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEA2), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEB1), handleEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEB2), handleEncoderB, CHANGE);
}

void setupMotorPWM(){
  ledcSetup(pwmA1_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmA2_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmB1_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmB2_Ch, pwmFreq, pwmResolution);
  ledcAttachPin(MTA1, pwmA1_Ch);
  ledcAttachPin(MTA2, pwmA2_Ch);
  ledcAttachPin(MTB1, pwmB1_Ch);
  ledcAttachPin(MTB2, pwmB2_Ch);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  Wire.begin(SDA_PIN, SCL_PIN);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  setupEncoderInterrupt();
  setupMotorPWM();

  // Run Motors
  setMotorA(motorPowerA); // 'motorPowerA' range is -1023 to 1023
  setMotorB(motorPowerB); // 'motorPowerB' range is -1023 to 1023
}

void loop() {
  static long lastCountA = 0, lastCountB = 0;
  static unsigned long lastTime = 0;

  unsigned long now = millis();
  if (now - lastTime >= 100) {
    long deltaA = encoderCountA - lastCountA;
    long deltaB = encoderCountB - lastCountB;
    lastCountA = encoderCountA;
    lastCountB = encoderCountB;

    float ticksPerSecA = deltaA * 10.0;
    float ticksPerSecB = deltaB * 10.0;

    float rpmA = (ticksPerSecA * 60.0) / (ticksPerRev * gearRatio);
    float rpmB = (ticksPerSecB * 60.0) / (ticksPerRev * gearRatio);

    // Display on OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setRotation(2);
    display.setTextColor(SSD1306_WHITE);
    display.printf("Motor A: %s\n", rpmA >= 0 ? "REV" : "FWD");
    display.printf("RPM: %.1f\n", fabs(rpmA));
    display.printf("Motor B: %s\n", rpmB >= 0 ? "FWD" : "REV");
    display.printf("RPM: %.1f\n", fabs(rpmB));
    display.display();

    lastTime = now;
  }

  // Serial input: "A 600" or "B -500"
  if (Serial.available()) {
    char motor = Serial.read();
    int value = Serial.parseInt();
    if (motor == 'A') {
      motorPowerA = value;
      setMotorA(motorPowerA);
    } else if (motor == 'B') {
      motorPowerB = value;
      setMotorB(motorPowerB);
    }
    // flush remaining
    while (Serial.available()) Serial.read();
  }
}
