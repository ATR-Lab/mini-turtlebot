// ===== MS18 Servo Control with Joystick =====
// Board: Arduino Nano ESP32 (ESP32-S3)

#define IRD1 D2
#define IRD2 A1   // analog joystick X-axis input

void setup() {
  Serial.begin(115200);

}

void loop() {
  Serial.println(analogRead(IRD1));
}
