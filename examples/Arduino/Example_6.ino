#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MPU6050.h"  // Electronic Cats library

// ---------- I2C ----------
#define SDA_PIN A4
#define SCL_PIN A5

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- IMU ----------
MPU6050 imu;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
    while (true);
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setRotation(2);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Initializing MPU...");
  display.display();

  // MPU6050 init
  imu.initialize();
  if (!imu.testConnection()) {
    display.println("MPU6050 not connected!");
    display.display();
    while (1);
  }

  display.println("MPU OK");
  display.display();
  delay(1000);
}

void loop() {
  int16_t ax, ay, az;
  imu.getAcceleration(&ax, &ay, &az);

  // Convert raw to g (sensitivity = 16384 LSB/g for ±2g range)
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Accel (g):");
  display.print("X: "); display.println(ax_g, 2);
  display.print("Y: "); display.println(ay_g, 2);
  display.print("Z: "); display.println(az_g, 2);
  display.display();

  delay(200);
}
