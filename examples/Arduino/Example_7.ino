#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>

// ---------- I2C PINS ----------
#define SDA_PIN A4
#define SCL_PIN A5

// ---------- OLED CONFIG ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- VL53L0X CONFIG ----------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int distanceCM = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED INIT
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }
  display.setTextSize(1);
  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();

  // VL53L0X INIT
  if (!lox.begin()) {
    Serial.println("VL53L0X not found");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("LiDAR ERR");
    display.display();
    while (1);
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LiDAR OK");
  display.display();
  delay(100);
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass true to get debugging info

  if (measure.RangeStatus != 4) {
    distanceCM = measure.RangeMilliMeter / 10;  // mm to cm
  } else {
    distanceCM = 0;  // out of range
  }

  // Show on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(distanceCM);
  display.println(" cm");
  display.display();

  delay(200);
}
