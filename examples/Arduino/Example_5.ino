#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// I2C Pins for your ESP32-S3
#define SDA_PIN A4
#define SCL_PIN A5

// OLED Display Config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Use -1 if no reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is default I2C address
    Serial.println("SSD1306 allocation failed");
    while (true);  // Stop if display doesn't initialize
  }

  display.clearDisplay();
  display.setTextSize(1);             // Font size
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);            // Top-left corner

  display.println("Hello from ESP32-S3!");
  display.println("OLED is working.");
  display.display();  // Push to screen

  delay(2000);  // Wait 2 seconds

  // Draw a rectangle
  display.clearDisplay();
  display.drawRect(10, 10, 100, 40, SSD1306_WHITE);
  display.setCursor(20, 25);
  display.println("Mini TurtleBot");
  display.display();
}

void loop() {
  // Nothing in loop — static display
}
