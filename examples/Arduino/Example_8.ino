#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- Wi-Fi ----------
const char* ssid     = "Kim-Elecom-2G";
const char* password = "discovery";

// ---------- UDP ----------
WiFiUDP udp;
const int UDP_PORT = 5601;
char incoming[32];

// ---------- I2C ----------
#define SDA A4
#define SCL A5

// ---------- LED ----------
#define LED1 D6
#define LED2 D7

void setup() {
  // OLED init
  Wire.begin(SDA, SCL);  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);  // OLED init failed
  }
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.display();

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  // WiFi Connect
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    display.print(".");
    display.display();
  }

  // Display IP address
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi Connected");
  display.print("IP: ");
  display.println(WiFi.localIP());
  display.display();
  delay(1500);

  // Start UDP
  udp.begin(UDP_PORT);
  display.println("UDP Ready");
  display.display();
  delay(1000);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incoming, sizeof(incoming) - 1);
    if (len > 0) incoming[len] = '\0';

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Received:");
    display.println(incoming);

    // LED control
    if (strcmp(incoming, "L1") == 0) digitalWrite(LED1, HIGH);
    else if (strcmp(incoming, "L0") == 0) digitalWrite(LED1, LOW);
    else if (strcmp(incoming, "R1") == 0) digitalWrite(LED2, HIGH);
    else if (strcmp(incoming, "R0") == 0) digitalWrite(LED2, LOW);

    display.display();
  }
}
