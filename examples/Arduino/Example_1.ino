// ---------- LED Pins ----------
#define LED1 D6
#define LED2 D7

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void loop() {
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(500);  // on for 500 ms

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(500);  // off for 500 ms
}
