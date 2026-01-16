/*
 * LIMIT SWITCH TEST
 * -----------------
 * - Baca status limit switch
 * - Print ke Serial Monitor
 * - Aman, simpel, dan tidak blocking
 */

const int LIMIT_PIN = 11;   // ganti sesuai pin kamu

void setup() {
  Serial.begin(9600);
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  Serial.println("=== LIMIT SWITCH TEST START ===");
  Serial.println("Tekan dan lepaskan limit switch...");
}

void loop() {
  static bool lastState = HIGH;

  bool currentState = digitalRead(LIMIT_PIN);

  if (currentState != lastState) {
    lastState = currentState;

    if (currentState == LOW) {
      Serial.println("LIMIT SWITCH: TERTEKAN");
    } else {
      Serial.println("LIMIT SWITCH: LEPAS");
    }
  }

  delay(10); // debounce kasar, cukup untuk tes
}
