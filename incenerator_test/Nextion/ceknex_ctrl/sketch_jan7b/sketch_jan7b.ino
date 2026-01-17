void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // RX1=19 TX1=18
  Serial.println("Mega ready");
}

void loop() {
  if (Serial1.available()) {
    String s = Serial1.readStringUntil('\n');
    s.trim();
    Serial.print("Dari ESP32: ");
    Serial.println(s);

    if (s == "STOP") {
      // aksi stop kamu di sini
      Serial.println("STOP diterima!");
    }
  }
}
