#define RELAY_BLOWER   32
#define RELAY_BURNER   34
#define RELAY_IGNITION 34   // iya, sama. hidup bersama, mati bersama.

void setup() {
  Serial.begin(9600);

  pinMode(RELAY_BLOWER, OUTPUT);
  pinMode(RELAY_BURNER, OUTPUT);
  pinMode(RELAY_IGNITION, OUTPUT);

  // asumsi relay ACTIVE LOW
  digitalWrite(RELAY_BLOWER, HIGH);
  digitalWrite(RELAY_BURNER, HIGH);
  digitalWrite(RELAY_IGNITION, HIGH);

  Serial.println(F("=== RELAY TEST MODE ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("BLOWER ON / OFF"));
  Serial.println(F("BURNER ON / OFF"));
  Serial.println(F("IGN ON / OFF"));
  Serial.println(F("ALL OFF"));
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "BLOWER ON") {
      digitalWrite(RELAY_BLOWER, LOW);
      Serial.println(F("Blower ON"));
    }
    else if (cmd == "BLOWER OFF") {
      digitalWrite(RELAY_BLOWER, HIGH);
      Serial.println(F("Blower OFF"));
    }
    else if (cmd == "BURNER ON") {
      digitalWrite(RELAY_BURNER, LOW);
      Serial.println(F("Burner ON"));
    }
    else if (cmd == "BURNER OFF") {
      digitalWrite(RELAY_BURNER, HIGH);
      Serial.println(F("Burner OFF"));
    }
    else if (cmd == "IGN ON") {
      digitalWrite(RELAY_IGNITION, LOW);
      Serial.println(F("Ignition ON"));
    }
    else if (cmd == "IGN OFF") {
      digitalWrite(RELAY_IGNITION, HIGH);
      Serial.println(F("Ignition OFF"));
    }
    else if (cmd == "ALL OFF") {
      digitalWrite(RELAY_BLOWER, HIGH);
      digitalWrite(RELAY_BURNER, HIGH);
      digitalWrite(RELAY_IGNITION, HIGH);
      Serial.println(F("All relays OFF"));
    }
    else {
      Serial.print(F("Unknown command: "));
      Serial.println(cmd);
    }
  }
}
