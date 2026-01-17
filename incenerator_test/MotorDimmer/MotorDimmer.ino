#include <RBDdimmer.h>

#define USE_SERIAL Serial

#define DIM1_PIN 43
#define DIM2_PIN 44
#define DIM3_PIN 45
#define DIM4_PIN 46

// RobotDyn/RBDdimmer untuk MEGA: cukup pakai output pin (ZC fixed D2)
dimmerLamp dim1(DIM1_PIN);
dimmerLamp dim2(DIM2_PIN);
dimmerLamp dim3(DIM3_PIN);
dimmerLamp dim4(DIM4_PIN);

int v1 = 0, v2 = 0, v3 = 0, v4 = 0;

void setup() {
  USE_SERIAL.begin(115200);

  dim1.begin(NORMAL_MODE, ON);
  dim2.begin(NORMAL_MODE, ON);
  dim3.begin(NORMAL_MODE, ON);
  dim4.begin(NORMAL_MODE, ON);

  USE_SERIAL.println("4CH Dimmer Ready");
  USE_SERIAL.println("Input:");
  USE_SERIAL.println("  - 1..100   (set ALL channels)");
  USE_SERIAL.println("  - C1=xx / C2=xx / C3=xx / C4=xx");
}

static void applyAll() {
  dim1.setPower(v1);
  dim2.setPower(v2);
  dim3.setPower(v3);
  dim4.setPower(v4);
}

static void printStatus() {
  USE_SERIAL.print("C1=");
  USE_SERIAL.print(dim1.getPower());
  USE_SERIAL.print("%  C2=");
  USE_SERIAL.print(dim2.getPower());
  USE_SERIAL.print("%  C3=");
  USE_SERIAL.print(dim3.getPower());
  USE_SERIAL.print("%  C4=");
  USE_SERIAL.print(dim4.getPower());
  USE_SERIAL.println("%");
}

void loop() {
  // Mode 1: per-channel command via line, ex: C3=75
  if (USE_SERIAL.available()) {
    // peek apakah format "C"
    char c = USE_SERIAL.peek();

    if (c == 'C') {
      String in = USE_SERIAL.readStringUntil('\n');
      in.trim();

      int ch = 0, val = 0;
      // Parsing sederhana: Cn=val
      if (in.length() >= 4 && in.charAt(0) == 'C') {
        ch = in.substring(1, 2).toInt();
        int eq = in.indexOf('=');
        if (eq > 0) val = in.substring(eq + 1).toInt();
      }

      val = constrain(val, 0, 100);

      bool changed = false;
      if (ch == 1) { v1 = val; changed = true; }
      else if (ch == 2) { v2 = val; changed = true; }
      else if (ch == 3) { v3 = val; changed = true; }
      else if (ch == 4) { v4 = val; changed = true; }

      if (changed) {
        applyAll();
        printStatus();
      }
    }
    else {
      // Mode 2: angka saja -> set semua channel
      int buf = USE_SERIAL.parseInt();
      if (buf != 0) {
        buf = constrain(buf, 0, 100);
        v1 = v2 = v3 = v4 = buf;
        applyAll();
        printStatus();
      }
      delay(200); // biar parseInt tidak keburu
    }
  }

  delay(50);
}
