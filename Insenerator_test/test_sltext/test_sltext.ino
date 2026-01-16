#include <Arduino.h>
#include <Nextion.h>

// =================== NEXTION SERIAL ===================
#define NEXTION_RX 16
#define NEXTION_TX 17
#define nexSerial Serial2
static const uint32_t NEXTION_BAUD = 115200;

// =================== NEXTION OBJECTS ===================
// SESUAIKAN INI DENGAN HMI KAMU:
// page3 id = 3
// slt0 component id = 5
// slt1 component id = 6
// name harus persis: "slt0" & "slt1"
NexScrolltext stLog   = NexScrolltext(3, 5, "slt0");
NexScrolltext stBatch = NexScrolltext(3, 6, "slt1");

// =================== MCU BUFFERS ===================
String logBuf0;
String logBuf1;

static const size_t LOG_MAX_CHARS   = 900;  // batas aman (sesuaikan)
static const size_t BATCH_MAX_CHARS = 900;

bool logPageActive = false;  // true kalau page3 aktif
bool logDirty = false;
bool batchDirty = false;

// ============ Helpers ============
static inline void trimFrontToLimit(String &s, size_t maxChars) {
  if (s.length() <= maxChars) return;

  // buang dari depan sampai ketemu '\r' agar potongnya rapi per baris
  int cut = s.length() - (int)maxChars;
  int pos = s.indexOf("\\r", cut);
  if (pos < 0) pos = cut;
  s.remove(0, pos + 1);
}

static inline void appendLine(String &buf, const String &line, size_t maxChars) {
  buf += line;
  buf += "\\r"; // newline Nextion yang paling aman
  trimFrontToLimit(buf, maxChars);
}

// Append log/batch ke buffer MCU (tanpa kirim ke Nextion dulu)
void uiAppendLogBuffered(const String &s) {
  appendLine(logBuf0, s, LOG_MAX_CHARS);
  logDirty = true;
}

void uiAppendBatchBuffered(const String &s) {
  appendLine(logBuf1, s, BATCH_MAX_CHARS);
  batchDirty = true;
}

// Flush ke Nextion hanya jika page3 aktif
void uiFlushIfActive() {
  if (!logPageActive) return;

  if (logDirty) {
    bool ok = stLog.setText(logBuf0.c_str());
    Serial.print("[FLUSH] slt0 setText: ");
    Serial.println(ok ? "OK" : "FAIL");
    logDirty = false;
  }
  if (batchDirty) {
    bool ok = stBatch.setText(logBuf1.c_str());
    Serial.print("[FLUSH] slt1 setText: ");
    Serial.println(ok ? "OK" : "FAIL");
    batchDirty = false;
  }
}

// =================== Read "prints" messages from Nextion ===================
// Nextion prints "P3_ON",0 akan mengirim string lalu terminator 0xFF 0xFF 0xFF.
// Kita cukup kumpulkan sampai ketemu 0xFF pertama.
String nexIn;

void pollNextionMessages() {
  while (nexSerial.available()) {
    uint8_t b = (uint8_t)nexSerial.read();

    if (b == 0xFF) {
      // end of message (first 0xFF)
      if (nexIn.length() > 0) {
        nexIn.trim();

        Serial.print("[NEX RX] ");
        Serial.println(nexIn);

        if (nexIn == "P3_ON") {
          logPageActive = true;
          // begitu page aktif, paksa flush semua
          logDirty = true;
          batchDirty = true;
          uiFlushIfActive();
        } else if (nexIn == "P3_OFF") {
          logPageActive = false;
        }

        nexIn = "";
      }

      // buang 0xFF berikutnya (terminator total 3 byte)
      while (nexSerial.peek() == 0xFF) {
        nexSerial.read();
      }
    } else {
      // simpan karakter printable saja (optional)
      if (b >= 32 && b <= 126) {
        nexIn += (char)b;
        if (nexIn.length() > 32) { // anti buffer liar
          nexIn = "";
        }
      }
    }
  }
}

// current page id yang terakhir dilaporkan Nextion
int currentPageId = -1;

void pollNextionSendme() {
  while (nexSerial.available()) {
    uint8_t b = (uint8_t)nexSerial.read();

    // sendme response: 0x66, <pageid>, 0xFF,0xFF,0xFF
    if (b == 0x66) {
      // tunggu sampai ada pageid
      while (!nexSerial.available()) { /* wait */ }
      uint8_t pid = (uint8_t)nexSerial.read();

      // buang terminator 0xFF 0xFF 0xFF (kalau ada)
      for (int i = 0; i < 3; i++) {
        while (!nexSerial.available()) { /* wait */ }
        (void)nexSerial.read();
      }

      currentPageId = pid;
      Serial.print("[SENDME] page = ");
      Serial.println(currentPageId);

      // page3 aktif?
      if (currentPageId == 3) {
        logPageActive = true;
        logDirty = true;
        batchDirty = true;
        uiFlushIfActive();   // langsung flush begitu masuk page3
      } else {
        logPageActive = false;
      }
    }
  }
}


// =================== DEMO GENERATOR ===================
unsigned long lastDemoMs = 0;
int counterLog = 0;
int counterBatch = 0;

void generateDemoLogs() {
  if (millis() - lastDemoMs < 700) return;
  lastDemoMs = millis();

  counterLog++;
  counterBatch++;

  uiAppendLogBuffered(String("LOG ") + counterLog + "  Temp=" + String(random(20, 120)) + "C");
  uiAppendBatchBuffered(String("BATCH ") + counterBatch + "  W=" + String(random(1, 30)) + "kg");

  // kalau page3 aktif, flush jalan di loop via uiFlushIfActive()
}

// =================== SETUP & LOOP ===================
void setup() {
  Serial.begin(115200);
  delay(300);

  nexSerial.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  delay(300);

  nexInit();
  Serial.println("Nextion init done");

  // start buffers bersih
  logBuf0 = "";
  logBuf1 = "";
  logDirty = true;
  batchDirty = true;

  // NOTE:
  // Kita TIDAK memaksa pindah page.
  // Jadi setText() hanya akan dilakukan setelah menerima "P3_ON".
  Serial.println("Waiting for page3 activation (P3_ON) from Nextion...");
  Serial.println("Open page3 once to activate, after that it will update automatically when you revisit page3.");
}

void loop() {
  pollNextionMessages();

  // bikin demo log agar kelihatan buffer ngisi walau page3 belum dibuka
  generateDemoLogs();

  pollNextionSendme();
  uiFlushIfActive();
}
