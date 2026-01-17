// Contoh: ESP32 + Nextion dual state button b1 & textbox t1
// TX2 (ESP32) -> RX Nextion
// RX2 (ESP32) -> TX Nextion

#define nexSerial Serial2      // dipakai di library Nextion
#include <Nextion.h>

/*
   Sesuaikan pin UART untuk Nextion:
   Contoh:
   - ESP32 GPIO 16 = RX2 (terhubung ke TX Nextion)
   - ESP32 GPIO 17 = TX2 (terhubung ke RX Nextion)
*/

// ----- Deklarasi objek Nextion -----
// Format: NexDSButton(page_id, component_id, "objname");
NexDSButton b1 = NexDSButton(0, 9, "btAuto");   // page0, id1, nama b1
NexText    t1 = NexText   (0, 2, "tStat");     // page0, id2, nama t1

// Daftar komponen yang akan "didengarkan" event-nya
NexTouch *nex_listen_list[] = {
  &b1,
  NULL
};

// ----- Callback untuk tombol b1 (saat dilepas / pop) -----
void b1PopCallback(void *ptr) {
  uint32_t val = 0;
  b1.getValue(&val);   // baca status dual state (0 atau 1)

  if (val == 0) {
    t1.setText("A");
  } else {
    t1.setText("B");
  }
}
void printNextionLog() {
  char buffer[100];          // ukuran buffer maksimal
  memset(buffer, 0, sizeof(buffer));

  if (t1.getText(buffer, sizeof(buffer))) {
    Serial.print("Current Nextion log text: ");
    Serial.println(buffer);
  } else {
    Serial.println("Failed to read text from Nextion object 'log'");
  }
}
void setup() {
  Serial.begin(9600);                // debug ke Serial Monitor
  t1.setText("HELLO");
  // Inisialisasi UART ke Nextion: baud 9600, RX=16, TX=17
  Serial.println("nexbegin");
  nexSerial.begin(9600, SERIAL_8N1, 16, 17);

  // Inisialisasi Nextion
  Serial.println("nexinit");
  nexInit();

  // Daftarkan callback untuk b1 (event "pop" = saat dilepas)
  b1.attachPop(b1PopCallback);

  // Set tampilan awal
  t1.setText("A");  // mulai dari huruf A
  b1.setValue(0);   // pastikan state tombol = 0
  Serial.println("reboot");
}

unsigned long lastPrint = 0;

void loop() {
  // Serial.println("loop");
  nexLoop(nex_listen_list);
  // Serial.println("loop2");

  if (millis() - lastPrint >= 2000) {
    lastPrint = millis();
    printNextionLog();   // <<<<< membaca text dari Nextion
  }
}
