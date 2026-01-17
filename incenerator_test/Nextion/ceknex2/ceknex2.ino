#include <Nextion.h>

// ================== NEXTION UART ==================
#define NEXTION_RX 25
#define NEXTION_TX 26

// ================== MEGA UART ==================
#define MEGA_RX 16   // ESP32 RX ← Mega TX2
#define MEGA_TX 17   // ESP32 TX → Mega RX2

// ================== NEXTION OBJECT ==================
NexButton bStop = NexButton(0, 13, "bStop");
NexText   tStat = NexText(0, 2, "tStat");

NexTouch *nex_listen_list[] = {
  &bStop,
  NULL
};

// ================== CALLBACK ==================
void bStopPopCallback(void *ptr) {
  Serial.println("bStop ditekan");

  // Update text di Nextion
  tStat.setText("STOP");

  // Kirim perintah ke Arduino Mega
  Serial1.println("STOP");
}

// ================== SETUP ==================
void setup() {
  // Serial.begin(115200);
  Serial.begin(9600, SERIAL_8N1,16,17);  // Debug serial & komunikasi ke CTRL
  nexSerial.begin(9600, SERIAL_8N1, 25, 26);
  nexInit();

  bStop.attachPop(bStopPopCallback, &bStop);

  tStat.setText("READY");

  Serial.println("ESP32 siap: Nextion + Kirim ke Mega");
}

// ================== LOOP ==================
void loop() {
  nexLoop(nex_listen_list);
  // Serial.println("a");
}
