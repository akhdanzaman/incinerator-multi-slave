#include <Arduino.h>


static const uint8_t NODE_ID = 4;
static const int PIN_RS485_DE = 4;
static const int RS485_RX = 16;
static const int RS485_TX = 17;


RS485Simple bus;
String rxLine;


// ===== LIDAR UART example (sesuaikan) =====
static const int LIDAR_RX = 12;
static const int LIDAR_TX = 13;
static const uint32_t LIDAR_BAUD = 460800;


HardwareSerial LSerial(2);


// volume state
float batchVol = 0.0f;
float totalVol = 0.0f;


void sendToMaster(const String &payload){
bus.sendLine(String("N") + String(NODE_ID<10?"0":"") + String(NODE_ID) + ":" + payload + "\n");
}


// TODO: isi sesuai algoritma kamu
void lidarInit(){
LSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}


void lidarUpdate(){
// placeholder: di sini kamu proses frame lidar
// update batchVol dan totalVol
}


void handlePayload(const String &payload){
if (payload == "GET") {
sendToMaster(String("LIDAR:") + String(batchVol,3) + "," + String(totalVol,3));
}
if (payload == "RESET_BATCH") {
batchVol = 0.0f;
sendToMaster("LOG:INFO,LIDAR,BATCH_RESET");
}
}


void processRx(){
while (bus.readLine(rxLine)) {
if (!rxLine.startsWith("TO")) { rxLine=""; continue; }
int id = rxLine.substring(2,4).toInt();
if (id != NODE_ID) { rxLine=""; continue; }
int c = rxLine.indexOf(':');
if (c<0) { rxLine=""; continue; }
String payload = rxLine.substring(c+1);
handlePayload(payload);
rxLine="";
}
}


void setup(){
Serial.begin(115200);
bus.begin(Serial1, 9600, RS485_RX, RS485_TX, PIN_RS485_DE);
lidarInit();
sendToMaster("LOG:INFO,BOOT,SLAVE4_READY");
}


void loop(){
lidarUpdate();
processRx();
}