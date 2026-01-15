#include <Arduino.h>
#include "HX711.h"


static const uint8_t NODE_ID = 3;
static const int PIN_RS485_DE = 4;
static const int RS485_RX = 16;
static const int RS485_TX = 17;


RS485Simple bus;
String rxLine;


// HX711 pins (ubah sesuai wiring)
const int HX711_DT = 32;
const int HX711_SCK = 33;


HX711 scale;
float scale_factor = 2123.157f;


// simple smoothing
static float wEma = 0;
static bool wInit = false;
static const float ALPHA = 0.18f;


void sendToMaster(const String &payload){
bus.sendLine(String("N") + String(NODE_ID<10?"0":"") + String(NODE_ID) + ":" + payload + "\n");
}


float readWeightKg(){
if (!scale.is_ready()) return NAN;
float w = scale.get_units(1);
if (!isfinite(w)) return NAN;


if (!wInit) { wInit=true; wEma=w; }
wEma = (ALPHA*w) + ((1.0f-ALPHA)*wEma);
if (wEma < 0 && wEma > -0.2f) wEma = 0;
return wEma;
}


void handlePayload(const String &payload){
if (payload == "REQ_TEMP") {
float w = readWeightKg();
if (isfinite(w)) sendToMaster(String("WEIGHT:") + String(w,2));
else sendToMaster("WEIGHT:NaN");
}
if (payload == "TARE") {
scale.tare(20);
sendToMaster("LOG:INFO,HX711,TARED");
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


scale.begin(HX711_DT, HX711_SCK);
scale.set_scale(scale_factor);
scale.tare(20);


sendToMaster("LOG:INFO,BOOT,SLAVE3_READY");
}


void loop(){
processRx();
}