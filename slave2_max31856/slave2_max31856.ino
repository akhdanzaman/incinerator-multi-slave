#include <Arduino.h>
#include <SPI.h>
#include <MAX31856.h>


static const uint8_t NODE_ID = 2;
static const int PIN_RS485_DE = 4;
static const int RS485_RX = 16;
static const int RS485_TX = 17;


RS485Simple bus;
String rxLine;


#define SCK 18
#define CS 5
#define SDI 23
#define SDO 19


#define CR0_INIT (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K + CR0_NOISE_FILTER_50HZ)
#define CR1_INIT (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_S)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))


MAX31856 *temperature;


void sendToMaster(const String &payload){
bus.sendLine(String("N") + String(NODE_ID<10?"0":"") + String(NODE_ID) + ":" + payload);
}


float readTempC(){
float t = temperature->readThermocouple(CELSIUS);
if (!isfinite(t) || t>5000.0f) return NAN;
return t;
}


void handlePayload(const String &payload){
if (payload == "GET") {
float t = readTempC();
if (isfinite(t)) sendToMaster(String("TEMP:") + String(t,1));
else sendToMaster("TEMP:NaN");
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


SPI.begin();
temperature = new MAX31856(SDI, SDO, CS, SCK);
temperature->writeRegister(REGISTER_CR0, CR0_INIT);
temperature->writeRegister(REGISTER_CR1, CR1_INIT);
temperature->writeRegister(REGISTER_MASK, MASK_INIT);
delay(200);


sendToMaster("LOG:INFO,BOOT,SLAVE2_READY");
}


void loop(){
processRx();
}