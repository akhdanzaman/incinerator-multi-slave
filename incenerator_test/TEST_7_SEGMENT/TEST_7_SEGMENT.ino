#define DATA_PIN   23
#define CLOCK_PIN  25
#define LATCH_PIN  24

unsigned long lastRandomUpdate = 0;
float massValue = 0;
byte outX1, outX2, outY1, outY2;   // hasil konversi siap ditampilkan

//Dibuat 3 fungsi Array untuk mapping tiap-tiap digit karena setiap digit pada seven segment punya maping yang berbeda-beda
byte segDigit14[10] = { //maping untuk digit 1 dan 4
  0b00101000, // 0
  0b01111110, // 1
  0b00110001, // 2  
  0b00110100, // 3
  0b01100110, // 4
  0b10100100, // 5
  0b10100000, // 6
  0b00111110, // 7
  0b00100000, // 8
  0b00100100  // 9
};

byte segDigit2[10] = { //maping untuk digit 2 (mirip seperti digit 1 dan 4, hanya saja segment dot-nya selalu menyala sebagai tanda koma)
  0b00001000, // 0
  0b01011110, // 1
  0b00010001, // 2  
  0b00010100, // 3
  0b01000110, // 4
  0b10000100, // 5
  0b10000000, // 6
  0b00011110, // 7
  0b00000000, // 8
  0b00000100  // 9
};

byte segDigit3[10] = { //maping untuk digit 3
  0b00101000, // 0
  0b11101011, // 1
  0b00110001, // 2  
  0b10100001, // 3
  0b11100010, // 4
  0b10100100, // 5
  0b00100100, // 6
  0b11101001, // 7
  0b00100000, // 8
  0b10100000  // 9
};

// --- FUNGSI KIRIM KE 74HC595 ---
void sendByte(byte b) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b);
  digitalWrite(LATCH_PIN, HIGH);
}
/* Berikut dibuat dua fungsi berbeda untuk menampilkan dua nilai sensor yang berbeda
Sensor Load cell format XX.xx
Sensor Suhu format XXXX
*/
void convertTemperatureValueto7Segment(float tempValue) {

  // --- ERROR CHECK ---
  // Jika suhu melebihi batas aman → tampilkan pesan error
  if (tempValue > 1500.0f) {
    pesanError();
    return;   // Stop agar tidak lanjut ke proses konversi normal
  }

  // Safety untuk nilai negatif
  if (tempValue < 0) tempValue = 0;

  // Ambil nilai integer saja untuk suhu
  int tempIntVal = (int)tempValue;

  // Pecah digit ribuan → satuan
  int tX1 = (tempIntVal / 1000) % 10;  // ribuan
  int tX2 = (tempIntVal / 100)  % 10;  // ratusan
  int tX3 = (tempIntVal / 10)   % 10;  // puluhan
  int tX4 = tempIntVal % 10;           // satuan

  // Nilai 7-segment kosong
  const byte SEG_T_KOSONG = 0b11111111;

  // Output buffer untuk 4 digit suhu
  byte tOut1, tOut2, tOut3, tOut4;

  // Rules: leading zero = kosong
  tOut1 = (tX1 == 0) ? SEG_T_KOSONG : segDigit14[tX1];

  if (tX1 == 0 && tX2 == 0)
    tOut2 = SEG_T_KOSONG;
  else
    tOut2 = segDigit14[tX2];

  if (tX1 == 0 && tX2 == 0 && tX3 == 0)
    tOut3 = SEG_T_KOSONG;
  else
    tOut3 = segDigit3[tX3];

  // Digit satuan selalu tampil
  tOut4 = segDigit14[tX4];

  // Kirim ke driver seven segment
  sendByte(~tOut1);
  sendByte(~tOut2);
  sendByte(~tOut3);
  sendByte(~tOut4);
}


void convertMassValueto7Segment(float massValue) {

  // --- ERROR CHECK ---
  if (massValue > 250.0f) {
    pesanError();
    return;   // stop agar tidak lanjut ke proses normal
  }

  // Pastikan nilai positif
  if (massValue < 0) massValue = 0;

  // Ambil nilai integer dan desimal
  int nilaiInt = (int)massValue;                      // contoh: 5
  int nilaiDec = (int)((massValue - nilaiInt) * 100); // contoh: 43

  // Ekstrak digit integer dan desimal
  int X1 = (nilaiInt / 10) % 10;  // puluhan
  int X2 = nilaiInt % 10;         // satuan
  int Y1 = (nilaiDec / 10) % 10;
  int Y2 =  nilaiDec % 10;

  // ==== Mapping sesuai array Anda ====
  // Jika hanya 1 digit → digit pertama kosong
  if (nilaiInt < 10) {
    outX1 = 0b11111111;   // KOSONG
  } else {
    outX1 = segDigit14[X1];
  }

  outX2 = segDigit2[X2];
  outY1 = segDigit3[Y1];
  outY2 = segDigit14[Y2];

  // ==== Kirim ke seven segment ====
  sendByte(~outX1); // DIGIT 1
  sendByte(~outX2); // DIGIT 2
  sendByte(~outY1); // DIGIT 3
  sendByte(~outY2); // DIGIT 4

  // Debug opsional
//  Serial.print("Digit: ");
//  Serial.print(X1); Serial.print(" ");
//  Serial.print(X2); Serial.print(" ");
//  Serial.print(Y1); Serial.print(" ");
//  Serial.println(Y2);
//
//  Serial.print("Mapped: ");
//  Serial.print(outX1, BIN); Serial.print(" ");
//  Serial.print(outX2, BIN); Serial.print(" ");
//  Serial.print(outY1, BIN); Serial.print(" ");
//  Serial.println(outY2, BIN);
}


 
 void updateRandomMassValueEvery2s() {
   if (millis() - lastRandomUpdate >= 2000) {
     lastRandomUpdate = millis();

     int nilaiIntRandom = random(0, 100);      
     int nilaiDecRandom = random(0, 100);      

     massValue = nilaiIntRandom + (nilaiDecRandom / 100.0);
     convertMassValueto7Segment(massValue);

     Serial.print("Random value: ");
     Serial.println(massValue);
   }
 }


// --- Fungsi baru: generate nilai ribuan (0–9999) tiap 2 detik ---
void updateRandomTemperatureEvery2s() {
  static unsigned long lastTempRandomUpdate = 0;

  if (millis() - lastTempRandomUpdate >= 2000) {
    lastTempRandomUpdate = millis();

    // generate nilai random ribuan
    float randomTempValue = random(0, 10000);   // 0–9999

    // tampilkan ke seven segment
    convertTemperatureValueto7Segment(randomTempValue);

    // debug serial
    Serial.print("Random Temp Value: ");
    Serial.println(randomTempValue);
  }
}

void pesanError(){
  byte tandaMin = 0b11110111; 
  byte hurufE = 0b10100001; 
  byte hurufR1= 0b01111100; 
  byte hurufR2= 0b10101011; 
  
  sendByte(~tandaMin); 
  sendByte(~hurufE); 
  sendByte(~hurufR1); 
  sendByte(~hurufR2); 
}


void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  Serial.begin (9600);
}

void loop() {
//  updateRandomMassValueEvery2s();
//  updateRandomTemperatureEvery2s();
}
