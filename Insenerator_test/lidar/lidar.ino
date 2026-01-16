/*
 * RPLIDAR C1 High Speed Parser (No Library Required)
 * Optimized for ESP32 Hardware Serial
 */

#define RXD2 12
#define TXD2 13

// Struktur data untuk menyimpan hasil
struct LidarPoint {
  float distance;
  float angle;
  byte quality;
  bool startBit;
};

void setup() {
  // Serial Monitor (Laptop)
  Serial.begin(115200);
  
  // Serial Lidar (C1)
  // Increase buffer size to prevent overflow at high speed
  Serial1.setRxBufferSize(4096); 
  Serial1.begin(460800, SERIAL_8N1, RXD2, TXD2);
  
  delay(1000);
  Serial.println("Resetting Lidar...");
  
  // 1. Stop Scan & Motor (Reset state)
  // Command: A5 25
  byte stopCmd[] = {0xA5, 0x25};
  Serial1.write(stopCmd, 2);
  delay(100);

  // 2. Start Scan (Legacy Mode)
  // Command: A5 20
  byte startCmd[] = {0xA5, 0x20};
  Serial1.write(startCmd, 2);
  
  Serial.println("Waiting for data stream...");
}

void loop() {
  // Serial.println("aaa");
  // Kita butuh minimal 5 byte untuk satu paket data pengukuran
  if (Serial1.available() >= 5) {
    
    // Cek header byte pertama tanpa menghapusnya dari buffer (peek)
    byte sync0 = Serial1.peek();
    
    // Validasi Paket Slamtec Standard:
    // Byte 0 harus memiliki pola bit tertentu
    // (sync0 & 0x03) harusnya bernilai 0x01 atau 0x02 tergantung check bit
    // Bit 0 dari Byte 0 adalah kebalikan dari Bit 1 dari Byte 0
    bool syncBit1 = (sync0 >> 1) & 0x01;
    bool syncBit0 = sync0 & 0x01;
    
    if (syncBit1 == !syncBit0) {
       // --- HEADER VALID, BACA 5 BYTE ---
       byte buf[5];
       Serial1.readBytes(buf, 5);
       
       // --- DECODING (Sesuai Datasheet Slamtec) ---
       
       // 1. Quality
       byte quality = buf[0] >> 2;
       
       // 2. Angle (Sudut)
       // Rumus: (Byte1 + Byte2 << 7) / 64.0
       float angle = ((buf[1] >> 1) | (buf[2] << 7)) / 64.0;
       
       // 3. Distance (Jarak)
       // Rumus: (Byte3 + Byte4 << 8) / 4.0
       float distance = (buf[3] | (buf[4] << 8)) / 4.0;
       
       // --- FILTERING & OUTPUT ---
       // Tampilkan hanya jika jarak valid (>0) dan sudut di depan (biar tidak spamming)
       if (distance > 0) {
          // Contoh: Tampilkan data di sudut 0 derajat (Depan) +/- 5 derajat
          if (angle < 5 || angle > 355) {
             Serial.print("Dist: ");
             Serial.print(distance);
             Serial.print(" mm | Angle: ");
             Serial.println(angle);
          }
       }
       
    } else {
       // --- HEADER INVALID (Geser 1 byte) ---
       // Jika byte pertama bukan header yang valid, buang 1 byte sampah
       // dan coba cari header di byte berikutnya.
       Serial1.read(); 
    }
  }
}