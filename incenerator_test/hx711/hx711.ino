#include <Arduino.h>
#include "HX711.h"

// Pin Definisi (Sesuaikan dengan Arduino Mega Anda)
const int LOADCELL_DOUT_PIN = 41;
const int LOADCELL_SCK_PIN  = 40;

HX711 scale;

// --- GANTI ANGKA INI DENGAN HASIL HITUNGAN ANDA ---
float calibration_factor =2123.157; // <--- Masukkan nilai kalibrasi di sini
// --------------------------------------------------

void setup() {
  Serial.begin(9600);
  Serial.println("Memulai Timbangan...");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Mengatur faktor kalibrasi
  scale.set_scale(calibration_factor);

  // Reset timbangan ke 0 (pastikan tidak ada beban saat menyala)
  Serial.println("Tare... (kosongkan timbangan)");
  scale.tare(); 
  
  Serial.println("Timbangan siap. Silakan letakkan beban.");
}

void loop() {
  // get_units(5) artinya mengambil rata-rata dari 5 pembacaan agar stabil
  float berat = scale.get_units(5);

  Serial.print("Berat: ");
  Serial.print(berat, 2); // Menampilkan 2 angka di belakang koma
  Serial.println(" "); // Satuan mengikuti satuan berat saat kalibrasi (gram/kg)

  // Opsional: Power down jika ingin hemat daya (biasanya tidak perlu untuk project listrik PLN)
  // scale.power_down();
  // delay(500);
  // scale.power_up();
  
  delay(500); // Delay agar serial monitor tidak terlalu cepat
}