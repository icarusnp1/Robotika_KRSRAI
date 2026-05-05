// ============================================================
//  Quadruped Robot - Tes 8 Servo Continuous 360°
//  Hardware : ESP32 + PCA9685 + 8x Servo Continuous 360°
//  Library  : Adafruit PWM Servo Driver Library
//
//  Wiring PCA9685 → ESP32:
//    VCC  → 3.3V
//    GND  → GND (common ground)
//    SDA  → GPIO 21
//    SCL  → GPIO 22
//    V+   → 6V dari stepdown HW-131 (power servo)
//
//  Channel servo:
//    CH0  = Kaki Depan Kiri  - Hip
//    CH1  = Kaki Depan Kiri  - Knee
//    CH2  = Kaki Depan Kanan - Hip
//    CH3  = Kaki Depan Kanan - Knee
//    CH4  = Kaki Belakang Kiri  - Hip
//    CH5  = Kaki Belakang Kiri  - Knee
//    CH6  = Kaki Belakang Kanan - Hip
//    CH7  = Kaki Belakang Kanan - Knee
//
//  Perintah Serial Monitor (9600 baud):
//    f  → semua servo putar MAJU   (700µs)
//    b  → semua servo putar BALIK  (1200µs)
//    s  → semua servo STOP         (1500µs)
//    0-7 → pilih servo tertentu saja
//    F  → servo terpilih MAJU
//    B  → servo terpilih BALIK
//    S  → servo terpilih STOP
//    r  → reset pilihan (semua servo aktif)
//    ?  → tampilkan menu bantuan
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- Inisialisasi PCA9685 dengan alamat default 0x40 ---
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// --- Frekuensi PWM untuk servo (standar 50Hz) ---
#define SERVO_FREQ     50

// --- Konversi microseconds ke tick PCA9685 ---
// Rumus: tick = (us / 20000.0) * 4096
// 20000µs = periode 1 siklus pada 50Hz
#define US_TO_TICK(us) ((uint16_t)((us / 20000.0) * 4096))

// --- Nilai pulse microseconds ---
// Sesuaikan nilai ini jika servo kamu berbeda karakteristiknya
#define PULSE_STOP     1500   // 1500µs = berhenti (netral)
#define PULSE_MAJU     700    // 700µs  = putar arah A (pelan)
#define PULSE_BALIK    1200   // 1200µs = putar arah B (berlawanan, pelan)

// Konversi ke tick
const uint16_t TICK_STOP  = US_TO_TICK(PULSE_STOP);
const uint16_t TICK_MAJU  = US_TO_TICK(PULSE_MAJU);
const uint16_t TICK_BALIK = US_TO_TICK(PULSE_BALIK);

// --- Jumlah servo ---
#define SERVO_COUNT 8

// --- State servo ---
uint16_t servoState[SERVO_COUNT];  // tick saat ini tiap servo
int8_t   selectedServo = -1;       // -1 = semua servo

// --- Nama servo untuk debugging ---
const char* servoName[SERVO_COUNT] = {
  "CH0 Depan Kiri Hip",
  "CH1 Depan Kiri Knee",
  "CH2 Depan Kanan Hip",
  "CH3 Depan Kanan Knee",
  "CH4 Belakang Kiri Hip",
  "CH5 Belakang Kiri Knee",
  "CH6 Belakang Kanan Hip",
  "CH7 Belakang Kanan Knee"
};

// ============================================================
//  FUNGSI UTAMA
// ============================================================

// Set satu servo berdasarkan nilai tick
void setServo(uint8_t ch, uint16_t tick) {
  pca.setPWM(ch, 0, tick);
  servoState[ch] = tick;
}

// Set semua servo sekaligus
void setAllServo(uint16_t tick) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServo(i, tick);
  }
}

// Stop semua servo (aman)
void stopAll() {
  setAllServo(TICK_STOP);
  Serial.println("[OK] Semua servo STOP");
}

// Tampilkan status semua servo
void printStatus() {
  Serial.println("\n--- Status Servo ---");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    String kondisi = "STOP";
    if (servoState[i] == TICK_MAJU)  kondisi = "MAJU";
    if (servoState[i] == TICK_BALIK) kondisi = "BALIK";
    Serial.printf("  %s → %s (tick: %d)\n", servoName[i], kondisi.c_str(), servoState[i]);
  }
  if (selectedServo == -1) {
    Serial.println("Mode: SEMUA servo");
  } else {
    Serial.printf("Mode: servo terpilih = CH%d\n", selectedServo);
  }
  Serial.println("--------------------\n");
}

// Tampilkan menu
void printMenu() {
  Serial.println("\n====== MENU KONTROL SERVO ======");
  Serial.println("  f   → Semua servo MAJU  (700µs)");
  Serial.println("  b   → Semua servo BALIK (1200µs)");
  Serial.println("  s   → Semua servo STOP  (1500µs)");
  Serial.println("  0-7 → Pilih channel servo tertentu");
  Serial.println("  F   → Servo terpilih MAJU");
  Serial.println("  B   → Servo terpilih BALIK");
  Serial.println("  S   → Servo terpilih STOP");
  Serial.println("  r   → Reset pilihan (semua servo)");
  Serial.println("  p   → Tampilkan status semua servo");
  Serial.println("  ?   → Tampilkan menu ini");
  Serial.println("================================\n");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n===========================");
  Serial.println(" Quadruped Servo Test v1.0");
  Serial.println("===========================");

  // Inisialisasi I2C dengan pin ESP32
  Wire.begin(21, 22);  // SDA=21, SCL=22

  // Inisialisasi PCA9685
  pca.begin();
  pca.setOscillatorFrequency(27000000);  // Kalibrasi osilator internal
  pca.setPWMFreq(SERVO_FREQ);

  delay(100);

  // Stop semua servo saat startup (aman)
  stopAll();
  Serial.println("[INFO] PCA9685 siap. Semua servo dalam posisi STOP.");
  Serial.println("[INFO] Pastikan power servo (V+) sudah tersambung ke stepdown.");

  printMenu();
}

// ============================================================
//  LOOP - Baca perintah dari Serial Monitor
// ============================================================
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Buang karakter newline/carriage return
    if (cmd == '\n' || cmd == '\r') return;

    Serial.printf("[CMD] '%c' diterima\n", cmd);

    switch (cmd) {

      // --- Kontrol semua servo ---
      case 'f':
        setAllServo(TICK_MAJU);
        Serial.printf("[OK] Semua servo MAJU (tick: %d = ~700µs)\n", TICK_MAJU);
        break;

      case 'b':
        setAllServo(TICK_BALIK);
        Serial.printf("[OK] Semua servo BALIK (tick: %d = ~1200µs)\n", TICK_BALIK);
        break;

      case 's':
        stopAll();
        break;

      // --- Pilih servo tertentu (0-7) ---
      case '0': case '1': case '2': case '3':
      case '4': case '5': case '6': case '7':
        selectedServo = cmd - '0';
        Serial.printf("[SEL] Servo terpilih: CH%d - %s\n",
                      selectedServo, servoName[selectedServo]);
        Serial.println("      Gunakan F/B/S untuk kontrol servo ini");
        break;

      // --- Kontrol servo terpilih ---
      case 'F':
        if (selectedServo == -1) {
          Serial.println("[!] Pilih channel dulu (tekan 0-7)");
        } else {
          setServo(selectedServo, TICK_MAJU);
          Serial.printf("[OK] CH%d MAJU\n", selectedServo);
        }
        break;

      case 'B':
        if (selectedServo == -1) {
          Serial.println("[!] Pilih channel dulu (tekan 0-7)");
        } else {
          setServo(selectedServo, TICK_BALIK);
          Serial.printf("[OK] CH%d BALIK\n", selectedServo);
        }
        break;

      case 'S':
        if (selectedServo == -1) {
          Serial.println("[!] Pilih channel dulu (tekan 0-7)");
        } else {
          setServo(selectedServo, TICK_STOP);
          Serial.printf("[OK] CH%d STOP\n", selectedServo);
        }
        break;

      // --- Reset pilihan ---
      case 'r':
        selectedServo = -1;
        Serial.println("[OK] Mode reset → semua servo aktif");
        break;

      // --- Status & menu ---
      case 'p':
        printStatus();
        break;

      case '?':
        printMenu();
        break;

      default:
        Serial.printf("[?] Perintah '%c' tidak dikenal. Ketik '?' untuk menu.\n", cmd);
        break;
    }
  }
}