#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define SERVO_COUNT 8

#define US_TO_TICK(us) ((uint16_t)(((float)(us) / 20000.0) * 4096))

// ==========================
// DURASI DEFAULT
// Silakan tuning nanti
// ==========================
#define DEFAULT_KNEE_NAIK_MS   300
#define DEFAULT_KNEE_TURUN_MS  300
#define DEFAULT_HIP_MAJU_MS    400
#define DEFAULT_HIP_RESET_MS   400
#define JEDA_MS                150

// ==========================
// STRUKTUR KONFIGURASI KAKI
// ==========================
struct LegConfig {
  const char* nama;

  uint8_t hipCH;
  uint8_t kneeCH;

  uint16_t hipMaju;
  uint16_t hipBalik;
  uint16_t hipStop;

  uint16_t kneeNaik;
  uint16_t kneeTurun;
  uint16_t kneeStop;

  uint16_t kneeNaikMs;
  uint16_t kneeTurunMs;
  uint16_t hipMajuMs;
  uint16_t hipResetMs;
};

// ==========================
// INDEX KAKI
// ==========================
#define KAKI_BELAKANG_KIRI   0
#define KAKI_DEPAN_KIRI      1
#define KAKI_DEPAN_KANAN     2
#define KAKI_BELAKANG_KANAN  3

// ============================================================
// KONFIGURASI PWM 4 KAKI
// Urutan:
// 1. Belakang kiri
// 2. Depan kiri
// 3. Depan kanan
// 4. Belakang kanan
// ============================================================
LegConfig kaki[4] = {
  // 1. KAKI BELAKANG KIRI
  {
    "Belakang Kiri",
    0, 1,

    // Hip CH0
    1600, 1090, 1375,

    // Knee CH1
    900, 1650, 1375,

    DEFAULT_KNEE_NAIK_MS,
    DEFAULT_KNEE_TURUN_MS,
    DEFAULT_HIP_MAJU_MS,
    DEFAULT_HIP_RESET_MS
  },

  // 2. KAKI DEPAN KIRI
  {
    "Depan Kiri",
    2, 3,

    // Hip CH2
    1555, 1050, 1375,

    // Knee CH3
    900, 1650, 1375,

    DEFAULT_KNEE_NAIK_MS,
    DEFAULT_KNEE_TURUN_MS,
    DEFAULT_HIP_MAJU_MS,
    DEFAULT_HIP_RESET_MS
  },

  // 3. KAKI DEPAN KANAN
  {
    "Depan Kanan",
    4, 5,

    // Hip CH4
    1630, 1055, 1375,

    // Knee CH5
    900, 1650, 1375,

    DEFAULT_KNEE_NAIK_MS,
    DEFAULT_KNEE_TURUN_MS,
    DEFAULT_HIP_MAJU_MS,
    DEFAULT_HIP_RESET_MS
  },

  // 4. KAKI BELAKANG KANAN
  // Catatan: data nomor 4 dari kamu aku anggap sebagai belakang kanan
  {
    "Belakang Kanan",
    6, 7,

    // Hip CH6
    1630, 1090, 1375,

    // Knee CH7
    900, 1650, 1375,

    DEFAULT_KNEE_NAIK_MS,
    DEFAULT_KNEE_TURUN_MS,
    DEFAULT_HIP_MAJU_MS,
    DEFAULT_HIP_RESET_MS
  }
};

// ============================================================
// BASIC SERVO CONTROL
// ============================================================
void setServoUS(uint8_t ch, uint16_t pulseUS) {
  if (ch >= SERVO_COUNT) return;
  pca.setPWM(ch, 0, US_TO_TICK(pulseUS));
}

void stopLeg(const LegConfig &leg) {
  setServoUS(leg.hipCH, leg.hipStop);
  setServoUS(leg.kneeCH, leg.kneeStop);
}

void stopAll() {
  for (uint8_t i = 0; i < 4; i++) {
    stopLeg(kaki[i]);
  }

  Serial.println("[STOP] Semua servo berhenti.");
}

void gerakServoUS(uint8_t ch, uint16_t pulseGerak, uint16_t pulseStop, uint16_t durasiMs) {
  setServoUS(ch, pulseGerak);
  delay(durasiMs);

  setServoUS(ch, pulseStop);
  delay(JEDA_MS);
}

// ============================================================
// GERAK SATU KAKI MAJU
// Pola:
// 1. Knee naik
// 2. Hip maju
// 3. Knee turun
// 4. Hip balik/reset/dorong
// ============================================================
void stepMaju(const LegConfig &leg) {
  Serial.println();
  Serial.print("[STEP MAJU] ");
  Serial.println(leg.nama);

  Serial.println("1. Knee naik");
  gerakServoUS(
    leg.kneeCH,
    leg.kneeNaik,
    leg.kneeStop,
    leg.kneeNaikMs
  );

  Serial.println("2. Hip maju");
  gerakServoUS(
    leg.hipCH,
    leg.hipMaju,
    leg.hipStop,
    leg.hipMajuMs
  );

  Serial.println("3. Knee turun");
  gerakServoUS(
    leg.kneeCH,
    leg.kneeTurun,
    leg.kneeStop,
    leg.kneeTurunMs
  );

  Serial.println("4. Hip balik/reset/dorong");
  gerakServoUS(
    leg.hipCH,
    leg.hipBalik,
    leg.hipStop,
    leg.hipResetMs
  );

  stopLeg(leg);

  Serial.print("[DONE MAJU] ");
  Serial.println(leg.nama);
}

// ============================================================
// GERAK SATU KAKI MUNDUR
// Pola:
// 1. Knee naik
// 2. Hip mundur
// 3. Knee turun
// 4. Hip maju/reset/dorong
// ============================================================
void stepMundur(const LegConfig &leg) {
  Serial.println();
  Serial.print("[STEP MUNDUR] ");
  Serial.println(leg.nama);

  Serial.println("1. Knee naik");
  gerakServoUS(
    leg.kneeCH,
    leg.kneeNaik,
    leg.kneeStop,
    leg.kneeNaikMs
  );

  Serial.println("2. Hip mundur");
  gerakServoUS(
    leg.hipCH,
    leg.hipBalik,
    leg.hipStop,
    leg.hipMajuMs
  );

  Serial.println("3. Knee turun");
  gerakServoUS(
    leg.kneeCH,
    leg.kneeTurun,
    leg.kneeStop,
    leg.kneeTurunMs
  );

  Serial.println("4. Hip maju/reset/dorong");
  gerakServoUS(
    leg.hipCH,
    leg.hipMaju,
    leg.hipStop,
    leg.hipResetMs
  );

  stopLeg(leg);

  Serial.print("[DONE MUNDUR] ");
  Serial.println(leg.nama);
}

// ============================================================
// JALAN MAJU 1 SIKLUS
// Urutan sesuai permintaan:
// 1. Belakang kiri
// 2. Depan kiri
// 3. Depan kanan
// 4. Belakang kanan
// ============================================================
void jalanMajuSatuSiklus() {
  Serial.println();
  Serial.println("===== JALAN MAJU 1 SIKLUS =====");

  stepMaju(kaki[KAKI_BELAKANG_KIRI]);
  stepMaju(kaki[KAKI_DEPAN_KIRI]);
  stepMaju(kaki[KAKI_DEPAN_KANAN]);
  stepMaju(kaki[KAKI_BELAKANG_KANAN]);

  stopAll();

  Serial.println("===== SELESAI MAJU 1 SIKLUS =====");
}

// ============================================================
// JALAN MUNDUR 1 SIKLUS
// Urutan sama, arah hip dibalik
// ============================================================
void jalanMundurSatuSiklus() {
  Serial.println();
  Serial.println("===== JALAN MUNDUR 1 SIKLUS =====");

  stepMundur(kaki[KAKI_BELAKANG_KIRI]);
  stepMundur(kaki[KAKI_DEPAN_KIRI]);
  stepMundur(kaki[KAKI_DEPAN_KANAN]);
  stepMundur(kaki[KAKI_BELAKANG_KANAN]);

  stopAll();

  Serial.println("===== SELESAI MUNDUR 1 SIKLUS =====");
}

// ============================================================
// HELP MENU
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("===== QUADRUPED 4 KAKI - MAJU & MUNDUR =====");
  Serial.println("q = jalan maju 1 siklus");
  Serial.println("w = jalan mundur 1 siklus");
  Serial.println("l = stop semua servo");
  Serial.println();
  Serial.println("Test maju per kaki:");
  Serial.println("1 = maju belakang kiri");
  Serial.println("2 = maju depan kiri");
  Serial.println("3 = maju depan kanan");
  Serial.println("4 = maju belakang kanan");
  Serial.println();
  Serial.println("Test mundur per kaki:");
  Serial.println("5 = mundur belakang kiri");
  Serial.println("6 = mundur depan kiri");
  Serial.println("7 = mundur depan kanan");
  Serial.println("8 = mundur belakang kanan");
  Serial.println();
  Serial.println("Mapping channel:");
  Serial.println("CH0/CH1 = Belakang Kiri");
  Serial.println("CH2/CH3 = Depan Kiri");
  Serial.println("CH4/CH5 = Depan Kanan");
  Serial.println("CH6/CH7 = Belakang Kanan");
  Serial.println("============================================");
  Serial.println();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  delay(1000);

  Wire.begin(21, 22);

  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);
  delay(100);

  stopAll();

  Serial.println("[BOOT] ESP32 + PCA9685 siap.");
  Serial.println("[BOOT] Mode 4 kaki maju/mundur aktif.");
  printHelp();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    cmd = tolower(cmd);

    switch (cmd) {
      case 'q':
        jalanMajuSatuSiklus();
        break;

      case 'w':
        jalanMundurSatuSiklus();
        break;

      case 'l':
        stopAll();
        break;

      case '1':
        stepMaju(kaki[KAKI_BELAKANG_KIRI]);
        break;

      case '2':
        stepMaju(kaki[KAKI_DEPAN_KIRI]);
        break;

      case '3':
        stepMaju(kaki[KAKI_DEPAN_KANAN]);
        break;

      case '4':
        stepMaju(kaki[KAKI_BELAKANG_KANAN]);
        break;

      case '5':
        stepMundur(kaki[KAKI_BELAKANG_KIRI]);
        break;

      case '6':
        stepMundur(kaki[KAKI_DEPAN_KIRI]);
        break;

      case '7':
        stepMundur(kaki[KAKI_DEPAN_KANAN]);
        break;

      case '8':
        stepMundur(kaki[KAKI_BELAKANG_KANAN]);
        break;

      case '\n':
      case '\r':
      case ' ':
        break;

      default:
        Serial.print("[!] Command tidak dikenal: ");
        Serial.println(cmd);
        break;
    }
  }
}