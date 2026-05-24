#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ctype.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define SERVO_COUNT 8

#define US_TO_TICK(us) ((uint16_t)(((float)(us) / 20000.0) * 4096))

#define DEFAULT_STOP_US 1375

#define JEDA_MS 150

// ============================================================
// INDEX KAKI
// ============================================================
#define LEG_BL 0   // Belakang Kiri
#define LEG_FL 1   // Depan Kiri
#define LEG_FR 2   // Depan Kanan
#define LEG_BR 3   // Belakang Kanan

// ============================================================
// STRUKTUR KAKI
// ============================================================
struct LegConfig {
  const char* nama;

  uint8_t hipCH;
  uint8_t kneeCH;

  // HIP
  uint16_t hipMajuUS;
  uint16_t hipBalikUS;
  uint16_t hipStopUS;

  // KNEE
  uint16_t kneeNaikUS;
  uint16_t kneeTurunSedikitUS;
  uint16_t kneeHoldUS;
  uint16_t kneeStopUS;

  // DURASI
  uint16_t kneeTurunSedikitMs;
  uint16_t kneeNaikMs;
  uint16_t hipPrepareMs;
  uint16_t hipPushMs;

  // LOGIKA ARAH MAJU
  uint16_t hipPrepareMajuUS;
  uint16_t hipPushMajuUS;
};

// ============================================================
// KONFIGURASI 4 KAKI
//
// Mapping:
// CH0 = HIP  Belakang Kiri
// CH1 = KNEE Belakang Kiri
//
// CH2 = HIP  Depan Kiri
// CH3 = KNEE Depan Kiri
//
// CH4 = HIP  Depan Kanan
// CH5 = KNEE Depan Kanan
//
// CH6 = HIP  Belakang Kanan
// CH7 = KNEE Belakang Kanan
// ============================================================

LegConfig kaki[4] = {
  // ==========================================================
  // 0. BELAKANG KIRI
  // ==========================================================
  {
    "Belakang Kiri",

    0, 1,

    // hipMaju, hipBalik, hipStop
    1600, 1090, 1375,

    // kneeNaik, kneeTurunSedikit, kneeHold, kneeStop
    1050, 1660, 1550, 1375,

    // kneeTurunSedikitMs, kneeNaikMs, hipPrepareMs, hipPushMs
    180, 180, 400, 400,

    // Logika sisi kiri:
    // prepare maju = BALIK
    // push maju    = MAJU
    1090, 1600
  },

  // ==========================================================
  // 1. DEPAN KIRI
  // ==========================================================
  {
    "Depan Kiri",

    2, 3,

    // hipMaju, hipBalik, hipStop
    1645, 1070, 1375,

    // kneeNaik, kneeTurunSedikit, kneeHold, kneeStop
    1110, 1620, 1510, 1375,

    // kneeTurunSedikitMs, kneeNaikMs, hipPrepareMs, hipPushMs
    180, 180, 400, 400,

    // Logika sisi kiri:
    // prepare maju = BALIK
    // push maju    = MAJU
    1070, 1645
  },

  // ==========================================================
  // 2. DEPAN KANAN
  // ==========================================================
  {
    "Depan Kanan",

    4, 5,

    // hipMaju, hipBalik, hipStop
    1630, 1095, 1375,

    // kneeNaik, kneeTurunSedikit, kneeHold, kneeStop
    1050, 1670, 1500, 1375,

    // kneeTurunSedikitMs, kneeNaikMs, hipPrepareMs, hipPushMs
    180, 180, 400, 400,

    // Logika sisi kanan:
    // prepare maju = MAJU
    // push maju    = BALIK
    1630, 1095
  },

  // ==========================================================
  // 3. BELAKANG KANAN
  // ==========================================================
  {
    "Belakang Kanan",

    6, 7,

    // hipMaju, hipBalik, hipStop
    1630, 1090, 1375,

    // kneeNaik, kneeTurunSedikit, kneeHold, kneeStop
    1050, 1670, 1500, 1375,

    // kneeTurunSedikitMs, kneeNaikMs, hipPrepareMs, hipPushMs
    180, 180, 400, 400,

    // Logika sisi kanan:
    // prepare maju = MAJU
    // push maju    = BALIK
    1630, 1090
  }
};

// ============================================================
// URUTAN TAHAP 2
// ============================================================
uint8_t prepareOrder[4] = {
  LEG_FL,
  LEG_FR,
  LEG_BL,
  LEG_BR
};

// ============================================================
// BASIC SERVO CONTROL
// ============================================================
void setServoUS(uint8_t ch, uint16_t pulseUS) {
  if (ch >= SERVO_COUNT) return;
  pca.setPWM(ch, 0, US_TO_TICK(pulseUS));
}

void stopHip(const LegConfig &leg) {
  setServoUS(leg.hipCH, leg.hipStopUS);
}

void stopKnee(const LegConfig &leg) {
  setServoUS(leg.kneeCH, leg.kneeStopUS);
}

void stopLeg(const LegConfig &leg) {
  stopHip(leg);
  stopKnee(leg);
}

void stopAll() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoUS(i, DEFAULT_STOP_US);
  }

  for (uint8_t i = 0; i < 4; i++) {
    stopLeg(kaki[i]);
  }

  Serial.println("[STOP] Semua servo stop.");
}

// ============================================================
// TAHAP 1
// Semua knee turun sedikit sampai sejajar, lalu hold.
// ============================================================
void tahap1KneeSejajarHold() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("TAHAP 1: SEMUA KNEE TURUN SEJAJAR");
  Serial.println("======================================");

  // Semua knee turun sedikit
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print("[KNEE TURUN] ");
    Serial.println(kaki[i].nama);

    setServoUS(kaki[i].kneeCH, kaki[i].kneeTurunSedikitUS);
  }

  // Ambil durasi terbesar
  uint16_t maxTurunMs = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (kaki[i].kneeTurunSedikitMs > maxTurunMs) {
      maxTurunMs = kaki[i].kneeTurunSedikitMs;
    }
  }

  delay(maxTurunMs);

  // Semua knee hold agar posisi sejajar dan menahan beban
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print("[KNEE HOLD] ");
    Serial.println(kaki[i].nama);

    setServoUS(kaki[i].kneeCH, kaki[i].kneeHoldUS);
  }

  Serial.println("[DONE] Tahap 1 selesai. Semua knee sejajar dan hold.");
}

// ============================================================
// KNEE NAIK MENTOK SATU KAKI
// ============================================================
void kneeNaikMentok(const LegConfig &leg) {
  Serial.print("[KNEE NAIK MENTOK] ");
  Serial.println(leg.nama);

  setServoUS(leg.kneeCH, leg.kneeNaikUS);
  delay(leg.kneeNaikMs);

  stopKnee(leg);
  delay(JEDA_MS);
}

// ============================================================
// KNEE TURUN LAGI SAMPAI SEJAJAR + HOLD
// ============================================================
void kneeTurunSejajarHold(const LegConfig &leg) {
  Serial.print("[KNEE TURUN SEJAJAR] ");
  Serial.println(leg.nama);

  setServoUS(leg.kneeCH, leg.kneeTurunSedikitUS);
  delay(leg.kneeTurunSedikitMs);

  setServoUS(leg.kneeCH, leg.kneeHoldUS);
  delay(JEDA_MS);
}

// ============================================================
// PREPARE SATU KAKI MAJU
//
// Urutan:
// 1. Knee naik mentok
// 2. Hip maju ke depan
// 3. Knee turun lagi sejajar + hold
// ============================================================
void prepareSatuKakiMaju(uint8_t legIndex) {
  LegConfig &leg = kaki[legIndex];

  Serial.println();
  Serial.print("===== PREPARE MAJU: ");
  Serial.print(leg.nama);
  Serial.println(" =====");

  // 1. Knee naik mentok
  kneeNaikMentok(leg);

  // 2. Hip maju ke depan
  Serial.print("[HIP MAJU KE DEPAN] ");
  Serial.println(leg.nama);

  setServoUS(leg.hipCH, leg.hipPrepareMajuUS);
  delay(leg.hipPrepareMs);

  stopHip(leg);
  delay(JEDA_MS);

  // 3. Knee turun lagi sampai sejajar + hold
  kneeTurunSejajarHold(leg);

  Serial.print("[DONE PREPARE] ");
  Serial.println(leg.nama);
}

// ============================================================
// SEMUA HIP PUSH MUNDUR BERSAMAAN
//
// Semua kaki sudah menapak/sejajar.
// Semua hip bergerak mundur/push agar robot terdorong maju.
// ============================================================
void pushSemuaHipMaju() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("PUSH SEMUA HIP: ROBOT MAJU");
  Serial.println("======================================");

  // Pastikan semua knee tetap hold
  for (uint8_t i = 0; i < 4; i++) {
    setServoUS(kaki[i].kneeCH, kaki[i].kneeHoldUS);
  }

  delay(JEDA_MS);

  // Semua hip push bersamaan
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print("[HIP PUSH] ");
    Serial.println(kaki[i].nama);

    setServoUS(kaki[i].hipCH, kaki[i].hipPushMajuUS);
  }

  // Stop hip berdasarkan durasi masing-masing
  bool stopped[4] = {false, false, false, false};
  uint8_t stoppedCount = 0;
  uint32_t startTime = millis();

  while (stoppedCount < 4) {
    uint32_t elapsed = millis() - startTime;

    for (uint8_t i = 0; i < 4; i++) {
      if (!stopped[i] && elapsed >= kaki[i].hipPushMs) {
        stopHip(kaki[i]);
        stopped[i] = true;
        stoppedCount++;

        Serial.print("[HIP STOP] ");
        Serial.println(kaki[i].nama);
      }
    }

    delay(5);
  }

  // Setelah push, knee tetap hold agar robot tidak langsung turun/naik aneh
  for (uint8_t i = 0; i < 4; i++) {
    setServoUS(kaki[i].kneeCH, kaki[i].kneeHoldUS);
  }

  Serial.println("[DONE] Push semua hip selesai.");
}

// ============================================================
// TAHAP 2
//
// Urutan:
// 1. Depan kiri
// 2. Depan kanan
// 3. Belakang kiri
// 4. Belakang kanan
// 5. Semua hip push mundur bersamaan
// ============================================================
void tahap2PrepareLaluPush() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("TAHAP 2: PREPARE SATU-SATU + PUSH");
  Serial.println("======================================");

  // Pastikan sebelum tahap 2, semua knee dalam posisi hold sejajar
  for (uint8_t i = 0; i < 4; i++) {
    setServoUS(kaki[i].kneeCH, kaki[i].kneeHoldUS);
  }

  delay(JEDA_MS);

  // Kaki maju satu per satu
  for (uint8_t i = 0; i < 4; i++) {
    prepareSatuKakiMaju(prepareOrder[i]);
  }

  // Semua hip push bersamaan
  pushSemuaHipMaju();

  Serial.println("[DONE] Tahap 2 selesai.");
}

// ============================================================
// HELP
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("===== QUADRUPED 2 TAHAP TEST =====");
  Serial.println("q = Tahap 1: semua knee turun sejajar + hold");
  Serial.println("w = Tahap 2: kaki maju satu per satu + semua hip push");
  Serial.println("l = emergency stop semua servo");
  Serial.println("h = help");
  Serial.println();
  Serial.println("Urutan Tahap 2:");
  Serial.println("1. Depan kiri: knee naik mentok -> hip maju -> knee turun sejajar");
  Serial.println("2. Depan kanan: knee naik mentok -> hip maju -> knee turun sejajar");
  Serial.println("3. Belakang kiri: knee naik mentok -> hip maju -> knee turun sejajar");
  Serial.println("4. Belakang kanan: knee naik mentok -> hip maju -> knee turun sejajar");
  Serial.println("5. Semua hip push bersamaan agar robot maju");
  Serial.println();
  Serial.println("Mapping:");
  Serial.println("CH0/CH1 = Belakang Kiri");
  Serial.println("CH2/CH3 = Depan Kiri");
  Serial.println("CH4/CH5 = Depan Kanan");
  Serial.println("CH6/CH7 = Belakang Kanan");
  Serial.println("==================================");
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

  Serial.println();
  Serial.println("[BOOT] ESP32 + PCA9685 siap.");
  Serial.println("[BOOT] Mode 2 tahap aktif.");
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
        tahap1KneeSejajarHold();
        break;

      case 'w':
        tahap2PrepareLaluPush();
        break;

      case 'l':
        stopAll();
        break;

      case 'h':
        printHelp();
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