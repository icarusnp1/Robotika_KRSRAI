#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ctype.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define SERVO_COUNT 8

#define US_TO_TICK(us) ((uint16_t)(((float)(us) / 20000.0) * 4096))

// ============================================================
// SETTING DASAR
// ============================================================
#define DEFAULT_STOP_US 1375

// ============================================================
// SETTING WAKTU UTAMA
// Ubah semua durasi di bagian ini saja
// ============================================================

// Tahap awal: semua kaki turun sampai robot tegak/sejajar
// Ini 1 nilai untuk semua kaki
#define TIME_TEGAK_SEJAJAR_MS 220

// Jeda umum
#define TIME_JEDA_NORMAL_MS 150
#define TIME_JEDA_CEPAT_MS  30

// Gerakan maju cepat per satu kaki
#define TIME_FOOT_UP_FAST_MS       120
#define TIME_FOOT_DOWN_FAST_MS     90
#define TIME_HIP_PREPARE_FAST_MS   190

// Push sisi / push badan
#define TIME_PUSH_FULL_MS          20
#define TIME_PUSH_RAMP_STEPS       10
#define TIME_PUSH_RAMP_DELAY_MS    30

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

  // FOOT / KNEE TAHAP AWAL
  uint16_t footUpUS;
  uint16_t footDownStage1US;
  uint16_t footHoldUS;
  uint16_t footStopUS;

  // FOOT / KNEE SAAT MAJU PER KAKI
  uint16_t footUpFastUS;
  uint16_t footDownFastUS;
  uint16_t footHoldFastUS;

  // ARAH MAJU
  uint16_t hipPrepareMajuUS;
  uint16_t hipPushMajuUS;
};

// ============================================================
// KONFIGURASI 4 KAKI
//
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

    // footUp, footDownStage1, footHold, footStop
    1050, 1740, 1590, 1375,

    // footUpFast, footDownFast, footHoldFast
    1000, 1740, 1590,

    // Sisi kiri:
    // prepare maju = BALIK
    // push maju    = MAJU
    1000, 1680 //HIP
  },

  // ==========================================================
  // 1. DEPAN KIRI
  // ==========================================================
  {
    "Depan Kiri",

    2, 3,

    // hipMaju, hipBalik, hipStop
    1645, 1070, 1375,

    // footUp, footDownStage1, footHold, footStop
    1110, 1740, 1590, 1375,

    // footUpFast, footDownFast, footHoldFast
    1000, 1740, 1590,

    // Sisi kiri:
    // prepare maju = BALIK
    // push maju    = MAJU
    1050, 1645 //HIP
  },

  // ==========================================================
  // 2. DEPAN KANAN
  // ==========================================================
  {
    "Depan Kanan",

    4, 5,

    // hipMaju, hipBalik, hipStop
    1630, 1095, 1375,

    // footUp, footDownStage1, footHold, footStop
    1050, 1740, 1590, 1375,

    // footUpFast, footDownFast, footHoldFast
    1000, 1730, 1590,

    // Sisi kanan:
    // prepare maju = MAJU
    // push maju    = BALIK
    // Disamakan kekuatannya dengan sisi kiri
    1680, 1030 //HIP
  },

  // ==========================================================
  // 3. BELAKANG KANAN
  // ==========================================================
  {
    "Belakang Kanan",

    6, 7,

    // hipMaju, hipBalik, hipStop
    1630, 1090, 1375,

    // footUp, footDownStage1, footHold, footStop
    1050, 1740, 1590, 1375,

    // footUpFast, footDownFast, footHoldFast
    1000, 1730, 1590,

    // Sisi kanan:
    // prepare maju = MAJU
    // push maju    = BALIK
    // Disamakan kekuatannya dengan sisi kiri
    1680, 1060 //HIP
  }
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

void stopFoot(const LegConfig &leg) {
  setServoUS(leg.kneeCH, leg.footStopUS);
}

void stopLeg(const LegConfig &leg) {
  stopHip(leg);
  stopFoot(leg);
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

void stopAllHip() {
  for (uint8_t i = 0; i < 4; i++) {
    stopHip(kaki[i]);
  }

  Serial.println("[STOP] Semua hip stop.");
}

// ============================================================
// HOLD FOOT
// ============================================================
void holdAllFoot() {
  for (uint8_t i = 0; i < 4; i++) {
    setServoUS(kaki[i].kneeCH, kaki[i].footHoldUS);
  }
}

void holdAllFootExcept(uint8_t activeLegIndex) {
  for (uint8_t i = 0; i < 4; i++) {
    if (i == activeLegIndex) continue;
    setServoUS(kaki[i].kneeCH, kaki[i].footHoldUS);
  }
}

// ============================================================
// TAHAP AWAL
// Semua FOOT_DOWN sejajar + HOLD
// Waktu turun sejajar pakai 1 nilai global:
// TIME_TEGAK_SEJAJAR_MS
// ============================================================
void tahapAwalFootDownHold() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("TAHAP AWAL: SEMUA FOOT_DOWN + HOLD");
  Serial.println("======================================");

  for (uint8_t i = 0; i < 4; i++) {
    Serial.print("[FOOT_DOWN] ");
    Serial.println(kaki[i].nama);

    setServoUS(kaki[i].kneeCH, kaki[i].footDownStage1US);
  }

  delay(TIME_TEGAK_SEJAJAR_MS);

  holdAllFoot();

  Serial.println("[DONE] Semua kaki sejajar dan hold.");
}

// ============================================================
// MAJU CEPAT PER SATU KAKI
//
// Urutan:
// 1. Kaki lain tetap HOLD
// 2. FOOT_UP cepat
// 3. HIP maju cepat
// 4. FOOT_DOWN cepat + HOLD
// ============================================================
void majuCepatSatuKaki(uint8_t legIndex) {
  LegConfig &leg = kaki[legIndex];

  Serial.println();
  Serial.print("===== MAJU CEPAT SATU KAKI: ");
  Serial.print(leg.nama);
  Serial.println(" =====");

  // Kaki lain tetap menahan beban
  holdAllFootExcept(legIndex);
  delay(TIME_JEDA_CEPAT_MS);

  // 1. FOOT_UP cepat
  Serial.print("[FOOT_UP CEPAT] ");
  Serial.println(leg.nama);

  setServoUS(leg.kneeCH, leg.footUpFastUS);
  delay(TIME_FOOT_UP_FAST_MS);

  stopFoot(leg);
  delay(TIME_JEDA_CEPAT_MS);

  // 2. HIP maju cepat
  Serial.print("[HIP MAJU CEPAT] ");
  Serial.println(leg.nama);

  setServoUS(leg.hipCH, leg.hipPrepareMajuUS);
  delay(TIME_HIP_PREPARE_FAST_MS);

  stopHip(leg);
  delay(TIME_JEDA_CEPAT_MS);

  // 3. FOOT_DOWN cepat + HOLD
  Serial.print("[FOOT_DOWN CEPAT + HOLD] ");
  Serial.println(leg.nama);

  setServoUS(leg.kneeCH, leg.footDownFastUS);
  delay(TIME_FOOT_DOWN_FAST_MS);

  setServoUS(leg.kneeCH, leg.footHoldFastUS);
  delay(TIME_JEDA_CEPAT_MS);

  holdAllFoot();

  Serial.print("[DONE] Maju cepat: ");
  Serial.println(leg.nama);
}

// ============================================================
// RAMP DUA HIP BERSAMAAN UNTUK PUSH PENUH TAPI HALUS
// ============================================================
void rampTwoHipToTarget(uint8_t legA, uint8_t legB, uint16_t targetA, uint16_t targetB) {
  LegConfig &a = kaki[legA];
  LegConfig &b = kaki[legB];

  for (int step = 1; step <= TIME_PUSH_RAMP_STEPS; step++) {
    uint16_t valueA = a.hipStopUS + (((int16_t)targetA - (int16_t)a.hipStopUS) * step) / TIME_PUSH_RAMP_STEPS;
    uint16_t valueB = b.hipStopUS + (((int16_t)targetB - (int16_t)b.hipStopUS) * step) / TIME_PUSH_RAMP_STEPS;

    setServoUS(a.hipCH, valueA);
    setServoUS(b.hipCH, valueB);

    delay(TIME_PUSH_RAMP_DELAY_MS);
  }
}

void rampTwoHipToStop(uint8_t legA, uint8_t legB, uint16_t fromA, uint16_t fromB) {
  LegConfig &a = kaki[legA];
  LegConfig &b = kaki[legB];

  for (int step = TIME_PUSH_RAMP_STEPS; step >= 1; step--) {
    uint16_t valueA = a.hipStopUS + (((int16_t)fromA - (int16_t)a.hipStopUS) * step) / TIME_PUSH_RAMP_STEPS;
    uint16_t valueB = b.hipStopUS + (((int16_t)fromB - (int16_t)b.hipStopUS) * step) / TIME_PUSH_RAMP_STEPS;

    setServoUS(a.hipCH, valueA);
    setServoUS(b.hipCH, valueB);

    delay(TIME_PUSH_RAMP_DELAY_MS);
  }

  stopHip(a);
  stopHip(b);
}

// ============================================================
// PUSH SATU SISI KE BELAKANG
//
// Push memakai hipPushMajuUS penuh.
// Perlahan/halusnya diatur dari ramp.
// ============================================================
void pushSisiKeBelakang(uint8_t legA, uint8_t legB, const char* namaSisi) {
  LegConfig &a = kaki[legA];
  LegConfig &b = kaki[legB];

  Serial.println();
  Serial.print("===== PUSH SISI ");
  Serial.print(namaSisi);
  Serial.println(" KE BELAKANG =====");

  holdAllFoot();
  delay(TIME_JEDA_CEPAT_MS);

  uint16_t targetA = a.hipPushMajuUS;
  uint16_t targetB = b.hipPushMajuUS;

  Serial.print("[TARGET PUSH FULL] ");
  Serial.print(a.nama);
  Serial.print(" = ");
  Serial.println(targetA);

  Serial.print("[TARGET PUSH FULL] ");
  Serial.print(b.nama);
  Serial.print(" = ");
  Serial.println(targetB);

  // Ramp naik sampai PWM push penuh
  rampTwoHipToTarget(legA, legB, targetA, targetB);

  // Tahan agar benar-benar mendorong ke belakang
  delay(TIME_PUSH_FULL_MS);

  // Ramp turun kembali ke stop
  rampTwoHipToStop(legA, legB, targetA, targetB);

  holdAllFoot();

  Serial.print("[DONE] Push sisi ");
  Serial.print(namaSisi);
  Serial.println(" selesai.");
}

void pushKiriKeBelakang() {
  pushSisiKeBelakang(LEG_FL, LEG_BL, "KIRI");
}

void pushKananKeBelakang() {
  pushSisiKeBelakang(LEG_FR, LEG_BR, "KANAN");
}

// ============================================================
// PUSH SEMUA HIP KE BELAKANG BERSAMAAN
// ============================================================
void pushSemuaKeBelakang() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("PUSH SEMUA HIP KE BELAKANG");
  Serial.println("======================================");

  holdAllFoot();
  delay(TIME_JEDA_CEPAT_MS);

  for (uint8_t i = 0; i < 4; i++) {
    Serial.print("[HIP PUSH] ");
    Serial.print(kaki[i].nama);
    Serial.print(" = ");
    Serial.println(kaki[i].hipPushMajuUS);

    setServoUS(kaki[i].hipCH, kaki[i].hipPushMajuUS);
  }

  delay(TIME_PUSH_FULL_MS);

  stopAllHip();
  holdAllFoot();

  Serial.println("[DONE] Push semua hip selesai.");
}

// ============================================================
// HELP
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("===== MODE MANUAL: 4 KAKI MAJU + PUSH SISI FULL =====");
  Serial.println("q = tahap awal: semua FOOT_DOWN sejajar + HOLD");
  Serial.println();
  Serial.println("Kontrol maju cepat per kaki:");
  Serial.println("1 = Depan kiri maju cepat");
  Serial.println("2 = Depan kanan maju cepat");
  Serial.println("3 = Belakang kiri maju cepat");
  Serial.println("4 = Belakang kanan maju cepat");
  Serial.println();
  Serial.println("Kontrol push ke belakang per sisi:");
  Serial.println("5 = Push sisi kiri  ke belakang (Depan kiri + Belakang kiri)");
  Serial.println("6 = Push sisi kanan ke belakang (Depan kanan + Belakang kanan)");
  Serial.println("7 = Push semua hip ke belakang");
  Serial.println();
  Serial.println("x = stop semua hip");
  Serial.println("l = emergency stop semua servo");
  Serial.println("h = help");
  Serial.println();
  Serial.println("Contoh urutan:");
  Serial.println("q -> 1 -> 2 -> 3 -> 4 -> 5 -> 6");
  Serial.println();
  Serial.println("Setting waktu ada di bagian atas:");
  Serial.println("TIME_TEGAK_SEJAJAR_MS");
  Serial.println("TIME_FOOT_UP_FAST_MS");
  Serial.println("TIME_FOOT_DOWN_FAST_MS");
  Serial.println("TIME_HIP_PREPARE_FAST_MS");
  Serial.println("TIME_PUSH_FULL_MS");
  Serial.println("=======================================================");
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
  Serial.println("[BOOT] Mode manual 4 kaki + push sisi full aktif.");

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
        tahapAwalFootDownHold();
        break;

      case '1':
        majuCepatSatuKaki(LEG_FL);
        break;

      case '2':
        majuCepatSatuKaki(LEG_FR);
        break;

      case '3':
        majuCepatSatuKaki(LEG_BL);
        break;

      case '4':
        majuCepatSatuKaki(LEG_BR);
        break;

      case '5':
        pushKiriKeBelakang();
        break;

      case '6':
        pushKananKeBelakang();
        break;

      case '7':
        pushSemuaKeBelakang();
        break;

      case 'x':
        stopAllHip();
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