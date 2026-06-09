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
// SETTING WAKTU PELAN
// Semua waktu utama minimal 1000 ms agar mudah diamati.
// Ubah bagian ini kalau ingin mempercepat/memperlambat.
// ============================================================

// q = semua kaki turun sampai robot tegak/sejajar
#define TIME_TEGAK_SEJAJAR_MS      400

// jeda umum
#define TIME_JEDA_NORMAL_MS        1000
#define TIME_JEDA_CEPAT_MS         1000

// jeda antar langkah otomatis
#define TIME_AUTO_STEP_GAP_MS      1000

// gerakan 1,2,3,4
// FOOT_UP_MOVE  = kaki/assist naik
// HIP_PREPARE   = hip kaki aktif maju
// FOOT_DOWN     = kaki/assist turun lagi
#define TIME_FOOT_UP_MOVE_MS       1000
#define TIME_FOOT_DOWN_MOVE_MS     1000
#define TIME_HIP_PREPARE_MOVE_MS   1000

// push 5,6,7
// Total ramp = TIME_PUSH_RAMP_STEPS * TIME_PUSH_RAMP_DELAY_MS
// 10 * 100 ms = 1000 ms
#define TIME_PUSH_FULL_MS          1000
#define TIME_PUSH_RAMP_STEPS       10
#define TIME_PUSH_RAMP_DELAY_MS    100

// ============================================================
// PWM TUNING GUIDE
// ============================================================
//
// STOP / NETRAL = 1375
//
// FOOT_UP   = kaki terangkat dari tanah.
// Biasanya PWM lebih kecil dari 1375.
// Contoh: 1050, 1000, 950.
//
// FOOT_DOWN = kaki menekan tanah / robot naik.
// Biasanya PWM lebih besar dari 1375.
// Contoh: 1680, 1740.
//
// FOOT_HOLD = menahan kaki tetap menopang beban.
// Biasanya masih di atas 1375, tapi lebih ringan dari FOOT_DOWN.
// Contoh: 1590.
//
// HIP_PREPARE_MAJU = hip maju ke depan sebelum push.
// HIP_PUSH_MAJU    = hip dorong ke belakang agar badan maju.
//
// Sisi kiri dan kanan arahnya mirror:
// KIRI  prepare cenderung kecil, push cenderung besar.
// KANAN prepare cenderung besar, push cenderung kecil.
//
// ============================================================


// ============================================================
// CHANNEL MAPPING
// ============================================================
//
// Belakang Kiri  = HIP CH0, KNEE CH1
// Depan Kiri     = HIP CH2, KNEE CH3
// Depan Kanan    = HIP CH4, KNEE CH5
// Belakang Kanan = HIP CH6, KNEE CH7
//
// ============================================================


// ============================================================
// PWM TUNING - BELAKANG KIRI
// ============================================================
#define BL_HIP_CH 0
#define BL_KNEE_CH 1

#define BL_HIP_MAJU_US 1580
#define BL_HIP_BALIK_US 1140
#define BL_HIP_STOP_US 1375

// q / tahap awal
#define BL_FOOT_UP_US 1100
#define BL_FOOT_DOWN_STAGE1_US 1650
#define BL_FOOT_HOLD_US 1550
#define BL_FOOT_STOP_US 1375

// gerak 1,2,3,4
#define BL_FOOT_UP_MOVE_US 1050
#define BL_FOOT_DOWN_MOVE_US 1680
#define BL_FOOT_HOLD_MOVE_US 1550

// arah jalan maju
#define BL_HIP_PREPARE_MAJU_US 1080
#define BL_HIP_PUSH_MAJU_US 1640


// ============================================================
// PWM TUNING - DEPAN KIRI
// ============================================================
#define FL_HIP_CH 2
#define FL_KNEE_CH 3

#define FL_HIP_MAJU_US 1600
#define FL_HIP_BALIK_US 1140
#define FL_HIP_STOP_US 1375

// q / tahap awal
#define FL_FOOT_UP_US 1100
#define FL_FOOT_DOWN_STAGE1_US 1660
#define FL_FOOT_HOLD_US 1550
#define FL_FOOT_STOP_US 1375

// gerak 1,2,3,4
#define FL_FOOT_UP_MOVE_US 1050
#define FL_FOOT_DOWN_MOVE_US 1680
#define FL_FOOT_HOLD_MOVE_US 1550

// arah jalan maju
#define FL_HIP_PREPARE_MAJU_US 1100
#define FL_HIP_PUSH_MAJU_US 1620


// ============================================================
// PWM TUNING - DEPAN KANAN
// ============================================================
#define FR_HIP_CH 4
#define FR_KNEE_CH 5

#define FR_HIP_MAJU_US 1580
#define FR_HIP_BALIK_US 1140
#define FR_HIP_STOP_US 1375

// q / tahap awal
#define FR_FOOT_UP_US 1100
#define FR_FOOT_DOWN_STAGE1_US 1680
#define FR_FOOT_HOLD_US 1570
#define FR_FOOT_STOP_US 1375

// gerak 1,2,3,4
#define FR_FOOT_UP_MOVE_US 1050
#define FR_FOOT_DOWN_MOVE_US 1670
#define FR_FOOT_HOLD_MOVE_US 1570

// arah jalan maju
#define FR_HIP_PREPARE_MAJU_US 1630
#define FR_HIP_PUSH_MAJU_US 1090


// ============================================================
// PWM TUNING - BELAKANG KANAN
// ============================================================
#define BR_HIP_CH 6
#define BR_KNEE_CH 7

#define BR_HIP_MAJU_US 1580
#define BR_HIP_BALIK_US 1140
#define BR_HIP_STOP_US 1375

// q / tahap awal
#define BR_FOOT_UP_US 1100
#define BR_FOOT_DOWN_STAGE1_US 1650
#define BR_FOOT_HOLD_US 1550
#define BR_FOOT_STOP_US 1375

// gerak 1,2,3,4
#define BR_FOOT_UP_MOVE_US 1050
#define BR_FOOT_DOWN_MOVE_US 1670
#define BR_FOOT_HOLD_MOVE_US 1550

// arah jalan maju
#define BR_HIP_PREPARE_MAJU_US 1630
#define BR_HIP_PUSH_MAJU_US 1110


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

  uint16_t hipMajuUS;
  uint16_t hipBalikUS;
  uint16_t hipStopUS;

  uint16_t footUpUS;
  uint16_t footDownStage1US;
  uint16_t footHoldUS;
  uint16_t footStopUS;

  uint16_t footUpMoveUS;
  uint16_t footDownMoveUS;
  uint16_t footHoldMoveUS;

  uint16_t hipPrepareMajuUS;
  uint16_t hipPushMajuUS;
};

// ============================================================
// KONFIGURASI KAKI
// Jangan tuning angka di bawah ini.
// Tuning angka dilakukan di bagian PWM TUNING di atas.
// ============================================================
LegConfig kaki[4] = {
  {
    "Belakang Kiri",
    BL_HIP_CH, BL_KNEE_CH,
    BL_HIP_MAJU_US, BL_HIP_BALIK_US, BL_HIP_STOP_US,
    BL_FOOT_UP_US, BL_FOOT_DOWN_STAGE1_US, BL_FOOT_HOLD_US, BL_FOOT_STOP_US,
    BL_FOOT_UP_MOVE_US, BL_FOOT_DOWN_MOVE_US, BL_FOOT_HOLD_MOVE_US,
    BL_HIP_PREPARE_MAJU_US, BL_HIP_PUSH_MAJU_US
  },

  {
    "Depan Kiri",
    FL_HIP_CH, FL_KNEE_CH,
    FL_HIP_MAJU_US, FL_HIP_BALIK_US, FL_HIP_STOP_US,
    FL_FOOT_UP_US, FL_FOOT_DOWN_STAGE1_US, FL_FOOT_HOLD_US, FL_FOOT_STOP_US,
    FL_FOOT_UP_MOVE_US, FL_FOOT_DOWN_MOVE_US, FL_FOOT_HOLD_MOVE_US,
    FL_HIP_PREPARE_MAJU_US, FL_HIP_PUSH_MAJU_US
  },

  {
    "Depan Kanan",
    FR_HIP_CH, FR_KNEE_CH,
    FR_HIP_MAJU_US, FR_HIP_BALIK_US, FR_HIP_STOP_US,
    FR_FOOT_UP_US, FR_FOOT_DOWN_STAGE1_US, FR_FOOT_HOLD_US, FR_FOOT_STOP_US,
    FR_FOOT_UP_MOVE_US, FR_FOOT_DOWN_MOVE_US, FR_FOOT_HOLD_MOVE_US,
    FR_HIP_PREPARE_MAJU_US, FR_HIP_PUSH_MAJU_US
  },

  {
    "Belakang Kanan",
    BR_HIP_CH, BR_KNEE_CH,
    BR_HIP_MAJU_US, BR_HIP_BALIK_US, BR_HIP_STOP_US,
    BR_FOOT_UP_US, BR_FOOT_DOWN_STAGE1_US, BR_FOOT_HOLD_US, BR_FOOT_STOP_US,
    BR_FOOT_UP_MOVE_US, BR_FOOT_DOWN_MOVE_US, BR_FOOT_HOLD_MOVE_US,
    BR_HIP_PREPARE_MAJU_US, BR_HIP_PUSH_MAJU_US
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
// FOOT HOLD
// ============================================================
void holdAllFoot() {
  for (uint8_t i = 0; i < 4; i++) {
    setServoUS(kaki[i].kneeCH, kaki[i].footHoldUS);
  }
}

void holdAllFootExceptTwo(uint8_t activeLegIndex, uint8_t assistLegIndex) {
  for (uint8_t i = 0; i < 4; i++) {
    if (i == activeLegIndex || i == assistLegIndex) continue;
    setServoUS(kaki[i].kneeCH, kaki[i].footHoldUS);
  }
}

// ============================================================
// q = TAHAP AWAL
// Semua FOOT_DOWN sampai robot tegak/sejajar, lalu HOLD
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
// FOOT UP MOVE
// Untuk gerakan 1,2,3,4
// ============================================================
void footUpMove(uint8_t legIndex) {
  LegConfig &leg = kaki[legIndex];

  Serial.print("[FOOT_UP MOVE] ");
  Serial.println(leg.nama);

  setServoUS(leg.kneeCH, leg.footUpMoveUS);
  delay(TIME_FOOT_UP_MOVE_MS);

  stopFoot(leg);
  delay(TIME_JEDA_CEPAT_MS);
}

// ============================================================
// FOOT DOWN MOVE + HOLD
// Untuk gerakan 1,2,3,4
// ============================================================
void footDownMoveHold(uint8_t legIndex) {
  LegConfig &leg = kaki[legIndex];

  Serial.print("[FOOT_DOWN MOVE + HOLD] ");
  Serial.println(leg.nama);

  setServoUS(leg.kneeCH, leg.footDownMoveUS);
  delay(TIME_FOOT_DOWN_MOVE_MS);

  setServoUS(leg.kneeCH, leg.footHoldMoveUS);
  delay(TIME_JEDA_CEPAT_MS);
}

// ============================================================
// MAJU PER KAKI DENGAN ASSIST DIAGONAL
//
// Contoh command 1:
// Active = Depan Kiri
// Assist = Belakang Kanan
//
// Urutan:
// 1. Kaki lain tetap hold
// 2. Assist diagonal naik dulu
// 3. Kaki aktif naik
// 4. Hip aktif maju
// 5. Kaki aktif turun + hold
// 6. Assist turun + hold
// 7. Semua hold
// ============================================================
void majuSatuKakiDenganAssist(uint8_t activeLegIndex, uint8_t assistLegIndex) {
  LegConfig &active = kaki[activeLegIndex];
  LegConfig &assist = kaki[assistLegIndex];

  Serial.println();
  Serial.print("===== MAJU DENGAN ASSIST: ");
  Serial.print(active.nama);
  Serial.print(" | Assist: ");
  Serial.print(assist.nama);
  Serial.println(" =====");

  holdAllFootExceptTwo(activeLegIndex, assistLegIndex);
  delay(TIME_JEDA_CEPAT_MS);

  Serial.println("[STEP] Assist diagonal FOOT_UP");
  footUpMove(assistLegIndex);

  Serial.println("[STEP] Active FOOT_UP");
  footUpMove(activeLegIndex);

  Serial.print("[HIP PREPARE MAJU] ");
  Serial.println(active.nama);

  setServoUS(active.hipCH, active.hipPrepareMajuUS);
  delay(TIME_HIP_PREPARE_MOVE_MS);

  stopHip(active);
  delay(TIME_JEDA_CEPAT_MS);

  Serial.println("[STEP] Active FOOT_DOWN + HOLD");
  footDownMoveHold(activeLegIndex);

  Serial.println("[STEP] Assist FOOT_DOWN + HOLD");
  footDownMoveHold(assistLegIndex);

  holdAllFoot();

  Serial.print("[DONE] Maju dengan assist: ");
  Serial.println(active.nama);
}

// ============================================================
// COMMAND 1,2,3,4
// ============================================================
void command1_DepanKiriMaju() {
  majuSatuKakiDenganAssist(LEG_FL, LEG_BR);
}

void command2_DepanKananMaju() {
  majuSatuKakiDenganAssist(LEG_FR, LEG_BL);
}

void command3_BelakangKiriMaju() {
  majuSatuKakiDenganAssist(LEG_BL, LEG_FR);
}

void command4_BelakangKananMaju() {
  majuSatuKakiDenganAssist(LEG_BR, LEG_FL);
}

// ============================================================
// RAMP HIP
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
// PUSH SISI
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

  Serial.print("[TARGET PUSH] ");
  Serial.print(a.nama);
  Serial.print(" = ");
  Serial.println(targetA);

  Serial.print("[TARGET PUSH] ");
  Serial.print(b.nama);
  Serial.print(" = ");
  Serial.println(targetB);

  rampTwoHipToTarget(legA, legB, targetA, targetB);

  delay(TIME_PUSH_FULL_MS);

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
// PUSH SEMUA
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
// OTOMATIS MAJU
// Urutan: 3 -> 1 -> 5 -> 4 -> 2 -> 6
// ============================================================
void majuOtomatisSatuSiklus() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("MAJU OTOMATIS 1 SIKLUS - SLOW MODE");
  Serial.println("Urutan: 3 -> 1 -> 5 -> 4 -> 2 -> 6");
  Serial.println("======================================");

  command3_BelakangKiriMaju();
  delay(TIME_AUTO_STEP_GAP_MS);

  command1_DepanKiriMaju();
  delay(TIME_AUTO_STEP_GAP_MS);

  pushKiriKeBelakang();
  delay(TIME_AUTO_STEP_GAP_MS);

  command4_BelakangKananMaju();
  delay(TIME_AUTO_STEP_GAP_MS);

  command2_DepanKananMaju();
  delay(TIME_AUTO_STEP_GAP_MS);

  pushKananKeBelakang();
  delay(TIME_AUTO_STEP_GAP_MS);

  Serial.println("[DONE] Maju otomatis 1 siklus selesai.");
}

// ============================================================
// OTOMATIS MAJU DARI AWAL
// q -> 3 -> 1 -> 5 -> 4 -> 2 -> 6
// ============================================================
void majuOtomatisDenganTahapAwal() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("MAJU OTOMATIS DENGAN TAHAP AWAL - SLOW MODE");
  Serial.println("======================================");

  tahapAwalFootDownHold();
  delay(TIME_AUTO_STEP_GAP_MS);

  majuOtomatisSatuSiklus();

  Serial.println("[DONE] Maju otomatis dengan tahap awal selesai.");
}

// ============================================================
// BELOK KANAN
// 3 -> 1 -> 5
// ============================================================
void belokKananOtomatis() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("BELOK KANAN OTOMATIS - SLOW MODE");
  Serial.println("Urutan: 3 -> 1 -> 5");
  Serial.println("======================================");

  command3_BelakangKiriMaju();
  delay(TIME_AUTO_STEP_GAP_MS);

  command1_DepanKiriMaju();
  delay(TIME_AUTO_STEP_GAP_MS);

  pushKiriKeBelakang();
  delay(TIME_AUTO_STEP_GAP_MS);

  Serial.println("[DONE] Belok kanan selesai.");
}

// ============================================================
// BELOK KIRI
// 4 -> 2 -> 6
// ============================================================
void belokKiriOtomatis() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("BELOK KIRI OTOMATIS - SLOW MODE");
  Serial.println("Urutan: 4 -> 2 -> 6");
  Serial.println("======================================");

  command4_BelakangKananMaju();
  delay(TIME_AUTO_STEP_GAP_MS);

  command2_DepanKananMaju();
  delay(TIME_AUTO_STEP_GAP_MS);

  pushKananKeBelakang();
  delay(TIME_AUTO_STEP_GAP_MS);

  Serial.println("[DONE] Belok kiri selesai.");
}

// ============================================================
// HELP
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("===== MODE ASSIST DIAGONAL - SLOW TUNING =====");
  Serial.println("q = tahap awal: semua FOOT_DOWN sejajar + HOLD");
  Serial.println();
  Serial.println("1 = Depan kiri maju, assist Belakang kanan");
  Serial.println("2 = Depan kanan maju, assist Belakang kiri");
  Serial.println("3 = Belakang kiri maju, assist Depan kanan");
  Serial.println("4 = Belakang kanan maju, assist Depan kiri");
  Serial.println();
  Serial.println("5 = Push sisi kiri ke belakang");
  Serial.println("6 = Push sisi kanan ke belakang");
  Serial.println("7 = Push semua hip ke belakang");
  Serial.println();
  Serial.println("m = maju otomatis: 3 -> 1 -> 5 -> 4 -> 2 -> 6");
  Serial.println("a = q + maju otomatis");
  Serial.println("r = belok kanan: 3 -> 1 -> 5");
  Serial.println("t = belok kiri : 4 -> 2 -> 6");
  Serial.println();
  Serial.println("x = stop semua hip");
  Serial.println("l = emergency stop semua servo");
  Serial.println("h = help");
  Serial.println();
  Serial.println("SEMUA WAKTU MINIMAL 1 DETIK UNTUK OBSERVASI.");
  Serial.println("Tuning waktu ada di bagian SETTING WAKTU.");
  Serial.println("Tuning PWM ada di bagian PWM TUNING.");
  Serial.println("==================================================");
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
  Serial.println("[BOOT] Mode slow tuning aktif.");

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
        command1_DepanKiriMaju();
        break;

      case '2':
        command2_DepanKananMaju();
        break;

      case '3':
        command3_BelakangKiriMaju();
        break;

      case '4':
        command4_BelakangKananMaju();
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

      case 'm':
        majuOtomatisSatuSiklus();
        break;

      case 'a':
        majuOtomatisDenganTahapAwal();
        break;

      case 'r':
        belokKananOtomatis();
        break;

      case 't':
        belokKiriOtomatis();
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