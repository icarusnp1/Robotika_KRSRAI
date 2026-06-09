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
// SETTING WAKTU
// ============================================================

// Lama masing-masing hip maju/mundur satu per satu
#define TIME_HIP_PREPARE_SINGLE_MS 500

// Lama push dua hip / semua hip
#define TIME_HIP_PUSH_PAIR_MS      500
#define TIME_HIP_PUSH_ALL_MS       500

// Jeda antar gerakan
#define TIME_JEDA_MS              200
#define TIME_AUTO_GAP_MS          300

// Ramp agar gerakan tidak menghentak
#define USE_RAMP                  true
#define RAMP_STEPS                10
#define RAMP_DELAY_MS             30

// ============================================================
// CHANNEL MAPPING
// ============================================================
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
//
// Mode ini hanya menggerakkan HIP.
// KNEE tidak digerakkan.
// ============================================================


// ============================================================
// PWM TUNING - HIP ONLY
//
// STOP / NETRAL = 1375
//
// MAJU:
// KIRI  prepare = kecil, push = besar
// KANAN prepare = besar, push = kecil
//
// MUNDUR:
// Kebalikan dari maju.
// KIRI  prepare = besar, push = kecil
// KANAN prepare = kecil, push = besar
// ============================================================

// ----------------------
// DEPAN KIRI
// ----------------------
#define FL_HIP_CH          2
#define FL_HIP_STOP_US     1375
#define FL_HIP_PREPARE_US  1030
#define FL_HIP_PUSH_US     1640

// ----------------------
// BELAKANG KIRI
// ----------------------
#define BL_HIP_CH          0
#define BL_HIP_STOP_US     1375
#define BL_HIP_PREPARE_US  1050
#define BL_HIP_PUSH_US     1675

// ----------------------
// DEPAN KANAN
// ----------------------
#define FR_HIP_CH          4
#define FR_HIP_STOP_US     1375
#define FR_HIP_PREPARE_US  1680
#define FR_HIP_PUSH_US     1030

// ----------------------
// BELAKANG KANAN
// ----------------------
#define BR_HIP_CH          6
#define BR_HIP_STOP_US     1375
#define BR_HIP_PREPARE_US  1680
#define BR_HIP_PUSH_US     1060

// ============================================================
// BASIC SERVO CONTROL
// ============================================================
void setServoUS(uint8_t ch, uint16_t pulseUS) {
  if (ch >= SERVO_COUNT) return;
  pca.setPWM(ch, 0, US_TO_TICK(pulseUS));
}

void stopAllHip() {
  setServoUS(FL_HIP_CH, FL_HIP_STOP_US);
  setServoUS(BL_HIP_CH, BL_HIP_STOP_US);
  setServoUS(FR_HIP_CH, FR_HIP_STOP_US);
  setServoUS(BR_HIP_CH, BR_HIP_STOP_US);

  Serial.println("[STOP] Semua HIP stop.");
}

void stopAllServo() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoUS(i, DEFAULT_STOP_US);
  }

  Serial.println("[STOP] Semua servo stop.");
}

// ============================================================
// RAMP HELPER
// ============================================================
void rampOneHipToTarget(uint8_t ch, uint16_t stopUS, uint16_t targetUS) {
  if (!USE_RAMP) {
    setServoUS(ch, targetUS);
    return;
  }

  for (int step = 1; step <= RAMP_STEPS; step++) {
    uint16_t value = stopUS + (((int16_t)targetUS - (int16_t)stopUS) * step) / RAMP_STEPS;
    setServoUS(ch, value);
    delay(RAMP_DELAY_MS);
  }
}

void rampOneHipToStop(uint8_t ch, uint16_t stopUS, uint16_t fromUS) {
  if (!USE_RAMP) {
    setServoUS(ch, stopUS);
    return;
  }

  for (int step = RAMP_STEPS; step >= 1; step--) {
    uint16_t value = stopUS + (((int16_t)fromUS - (int16_t)stopUS) * step) / RAMP_STEPS;
    setServoUS(ch, value);
    delay(RAMP_DELAY_MS);
  }

  setServoUS(ch, stopUS);
}

void rampTwoHipToTarget(
  uint8_t chA, uint16_t stopA, uint16_t targetA,
  uint8_t chB, uint16_t stopB, uint16_t targetB
) {
  if (!USE_RAMP) {
    setServoUS(chA, targetA);
    setServoUS(chB, targetB);
    return;
  }

  for (int step = 1; step <= RAMP_STEPS; step++) {
    uint16_t valueA = stopA + (((int16_t)targetA - (int16_t)stopA) * step) / RAMP_STEPS;
    uint16_t valueB = stopB + (((int16_t)targetB - (int16_t)stopB) * step) / RAMP_STEPS;

    setServoUS(chA, valueA);
    setServoUS(chB, valueB);

    delay(RAMP_DELAY_MS);
  }
}

void rampTwoHipToStop(
  uint8_t chA, uint16_t stopA, uint16_t fromA,
  uint8_t chB, uint16_t stopB, uint16_t fromB
) {
  if (!USE_RAMP) {
    setServoUS(chA, stopA);
    setServoUS(chB, stopB);
    return;
  }

  for (int step = RAMP_STEPS; step >= 1; step--) {
    uint16_t valueA = stopA + (((int16_t)fromA - (int16_t)stopA) * step) / RAMP_STEPS;
    uint16_t valueB = stopB + (((int16_t)fromB - (int16_t)stopB) * step) / RAMP_STEPS;

    setServoUS(chA, valueA);
    setServoUS(chB, valueB);

    delay(RAMP_DELAY_MS);
  }

  setServoUS(chA, stopA);
  setServoUS(chB, stopB);
}

void rampAllHipToTarget(
  uint16_t targetFL,
  uint16_t targetBL,
  uint16_t targetFR,
  uint16_t targetBR
) {
  if (!USE_RAMP) {
    setServoUS(FL_HIP_CH, targetFL);
    setServoUS(BL_HIP_CH, targetBL);
    setServoUS(FR_HIP_CH, targetFR);
    setServoUS(BR_HIP_CH, targetBR);
    return;
  }

  for (int step = 1; step <= RAMP_STEPS; step++) {
    uint16_t valueFL = FL_HIP_STOP_US + (((int16_t)targetFL - (int16_t)FL_HIP_STOP_US) * step) / RAMP_STEPS;
    uint16_t valueBL = BL_HIP_STOP_US + (((int16_t)targetBL - (int16_t)BL_HIP_STOP_US) * step) / RAMP_STEPS;
    uint16_t valueFR = FR_HIP_STOP_US + (((int16_t)targetFR - (int16_t)FR_HIP_STOP_US) * step) / RAMP_STEPS;
    uint16_t valueBR = BR_HIP_STOP_US + (((int16_t)targetBR - (int16_t)BR_HIP_STOP_US) * step) / RAMP_STEPS;

    setServoUS(FL_HIP_CH, valueFL);
    setServoUS(BL_HIP_CH, valueBL);
    setServoUS(FR_HIP_CH, valueFR);
    setServoUS(BR_HIP_CH, valueBR);

    delay(RAMP_DELAY_MS);
  }
}

void rampAllHipToStop(
  uint16_t fromFL,
  uint16_t fromBL,
  uint16_t fromFR,
  uint16_t fromBR
) {
  if (!USE_RAMP) {
    stopAllHip();
    return;
  }

  for (int step = RAMP_STEPS; step >= 1; step--) {
    uint16_t valueFL = FL_HIP_STOP_US + (((int16_t)fromFL - (int16_t)FL_HIP_STOP_US) * step) / RAMP_STEPS;
    uint16_t valueBL = BL_HIP_STOP_US + (((int16_t)fromBL - (int16_t)BL_HIP_STOP_US) * step) / RAMP_STEPS;
    uint16_t valueFR = FR_HIP_STOP_US + (((int16_t)fromFR - (int16_t)FR_HIP_STOP_US) * step) / RAMP_STEPS;
    uint16_t valueBR = BR_HIP_STOP_US + (((int16_t)fromBR - (int16_t)BR_HIP_STOP_US) * step) / RAMP_STEPS;

    setServoUS(FL_HIP_CH, valueFL);
    setServoUS(BL_HIP_CH, valueBL);
    setServoUS(FR_HIP_CH, valueFR);
    setServoUS(BR_HIP_CH, valueBR);

    delay(RAMP_DELAY_MS);
  }

  stopAllHip();
}

// ============================================================
// GERAK HIP SATU PER SATU
// ============================================================
void hipSingleMove(const char* nama, uint8_t ch, uint16_t stopUS, uint16_t targetUS, const char* mode) {
  Serial.println();
  Serial.print("[HIP ");
  Serial.print(mode);
  Serial.print(" SINGLE] ");
  Serial.println(nama);

  rampOneHipToTarget(ch, stopUS, targetUS);
  delay(TIME_HIP_PREPARE_SINGLE_MS);
  rampOneHipToStop(ch, stopUS, targetUS);

  Serial.print("[DONE] ");
  Serial.print(nama);
  Serial.print(" selesai ");
  Serial.println(mode);
}

// ============================================================
// PUSH DUA HIP BERSAMAAN
// ============================================================
void hipPushPair(
  const char* namaSisi,
  uint8_t chA, uint16_t stopA, uint16_t pushA,
  uint8_t chB, uint16_t stopB, uint16_t pushB
) {
  Serial.println();
  Serial.print("[HIP PUSH PAIR] ");
  Serial.println(namaSisi);

  rampTwoHipToTarget(chA, stopA, pushA, chB, stopB, pushB);
  delay(TIME_HIP_PUSH_PAIR_MS);
  rampTwoHipToStop(chA, stopA, pushA, chB, stopB, pushB);

  Serial.print("[DONE] Push ");
  Serial.print(namaSisi);
  Serial.println(" selesai.");
}

// ============================================================
// COMMAND MANUAL MAJU - SISI KIRI
// ============================================================

// 1 = depan kiri maju
void command1_DepanKiriMaju() {
  hipSingleMove(
    "Depan Kiri",
    FL_HIP_CH,
    FL_HIP_STOP_US,
    FL_HIP_PREPARE_US,
    "MAJU"
  );
}

// 2 = belakang kiri maju
void command2_BelakangKiriMaju() {
  hipSingleMove(
    "Belakang Kiri",
    BL_HIP_CH,
    BL_HIP_STOP_US,
    BL_HIP_PREPARE_US,
    "MAJU"
  );
}

// 3 = push kiri untuk maju
void command3_PushKiriMaju() {
  hipPushPair(
    "Kiri MAJU: Depan Kiri + Belakang Kiri",
    FL_HIP_CH,
    FL_HIP_STOP_US,
    FL_HIP_PUSH_US,
    BL_HIP_CH,
    BL_HIP_STOP_US,
    BL_HIP_PUSH_US
  );
}

// ============================================================
// COMMAND MANUAL MAJU - SISI KANAN
// ============================================================

// 4 = depan kanan maju
void command4_DepanKananMaju() {
  hipSingleMove(
    "Depan Kanan",
    FR_HIP_CH,
    FR_HIP_STOP_US,
    FR_HIP_PREPARE_US,
    "MAJU"
  );
}

// 5 = belakang kanan maju
void command5_BelakangKananMaju() {
  hipSingleMove(
    "Belakang Kanan",
    BR_HIP_CH,
    BR_HIP_STOP_US,
    BR_HIP_PREPARE_US,
    "MAJU"
  );
}

// 6 = push kanan untuk maju
void command6_PushKananMaju() {
  hipPushPair(
    "Kanan MAJU: Depan Kanan + Belakang Kanan",
    FR_HIP_CH,
    FR_HIP_STOP_US,
    FR_HIP_PUSH_US,
    BR_HIP_CH,
    BR_HIP_STOP_US,
    BR_HIP_PUSH_US
  );
}

// ============================================================
// PUSH SEMUA MAJU
// ============================================================
void command7_PushSemuaMaju() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("7 = PUSH SEMUA HIP UNTUK MAJU");
  Serial.println("======================================");

  rampAllHipToTarget(
    FL_HIP_PUSH_US,
    BL_HIP_PUSH_US,
    FR_HIP_PUSH_US,
    BR_HIP_PUSH_US
  );

  delay(TIME_HIP_PUSH_ALL_MS);

  rampAllHipToStop(
    FL_HIP_PUSH_US,
    BL_HIP_PUSH_US,
    FR_HIP_PUSH_US,
    BR_HIP_PUSH_US
  );

  Serial.println("[DONE] Push semua maju selesai.");
}

// ============================================================
// PUSH SEMUA MUNDUR
// Kebalikan dari push maju:
// target push mundur = prepare maju
// ============================================================
void command8_PushSemuaMundur() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("8 = PUSH SEMUA HIP UNTUK MUNDUR");
  Serial.println("======================================");

  rampAllHipToTarget(
    FL_HIP_PREPARE_US,
    BL_HIP_PREPARE_US,
    FR_HIP_PREPARE_US,
    BR_HIP_PREPARE_US
  );

  delay(TIME_HIP_PUSH_ALL_MS);

  rampAllHipToStop(
    FL_HIP_PREPARE_US,
    BL_HIP_PREPARE_US,
    FR_HIP_PREPARE_US,
    BR_HIP_PREPARE_US
  );

  Serial.println("[DONE] Push semua mundur selesai.");
}

// ============================================================
// GERAK MUNDUR PER KAKI
// Kebalikan dari prepare maju:
// target mundur = push maju
// ============================================================
void depanKiriMundur() {
  hipSingleMove("Depan Kiri", FL_HIP_CH, FL_HIP_STOP_US, FL_HIP_PUSH_US, "MUNDUR");
}

void belakangKiriMundur() {
  hipSingleMove("Belakang Kiri", BL_HIP_CH, BL_HIP_STOP_US, BL_HIP_PUSH_US, "MUNDUR");
}

void depanKananMundur() {
  hipSingleMove("Depan Kanan", FR_HIP_CH, FR_HIP_STOP_US, FR_HIP_PUSH_US, "MUNDUR");
}

void belakangKananMundur() {
  hipSingleMove("Belakang Kanan", BR_HIP_CH, BR_HIP_STOP_US, BR_HIP_PUSH_US, "MUNDUR");
}

// ============================================================
// BELOK KANAN
//
// Ketentuan:
// kaki kiri depan maju
// kaki kiri belakang maju
// push kiri
// ============================================================
void belokKanan() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("BELOK KANAN");
  Serial.println("Urutan: Depan kiri -> Belakang kiri -> Push kiri");
  Serial.println("======================================");

  command1_DepanKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  command2_BelakangKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  command3_PushKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  Serial.println("[DONE] Belok kanan selesai.");
}

// ============================================================
// BELOK KIRI
//
// Ketentuan:
// kaki kanan depan maju
// kaki kanan belakang maju
// push kanan
// ============================================================
void belokKiri() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("BELOK KIRI");
  Serial.println("Urutan: Depan kanan -> Belakang kanan -> Push kanan");
  Serial.println("======================================");

  command4_DepanKananMaju();
  delay(TIME_AUTO_GAP_MS);

  command5_BelakangKananMaju();
  delay(TIME_AUTO_GAP_MS);

  command6_PushKananMaju();
  delay(TIME_AUTO_GAP_MS);

  Serial.println("[DONE] Belok kiri selesai.");
}

// ============================================================
// MAJU OTOMATIS VERSI DEPAN DULU
//
// Ketentuan:
// kiri depan -> kanan depan -> kiri belakang -> kanan belakang -> push semua
// ============================================================
void majuOtomatisDepanDulu() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("MAJU OTOMATIS - DEPAN DULU");
  Serial.println("Urutan: FL -> FR -> BL -> BR -> Push semua");
  Serial.println("======================================");

  command1_DepanKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  command4_DepanKananMaju();
  delay(TIME_AUTO_GAP_MS);

  command2_BelakangKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  command5_BelakangKananMaju();
  delay(TIME_AUTO_GAP_MS);

  command7_PushSemuaMaju();
  delay(TIME_AUTO_GAP_MS);

  Serial.println("[DONE] Maju otomatis depan dulu selesai.");
}

// ============================================================
// MAJU OTOMATIS VERSI BELAKANG DULU
//
// Pendapatku: ini layak dites karena kaki belakang
// biasanya lebih berperan sebagai pendorong.
// ============================================================
void majuOtomatisBelakangDulu() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("MAJU OTOMATIS - BELAKANG DULU");
  Serial.println("Urutan: BL -> BR -> FL -> FR -> Push semua");
  Serial.println("======================================");

  command2_BelakangKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  command5_BelakangKananMaju();
  delay(TIME_AUTO_GAP_MS);

  command1_DepanKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  command4_DepanKananMaju();
  delay(TIME_AUTO_GAP_MS);

  command7_PushSemuaMaju();
  delay(TIME_AUTO_GAP_MS);

  Serial.println("[DONE] Maju otomatis belakang dulu selesai.");
}

// ============================================================
// MUNDUR OTOMATIS
//
// Kebalikan dari maju depan-dulu.
// Maju depan-dulu: FL -> FR -> BL -> BR -> push maju
// Mundur reverse : BR -> BL -> FR -> FL -> push mundur
// ============================================================
void mundurOtomatis() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("MUNDUR OTOMATIS");
  Serial.println("Urutan: BR -> BL -> FR -> FL -> Push semua mundur");
  Serial.println("======================================");

  belakangKananMundur();
  delay(TIME_AUTO_GAP_MS);

  belakangKiriMundur();
  delay(TIME_AUTO_GAP_MS);

  depanKananMundur();
  delay(TIME_AUTO_GAP_MS);

  depanKiriMundur();
  delay(TIME_AUTO_GAP_MS);

  command8_PushSemuaMundur();
  delay(TIME_AUTO_GAP_MS);

  Serial.println("[DONE] Mundur otomatis selesai.");
}

// ============================================================
// HELP
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("===== MODE HIP ONLY FULL BUILD =====");
  Serial.println("Knee tidak digerakkan di mode ini.");
  Serial.println();
  Serial.println("Manual maju:");
  Serial.println("1 = hip depan kiri maju");
  Serial.println("2 = hip belakang kiri maju");
  Serial.println("3 = push kiri maju");
  Serial.println("4 = hip depan kanan maju");
  Serial.println("5 = hip belakang kanan maju");
  Serial.println("6 = push kanan maju");
  Serial.println("7 = push semua maju");
  Serial.println("8 = push semua mundur");
  Serial.println();
  Serial.println("Otomatis:");
  Serial.println("r = belok kanan: FL -> BL -> push kiri");
  Serial.println("t = belok kiri : FR -> BR -> push kanan");
  Serial.println("m = maju otomatis depan dulu: FL -> FR -> BL -> BR -> push semua");
  Serial.println("n = maju otomatis belakang dulu: BL -> BR -> FL -> FR -> push semua");
  Serial.println("b = mundur otomatis kebalikan maju");
  Serial.println();
  Serial.println("x = stop semua hip");
  Serial.println("l = stop semua servo");
  Serial.println("h = help");
  Serial.println();
  Serial.println("PWM tuning:");
  Serial.println("MAJU kiri: prepare kecil, push besar");
  Serial.println("MAJU kanan: prepare besar, push kecil");
  Serial.println("MUNDUR = kebalikan dari maju");
  Serial.println("Netral = 1375");
  Serial.println("=====================================");
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

  stopAllServo();

  Serial.println();
  Serial.println("[BOOT] ESP32 + PCA9685 siap.");
  Serial.println("[BOOT] Mode hip-only full build aktif.");

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
      case '1':
        command1_DepanKiriMaju();
        break;

      case '2':
        command2_BelakangKiriMaju();
        break;

      case '3':
        command3_PushKiriMaju();
        break;

      case '4':
        command4_DepanKananMaju();
        break;

      case '5':
        command5_BelakangKananMaju();
        break;

      case '6':
        command6_PushKananMaju();
        break;

      case '7':
        command7_PushSemuaMaju();
        break;

      case '8':
        command8_PushSemuaMundur();
        break;

      case 'r':
        belokKanan();
        break;

      case 't':
        belokKiri();
        break;

      case 'm':
        majuOtomatisDepanDulu();
        break;

      case 'n':
        majuOtomatisBelakangDulu();
        break;

      case 'b':
        mundurOtomatis();
        break;

      case 'x':
        stopAllHip();
        break;

      case 'l':
        stopAllServo();
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