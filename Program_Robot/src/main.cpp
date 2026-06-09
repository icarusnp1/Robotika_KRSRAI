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

// Lama masing-masing hip maju satu per satu
#define TIME_HIP_PREPARE_SINGLE_MS 500

// Lama push dua hip bersamaan
#define TIME_HIP_PUSH_PAIR_MS      500

// Jeda antar gerakan manual/otomatis
#define TIME_JEDA_MS              200
#define TIME_AUTO_GAP_MS          300

// Ramp agar gerakan tidak terlalu menghentak
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
// KIRI:
// prepare maju = angka kecil
// push maju    = angka besar
//
// KANAN:
// prepare maju = angka besar
// push maju    = angka kecil
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
#define BL_HIP_PREPARE_US  1020
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

// ============================================================
// GERAK HIP MAJU SATU PER SATU
// ============================================================
void hipPrepareSingle(const char* nama, uint8_t ch, uint16_t stopUS, uint16_t prepareUS) {
  Serial.println();
  Serial.print("[HIP PREPARE SINGLE] ");
  Serial.println(nama);

  rampOneHipToTarget(ch, stopUS, prepareUS);
  delay(TIME_HIP_PREPARE_SINGLE_MS);
  rampOneHipToStop(ch, stopUS, prepareUS);

  Serial.print("[DONE] ");
  Serial.print(nama);
  Serial.println(" selesai maju.");
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
// COMMAND 1, 2, 3 - SISI KIRI
// ============================================================

// 1 = kaki depan kiri maju
void command1_DepanKiriMaju() {
  hipPrepareSingle(
    "Depan Kiri",
    FL_HIP_CH,
    FL_HIP_STOP_US,
    FL_HIP_PREPARE_US
  );
}

// 2 = kaki belakang kiri maju
void command2_BelakangKiriMaju() {
  hipPrepareSingle(
    "Belakang Kiri",
    BL_HIP_CH,
    BL_HIP_STOP_US,
    BL_HIP_PREPARE_US
  );
}

// 3 = push kiri bersamaan
void command3_PushKiri() {
  hipPushPair(
    "Kiri: Depan Kiri + Belakang Kiri",
    FL_HIP_CH,
    FL_HIP_STOP_US,
    FL_HIP_PUSH_US,
    BL_HIP_CH,
    BL_HIP_STOP_US,
    BL_HIP_PUSH_US
  );
}

// ============================================================
// COMMAND 4, 5, 6 - SISI KANAN
// ============================================================

// 4 = kaki depan kanan maju
void command4_DepanKananMaju() {
  hipPrepareSingle(
    "Depan Kanan",
    FR_HIP_CH,
    FR_HIP_STOP_US,
    FR_HIP_PREPARE_US
  );
}

// 5 = kaki belakang kanan maju
void command5_BelakangKananMaju() {
  hipPrepareSingle(
    "Belakang Kanan",
    BR_HIP_CH,
    BR_HIP_STOP_US,
    BR_HIP_PREPARE_US
  );
}

// 6 = push kanan bersamaan
void command6_PushKanan() {
  hipPushPair(
    "Kanan: Depan Kanan + Belakang Kanan",
    FR_HIP_CH,
    FR_HIP_STOP_US,
    FR_HIP_PUSH_US,
    BR_HIP_CH,
    BR_HIP_STOP_US,
    BR_HIP_PUSH_US
  );
}

// ============================================================
// OTOMATIS MAJU
//
// Urutan:
// 1. Depan kiri maju
// 2. Belakang kiri maju
// 3. Push kiri bersamaan
// 4. Depan kanan maju
// 5. Belakang kanan maju
// 6. Push kanan bersamaan
// ============================================================
void majuOtomatis() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("MAJU OTOMATIS HIP ONLY");
  Serial.println("Urutan: 1 -> 2 -> 3 -> 4 -> 5 -> 6");
  Serial.println("======================================");

  command1_DepanKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  command2_BelakangKiriMaju();
  delay(TIME_AUTO_GAP_MS);

  command3_PushKiri();
  delay(TIME_AUTO_GAP_MS);

  command4_DepanKananMaju();
  delay(TIME_AUTO_GAP_MS);

  command5_BelakangKananMaju();
  delay(TIME_AUTO_GAP_MS);

  command6_PushKanan();
  delay(TIME_AUTO_GAP_MS);

  Serial.println("[DONE] Maju otomatis selesai.");
}

// ============================================================
// HELP
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("===== MODE HIP ONLY: MAJU SATU-SATU, PUSH BERSAMA =====");
  Serial.println("Knee tidak digerakkan di mode ini.");
  Serial.println();
  Serial.println("Sisi kiri:");
  Serial.println("1 = hip depan kiri maju");
  Serial.println("2 = hip belakang kiri maju");
  Serial.println("3 = push kiri bersamaan");
  Serial.println();
  Serial.println("Sisi kanan:");
  Serial.println("4 = hip depan kanan maju");
  Serial.println("5 = hip belakang kanan maju");
  Serial.println("6 = push kanan bersamaan");
  Serial.println();
  Serial.println("m = otomatis: 1 -> 2 -> 3 -> 4 -> 5 -> 6");
  Serial.println("x = stop semua hip");
  Serial.println("l = stop semua servo");
  Serial.println("h = help");
  Serial.println();
  Serial.println("PWM tuning:");
  Serial.println("Kiri  prepare kecil, push besar");
  Serial.println("Kanan prepare besar, push kecil");
  Serial.println("Netral = 1375");
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

  stopAllServo();

  Serial.println();
  Serial.println("[BOOT] ESP32 + PCA9685 siap.");
  Serial.println("[BOOT] Mode hip-only satu-satu + push bersama aktif.");

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
        command3_PushKiri();
        break;

      case '4':
        command4_DepanKananMaju();
        break;

      case '5':
        command5_BelakangKananMaju();
        break;

      case '6':
        command6_PushKanan();
        break;

      case 'm':
        majuOtomatis();
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