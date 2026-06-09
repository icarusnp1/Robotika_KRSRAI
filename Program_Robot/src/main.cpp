#define BLYNK_TEMPLATE_ID "ISI_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "ISI_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN "ISI_AUTH_TOKEN"

#define BLYNK_PRINT Serial

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <ctype.h>

// ============================================================
// WIFI / BLYNK
// ============================================================
char ssid[] = "ISI_WIFI";
char pass[] = "ISI_PASSWORD";

// ============================================================
// OBJECT
// ============================================================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
Servo capit;

// ============================================================
// SETTING DASAR
// ============================================================
#define SERVO_FREQ 50
#define SERVO_COUNT 8
#define DEFAULT_STOP_US 1375

#define US_TO_TICK(us) ((uint16_t)(((float)(us) / 20000.0) * 4096))

// ============================================================
// GRIPPER SETTING
// ============================================================
// Dari kode lama:
// capit.write(50) = capit / close
// capit.write(0)  = open
#define GRIPPER_PIN 18
#define GRIPPER_OPEN_ANGLE 0
#define GRIPPER_CLOSE_ANGLE 50

// Sensor ultrasonic
#define TRIG_PIN 5
#define ECHO_PIN 19

// Sensor warna TCS3200
#define S0 27
#define S1 14
#define S2 25
#define S3 33
#define SENSOR_OUT 26

// Kalibrasi warna dari kode lama
int redMin   = 15,  greenMin  = 18,  blueMin  = 12;
int redMax   = 200, greenMax  = 210, blueMax  = 190;

// Jarak deteksi
#define DETECT_DISTANCE_CM 15.0
#define GRIP_DISTANCE_CM   5.0

// Auto approach
#define SENSOR_INTERVAL_MS 300
#define AUTO_APPROACH_GAP_MS 500
#define MAX_AUTO_APPROACH_CYCLES 6

// ============================================================
// SETTING WAKTU KAKI
// ============================================================
#define TIME_HIP_PREPARE_SINGLE_MS 500
#define TIME_HIP_PUSH_PAIR_MS      500
#define TIME_HIP_PUSH_ALL_MS       500

#define TIME_JEDA_MS              200
#define TIME_AUTO_GAP_MS          300

#define USE_RAMP                  true
#define RAMP_STEPS                10
#define RAMP_DELAY_MS             30

// ============================================================
// PWM TUNING - HIP ONLY
// ============================================================
// STOP / NETRAL = 1375
//
// MAJU:
// KIRI  prepare = kecil, push = besar
// KANAN prepare = besar, push = kecil
//
// MUNDUR:
// Kebalikan dari maju.
// ============================================================

// DEPAN KIRI
#define FL_HIP_CH          2
#define FL_HIP_STOP_US     1375
#define FL_HIP_PREPARE_US  1030
#define FL_HIP_PUSH_US     1640

// BELAKANG KIRI
#define BL_HIP_CH          0
#define BL_HIP_STOP_US     1375
#define BL_HIP_PREPARE_US  1050
#define BL_HIP_PUSH_US     1675

// DEPAN KANAN
#define FR_HIP_CH          4
#define FR_HIP_STOP_US     1375
#define FR_HIP_PREPARE_US  1680
#define FR_HIP_PUSH_US     1030

// BELAKANG KANAN
#define BR_HIP_CH          6
#define BR_HIP_STOP_US     1375
#define BR_HIP_PREPARE_US  1680
#define BR_HIP_PUSH_US     1060

// ============================================================
// STATE
// ============================================================
bool requestMaju = false;
bool requestKanan = false;
bool requestKiri = false;
bool requestMundur = false;
bool requestStop = false;

bool movementBusy = false;
bool emergencyStop = false;

bool autoGripperEnabled = false;

float jarak = 999;
int red = 0, green = 0, blue = 0;
int rNorm = 0, gNorm = 0, bNorm = 0;
bool warnaMerah = false;

unsigned long lastSensorRead = 0;
unsigned long lastAutoApproach = 0;
int autoApproachCycles = 0;

enum AutoGripState {
  GRIPPER_CLOSED_IDLE,
  GRIPPER_OPEN_APPROACH,
  GRIPPER_OBJECT_GRIPPED
};

AutoGripState gripState = GRIPPER_CLOSED_IDLE;

// ============================================================
// FORWARD DECLARATION
// ============================================================
void stopAllHip();
void stopAllServo();
bool majuOtomatisDepanDulu();
bool belokKanan();
bool belokKiri();
bool mundurOtomatis();
void openGripper();
void closeGripper();
void updateStatus(const char* msg);
bool smartDelay(unsigned long ms);

// ============================================================
// BLYNK CONTROL
// ============================================================

BLYNK_WRITE(V0) {
  int tombol = param.asInt();

  if (tombol == 1) {
    closeGripper();
    gripState = GRIPPER_CLOSED_IDLE;
    updateStatus("Gripper manual: CLOSE");
  } else {
    openGripper();
    gripState = GRIPPER_OPEN_APPROACH;
    updateStatus("Gripper manual: OPEN");
  }
}

BLYNK_WRITE(V1) {
  if (param.asInt() == 1) requestMaju = true;
}

BLYNK_WRITE(V2) {
  if (param.asInt() == 1) requestKanan = true;
}

BLYNK_WRITE(V3) {
  if (param.asInt() == 1) requestKiri = true;
}

BLYNK_WRITE(V4) {
  if (param.asInt() == 1) requestMundur = true;
}

BLYNK_WRITE(V5) {
  if (param.asInt() == 1) {
    requestStop = true;
    emergencyStop = true;
  }
}

BLYNK_WRITE(V6) {
  autoGripperEnabled = param.asInt();

  if (autoGripperEnabled) {
    closeGripper();
    gripState = GRIPPER_CLOSED_IDLE;
    autoApproachCycles = 0;
    updateStatus("Auto gripper ON: gripper closed, ready detect");
  } else {
    updateStatus("Auto gripper OFF");
  }
}

// ============================================================
// SMART DELAY
// Supaya Blynk tetap hidup saat robot bergerak.
// ============================================================
bool smartDelay(unsigned long ms) {
  unsigned long start = millis();

  while (millis() - start < ms) {
    Blynk.run();

    if (emergencyStop || requestStop) {
      stopAllHip();
      updateStatus("Emergency stop during movement");
      return false;
    }

    delay(5);
  }

  return true;
}

void updateStatus(const char* msg) {
  Serial.println(msg);
  Blynk.virtualWrite(V7, msg);
}

// ============================================================
// GRIPPER CONTROL
// ============================================================
void openGripper() {
  capit.write(GRIPPER_OPEN_ANGLE);
  Blynk.virtualWrite(V0, 0);
  Serial.println("[GRIPPER] OPEN");
}

void closeGripper() {
  capit.write(GRIPPER_CLOSE_ANGLE);
  Blynk.virtualWrite(V0, 1);
  Serial.println("[GRIPPER] CLOSE");
}

// ============================================================
// SENSOR WARNA
// ============================================================
int bacaSampel(int s2State, int s3State, int jumlahSampel = 5) {
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);
  delay(10);

  long total = 0;
  int valid = 0;

  for (int i = 0; i < jumlahSampel; i++) {
    long val = pulseIn(SENSOR_OUT, LOW, 50000);

    if (val > 0) {
      total += val;
      valid++;
    }

    delay(5);
  }

  return (valid > 0) ? (total / valid) : 999;
}

int normalisasi(int nilai, int minVal, int maxVal) {
  nilai = constrain(nilai, minVal, maxVal);
  return map(nilai, minVal, maxVal, 255, 0);
}

void bacaUltrasonik() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long durasi = pulseIn(ECHO_PIN, HIGH, 30000);

  if (durasi <= 0) {
    jarak = 999;
  } else {
    jarak = durasi * 0.034 / 2;
  }
}

void bacaWarna() {
  red   = bacaSampel(LOW, LOW);
  green = bacaSampel(HIGH, HIGH);
  blue  = bacaSampel(LOW, HIGH);

  rNorm = normalisasi(red, redMin, redMax);
  gNorm = normalisasi(green, greenMin, greenMax);
  bNorm = normalisasi(blue, blueMin, blueMax);

  int selisihRG = rNorm - gNorm;
  int selisihRB = rNorm - bNorm;

  warnaMerah = false;

  if (rNorm > 120 && selisihRG > 40 && selisihRB > 40) {
    warnaMerah = true;
  }
}

void bacaSensorTarget() {
  bacaUltrasonik();
  bacaWarna();

  Serial.print("[SENSOR] Jarak=");
  Serial.print(jarak);
  Serial.print(" cm | R=");
  Serial.print(rNorm);
  Serial.print(" G=");
  Serial.print(gNorm);
  Serial.print(" B=");
  Serial.print(bNorm);
  Serial.print(" | Merah=");
  Serial.println(warnaMerah ? "YA" : "TIDAK");
}

bool objekTerdeteksiAwal() {
  return warnaMerah && jarak > 0 && jarak <= DETECT_DISTANCE_CM;
}

bool objekSiapDicengkeram() {
  return warnaMerah && jarak > 0 && jarak <= GRIP_DISTANCE_CM;
}

// ============================================================
// PCA / HIP CONTROL
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

  Serial.println("[STOP] Semua servo PCA stop.");
}

// ============================================================
// RAMP HELPER
// ============================================================
bool rampOneHipToTarget(uint8_t ch, uint16_t stopUS, uint16_t targetUS) {
  if (!USE_RAMP) {
    setServoUS(ch, targetUS);
    return true;
  }

  for (int step = 1; step <= RAMP_STEPS; step++) {
    uint16_t value = stopUS + (((int16_t)targetUS - (int16_t)stopUS) * step) / RAMP_STEPS;
    setServoUS(ch, value);

    if (!smartDelay(RAMP_DELAY_MS)) return false;
  }

  return true;
}

bool rampOneHipToStop(uint8_t ch, uint16_t stopUS, uint16_t fromUS) {
  if (!USE_RAMP) {
    setServoUS(ch, stopUS);
    return true;
  }

  for (int step = RAMP_STEPS; step >= 1; step--) {
    uint16_t value = stopUS + (((int16_t)fromUS - (int16_t)stopUS) * step) / RAMP_STEPS;
    setServoUS(ch, value);

    if (!smartDelay(RAMP_DELAY_MS)) return false;
  }

  setServoUS(ch, stopUS);
  return true;
}

bool rampTwoHipToTarget(
  uint8_t chA, uint16_t stopA, uint16_t targetA,
  uint8_t chB, uint16_t stopB, uint16_t targetB
) {
  if (!USE_RAMP) {
    setServoUS(chA, targetA);
    setServoUS(chB, targetB);
    return true;
  }

  for (int step = 1; step <= RAMP_STEPS; step++) {
    uint16_t valueA = stopA + (((int16_t)targetA - (int16_t)stopA) * step) / RAMP_STEPS;
    uint16_t valueB = stopB + (((int16_t)targetB - (int16_t)stopB) * step) / RAMP_STEPS;

    setServoUS(chA, valueA);
    setServoUS(chB, valueB);

    if (!smartDelay(RAMP_DELAY_MS)) return false;
  }

  return true;
}

bool rampTwoHipToStop(
  uint8_t chA, uint16_t stopA, uint16_t fromA,
  uint8_t chB, uint16_t stopB, uint16_t fromB
) {
  if (!USE_RAMP) {
    setServoUS(chA, stopA);
    setServoUS(chB, stopB);
    return true;
  }

  for (int step = RAMP_STEPS; step >= 1; step--) {
    uint16_t valueA = stopA + (((int16_t)fromA - (int16_t)stopA) * step) / RAMP_STEPS;
    uint16_t valueB = stopB + (((int16_t)fromB - (int16_t)stopB) * step) / RAMP_STEPS;

    setServoUS(chA, valueA);
    setServoUS(chB, valueB);

    if (!smartDelay(RAMP_DELAY_MS)) return false;
  }

  setServoUS(chA, stopA);
  setServoUS(chB, stopB);

  return true;
}

bool rampAllHipToTarget(
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
    return true;
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

    if (!smartDelay(RAMP_DELAY_MS)) return false;
  }

  return true;
}

bool rampAllHipToStop(
  uint16_t fromFL,
  uint16_t fromBL,
  uint16_t fromFR,
  uint16_t fromBR
) {
  if (!USE_RAMP) {
    stopAllHip();
    return true;
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

    if (!smartDelay(RAMP_DELAY_MS)) return false;
  }

  stopAllHip();
  return true;
}

// ============================================================
// GERAK HIP
// ============================================================
bool hipSingleMove(const char* nama, uint8_t ch, uint16_t stopUS, uint16_t targetUS, const char* mode) {
  Serial.println();
  Serial.print("[HIP ");
  Serial.print(mode);
  Serial.print(" SINGLE] ");
  Serial.println(nama);

  if (!rampOneHipToTarget(ch, stopUS, targetUS)) return false;
  if (!smartDelay(TIME_HIP_PREPARE_SINGLE_MS)) return false;
  if (!rampOneHipToStop(ch, stopUS, targetUS)) return false;

  Serial.print("[DONE] ");
  Serial.print(nama);
  Serial.print(" selesai ");
  Serial.println(mode);

  return true;
}

bool hipPushPair(
  const char* namaSisi,
  uint8_t chA, uint16_t stopA, uint16_t pushA,
  uint8_t chB, uint16_t stopB, uint16_t pushB
) {
  Serial.println();
  Serial.print("[HIP PUSH PAIR] ");
  Serial.println(namaSisi);

  if (!rampTwoHipToTarget(chA, stopA, pushA, chB, stopB, pushB)) return false;
  if (!smartDelay(TIME_HIP_PUSH_PAIR_MS)) return false;
  if (!rampTwoHipToStop(chA, stopA, pushA, chB, stopB, pushB)) return false;

  Serial.print("[DONE] Push ");
  Serial.print(namaSisi);
  Serial.println(" selesai.");

  return true;
}

// ============================================================
// COMMAND KAKI
// ============================================================
bool command1_DepanKiriMaju() {
  return hipSingleMove("Depan Kiri", FL_HIP_CH, FL_HIP_STOP_US, FL_HIP_PREPARE_US, "MAJU");
}

bool command2_BelakangKiriMaju() {
  return hipSingleMove("Belakang Kiri", BL_HIP_CH, BL_HIP_STOP_US, BL_HIP_PREPARE_US, "MAJU");
}

bool command3_PushKiriMaju() {
  return hipPushPair(
    "Kiri MAJU: Depan Kiri + Belakang Kiri",
    FL_HIP_CH, FL_HIP_STOP_US, FL_HIP_PUSH_US,
    BL_HIP_CH, BL_HIP_STOP_US, BL_HIP_PUSH_US
  );
}

bool command4_DepanKananMaju() {
  return hipSingleMove("Depan Kanan", FR_HIP_CH, FR_HIP_STOP_US, FR_HIP_PREPARE_US, "MAJU");
}

bool command5_BelakangKananMaju() {
  return hipSingleMove("Belakang Kanan", BR_HIP_CH, BR_HIP_STOP_US, BR_HIP_PREPARE_US, "MAJU");
}

bool command6_PushKananMaju() {
  return hipPushPair(
    "Kanan MAJU: Depan Kanan + Belakang Kanan",
    FR_HIP_CH, FR_HIP_STOP_US, FR_HIP_PUSH_US,
    BR_HIP_CH, BR_HIP_STOP_US, BR_HIP_PUSH_US
  );
}

bool command7_PushSemuaMaju() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("7 = PUSH SEMUA HIP UNTUK MAJU");
  Serial.println("======================================");

  if (!rampAllHipToTarget(FL_HIP_PUSH_US, BL_HIP_PUSH_US, FR_HIP_PUSH_US, BR_HIP_PUSH_US)) return false;
  if (!smartDelay(TIME_HIP_PUSH_ALL_MS)) return false;
  if (!rampAllHipToStop(FL_HIP_PUSH_US, BL_HIP_PUSH_US, FR_HIP_PUSH_US, BR_HIP_PUSH_US)) return false;

  Serial.println("[DONE] Push semua maju selesai.");
  return true;
}

bool command8_PushSemuaMundur() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("8 = PUSH SEMUA HIP UNTUK MUNDUR");
  Serial.println("======================================");

  if (!rampAllHipToTarget(FL_HIP_PREPARE_US, BL_HIP_PREPARE_US, FR_HIP_PREPARE_US, BR_HIP_PREPARE_US)) return false;
  if (!smartDelay(TIME_HIP_PUSH_ALL_MS)) return false;
  if (!rampAllHipToStop(FL_HIP_PREPARE_US, BL_HIP_PREPARE_US, FR_HIP_PREPARE_US, BR_HIP_PREPARE_US)) return false;

  Serial.println("[DONE] Push semua mundur selesai.");
  return true;
}

// ============================================================
// GERAK OTOMATIS
// ============================================================
bool belokKanan() {
  updateStatus("Belok kanan");
  if (!command1_DepanKiriMaju()) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;
  if (!command2_BelakangKiriMaju()) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;
  if (!command3_PushKiriMaju()) return false;
  return true;
}

bool belokKiri() {
  updateStatus("Belok kiri");
  if (!command4_DepanKananMaju()) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;
  if (!command5_BelakangKananMaju()) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;
  if (!command6_PushKananMaju()) return false;
  return true;
}

bool majuOtomatisDepanDulu() {
  updateStatus("Maju otomatis: depan dulu");

  if (!command1_DepanKiriMaju()) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;

  if (!command4_DepanKananMaju()) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;

  if (!command2_BelakangKiriMaju()) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;

  if (!command5_BelakangKananMaju()) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;

  if (!command7_PushSemuaMaju()) return false;

  updateStatus("Maju selesai");
  return true;
}

bool mundurOtomatis() {
  updateStatus("Mundur otomatis");

  if (!hipSingleMove("Belakang Kanan", BR_HIP_CH, BR_HIP_STOP_US, BR_HIP_PUSH_US, "MUNDUR")) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;

  if (!hipSingleMove("Belakang Kiri", BL_HIP_CH, BL_HIP_STOP_US, BL_HIP_PUSH_US, "MUNDUR")) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;

  if (!hipSingleMove("Depan Kanan", FR_HIP_CH, FR_HIP_STOP_US, FR_HIP_PUSH_US, "MUNDUR")) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;

  if (!hipSingleMove("Depan Kiri", FL_HIP_CH, FL_HIP_STOP_US, FL_HIP_PUSH_US, "MUNDUR")) return false;
  if (!smartDelay(TIME_AUTO_GAP_MS)) return false;

  if (!command8_PushSemuaMundur()) return false;

  updateStatus("Mundur selesai");
  return true;
}

// ============================================================
// RUN MOVEMENT WRAPPER
// ============================================================
void runMovement(bool (*movementFunc)(), const char* namaGerak) {
  if (movementBusy) return;

  emergencyStop = false;
  requestStop = false;
  movementBusy = true;

  updateStatus(namaGerak);

  bool sukses = movementFunc();

  stopAllHip();

  movementBusy = false;

  if (sukses) {
    updateStatus("Gerak selesai");
  } else {
    updateStatus("Gerak dihentikan");
  }
}

// ============================================================
// AUTO GRIPPER STATE MACHINE
// ============================================================
void handleAutoGripper() {
  if (!autoGripperEnabled) return;
  if (movementBusy) return;

  if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
    bacaSensorTarget();
    lastSensorRead = millis();
  }

  switch (gripState) {
    case GRIPPER_CLOSED_IDLE:
      // State awal: gripper harus tertutup.
      closeGripper();

      if (objekTerdeteksiAwal()) {
        stopAllHip();
        openGripper();

        gripState = GRIPPER_OPEN_APPROACH;
        autoApproachCycles = 0;
        lastAutoApproach = millis();

        updateStatus("Objek merah terdeteksi: gripper open");
      }
      break;

    case GRIPPER_OPEN_APPROACH:
      // Gripper tetap terbuka, robot maju sedikit demi sedikit.
      if (objekSiapDicengkeram()) {
        stopAllHip();
        closeGripper();

        gripState = GRIPPER_OBJECT_GRIPPED;
        autoGripperEnabled = false;
        Blynk.virtualWrite(V6, 0);

        updateStatus("Objek masuk gripper: CLOSE / GRIP");
        break;
      }

      if (autoApproachCycles >= MAX_AUTO_APPROACH_CYCLES) {
        stopAllHip();
        updateStatus("Auto approach max. Cek posisi objek.");
        break;
      }

      if (millis() - lastAutoApproach >= AUTO_APPROACH_GAP_MS) {
        updateStatus("Approach maju dengan gripper open");

        movementBusy = true;
        emergencyStop = false;
        requestStop = false;

        bool sukses = majuOtomatisDepanDulu();

        stopAllHip();
        movementBusy = false;

        autoApproachCycles++;
        lastAutoApproach = millis();

        if (!sukses) {
          updateStatus("Approach dihentikan");
        }
      }
      break;

    case GRIPPER_OBJECT_GRIPPED:
      // Objek sudah dicapit.
      stopAllHip();
      break;
  }
}

// ============================================================
// HANDLE COMMAND BLYNK
// ============================================================
void handleBlynkRequests() {
  if (requestStop) {
    requestStop = false;
    emergencyStop = true;
    stopAllHip();
    updateStatus("STOP dari Blynk");
    return;
  }

  if (movementBusy) return;

  if (requestMaju) {
    requestMaju = false;
    runMovement(majuOtomatisDepanDulu, "Blynk: Maju");
  }

  if (requestKanan) {
    requestKanan = false;
    runMovement(belokKanan, "Blynk: Belok kanan");
  }

  if (requestKiri) {
    requestKiri = false;
    runMovement(belokKiri, "Blynk: Belok kiri");
  }

  if (requestMundur) {
    requestMundur = false;
    runMovement(mundurOtomatis, "Blynk: Mundur");
  }
}

// ============================================================
// SERIAL COMMAND
// ============================================================
void handleSerialCommand() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  cmd = tolower(cmd);

  switch (cmd) {
    case 'm':
      runMovement(majuOtomatisDepanDulu, "Serial: Maju");
      break;

    case 'r':
      runMovement(belokKanan, "Serial: Belok kanan");
      break;

    case 't':
      runMovement(belokKiri, "Serial: Belok kiri");
      break;

    case 'b':
      runMovement(mundurOtomatis, "Serial: Mundur");
      break;

    case 'o':
      openGripper();
      updateStatus("Serial: Gripper open");
      break;

    case 'c':
      closeGripper();
      updateStatus("Serial: Gripper close");
      break;

    case 's':
      bacaSensorTarget();
      break;

    case 'x':
      stopAllHip();
      updateStatus("Serial: Stop hip");
      break;

    case 'l':
      emergencyStop = true;
      stopAllServo();
      updateStatus("Serial: Stop all PCA");
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

void printHelp() {
  Serial.println();
  Serial.println("===== ROBOT KAKI + GRIPPER + BLYNK =====");
  Serial.println("Serial:");
  Serial.println("m = maju");
  Serial.println("r = belok kanan");
  Serial.println("t = belok kiri");
  Serial.println("b = mundur");
  Serial.println("o = gripper open");
  Serial.println("c = gripper close");
  Serial.println("s = baca sensor");
  Serial.println("x = stop hip");
  Serial.println("l = stop all PCA");
  Serial.println();
  Serial.println("Blynk:");
  Serial.println("V0 = gripper manual");
  Serial.println("V1 = maju");
  Serial.println("V2 = belok kanan");
  Serial.println("V3 = belok kiri");
  Serial.println("V4 = mundur");
  Serial.println("V5 = stop");
  Serial.println("V6 = auto gripper");
  Serial.println("V7 = status");
  Serial.println("========================================");
  Serial.println();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);
  delay(100);

  stopAllServo();

  capit.attach(GRIPPER_PIN);
  closeGripper();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);

  // Frequency scaling TCS3200 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.println("[BOOT] Connecting Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Blynk.virtualWrite(V0, 1);
  Blynk.virtualWrite(V6, 0);
  Blynk.virtualWrite(V7, "System ready. Gripper closed.");

  gripState = GRIPPER_CLOSED_IDLE;

  Serial.println("[BOOT] System ready.");
  printHelp();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  Blynk.run();

  handleSerialCommand();
  handleBlynkRequests();
  handleAutoGripper();
}