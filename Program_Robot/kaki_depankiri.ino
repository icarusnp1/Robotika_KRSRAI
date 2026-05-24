#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ctype.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define SERVO_COUNT 8

#define US_TO_TICK(us) ((uint16_t)(((float)(us) / 20000.0) * 4096))

// ============================================================
// SETTING CHANNEL KAKI DEPAN KIRI
// CH GENAP = HIP
// CH GANJIL = KNEE
// ============================================================
#define HIP_CH   2
#define KNEE_CH  3

// ============================================================
// PWM HIP KAKI DEPAN KIRI
// Silakan tuning ulang jika perlu
// ============================================================
#define HIP_MAJU_US   1645
#define HIP_BALIK_US  1070
#define HIP_STOP_US   1375

// ============================================================
// PWM KNEE KAKI DEPAN KIRI
// KNEE_TURUN_US = menurunkan kaki
// KNEE_NAIK_US  = boost naik agar balik mentok
// ============================================================
#define KNEE_TURUN_US  1680
#define KNEE_NAIK_US   1110
#define KNEE_STOP_US   1375

#define DEFAULT_STOP_US 1375

// ============================================================
// DURASI GERAK
// ============================================================
#define DURASI_HIP_SWING_MS        400
#define DURASI_KNEE_TURUN_MS       300
#define DURASI_HIP_PUSH_MS         400
#define DURASI_RELEASE_MS          150
#define DURASI_KNEE_BOOST_NAIK_MS  180
#define JEDA_MS                    150

// ============================================================
// BASIC SERVO CONTROL
// ============================================================
void setServoUS(uint8_t ch, uint16_t pulseUS) {
  if (ch >= SERVO_COUNT) return;
  pca.setPWM(ch, 0, US_TO_TICK(pulseUS));
}

void stopHip() {
  setServoUS(HIP_CH, HIP_STOP_US);
}

void stopKnee() {
  setServoUS(KNEE_CH, KNEE_STOP_US);
}

void stopLeg() {
  stopHip();
  stopKnee();
}

void stopAll() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoUS(i, DEFAULT_STOP_US);
  }

  stopLeg();

  Serial.println("[STOP] Semua servo diberi sinyal stop.");
}

// ============================================================
// BOOST KNEE NAIK
// Dipakai di akhir gerakan agar knee benar-benar balik ke atas
// ============================================================
void boostKneeNaik() {
  Serial.println("5. Boost knee naik sebentar agar kembali mentok");

  setServoUS(KNEE_CH, KNEE_NAIK_US);
  delay(DURASI_KNEE_BOOST_NAIK_MS);

  stopKnee();
  delay(JEDA_MS);
}

// ============================================================
// PULSE TEST MANUAL
// ============================================================
void pulseServoUS(uint8_t ch, uint16_t gerakUS, uint16_t stopUS, uint16_t durasiMs) {
  setServoUS(ch, gerakUS);
  delay(durasiMs);
  setServoUS(ch, stopUS);
  delay(JEDA_MS);
}

// ============================================================
// LANGKAH MAJU - KAKI DEPAN KIRI
//
// Karena arah hip depan kiri terbalik:
// MAJU  = pakai HIP_BALIK_US saat swing
// PUSH  = pakai HIP_MAJU_US saat dorong/reset
//
// State:
// 0. Knee sudah naik otomatis
// 1. Hip swing ke depan
// 2. Knee turun dan ditahan
// 3. Hip dorong sambil knee tetap turun
// 4. Release knee
// 5. Boost knee naik
// ============================================================
void langkahMajuStateBaru() {
  Serial.println();
  Serial.println("[STEP] LANGKAH MAJU - DEPAN KIRI - STATE BARU");

  Serial.println("0. State awal: knee sudah naik otomatis");
  stopLeg();
  delay(JEDA_MS);

  Serial.println("1. Hip maju / swing ke depan");
  setServoUS(HIP_CH, HIP_BALIK_US);
  delay(DURASI_HIP_SWING_MS);
  stopHip();
  delay(JEDA_MS);

  Serial.println("2. Knee turun dan ditahan");
  setServoUS(KNEE_CH, KNEE_TURUN_US);
  delay(DURASI_KNEE_TURUN_MS);

  Serial.println("3. Knee tetap ditahan turun, hip balik/dorong");
  setServoUS(HIP_CH, HIP_MAJU_US);
  delay(DURASI_HIP_PUSH_MS);
  stopHip();
  delay(JEDA_MS);

  Serial.println("4. Release knee, knee akan otomatis naik");
  stopKnee();
  delay(DURASI_RELEASE_MS);

  boostKneeNaik();

  stopLeg();

  Serial.println("[DONE] Langkah maju selesai.");
}

// ============================================================
// LANGKAH MUNDUR - KAKI DEPAN KIRI
//
// Karena arah hip depan kiri terbalik:
// MUNDUR = pakai HIP_MAJU_US saat swing
// PUSH   = pakai HIP_BALIK_US saat dorong/reset
// ============================================================
void langkahMundurStateBaru() {
  Serial.println();
  Serial.println("[STEP] LANGKAH MUNDUR - DEPAN KIRI - STATE BARU");

  Serial.println("0. State awal: knee sudah naik otomatis");
  stopLeg();
  delay(JEDA_MS);

  Serial.println("1. Hip mundur / swing ke belakang");
  setServoUS(HIP_CH, HIP_MAJU_US);
  delay(DURASI_HIP_SWING_MS);
  stopHip();
  delay(JEDA_MS);

  Serial.println("2. Knee turun dan ditahan");
  setServoUS(KNEE_CH, KNEE_TURUN_US);
  delay(DURASI_KNEE_TURUN_MS);

  Serial.println("3. Knee tetap ditahan turun, hip maju/dorong");
  setServoUS(HIP_CH, HIP_BALIK_US);
  delay(DURASI_HIP_PUSH_MS);
  stopHip();
  delay(JEDA_MS);

  Serial.println("4. Release knee, knee akan otomatis naik");
  stopKnee();
  delay(DURASI_RELEASE_MS);

  boostKneeNaik();

  stopLeg();

  Serial.println("[DONE] Langkah mundur selesai.");
}

// ============================================================
// MANUAL TEST
// ============================================================
void testHipMaju() {
  Serial.println("[TEST] Hip MAJU_US");
  pulseServoUS(HIP_CH, HIP_MAJU_US, HIP_STOP_US, DURASI_HIP_SWING_MS);
}

void testHipBalik() {
  Serial.println("[TEST] Hip BALIK_US");
  pulseServoUS(HIP_CH, HIP_BALIK_US, HIP_STOP_US, DURASI_HIP_SWING_MS);
}

void holdKneeTurun() {
  Serial.println("[TEST] Knee turun dan DITAHAN. Tekan f untuk release.");
  setServoUS(KNEE_CH, KNEE_TURUN_US);
}

void releaseKnee() {
  Serial.println("[TEST] Knee release / stop.");
  stopKnee();
}

void testKneeNaikManual() {
  Serial.println("[TEST] Knee boost naik manual");
  pulseServoUS(KNEE_CH, KNEE_NAIK_US, KNEE_STOP_US, DURASI_KNEE_BOOST_NAIK_MS);
}

// ============================================================
// HELP
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("===== KALIBRASI KAKI DEPAN KIRI - STATE BARU =====");
  Serial.println("q = langkah maju state baru");
  Serial.println("w = langkah mundur state baru");
  Serial.println("l = stop semua servo");
  Serial.println();
  Serial.println("Manual test:");
  Serial.println("a = test HIP_MAJU_US");
  Serial.println("s = test HIP_BALIK_US");
  Serial.println("d = knee turun dan ditahan");
  Serial.println("f = release knee / stop knee");
  Serial.println("z = test boost knee naik");
  Serial.println("h = tampilkan help");
  Serial.println();
  Serial.println("Channel:");
  Serial.println("CH2 = HIP depan kiri");
  Serial.println("CH3 = KNEE depan kiri");
  Serial.println();
  Serial.println("State maju:");
  Serial.println("Hip swing depan -> knee turun tahan -> hip dorong -> release -> boost knee naik");
  Serial.println();
  Serial.println("Catatan:");
  Serial.println("Arah hip depan kiri sudah dibalik.");
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
  Serial.println("[BOOT] Mode kalibrasi kaki depan kiri aktif.");
  Serial.print("[INFO] HIP_CH  = CH");
  Serial.println(HIP_CH);
  Serial.print("[INFO] KNEE_CH = CH");
  Serial.println(KNEE_CH);

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
        langkahMajuStateBaru();
        break;

      case 'w':
        langkahMundurStateBaru();
        break;

      case 'l':
        stopAll();
        break;

      case 'a':
        testHipMaju();
        break;

      case 's':
        testHipBalik();
        break;

      case 'd':
        holdKneeTurun();
        break;

      case 'f':
        releaseKnee();
        break;

      case 'z':
        testKneeNaikManual();
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