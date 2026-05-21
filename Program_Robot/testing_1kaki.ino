#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define SERVO_COUNT 8

#define US_TO_TICK(us) ((uint16_t)((us / 20000.0) * 4096))

// ==========================
// PWM KHUSUS CH0 / HIP
// Silakan ubah sendiri
// ==========================
#define CH0_MAJU   1600
#define CH0_BALIK  1150
#define CH0_STOP   1375

// ==========================
// PWM KHUSUS CH1 / KNEE
// Silakan ubah sendiri
// ==========================
#define CH1_TURUN   1600
#define CH1_NAIK  1150
#define CH1_STOP   1375

const uint16_t TICK_CH0_MAJU  = US_TO_TICK(CH0_MAJU);
const uint16_t TICK_CH0_BALIK = US_TO_TICK(CH0_BALIK);
const uint16_t TICK_CH0_STOP  = US_TO_TICK(CH0_STOP);

const uint16_t TICK_CH1_NAIK  = US_TO_TICK(CH1_NAIK);
const uint16_t TICK_CH1_TURUN = US_TO_TICK(CH1_TURUN);
const uint16_t TICK_CH1_STOP  = US_TO_TICK(CH1_STOP);

// Kaki depan kiri
#define HIP_CH  0
#define KNEE_CH 1

// Durasi gerakan
#define DURASI_KNEE 300
#define DURASI_HIP  400
#define JEDA        150

void setServo(uint8_t ch, uint16_t tick) {
  if (ch >= SERVO_COUNT) return;
  pca.setPWM(ch, 0, tick);
}

void stopHip() {
  setServo(HIP_CH, TICK_CH0_STOP);
}

void stopKnee() {
  setServo(KNEE_CH, TICK_CH1_STOP);
}

void stopAll() {
  stopHip();
  stopKnee();

  for (uint8_t i = 2; i < SERVO_COUNT; i++) {
    pca.setPWM(i, 0, US_TO_TICK(1375));
  }

  Serial.println("[STOP] Semua servo berhenti.");
}

void gerakServo(uint8_t ch, uint16_t tickGerak, uint16_t tickStop, int durasi) {
  setServo(ch, tickGerak);
  delay(durasi);
  setServo(ch, tickStop);
  delay(JEDA);
}

// q = langkah maju
void langkahMajuSatuKaki() {
  Serial.println("[STEP] Langkah MAJU satu kaki");

  // 1. Knee naik
  Serial.println("1. CH1 Knee naik");
  gerakServo(KNEE_CH, TICK_CH1_NAIK, TICK_CH1_STOP, DURASI_KNEE);

  // 2. Hip maju
  Serial.println("2. CH0 Hip maju");
  gerakServo(HIP_CH, TICK_CH0_MAJU, TICK_CH0_STOP, DURASI_HIP);

  // 3. Knee turun
  Serial.println("3. CH1 Knee turun");
  gerakServo(KNEE_CH, TICK_CH1_TURUN, TICK_CH1_STOP, DURASI_KNEE);

  // 4. Hip balik/reset
  Serial.println("4. CH0 Hip balik/reset");
  gerakServo(HIP_CH, TICK_CH0_BALIK, TICK_CH0_STOP, DURASI_HIP);

  stopAll();
  Serial.println("[STEP] Langkah MAJU selesai.");
}

// w = langkah mundur
void langkahMundurSatuKaki() {
  Serial.println("[STEP] Langkah MUNDUR satu kaki");

  // 1. Knee naik
  Serial.println("1. CH1 Knee naik");
  gerakServo(KNEE_CH, TICK_CH1_NAIK, TICK_CH1_STOP, DURASI_KNEE);

  // 2. Hip mundur
  Serial.println("2. CH0 Hip mundur");
  gerakServo(HIP_CH, TICK_CH0_BALIK, TICK_CH0_STOP, DURASI_HIP);

  // 3. Knee turun
  Serial.println("3. CH1 Knee turun");
  gerakServo(KNEE_CH, TICK_CH1_TURUN, TICK_CH1_STOP, DURASI_KNEE);

  // 4. Hip balik/reset
  Serial.println("4. CH0 Hip balik/reset");
  gerakServo(HIP_CH, TICK_CH0_MAJU, TICK_CH0_STOP, DURASI_HIP);

  stopAll();
  Serial.println("[STEP] Langkah MUNDUR selesai.");
}

void printHelp() {
  Serial.println();
  Serial.println("===== TEST 1 KAKI PWM TERPISAH =====");
  Serial.println("q = langkah maju 1 kaki");
  Serial.println("w = langkah mundur 1 kaki");
  Serial.println("l = stop semua servo");
  Serial.println();
  Serial.println("CH0 = Hip");
  Serial.println("CH1 = Knee");
  Serial.println("===================================");
  Serial.println();
}

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
  Serial.println("[BOOT] Mode testing 1 kaki PWM terpisah aktif.");
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    cmd = tolower(cmd);

    if (cmd == 'q') {
      langkahMajuSatuKaki();
    } 
    else if (cmd == 'w') {
      langkahMundurSatuKaki();
    } 
    else if (cmd == 'l') {
      stopAll();
    }
  }
}