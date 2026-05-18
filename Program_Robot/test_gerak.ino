#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define SERVO_COUNT 8

#define US_TO_TICK(us) ((uint16_t)((us / 20000.0) * 4096))

#define PULSE_MAJU  1550
#define PULSE_BALIK 1200
#define PULSE_STOP  1375

const uint16_t TICK_MAJU  = US_TO_TICK(PULSE_MAJU);
const uint16_t TICK_BALIK = US_TO_TICK(PULSE_BALIK);
const uint16_t TICK_STOP  = US_TO_TICK(PULSE_STOP);

bool arahMaju[SERVO_COUNT] = {
  true, true, true, true,
  true, true, true, true
};

void setServo(uint8_t ch, uint16_t tick) {
  if (ch >= SERVO_COUNT) return;
  pca.setPWM(ch, 0, tick);
}

void stopAll() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServo(i, TICK_STOP);
  }

  Serial.println("[STOP] Semua servo berhenti.");
}

void toggleServo(uint8_t ch) {
  if (ch >= SERVO_COUNT) return;

  if (arahMaju[ch]) {
    setServo(ch, TICK_MAJU);
    Serial.printf("[MOVE] CH%d MAJU\n", ch);
  } else {
    setServo(ch, TICK_BALIK);
    Serial.printf("[MOVE] CH%d MUNDUR\n", ch);
  }

  arahMaju[ch] = !arahMaju[ch];
}

void printHelp() {
  Serial.println();
  Serial.println("===== QUADRUPED TOGGLE CONTROL =====");
  Serial.println("q = CH0 toggle maju/mundur");
  Serial.println("w = CH1 toggle maju/mundur");
  Serial.println("e = CH2 toggle maju/mundur");
  Serial.println("r = CH3 toggle maju/mundur");
  Serial.println();
  Serial.println("a = CH4 toggle maju/mundur");
  Serial.println("s = CH5 toggle maju/mundur");
  Serial.println("d = CH6 toggle maju/mundur");
  Serial.println("f = CH7 toggle maju/mundur");
  Serial.println();
  Serial.println("l = STOP semua servo");
  Serial.println("====================================");
  Serial.println();
}

void processCommand(char cmd) {
  cmd = tolower(cmd);

  switch (cmd) {
    case 'q': toggleServo(0); break;
    case 'w': toggleServo(1); break;
    case 'e': toggleServo(2); break;
    case 'r': toggleServo(3); break;

    case 'a': toggleServo(4); break;
    case 's': toggleServo(5); break;
    case 'd': toggleServo(6); break;
    case 'f': toggleServo(7); break;

    case 'l':
      stopAll();
      break;

    case '\n':
    case '\r':
    case ' ':
      break;

    default:
      Serial.printf("[!] Command '%c' tidak dikenal.\n", cmd);
      break;
  }
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
  Serial.println("[BOOT] Mode toggle 1 tombol per servo aktif.");
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    processCommand(cmd);
  }
}