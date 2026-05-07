// ============================================================
//  Servo Continuous 360 - Kalibrasi Titik Netral
//  Hardware : ESP32 + PCA9685
//
//  Wiring PCA9685 -> ESP32:
//    VCC -> 3.3V | GND -> GND | SDA -> GPIO21 | SCL -> GPIO22
//
//  PERINTAH SERIAL (9600 baud):
//    +     -> naikkan pulse 10us  (makin jauh dari netral)
//    -     -> turunkan pulse 10us (makin dekat ke netral)
//    ++    -> naikkan pulse 50us (kasar)
//    --    -> turunkan pulse 50us (kasar)
//    0     -> langsung set ke 1500us (perkiraan netral)
//    val:1480 -> set langsung ke nilai tertentu
//    ch:0  -> pindah channel servo (0-7)
//    go    -> kirim pulse sekarang ke servo
//    save  -> tampilkan nilai netral yang ditemukan
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define US_TO_TICK(us) ((uint16_t)((us / 20000.0) * 4096))

uint16_t currentUS  = 1500;   // mulai dari perkiraan netral
uint8_t  currentCH  = 0;      // channel aktif
String   inputBuffer = "";

// ── Kirim pulse ke servo aktif ────────────────────────────
void sendPulse() {
  uint16_t tick = US_TO_TICK(currentUS);
  pca.setPWM(currentCH, 0, tick);
  Serial.printf("  CH%d | %dus (tick:%d) | ", currentCH, currentUS, tick);

  // Indikator visual posisi relatif ke 1500
  if (currentUS < 1480)       Serial.print("<<< MAJU");
  else if (currentUS > 1520)  Serial.print(">>> BALIK");
  else                         Serial.print("[ NETRAL? ]");

  Serial.println();
}

void printHelp() {
  Serial.println("\n===== KALIBRASI TITIK NETRAL SERVO =====");
  Serial.println("  +        -> +10us (naik)");
  Serial.println("  -        -> -10us (turun)");
  Serial.println("  ++       -> +50us (naik kasar)");
  Serial.println("  --       -> -50us (turun kasar)");
  Serial.println("  0        -> reset ke 1500us");
  Serial.println("  val:1490 -> set nilai langsung");
  Serial.println("  ch:0     -> ganti channel (0-7)");
  Serial.println("  go       -> kirim pulse ulang");
  Serial.println("  save     -> tampilkan hasil kalibrasi");
  Serial.println("  help     -> menu ini");
  Serial.println("");
  Serial.println("  TUJUAN: cari nilai dimana servo BERHENTI TOTAL");
  Serial.println("  Mulai dari 1500us, naik/turun sampai servo diam");
  Serial.println("=========================================\n");
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.printf("\n>> %s\n", cmd.c_str());

  if (cmd == "+") {
    currentUS += 10;
    sendPulse();
  }
  else if (cmd == "-") {
    currentUS -= 10;
    sendPulse();
  }
  else if (cmd == "++") {
    currentUS += 50;
    sendPulse();
  }
  else if (cmd == "--") {
    currentUS -= 50;
    sendPulse();
  }
  else if (cmd == "0") {
    currentUS = 1500;
    sendPulse();
  }
  else if (cmd == "go") {
    sendPulse();
  }
  else if (cmd == "help") {
    printHelp();
  }
  else if (cmd == "save") {
    Serial.println("\n===== HASIL KALIBRASI =====");
    Serial.printf("  Channel  : CH%d\n", currentCH);
    Serial.printf("  Titik netral ditemukan di : %dus\n", currentUS);
    Serial.println("");
    Serial.println("  Salin nilai ini ke kode utama:");
    Serial.printf("  #define PULSE_STOP  %d\n", currentUS);
    Serial.printf("  #define PULSE_MAJU  %d   // contoh: %d (pelan) atau %d (cepat)\n",
                  currentUS - 200, currentUS - 100, currentUS - 500);
    Serial.printf("  #define PULSE_BALIK %d   // contoh: %d (pelan) atau %d (cepat)\n",
                  currentUS + 200, currentUS + 100, currentUS + 500);
    Serial.println("===========================\n");
  }
  else if (cmd.startsWith("val:")) {
    int val = cmd.substring(4).toInt();
    if (val < 500 || val > 2500) {
      Serial.println("[!] Nilai harus antara 500 - 2500 us");
      return;
    }
    currentUS = val;
    sendPulse();
  }
  else if (cmd.startsWith("ch:")) {
    int ch = cmd.substring(3).toInt();
    if (ch < 0 || ch > 7) {
      Serial.println("[!] Channel harus 0-7");
      return;
    }
    // Stop channel lama dulu
    pca.setPWM(currentCH, 0, US_TO_TICK(1500));
    currentCH = ch;
    currentUS = 1500;
    Serial.printf("[CH] Pindah ke CH%d, reset ke 1500us\n", currentCH);
    sendPulse();
  }
  else {
    Serial.printf("[?] '%s' tidak dikenal. Ketik 'help'.\n", cmd.c_str());
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

  // Stop semua channel dulu
  for (uint8_t i = 0; i < 16; i++) {
    pca.setPWM(i, 0, US_TO_TICK(1500));
  }

  Serial.println("\n[BOOT] Kalibrasi Servo siap.");
  Serial.println("[BOOT] Semua channel diset 1500us (perkiraan netral).");
  Serial.println("[BOOT] Amati servo — kalau masih berputar, gunakan + atau - untuk mencari titik berhenti.");
  printHelp();

  // Langsung kirim pulse ke CH0
  sendPulse();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}
