// ============================================================
//  Quadruped Robot - Multi Servo Pulse Mode (maju-stop-maju)
//  Hardware : ESP32 + PCA9685 + 8x Servo Continuous 360
//
//  Wiring PCA9685 -> ESP32:
//    VCC  -> 3.3V ESP32
//    GND  -> GND (common ground)
//    SDA  -> GPIO 21
//    SCL  -> GPIO 22
//    V+   -> 6V dari stepdown HW-131 (power servo)
//
//  Channel servo:
//    CH0  = Kaki Depan Kiri   - Hip
//    CH1  = Kaki Depan Kiri   - Knee
//    CH2  = Kaki Depan Kanan  - Hip
//    CH3  = Kaki Depan Kanan  - Knee
//    CH4  = Kaki Belakang Kiri  - Hip
//    CH5  = Kaki Belakang Kiri  - Knee
//    CH6  = Kaki Belakang Kanan - Hip
//    CH7  = Kaki Belakang Kanan - Knee
//
//  PERINTAH SERIAL (9600 baud, akhiri dengan Enter):
//
//  Pilih servo:
//    sel:0          -> pilih CH0 saja
//    sel:0,1        -> pilih CH0 dan CH1
//    sel:0,1,2,3    -> pilih beberapa sekaligus
//    sel:all        -> pilih semua 8 servo
//
//  Mulai pulse loop (maju-stop-maju-stop...):
//    go:f           -> pulse mode MAJU
//    go:b           -> pulse mode BALIK
//
//  Hentikan pulse loop:
//    go:s           -> stop servo terpilih + hentikan loop
//    stopall        -> stop semua servo + hentikan semua loop
//
//  Atur durasi (ms), berlaku untuk servo terpilih:
//    delay:gerak:500     -> durasi gerak 500ms
//    delay:jeda:500      -> durasi stop/jeda 500ms
//    delay:500           -> set keduanya sekaligus
//
//  Info:
//    status   -> lihat state & delay semua servo
//    help     -> tampilkan menu ini
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50

// tick = (us / 20000.0) * 4096 pada frekuensi 50Hz
#define US_TO_TICK(us) ((uint16_t)((us / 20000.0) * 4096))

#define PULSE_MAJU  700
#define PULSE_BALIK 1200
#define PULSE_STOP  1500

const uint16_t TICK_MAJU  = US_TO_TICK(PULSE_MAJU);
const uint16_t TICK_BALIK = US_TO_TICK(PULSE_BALIK);
const uint16_t TICK_STOP  = US_TO_TICK(PULSE_STOP);

#define SERVO_COUNT 8

// ── Per-servo state ───────────────────────────────────────
struct ServoCtrl {
  bool     selected;        // apakah dipilih user
  bool     pulsing;         // apakah sedang dalam pulse loop
  uint16_t moveTick;        // tick saat gerak (maju/balik)
  uint16_t delayGerak;      // durasi gerak (ms)
  uint16_t delayJeda;       // durasi stop/jeda (ms)
  uint32_t lastChange;      // millis() terakhir state berubah
  bool     isMoving;        // fase saat ini: true=gerak, false=jeda
};

ServoCtrl servo[SERVO_COUNT];

const char* servoName[SERVO_COUNT] = {
  "CH0 DepanKiri-Hip",
  "CH1 DepanKiri-Knee",
  "CH2 DepanKanan-Hip",
  "CH3 DepanKanan-Knee",
  "CH4 BlkgKiri-Hip",
  "CH5 BlkgKiri-Knee",
  "CH6 BlkgKanan-Hip",
  "CH7 BlkgKanan-Knee"
};

String inputBuffer = "";

// ============================================================
//  HELPER
// ============================================================
void setPWM(uint8_t ch, uint16_t tick) {
  pca.setPWM(ch, 0, tick);
}

const char* tickLabel(uint16_t tick) {
  if (tick == TICK_MAJU)  return "MAJU ";
  if (tick == TICK_BALIK) return "BALIK";
  return "STOP ";
}

// ============================================================
//  STOP SEMUA
// ============================================================
void stopAll() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servo[i].pulsing   = false;
    servo[i].isMoving  = false;
    setPWM(i, TICK_STOP);
  }
  Serial.println("[OK] Semua servo STOP, pulse loop dihentikan.");
}

// ============================================================
//  STATUS
// ============================================================
void printStatus() {
  Serial.println("\n--- STATUS SERVO ---");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    const char* fase = servo[i].pulsing
                       ? (servo[i].isMoving ? "GERAK" : "JEDA ")
                       : "     ";
    Serial.printf(
      "  %s | pulse:%s fase:%s | gerak:%dms jeda:%dms | %s\n",
      servoName[i],
      servo[i].pulsing  ? "ON " : "OFF",
      fase,
      servo[i].delayGerak,
      servo[i].delayJeda,
      servo[i].selected ? "[DIPILIH]" : ""
    );
  }
  Serial.println("--------------------\n");
}

// ============================================================
//  MENU
// ============================================================
void printHelp() {
  Serial.println("\n===== QUADRUPED PULSE-MODE CONTROLLER =====");
  Serial.println("  PILIH SERVO:");
  Serial.println("   sel:0          -> CH0 saja");
  Serial.println("   sel:0,1        -> CH0 & CH1");
  Serial.println("   sel:0,1,2,3    -> beberapa sekaligus");
  Serial.println("   sel:all        -> semua servo");
  Serial.println("  PULSE LOOP (maju-stop-maju-stop...):");
  Serial.println("   go:f  -> mulai pulse MAJU");
  Serial.println("   go:b  -> mulai pulse BALIK");
  Serial.println("   go:s  -> stop servo terpilih");
  Serial.println("  ATUR DURASI (untuk servo terpilih):");
  Serial.println("   delay:gerak:500  -> durasi gerak 500ms");
  Serial.println("   delay:jeda:500   -> durasi jeda  500ms");
  Serial.println("   delay:500        -> set keduanya 500ms");
  Serial.println("  LAINNYA:");
  Serial.println("   status   -> lihat status semua servo");
  Serial.println("   stopall  -> stop semua (darurat)");
  Serial.println("   help     -> menu ini");
  Serial.println("  CONTOH:");
  Serial.println("   sel:0,1  -> go:f               (CH0&CH1 pulse maju)");
  Serial.println("   sel:2,3  -> go:b               (CH2&CH3 pulse balik)");
  Serial.println("   sel:0,1  -> delay:gerak:300    (ubah durasi gerak)");
  Serial.println("   sel:all  -> go:s               (stop semua)");
  Serial.println("===========================================\n");
}

// ============================================================
//  PROSES sel:...
// ============================================================
void processSelect(String arg) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) servo[i].selected = false;
  arg.trim(); arg.toLowerCase();

  if (arg == "all") {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) servo[i].selected = true;
    Serial.println("[SEL] Semua servo dipilih.");
    return;
  }

  String token = "";
  int count = 0;
  for (int i = 0; i <= (int)arg.length(); i++) {
    char c = (i < (int)arg.length()) ? arg.charAt(i) : ',';
    if (c == ',') {
      token.trim();
      if (token.length() > 0) {
        int ch = token.toInt();
        if (ch >= 0 && ch < SERVO_COUNT) {
          servo[ch].selected = true;
          count++;
        } else {
          Serial.printf("[!] CH%d tidak valid (0-%d).\n", ch, SERVO_COUNT - 1);
        }
      }
      token = "";
    } else { token += c; }
  }

  if (count == 0) { Serial.println("[!] Tidak ada servo valid."); return; }

  Serial.print("[SEL] Terpilih: ");
  bool first = true;
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (servo[i].selected) {
      if (!first) Serial.print(", ");
      Serial.printf("CH%d", i);
      first = false;
    }
  }
  Serial.println();
}

// ============================================================
//  PROSES go:...
// ============================================================
void processGo(String arg) {
  arg.trim(); arg.toLowerCase();

  bool anySelected = false;
  for (uint8_t i = 0; i < SERVO_COUNT; i++)
    if (servo[i].selected) { anySelected = true; break; }

  if (!anySelected) {
    Serial.println("[!] Belum ada servo dipilih. Gunakan sel:... dulu.");
    return;
  }

  if (arg == "s") {
    // Stop servo terpilih saja
    Serial.print("[GO] STOP -> ");
    bool first = true;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      if (servo[i].selected) {
        servo[i].pulsing  = false;
        servo[i].isMoving = false;
        setPWM(i, TICK_STOP);
        if (!first) Serial.print(", ");
        Serial.printf("CH%d", i);
        first = false;
      }
    }
    Serial.println();
    return;
  }

  uint16_t moveTick;
  const char* label;
  if (arg == "f") {
    moveTick = TICK_MAJU;
    label = "pulse MAJU (700us)";
  } else if (arg == "b") {
    moveTick = TICK_BALIK;
    label = "pulse BALIK (1200us)";
  } else {
    Serial.printf("[!] Arah '%s' tidak dikenal. Gunakan f, b, atau s.\n", arg.c_str());
    return;
  }

  // Mulai pulse loop untuk servo terpilih
  Serial.printf("[GO] Mulai %s -> ", label);
  bool first = true;
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (servo[i].selected) {
      servo[i].pulsing    = true;
      servo[i].moveTick   = moveTick;
      servo[i].isMoving   = true;     // mulai dari fase GERAK
      servo[i].lastChange = millis();
      setPWM(i, moveTick);            // langsung gerak sekarang
      if (!first) Serial.print(", ");
      Serial.printf("CH%d", i);
      first = false;
    }
  }
  Serial.println(" [loop: gerak-stop-gerak-stop...]");
}

// ============================================================
//  PROSES delay:...
//  Format:
//    delay:500           -> set gerak & jeda keduanya
//    delay:gerak:500     -> hanya durasi gerak
//    delay:jeda:500      -> hanya durasi jeda
// ============================================================
void processDelay(String arg) {
  arg.trim(); arg.toLowerCase();

  bool anySelected = false;
  for (uint8_t i = 0; i < SERVO_COUNT; i++)
    if (servo[i].selected) { anySelected = true; break; }

  if (!anySelected) {
    Serial.println("[!] Pilih servo dulu dengan sel:...");
    return;
  }

  int secondColon = arg.indexOf(':');

  if (secondColon < 0) {
    // delay:500 → set keduanya
    uint16_t ms = (uint16_t)arg.toInt();
    if (ms < 100 || ms > 10000) {
      Serial.println("[!] Nilai delay harus 100-10000 ms.");
      return;
    }
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      if (servo[i].selected) {
        servo[i].delayGerak = ms;
        servo[i].delayJeda  = ms;
      }
    }
    Serial.printf("[DELAY] Gerak & jeda diset %dms untuk servo terpilih.\n", ms);
  } else {
    // delay:gerak:500 atau delay:jeda:500
    String tipe = arg.substring(0, secondColon);
    uint16_t ms = (uint16_t)arg.substring(secondColon + 1).toInt();

    if (ms < 100 || ms > 10000) {
      Serial.println("[!] Nilai delay harus 100-10000 ms.");
      return;
    }
    if (tipe == "gerak") {
      for (uint8_t i = 0; i < SERVO_COUNT; i++)
        if (servo[i].selected) servo[i].delayGerak = ms;
      Serial.printf("[DELAY] Durasi GERAK diset %dms.\n", ms);
    } else if (tipe == "jeda") {
      for (uint8_t i = 0; i < SERVO_COUNT; i++)
        if (servo[i].selected) servo[i].delayJeda = ms;
      Serial.printf("[DELAY] Durasi JEDA diset %dms.\n", ms);
    } else {
      Serial.printf("[!] Tipe '%s' tidak dikenal. Gunakan 'gerak' atau 'jeda'.\n", tipe.c_str());
    }
  }
}

// ============================================================
//  PROSES PERINTAH UTAMA
// ============================================================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.printf("\n>> %s\n", cmd.c_str());

  if (cmd == "status")  { printStatus(); return; }
  if (cmd == "stopall") { stopAll();     return; }
  if (cmd == "help")    { printHelp();   return; }

  int colonIdx = cmd.indexOf(':');
  if (colonIdx < 0) {
    Serial.printf("[?] '%s' tidak dikenal. Ketik 'help'.\n", cmd.c_str());
    return;
  }

  String prefix = cmd.substring(0, colonIdx);
  String arg    = cmd.substring(colonIdx + 1);
  prefix.trim(); prefix.toLowerCase();

  if      (prefix == "sel")   processSelect(arg);
  else if (prefix == "go")    processGo(arg);
  else if (prefix == "delay") processDelay(arg);
  else Serial.printf("[?] Prefix '%s' tidak dikenal. Ketik 'help'.\n", prefix.c_str());
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  delay(1000);

  Wire.begin(21, 22);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);
  delay(100);

  // Init semua servo
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servo[i].selected   = false;
    servo[i].pulsing    = false;
    servo[i].isMoving   = false;
    servo[i].moveTick   = TICK_MAJU;
    servo[i].delayGerak = 500;   // default 500ms
    servo[i].delayJeda  = 500;   // default 500ms
    servo[i].lastChange = 0;
    setPWM(i, TICK_STOP);
  }

  Serial.println("\n[BOOT] Quadruped Pulse-Mode Controller siap.");
  Serial.println("[BOOT] Semua servo STOP. Default delay gerak & jeda: 500ms.");
  printHelp();
}

// ============================================================
//  LOOP — Pulse engine non-blocking (pakai millis)
// ============================================================
void loop() {
  // ── Baca Serial ────────────────────────────────────────
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

  // ── Pulse engine non-blocking ──────────────────────────
  // Setiap servo yang pulsing=true diatur mandiri tanpa delay()
  uint32_t now = millis();
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (!servo[i].pulsing) continue;

    uint16_t durasi = servo[i].isMoving
                      ? servo[i].delayGerak
                      : servo[i].delayJeda;

    if (now - servo[i].lastChange >= durasi) {
      servo[i].isMoving   = !servo[i].isMoving;   // toggle fase
      servo[i].lastChange = now;

      if (servo[i].isMoving) {
        setPWM(i, servo[i].moveTick);
        Serial.printf("[PULSE] CH%d GERAK\n", i);
      } else {
        setPWM(i, TICK_STOP);
        Serial.printf("[PULSE] CH%d JEDA\n", i);
      }
    }
  }
}
