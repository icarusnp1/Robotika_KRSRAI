#define BLYNK_TEMPLATE_ID "TMPL6gX4G1lcA"
#define BLYNK_TEMPLATE_NAME "Gripper"
#define BLYNK_AUTH_TOKEN "kFATcuwqIqXOg0nexab83ueXmVk7auHI"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

char ssid[] = "NHP";
char pass[] = "permana3";

Servo capit;
int pinServo = 18;

// Ultrasonik
const int trigPin = 5;
const int echoPin = 19;
long durasi;
float jarak;

// Sensor Warna TCS3200
#define S0 27
#define S1 14
#define S2 25
#define S3 33
#define sensorOut 26

int red = 0, green = 0, blue = 0;

//variabel global
bool pauseDeteksi = false;
unsigned long waktuOff = 0;

// Nilai Kalibrasi (isi setelah kalibrasi)
// Putih (nilai pulse minimum tiap channel saat objek putih)
int redMin   = 15,  greenMin  = 18,  blueMin  = 12;
// Hitam (nilai pulse maksimum tiap channel saat objek hitam)
int redMax   = 200, greenMax  = 210, blueMax  = 190;

// Baca rata-rata beberapa sampel untuk stabilitas
int bacaSampel(int s2State, int s3State, int jumlahSampel = 5) {
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);
  delay(10); // biarkan filter stabil
  
  long total = 0;
  int valid  = 0;
  for (int i = 0; i < jumlahSampel; i++) {
    long val = pulseIn(sensorOut, LOW, 50000);
    if (val > 0) {
      total += val;
      valid++;
    }
    delay(5);
  }
  return (valid > 0) ? (total / valid) : 999;
}

// Map ke 0-255 (255 = warna paling kuat)
int normalisasi(int nilai, int minVal, int maxVal) {
  nilai = constrain(nilai, minVal, maxVal);
  // Pulse kecil = intensitas tinggi, maka dibalik
  return map(nilai, minVal, maxVal, 255, 0);
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  capit.attach(pinServo);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Frequency scaling 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  capit.write(0);
  Serial.println("System Ready");
  Serial.println("Jalankan kalibrasi: arahkan sensor ke PUTIH lalu HITAM");
}

BLYNK_WRITE(V0) {
  int tombol = param.asInt();

  if (tombol == 1) {
    capit.write(50);
    Serial.println("BLYNK ON -> Servo Capit");
  } else {
    capit.write(0);
    Serial.println("BLYNK OFF -> Servo Open");

    pauseDeteksi = true;
    waktuOff = millis();
  }
}

void loop() {
  Blynk.run();

  // TUNGGU 5 DETIK SETELAH TOMBOL OFF
if (pauseDeteksi) {
  if (millis() - waktuOff < 5000) {
    Serial.print("Deteksi aktif dalam ");
    Serial.print((5000 - (millis() - waktuOff)) / 1000.0);
    Serial.println(" detik");

    delay(200);
    return;
  } else {
    pauseDeteksi = false;
    Serial.println("Deteksi aktif kembali");
  }
}

  // BACA ULTRASONIK
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  durasi = pulseIn(echoPin, HIGH, 30000);
  jarak  = durasi * 0.034 / 2;

  Serial.print("Jarak: ");
  Serial.print(jarak);
  Serial.println(" cm");

  // BACA WARNA (rata-rata 5 sampel)
  red   = bacaSampel(LOW,  LOW);   // RED filter
  green = bacaSampel(HIGH, HIGH);  // GREEN filter
  blue  = bacaSampel(LOW,  HIGH);  // BLUE filter

  // NORMALISASI ke 0-255
  int rNorm = normalisasi(red,   redMin,   redMax);
  int gNorm = normalisasi(green, greenMin, greenMax);
  int bNorm = normalisasi(blue,  blueMin,  blueMax);

  Serial.print("Raw  R="); Serial.print(red);
  Serial.print(" G=");     Serial.print(green);
  Serial.print(" B=");     Serial.println(blue);

  Serial.print("Norm R="); Serial.print(rNorm);
  Serial.print(" G=");     Serial.print(gNorm);
  Serial.print(" B=");     Serial.println(bNorm);

  // DETEKSI MERAH
  // Merah dominan: R jauh lebih besar dari G dan B
  bool warnaMerah = false;
  int selisihRG   = rNorm - gNorm;
  int selisihRB   = rNorm - bNorm;

  if (rNorm > 120 &&        // R cukup kuat
      selisihRG > 40 &&     // R lebih besar dari G minimal 40
      selisihRB > 40) {     // R lebih besar dari B minimal 40
    warnaMerah = true;
    Serial.println(">> WARNA MERAH TERDETEKSI");
  } else {
    Serial.println("-- Bukan merah");
    Serial.print("   rNorm="); Serial.print(rNorm);
    Serial.print(" selisihRG="); Serial.print(selisihRG);
    Serial.print(" selisihRB="); Serial.println(selisihRB);
  }

  // AUTO CAPIT 
  if (jarak > 0 && jarak <= 10 && warnaMerah) {
    capit.write(50);
    Serial.println("MERAH + OBJEK DEKAT -> Servo Capit");
    Blynk.virtualWrite(V0, 1);
  }

  delay(200);
}
