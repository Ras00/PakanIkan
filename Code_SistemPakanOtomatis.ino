#include "arduino_secrets.h"
#include "arduino_secrets.h"
#include "thingProperties.h"
#include <ESP32Servo.h>

// ---------------------- SERVO -----------------------
Servo myServo;

// Flags dan variabel status
bool prevPakanHabis = false;
bool prevAirSangatKeruh = false;

// Untuk manual
unsigned long tSwitchON = 0;
unsigned long tServoExecuted = 0;
bool menungguDelay = false;

// Untuk otomatis / schedule
bool feedingActive = false;
unsigned long waktuServoTerbuka_ms = 0;
unsigned long waktuServoTertutup_ms = 0;

String waktuJadwalString = "";  
int feedingIndex = 1;

// Timezone WIB = UTC+7
const unsigned long TIMEZONE_OFFSET = 7UL * 3600UL;

// Sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 14
#define TURBIDITY_PIN 34

// ===============================================================
//    KONVERSI UNIX TIME → HH:MM:SS (Dengan Timezone WIB FIXED)
// ===============================================================
String formatTimeWIB(unsigned long unixTimeUTC) {
  unsigned long localUnix = unixTimeUTC + TIMEZONE_OFFSET;

  int hours = (localUnix % 86400L) / 3600;
  int minutes = (localUnix % 3600) / 60;
  int seconds = localUnix % 60;

  char buffer[20];
  sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
  return String(buffer);
}

// ===============================================================
//                         SETUP
// ===============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  myServo.attach(18);
  myServo.write(0);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TURBIDITY_PIN, INPUT);

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  Serial.println("Sistem Siap — RTC IoT Cloud aktif!");
}

// ===============================================================
//                           LOOP
// ===============================================================
void loop() {

  ArduinoCloud.update();

  bacaJarakPakan();
  bacaKekeruhanAir();
  kontrolServo();

  delay(500);
}

// ===============================================================
//                   SENSOR ULTRASONIK
// ===============================================================
// ===============================================================
//        KONVERSI JARAK ULTRASONIK → PERSENTASE PAKAN
// ===============================================================
// ===============================================================
//        KONVERSI JARAK ULTRASONIK → PERSENTASE PAKAN (DIBALIK)
// ===============================================================
float hitungPersentasePakan(float jarak_cm) {
  const float JARAK_MAKS = 35.0;

  if (jarak_cm <= 0) return 100.0;        // pakan penuh
  if (jarak_cm >= JARAK_MAKS) return 0.0; // pakan habis

  float persen = (1.0 - (jarak_cm / JARAK_MAKS)) * 100.0;
  return constrain(persen, 0.0, 100.0);
}

void bacaJarakPakan() {
  if (feedingActive) return;   // cegah update saat feeding

  long duration;
  float jarak_cm = -1;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    statusPakan = "Sensor error";
    jarak_cm = -1;
    pakanHabis = false;
    persenPakan = 0;
  } 
  else {
    jarak_cm = duration * 0.034 / 2.0;
    distance = jarak_cm;

    // ================= PERSENTASE PAKAN =================
    float persentasePakan = hitungPersentasePakan(jarak_cm);
    persenPakan = persentasePakan;

    // ================= STATUS LOGIS =====================
    if (persentasePakan <= 20.0) {
      statusPakan = "Pakan habis";
      pakanHabis = true;
    }
    else if (persentasePakan <= 50.0) {
      statusPakan = "Pakan hampir habis";
      pakanHabis = false;
    }
    else {
      statusPakan = "Pakan tersedia";
      pakanHabis = false;
    }
  }

  Serial.print("Jarak: ");
  Serial.print(jarak_cm);
  Serial.print(" cm | Persentase: ");
  Serial.print(persenPakan);
  Serial.print(" % | Status: ");
  Serial.println(statusPakan);
}

// ===============================================================
//                   SENSOR KEKERUHAN AIR
// ===============================================================
float linmap(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

void bacaKekeruhanAir() {
   if (feedingActive) return;   // <<< MENCEGAH PRINT SAAT FEEDING

  int turbidityValue = analogRead(TURBIDITY_PIN);
  float kal = (turbidityValue - 2717.6f) / -18.189f;
  float mappedNTU;

  if (kal < 20.0f) {
    mappedNTU = linmap(kal, -75.73f, 20.0f, 1.0f, 20.0f);
    mappedNTU = constrain(mappedNTU, 1.0f, 20.0f);
  } else {
    mappedNTU = kal;
  }

  ntuValue = mappedNTU;

  if (mappedNTU < 20.0f) statusAir = "Air Jernih";
  else if (mappedNTU <= 50.0f) statusAir = "Mulai Keruh";
  else statusAir = "Air Sangat Keruh";

  airSangatKeruh = (statusAir == "Air Sangat Keruh");

  Serial.print("NTU: ");
  Serial.print(mappedNTU);
  Serial.print(" | Status: ");
  Serial.println(statusAir);
}

// ===============================================================
//                     KONTROL SERVO
// ===============================================================
void kontrolServo() {

  unsigned long nowUnix = getTime();               // UTC
  String nowStr = formatTimeWIB(nowUnix);          // ➜ WIB FIX

  // Jika air sangat keruh → servo dikunci
  if (airSangatKeruh) {
    Serial.println("🚫 Servo dikunci! Air keruh.");
    myServo.write(3);
    statusServo = "Terkunci (Air Keruh)";
    return;
  }

  // ---------------------------------------------------------------
  //                        MODE OTOMATIS
  // ---------------------------------------------------------------
  if (mode) {

    // Jika jadwal sedang aktif
    if (servoSchedule.isActive()) {
      
      // Baru masuk jadwal
      if (!feedingActive) {
        feedingActive = true;

        waktuServoTerbuka_ms = millis();
        waktuJadwalString = formatTimeWIB(getTime());  // ➜ WIB FIX

        Serial.println("\n========== FEEDING #" + String(feedingIndex) + " ==========");
        Serial.println("⏰ Jadwal Aktif Pada: " + waktuJadwalString);
        Serial.println("▶ Servo BUKA!");
        
      }

      myServo.write(25);  // servo open
    }

    // Jadwal selesai → tutup servo
    else {

      if (feedingActive) {

        feedingActive = false;
        waktuServoTertutup_ms = millis();

        String waktuTutupReal = formatTimeWIB(getTime());   // ➜ WIB FIX
        unsigned long durasi_ms = waktuServoTertutup_ms - waktuServoTerbuka_ms;

        Serial.println("⏹ Servo TUTUP Pada: " + waktuTutupReal);
        Serial.print("⏱ Durasi Servo Terbuka: ");
        Serial.print(durasi_ms / 1000.0);
        Serial.println(" detik");

        Serial.println("====================================\n");
        feedingIndex++;
      }

      myServo.write(3);  // close
    }

    return;
  }

  // ---------------------------------------------------------------
  //                        MODE MANUAL
  // ---------------------------------------------------------------
  if (servoSwitch) {
    myServo.write(25);

    if (menungguDelay) {
      tServoExecuted = micros();
      unsigned long delay_us = tServoExecuted - tSwitchON;

      Serial.print("⏱ Delay respon manual: ");
      Serial.print(delay_us);
      Serial.println(" us");

      menungguDelay = false;
    }
  }

  else {
    myServo.write(3);
  }
}

// ===============================================================
//                           CALLBACKS
// ===============================================================
void onModeChange() {}

void onServoSwitchChange() {
  if (servoSwitch) {
    tSwitchON = micros();
    menungguDelay = true;
  }
}

void onServoScheduleChange() {}
void onDistanceChange() {}
void onStatusPakanChange() {}
void onStatusPakan1Change() {}
void onPakanHabisChange() {}

void onAirSangatKeruhChange() {
  if (airSangatKeruh) {
    myServo.write(3);
    statusServo = "Terkunci (Air Keruh)";
  }
}

/*
  Since ServoSchedule2 is READ_WRITE variable, onServoSchedule2Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onServoSchedule2Change()  {
  // Add your code here to act upon ServoSchedule2 change
}