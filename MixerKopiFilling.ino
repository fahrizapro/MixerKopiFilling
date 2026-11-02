// ================================================================
// Mixer Kopi + Filling - Arduino Uno + BTS7960   (VERSI HEMAT FLASH)
// Encoder D2 | PWM D9 | Relay D13 | HC-SR04 TRIG D11 ECHO D12
// TCS3200 S0=D3 S1=D4 S2=D5 S3=D6 OUT=A0 | LCD 20x4 + Keypad via I2C
//
// PENGADUK = TIMER MURNI. Sensor warna hanya monitoring homogenisasi.
//
// FILLING = TIMER 10 ZONA (tanpa tetes, keputusan = timer). Tangki D30 cm.
//   1. TARE : rata-rata 5x jarak awal -> tinggi air awal, cek stok, zona awal.
//   2. Valve dibuka kontinyu. Tiap loop: terisi += Q(zona) x dt.
//      Sensor ultrasonik HANYA menentukan zona (Q) -> BUKAN penutup valve.
//   3. Valve ditutup saat terisi >= target - TRIM. Tanpa delay() (non-blocking).
//   4. 5 dtk setelah tutup dicetak baris VERIF (timer vs level) untuk kalibrasi.
//   Zona (tinggi air dari dasar): Z1 >=27 | Z2 >=24 | ... | Z9 >=3 | Z10 <3 cm.
//   KALIBRASI Q wajib: isi tangki sampai tengah zona, ukur waktu 1 L (gelas
//   ukur), Q = 1000 / detik, rata-rata 3x. Set: "Q1 135" lalu SAVE.
//   Debug: USD ON (kolom kiri mentah, kanan hasil EMA).
//
// JENIS KOPI (waktu = nomor, 1/2/3 mnt):
//   1=DAYLIGHT SHINE, 2=COFFEE VARIANT, 3=NIGHTFALL BLOOM.
//
// KEYPAD: A=input RPM (A lagi=OK, *=titik, #=hapus/batal)
//         B=jenis kopi (1/2/3 langsung pilih, #=batal)
//         C=filling liter (1-5=L, 0=0.5L; di layar siap *=+-0.5L, C=ubah)
//         D=START, #=STOP saat proses berjalan.
//
// SERIAL MONITOR 115200 + Newline:
//   RPM n      : set target RPM (0 < n <= MAX)
//   KOPI n     : 1=DAYLIGHT SHINE, 2=COFFEE VARIANT, 3=NIGHTFALL BLOOM
//   FILL n     : target liter, kelipatan 0.5 maks 5 (misal FILL 1.5)
//   START      : mulai (filling jika liter dipilih, else mengaduk)
//   STOP       : hentikan proses
//   AUTO n     : motor auto ke n RPM | MANUAL n : motor manual PWM 0-255
//   KP n KI n KD n : konstanta PID | EMA n : penghalus RPM (0.05-1.0)
//   CAL n      : faktor kalibrasi RPM | MAX n : batas RPM
//   Q z v      : debit zona z (1-10) = v mL/dtk, 5-200 (misal Q1 135.4)
//   ZONA       : lihat tabel batas zona + debit Q + REF + TRIM
//   REF n      : jarak muka sensor ke dasar tangki, cm (default 45.1)
//   TRIM n     : -50..+50 mL (+ tutup lebih awal, - tutup telat)
//   SAVE       : simpan semua setting ke EEPROM
//   RAW ON/OFF : data RPM (PV SV PWM) | COL ON/OFF : data warna RGB+homogen
//   USD ON/OFF : data ultrasonik (mentah + EMA).
// Telemetri OTOMATIS tiap 0.2 dtk saat running (tanpa perintah).
// ================================================================

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Keypad_I2C.h>

// ---------------- Pin ----------------
const uint8_t PIN_ENCODER = 2;
const uint8_t PIN_PWM     = 9;
const uint8_t PIN_RELAY   = 13;
const uint8_t PIN_TRIG    = 11;
const uint8_t PIN_ECHO    = 12;
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut A0

const uint8_t  PULSES_PER_REV = 20;
const unsigned long SAMPLE_MS = 50;
const unsigned long PID_MS    = 100;
const unsigned long RAW_MS    = 200;
const unsigned long MON_MS    = 200;   // interval telemetri otomatis saat running
const unsigned long LCD_MS    = 60;
const unsigned long KEY_MS    = 20;
const unsigned long ZERO_TIMEOUT_MS = 500;

// ================================================================
// PRESET YANG BISA DIUBAH
// ================================================================
const unsigned long STIR_MS[3] = { 60000UL, 120000UL, 180000UL };  // 1 mnt, 2 mnt, 3 mnt
const char COFFEE_NAME[3][16]  = { "DAYLIGHT SHINE", "COFFEE VARIANT", "NIGHTFALL BLOOM" };
const bool RELAY_ACTIVE_LOW = false;                       // false jika relay aktif HIGH

// ---- Filling timer 10 zona ----
const uint8_t  FILL_MAX_HALF = 10;        // 10 x 0.5 L = 5 L
const uint8_t  ZONES         = 10;
const float    TANK_AREA_CM2 = 3.14159265 * 15.0 * 15.0;   // tangki D30 cm
const float    MIN_LEVEL_CM  = 0.0;       // level minimum (cm dari dasar) = tutup paksa
const float    HYST_CM       = 0.7;       // histeresis pindah zona (anti-flicker)
// Batas bawah tiap zona (cm dari dasar), Z1 paling penuh .. Z10 paling kosong. Di PROGMEM.
const float BATAS[ZONES] PROGMEM = { 27, 24, 21, 18, 15, 12, 9, 6, 3, 0 };
const float Q_MIN_ML = 5.0, Q_MAX_ML = 200.0;      // batas valid debit (mL/dtk)
// Debit tiap zona (mL/dtk), Z1..Z10. GANTI dengan hasil kalibrasi (Q z v lalu SAVE).
float qZona[ZONES] = { 135.4, 131.2, 129.8, 120.2, 110.7, 100.9, 91.2, 81.0, 70.7, 60.4 };
float levelRef = 45.1;                    // cm: jarak sensor ke dasar (44.5 + offset 0.6)
float fillTrim = 0.0;                     // mL: + tutup lebih awal, - tutup lebih telat
const float TRIM_MAX_ML = 50.0, TRIM_MIN_ML = -50.0;
const uint8_t  TARE_N       = 5;          // tare = rata-rata 5 baca
const unsigned long TARE_TIMEOUT_MS = 10000UL;
const unsigned long VERIF_WAIT_MS   = 5000UL;   // tunggu air tenang sebelum cetak VERIF

// ---- Ultrasonik: d/58.3 + median-5 pra-filter + gate jangkauan + EMA ----
const unsigned long US_PING_MS = 100;        // jeda ping saat standby/debug
const unsigned long US_PING_FILL_MS = 60;    // jeda ping saat TARE/FILLING
const uint8_t  US_MED_N = 5;          // median pra-filter: kebal 2 sampel sampah/jendela
const float    US_MIN_CM = 2.0, US_MAX_CM = 150.0;  // buang gema/artefak di luar tangki
const float    US_EMA = 0.3;          // EMA: kecil = lebih halus, lebih lambat
const float    US_JUMP_CM = 2.0;      // abaikan sampel yang melonjak > ini per ping (noise relay/gema)
const unsigned long US_BLANK_MS = 500;   // abaikan semua sampel 0.5 dtk tiap relay klik

// ---- Sensor warna (monitoring homogenisasi, NON-blocking, timer tetap penentu selesai) ----
const unsigned long COLOR_MS = 500;
const uint8_t  COLOR_WIN     = 8;           // jendela lebih besar = lebih stabil
const uint8_t  COLOR_AVG     = 5;           // trimmed-mean: buang min & max

// ---------------- Keypad I2C ----------------
#define KEYPAD_ADDR 0x20
const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '4', '7', '*'},
  {'2', '5', '8', '0'},
  {'3', '6', '9', '#'},
  {'A', 'B', 'C', 'D'}
};
byte rowPins[ROWS] = {0, 1, 2, 3};
byte colPins[COLS] = {4, 5, 6, 7};
Keypad_I2C keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS, KEYPAD_ADDR, PCF8574);

LiquidCrystal_I2C lcd(0x27, 20, 4);         // ganti 0x3F jika perlu

// ---------------- Pengukuran pulsa encoder ----------------
const uint8_t  RING = PULSES_PER_REV + 1;
const unsigned long MIN_PULSE_US = 15000;
const float MISS_RATIO = 1.5;

volatile unsigned long pulseTs[RING];
volatile unsigned long pulseIdx = 0;
volatile uint8_t       validCnt = 0;
volatile unsigned long lastPulseUs = 0;
volatile float         avgIv = 0;

// ---------------- Parameter ----------------
float emaAlpha = 0.2;
float Kp = 2.4, Ki = 1.4, Kd = 0.0;
float maxRpm = 90.0;

const float I_MAX = 255.0;
const int   SLEW  = 25;
const float KI_BOOST   = 2.0;
const float BOOST_BAND = 6.0;

// ---------------- Status motor ----------------
enum Mode { MODE_MANUAL, MODE_AUTO };
Mode  mode       = MODE_MANUAL;
float targetRpm  = 0;
int   currentPwm = 0;
float calFactor  = 0.4941;

float rpmRaw = 0, rpmEma = 0, rpmCal = 0;
bool  emaInit = false;
float integralTerm = 0, lastRpmCal = 0;

bool rawShow = false, colShow = false;
bool wasRunning = false;                  // tepi naik running -> cetak header telemetri
unsigned long tSample = 0, tPid = 0, tRaw = 0, tLcd = 0, tKey = 0, tMon = 0;

// ---------------- State machine ----------------
enum Stage {
  ST_IDLE, ST_IN_RPM, ST_IN_COFFEE, ST_IN_LITER, ST_FILL_READY,
  ST_TARE, ST_FILLING, ST_STIR, ST_COLOR, ST_DONE
};
Stage stage = ST_IDLE;

float    setRpm     = 0;
int8_t   coffeeType = -1;        // 0 DAYLIGHT SHINE (1 mnt), 1 COFFEE VARIANT (2 mnt), 2 NIGHTFALL BLOOM (3 mnt)
uint8_t  fillHalf   = 0;         // volume target dalam satuan 0.5 L (1..10)
char     entryBuf[8] = "";

unsigned long stirStart = 0, stirDur = 0;

float   tareDist = 0, tareSum = 0;          // jarak awal sensor (cm) & akumulator tare
uint8_t tareCnt = 0;
bool    tareReady = false;                  // tare selesai, menunggu jeda buka valve
unsigned long tareDoneAt = 0;
const unsigned long TARE_WAIT_MS = 2000;     // jeda 2 dtk setelah tare sebelum valve dibuka
float   fillTargetMl = 0;                   // target volume (mL), diset saat start
float   dispensedMl  = 0;                   // volume terintegrasi: sum Q(zona) x dt
int8_t  fillZone     = -1;                  // zona aktif 0..9 (-1 = belum)
unsigned long fillLast = 0, fillTimeout = 0, verifAt = 0;
float   usEma = -1;                         // hasil filter EMA (-1 = belum ada)
uint8_t usRej = 0;                          // hitungan lonjakan beruntun
float   medBuf[5];                          // jendela median pra-filter
uint8_t medN = 0, medIdx = 0;
bool    usShow = false;                     // USD ON: cetak mentah + hasil EMA
unsigned long usBlankUntil = 0;             // masa blanking transien relay
unsigned long fillStart = 0, tareStart = 0, tUs = 0;

float cR[COLOR_WIN], cG[COLOR_WIN], cB[COLOR_WIN];
uint8_t colorIdx = 0, colorCount = 0;
bool    colorFullInit = false;          // jendela penuh pertama sudah diisi?
float   colorHomog  = 0;                // homogenisasi 0-100, monitoring saja
float   lastR = 0, lastG = 0, lastB = 0; // mentah RGB terakhir untuk telemetri
unsigned long tColor = 0;

char doneTitle[21], doneL1[21], doneL2[21];

char lcdMsg[21] = "";
unsigned long msgUntil = 0;
char lcdBuf[4][21];
char lcdShown[4][21];

// ---------------- EEPROM ----------------
struct Settings { uint16_t magic; float kp, ki, kd, cal, maxRpm, alpha, trim; float q[ZONES]; float ref; };
const uint16_t SETTINGS_MAGIC = 0xA55E;     // beda dari versi lama (layout berubah)

void saveSettings() {
  Settings s;
  s.magic = SETTINGS_MAGIC;
  s.kp = Kp; s.ki = Ki; s.kd = Kd; s.cal = calFactor;
  s.maxRpm = maxRpm; s.alpha = emaAlpha; s.trim = fillTrim;
  memcpy(s.q, qZona, sizeof qZona);
  s.ref = levelRef;
  EEPROM.put(0, s);
}

void loadSettings() {
  Settings s;
  EEPROM.get(0, s);
  if (s.magic != SETTINGS_MAGIC) return;
  if (!(s.cal > 0) || !(s.maxRpm > 0) || !(s.alpha > 0 && s.alpha <= 1)) return;
  if (s.kp < 0 || s.ki < 0 || s.kd < 0) return;
  if (!(s.trim >= TRIM_MIN_ML && s.trim <= TRIM_MAX_ML)) return;
  if (!(s.ref >= 10 && s.ref <= 150)) return;
  for (uint8_t i = 0; i < ZONES; i++)
    if (!(s.q[i] >= Q_MIN_ML && s.q[i] <= Q_MAX_ML)) return;
  Kp = s.kp; Ki = s.ki; Kd = s.kd;
  calFactor = s.cal; maxRpm = s.maxRpm; emaAlpha = s.alpha;
  fillTrim = s.trim; levelRef = s.ref;
  memcpy(qZona, s.q, sizeof qZona);
}

// Target volume tempat valve ditutup (mL) = target - trim, minimal 10 mL
float fillGoalMl() {
  float g = fillTargetMl - fillTrim;
  return (g < 10.0) ? 10.0 : g;
}

// Zona murni dari tinggi air (cm dari dasar): 0 = Z1 .. 9 = Z10
int8_t zoneOf(float h) {
  for (uint8_t i = 0; i < ZONES; i++)
    if (h >= pgm_read_float(&BATAS[i])) return i;
  return ZONES - 1;
}

// Zona dengan histeresis (keputusan timer) agar tidak flicker di perbatasan
int8_t zoneHyst(float h, int8_t z) {
  if (z > 0 && h >= pgm_read_float(&BATAS[z - 1]) + HYST_CM * 0.5) return z - 1;
  if (z < ZONES - 1 && h < pgm_read_float(&BATAS[z]) - HYST_CM) return z + 1;
  return z;
}

// Durasi pengadukan (ms) sesuai jenis kopi
unsigned long stirMsFor(int8_t c) {
  return STIR_MS[c];
}

// ================================================================
// ISR encoder
// ================================================================
void isrEncoder() {
  unsigned long t  = micros();
  unsigned long dt = t - lastPulseUs;
  if (dt < MIN_PULSE_US) return;

  if (validCnt == 0) {
    pulseIdx++;
    pulseTs[pulseIdx % RING] = t;
    validCnt = 1;
    avgIv = 0;
    lastPulseUs = t;
    return;
  }

  uint8_t p = 1;
  if (validCnt >= 5 && avgIv > 0) {
    float r = (float)dt / avgIv;
    if (r >= MISS_RATIO) {
      p = (uint8_t)(r + 0.5);
      if (p > 3) p = 1;
    }
  }

  unsigned long step = dt / p;
  for (uint8_t k = 1; k <= p; k++) {
    pulseIdx++;
    pulseTs[pulseIdx % RING] = lastPulseUs + step * k;
  }
  pulseTs[pulseIdx % RING] = t;

  validCnt += p;
  if (validCnt > RING) validCnt = RING;

  float iv = (float)step;
  avgIv = (avgIv == 0) ? iv : (0.85 * avgIv + 0.15 * iv);
  lastPulseUs = t;
}

// ================================================================
// Util motor
// ================================================================
void setPwm(int v) {
  currentPwm = constrain(v, 0, 255);
  analogWrite(PIN_PWM, currentPwm);
}

float feedForward() {
  return constrain(targetRpm * 255.0 / maxRpm, 0, 255);
}

void resetPid() {
  integralTerm = constrain(currentPwm - feedForward(), -I_MAX, I_MAX);
  lastRpmCal = rpmCal;
}

// ================================================================
// Hardware: valve, ultrasonik, warna
// ================================================================
void valveOn()  { digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? LOW  : HIGH); }
void valveOff() { digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW ); }

// ---- Ultrasonik: rumus tetap (343 m/s), tanpa suhu ----
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long d = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (d == 0) return -1;
  return d * 0.01715;                      // = d * 0.0343 / 2
}

// Baca dengan jeda antar ping (lebih cepat saat TARE/FILLING).
// Return < 0 jika belum waktunya / gagal.
float usRead(unsigned long now) {
  unsigned long gap = (stage == ST_TARE || stage == ST_FILLING) ? US_PING_FILL_MS : US_PING_MS;
  if (now - tUs < gap) return -2;   // belum waktunya
  tUs = now;
  return readDistanceCm();                 // -1 jika tidak ada echo
}

// Filter 3 lapis: median-5 pra-filter + gate jangkauan + EMA + tolak lonjakan.
// Return -2/-1 teruskan dari usRead, selain itu nilai halus.
// Warm-up: EMA baru di-seed dari median valid (sampel 1-4 tidak dipakai).
// Cetak mentah\thasil jika USD ON.
float usSample(unsigned long now) {
  float d = usRead(now);
  if (d < 0) return d;
  if (d < US_MIN_CM || d > US_MAX_CM) return (usEma < 0) ? -1 : usEma;  // artefak/gema jauh
  medBuf[medIdx] = d;                       // median-5: kebal hingga 2 sampah/jendela
  medIdx = (medIdx + 1) % US_MED_N;
  if (medN < US_MED_N) medN++;
  if (medN >= US_MED_N) {
    float t[US_MED_N];
    for (uint8_t i = 0; i < US_MED_N; i++) t[i] = medBuf[i];
    for (uint8_t i = 1; i < US_MED_N; i++) {
      float x = t[i];
      int8_t j = i - 1;
      while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; }
      t[j + 1] = x;
    }
    d = t[US_MED_N / 2];
  } else if (usEma < 0) {
    return -2;                              // warm-up: belum ada median valid
  }
  if (usEma < 0) {
    usEma = d; usRej = 0;
  } else if (now < usBlankUntil) {
    // masa transien klik relay: pertahankan nilai lama
  } else if (fabs(d - usEma) > US_JUMP_CM) {
    // lonjakan > 2 cm: abaikan, KECUALI bertahan 20x beruntun (~1.2 dtk pada
    // ping 60 ms) (= level benar-benar pindah / EMA sempat mengunci gema)
    if (++usRej >= 20) { usEma = d; usRej = 0; }
  } else {
    usEma += US_EMA * (d - usEma);
    usRej = 0;
  }
  if (usShow) { Serial.print(d, 2); Serial.print('\t'); Serial.println(usEma, 2); }
  return usEma;
}

// Lebar pulsa (us) kanal R,G,B. Nilai kecil = intensitas tinggi.
// Perbaikan akurasi: buang sampel pertama tiap ganti filter (settling),
// trimmed-mean per kanal (buang min & max), tolak cahaya jenuh/gelap.
bool readColor(float &r, float &g, float &b) {
  float v[3];
  for (uint8_t c = 0; c < 3; c++) {
    digitalWrite(S2, c == 1);                 // R: LL, G: HH, B: LH
    digitalWrite(S3, c != 0);
    delay(10);
    pulseIn(sensorOut, LOW, 30000UL);         // buang: fotodioda belum stabil
    unsigned long s[COLOR_AVG];
    for (uint8_t i = 0; i < COLOR_AVG; i++) {
      unsigned long p = pulseIn(sensorOut, LOW, 30000UL);
      if (p < 5 || p > 25000UL) return false; // jenuh / gelap / timeout
      s[i] = p;
    }
    for (uint8_t i = 1; i < COLOR_AVG; i++) { // urutkan
      unsigned long x = s[i];
      int8_t j = i - 1;
      while (j >= 0 && s[j] > x) { s[j + 1] = s[j]; j--; }
      s[j + 1] = x;
    }
    unsigned long sum = 0;
    for (uint8_t i = 1; i < COLOR_AVG - 1; i++) sum += s[i];
    v[c] = (float)sum / (COLOR_AVG - 2);
  }
  r = v[0]; g = v[1]; b = v[2];
  return true;
}

// Reset monitoring warna saat mulai mengaduk
void colorReset() {
  colorIdx = 0; colorCount = 0;
  colorFullInit = false; colorHomog = 0;
  lastR = 0; lastG = 0; lastB = 0;
  tColor = 0;
}

// ================================================================
// Urutan proses
// ================================================================
void showMsg(const char* s) {
  strncpy(lcdMsg, s, 20); lcdMsg[20] = 0;
  msgUntil = millis() + 1500;
}
void showMsgP(const char* s) {
  strncpy_P(lcdMsg, s, 20); lcdMsg[20] = 0;
  msgUntil = millis() + 1500;
}

// semua argumen string di PROGMEM (PSTR)
void setDone(const char* t, const char* l1, const char* l2) {
  strncpy_P(doneTitle, t,  20); doneTitle[20] = 0;
  strncpy_P(doneL1,    l1, 20); doneL1[20]    = 0;
  strncpy_P(doneL2,    l2, 20); doneL2[20]    = 0;
  stage = ST_DONE;
}

void hardwareOff() {
  valveOff();
  mode = MODE_MANUAL;
  targetRpm = 0;
  integralTerm = 0;
  setPwm(0);
}

void abortRun(const char* whyP) {
  bool running = (stage == ST_TARE || stage == ST_FILLING ||
                  stage == ST_STIR || stage == ST_COLOR);
  hardwareOff();
  if (running) setDone(PSTR("DIHENTIKAN"), whyP, PSTR("Motor & valve OFF"));
  else         stage = ST_IDLE;
}

void startStir() {
  if (setRpm <= 0)    { showMsgP(PSTR("Set RPM dulu (A)")); return; }
  if (coffeeType < 0) { showMsgP(PSTR("Pilih kopi (B)"));   return; }
  verifAt   = 0;                        // batalkan cetak VERIF filling yang tertunda
  stirStart = millis();
  stirDur   = stirMsFor(coffeeType);
  targetRpm = setRpm;
  mode      = MODE_AUTO;
  resetPid();
  colorReset();                         // monitoring homogen mulai dari nol
  stage     = ST_STIR;
}

void startFill() {
  if (fillHalf < 1 || fillHalf > FILL_MAX_HALF) { showMsgP(PSTR("Pilih liter (C)")); return; }
  valveOff();
  fillTargetMl = fillHalf * 500.0;
  tareCnt = 0; tareSum = 0; tareDist = 0;
  tareReady = false;
  dispensedMl = 0; fillZone = -1; verifAt = 0;
  usEma = -1; usRej = 0; tUs = 0; usBlankUntil = 0; medN = 0; medIdx = 0;
  tareStart = millis();
  stage = ST_TARE;
}

// TARE: rata-rata 5 baca (hasil EMA) valid, lalu jeda 2 dtk (valve tetap tutup,
// air tenang). Setelah itu cek stok, tentukan zona awal, buka valve.
void tareStep(unsigned long now) {
  if (tareReady) {                           // tare sudah selesai, hitung jeda
    if (now - tareDoneAt >= TARE_WAIT_MS) {
      float h0 = levelRef - tareDist;        // tinggi air awal dari dasar (cm)
      float stokMl = TANK_AREA_CM2 * (h0 - MIN_LEVEL_CM);
      if (stokMl < fillTargetMl) {           // stok tidak cukup: valve tidak dibuka
        char a[8];
        dtostrf(stokMl / 1000.0, 4, 2, a);
        setDone(PSTR("STOK KURANG"), PSTR(""), PSTR("Isi tangki dulu"));
        snprintf_P(doneL1, sizeof doneL1, PSTR("Siap: %s L"), a);
        return;
      }
      fillZone = zoneOf(h0);
      float qMin = qZona[0];                 // timeout = waktu isi pada debit terkecil x2 + 30 dtk
      for (uint8_t i = 1; i < ZONES; i++) if (qZona[i] < qMin) qMin = qZona[i];
      fillTimeout = (unsigned long)(fillGoalMl() / (qMin * 0.5) * 1000.0) + 30000UL;
      dispensedMl = 0;
      fillStart = fillLast = now;
      valveOn();
      usBlankUntil = now + US_BLANK_MS;      // abaikan transien klik relay
      stage = ST_FILLING;
    }
    return;
  }
  float d = usSample(now);
  if (d == -2) return;                       // belum waktunya ping / warm-up median
  if (d < 0) {                               // tidak ada echo
    if (now - tareStart > TARE_TIMEOUT_MS) {
      hardwareOff();
      setDone(PSTR("ERROR SENSOR"), PSTR("HC-SR04 tdk terbaca"), PSTR("Cek TRIG/ECHO"));
    }
    return;
  }
  tareSum += d;
  tareCnt++;
  if (tareCnt < TARE_N) return;

  tareDist  = tareSum / TARE_N;
  tareReady = true;                          // mulai hitung jeda 2 dtk, valve tetap tutup
  tareDoneAt = now;
}

// FILLING timer 10 zona. Dipanggil tiap loop (non-blocking):
//   terisi += Q(zona aktif) x dt ; tutup saat terisi >= target - trim.
// Sensor hanya memperbarui zona. Jika sensor putus, zona terakhir dipakai
// dan timer tetap menutup valve (keputusan = timer).
void fillStep(unsigned long now) {
  dispensedMl += qZona[fillZone] * (now - fillLast) / 1000.0;
  fillLast = now;

  if (dispensedMl >= fillGoalMl()) {
    valveOff();                              // tutup SEKALI, langsung selesai
    usBlankUntil = millis() + US_BLANK_MS;
    verifAt = millis() + VERIF_WAIT_MS;      // cetak VERIF setelah air tenang
    char a[8];
    dtostrf(fillHalf * 0.5, 3, 1, a);
    setDone(PSTR("FILLING SELESAI"), PSTR(""), PSTR(""));
    snprintf_P(doneL1, sizeof doneL1, PSTR("%s L selesai"), a);
    doneL2[0] = 0;
    return;
  }
  if (now - fillStart > fillTimeout) {
    hardwareOff();
    setDone(PSTR("TIMEOUT FILLING"), PSTR("Valve ditutup"), PSTR("Cek aliran/sensor"));
    return;
  }

  float d = usSample(now);
  if (d < 0) return;                         // belum waktunya / sampel gagal
  float h = levelRef - d;                    // tinggi air dari dasar (cm)
  fillZone = zoneHyst(h, fillZone);
  if (h <= MIN_LEVEL_CM) {                   // tangki kosong: tutup paksa
    hardwareOff();
    setDone(PSTR("LEVEL MINIMUM"), PSTR("Valve ditutup"), PSTR("Isi tangki dulu"));
  }
}

// Verifikasi (hanya info kalibrasi, tidak mengoreksi): bandingkan volume
// hitungan timer dengan penurunan level terukur setelah air tenang.
void fillVerify(unsigned long now) {
  if (!verifAt || (long)(now - verifAt) < 0) return;
  verifAt = 0;
  if (usEma < 0) return;
  float estMl = dispensedMl + fillTrim;                  // timer + tetesan setelah tutup
  float lvlMl = TANK_AREA_CM2 * (usEma - tareDist);      // dari penurunan level
  Serial.print(F("VERIF: timer=")); Serial.print(estMl, 0);
  Serial.print(F(" level=")); Serial.print(lvlMl, 0);
  Serial.print(F(" mL selisih=")); Serial.print((lvlMl - fillTargetMl) / fillTargetMl * 100.0, 1);
  Serial.print(F("% Z")); Serial.println(fillZone + 1);
}

// Monitoring homogenisasi (NON-blocking): dipanggil saat ST_STIR berjalan.
// Memakai rasio warna ternormalisasi (r/total dst) agar tahan perubahan
// jarak/cahaya. Tidak pernah menghentikan motor; timer yang menentukan selesai.
void colorMonitor(unsigned long now) {
  if (now - tColor < COLOR_MS) return;
  tColor = now;

  float r, g, b;
  if (!readColor(r, g, b)) return;          // pertahankan nilai lama jika gagal
  lastR = r; lastG = g; lastB = b;          // simpan mentah untuk telemetri

  float tot = r + g + b;
  if (!(tot > 0)) return;
  float rn = r / tot, gn = g / tot, bn = b / tot;

  cR[colorIdx] = rn; cG[colorIdx] = gn; cB[colorIdx] = bn;
  colorIdx = (colorIdx + 1) % COLOR_WIN;
  if (colorCount < COLOR_WIN) colorCount++;
  if (colorCount < COLOR_WIN) { colorHomog = 0; return; }

  float worst = 0;
  for (uint8_t c = 0; c < 3; c++) {
    float *arr = (c == 0) ? cR : (c == 1) ? cG : cB;
    float mn = arr[0], mx = arr[0], sum = 0;
    for (uint8_t i = 0; i < COLOR_WIN; i++) {
      if (arr[i] < mn) mn = arr[i];
      if (arr[i] > mx) mx = arr[i];
      sum += arr[i];
    }
    float mean = sum / COLOR_WIN;
    if (mean > 0.001) {
      float dev = (mx - mn) / mean * 100.0;
      if (dev > worst) worst = dev;
    }
  }
  float inst = constrain(100.0 - worst * 4.0, 0, 100);  // rasio kecil -> skala x4
  if (!colorFullInit) { colorHomog = inst; colorFullInit = true; }
  else colorHomog = 0.4 * inst + 0.6 * colorHomog;       // haluskan tampilan

  if (colShow) {
    Serial.print(r, 0); Serial.print(' ');
    Serial.print(g, 0); Serial.print(' ');
    Serial.print(b, 0); Serial.print(' ');
    Serial.println(colorHomog, 1);
  }
}

void stirFinishWithHomog() {
  hardwareOff();
  char a[8];
  dtostrf(colorHomog, 4, 1, a);
  setDone(PSTR("ADUK SELESAI"), PSTR(""), PSTR("Motor berhenti"));
  snprintf_P(doneL1, sizeof doneL1, PSTR("Homogen: %s%%"), a);
}

void processStage() {
  unsigned long now = millis();
  switch (stage) {
    case ST_TARE:    tareStep(now);   break;
    case ST_FILLING: fillStep(now);   break;
    case ST_STIR:
      colorMonitor(now);                        // monitoring paralel, timer tetap jalan
      if (now - stirStart >= stirDur) stirFinishWithHomog();
      break;
    case ST_COLOR:                              // kompatibilitas: tidak dipakai lagi
      colorMonitor(now);
      if (now - stirStart >= stirDur) stirFinishWithHomog();
      break;
    default:
      if (usShow || verifAt) usSample(now);  // debug ultrasonik / level untuk VERIF
      fillVerify(now);
      break;
  }
}

// ================================================================
// Perintah Serial (ringkas)
// ================================================================
void printZona() {
  for (uint8_t i = 0; i < ZONES; i++) {
    Serial.print('Z'); Serial.print(i + 1);
    Serial.print(F(" >= ")); Serial.print(pgm_read_float(&BATAS[i]), 0);
    Serial.print(F(" cm : ")); Serial.print(qZona[i], 1); Serial.println(F(" mL/s"));
  }
  Serial.print(F("REF=")); Serial.print(levelRef, 1);
  Serial.print(F(" cm | TRIM=")); Serial.print(fillTrim, 1); Serial.println(F(" mL"));
}

bool processCommand(char* s) {
  char* arg = strchr(s, ' ');
  if (arg) { *arg = 0; arg++; } else arg = (char*)"";
  for (char* p = s; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
  float v = atof(arg);

  if (stage == ST_DONE) stage = ST_IDLE;   // samakan keypad: layar selesai bubar
                                           // oleh perintah apapun, jadi serial
                                           // tetap bisa START/FILL lagi
  if      (!strcmp(s, "AUTO"))   { targetRpm = setRpm = max(v, 0.0f); mode = MODE_AUTO; resetPid(); }
  else if (!strcmp(s, "MANUAL")) { mode = MODE_MANUAL; setPwm((int)v); }
  else if (!strcmp(s, "RPM"))    { if (v <= 0 || v > maxRpm) return false; setRpm = v; }
  else if (!strcmp(s, "KOPI"))   {
    if (v < 1 || v > 3) return false;
    coffeeType = (int)v - 1;
  }
  else if (!strcmp(s, "FILL"))   {
    float h = v * 2;
    uint8_t u = (uint8_t)(h + 0.5);
    if (u < 1 || u > FILL_MAX_HALF || fabs(h - u) > 0.05 || (stage != ST_IDLE && stage != ST_FILL_READY)) return false;
    fillHalf = u; stage = ST_FILL_READY;
  }
  else if (!strcmp(s, "START"))  {
    if (stage == ST_FILL_READY) startFill();
    else if (stage == ST_IDLE)  startStir();
    else return false;
  }
  else if (!strcmp(s, "STOP"))   { abortRun(PSTR("Perintah STOP")); }
  else if (!strcmp(s, "KP"))     { if (v < 0) return false; Kp = v; resetPid(); }
  else if (!strcmp(s, "KI"))     { if (v < 0) return false; Ki = v; resetPid(); }
  else if (!strcmp(s, "KD"))     { if (v < 0) return false; Kd = v; resetPid(); }
  else if (!strcmp(s, "CAL"))    { if (v <= 0) return false; calFactor = v; resetPid(); }
  else if (!strcmp(s, "MAX"))    { if (v <= 0) return false; maxRpm = v; resetPid(); }
  else if (!strcmp(s, "EMA"))    { if (v < 0.05 || v > 1.0) return false; emaAlpha = v; }
  else if (!strcmp(s, "TRIM"))   { if (v < TRIM_MIN_ML || v > TRIM_MAX_ML) return false; fillTrim = v; }
  else if (!strcmp(s, "REF"))    { if (v < 10 || v > 150) return false; levelRef = v; }
  else if (!strcmp(s, "ZONA") || !strcmp(s, "Z")) { printZona(); }
  else if (s[0] == 'Q' && s[1] >= '0' && s[1] <= '9') {   // Q z v : debit zona z
    uint8_t z = atoi(s + 1);
    if (z < 1 || z > ZONES || v < Q_MIN_ML || v > Q_MAX_ML) return false;
    qZona[z - 1] = v;
  }
  else if (!strcmp(s, "USD"))    { usShow = (strcmp(arg, "ON") == 0 || strcmp(arg, "on") == 0); }
  else if (!strcmp(s, "SAVE"))   { saveSettings(); }
  else if (!strcmp(s, "RAW"))    { rawShow = (strcmp(arg, "ON") == 0 || strcmp(arg, "on") == 0); }
  else if (!strcmp(s, "COL"))    { colShow = (strcmp(arg, "ON") == 0 || strcmp(arg, "on") == 0); }
  else return false;
  return true;
}

void readSerial() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) {
        buf[n] = 0;
        Serial.println(processCommand(buf) ? F("OK") : F("ERR"));
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ================================================================
// Keypad
// ================================================================
void commitRpm() {
  float v = atof(entryBuf);
  if (v <= 0)     { showMsgP(PSTR("RPM harus > 0")); return; }
  if (v > maxRpm) { showMsgP(PSTR("Melebihi RPM maks")); return; }
  setRpm = v;
  stage = ST_IDLE;
}

void handleKey(char k) {
  uint8_t len = strlen(entryBuf);
  switch (stage) {

    case ST_IDLE:
      if      (k == 'A') { entryBuf[0] = 0; stage = ST_IN_RPM; }
      else if (k == 'B') stage = ST_IN_COFFEE;
      else if (k == 'C') stage = ST_IN_LITER;
      else if (k == 'D') startStir();
      break;

    case ST_IN_RPM:
      if (k >= '0' && k <= '9') {
        if (len < 6) { entryBuf[len] = k; entryBuf[len + 1] = 0; }
      }
      else if (k == '*') {
        if (!strchr(entryBuf, '.') && len < 5) {
          if (len == 0) { entryBuf[0] = '0'; len = 1; }
          entryBuf[len] = '.'; entryBuf[len + 1] = 0;
        }
      }
      else if (k == '#') {
        if (len == 0) stage = ST_IDLE; else entryBuf[len - 1] = 0;
      }
      else if (k == 'A') commitRpm();
      break;

    case ST_IN_COFFEE:
      if (k >= '1' && k <= '3') { coffeeType = k - '1'; stage = ST_IDLE; }
      else if (k == '#')         stage = ST_IDLE;
      break;

    case ST_IN_LITER:
      if (k >= '0' && k <= '5') { fillHalf = (k == '0') ? 1 : (k - '0') * 2; stage = ST_FILL_READY; }
      else if (k == '#') stage = ST_IDLE;
      break;

    case ST_FILL_READY:
      if      (k == 'D') startFill();
      else if (k == 'C') stage = ST_IN_LITER;
      else if (k == '*') {                         // +/- 0.5 L
        if (fillHalf & 1) { if (fillHalf > 1) fillHalf--; }
        else if (fillHalf < FILL_MAX_HALF) fillHalf++;
      }
      else if (k == '#') { fillHalf = 0; stage = ST_IDLE; }
      break;

    case ST_TARE:
    case ST_FILLING:
    case ST_STIR:
    case ST_COLOR:
      if (k == '#') abortRun(PSTR("Ditekan tombol #"));
      break;

    case ST_DONE:
      stage = ST_IDLE;
      break;
  }
}

void readKeypad() {
  unsigned long now = millis();
  if (now - tKey < KEY_MS) return;
  tKey = now;
  char k = keypad.getKey();
  if (k) handleKey(k);
}

// ================================================================
// LCD (1 baris per siklus)
// ================================================================
void setLine(uint8_t r, const char* s) {
  uint8_t i = 0;
  while (i < 20 && s[i]) { lcdBuf[r][i] = s[i]; i++; }
  while (i < 20) lcdBuf[r][i++] = ' ';
  lcdBuf[r][20] = 0;
}

void setLineP(uint8_t r, const char* s) {
  uint8_t i = 0;
  char c;
  while (i < 20 && (c = pgm_read_byte(s + i))) { lcdBuf[r][i] = c; i++; }
  while (i < 20) lcdBuf[r][i++] = ' ';
  lcdBuf[r][20] = 0;
}
#define LP(r, s) setLineP(r, PSTR(s))

// ---- Helper tampilan interaktif ----
// Titik animasi: "", ".", "..", "..."
const char* dotsStr() {
  static const char d[4][4] = { "", ".", "..", "..." };
  return d[(millis() / 400) % 4];
}

// Kedip 0,5 detik
bool blinkOn() { return (millis() / 500) % 2; }

void buildLines() {
  char t[26], a[10], b[10];
  bool ready = (setRpm > 0 && coffeeType >= 0);
  char cur = blinkOn() ? '_' : ' ';

  switch (stage) {

    // ---------------- LAYAR AWAL ----------------
    case ST_IDLE: {
      // tanda ">" menunjuk langkah berikutnya yang belum diisi
      char m0 = (setRpm <= 0) ? '>' : ' ';
      char m1 = (setRpm > 0 && coffeeType < 0) ? '>' : ' ';

      if (setRpm > 0) {
        dtostrf(setRpm, 4, 1, a);
        snprintf_P(t, sizeof t, PSTR("%cRPM  : %s rpm"), m0, a);
      } else snprintf_P(t, sizeof t, PSTR("%cRPM  : -- (tekan A)"), m0);
      setLine(0, t);

      if (coffeeType >= 0)
        snprintf_P(t, sizeof t, PSTR("%c%s"), m1, COFFEE_NAME[coffeeType]);
      else snprintf_P(t, sizeof t, PSTR("%cKopi : -- (tekan B)"), m1);
      setLine(1, t);

      if (coffeeType >= 0) {
        unsigned long s = stirMsFor(coffeeType) / 1000UL;
        snprintf_P(t, sizeof t, PSTR(" Waktu: %02u:%02u mnt"),
                   (unsigned)(s / 60), (unsigned)(s % 60));
      } else strcpy_P(t, PSTR(" Waktu: --:--"));
      setLine(2, t);

      bool ph = (millis() / 2000UL) % 2;
      if (ready) {
        if (!ph) { if (blinkOn()) LP(3, ">> D = MULAI <<"); else LP(3, "   D = MULAI"); }
        else LP(3, "A/B=ubah  C=Isi");
      } else {
        if (!ph) LP(3, "A=RPM B=Kopi C=Isi");
        else     LP(3, "D=Start   #=Stop");
      }
      break;
    }

    // ---------------- INPUT RPM ----------------
    case ST_IN_RPM:
      snprintf_P(t, sizeof t, PSTR("INPUT RPM (maks %d)"), (int)maxRpm);
      setLine(0, t);
      snprintf_P(t, sizeof t, PSTR("Nilai > %s%c"), entryBuf, cur);
      setLine(1, t);
      dtostrf(setRpm, 5, 1, a);
      snprintf_P(t, sizeof t, PSTR("Sebelumnya: %s"), a);
      setLine(2, t);
      LP(3, "*=titik #=hapus A=OK");
      break;

    // ---------------- PILIH KOPI (1=DAYLIGHT SHINE 1mnt, 2=COFFEE VARIANT 2mnt, 3=NIGHTFALL BLOOM 3mnt) ----------------
    case ST_IN_COFFEE:
      LP(0, "PILIH KOPI   #=batal");
      for (uint8_t i = 0; i < 3; i++) {
        char mk = (coffeeType == i) ? '*' : ' ';
        snprintf_P(t, sizeof t, PSTR("%u.%-15s %c"),
                   (unsigned)(i + 1), COFFEE_NAME[i], mk);
        setLine(1 + i, t);
      }
      break;

    // ---------------- PILIH LITER ----------------
    case ST_IN_LITER:
      LP(0, "MODE FILLING");
      LP(1, "1-5=liter  0=0.5 L");
      if (fillHalf > 0) {
        dtostrf(fillHalf * 0.5, 3, 1, a);
        snprintf_P(t, sizeof t, PSTR("Terakhir: %s L"), a);
        setLine(2, t);
      } else LP(2, "Lalu * = +/- 0.5 L");
      LP(3, "#=batal");
      break;

    case ST_FILL_READY:
      LP(0, "FILLING SIAP");
      dtostrf(fillHalf * 0.5, 3, 1, a);
      snprintf_P(t, sizeof t, PSTR("Volume : %s L"), a);
      setLine(1, t);
      setLine(2, "");
      {
        uint8_t ph3 = (millis() / 2000UL) % 3;
        if      (ph3 == 1) LP(3, "*=+/-0.5L  C=ubah");
        else if (ph3 == 2) LP(3, "#=batal");
        else if (blinkOn()) LP(3, ">> D = MULAI <<");
        else                LP(3, "   D = MULAI");
      }
      break;

    // ---------------- TARE (tanpa bar) ----------------
    case ST_TARE:
      LP(0, "FILLING - TARE");
      if (tareReady) {
        unsigned long w = (millis() - tareDoneAt >= TARE_WAIT_MS) ? 0
                        : (TARE_WAIT_MS - (millis() - tareDoneAt) + 999UL) / 1000UL;
        snprintf_P(t, sizeof t, PSTR("Buka valve %lus"), w);
        setLine(1, t);
      } else {
        snprintf_P(t, sizeof t, PSTR("Ukur jarak awal%s"), dotsStr());
        setLine(1, t);
      }
      LP(2, "Valve TUTUP");
      LP(3, "#=stop");
      break;

    // ---------------- FILLING ----------------
    case ST_FILLING: {
      dtostrf(fillHalf * 0.5, 3, 1, b);
      snprintf_P(t, sizeof t, PSTR("FILL %sL VALVE:ON"), b);
      setLine(0, t);
      snprintf_P(t, sizeof t, PSTR("Target: %s L"), b);
      setLine(1, t);
      LP(2, "Mengisi...");
      LP(3, "#=stop");
      break;
    }

    // ---------------- MENGADUK (timer, tanpa bar): SV/PV + Homogen ----------------
    case ST_STIR:
    case ST_COLOR: {                            // ST_COLOR tidak dipakai lagi
      unsigned long el = millis() - stirStart;
      if (el > stirDur) el = stirDur;
      long rem = (long)((stirDur - el) / 1000UL);
      snprintf_P(t, sizeof t, PSTR(">> %s"), COFFEE_NAME[coffeeType]);
      setLine(0, t);
      dtostrf(targetRpm, 4, 1, a);
      dtostrf(rpmCal, 4, 1, b);
      snprintf_P(t, sizeof t, PSTR("SV:%s PV:%s"), a, b);
      setLine(1, t);
      if (colorCount >= COLOR_WIN) dtostrf(colorHomog, 4, 1, a);
      else strcpy(a, "--.-");
      snprintf_P(t, sizeof t, PSTR("Homogen:%s%%   %u:%02u"), a,
                 (unsigned)(rem / 60), (unsigned)(rem % 60));
      setLine(2, t);
      snprintf_P(t, sizeof t, PSTR("PWM = %d   #=stop"), currentPwm);
      setLine(3, t);
      break;
    }

    // ---------------- HASIL AKHIR ----------------
    case ST_DONE:
      setLine(0, doneTitle);
      setLine(1, doneL1);
      setLine(2, doneL2);
      if (blinkOn()) LP(3, "Tekan tombol apapun");
      else           setLine(3, "");
      break;
  }

  if ((long)(msgUntil - millis()) > 0) setLine(3, lcdMsg);
}

void updateLcd() {
  unsigned long now = millis();
  if (now - tLcd < LCD_MS) return;
  tLcd = now;

  buildLines();

  static uint8_t startRow = 0;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t r = (startRow + i) % 4;
    if (strcmp(lcdBuf[r], lcdShown[r]) != 0) {
      lcd.setCursor(0, r);
      lcd.print(lcdBuf[r]);
      strcpy(lcdShown[r], lcdBuf[r]);
      startRow = (r + 1) % 4;
      break;
    }
  }
}

// ================================================================
// Hitung RPM (window 1 putaran) + EMA
// ================================================================
void updateRpm() {
  unsigned long now = millis();
  if (now - tSample < SAMPLE_MS) return;
  tSample = now;

  noInterrupts();
  unsigned long lastUs = lastPulseUs;
  unsigned long idx    = pulseIdx;
  uint8_t n            = validCnt;
  float   avg          = avgIv;
  uint8_t span         = (n >= 2) ? (n - 1) : 0;
  unsigned long tNew   = pulseTs[idx % RING];
  unsigned long tOld   = (span > 0) ? pulseTs[(idx + RING - span) % RING] : 0;
  interrupts();

  unsigned long elapsed = micros() - lastUs;
  const float K_RPM = 60000000.0 / PULSES_PER_REV;

  if (elapsed > ZERO_TIMEOUT_MS * 1000UL) {
    rpmRaw = 0;
    noInterrupts(); validCnt = 0; avgIv = 0; interrupts();
  }
  else {
    if (span >= 2 && tNew != tOld) {
      rpmRaw = span * K_RPM / (float)(tNew - tOld);
    }
    if (avg > 0 && elapsed > 2.5 * avg) {
      float ub = K_RPM / (float)elapsed;
      if (ub < rpmRaw) rpmRaw = ub;
    }
  }

  if (!emaInit) { rpmEma = rpmRaw; emaInit = true; }
  else          { rpmEma = emaAlpha * rpmRaw + (1.0 - emaAlpha) * rpmEma; }

  if (rpmRaw == 0 && rpmEma < 0.5) rpmEma = 0;

  rpmCal = rpmEma * calFactor;
}

// ================================================================
// Kontrol: Feedforward + PID (mode AUTO)
// ================================================================
void controlLoop() {
  unsigned long now = millis();
  if (now - tPid < PID_MS) return;
  float dt = (now - tPid) / 1000.0;
  tPid = now;

  if (mode != MODE_AUTO) return;

  if (targetRpm <= 0) {
    integralTerm = 0;
    lastRpmCal = rpmCal;
    setPwm(0);
    return;
  }

  float ff    = feedForward();
  float error = targetRpm - rpmCal;

  float deriv = -(rpmCal - lastRpmCal) / dt;
  lastRpmCal = rpmCal;

  float pd  = Kp * error + Kd * deriv;
  float out = ff + pd + integralTerm;

  bool satHigh = (out >= 255 && error > 0);
  bool satLow  = (out <= 0   && error < 0);
  if (!satHigh && !satLow) {
    float kiEff = Ki;
    float ae = fabs(error);
    if (ae < BOOST_BAND && ae > 0.5) kiEff = Ki * KI_BOOST;
    integralTerm += error * kiEff * dt;
    integralTerm = constrain(integralTerm, -I_MAX, I_MAX);
  }

  out = ff + pd + integralTerm;

  int desired = constrain((int)(out + 0.5), 0, 255);
  int next = currentPwm + constrain(desired - currentPwm, -SLEW, SLEW);
  setPwm(next);
}

void showRaw() {
  if (!rawShow) return;
  unsigned long now = millis();
  if (now - tRaw < RAW_MS) return;
  tRaw = now;
  Serial.print(rpmCal, 1);
  Serial.print('\t');
  Serial.print(targetRpm, 1);
  Serial.print('\t');
  Serial.println(currentPwm);
}

// Nama singkat stage untuk telemetri
const __FlashStringHelper* stageName() {
  switch (stage) {
    case ST_IDLE:     return F("IDLE");
    case ST_IN_RPM:   return F("IN_RPM");
    case ST_IN_COFFEE:return F("IN_KOPI");
    case ST_IN_LITER: return F("IN_LITER");
    case ST_FILL_READY:return F("FILL_RDY");
    case ST_TARE:     return F("TARE");
    case ST_FILLING:  return F("FILL");
    case ST_STIR:     return F("STIR");
    case ST_COLOR:    return F("COLOR");
    case ST_DONE:     return F("DONE");
  }
  return F("?");
}

bool isRunning() {
  return stage == ST_TARE || stage == ST_FILLING ||
         stage == ST_STIR || stage == ST_COLOR;
}

// Header sekali tiap mulai running: legenda kolom + setting statis
void monHeader() {
  Serial.println(F("MON: T ST SV PV PWM SISA HOM R G B"));
  Serial.print(F("SET Kp=")); Serial.print(Kp, 2);
  Serial.print(F(" Ki=")); Serial.print(Ki, 2);
  Serial.print(F(" Kd=")); Serial.print(Kd, 2);
  Serial.print(F(" EMA=")); Serial.print(emaAlpha, 2);
  Serial.print(F(" CAL=")); Serial.print(calFactor, 4);
  Serial.print(F(" MAX=")); Serial.println(maxRpm, 1);
}

// Telemetri otomatis tiap 200 ms: mulai saat running (tekan D),
// berhenti sendiri saat selesai. Tanpa perintah.
// Mode STIR: T ST SV PV PWM SISA HOM R G B (tanpa parameter filling).
// Mode FILL (TARE/FILLING): T ST VOL ISI GOAL ZONA Q SISA VALVE
//   VOL=L target | ISI=terisi mL (timer) | GOAL=target tutup mL (target-trim)
//   ZONA=zona aktif | Q=debit zona mL/dtk | SISA=perkiraan sisa waktu dtk
//   VALVE=1 terbuka
void showMon() {
  static Stage monHeadStage = ST_IDLE;
  bool run = isRunning();
  if (run && !wasRunning) { monHeader(); tMon = 0; monHeadStage = ST_IDLE; }
  wasRunning = run;
  if (!run) return;
  bool isFill = (stage == ST_TARE || stage == ST_FILLING);
  Stage wantHead = isFill ? ST_FILLING : ST_STIR;
  if (wantHead != monHeadStage) {
    monHeadStage = wantHead;
    tMon = 0;
    if (isFill) Serial.println(F("MON: T ST VOL ISI GOAL ZONA Q SISA VALVE"));
    else        Serial.println(F("MON: T ST SV PV PWM SISA HOM R G B"));
  }
  unsigned long now = millis();
  if (now - tMon < MON_MS) return;
  tMon = now;
  Serial.print(F("T=")); Serial.print(now / 1000UL);
  Serial.print(F(" ST=")); Serial.print(stageName());
  if (isFill) {
    float q = (fillZone >= 0) ? qZona[fillZone] : 0;
    float sisa = (stage == ST_FILLING && q > 0) ? (fillGoalMl() - dispensedMl) / q : 0;
    if (sisa < 0) sisa = 0;
    Serial.print(F(" VOL=")); Serial.print(fillHalf * 0.5, 1);
    Serial.print(F(" ISI=")); Serial.print(dispensedMl, 0);
    Serial.print(F(" GOAL=")); Serial.print(fillGoalMl(), 2);
    Serial.print(F(" ZONA=")); Serial.print(fillZone + 1);
    Serial.print(F(" Q=")); Serial.print(q, 1);
    Serial.print(F(" SISA=")); Serial.print(sisa, 1);
    Serial.print(F(" VALVE=")); Serial.println(stage == ST_FILLING ? 1 : 0);
    return;
  }
  long sisa = -1;
  if (stage == ST_STIR && stirDur > 0) {
    unsigned long el = now - stirStart;
    sisa = (el >= stirDur) ? 0 : (long)((stirDur - el) / 1000UL);
  }
  Serial.print(F("T=")); Serial.print(now / 1000UL);
  Serial.print(F(" ST=")); Serial.print(stageName());
  Serial.print(F(" SV=")); Serial.print(targetRpm, 1);
  Serial.print(F(" PV=")); Serial.print(rpmCal, 1);
  Serial.print(F(" PWM=")); Serial.print(currentPwm);
  Serial.print(F(" SISA=")); Serial.print(sisa);
  Serial.print(F(" HOM=")); Serial.print(colorHomog, 1);
  Serial.print(F(" R=")); Serial.print(lastR, 0);
  Serial.print(F(" G=")); Serial.print(lastG, 0);
  Serial.print(F(" B=")); Serial.println(lastB, 0);
}

// ================================================================
void setup() {
  Serial.begin(115200);

  valveOff();                                    // level OFF sebelum jadi OUTPUT
  pinMode(PIN_RELAY, OUTPUT);
  valveOff();

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_ENCODER, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);                        // skala frekuensi 20%
  digitalWrite(S1, LOW);

  TCCR1B = (TCCR1B & 0xF8) | 0x02;               // PWM D9 ~3,9 kHz

  loadSettings();
  setPwm(0);

  Wire.begin();
  Wire.setClock(400000);
  keypad.begin();

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0); lcd.print(F("  MIXER KOPI + ISI  "));
  lcd.setCursor(0, 2); lcd.print(F("      Siap...       "));
  delay(1000);
  lcd.clear();
  memset(lcdShown, 0, sizeof lcdShown);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), isrEncoder, RISING);
}

void loop() {
  readSerial();
  readKeypad();
  updateRpm();
  controlLoop();
  processStage();
  showRaw();
  showMon();
  updateLcd();
}
