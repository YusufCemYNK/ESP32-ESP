#include <Servo.h>
#include <PID_v1.h>

/*
  Arduino UNO - Elektrostatik Filtre Kontrolü
  -------------------------------------------------
  Algoritma-1:
    - ACS712 (5A) ile trafo akımı okunur.
    - 2 adet GP2Y1010AU0F ile giriş/çıkış toz değerleri okunur.
    - Filtre verimine göre dinamik istenen akım (PID setpoint) üretilir.

  Algoritma-2:
    - PID, ölçülen akımı hedef akıma göre kontrol eder ve servo açısını üretir.
    - Servo 0..180° -> Trafo potansiyometresi 90..270° kabul edilir.
    - Bu dönüşüm ile yaklaşık 8kV..15kV aralığı ayarlanır.
    - Ark algılanırsa servo hızlıca geri çekilir, bir süre beklenir,
      sonra yavaşça ark eşiğine doğru tekrar artırılır.
*/

// ------------------------ Pin Tanımları (Arduino UNO) ------------------------
const uint8_t SERVO_PIN = 9;

const uint8_t ACS712_PIN = A0;

const uint8_t DUST1_LED_PIN = 2;
const uint8_t DUST1_ANALOG_PIN = A1; // giriş tozu

const uint8_t DUST2_LED_PIN = 3;
const uint8_t DUST2_ANALOG_PIN = A2; // çıkış tozu

const uint8_t STATUS_LED_PIN = LED_BUILTIN;

// ------------------------ Sistem Parametreleri ------------------------
const float ACS712_VREF = 5.0f;        // UNO ADC referansı
const float ACS712_SENSITIVITY = 0.185f; // V/A (ACS712 5A)
const float ACS712_ZERO_VOLT = 2.5f;   // 0A offset

// Trafo/servo modeli
const float VOLTAGE_MIN_KV = 8.0f;   // Pot 90° civarı
const float VOLTAGE_MAX_KV = 15.0f;  // Pot 270° civarı
const int SERVO_MIN_DEG = 0;
const int SERVO_MAX_DEG = 180;

// 15kV'ta yaklaşık 10mA bilgisine göre hedef aralık
const float CURRENT_MIN_MA = 0.5f;
const float CURRENT_MAX_MA = 10.0f;
const float CURRENT_DEFAULT_MA = 4.0f;

// PID parametreleri (başlangıç değeri, sahada tuning gerektirir)
double Kp = 8.5;
double Ki = 1.2;
double Kd = 0.35;

// Güvenlik / ark yönetimi
const float ARC_SPIKE_MA = 1.2f;           // örnekler arası ani artış eşiği
const float HARD_OVERCURRENT_MA = 11.0f;   // mutlak güvenlik sınırı
const uint8_t ARC_DROP_DEG = 20;           // arklanmada geri çekme
const unsigned long ARC_HOLD_MS = 2500UL;  // geri çekildikten sonra bekleme
const unsigned long ARC_RECOVER_STEP_MS = 250UL;
const uint8_t ARC_RECOVER_STEP_DEG = 1;

// Zamanlayıcılar
const unsigned long CONTROL_PERIOD_MS = 100UL;
const unsigned long STATUS_PERIOD_MS = 1000UL;
const unsigned long EFFICIENCY_PERIOD_MS = 500UL;

// ------------------------ Global Değişkenler ------------------------
Servo hvServo;

// PID değişkenleri
// input: ölçülen akım (mA)
// setpoint: istenen akım (mA)
// output: servo açısı (0..180)
double pidInput = 0.0;
double pidSetpoint = CURRENT_DEFAULT_MA;
double pidOutput = 0.0;

PID currentPid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

float measuredCurrentmA = 0.0f;
float desiredCurrentmA = CURRENT_DEFAULT_MA;

float dustIn = 0.0f;
float dustOut = 0.0f;
float efficiencyPct = 0.0f;

int servoAngleDeg = 20;
int servoAngleBeforeArc = 20;

bool arcRecoveryActive = false;
bool emergencyStop = false;
unsigned long arcEventTime = 0;
unsigned long lastRecoverStepTime = 0;

unsigned long lastControlTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastEfficiencyTime = 0;

// ------------------------ Yardımcı Fonksiyonlar ------------------------
float adcToVoltage(int adcValue) {
  return (adcValue * ACS712_VREF) / 1023.0f;
}

float readCurrentmA() {
  const uint8_t N = 64;
  long total = 0;

  for (uint8_t i = 0; i < N; i++) {
    total += analogRead(ACS712_PIN);
    delayMicroseconds(200);
  }

  float avgAdc = total / (float)N;
  float sensorVolt = adcToVoltage((int)avgAdc);
  float currentA = (sensorVolt - ACS712_ZERO_VOLT) / ACS712_SENSITIVITY;
  float currentmA = fabs(currentA) * 1000.0f;

  if (currentmA < 40.0f) currentmA = 0.0f; // düşük seviyede gürültü bastırma
  return currentmA;
}

float readDustRawUgM3(uint8_t ledPin, uint8_t analogPin) {
  // GP2Y1010AU0F tipik okuma zamanlaması
  digitalWrite(ledPin, LOW);
  delayMicroseconds(280);

  int adc = analogRead(analogPin);

  delayMicroseconds(40);
  digitalWrite(ledPin, HIGH);
  delayMicroseconds(9680);

  float voltage = adcToVoltage(adc);
  // Yaklaşık model: Dust(mg/m3) = 0.17*V - 0.1
  float mgm3 = (0.17f * voltage) - 0.10f;
  if (mgm3 < 0) mgm3 = 0;

  return mgm3 * 1000.0f; // ug/m3
}

void updateDustAndEfficiency() {
  static float inFiltered = 0.0f;
  static float outFiltered = 0.0f;
  const float alpha = 0.2f;

  float inNow = readDustRawUgM3(DUST1_LED_PIN, DUST1_ANALOG_PIN);
  float outNow = readDustRawUgM3(DUST2_LED_PIN, DUST2_ANALOG_PIN);

  inFiltered = (alpha * inNow) + ((1.0f - alpha) * inFiltered);
  outFiltered = (alpha * outNow) + ((1.0f - alpha) * outFiltered);

  dustIn = inFiltered;
  dustOut = outFiltered;

  if (dustIn < 1.0f) {
    efficiencyPct = 100.0f;
  } else {
    efficiencyPct = 100.0f * (1.0f - (dustOut / dustIn));
    efficiencyPct = constrain(efficiencyPct, 0.0f, 100.0f);
  }
}

float computeDesiredCurrentFromEfficiency(float measuredCurrent, float effPct) {
  // Algoritma-1:
  // - Verim düşükse hedef akım yükseltilir.
  // - Verim çok yüksekse enerji için biraz düşürülür.
  // - Ölçülen akım bilgisi ile hedef yumuşatılır.

  float target = desiredCurrentmA;

  if (effPct < 75.0f) {
    target += 0.35f;
  } else if (effPct > 96.0f) {
    target -= 0.20f;
  }

  // Mevcut akım hedefin çok altındaysa biraz daha agresif ol
  if (measuredCurrent < (target - 0.8f)) {
    target += 0.15f;
  }

  target = constrain(target, CURRENT_MIN_MA, CURRENT_MAX_MA);
  return target;
}

float servoToKv(int servoDeg) {
  float ratio = (servoDeg - SERVO_MIN_DEG) / float(SERVO_MAX_DEG - SERVO_MIN_DEG);
  return VOLTAGE_MIN_KV + ratio * (VOLTAGE_MAX_KV - VOLTAGE_MIN_KV);
}

float servoToPotDeg(int servoDeg) {
  // İstenen dönüşüm: servo 0° -> pot 90°, servo 180° -> pot 270°
  return 90.0f + servoDeg;
}

void setServoSafe(int angleDeg) {
  servoAngleDeg = constrain(angleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  hvServo.write(servoAngleDeg);
}

void triggerArcProtection() {
  servoAngleBeforeArc = servoAngleDeg;
  setServoSafe(servoAngleDeg - ARC_DROP_DEG);
  arcEventTime = millis();
  lastRecoverStepTime = millis();
  arcRecoveryActive = true;

  Serial.println(F("[ARK] Ark algilandi, servo geri cekildi."));
}

void processArcRecovery() {
  if (!arcRecoveryActive) return;

  unsigned long now = millis();

  if ((now - arcEventTime) < ARC_HOLD_MS) {
    return; // bekleme süresi
  }

  if ((now - lastRecoverStepTime) >= ARC_RECOVER_STEP_MS) {
    lastRecoverStepTime = now;

    if (servoAngleDeg < servoAngleBeforeArc) {
      setServoSafe(servoAngleDeg + ARC_RECOVER_STEP_DEG);
    } else {
      arcRecoveryActive = false;
      Serial.println(F("[ARK] Kademeli geri donus tamamlandi."));
    }
  }
}

void checkHardSafety(float currentNow, float deltaCurrent) {
  if (currentNow >= HARD_OVERCURRENT_MA) {
    emergencyStop = true;
    setServoSafe(SERVO_MIN_DEG);
    currentPid.SetMode(MANUAL);
    Serial.println(F("[ACIL] Asiri akim! Sistem durduruldu."));
    return;
  }

  if (deltaCurrent > ARC_SPIKE_MA && !arcRecoveryActive) {
    triggerArcProtection();
  }
}

void printStatus() {
  Serial.println(F("----------------------"));
  Serial.print(F("I_olculen (mA): "));
  Serial.println(measuredCurrentmA, 2);

  Serial.print(F("I_hedef   (mA): "));
  Serial.println(desiredCurrentmA, 2);

  Serial.print(F("PID out servo(deg): "));
  Serial.println(servoAngleDeg);

  Serial.print(F("Pot tahmini (deg): "));
  Serial.println(servoToPotDeg(servoAngleDeg), 1);

  Serial.print(F("HV tahmini  (kV): "));
  Serial.println(servoToKv(servoAngleDeg), 2);

  Serial.print(F("Toz Giris/ Cikis (ug/m3): "));
  Serial.print(dustIn, 1);
  Serial.print(F(" / "));
  Serial.println(dustOut, 1);

  Serial.print(F("Verim (%): "));
  Serial.println(efficiencyPct, 1);

  Serial.print(F("Arc recovery: "));
  Serial.println(arcRecoveryActive ? F("AKTIF") : F("pasif"));
}

void setup() {
  Serial.begin(115200);

  pinMode(DUST1_LED_PIN, OUTPUT);
  pinMode(DUST2_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  digitalWrite(DUST1_LED_PIN, HIGH);
  digitalWrite(DUST2_LED_PIN, HIGH);

  hvServo.attach(SERVO_PIN);
  setServoSafe(20);

  currentPid.SetOutputLimits(SERVO_MIN_DEG, SERVO_MAX_DEG);
  currentPid.SetSampleTime(CONTROL_PERIOD_MS);
  currentPid.SetMode(AUTOMATIC);

  Serial.println(F("UNO ESP filtre kontrolu basladi."));
  Serial.println(F("Servo 0..180 = Pot 90..270, KV ~8..15"));
}

void loop() {
  unsigned long now = millis();
  static float lastCurrent = 0.0f;

  if (!emergencyStop && (now - lastEfficiencyTime >= EFFICIENCY_PERIOD_MS)) {
    lastEfficiencyTime = now;
    updateDustAndEfficiency();
    desiredCurrentmA = computeDesiredCurrentFromEfficiency(measuredCurrentmA, efficiencyPct);
    pidSetpoint = desiredCurrentmA;
  }

  if (!emergencyStop && (now - lastControlTime >= CONTROL_PERIOD_MS)) {
    lastControlTime = now;

    measuredCurrentmA = readCurrentmA();
    pidInput = measuredCurrentmA;

    float deltaCurrent = fabs(measuredCurrentmA - lastCurrent);
    lastCurrent = measuredCurrentmA;

    checkHardSafety(measuredCurrentmA, deltaCurrent);

    if (!emergencyStop) {
      currentPid.Compute();
      if (!arcRecoveryActive) {
        setServoSafe((int)pidOutput);
      }
    }
  }

  if (!emergencyStop) {
    processArcRecovery();
  }

  digitalWrite(STATUS_LED_PIN, emergencyStop ? ((now / 150) % 2) : ((now / 1000) % 2));

  if (now - lastStatusTime >= STATUS_PERIOD_MS) {
    lastStatusTime = now;
    printStatus();
  }
}
