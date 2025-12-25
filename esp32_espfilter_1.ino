// Pin Tanımlamaları
#define SERVO_PIN 13
#define CURRENT_SENSOR_PIN 34        // Korona akımı ölçümü
#define DUST_SENSOR_INPUT_PIN 35     // Filtre girişi
#define DUST_SENSOR_OUTPUT_PIN 32    // Filtre çıkışı
#define VOLTAGE_FEEDBACK_PIN 33      // Çıkış voltajı izleme (opsiyonel)
#define LED_BUILTIN 2
// Servo ayarları
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// PID Parametreleri (Korona akımı kontrolü için)
#define TARGET_CURRENT_MIN 0.1       // Minimum korona akımı (mA)
#define TARGET_CURRENT_MAX 5.0       // Maksimum korona akımı (mA)
#define DEFAULT_TARGET_CURRENT 1.5   // Başlangıç hedef akımı (mA)

// Korona voltaj aralığı (tahmini)
#define CORONA_VOLTAGE_MIN 5000      // Minimum çalışma voltajı (V)
#define CORONA_VOLTAGE_MAX 30000     // Maksimum çalışma voltajı (V)

// Güvenlik limitleri
#define MAX_SAFE_CURRENT 4.5         // Maksimum güvenli akım (mA)
#define OVERCURRENT_TIMEOUT 5000     // Aşırı akım süresi (ms)
#include <Wire.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

// Servo nesnesi
Servo voltageControlServo;

// PID Kontrol Değişkenleri (AKIM bazlı)
double pidSetpoint, pidInput, pidOutput;

// PID parametreleri - Korona akım kontrolü için optimize
// DİKKAT: Bu değerler deneme-yanılma ile ayarlanmalı
double Kp = 2.0;    // Oransal kazanç
double Ki = 0.5;    // İntegral kazanç
double Kd = 0.1;    // Türev kazanç

// PID kontrolcü
PID coronaPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Sistem değişkenleri
float coronaCurrent = 0;            // Korona akımı (mA)
float targetCoronaCurrent = DEFAULT_TARGET_CURRENT;
float dustInputConcentration = 0;   // Giriş toz konsantrasyonu
float dustOutputConcentration = 0;  // Çıkış toz konsantrasyonu
float filterEfficiency = 0;         // Filtre verimliliği (%)
int servoAngle = 90;                // Mevcut servo açısı
float estimatedVoltage = 0;         // Tahmini çıkış voltajı

// Kalibrasyon değerleri
const float CURRENT_SENSOR_SENSITIVITY = 0.185; // ACS712 için (V/A)
const float CURRENT_SENSOR_OFFSET = 1.65;       // 2.5V = 0A

// Güvenlik değişkenleri
bool safetyLock = false;
unsigned long overcurrentStartTime = 0;
float readDustSensor(int pin) {
    // GP2Y1010AU0F toz sensörü için okuma fonksiyonu
    
    // LED'i aç
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(280);
    
    // Analog okuma
    int sensorValue = analogRead(pin);
    
    // LED'i kapat
    delayMicroseconds(40);
    digitalWrite(pin, HIGH);
    delayMicroseconds(9680);
    
    // Gerilimi hesapla (ESP32 ADC referansı 3.3V)
    float voltage = sensorValue * (3.3 / 4095.0);
    
    // Toz yoğunluğu hesaplama (μg/m³)
    // GP2Y1010AU0F için formül: dust = 0.17 * voltage - 0.1
    float dustDensity = (0.17 * voltage - 0.1) * 1000;
    
    // Negatif değerleri sıfırla
    if (dustDensity < 0) {
        dustDensity = 0;
    }
    
    return dustDensity;
}
float readCoronaCurrent() {
    // Hassas akım ölçümü için ortalama alma
    const int NUM_SAMPLES = 50;
    long sum = 0;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(CURRENT_SENSOR_PIN);
        delayMicroseconds(100);  // Örnekleme aralığı
    }
    
    float averageADC = sum / (float)NUM_SAMPLES;
    
    // ADC'yi volta çevir (ESP32: 0-3.3V, 0-4095)
    float voltage = (averageADC / 4095.0) * 3.3;
    
    // Akım hesaplama (ACS712 5A)
    // 2.5V = 0A, 0.185V/A sensitivite
    float currentAmps = (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
    
    // mA'ye çevir ve pozitif yap
    float currentmA = abs(currentAmps * 1000.0);
    
    // Düşük seviye gürültü filtresi
    if (currentmA < 0.05) {
        currentmA = 0.0;
    }
    
    return currentmA;
}
float estimateCoronaVoltage() {
    // Korona akımına göre voltaj tahmini
    // Bu formül sisteminize göre kalibre edilmelidir
    
    if (coronaCurrent < 0.1) {
        return 0;  // Korona başlamadı
    }
    
    // Lineer yaklaşım (gerçek değerler kalibrasyonla belirlenmeli)
    // Korona voltajı ≈ sabit + (akım × empedans)
    const float BASE_VOLTAGE = 5000.0;    // Korona başlangıç voltajı
    const float IMPEDANCE_FACTOR = 5000.0; // Sistem empedansı
    
    float estimatedV = BASE_VOLTAGE + (coronaCurrent * IMPEDANCE_FACTOR);
    
    // Maksimum voltaj sınırı
    if (estimatedV > CORONA_VOLTAGE_MAX) {
        estimatedV = CORONA_VOLTAGE_MAX;
    }
    
    return estimatedV;
}
void setupPIDController() {
    // PID parametrelerini ayarla
    pidInput = readCoronaCurrent();
    pidSetpoint = targetCoronaCurrent;
    
    // PID sınırlarını belirle
    coronaPID.SetOutputLimits(SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    coronaPID.SetSampleTime(100);  // 100ms örnekleme
    coronaPID.SetMode(AUTOMATIC);
    
    // PID penceresi modu (opsiyonel, daha stabil kontrol için)
    coronaPID.SetControllerDirection(DIRECT);
    
    Serial.println("PID Kontrolcü başlatıldı");
    Serial.print("Hedef Korona Akımı: ");
    Serial.print(pidSetpoint, 2);
    Serial.println(" mA");
}

void updatePIDControl() {
    // Gerçek zamanlı akım ölçümü
    coronaCurrent = readCoronaCurrent();
    pidInput = coronaCurrent;
    
    // PID hesaplaması
    coronaPID.Compute();
    
    // Servo pozisyonunu güncelle
    servoAngle = (int)pidOutput;
    setServoPosition(servoAngle);
    
    // Voltaj tahmini
    estimatedVoltage = estimateCoronaVoltage();
    
    // Güvenlik kontrolleri
    performSafetyChecks();
    
    // Verimlilik hesaplama
    calculateFilterEfficiency();
}
void setServoPosition(int angle) {
    // Açı sınırlarını kontrol et
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Servo pozisyonunu ayarla
    voltageControlServo.write(angle);
    
    // Pozisyon değişikliği için bekle (servo hızına bağlı)
    delay(20);
    
    // Debug bilgisi
    static int lastAngle = -1;
    if (abs(angle - lastAngle) > 2) {  // 2 dereceden fazla değişimde
        Serial.print("Servo pozisyonu: ");
        Serial.print(angle);
        Serial.print("° -> Tahmini Voltaj: ");
        Serial.print(estimatedVoltage / 1000.0, 1);
        Serial.println(" kV");
        lastAngle = angle;
    }
}

void calibrateServoRange() {
    Serial.println("Servo kalibrasyonu başlıyor...");
    
    // Minimum pozisyon
    Serial.println("Minimum pozisyona gidiliyor...");
    setServoPosition(SERVO_MIN_ANGLE);
    delay(2000);
    
    // Maksimum pozisyon
    Serial.println("Maksimum pozisyona gidiliyor...");
    setServoPosition(SERVO_MAX_ANGLE);
    delay(2000);
    
    // Orta nokta
    Serial.println("Orta pozisyona gidiliyor...");
    setServoPosition((SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) / 2);
    delay(1000);
    
    Serial.println("Servo kalibrasyonu tamamlandı.");
}
void readDustSensors() {
    // Giriş toz sensörü
    dustInputConcentration = readDustSensor(DUST_SENSOR_INPUT_PIN);
    
    // Çıkış toz sensörü
    dustOutputConcentration = readDustSensor(DUST_SENSOR_OUTPUT_PIN);
    
    // Kısa süreli filtreleme (ani değişimleri yumuşat)
    static float filteredInput = 0;
    static float filteredOutput = 0;
    const float FILTER_FACTOR = 0.1;
    
    filteredInput = (FILTER_FACTOR * dustInputConcentration) + 
                   ((1 - FILTER_FACTOR) * filteredInput);
    filteredOutput = (FILTER_FACTOR * dustOutputConcentration) + 
                    ((1 - FILTER_FACTOR) * filteredOutput);
    
    dustInputConcentration = filteredInput;
    dustOutputConcentration = filteredOutput;
}

float calculateFilterEfficiency() {
    readDustSensors();
    
    // Verimlilik hesaplama
    if (dustInputConcentration <= 0) {
        filterEfficiency = 100.0;  // Girişte toz yok
    } else {
        filterEfficiency = 100.0 * (1.0 - (dustOutputConcentration / dustInputConcentration));
        
        // Sınırları koru
        filterEfficiency = constrain(filterEfficiency, 0.0, 100.0);
    }
    
    return filterEfficiency;
}

void adjustTargetCurrentBasedOnEfficiency() {
    // Verimliliğe göre hedef akımı otomatik ayarla
    static unsigned long lastAdjustment = 0;
    
    if (millis() - lastAdjustment > 30000) {  // Her 30 saniyede bir
        lastAdjustment = millis();
        
        if (filterEfficiency < 70.0 && targetCoronaCurrent < TARGET_CURRENT_MAX) {
            // Verimlilik düşük, akımı artır
            targetCoronaCurrent = min(targetCoronaCurrent * 1.2, TARGET_CURRENT_MAX);
            pidSetpoint = targetCoronaCurrent;
            Serial.println("Verimlilik düşük -> Akım artırıldı");
        }
        else if (filterEfficiency > 95.0 && targetCoronaCurrent > TARGET_CURRENT_MIN * 1.5) {
            // Verimlilik çok yüksek, akımı azalt (enerji tasarrufu)
            targetCoronaCurrent = max(targetCoronaCurrent * 0.8, TARGET_CURRENT_MIN);
            pidSetpoint = targetCoronaCurrent;
            Serial.println("Yüksek verimlilik -> Akım azaltıldı (tasarruf)");
        }
    }
}
void performSafetyChecks() {
    // Aşırı akım kontrolü
    if (coronaCurrent > MAX_SAFE_CURRENT) {
        if (overcurrentStartTime == 0) {
            overcurrentStartTime = millis();
            Serial.println("UYARI: Aşırı akım tespit edildi!");
        }
        
        // Aşırı akım süresi kontrolü
        if (millis() - overcurrentStartTime > OVERCURRENT_TIMEOUT) {
            emergencyShutdown("Uzun süreli aşırı akım!");
        }
    } else {
        overcurrentStartTime = 0;  // Sıfırla
    }
    
    // Ark (spark) tespiti (hızlı akım artışı)
    static float lastCurrent = 0;
    float currentRiseRate = abs(coronaCurrent - lastCurrent);
    
    if (currentRiseRate > 2.0) {  // 2mA/100ms'den fazla artış
        Serial.println("UYARI: Hızlı akım artışı! Olası ark tespiti.");
        // Servoyu geri çek (voltajı düşür)
        setServoPosition(servoAngle - 20);
    }
    lastCurrent = coronaCurrent;
    
    // Voltaj sınırı kontrolü
    if (estimatedVoltage > CORONA_VOLTAGE_MAX) {
        emergencyShutdown("Aşırı voltaj tespit edildi!");
    }
}

void emergencyShutdown(String reason) {
    Serial.println("ACİL DURUM: " + reason);
    Serial.println("Sistem kapatılıyor...");
    
    // Servoyu minimum pozisyona getir (voltajı sıfırla)
    setServoPosition(SERVO_MIN_ANGLE);
    
    // PID'yi durdur
    coronaPID.SetMode(MANUAL);
    
    // Güvenlik kilidi
    safetyLock = true;
    
    // Alarm ver (LED veya buzzer)
    for (int i = 0; i < 10; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
    
    Serial.println("Lütfen sistemi kontrol edin ve yeniden başlatın.");
}
void setupManualControl() {
    // Potansiyometre veya butonlarla manuel kontrol
    // Bu fonksiyon setup() içinde çağrılır
    pinMode(34, INPUT);  // Potansiyometre için
}

void checkManualOverride() {
    // Manuel kontrol modu için
    int potValue = analogRead(34);
    
    if (potValue < 100) {
        // Otomatik mod
        coronaPID.SetMode(AUTOMATIC);
        Serial.println("Otomatik mod aktif");
    } 
    else if (potValue > 4000) {
        // Manuel mod - sabit voltaj
        coronaPID.SetMode(MANUAL);
        setServoPosition(120);  // Sabit pozisyon
        Serial.println("Manuel mod aktif - Sabit voltaj");
    }
    else {
        // Manuel mod - pot ile kontrol
        coronaPID.SetMode(MANUAL);
        int manualAngle = map(potValue, 100, 4000, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        setServoPosition(manualAngle);
        
        static int lastManualAngle = -1;
        if (manualAngle != lastManualAngle) {
            Serial.print("Manuel kontrol: ");
            Serial.print(manualAngle);
            Serial.println("°");
            lastManualAngle = manualAngle;
        }
    }
}
void displaySystemStatus() {
    static unsigned long lastDisplay = 0;
    
    if (millis() - lastDisplay > 1000) {  // Her 1 saniyede
        lastDisplay = millis();
        
        Serial.println("\n=== KORONA FİLTRE SİSTEM DURUMU ===");
        Serial.print("Korona Akımı: ");
        Serial.print(coronaCurrent, 2);
        Serial.print(" mA / Hedef: ");
        Serial.print(targetCoronaCurrent, 2);
        Serial.println(" mA");
        
        Serial.print("Tahmini Voltaj: ");
        Serial.print(estimatedVoltage / 1000.0, 1);
        Serial.println(" kV");
        
        Serial.print("Servo Pozisyonu: ");
        Serial.print(servoAngle);
        Serial.println("°");
        
        Serial.print("Filtre Verimliliği: ");
        Serial.print(filterEfficiency, 1);
        Serial.println(" %");
        
        Serial.print("Giriş Tozu: ");
        Serial.print(dustInputConcentration, 0);
        Serial.print(" μg/m³ | Çıkış Tozu: ");
        Serial.print(dustOutputConcentration, 0);
        Serial.println(" μg/m³");
        
        Serial.print("PID Çıkış: ");
        Serial.println(pidOutput, 1);
        
        if (safetyLock) {
            Serial.println("!!! GÜVENLİK KİLİDİ AKTİF !!!");
        }
        
        Serial.println("==================================\n");
    }
}

void logSystemData() {
    // SD kart veya seri porta veri kaydı
    static unsigned long lastLog = 0;
    
    if (millis() - lastLog > 5000) {  // Her 5 saniyede
        lastLog = millis();
        
        // CSV formatında kayıt
        Serial.print("DATA,");
        Serial.print(millis() / 1000.0, 1);
        Serial.print(",");
        Serial.print(coronaCurrent, 3);
        Serial.print(",");
        Serial.print(targetCoronaCurrent, 2);
        Serial.print(",");
        Serial.print(estimatedVoltage, 0);
        Serial.print(",");
        Serial.print(servoAngle);
        Serial.print(",");
        Serial.print(dustInputConcentration, 1);
        Serial.print(",");
        Serial.print(dustOutputConcentration, 1);
        Serial.print(",");
        Serial.print(filterEfficiency, 1);
        Serial.print(",");
        Serial.println(safetyLock ? "1" : "0");
    }
}
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("KORONA DEŞARJLI HAVA FİLTRESİ SİSTEMİ");
    Serial.println("=====================================");
    
    // Servo motoru başlat
    voltageControlServo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    
    // Kalibrasyon
    calibrateServoRange();
    
    // PID kontrolcüyü başlat
    setupPIDController();
    
    // Manuel kontrol için
    setupManualControl();
    
    // Başlangıç pozisyonu (güvenli başlangıç)
    setServoPosition(45);  // Düşük voltajda başla
    
    Serial.println("\nSistem başlatıldı. Korona akımı kontrolü aktif.");
    Serial.println("DİKKAT: Yüksek voltaj tehlikesi! Güvenlik önlemlerini alın.");
}
void loop() {
    static unsigned long lastPIDUpdate = 0;
    static unsigned long lastEfficiencyUpdate = 0;
    
    // PID güncelleme (100ms'de bir)
    if (millis() - lastPIDUpdate >= 100 && !safetyLock) {
        lastPIDUpdate = millis();
        updatePIDControl();
    }
    
    // Verimlilik hesaplama (2 saniyede bir)
    if (millis() - lastEfficiencyUpdate >= 2000) {
        lastEfficiencyUpdate = millis();
        calculateFilterEfficiency();
        
        // Verimliliğe göre otomatik ayar (opsiyonel)
        adjustTargetCurrentBasedOnEfficiency();
    }
    
    // Manuel kontrol kontrolü
    checkManualOverride();
    
    // Sistem durumunu göster
    displaySystemStatus();
    
    // Veri kaydı
    logSystemData();
    
    // Güvenlik kilidi kontrolü
    if (safetyLock) {
        // Kilidi sadece yeniden başlatma ile kaldır
        delay(1000);
    }
}
void runCalibration() {
    Serial.println("\n=== SİSTEM KALİBRASYONU ===");
    
    // 1. Servo kalibrasyonu
    calibrateServoRange();
    
    // 2. Akım sensörü kalibrasyonu
    Serial.println("Akım sensörü kalibrasyonu...");
    Serial.println("Lütfen yük bağlı değilken bekleyin (3 sn)...");
    delay(3000);
    
    float zeroCurrent = readCoronaCurrent();
    Serial.print("Sıfır akım okuması: ");
    Serial.print(zeroCurrent, 3);
    Serial.println(" mA");
    
    // 3. Voltaj-akım karakteristiği çıkarma
    Serial.println("\nVoltaj-akım karakteristiği çıkarılıyor...");
    Serial.println("DİKKAT: Bu işlem yüksek voltaj üretecektir!");
    Serial.println("3 saniye içinde başlayacak...");
    delay(3000);
    
    for (int angle = 30; angle <= 150; angle += 10) {
        setServoPosition(angle);
        delay(2000);  // Kararlı duruma gelmesini bekle
        
        float current = readCoronaCurrent();
        float voltage = estimateCoronaVoltage();
        
        Serial.print("Açı: ");
        Serial.print(angle);
        Serial.print("° -> Akım: ");
        Serial.print(current, 2);
        Serial.print(" mA, Tahmini Voltaj: ");
        Serial.print(voltage / 1000.0, 1);
        Serial.println(" kV");
        
        // Güvenlik kontrolü
        if (current > MAX_SAFE_CURRENT) {
            Serial.println("Aşırı akım! Kalibrasyon durduruluyor.");
            break;
        }
    }
    
    // Güvenli pozisyona dön
    setServoPosition(45);
    
    Serial.println("Kalibrasyon tamamlandı.");
}
