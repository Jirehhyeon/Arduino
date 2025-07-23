# Lesson 08: 센서 활용 심화

## 🎯 학습 목표
- 다양한 센서의 작동 원리와 특성 이해하기
- 센서 데이터의 정확도와 신뢰성 향상 기법 익히기
- 센서 융합(Sensor Fusion) 기술 구현하기
- 실제 환경에서의 센서 보정과 필터링 적용하기
- IoT 센서 네트워크 구축하기

## 📖 1. 센서 분류와 특성

### 1.1 센서 종류별 분류
```
🌡️ 환경 센서 (Environmental)
- 온도: DS18B20, LM35, DHT22
- 습도: DHT22, SHT30, HIH6130  
- 압력: BMP280, MS5611
- 가스: MQ-2~MQ-9 시리즈
- 조도: BH1750, VEML7700
- UV: VEML6070, SI1145

🎯 위치/움직임 센서 (Motion/Position)
- 거리: HC-SR04, VL53L0X, GP2Y0A21
- 가속도: ADXL345, MPU6050
- 자이로: L3G4200D, MPU6050  
- 지자기: HMC5883L, QMC5883L
- GPS: NEO-6M, NEO-8M

🔊 음향/진동 센서 (Audio/Vibration)
- 마이크: MAX4466, INMP441
- 초음파: HC-SR04, JSN-SR04T
- 진동: SW-420, 압전센서
- 사운드 레벨: 디지털 마이크

⚡ 전기/자기 센서 (Electrical/Magnetic)
- 전류: ACS712, INA219
- 전압: 분압회로, ADS1115
- 홀 센서: A3144, SS49E
- 근접: TCRT5000, 적외선

🌊 화학/바이오 센서 (Chemical/Bio)
- pH: pH4502C, SEN0161
- 수질: TDS, 전도도
- 심박: MAX30102, AD8232
- 지문: FPM10A, GT511C3

🎮 인터페이스 센서 (Interface)
- 터치: TTP223, MPR121
- 제스처: APDS-9960, PAJ7620
- 컬러: TCS3200, TCS34725
- 카메라: OV7670, ESP32-CAM
```

### 1.2 센서 특성 매개변수
```cpp
/*
  센서 성능 지표:
  
  1. 정확도 (Accuracy): 실제값과 측정값의 차이
  2. 정밀도 (Precision): 반복 측정의 일관성
  3. 분해능 (Resolution): 구분 가능한 최소 변화량
  4. 응답시간 (Response Time): 변화 감지 속도
  5. 선형성 (Linearity): 입력-출력 관계의 직선성
  6. 히스테리시스: 접근 방향에 따른 측정값 차이
  7. 드리프트: 시간에 따른 측정값 변화
  8. 노이즈: 불필요한 신호 변동
*/

// 센서 성능 평가 구조체
struct SensorSpecs {
  float accuracy;        // ±% 또는 절대값
  float precision;       // 표준편차
  float resolution;      // 최소 단위
  int responseTime;      // ms
  float linearityError;  // % 비선형성
  float hysteresis;      // % 히스테리시스
  float tempCoeff;       // %/°C 온도 계수  
  int operatingTempMin;  // °C
  int operatingTempMax;  // °C
  float supplyVoltage;   // V
  float powerConsumption; // mW
};
```

## 🌡️ 2. 정밀 온습도 측정 시스템

### 2.1 DHT22 + DS18B20 + BMP280 융합
```cpp
/*
  Multi_Sensor_Weather_Station.ino
  다중 센서 기상 관측소
  
  센서:
  - DHT22: 온습도 (상대습도, 공기 온도)
  - DS18B20: 정밀 온도 (방수, 토양/수온)
  - BMP280: 기압, 고도, 온도
  - BH1750: 조도
*/

#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>

// 센서 핀 정의
#define DHT_PIN 2
#define DHT_TYPE DHT22
#define DS18B20_PIN 3
#define BH1750_SDA A4
#define BH1750_SCL A5

// 센서 객체 생성
DHT dht(DHT_PIN, DHT_TYPE);
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
Adafruit_BMP280 bmp;
BH1750 lightMeter;

// 측정 데이터 구조체
struct WeatherData {
  // 온도 (3개 센서)
  float tempDHT;      // DHT22 온도
  float tempDS18B20;  // DS18B20 온도
  float tempBMP;      // BMP280 온도
  float tempFused;    // 융합된 온도
  
  // 습도
  float humidity;     // 상대습도 (%)
  float humidityAbs;  // 절대습도 (g/m³)
  
  // 기압 및 고도
  float pressure;     // 해면기압 (hPa) 
  float altitude;     // 고도 (m)
  
  // 조도
  float lux;          // 조도 (lux)
  
  // 파생 데이터
  float dewPoint;     // 이슬점 (°C)
  float heatIndex;    // 체감온도 (°C)
  float vapourPressure; // 증기압 (kPa)
  
  // 품질 지표
  float tempVariance; // 온도 센서간 분산
  int validSensors;   // 정상 센서 개수
  unsigned long timestamp; // 측정 시각
};

// 측정 이력 (이동평균용)
const int HISTORY_SIZE = 10;
WeatherData history[HISTORY_SIZE];
int historyIndex = 0;

// 센서 상태
bool sensorStatus[4] = {false, false, false, false}; // DHT, DS18B20, BMP, BH1750

void setup() {
  Serial.begin(115200);
  Serial.println("🌦️ 정밀 기상 관측소 시작!");
  
  // 센서 초기화
  initializeSensors();
  
  // 센서 상태 확인
  checkSensorStatus();
  
  Serial.println("📊 데이터 형식: JSON");
  Serial.println("측정 간격: 10초");
  Serial.println("========================================");
}

void loop() {
  WeatherData currentData;
  
  // 센서 데이터 수집
  if (collectSensorData(&currentData)) {
    
    // 데이터 처리 및 융합
    processSensorData(&currentData);
    
    // 이력에 저장
    saveToHistory(currentData);
    
    // 데이터 출력
    outputData(currentData);
    
    // 품질 검사
    performQualityCheck(currentData);
    
  } else {
    Serial.println("{\"error\": \"데이터 수집 실패\"}");
  }
  
  delay(10000);  // 10초 간격
}

void initializeSensors() {
  Serial.println("🔧 센서 초기화 중...");
  
  // DHT22 초기화
  dht.begin();
  Serial.println("  - DHT22 초기화 완료");
  
  // DS18B20 초기화
  ds18b20.begin();
  Serial.print("  - DS18B20 발견된 센서: ");
  Serial.println(ds18b20.getDeviceCount());
  
  // BMP280 초기화
  if (bmp.begin(0x76)) {  // I2C 주소
    Serial.println("  - BMP280 초기화 완료");
    
    // BMP280 설정
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_X2,  // 온도
                   Adafruit_BMP280::SAMPLING_X16, // 기압
                   Adafruit_BMP280::FILTER_X16,   // 필터
                   Adafruit_BMP280::STANDBY_MS_500);
  } else {
    Serial.println("  ❌ BMP280 초기화 실패");
  }
  
  // BH1750 초기화
  Wire.begin();
  if (lightMeter.begin()) {
    Serial.println("  - BH1750 초기화 완료");
  } else {
    Serial.println("  ❌ BH1750 초기화 실패");
  }
}

void checkSensorStatus() {
  Serial.println("🔍 센서 상태 확인...");
  
  // DHT22 테스트
  float testTemp = dht.readTemperature();
  sensorStatus[0] = !isnan(testTemp);
  Serial.print("  - DHT22: ");
  Serial.println(sensorStatus[0] ? "정상" : "오류");
  
  // DS18B20 테스트  
  ds18b20.requestTemperatures();
  float ds18Temp = ds18b20.getTempCByIndex(0);
  sensorStatus[1] = (ds18Temp != DEVICE_DISCONNECTED_C);
  Serial.print("  - DS18B20: ");
  Serial.println(sensorStatus[1] ? "정상" : "오류");
  
  // BMP280 테스트
  float testPressure = bmp.readPressure();
  sensorStatus[2] = (testPressure > 0);
  Serial.print("  - BMP280: ");
  Serial.println(sensorStatus[2] ? "정상" : "오류");
  
  // BH1750 테스트
  float testLux = lightMeter.readLightLevel();
  sensorStatus[3] = (testLux >= 0);
  Serial.print("  - BH1750: ");
  Serial.println(sensorStatus[3] ? "정상" : "오류");
}

bool collectSensorData(WeatherData* data) {
  data->timestamp = millis();
  data->validSensors = 0;
  
  // DHT22 데이터 읽기
  if (sensorStatus[0]) {
    data->tempDHT = dht.readTemperature();
    data->humidity = dht.readHumidity();
    
    if (!isnan(data->tempDHT) && !isnan(data->humidity)) {
      data->validSensors++;
    } else {
      data->tempDHT = NAN;
      data->humidity = NAN;
      sensorStatus[0] = false;
    }
  }
  
  // DS18B20 데이터 읽기
  if (sensorStatus[1]) {
    ds18b20.requestTemperatures();
    data->tempDS18B20 = ds18b20.getTempCByIndex(0);
    
    if (data->tempDS18B20 != DEVICE_DISCONNECTED_C) {
      data->validSensors++;
    } else {
      data->tempDS18B20 = NAN;
      sensorStatus[1] = false;
    }
  }
  
  // BMP280 데이터 읽기
  if (sensorStatus[2]) {
    data->tempBMP = bmp.readTemperature();
    data->pressure = bmp.readPressure() / 100.0; // Pa to hPa
    data->altitude = bmp.readAltitude(1013.25);   // 해면기압 기준
    
    if (data->pressure > 0) {
      data->validSensors++;
    } else {
      data->tempBMP = NAN;
      data->pressure = NAN;
      data->altitude = NAN;
      sensorStatus[2] = false;
    }
  }
  
  // BH1750 데이터 읽기  
  if (sensorStatus[3]) {
    data->lux = lightMeter.readLightLevel();
    
    if (data->lux >= 0) {
      data->validSensors++;
    } else {
      data->lux = NAN;
      sensorStatus[3] = false;
    }
  }
  
  return (data->validSensors > 0);
}

void processSensorData(WeatherData* data) {
  // 온도 센서 융합 (가중평균)
  float tempSum = 0;
  int tempCount = 0;
  float tempVariance = 0;
  
  if (!isnan(data->tempDHT)) {
    tempSum += data->tempDHT * 1.0;  // DHT22 가중치: 1.0
    tempCount++;
  }
  
  if (!isnan(data->tempDS18B20)) {
    tempSum += data->tempDS18B20 * 1.2;  // DS18B20 가중치: 1.2 (더 정확)
    tempCount++;
  }
  
  if (!isnan(data->tempBMP)) {
    tempSum += data->tempBMP * 0.8;  // BMP280 가중치: 0.8
    tempCount++;
  }
  
  if (tempCount > 0) {
    data->tempFused = tempSum / (tempCount + 0.2 + (tempCount > 1 ? 0.8 : 0) + (tempCount > 2 ? 0.8 : 0));
    
    // 온도 센서간 분산 계산
    float varianceSum = 0;
    int varianceCount = 0;
    
    if (!isnan(data->tempDHT)) {
      varianceSum += pow(data->tempDHT - data->tempFused, 2);
      varianceCount++;
    }
    if (!isnan(data->tempDS18B20)) {
      varianceSum += pow(data->tempDS18B20 - data->tempFused, 2);
      varianceCount++;
    }
    if (!isnan(data->tempBMP)) {
      varianceSum += pow(data->tempBMP - data->tempFused, 2);
      varianceCount++;
    }
    
    data->tempVariance = (varianceCount > 1) ? sqrt(varianceSum / varianceCount) : 0;
    
  } else {
    data->tempFused = NAN;
    data->tempVariance = NAN;
  }
  
  // 파생 데이터 계산
  if (!isnan(data->tempFused) && !isnan(data->humidity)) {
    // 이슬점 계산 (Magnus 공식)
    float a = 17.27;
    float b = 237.7;
    float alpha = ((a * data->tempFused) / (b + data->tempFused)) + log(data->humidity / 100.0);
    data->dewPoint = (b * alpha) / (a - alpha);
    
    // 체감온도 계산 (Heat Index)
    if (data->tempFused >= 27 && data->humidity >= 40) {
      data->heatIndex = calculateHeatIndex(data->tempFused, data->humidity);
    } else {
      data->heatIndex = data->tempFused;
    }
    
    // 절대습도 계산 (g/m³)
    data->humidityAbs = calculateAbsoluteHumidity(data->tempFused, data->humidity);
    
    // 증기압 계산
    data->vapourPressure = calculateVapourPressure(data->tempFused, data->humidity);
    
  } else {
    data->dewPoint = NAN;
    data->heatIndex = NAN;
    data->humidityAbs = NAN;
    data->vapourPressure = NAN;
  }
}

float calculateHeatIndex(float temp, float humidity) {
  // Heat Index 공식 (미국 기상청)
  float T = temp * 9.0/5.0 + 32.0;  // 화씨 변환
  float R = humidity;
  
  float HI = 0.5 * (T + 61.0 + ((T - 68.0) * 1.2) + (R * 0.094));
  
  if (HI >= 80) {
    HI = -42.379 + 2.04901523 * T + 10.14333127 * R 
       - 0.22475541 * T * R - 0.00683783 * T * T
       - 0.05481717 * R * R + 0.00122874 * T * T * R
       + 0.00085282 * T * R * R - 0.00000199 * T * T * R * R;
  }
  
  return (HI - 32.0) * 5.0/9.0;  // 섭씨 변환
}

float calculateAbsoluteHumidity(float temp, float relHumidity) {
  // 절대습도 = (상대습도 × 포화증기압) / (기체상수 × 절대온도)
  float satVapPressure = 6.112 * exp((17.67 * temp) / (temp + 243.5));
  return (relHumidity * satVapPressure) / (0.4615 * (temp + 273.15));
}

float calculateVapourPressure(float temp, float relHumidity) {
  // 증기압 계산 (kPa)
  float satVapPressure = 0.6112 * exp((17.67 * temp) / (temp + 243.5));
  return (relHumidity / 100.0) * satVapPressure;
}

void saveToHistory(WeatherData data) {
  history[historyIndex] = data;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

void outputData(WeatherData data) {
  // JSON 형식으로 출력
  Serial.println("{");
  Serial.print("  \"timestamp\": "); Serial.print(data.timestamp); Serial.println(",");
  Serial.print("  \"validSensors\": "); Serial.print(data.validSensors); Serial.println(",");
  
  // 온도 데이터
  Serial.println("  \"temperature\": {");
  if (!isnan(data.tempDHT)) {
    Serial.print("    \"dht22\": "); Serial.print(data.tempDHT, 2); Serial.println(",");
  }
  if (!isnan(data.tempDS18B20)) {
    Serial.print("    \"ds18b20\": "); Serial.print(data.tempDS18B20, 2); Serial.println(",");
  }
  if (!isnan(data.tempBMP)) {
    Serial.print("    \"bmp280\": "); Serial.print(data.tempBMP, 2); Serial.println(",");
  }
  if (!isnan(data.tempFused)) {
    Serial.print("    \"fused\": "); Serial.print(data.tempFused, 2); Serial.println(",");
    Serial.print("    \"variance\": "); Serial.print(data.tempVariance, 3);
  }
  Serial.println("\n  },");
  
  // 습도 데이터
  if (!isnan(data.humidity)) {
    Serial.println("  \"humidity\": {");
    Serial.print("    \"relative\": "); Serial.print(data.humidity, 1); Serial.println(",");
    if (!isnan(data.humidityAbs)) {
      Serial.print("    \"absolute\": "); Serial.print(data.humidityAbs, 2);
    }
    Serial.println("\n  },");
  }
  
  // 기압 데이터
  if (!isnan(data.pressure)) {
    Serial.println("  \"pressure\": {");
    Serial.print("    \"hPa\": "); Serial.print(data.pressure, 2); Serial.println(",");
    Serial.print("    \"altitude\": "); Serial.print(data.altitude, 1);
    Serial.println("\n  },");
  }
  
  // 조도 데이터
  if (!isnan(data.lux)) {
    Serial.print("  \"light\": "); Serial.print(data.lux, 1); Serial.println(",");
  }
  
  // 파생 데이터
  Serial.println("  \"derived\": {");
  if (!isnan(data.dewPoint)) {
    Serial.print("    \"dewPoint\": "); Serial.print(data.dewPoint, 2); Serial.println(",");
  }
  if (!isnan(data.heatIndex)) {
    Serial.print("    \"heatIndex\": "); Serial.print(data.heatIndex, 2); Serial.println(",");
  }
  if (!isnan(data.vapourPressure)) {
    Serial.print("    \"vapourPressure\": "); Serial.print(data.vapourPressure, 3);
  }
  Serial.println("\n  }");
  Serial.println("}");
  Serial.println();
}

void performQualityCheck(WeatherData data) {
  // 품질 경고
  if (data.tempVariance > 2.0) {
    Serial.println("⚠️ 경고: 온도 센서간 편차가 큽니다 (" + String(data.tempVariance, 2) + "°C)");
  }
  
  if (data.validSensors < 3) {
    Serial.println("⚠️ 경고: 일부 센서가 오프라인입니다 (" + String(data.validSensors) + "/4)");
  }
  
  // 범위 체크
  if (!isnan(data.tempFused) && (data.tempFused < -40 || data.tempFused > 85)) {
    Serial.println("⚠️ 경고: 온도가 측정 범위를 벗어났습니다");
  }
  
  if (!isnan(data.humidity) && (data.humidity < 0 || data.humidity > 100)) {
    Serial.println("⚠️ 경고: 습도가 유효 범위를 벗어났습니다");
  }
}
```

## 🎯 3. 고정밀 거리 측정 시스템

### 3.1 다중 거리 센서 융합
```cpp
/*
  Multi_Distance_Sensor.ino
  초음파 + 레이저 + 적외선 거리 센서 융합
  
  센서:
  - HC-SR04: 초음파 (2cm~400cm)
  - VL53L0X: 레이저 ToF (5cm~200cm)
  - GP2Y0A21: 적외선 (10cm~80cm)
*/

#include <Wire.h>
#include <VL53L0X.h>

// 핀 정의
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;
const int IR_PIN = A0;

// 센서 객체
VL53L0X laser;

// 측정 데이터
struct DistanceReading {
  float ultrasonic;     // 초음파 (cm)
  float laser;          // 레이저 (cm)
  float infrared;       // 적외선 (cm)
  float fused;          // 융합 거리 (cm)
  float confidence;     // 신뢰도 (0-1)
  unsigned long timestamp;
};

// 필터링을 위한 이력
const int FILTER_SIZE = 5;
float ultrasonicHistory[FILTER_SIZE];
float laserHistory[FILTER_SIZE];
float infraredHistory[FILTER_SIZE];
int historyIndex = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("🎯 고정밀 거리 측정 시스템");
  
  // 초음파 센서 핀 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // 레이저 센서 초기화
  Wire.begin();
  if (laser.init()) {
    Serial.println("✅ VL53L0X 레이저 센서 초기화 완료");
    laser.setTimeout(500);
    laser.startContinuous();
  } else {
    Serial.println("❌ VL53L0X 초기화 실패");
  }
  
  // 이력 버퍼 초기화
  for (int i = 0; i < FILTER_SIZE; i++) {
    ultrasonicHistory[i] = 0;
    laserHistory[i] = 0;
    infraredHistory[i] = 0;
  }
  
  Serial.println("시작 준비 완료!");
  Serial.println("거리(cm), 신뢰도, 사용센서");
}

void loop() {
  DistanceReading reading;
  reading.timestamp = millis();
  
  // 각 센서에서 거리 측정
  reading.ultrasonic = measureUltrasonic();
  reading.laser = measureLaser();
  reading.infrared = measureInfrared();
  
  // 데이터 필터링
  filterReadings(&reading);
  
  // 센서 융합
  fuseSensorData(&reading);
  
  // 결과 출력
  outputReading(reading);
  
  delay(100);
}

float measureUltrasonic() {
  // 초음파 측정 (개선된 방법)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms 타임아웃
  
  if (duration == 0) {
    return NAN;  // 측정 실패
  }
  
  float distance = duration * 0.034 / 2;  // 음속: 340m/s
  
  // 유효 범위 체크
  if (distance < 2 || distance > 400) {
    return NAN;
  }
  
  return distance;
}

float measureLaser() {
  if (!laser.init()) {
    return NAN;  // 센서 오류
  }
  
  uint16_t distance = laser.readRangeContinuousMillimeters();
  
  if (laser.timeoutOccurred() || distance >= 8190) {
    return NAN;  // 측정 실패 또는 범위 초과
  }
  
  return distance / 10.0;  // mm to cm
}

float measureInfrared() {
  // GP2Y0A21 적외선 센서 (아날로그)
  int rawValue = analogRead(IR_PIN);
  float voltage = rawValue * (5.0 / 1023.0);
  
  // GP2Y0A21 특성 곡선 (근사식)
  if (voltage < 0.4) {
    return NAN;  // 너무 멀거나 측정 불가
  }
  
  float distance = 27.726 * pow(voltage, -1.2045);
  
  // 유효 범위 체크 (10cm ~ 80cm)
  if (distance < 10 || distance > 80) {
    return NAN;
  }
  
  return distance;
}

void filterReadings(DistanceReading* reading) {
  // 이동평균 필터 적용
  ultrasonicHistory[historyIndex] = reading->ultrasonic;
  laserHistory[historyIndex] = reading->laser;
  infraredHistory[historyIndex] = reading->infrared;
  
  historyIndex = (historyIndex + 1) % FILTER_SIZE;
  
  // 필터링된 값 계산 (이상값 제외한 평균)
  reading->ultrasonic = robustAverage(ultrasonicHistory, FILTER_SIZE);
  reading->laser = robustAverage(laserHistory, FILTER_SIZE);
  reading->infrared = robustAverage(infraredHistory, FILTER_SIZE);
}

float robustAverage(float* array, int size) {
  float validValues[size];
  int validCount = 0;
  
  // NaN이 아닌 값들만 수집
  for (int i = 0; i < size; i++) {
    if (!isnan(array[i])) {
      validValues[validCount++] = array[i];
    }
  }
  
  if (validCount == 0) {
    return NAN;
  }
  
  // 단순 평균 (개선: 중앙값 또는 트림된 평균 가능)
  float sum = 0;
  for (int i = 0; i < validCount; i++) {
    sum += validValues[i];
  }
  
  return sum / validCount;
}

void fuseSensorData(DistanceReading* reading) {
  float weights[3] = {0, 0, 0};
  float weightedSum = 0;
  float totalWeight = 0;
  
  // 각 센서의 신뢰도 계산 및 가중치 설정
  
  // 초음파 센서 (HC-SR04)
  if (!isnan(reading->ultrasonic)) {
    // 거리에 따른 신뢰도 (20-200cm에서 최고)
    if (reading->ultrasonic >= 20 && reading->ultrasonic <= 200) {
      weights[0] = 0.8;
    } else if (reading->ultrasonic >= 10 && reading->ultrasonic <= 300) {
      weights[0] = 0.6;
    } else {
      weights[0] = 0.3;
    }
  }
  
  // 레이저 센서 (VL53L0X)  
  if (!isnan(reading->laser)) {
    // 근거리에서 높은 정확도
    if (reading->laser <= 100) {
      weights[1] = 1.0;
    } else if (reading->laser <= 150) {
      weights[1] = 0.8;
    } else {
      weights[1] = 0.6;
    }
  }
  
  // 적외선 센서 (GP2Y0A21)
  if (!isnan(reading->infrared)) {
    // 중간 거리에서 적당한 정확도
    if (reading->infrared >= 15 && reading->infrared <= 50) {
      weights[2] = 0.7;
    } else {
      weights[2] = 0.4;
    }
  }
  
  // 가중 평균 계산
  if (weights[0] > 0) {
    weightedSum += reading->ultrasonic * weights[0];
    totalWeight += weights[0];
  }
  
  if (weights[1] > 0) {
    weightedSum += reading->laser * weights[1];
    totalWeight += weights[1];
  }
  
  if (weights[2] > 0) {
    weightedSum += reading->infrared * weights[2];
    totalWeight += weights[2];
  }
  
  if (totalWeight > 0) {
    reading->fused = weightedSum / totalWeight;
    reading->confidence = totalWeight / 2.5; // 최대 신뢰도 정규화
    if (reading->confidence > 1.0) reading->confidence = 1.0;
  } else {
    reading->fused = NAN;
    reading->confidence = 0;
  }
}

void outputReading(DistanceReading reading) {
  Serial.print(reading.timestamp);
  Serial.print(",");
  
  // 개별 센서 값
  if (!isnan(reading.ultrasonic)) {
    Serial.print("US:");
    Serial.print(reading.ultrasonic, 1);
  } else {
    Serial.print("US:-");
  }
  Serial.print(",");
  
  if (!isnan(reading.laser)) {
    Serial.print("LS:");
    Serial.print(reading.laser, 1);
  } else {
    Serial.print("LS:-");
  }
  Serial.print(",");
  
  if (!isnan(reading.infrared)) {
    Serial.print("IR:");
    Serial.print(reading.infrared, 1);
  } else {
    Serial.print("IR:-");
  }
  Serial.print(",");
  
  // 융합 결과
  if (!isnan(reading.fused)) {
    Serial.print("융합:");
    Serial.print(reading.fused, 1);
    Serial.print("cm,신뢰도:");
    Serial.print(reading.confidence * 100, 0);
    Serial.print("%");
  } else {
    Serial.print("융합:측정불가");
  }
  
  Serial.println();
}
```

## 🧭 4. 9축 IMU 센서 활용

### 4.1 MPU6050/9250을 이용한 자세 측정
```cpp
/*
  IMU_Attitude_Estimation.ino
  9축 IMU를 이용한 정밀 자세 추정
  
  센서: MPU6050 (6축) 또는 MPU9250 (9축)
  - 3축 가속도계
  - 3축 자이로스코프  
  - 3축 지자기센서 (9250만)
*/

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// 칼만 필터 구조체
struct KalmanFilter {
  float Q_angle;    // 각도 프로세스 노이즈
  float Q_bias;     // 바이어스 프로세스 노이즈
  float R_measure;  // 측정 노이즈
  
  float angle;      // 추정된 각도
  float bias;       // 자이로 바이어스
  float rate;       // 각속도
  
  float P[2][2];    // 오차 공분산 행렬
};

// X, Y축 칼만 필터
KalmanFilter kalmanX, kalmanY;

// 센서 데이터
struct IMUData {
  // 원시 데이터
  int16_t ax, ay, az;     // 가속도
  int16_t gx, gy, gz;     // 자이로
  int16_t mx, my, mz;     // 지자기 (9250만)
  
  // 스케일된 데이터  
  float accelX, accelY, accelZ;  // g
  float gyroX, gyroY, gyroZ;     // deg/s
  float magX, magY, magZ;        // μT
  
  // 자세 각도
  float roll, pitch, yaw;        // deg
  float rollKalman, pitchKalman; // 칼만 필터 결과
  
  // 파생 데이터
  float temperature;             // °C
  unsigned long timestamp;
};

// 필터 상수
const float GYRO_SENSITIVITY = 131.0;    // LSB/(°/s)
const float ACCEL_SENSITIVITY = 16384.0; // LSB/g
const float MAG_SENSITIVITY = 0.6;       // μT/LSB

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("🧭 9축 IMU 자세 측정 시스템");
  
  Wire.begin();
  
  // MPU6050 초기화
  if (mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("✅ MPU6050 초기화 완료");
  } else {
    Serial.println("❌ MPU6050 초기화 실패");
    while(1);
  }
  
  // 센서 보정
  Serial.println("센서 보정 중... (5초간 정지 상태 유지)");
  calibrateSensor();
  
  // 칼만 필터 초기화
  initKalmanFilter(&kalmanX);
  initKalmanFilter(&kalmanY);
  
  Serial.println("Roll, Pitch, Yaw, RollK, PitchK, Temp");
  lastTime = micros();
}

void loop() {
  IMUData data;
  
  // 센서 데이터 읽기
  readIMUData(&data);
  
  // 자세 계산
  calculateAttitude(&data);
  
  // 결과 출력
  outputIMUData(data);
  
  delay(10);  // 100Hz
}

void calibrateSensor() {
  const int CALIB_SAMPLES = 1000;
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;
  
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    Vector rawAccel = mpu.readRawAccel();
    Vector rawGyro = mpu.readRawGyro();
    
    axSum += rawAccel.XAxis;
    aySum += rawAccel.YAxis;
    azSum += rawAccel.ZAxis;
    
    gxSum += rawGyro.XAxis;
    gySum += rawGyro.YAxis;
    gzSum += rawGyro.ZAxis;
    
    delay(5);
  }
  
  // 오프셋 설정 (간단화된 버전)
  mpu.setAccelOffsetX(axSum / CALIB_SAMPLES);
  mpu.setAccelOffsetY(aySum / CALIB_SAMPLES);
  mpu.setAccelOffsetZ((azSum / CALIB_SAMPLES) - 16384); // 1g 보정
  
  mpu.setGyroOffsetX(gxSum / CALIB_SAMPLES);
  mpu.setGyroOffsetY(gySum / CALIB_SAMPLES);
  mpu.setGyroOffsetZ(gzSum / CALIB_SAMPLES);
  
  Serial.println("보정 완료!");
}

void initKalmanFilter(KalmanFilter* kf) {
  kf->Q_angle = 0.001;    // 각도 프로세스 노이즈
  kf->Q_bias = 0.003;     // 바이어스 프로세스 노이즈
  kf->R_measure = 0.03;   // 측정 노이즈
  
  kf->angle = 0;
  kf->bias = 0;
  kf->rate = 0;
  
  kf->P[0][0] = 0;
  kf->P[0][1] = 0;
  kf->P[1][0] = 0;
  kf->P[1][1] = 0;
}

void readIMUData(IMUData* data) {
  data->timestamp = micros();
  
  // 원시 데이터 읽기
  Vector rawAccel = mpu.readRawAccel();
  Vector rawGyro = mpu.readRawGyro();
  
  data->ax = rawAccel.XAxis;
  data->ay = rawAccel.YAxis;
  data->az = rawAccel.ZAxis;
  
  data->gx = rawGyro.XAxis;
  data->gy = rawGyro.YAxis;
  data->gz = rawGyro.ZAxis;
  
  // 스케일 변환
  data->accelX = data->ax / ACCEL_SENSITIVITY;
  data->accelY = data->ay / ACCEL_SENSITIVITY;
  data->accelZ = data->az / ACCEL_SENSITIVITY;
  
  data->gyroX = data->gx / GYRO_SENSITIVITY;
  data->gyroY = data->gy / GYRO_SENSITIVITY;
  data->gyroZ = data->gz / GYRO_SENSITIVITY;
  
  // 온도
  data->temperature = mpu.readTemperature();
}

void calculateAttitude(IMUData* data) {
  unsigned long currentTime = data->timestamp;
  float dt = (currentTime - lastTime) / 1000000.0; // 초 단위
  lastTime = currentTime;
  
  // 가속도계로부터 Roll, Pitch 계산
  float accelRoll = atan2(data->accelY, data->accelZ) * 180.0 / PI;
  float accelPitch = atan2(-data->accelX, sqrt(data->accelY*data->accelY + data->accelZ*data->accelZ)) * 180.0 / PI;
  
  // 자이로스코프 적분으로 각도 추정 (간단한 방법)
  data->roll += data->gyroX * dt;
  data->pitch += data->gyroY * dt;
  data->yaw += data->gyroZ * dt;
  
  // 상보 필터 (Complementary Filter)
  float alpha = 0.98;  // 자이로 가중치
  data->roll = alpha * data->roll + (1 - alpha) * accelRoll;
  data->pitch = alpha * data->pitch + (1 - alpha) * accelPitch;
  
  // 칼만 필터 적용
  data->rollKalman = kalmanFilter(&kalmanX, accelRoll, data->gyroX, dt);
  data->pitchKalman = kalmanFilter(&kalmanY, accelPitch, data->gyroY, dt);
}

float kalmanFilter(KalmanFilter* kf, float newAngle, float newRate, float dt) {
  // 예측 단계
  kf->rate = newRate - kf->bias;
  kf->angle += dt * kf->rate;
  
  // 오차 공분산 예측
  kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
  kf->P[0][1] -= dt * kf->P[1][1];
  kf->P[1][0] -= dt * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt;
  
  // 혁신(Innovation) 계산
  float y = newAngle - kf->angle;
  
  // 혁신 공분산
  float S = kf->P[0][0] + kf->R_measure;
  
  // 칼만 이득
  float K[2];
  K[0] = kf->P[0][0] / S;
  K[1] = kf->P[1][0] / S;
  
  // 상태 업데이트
  kf->angle += K[0] * y;
  kf->bias += K[1] * y;
  
  // 오차 공분산 업데이트
  float P00_temp = kf->P[0][0];
  float P01_temp = kf->P[0][1];
  
  kf->P[0][0] -= K[0] * P00_temp;
  kf->P[0][1] -= K[0] * P01_temp;
  kf->P[1][0] -= K[1] * P00_temp;
  kf->P[1][1] -= K[1] * P01_temp;
  
  return kf->angle;
}

void outputIMUData(IMUData data) {
  Serial.print(data.roll, 2);
  Serial.print(",");
  Serial.print(data.pitch, 2);
  Serial.print(",");
  Serial.print(data.yaw, 2);
  Serial.print(",");
  Serial.print(data.rollKalman, 2);
  Serial.print(",");
  Serial.print(data.pitchKalman, 2);
  Serial.print(",");
  Serial.print(data.temperature, 1);
  Serial.println();
}
```

## 🌐 5. IoT 센서 네트워크

### 5.1 멀티 노드 센서 네트워크
```cpp
/*
  IoT_Sensor_Network.ino
  ESP8266/ESP32 기반 IoT 센서 네트워크
  
  기능:
  - WiFi 연결
  - MQTT 통신
  - JSON 데이터 전송
  - 원격 제어
  - OTA 업데이트
*/

#ifdef ESP32
  #include <WiFi.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// WiFi 설정
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT 설정
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* device_id = "sensor_node_01";

// 토픽 정의
String topic_data = String("sensors/") + device_id + "/data";
String topic_control = String("sensors/") + device_id + "/control";
String topic_status = String("sensors/") + device_id + "/status";

// 센서 설정
#define DHT_PIN 2
#define DHT_TYPE DHT22
#define LIGHT_PIN A0
#define MOTION_PIN 4
#define LED_PIN 5

DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient espClient;
PubSubClient mqtt(espClient);

#ifdef ESP32
  WebServer server(80);
#else
  ESP8266WebServer server(80);
#endif

// 센서 데이터 구조체
struct SensorData {
  float temperature;
  float humidity;
  int lightLevel;
  bool motionDetected;
  int wifiRSSI;
  unsigned long uptime;
  String timestamp;
};

// 시스템 상태
bool systemEnabled = true;
unsigned long lastSensorRead = 0;
unsigned long lastMQTTSend = 0;
unsigned long lastStatusSend = 0;

const unsigned long SENSOR_INTERVAL = 5000;   // 5초
const unsigned long MQTT_INTERVAL = 10000;    // 10초  
const unsigned long STATUS_INTERVAL = 60000;  // 1분

void setup() {
  Serial.begin(115200);
  Serial.println("\n🌐 IoT 센서 네트워크 노드 시작");
  
  // 핀 설정
  pinMode(MOTION_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // 센서 초기화
  dht.begin();
  
  // WiFi 연결
  setupWiFi();
  
  // MQTT 설정
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(mqttCallback);
  
  // 웹서버 설정
  setupWebServer();
  
  // mDNS 설정
  if (MDNS.begin(device_id)) {
    Serial.println("mDNS responder started");
  }
  
  Serial.println("시스템 준비 완료!");
  
  // 시작 메시지 전송
  sendStatusMessage("online");
}

void loop() {
  // WiFi 연결 확인
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }
  
  // MQTT 연결 확인
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  mqtt.loop();
  
  // 웹서버 처리
  server.handleClient();
  
  // 센서 데이터 읽기
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    SensorData data = readSensors();
    processData(data);
    lastSensorRead = millis();
  }
  
  // MQTT 데이터 전송
  if (millis() - lastMQTTSend > MQTT_INTERVAL) {
    sendSensorData();
    lastMQTTSend = millis();
  }
  
  // 상태 메시지 전송
  if (millis() - lastStatusSend > STATUS_INTERVAL) {
    sendStatusMessage("alive");
    lastStatusSend = millis();
  }
  
  delay(100);
}

void setupWiFi() {
  Serial.print("WiFi 연결 중: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi 연결됨! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi 연결 실패!");
  }
}

void setupWebServer() {
  // 루트 페이지
  server.on("/", handleRoot);
  
  // API 엔드포인트
  server.on("/api/data", handleAPIData);
  server.on("/api/status", handleAPIStatus);
  server.on("/api/control", HTTP_POST, handleAPIControl);
  
  server.begin();
  Serial.println("웹서버 시작됨");
}

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>IoT Sensor Node)</title>
    <meta charset='utf-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        .sensor-card { background: #f8f9fa; padding: 15px; margin: 10px 0; border-radius: 5px; }
        .value { font-size: 24px; font-weight: bold; color: #007bff; }
        button { background: #007bff; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; }
        button:hover { background: #0056b3; }
    </style>
</head>
<body>
    <div class='container'>
        <h1>🌐 IoT 센서 노드</h1>
        <div id='status'>연결 중...</div>
        
        <div class='sensor-card'>
            <h3>🌡️ 온도</h3>
            <div class='value' id='temperature'>--°C</div>
        </div>
        
        <div class='sensor-card'>
            <h3>💧 습도</h3>
            <div class='value' id='humidity'>--%</div>
        </div>
        
        <div class='sensor-card'>
            <h3>💡 조도</h3>
            <div class='value' id='light'>--</div>
        </div>
        
        <div class='sensor-card'>
            <h3>🚶 움직임</h3>
            <div class='value' id='motion'>--</div>
        </div>
        
        <button onclick='toggleLED()'>LED 토글</button>
        <button onclick='updateData()'>데이터 갱신</button>
    </div>
    
    <script>
        function updateData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('temperature').textContent = data.temperature.toFixed(1) + '°C';
                    document.getElementById('humidity').textContent = data.humidity.toFixed(1) + '%';
                    document.getElementById('light').textContent = data.lightLevel;
                    document.getElementById('motion').textContent = data.motionDetected ? '감지됨' : '없음';
                    document.getElementById('status').textContent = '마지막 업데이트: ' + new Date().toLocaleTimeString();
                });
        }
        
        function toggleLED() {
            fetch('/api/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({command: 'toggle_led'})
            });
        }
        
        // 자동 갱신
        setInterval(updateData, 5000);
        updateData();
    </script>
</body>
</html>
  )";
  
  server.send(200, "text/html", html);
}

void handleAPIData() {
  SensorData data = readSensors();
  
  DynamicJsonDocument doc(1024);
  doc["temperature"] = data.temperature;
  doc["humidity"] = data.humidity;
  doc["lightLevel"] = data.lightLevel;
  doc["motionDetected"] = data.motionDetected;
  doc["wifiRSSI"] = data.wifiRSSI;
  doc["uptime"] = data.uptime;
  doc["timestamp"] = data.timestamp;
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
}

void handleAPIStatus() {
  DynamicJsonDocument doc(512);
  doc["deviceId"] = device_id;
  doc["systemEnabled"] = systemEnabled;
  doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
  doc["mqttConnected"] = mqtt.connected();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["uptime"] = millis() / 1000;
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
}

void handleAPIControl() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, server.arg("plain"));
    
    String command = doc["command"];
    
    if (command == "toggle_led") {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      server.send(200, "application/json", "{\"result\":\"ok\"}");
      
    } else if (command == "enable_system") {
      systemEnabled = true;
      server.send(200, "application/json", "{\"result\":\"system enabled\"}");
      
    } else if (command == "disable_system") {
      systemEnabled = false;
      server.send(200, "application/json", "{\"result\":\"system disabled\"}");
      
    } else {
      server.send(400, "application/json", "{\"error\":\"unknown command\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"no data\"}");
  }
}

SensorData readSensors() {
  SensorData data;
  
  if (systemEnabled) {
    data.temperature = dht.readTemperature();
    data.humidity = dht.readHumidity();
    data.lightLevel = analogRead(LIGHT_PIN);
    data.motionDetected = digitalRead(MOTION_PIN);
  } else {
    data.temperature = NAN;
    data.humidity = NAN;
    data.lightLevel = 0;
    data.motionDetected = false;
  }
  
  data.wifiRSSI = WiFi.RSSI();
  data.uptime = millis() / 1000;
  data.timestamp = String(millis());
  
  return data;
}

void processData(SensorData data) {
  // 데이터 검증
  if (isnan(data.temperature) || isnan(data.humidity)) {
    Serial.println("⚠️ 센서 읽기 오류");
    return;
  }
  
  // 임계값 확인
  if (data.temperature > 35.0) {
    Serial.println("🌡️ 고온 경고!");
    // MQTT 알림 전송
    sendAlert("high_temperature", data.temperature);
  }
  
  if (data.motionDetected) {
    Serial.println("🚶 움직임 감지!");
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
}

void sendSensorData() {
  if (!mqtt.connected()) return;
  
  SensorData data = readSensors();
  
  DynamicJsonDocument doc(1024);
  doc["deviceId"] = device_id;
  doc["timestamp"] = millis();
  doc["temperature"] = data.temperature;
  doc["humidity"] = data.humidity;
  doc["lightLevel"] = data.lightLevel;
  doc["motionDetected"] = data.motionDetected;
  doc["wifiRSSI"] = data.wifiRSSI;
  doc["uptime"] = data.uptime;
  
  String payload;
  serializeJson(doc, payload);
  
  mqtt.publish(topic_data.c_str(), payload.c_str());
  Serial.println("📡 데이터 전송: " + payload);
}

void sendStatusMessage(String status) {
  if (!mqtt.connected()) return;
  
  DynamicJsonDocument doc(512);
  doc["deviceId"] = device_id;
  doc["status"] = status;
  doc["timestamp"] = millis();
  doc["freeHeap"] = ESP.getFreeHeap();
  
  String payload;
  serializeJson(doc, payload);
  
  mqtt.publish(topic_status.c_str(), payload.c_str());
}

void sendAlert(String type, float value) {
  if (!mqtt.connected()) return;
  
  DynamicJsonDocument doc(512);
  doc["deviceId"] = device_id;
  doc["alertType"] = type;
  doc["value"] = value;
  doc["timestamp"] = millis();
  
  String payload;
  serializeJson(doc, payload);
  
  String alertTopic = String("alerts/") + device_id;
  mqtt.publish(alertTopic.c_str(), payload.c_str());
}

void reconnectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("MQTT 연결 시도...");
    
    if (mqtt.connect(device_id)) {
      Serial.println("연결됨!");
      
      // 제어 토픽 구독
      mqtt.subscribe(topic_control.c_str());
      
      // 온라인 상태 전송
      sendStatusMessage("connected");
      
    } else {
      Serial.print("실패, rc=");
      Serial.print(mqtt.state());
      Serial.println(" 5초 후 재시도");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("MQTT 메시지 수신: " + String(topic) + " = " + message);
  
  // JSON 파싱
  DynamicJsonDocument doc(512);
  deserializeJson(doc, message);
  
  String command = doc["command"];
  
  if (command == "toggle_led") {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
  } else if (command == "enable_system") {
    systemEnabled = true;
    sendStatusMessage("system_enabled");
    
  } else if (command == "disable_system") {
    systemEnabled = false;
    sendStatusMessage("system_disabled");
    
  } else if (command == "restart") {
    sendStatusMessage("restarting");
    delay(1000);
    ESP.restart();
  }
}
```

## ✅ 핵심 정리

1. **센서 융합**을 통해 단일 센서의 한계를 극복할 수 있습니다.
2. **칼만 필터**와 **상보 필터**는 노이즈 제거와 정확도 향상에 필수적입니다.
3. **IoT 네트워크**로 원격 모니터링과 제어가 가능합니다.
4. **데이터 검증**과 **이상치 탐지**로 시스템 신뢰성을 높일 수 있습니다.

## 🚀 다음 단계

축하합니다! 이제 아두이노의 기초부터 고급 기능까지 모든 핵심 내용을 마스터했습니다. 
다음은 실제 프로젝트를 통해 배운 지식을 종합적으로 활용하는 단계입니다.

## 💻 최종 실습 과제

### 과제 1: 스마트 환경 모니터링
1. 다중 센서 융합 기상관측소 구축
2. 웹 기반 실시간 모니터링 대시보드
3. 임계값 초과시 자동 알림 시스템

### 과제 2: 정밀 제어 시스템
1. IMU 기반 자율 균형 로봇
2. 칼만 필터를 적용한 드론 제어
3. 센서 융합 기반 자율주행 시스템

### 과제 3: IoT 통합 플랫폼
1. 다중 노드 센서 네트워크 구축
2. MQTT 브로커를 통한 데이터 수집
3. 머신러닝을 활용한 패턴 분석

---

[← 이전 레슨: 인터럽트와 타이머](../07-interrupts-timers/README.md) | [🏠 메인으로 돌아가기](../../README.md)