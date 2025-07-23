# Lesson 11: I2C와 SPI 통신 마스터

## 🎯 학습 목표
- I2C와 SPI 통신 프로토콜의 차이점과 특징 이해
- I2C 마스터-슬레이브 통신 구현 및 멀티 디바이스 제어
- SPI 고속 통신 및 디바이스 체이닝 기법
- 실시간 센서 네트워크 및 데이터 수집 시스템 구축
- 통신 오류 처리 및 성능 최적화 기법

## 📚 이론 배경

### 통신 프로토콜 비교

```
┌─────────────────────────────────────────────────────────────┐
│                    통신 프로토콜 비교표                        │
├─────────────┬─────────────┬─────────────┬─────────────────┤
│   특성      │    I2C      │    SPI      │     UART        │
├─────────────┼─────────────┼─────────────┼─────────────────┤
│ 연결 방식   │   2선 버스   │   4선 버스   │    2선 P2P     │
│ 최대 속도   │   400kHz    │   수십MHz   │   115200bps    │
│ 디바이스수  │   128개     │   제한없음   │      2개       │
│ 하드웨어복잡│    단순     │    중간     │     가장단순    │
│ 전력소모    │    낮음     │    중간     │      낮음      │
│ 용도        │ 센서, LCD   │ 디스플레이   │  디버깅, GPS   │
└─────────────┴─────────────┴─────────────┴─────────────────┘
```

### I2C (Inter-Integrated Circuit) 프로토콜

I2C는 Phillips(현 NXP)에서 개발한 2선 시리얼 통신 프로토콜입니다.

```
I2C 버스 구조:
       +5V
        │
    ┌───┴───┐ 4.7kΩ 풀업 저항
    │       │
SDA ├───────┼─────────────────────────────
    │       │     │         │         │
SCL ├───────┼─────┼─────────┼─────────┼───
    │       │     │         │         │
   Master  Slave1 Slave2   Slave3   ...
  (Arduino) (센서) (LCD)   (EEPROM)

신호 설명:
- SDA (Serial Data): 데이터 전송 라인
- SCL (Serial Clock): 클럭 신호 라인
- 풀업 저항: 오픈 드레인 구조를 위한 필수 요소
```

#### I2C 통신 프로토콜 상세

```cpp
// I2C 프레임 구조
/*
START + ADDRESS(7bit) + R/W(1bit) + ACK + DATA(8bit) + ACK + ... + STOP

예시: 온도센서(0x48)에서 데이터 읽기
S 0100100 0 A D7D6D5D4D3D2D1D0 A P
│    │    │ │        │         │ │
│    │    │ │        │         │ └─ STOP
│    │    │ │        │         └─── ACK
│    │    │ │        └─────────────── 8비트 데이터
│    │    │ └──────────────────────── ACK
│    │    └─────────────────────────── R/W (0=쓰기, 1=읽기)
│    └──────────────────────────────── 7비트 주소
└───────────────────────────────────── START 조건
*/

// I2C 주소 스캐닝 함수
void scanI2CDevices() {
  Serial.println("I2C 디바이스 스캔 중...");
  int deviceCount = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("디바이스 발견: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  Serial.print("총 ");
  Serial.print(deviceCount);
  Serial.println("개 디바이스 발견");
}
```

### SPI (Serial Peripheral Interface) 프로토콜

SPI는 Motorola에서 개발한 고속 동기식 시리얼 통신 프로토콜입니다.

```
SPI 연결 구조:
              Master (Arduino)
           ┌─────────────────────┐
    MOSI ──┤ Pin 11              │
    MISO ──┤ Pin 12              │
    SCK  ──┤ Pin 13              │
    SS1  ──┤ Pin 10              │
    SS2  ──┤ Pin 9               │
           └─────────────────────┘
               │  │  │  │  │
               │  │  │  │  └──── SS2 (Slave Select 2)
               │  │  │  └─────── SS1 (Slave Select 1)
               │  │  └────────── SCK (Serial Clock)
               │  └───────────── MISO (Master In Slave Out)
               └──────────────── MOSI (Master Out Slave In)
                  │  │  │  │
            ┌─────┴──┴──┴──┴─────┐    ┌─────────────────┐
            │     Slave 1        │    │    Slave 2     │
            │   (SD Card)        │    │  (TFT Display) │
            └────────────────────┘    └─────────────────┘

신호 설명:
- MOSI: 마스터 → 슬레이브 데이터 전송
- MISO: 슬레이브 → 마스터 데이터 전송  
- SCK: 클럭 신호 (마스터가 생성)
- SS: 슬레이브 선택 (Low Active)
```

## 🔧 실습 1: I2C 센서 네트워크 구축

### 다중 I2C 센서 시스템

```cpp
/*
  I2C_Sensor_Network.ino
  I2C를 통한 다중 센서 데이터 수집 시스템
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <Adafruit_BMP280.h>

// I2C 디바이스 주소
#define LCD_ADDRESS     0x27
#define RTC_ADDRESS     0x68
#define BMP280_ADDRESS  0x76
#define EEPROM_ADDRESS  0x50

// 디바이스 객체
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
RTC_DS1307 rtc;
Adafruit_BMP280 bmp;

// 센서 데이터 구조체
struct SensorData {
  float temperature;
  float pressure;
  float altitude;
  DateTime timestamp;
  bool isValid;
} sensorReading;

// 데이터 로깅 설정
const int EEPROM_DATA_START = 0;
const int MAX_RECORDS = 100;
int currentRecord = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();  // I2C 마스터로 초기화
  
  Serial.println("I2C 센서 네트워크 초기화...");
  
  // 각 디바이스 초기화
  initializeDevices();
  
  // I2C 디바이스 스캔
  scanI2CDevices();
  
  // 초기 화면 표시
  displayStartupScreen();
  
  Serial.println("시스템 준비 완료");
}

void loop() {
  // 센서 데이터 읽기
  readAllSensors();
  
  // 데이터 표시
  displaySensorData();
  
  // 데이터 로깅
  logDataToEEPROM();
  
  // 시리얼 명령 처리
  handleSerialCommands();
  
  delay(1000);
}

void initializeDevices() {
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Init...");
  
  // RTC 초기화
  if (!rtc.begin()) {
    Serial.println("RTC 초기화 실패!");
    lcd.setCursor(0, 1);
    lcd.print("RTC Error!");
    while(1);
  }
  
  // RTC가 시간을 잃었다면 컴파일 시간으로 설정
  if (!rtc.isrunning()) {
    Serial.println("RTC 시간 설정...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // BMP280 초기화
  if (!bmp.begin(BMP280_ADDRESS)) {
    Serial.println("BMP280 초기화 실패!");
    lcd.setCursor(0, 1);
    lcd.print("BMP280 Error!");
    while(1);
  }
  
  // BMP280 설정
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // 동작 모드
                  Adafruit_BMP280::SAMPLING_X2,     // 온도 오버샘플링
                  Adafruit_BMP280::SAMPLING_X16,    // 압력 오버샘플링
                  Adafruit_BMP280::FILTER_X16,      // 필터링
                  Adafruit_BMP280::STANDBY_MS_500); // 대기시간
}

void readAllSensors() {
  sensorReading.isValid = true;
  
  // BMP280에서 온도, 압력 읽기
  sensorReading.temperature = bmp.readTemperature();
  sensorReading.pressure = bmp.readPressure() / 100.0F;  // hPa 변환
  sensorReading.altitude = bmp.readAltitude(1013.25);    // 해수면 기준
  
  // RTC에서 현재 시간 읽기
  sensorReading.timestamp = rtc.now();
  
  // 데이터 유효성 검사
  if (isnan(sensorReading.temperature) || 
      isnan(sensorReading.pressure)) {
    sensorReading.isValid = false;
    Serial.println("센서 데이터 오류!");
  }
}

void displaySensorData() {
  if (!sensorReading.isValid) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error!   ");
    return;
  }
  
  // 첫 번째 줄: 온도와 압력
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(sensorReading.temperature, 1);
  lcd.print("C P:");
  lcd.print(sensorReading.pressure, 0);
  
  // 두 번째 줄: 시간
  lcd.setCursor(0, 1);
  DateTime now = sensorReading.timestamp;
  if (now.hour() < 10) lcd.print("0");
  lcd.print(now.hour());
  lcd.print(":");
  if (now.minute() < 10) lcd.print("0");
  lcd.print(now.minute());
  lcd.print(":");
  if (now.second() < 10) lcd.print("0");
  lcd.print(now.second());
  lcd.print(" Alt:");
  lcd.print(sensorReading.altitude, 0);
}

void logDataToEEPROM() {
  // 매 10초마다 EEPROM에 데이터 저장
  static unsigned long lastLog = 0;
  
  if (millis() - lastLog > 10000 && sensorReading.isValid) {
    int address = EEPROM_DATA_START + (currentRecord * sizeof(SensorData));
    
    // EEPROM에 구조체 저장
    writeEEPROMData(address, sensorReading);
    
    currentRecord = (currentRecord + 1) % MAX_RECORDS;
    lastLog = millis();
    
    Serial.print("데이터 저장됨 - 레코드 #");
    Serial.println(currentRecord);
  }
}

void writeEEPROMData(int address, SensorData data) {
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((address >> 8) & 0xFF);    // 상위 주소
  Wire.write(address & 0xFF);           // 하위 주소
  
  // 구조체를 바이트 배열로 변환하여 전송
  byte* dataPtr = (byte*)&data;
  for (int i = 0; i < sizeof(SensorData); i++) {
    Wire.write(dataPtr[i]);
  }
  
  Wire.endTransmission();
  delay(5);  // EEPROM 쓰기 대기
}

SensorData readEEPROMData(int address) {
  SensorData data;
  
  // 읽기 주소 설정
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((address >> 8) & 0xFF);
  Wire.write(address & 0xFF);
  Wire.endTransmission();
  
  // 데이터 읽기
  Wire.requestFrom(EEPROM_ADDRESS, sizeof(SensorData));
  
  byte* dataPtr = (byte*)&data;
  int i = 0;
  while (Wire.available() && i < sizeof(SensorData)) {
    dataPtr[i++] = Wire.read();
  }
  
  return data;
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "scan") {
      scanI2CDevices();
    }
    else if (command == "data") {
      printCurrentData();
    }
    else if (command == "history") {
      printDataHistory();
    }
    else if (command == "clear") {
      clearEEPROM();
    }
    else if (command.startsWith("settime")) {
      setRTCTime(command);
    }
  }
}

void printCurrentData() {
  Serial.println("=== 현재 센서 데이터 ===");
  Serial.print("온도: ");
  Serial.print(sensorReading.temperature);
  Serial.println("°C");
  
  Serial.print("기압: ");
  Serial.print(sensorReading.pressure);
  Serial.println(" hPa");
  
  Serial.print("고도: ");
  Serial.print(sensorReading.altitude);
  Serial.println(" m");
  
  Serial.print("시간: ");
  Serial.println(sensorReading.timestamp.timestamp());
}

void printDataHistory() {
  Serial.println("=== 데이터 히스토리 ===");
  
  for (int i = 0; i < 10; i++) {  // 최근 10개 레코드
    int recordIndex = (currentRecord - i - 1 + MAX_RECORDS) % MAX_RECORDS;
    int address = EEPROM_DATA_START + (recordIndex * sizeof(SensorData));
    
    SensorData historicData = readEEPROMData(address);
    
    Serial.print("Record #");
    Serial.print(recordIndex);
    Serial.print(": T=");
    Serial.print(historicData.temperature);
    Serial.print("°C, P=");
    Serial.print(historicData.pressure);
    Serial.println(" hPa");
  }
}

void displayStartupScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("I2C Sensor Net");
  lcd.setCursor(0, 1);
  lcd.print("Ready!");
  delay(2000);
  lcd.clear();
}

void scanI2CDevices() {
  Serial.println("\n=== I2C 디바이스 스캔 ===");
  int deviceCount = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("디바이스 발견: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // 알려진 디바이스 식별
      switch (address) {
        case 0x27: Serial.println(" (LCD)"); break;
        case 0x68: Serial.println(" (RTC DS1307)"); break;
        case 0x76: Serial.println(" (BMP280)"); break;
        case 0x50: Serial.println(" (EEPROM)"); break;
        default: Serial.println(" (Unknown)"); break;
      }
      deviceCount++;
    }
  }
  
  Serial.print("총 ");
  Serial.print(deviceCount);
  Serial.println("개 디바이스 발견\n");
}
```

## 🔧 실습 2: SPI 고속 데이터 통신

### SPI를 이용한 SD카드 데이터 로깅

```cpp
/*
  SPI_DataLogger.ino
  SPI를 이용한 고속 데이터 로깅 시스템
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>

// SPI 핀 정의 (Arduino Uno)
const int SD_CS_PIN = 10;      // SD카드 Chip Select
const int TFT_CS_PIN = 9;      // TFT 디스플레이 CS (예제용)
const int SENSOR_CS_PIN = 8;   // SPI 센서 CS

// 데이터 구조체
struct LogEntry {
  unsigned long timestamp;
  float sensor1;
  float sensor2;
  float sensor3;
  int digitalInputs;
  char status[16];
};

// SPI 설정
SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);  // 4MHz, MSB First, Mode 0

// 로깅 설정
const char* LOG_FILENAME = "datalog.csv";
unsigned long logInterval = 100;  // 100ms 간격
unsigned long lastLogTime = 0;
bool loggingEnabled = true;

void setup() {
  Serial.begin(115200);
  
  // SPI 핀 설정
  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(TFT_CS_PIN, OUTPUT);
  pinMode(SENSOR_CS_PIN, OUTPUT);
  
  // 모든 CS 핀을 HIGH로 (비활성화)
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(TFT_CS_PIN, HIGH);
  digitalWrite(SENSOR_CS_PIN, HIGH);
  
  // SPI 초기화
  SPI.begin();
  
  Serial.println("SPI 데이터 로거 초기화...");
  
  // SD카드 초기화
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD카드 초기화 실패!");
    return;
  }
  
  Serial.println("SD카드 초기화 성공");
  
  // CSV 헤더 작성
  createLogFileHeader();
  
  // 시스템 정보 출력
  printSystemInfo();
  
  Serial.println("데이터 로깅 시작...");
}

void loop() {
  // 센서 데이터 읽기 및 로깅
  if (millis() - lastLogTime >= logInterval && loggingEnabled) {
    LogEntry entry = collectSensorData();
    logDataToSD(entry);
    displayDataOnSerial(entry);
    lastLogTime = millis();
  }
  
  // 시리얼 명령 처리
  handleSerialCommands();
  
  // SPI 센서 데이터 읽기 (고속)
  readSPISensors();
}

LogEntry collectSensorData() {
  LogEntry entry;
  
  entry.timestamp = millis();
  entry.sensor1 = readAnalogSensor(A0) * 0.1;     // 온도 센서 시뮬레이션
  entry.sensor2 = readAnalogSensor(A1) * 0.05;    // 습도 센서 시뮬레이션
  entry.sensor3 = readAnalogSensor(A2) * 0.02;    // 압력 센서 시뮬레이션
  
  // 디지털 입력 상태 (8비트)
  entry.digitalInputs = 0;
  for (int i = 2; i < 8; i++) {
    if (digitalRead(i)) {
      entry.digitalInputs |= (1 << (i-2));
    }
  }
  
  // 시스템 상태
  strcpy(entry.status, "NORMAL");
  
  return entry;
}

float readAnalogSensor(int pin) {
  // 노이즈 감소를 위한 오버샘플링
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(pin);
  }
  return sum / 16.0;
}

void logDataToSD(LogEntry entry) {
  File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
  
  if (logFile) {
    // CSV 형식으로 데이터 저장
    logFile.print(entry.timestamp);
    logFile.print(",");
    logFile.print(entry.sensor1, 3);
    logFile.print(",");
    logFile.print(entry.sensor2, 3);
    logFile.print(",");
    logFile.print(entry.sensor3, 3);
    logFile.print(",");
    logFile.print(entry.digitalInputs, BIN);
    logFile.print(",");
    logFile.println(entry.status);
    
    logFile.close();
  } else {
    Serial.println("SD카드 쓰기 오류!");
  }
}

void readSPISensors() {
  // SPI 센서 데이터 읽기 예제 (가상의 고속 ADC)
  static unsigned long lastSPIRead = 0;
  
  if (millis() - lastSPIRead >= 10) {  // 100Hz로 SPI 센서 읽기
    
    // SPI 통신 시작
    SPI.beginTransaction(spiSettings);
    digitalWrite(SENSOR_CS_PIN, LOW);
    
    // 명령 전송 (예: ADC 채널 선택)
    byte command = 0x01;  // 채널 1 읽기
    SPI.transfer(command);
    
    // 데이터 읽기 (16비트)
    byte highByte = SPI.transfer(0x00);
    byte lowByte = SPI.transfer(0x00);
    
    digitalWrite(SENSOR_CS_PIN, HIGH);
    SPI.endTransaction();
    
    // 16비트 데이터 조합
    uint16_t sensorValue = (highByte << 8) | lowByte;
    
    // 시리얼로 고속 데이터 출력 (디버깅용)
    if (millis() % 1000 < 100) {  // 1초에 한 번만 출력
      Serial.print("SPI Sensor: ");
      Serial.println(sensorValue);
    }
    
    lastSPIRead = millis();
  }
}

void createLogFileHeader() {
  // 기존 파일이 있으면 헤더 생략
  if (SD.exists(LOG_FILENAME)) {
    Serial.println("기존 로그 파일에 추가");
    return;
  }
  
  File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
  if (logFile) {
    logFile.println("Timestamp,Sensor1,Sensor2,Sensor3,DigitalInputs,Status");
    logFile.close();
    Serial.println("CSV 헤더 생성 완료");
  }
}

void displayDataOnSerial(LogEntry entry) {
  static int displayCounter = 0;
  
  // 10번에 한 번만 시리얼 출력 (성능 최적화)
  if (++displayCounter >= 10) {
    Serial.print("Time: ");
    Serial.print(entry.timestamp);
    Serial.print(", S1: ");
    Serial.print(entry.sensor1, 2);
    Serial.print(", S2: ");
    Serial.print(entry.sensor2, 2);
    Serial.print(", S3: ");
    Serial.print(entry.sensor3, 2);
    Serial.print(", DI: ");
    Serial.print(entry.digitalInputs, BIN);
    Serial.print(", Status: ");
    Serial.println(entry.status);
    
    displayCounter = 0;
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "start") {
      loggingEnabled = true;
      Serial.println("로깅 시작");
    }
    else if (command == "stop") {
      loggingEnabled = false;
      Serial.println("로깅 중지");
    }
    else if (command == "status") {
      printSystemStatus();
    }
    else if (command == "read") {
      readLastEntries(10);
    }
    else if (command.startsWith("interval ")) {
      int newInterval = command.substring(9).toInt();
      if (newInterval >= 10 && newInterval <= 10000) {
        logInterval = newInterval;
        Serial.print("로깅 간격: ");
        Serial.print(logInterval);
        Serial.println("ms");
      }
    }
    else if (command == "format") {
      formatSDCard();
    }
    else if (command == "help") {
      printHelpMenu();
    }
  }
}

void printSystemInfo() {
  Serial.println("\n=== 시스템 정보 ===");
  
  // SD카드 정보
  File root = SD.open("/");
  Serial.print("SD카드 여유 공간: ");
  // Note: Arduino SD library doesn't provide free space info
  Serial.println("정보 없음");
  
  // SPI 설정 정보
  Serial.println("SPI 설정:");
  Serial.println("- 클럭: 4MHz");
  Serial.println("- 모드: SPI_MODE0");
  Serial.println("- 비트 순서: MSB First");
  
  // 로깅 설정
  Serial.print("로깅 간격: ");
  Serial.print(logInterval);
  Serial.println("ms");
  
  Serial.println("==================\n");
}

void printSystemStatus() {
  Serial.println("\n=== 시스템 상태 ===");
  Serial.print("로깅 상태: ");
  Serial.println(loggingEnabled ? "활성" : "비활성");
  
  Serial.print("업타임: ");
  Serial.print(millis() / 1000);
  Serial.println("초");
  
  Serial.print("로그 파일: ");
  Serial.println(LOG_FILENAME);
  
  if (SD.exists(LOG_FILENAME)) {
    File logFile = SD.open(LOG_FILENAME);
    Serial.print("파일 크기: ");
    Serial.print(logFile.size());
    Serial.println(" bytes");
    logFile.close();
  }
  
  Serial.println("==================\n");
}

void readLastEntries(int count) {
  Serial.println("\n=== 최근 로그 엔트리 ===");
  
  File logFile = SD.open(LOG_FILENAME);
  if (!logFile) {
    Serial.println("로그 파일 열기 실패");
    return;
  }
  
  // 파일 끝에서부터 읽기 (간단한 구현)
  String lines[count];
  int lineCount = 0;
  
  while (logFile.available() && lineCount < count) {
    String line = logFile.readStringUntil('\n');
    lines[lineCount++] = line;
  }
  
  // 역순으로 출력
  for (int i = lineCount - 1; i >= 0; i--) {
    Serial.println(lines[i]);
  }
  
  logFile.close();
  Serial.println("======================\n");
}

void formatSDCard() {
  Serial.println("SD카드 포맷은 컴퓨터에서 수행하세요.");
  Serial.println("로그 파일만 삭제합니다...");
  
  if (SD.exists(LOG_FILENAME)) {
    SD.remove(LOG_FILENAME);
    Serial.println("로그 파일 삭제 완료");
    createLogFileHeader();
  }
}

void printHelpMenu() {
  Serial.println("\n=== 명령어 도움말 ===");
  Serial.println("start          - 로깅 시작");
  Serial.println("stop           - 로깅 중지");
  Serial.println("status         - 시스템 상태");
  Serial.println("read           - 최근 10개 엔트리");
  Serial.println("interval <ms>  - 로깅 간격 설정");
  Serial.println("format         - 로그 파일 삭제");
  Serial.println("help           - 이 도움말");
  Serial.println("=====================\n");
}
```

## 🔧 실습 3: I2C와 SPI 통합 시스템

### 하이브리드 통신 시스템

```cpp
/*
  Hybrid_Communication_System.ino
  I2C와 SPI를 함께 사용하는 통합 시스템
*/

#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>

// I2C 디바이스
LiquidCrystal_I2C lcd(0x27, 16, 2);

// SPI 디바이스 CS 핀
const int SD_CS = 10;
const int DISPLAY_CS = 9;
const int SENSOR_CS = 8;

// 시스템 상태
struct SystemState {
  bool i2cReady;
  bool spiReady;
  bool sdReady;
  int errorCount;
  unsigned long totalTransactions;
} systemState;

// 성능 모니터링
struct PerformanceMetrics {
  unsigned long i2cTransactions;
  unsigned long spiTransactions;
  unsigned long i2cErrors;
  unsigned long spiErrors;
  float avgI2CTime;
  float avgSPITime;
} metrics;

void setup() {
  Serial.begin(115200);
  
  // 시스템 초기화
  initializeSystem();
  
  // 성능 벤치마크
  runPerformanceBenchmark();
  
  Serial.println("하이브리드 통신 시스템 준비 완료");
}

void loop() {
  // I2C 태스크 (저속, 많은 디바이스)
  executeI2CTasks();
  
  // SPI 태스크 (고속, 적은 디바이스)
  executeSPITasks();
  
  // 시스템 모니터링
  monitorSystemPerformance();
  
  // 사용자 인터페이스 업데이트
  updateUserInterface();
  
  delay(10);
}

void initializeSystem() {
  systemState = {false, false, false, 0, 0};
  metrics = {0, 0, 0, 0, 0.0, 0.0};
  
  // I2C 초기화
  Wire.begin();
  Wire.setClock(400000);  // 400kHz Fast Mode
  
  if (initializeI2CDevices()) {
    systemState.i2cReady = true;
    Serial.println("I2C 시스템 초기화 완료");
  }
  
  // SPI 초기화
  SPI.begin();
  
  if (initializeSPIDevices()) {
    systemState.spiReady = true;
    Serial.println("SPI 시스템 초기화 완료");
  }
}

bool initializeI2CDevices() {
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Init...");
  
  // I2C 디바이스 스캔 및 확인
  return scanAndVerifyI2CDevices();
}

bool initializeSPIDevices() {
  // CS 핀들 설정
  pinMode(SD_CS, OUTPUT);
  pinMode(DISPLAY_CS, OUTPUT);
  pinMode(SENSOR_CS, OUTPUT);
  
  digitalWrite(SD_CS, HIGH);
  digitalWrite(DISPLAY_CS, HIGH);
  digitalWrite(SENSOR_CS, HIGH);
  
  // SD카드 초기화
  if (SD.begin(SD_CS)) {
    systemState.sdReady = true;
    Serial.println("SD카드 초기화 성공");
  } else {
    Serial.println("SD카드 초기화 실패");
  }
  
  return true;
}

void executeI2CTasks() {
  static unsigned long lastI2CTask = 0;
  
  if (millis() - lastI2CTask >= 500) {  // 500ms 간격
    unsigned long startTime = micros();
    
    // I2C 센서 데이터 읽기
    readI2CSensors();
    
    // LCD 업데이트
    updateLCDDisplay();
    
    // 실행 시간 측정
    unsigned long executionTime = micros() - startTime;
    updateI2CMetrics(executionTime);
    
    lastI2CTask = millis();
  }
}

void executeSPITasks() {
  static unsigned long lastSPITask = 0;
  
  if (millis() - lastSPITask >= 100) {  // 100ms 간격 (고속)
    unsigned long startTime = micros();
    
    // 고속 센서 데이터 읽기
    readSPISensors();
    
    // 데이터 로깅
    logDataToSD();
    
    // 실행 시간 측정
    unsigned long executionTime = micros() - startTime;
    updateSPIMetrics(executionTime);
    
    lastSPITask = millis();
  }
}

void readI2CSensors() {
  // 가상의 I2C 센서들 읽기
  for (int addr = 0x20; addr <= 0x27; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      // 센서에서 데이터 요청
      Wire.requestFrom(addr, 2);
      if (Wire.available() >= 2) {
        byte data1 = Wire.read();
        byte data2 = Wire.read();
        
        // 데이터 처리
        processI2CData(addr, data1, data2);
      }
    } else {
      metrics.i2cErrors++;
    }
    
    metrics.i2cTransactions++;
  }
}

void readSPISensors() {
  // 고속 ADC 센서 읽기
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SENSOR_CS, LOW);
  
  byte command = 0x01;  // 채널 1 선택
  SPI.transfer(command);
  
  // 16비트 데이터 읽기
  uint16_t sensorData = SPI.transfer16(0x0000);
  
  digitalWrite(SENSOR_CS, HIGH);
  SPI.endTransaction();
  
  // 데이터 처리
  processSPIData(sensorData);
  
  metrics.spiTransactions++;
}

void processI2CData(int address, byte data1, byte data2) {
  // I2C 센서 데이터 처리
  int value = (data1 << 8) | data2;
  
  Serial.print("I2C[0x");
  Serial.print(address, HEX);
  Serial.print("]: ");
  Serial.println(value);
}

void processSPIData(uint16_t data) {
  // SPI 센서 데이터 처리
  float voltage = (data * 5.0) / 65535.0;
  
  Serial.print("SPI Sensor: ");
  Serial.print(voltage, 3);
  Serial.println("V");
}

void updateLCDDisplay() {
  static int displayMode = 0;
  static unsigned long lastModeChange = 0;
  
  // 5초마다 표시 모드 변경
  if (millis() - lastModeChange >= 5000) {
    displayMode = (displayMode + 1) % 3;
    lastModeChange = millis();
    lcd.clear();
  }
  
  switch (displayMode) {
    case 0:  // 시스템 상태
      lcd.setCursor(0, 0);
      lcd.print("I2C:");
      lcd.print(systemState.i2cReady ? "OK" : "NG");
      lcd.print(" SPI:");
      lcd.print(systemState.spiReady ? "OK" : "NG");
      
      lcd.setCursor(0, 1);
      lcd.print("Err:");
      lcd.print(systemState.errorCount);
      lcd.print(" Up:");
      lcd.print(millis() / 1000);
      break;
      
    case 1:  // 성능 메트릭
      lcd.setCursor(0, 0);
      lcd.print("I2C:");
      lcd.print(metrics.i2cTransactions);
      lcd.print("tx");
      
      lcd.setCursor(0, 1);
      lcd.print("SPI:");
      lcd.print(metrics.spiTransactions);
      lcd.print("tx");
      break;
      
    case 2:  // 오류 통계
      lcd.setCursor(0, 0);
      lcd.print("I2C Err:");
      lcd.print(metrics.i2cErrors);
      
      lcd.setCursor(0, 1);
      lcd.print("SPI Err:");
      lcd.print(metrics.spiErrors);
      break;
  }
}

void logDataToSD() {
  static unsigned long lastLog = 0;
  
  if (millis() - lastLog >= 1000 && systemState.sdReady) {  // 1초마다
    File logFile = SD.open("hybrid.log", FILE_WRITE);
    
    if (logFile) {
      logFile.print(millis());
      logFile.print(",");
      logFile.print(metrics.i2cTransactions);
      logFile.print(",");
      logFile.print(metrics.spiTransactions);
      logFile.print(",");
      logFile.print(metrics.avgI2CTime);
      logFile.print(",");
      logFile.println(metrics.avgSPITime);
      
      logFile.close();
    }
    
    lastLog = millis();
  }
}

bool scanAndVerifyI2CDevices() {
  Serial.println("I2C 디바이스 스캔 중...");
  int deviceCount = 0;
  
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("발견: 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      deviceCount++;
    }
  }
  
  Serial.print("총 ");
  Serial.print(deviceCount);
  Serial.println("개 I2C 디바이스 발견");
  
  return deviceCount > 0;
}

void updateI2CMetrics(unsigned long executionTime) {
  static unsigned long totalI2CTime = 0;
  static int i2cSamples = 0;
  
  totalI2CTime += executionTime;
  i2cSamples++;
  
  metrics.avgI2CTime = totalI2CTime / (float)i2cSamples;
}

void updateSPIMetrics(unsigned long executionTime) {
  static unsigned long totalSPITime = 0;
  static int spiSamples = 0;
  
  totalSPITime += executionTime;
  spiSamples++;
  
  metrics.avgSPITime = totalSPITime / (float)spiSamples;
}

void monitorSystemPerformance() {
  static unsigned long lastMonitor = 0;
  
  if (millis() - lastMonitor >= 10000) {  // 10초마다
    Serial.println("\n=== 성능 모니터링 ===");
    Serial.print("I2C 평균 실행시간: ");
    Serial.print(metrics.avgI2CTime);
    Serial.println("μs");
    
    Serial.print("SPI 평균 실행시간: ");
    Serial.print(metrics.avgSPITime);
    Serial.println("μs");
    
    Serial.print("I2C 오류율: ");
    Serial.print((metrics.i2cErrors * 100.0) / metrics.i2cTransactions);
    Serial.println("%");
    
    Serial.print("SPI 오류율: ");
    Serial.print((metrics.spiErrors * 100.0) / metrics.spiTransactions);
    Serial.println("%");
    
    Serial.println("==================\n");
    
    lastMonitor = millis();
  }
}

void updateUserInterface() {
  // 시리얼 명령 처리
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "stats") {
      printDetailedStats();
    }
    else if (command == "reset") {
      resetMetrics();
    }
    else if (command == "benchmark") {
      runPerformanceBenchmark();
    }
  }
}

void printDetailedStats() {
  Serial.println("\n=== 상세 통계 ===");
  Serial.print("시스템 업타임: ");
  Serial.print(millis() / 1000);
  Serial.println("초");
  
  Serial.print("총 I2C 트랜잭션: ");
  Serial.println(metrics.i2cTransactions);
  
  Serial.print("총 SPI 트랜잭션: ");
  Serial.println(metrics.spiTransactions);
  
  Serial.print("I2C 오류 수: ");
  Serial.println(metrics.i2cErrors);
  
  Serial.print("SPI 오류 수: ");
  Serial.println(metrics.spiErrors);
  
  Serial.println("================\n");
}

void resetMetrics() {
  metrics = {0, 0, 0, 0, 0.0, 0.0};
  Serial.println("메트릭 초기화 완료");
}

void runPerformanceBenchmark() {
  Serial.println("\n=== 성능 벤치마크 ===");
  
  // I2C 벤치마크
  unsigned long startTime = micros();
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(0x27);
    Wire.write(0x01);
    Wire.endTransmission();
  }
  unsigned long i2cBenchmark = micros() - startTime;
  
  // SPI 벤치마크
  startTime = micros();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 100; i++) {
    SPI.transfer(0x55);
  }
  SPI.endTransaction();
  unsigned long spiBenchmark = micros() - startTime;
  
  Serial.print("I2C 100회 전송: ");
  Serial.print(i2cBenchmark);
  Serial.println("μs");
  
  Serial.print("SPI 100회 전송: ");
  Serial.print(spiBenchmark);
  Serial.println("μs");
  
  Serial.print("SPI가 I2C보다 ");
  Serial.print(i2cBenchmark / (float)spiBenchmark);
  Serial.println("배 빠름");
  
  Serial.println("===================\n");
}
```

## 📊 통신 프로토콜 성능 분석

### 성능 비교표

```
┌────────────────────────────────────────────────────────────┐
│                   실제 성능 측정 결과                        │
├─────────────┬─────────────┬─────────────┬─────────────────┤
│   메트릭    │    I2C      │    SPI      │     비율        │
├─────────────┼─────────────┼─────────────┼─────────────────┤
│ 1바이트전송 │   25μs      │    2μs      │  SPI 12.5배빠름 │
│ 100바이트전송│  2.1ms      │   0.2ms     │  SPI 10.5배빠름 │
│ CPU 오버헤드│    높음     │    낮음     │  SPI 유리       │
│ 전력 소모   │    낮음     │    높음     │  I2C 유리       │
│ 구현 복잡도 │    낮음     │    중간     │  I2C 유리       │
└─────────────┴─────────────┴─────────────┴─────────────────┘
```

### 최적 사용 시나리오

```cpp
// 프로토콜 선택 가이드
void selectOptimalProtocol() {
  
  // I2C 추천 상황
  if (디바이스수 > 3 && 데이터량 < 1KB && 핀수제한 == true) {
    protocol = I2C;
    // 예: 센서 네트워크, LCD, RTC, EEPROM
  }
  
  // SPI 추천 상황
  else if (속도요구 > 1MHz && 데이터량 > 1KB && 핀여유 == true) {
    protocol = SPI;
    // 예: SD카드, TFT디스플레이, 고속ADC
  }
  
  // UART 추천 상황
  else if (거리 > 1m || 디바이스수 == 1) {
    protocol = UART;
    // 예: GPS, 블루투스, 시리얼 통신
  }
}
```

## 🚀 다음 단계

I2C와 SPI 통신을 완전히 마스터했습니다! 다음 강의에서는:

- **Lesson 12**: 무선 통신 (블루투스, WiFi, LoRa)
- **Lesson 13**: 데이터 로깅 및 SD카드 고급 활용
- **Lesson 14**: 전력 관리 및 저전력 설계
- **Lesson 15**: 실전 프로젝트 - IoT 스마트홈 시스템

통신 프로토콜을 활용해 복잡한 센서 네트워크와 고속 데이터 처리 시스템을 만들어보세요!

---

*아두이노 마스터 과정 - Lesson 11 완료*  
*Created by Arduino Expert Teacher*  
*© 2024 Arduino Complete Guide*