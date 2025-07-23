# Lesson 13: 데이터 로깅 및 저장소 마스터

## 🎯 학습 목표
- SD카드를 이용한 고성능 데이터 로깅 시스템 구축
- EEPROM/Flash 메모리 활용 및 최적화 기법
- 실시간 데이터베이스 구조 설계 및 구현
- 대용량 센서 데이터 압축 및 효율적 저장
- 데이터 백업, 복구 및 원격 동기화 시스템

## 📚 이론 배경

### 아두이노 저장소 계층 구조

```
┌─────────────────────────────────────────────────────────────────┐
│                   아두이노 메모리 계층도                          │
├─────────────┬─────────────┬─────────────┬─────────────────────┤
│   저장소    │    크기     │    속도     │       특징          │
├─────────────┼─────────────┼─────────────┼─────────────────────┤
│    SRAM     │   2KB       │  매우빠름    │ 휘발성, 변수저장    │
│  EEPROM     │   1KB       │    보통     │ 비휘발성, 설정값    │
│   Flash     │   32KB      │    빠름     │ 프로그램 코드       │
│  SD Card    │   ~32GB     │    느림     │ 대용량 데이터       │
│  External   │   ~8MB      │    보통     │ 추가 저장공간       │
└─────────────┴─────────────┴─────────────┴─────────────────────┘
```

### 데이터 로깅 아키텍처

```
데이터 수집 → 전처리 → 버퍼링 → 압축 → 저장 → 동기화
     ↓         ↓        ↓        ↓      ↓        ↓
  센서읽기   필터링   메모리큐   알고리즘  SD/Flash  클라우드

실시간 처리 계층:
┌─────────────────────────────────────────────────────────────┐
│  센서 → ADC → 필터 → 큐 → 파일시스템 → 네트워크 → 저장소   │
│   1ms   100μs  500μs  1ms     10ms      100ms     1000ms  │
└─────────────────────────────────────────────────────────────┘
```

## 🔧 실습 1: 고성능 SD카드 데이터 로거

### 다중 센서 고속 로깅 시스템

```cpp
/*
  High_Performance_DataLogger.ino
  고성능 다중 센서 데이터 로깅 시스템
  
  특징:
  - 다중 센서 동시 수집
  - 압축 알고리즘 적용
  - 실시간 성능 모니터링
  - 자동 파일 롤링
  - 오류 복구 기능
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>

// 하드웨어 설정
const int SD_CS_PIN = 10;
const int LED_STATUS = 8;
const int BUTTON_PIN = 7;

// RTC 객체
RTC_DS3231 rtc;

// 로깅 설정
const int BUFFER_SIZE = 512;        // 버퍼 크기
const int MAX_FILE_SIZE = 10485760; // 10MB (파일 롤링)
const int SENSORS_COUNT = 8;        // 센서 개수
const int LOGGING_INTERVAL = 100;   // 100ms 간격

// 센서 데이터 구조체
struct SensorReading {
  uint32_t timestamp;
  float values[SENSORS_COUNT];
  uint8_t status;
  int16_t checksum;
};

// 원형 버퍼 구조체
struct CircularBuffer {
  SensorReading data[BUFFER_SIZE];
  volatile int head;
  volatile int tail;
  volatile int count;
  bool overflow;
} dataBuffer;

// 시스템 상태
struct LoggerStatus {
  unsigned long totalRecords;
  unsigned long currentFileSize;
  int currentFileNumber;
  float writeSpeed;         // records/sec
  int bufferUsage;         // %
  int errorCount;
  bool sdReady;
  bool rtcReady;
  String currentFileName;
} loggerStatus;

// 압축 설정
const int COMPRESSION_THRESHOLD = 100; // 100개 레코드마다 압축
struct CompressedData {
  uint32_t startTime;
  uint32_t interval;
  int recordCount;
  uint8_t compressedValues[BUFFER_SIZE * 2];
  int compressedSize;
};

void setup() {
  Serial.begin(115200);
  
  // 하드웨어 초기화
  initializeHardware();
  
  // SD카드 초기화
  if (!initializeSD()) {
    Serial.println("SD카드 초기화 실패!");
    return;
  }
  
  // RTC 초기화
  if (!initializeRTC()) {
    Serial.println("RTC 초기화 실패!");
  }
  
  // 버퍼 초기화
  initializeBuffer();
  
  // 시스템 상태 초기화
  initializeLoggerStatus();
  
  // 인터럽트 타이머 설정 (데이터 수집용)
  setupDataCollectionTimer();
  
  Serial.println("고성능 데이터 로거 시작!");
  printSystemInfo();
}

void loop() {
  // 버퍼에서 SD카드로 데이터 쓰기
  processDataBuffer();
  
  // 시스템 모니터링
  updateSystemStatus();
  
  // 사용자 명령 처리
  handleUserCommands();
  
  // 상태 LED 업데이트
  updateStatusLED();
  
  // 성능 통계 출력 (10초마다)
  static unsigned long lastStats = 0;
  if (millis() - lastStats >= 10000) {
    printPerformanceStats();
    lastStats = millis();
  }
  
  delay(10);
}

void initializeHardware() {
  pinMode(LED_STATUS, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // 아날로그 기준 전압 설정
  analogReference(EXTERNAL); // 더 정확한 측정을 위해
  
  Serial.println("하드웨어 초기화 완료");
}

bool initializeSD() {
  if (!SD.begin(SD_CS_PIN)) {
    loggerStatus.sdReady = false;
    return false;
  }
  
  // SD카드 정보 출력
  Sd2Card card;
  SdVolume volume;
  
  if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
    Serial.print("SD카드 타입: ");
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
      case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
      case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
      default:
        Serial.println("Unknown");
    }
  }
  
  loggerStatus.sdReady = true;
  return true;
}

bool initializeRTC() {
  if (!rtc.begin()) {
    loggerStatus.rtcReady = false;
    return false;
  }
  
  // RTC 정확도 확인
  if (rtc.lostPower()) {
    Serial.println("RTC 전원 손실 - 시간 재설정 필요");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // 32kHz 출력 활성화 (정확한 타이밍용)
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  
  loggerStatus.rtcReady = true;
  return true;
}

void initializeBuffer() {
  dataBuffer.head = 0;
  dataBuffer.tail = 0;
  dataBuffer.count = 0;
  dataBuffer.overflow = false;
  
  Serial.println("데이터 버퍼 초기화 완료");
}

void initializeLoggerStatus() {
  loggerStatus.totalRecords = 0;
  loggerStatus.currentFileSize = 0;
  loggerStatus.currentFileNumber = getNextFileNumber();
  loggerStatus.writeSpeed = 0.0;
  loggerStatus.bufferUsage = 0;
  loggerStatus.errorCount = 0;
  
  // 새 로그 파일 생성
  createNewLogFile();
}

void setupDataCollectionTimer() {
  // Timer1을 사용한 정확한 인터럽트 설정
  cli(); // 인터럽트 비활성화
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  // 100ms 간격으로 인터럽트 (16MHz / 1024 / 1562.5 = 10Hz)
  OCR1A = 1562;
  TCCR1B |= (1 << WGM12);   // CTC 모드
  TCCR1B |= (1 << CS12) | (1 << CS10); // 1024 분주
  TIMSK1 |= (1 << OCIE1A); // 인터럽트 활성화
  
  sei(); // 인터럽트 활성화
  
  Serial.println("데이터 수집 타이머 설정 완료");
}

// 타이머 인터럽트 서비스 루틴
ISR(TIMER1_COMPA_vect) {
  collectSensorData();
}

void collectSensorData() {
  static unsigned long lastCollection = 0;
  unsigned long currentTime = millis();
  
  // 인터럽트 오버런 체크
  if (currentTime - lastCollection < LOGGING_INTERVAL - 10) {
    return; // 너무 빠른 호출 무시
  }
  
  // 버퍼 가득 참 체크
  if (dataBuffer.count >= BUFFER_SIZE - 1) {
    dataBuffer.overflow = true;
    loggerStatus.errorCount++;
    return;
  }
  
  // 센서 데이터 읽기
  SensorReading reading;
  reading.timestamp = loggerStatus.rtcReady ? 
                     rtc.now().unixtime() : 
                     millis();
  
  // 8개 아날로그 센서 읽기 (고속 연속 읽기)
  for (int i = 0; i < SENSORS_COUNT; i++) {
    reading.values[i] = readAnalogSensorFast(i);
  }
  
  // 상태 정보
  reading.status = (loggerStatus.sdReady ? 0x01 : 0x00) |
                   (loggerStatus.rtcReady ? 0x02 : 0x00) |
                   (dataBuffer.overflow ? 0x04 : 0x00);
  
  // 체크섬 계산
  reading.checksum = calculateChecksum(&reading);
  
  // 버퍼에 추가 (원자적 연산)
  cli();
  dataBuffer.data[dataBuffer.head] = reading;
  dataBuffer.head = (dataBuffer.head + 1) % BUFFER_SIZE;
  dataBuffer.count++;
  sei();
  
  lastCollection = currentTime;
}

float readAnalogSensorFast(int sensorIndex) {
  // 고속 ADC 읽기 (평균화 없이)
  int rawValue = analogRead(sensorIndex);
  
  // 센서별 캘리브레이션 적용
  float calibratedValue = applySensorCalibration(sensorIndex, rawValue);
  
  return calibratedValue;
}

float applySensorCalibration(int sensor, int rawValue) {
  // 센서별 캘리브레이션 계수
  const float calibrationFactors[SENSORS_COUNT] = {
    1.0,    // 센서 0: 온도 (LM35)
    0.1,    // 센서 1: 습도
    0.05,   // 센서 2: 압력
    2.5,    // 센서 3: 전압
    0.01,   // 센서 4: 전류
    1.5,    // 센서 5: 조도
    0.5,    // 센서 6: 진동
    3.3     // 센서 7: 배터리 전압
  };
  
  const float offsets[SENSORS_COUNT] = {
    -50.0,  // 온도 오프셋
    0.0,    // 습도 오프셋
    1013.25, // 기압 기준값
    0.0,    // 전압 오프셋
    0.0,    // 전류 오프셋
    0.0,    // 조도 오프셋
    0.0,    // 진동 오프셋
    0.0     // 배터리 오프셋
  };
  
  return (rawValue * calibrationFactors[sensor]) + offsets[sensor];
}

int16_t calculateChecksum(SensorReading* reading) {
  int16_t checksum = 0;
  
  // 타임스탬프 체크섬
  checksum += (reading->timestamp & 0xFFFF);
  checksum += ((reading->timestamp >> 16) & 0xFFFF);
  
  // 센서 값들 체크섬 (정수로 변환)
  for (int i = 0; i < SENSORS_COUNT; i++) {
    int16_t intValue = (int16_t)(reading->values[i] * 100);
    checksum += intValue;
  }
  
  checksum += reading->status;
  
  return checksum;
}

void processDataBuffer() {
  const int BATCH_SIZE = 10; // 한 번에 처리할 레코드 수
  int processed = 0;
  
  unsigned long startTime = millis();
  
  // 배치 단위로 데이터 처리
  while (dataBuffer.count > 0 && processed < BATCH_SIZE) {
    SensorReading reading;
    
    // 버퍼에서 데이터 읽기 (원자적 연산)
    cli();
    if (dataBuffer.count > 0) {
      reading = dataBuffer.data[dataBuffer.tail];
      dataBuffer.tail = (dataBuffer.tail + 1) % BUFFER_SIZE;
      dataBuffer.count--;
    }
    sei();
    
    // SD카드에 쓰기
    if (writeRecordToSD(reading)) {
      loggerStatus.totalRecords++;
      processed++;
    } else {
      loggerStatus.errorCount++;
      // 쓰기 실패시 버퍼에 다시 넣기 (선택사항)
    }
    
    // 파일 크기 체크 및 롤링
    if (loggerStatus.currentFileSize > MAX_FILE_SIZE) {
      rollOverLogFile();
    }
  }
  
  // 쓰기 성능 계산
  if (processed > 0) {
    unsigned long elapsedTime = millis() - startTime;
    loggerStatus.writeSpeed = (processed * 1000.0) / elapsedTime;
  }
}

bool writeRecordToSD(SensorReading reading) {
  File logFile = SD.open(loggerStatus.currentFileName, FILE_WRITE);
  
  if (!logFile) {
    return false;
  }
  
  // 바이너리 형식으로 쓰기 (빠른 속도)
  size_t bytesWritten = logFile.write((uint8_t*)&reading, sizeof(SensorReading));
  
  logFile.close();
  
  if (bytesWritten == sizeof(SensorReading)) {
    loggerStatus.currentFileSize += bytesWritten;
    return true;
  }
  
  return false;
}

void rollOverLogFile() {
  Serial.print("파일 롤링: ");
  Serial.print(loggerStatus.currentFileName);
  Serial.print(" -> ");
  
  loggerStatus.currentFileNumber++;
  createNewLogFile();
  
  Serial.println(loggerStatus.currentFileName);
}

void createNewLogFile() {
  DateTime now = rtc.now();
  
  char fileName[32];
  sprintf(fileName, "LOG_%04d%02d%02d_%03d.DAT", 
          now.year(), now.month(), now.day(), 
          loggerStatus.currentFileNumber);
  
  loggerStatus.currentFileName = String(fileName);
  loggerStatus.currentFileSize = 0;
  
  // 파일 헤더 쓰기
  writeFileHeader();
}

void writeFileHeader() {
  File logFile = SD.open(loggerStatus.currentFileName, FILE_WRITE);
  
  if (logFile) {
    // 파일 헤더 구조체
    struct FileHeader {
      char signature[8];      // "ADUINO01"
      uint32_t version;       // 파일 포맷 버전
      uint32_t created;       // 생성 시간
      uint16_t sensorCount;   // 센서 개수
      uint16_t recordSize;    // 레코드 크기
      char description[64];   // 설명
    } header;
    
    strcpy(header.signature, "ADUINO01");
    header.version = 1;
    header.created = rtc.now().unixtime();
    header.sensorCount = SENSORS_COUNT;
    header.recordSize = sizeof(SensorReading);
    strcpy(header.description, "High Performance Data Logger");
    
    logFile.write((uint8_t*)&header, sizeof(FileHeader));
    logFile.close();
    
    loggerStatus.currentFileSize = sizeof(FileHeader);
  }
}

int getNextFileNumber() {
  int maxNumber = 0;
  
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    
    String fileName = entry.name();
    if (fileName.startsWith("LOG_") && fileName.endsWith(".DAT")) {
      // 파일명에서 번호 추출
      int underscoreIndex = fileName.lastIndexOf('_');
      int dotIndex = fileName.lastIndexOf('.');
      
      if (underscoreIndex > 0 && dotIndex > underscoreIndex) {
        String numberStr = fileName.substring(underscoreIndex + 1, dotIndex);
        int number = numberStr.toInt();
        if (number > maxNumber) {
          maxNumber = number;
        }
      }
    }
    entry.close();
  }
  root.close();
  
  return maxNumber + 1;
}

void updateSystemStatus() {
  // 버퍼 사용률 계산
  loggerStatus.bufferUsage = (dataBuffer.count * 100) / BUFFER_SIZE;
  
  // 오버플로우 감지
  if (dataBuffer.overflow) {
    Serial.println("경고: 데이터 버퍼 오버플로우!");
    dataBuffer.overflow = false;
  }
  
  // 메모리 사용량 체크
  int freeMemory = getFreeMemory();
  if (freeMemory < 200) {  // 200바이트 미만
    Serial.print("경고: 메모리 부족! ");
    Serial.print(freeMemory);
    Serial.println(" bytes");
  }
}

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  unsigned long currentTime = millis();
  
  // 시스템 상태에 따른 LED 패턴
  int blinkInterval;
  
  if (!loggerStatus.sdReady) {
    blinkInterval = 100;  // 빠른 깜빡임 (SD 오류)
  } else if (loggerStatus.errorCount > 10) {
    blinkInterval = 200;  // 중간 깜빡임 (일반 오류)
  } else if (loggerStatus.bufferUsage > 80) {
    blinkInterval = 300;  // 느린 깜빡임 (버퍼 부족)
  } else {
    blinkInterval = 1000; // 매우 느림 (정상)
  }
  
  if (currentTime - lastBlink >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(LED_STATUS, ledState);
    lastBlink = currentTime;
  }
}

void handleUserCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "status") {
      printDetailedStatus();
    }
    else if (command == "stats") {
      printPerformanceStats();
    }
    else if (command == "files") {
      listLogFiles();
    }
    else if (command.startsWith("read ")) {
      String fileName = command.substring(5);
      readLogFile(fileName);
    }
    else if (command == "compress") {
      compressOldFiles();
    }
    else if (command == "reset") {
      resetStatistics();
    }
    else if (command == "help") {
      printHelpMenu();
    }
    else {
      Serial.println("알 수 없는 명령: " + command);
      Serial.println("'help' 명령으로 도움말을 확인하세요.");
    }
  }
}

void printSystemInfo() {
  Serial.println("\n=== 시스템 정보 ===");
  Serial.print("컴파일 시간: ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  
  Serial.print("센서 개수: ");
  Serial.println(SENSORS_COUNT);
  
  Serial.print("로깅 간격: ");
  Serial.print(LOGGING_INTERVAL);
  Serial.println("ms");
  
  Serial.print("버퍼 크기: ");
  Serial.print(BUFFER_SIZE);
  Serial.println(" records");
  
  Serial.print("최대 파일 크기: ");
  Serial.print(MAX_FILE_SIZE / 1024 / 1024);
  Serial.println("MB");
  
  Serial.print("SD카드 상태: ");
  Serial.println(loggerStatus.sdReady ? "준비됨" : "오류");
  
  Serial.print("RTC 상태: ");
  Serial.println(loggerStatus.rtcReady ? "준비됨" : "오류");
  
  Serial.print("현재 로그 파일: ");
  Serial.println(loggerStatus.currentFileName);
  
  Serial.println("==================\n");
}

void printPerformanceStats() {
  Serial.println("\n=== 성능 통계 ===");
  Serial.print("총 기록된 레코드: ");
  Serial.println(loggerStatus.totalRecords);
  
  Serial.print("현재 쓰기 속도: ");
  Serial.print(loggerStatus.writeSpeed, 2);
  Serial.println(" records/sec");
  
  Serial.print("버퍼 사용률: ");
  Serial.print(loggerStatus.bufferUsage);
  Serial.println("%");
  
  Serial.print("오류 횟수: ");
  Serial.println(loggerStatus.errorCount);
  
  Serial.print("현재 파일 크기: ");
  Serial.print(loggerStatus.currentFileSize / 1024);
  Serial.println("KB");
  
  Serial.print("업타임: ");
  printUptime();
  
  Serial.print("메모리 사용량: ");
  Serial.print(getFreeMemory());
  Serial.println(" bytes");
  
  if (loggerStatus.rtcReady) {
    Serial.print("현재 시간: ");
    Serial.println(rtc.now().timestamp());
  }
  
  Serial.println("================\n");
}

void printDetailedStatus() {
  printSystemInfo();
  printPerformanceStats();
  
  Serial.println("=== 버퍼 상태 ===");
  Serial.print("Head: ");
  Serial.print(dataBuffer.head);
  Serial.print(", Tail: ");
  Serial.print(dataBuffer.tail);
  Serial.print(", Count: ");
  Serial.println(dataBuffer.count);
  
  Serial.print("Overflow: ");
  Serial.println(dataBuffer.overflow ? "예" : "아니오");
  
  // 최근 센서 값들
  if (dataBuffer.count > 0) {
    Serial.println("\n=== 최근 센서 값 ===");
    int recentIndex = (dataBuffer.head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    SensorReading recent = dataBuffer.data[recentIndex];
    
    for (int i = 0; i < SENSORS_COUNT; i++) {
      Serial.print("센서 ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(recent.values[i], 2);
    }
  }
  
  Serial.println("==================\n");
}

void listLogFiles() {
  Serial.println("\n=== 로그 파일 목록 ===");
  
  File root = SD.open("/");
  int fileCount = 0;
  long totalSize = 0;
  
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    
    String fileName = entry.name();
    if (fileName.startsWith("LOG_") && fileName.endsWith(".DAT")) {
      Serial.print(fileName);
      Serial.print("\t");
      Serial.print(entry.size());
      Serial.println(" bytes");
      
      fileCount++;
      totalSize += entry.size();
    }
    entry.close();
  }
  root.close();
  
  Serial.print("\n총 ");
  Serial.print(fileCount);
  Serial.print("개 파일, ");
  Serial.print(totalSize / 1024);
  Serial.println("KB");
  Serial.println("=====================\n");
}

void readLogFile(String fileName) {
  File logFile = SD.open(fileName);
  
  if (!logFile) {
    Serial.println("파일을 열 수 없습니다: " + fileName);
    return;
  }
  
  Serial.println("=== 파일 내용: " + fileName + " ===");
  
  // 파일 헤더 건너뛰기
  logFile.seek(sizeof(struct FileHeader));
  
  int recordCount = 0;
  const int maxRecords = 10; // 최대 10개 레코드만 표시
  
  while (logFile.available() && recordCount < maxRecords) {
    SensorReading reading;
    
    if (logFile.read((uint8_t*)&reading, sizeof(SensorReading)) == sizeof(SensorReading)) {
      Serial.print("Record ");
      Serial.print(recordCount + 1);
      Serial.print(": Time=");
      Serial.print(reading.timestamp);
      Serial.print(", Values=[");
      
      for (int i = 0; i < SENSORS_COUNT; i++) {
        Serial.print(reading.values[i], 2);
        if (i < SENSORS_COUNT - 1) Serial.print(", ");
      }
      
      Serial.print("], Status=0x");
      Serial.print(reading.status, HEX);
      Serial.print(", Checksum=");
      Serial.println(reading.checksum);
      
      recordCount++;
    } else {
      Serial.println("레코드 읽기 오류");
      break;
    }
  }
  
  if (recordCount == maxRecords && logFile.available()) {
    Serial.println("... (더 많은 레코드가 있습니다)");
  }
  
  logFile.close();
  Serial.println("===========================\n");
}

void printUptime() {
  unsigned long uptime = millis();
  unsigned long seconds = uptime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;
  
  if (days > 0) {
    Serial.print(days);
    Serial.print("일 ");
  }
  if (hours % 24 > 0) {
    Serial.print(hours % 24);
    Serial.print("시간 ");
  }
  if (minutes % 60 > 0) {
    Serial.print(minutes % 60);
    Serial.print("분 ");
  }
  Serial.print(seconds % 60);
  Serial.println("초");
}

void compressOldFiles() {
  Serial.println("파일 압축 기능은 향후 구현 예정입니다.");
  // 실제 구현에서는 zlib 또는 간단한 압축 알고리즘 사용
}

void resetStatistics() {
  loggerStatus.totalRecords = 0;
  loggerStatus.errorCount = 0;
  loggerStatus.writeSpeed = 0.0;
  
  Serial.println("통계가 초기화되었습니다.");
}

void printHelpMenu() {
  Serial.println("\n=== 명령어 도움말 ===");
  Serial.println("status    - 상세 시스템 상태");
  Serial.println("stats     - 성능 통계");
  Serial.println("files     - 로그 파일 목록");
  Serial.println("read <파일명> - 로그 파일 읽기");
  Serial.println("compress  - 파일 압축");
  Serial.println("reset     - 통계 초기화");
  Serial.println("help      - 이 도움말");
  Serial.println("====================\n");
}

// 메모리 사용량 확인 함수
int getFreeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
```

## 🔧 실습 2: EEPROM 활용 고급 기법

### 설정 관리 및 캘리브레이션 데이터 저장

```cpp
/*
  Advanced_EEPROM_Management.ino
  EEPROM을 이용한 고급 설정 관리 시스템
  
  특징:
  - 구조화된 설정 저장
  - CRC 검증
  - 버전 관리
  - 자동 백업/복구
  - 마모 평준화
*/

#include <EEPROM.h>
#include <ArduinoJson.h>

// EEPROM 레이아웃 정의
const int EEPROM_SIZE = 1024;           // Arduino Uno EEPROM 크기
const int CONFIG_START_ADDR = 0;        // 설정 시작 주소
const int CALIBRATION_START_ADDR = 256; // 캘리브레이션 시작 주소
const int LOG_START_ADDR = 512;         // 로그 시작 주소
const int BACKUP_START_ADDR = 768;      // 백업 시작 주소

// 설정 구조체
struct SystemConfig {
  uint32_t signature;        // 매직 넘버
  uint16_t version;          // 설정 버전
  uint16_t size;             // 구조체 크기
  
  // 센서 설정
  struct {
    float offset[8];         // 센서 오프셋
    float gain[8];           // 센서 게인
    bool enabled[8];         // 센서 활성화
    uint16_t sampleRate;     // 샘플링 레이트
  } sensors;
  
  // 로깅 설정
  struct {
    bool enabled;            // 로깅 활성화
    uint16_t interval;       // 로깅 간격 (ms)
    uint16_t maxFileSize;    // 최대 파일 크기 (KB)
    bool compression;        // 압축 사용
  } logging;
  
  // 통신 설정
  struct {
    char wifiSSID[32];       // WiFi SSID
    char wifiPassword[64];   // WiFi 비밀번호
    uint32_t baudRate;       // 시리얼 통신 속도
    bool bluetoothEnabled;   // 블루투스 활성화
  } communication;
  
  // 알람 설정
  struct {
    float thresholds[8];     // 임계값
    bool enabled[8];         // 알람 활성화
    uint16_t duration;       // 알람 지속시간
  } alarms;
  
  uint32_t writeCount;       // 쓰기 횟수 (마모 추적)
  uint32_t lastModified;     // 마지막 수정 시간
  uint16_t crc16;            // CRC 체크섬
};

// 캘리브레이션 데이터
struct CalibrationData {
  uint32_t signature;
  uint16_t version;
  
  struct {
    float linearCoeff[8][2]; // 선형 보정 계수 (ax + b)
    float polynomial[8][3];   // 다항식 보정 (ax² + bx + c)
    float referenceValues[8]; // 기준값
    uint32_t lastCalibrated;  // 마지막 캘리브레이션 시간
  } sensors;
  
  uint16_t crc16;
};

// 시스템 로그 엔트리
struct LogEntry {
  uint32_t timestamp;
  uint8_t level;            // 0=INFO, 1=WARNING, 2=ERROR
  uint8_t module;           // 모듈 ID
  uint16_t code;            // 오류/이벤트 코드
  uint32_t data;            // 추가 데이터
};

// 전역 변수
SystemConfig config;
CalibrationData calibration;
int nextLogIndex = 0;

// 매직 넘버 정의
const uint32_t CONFIG_SIGNATURE = 0x12345678;
const uint32_t CALIB_SIGNATURE = 0x87654321;

void setup() {
  Serial.begin(115200);
  
  Serial.println("EEPROM 고급 관리 시스템 시작");
  
  // EEPROM 초기화
  initializeEEPROM();
  
  // 설정 로드
  if (!loadSystemConfig()) {
    Serial.println("설정 로드 실패 - 기본값 사용");
    createDefaultConfig();
    saveSystemConfig();
  }
  
  // 캘리브레이션 데이터 로드
  if (!loadCalibrationData()) {
    Serial.println("캘리브레이션 데이터 로드 실패 - 기본값 사용");
    createDefaultCalibration();
    saveCalibrationData();
  }
  
  // 시스템 상태 출력
  printSystemInfo();
  
  // 로그 엔트리 추가
  addLogEntry(0, 0, 1000, millis()); // 시스템 시작
}

void loop() {
  // 사용자 명령 처리
  handleUserCommands();
  
  // 주기적인 설정 백업 (1시간마다)
  static unsigned long lastBackup = 0;
  if (millis() - lastBackup >= 3600000) { // 1시간
    backupConfiguration();
    lastBackup = millis();
  }
  
  // EEPROM 마모 모니터링
  monitorEEPROMWear();
  
  delay(100);
}

void initializeEEPROM() {
  // EEPROM이 처음 사용되는지 확인
  uint32_t signature = readUint32FromEEPROM(CONFIG_START_ADDR);
  
  if (signature != CONFIG_SIGNATURE) {
    Serial.println("EEPROM 첫 초기화 수행");
    
    // 전체 EEPROM 클리어
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0xFF);
    }
    
    // 초기화 완료 플래그 설정
    writeUint32ToEEPROM(CONFIG_START_ADDR, CONFIG_SIGNATURE);
  }
  
  Serial.println("EEPROM 초기화 완료");
}

bool loadSystemConfig() {
  // 메인 설정 로드 시도
  if (loadConfigFromAddress(CONFIG_START_ADDR, &config)) {
    // CRC 검증
    if (validateConfigCRC(&config)) {
      Serial.println("설정 로드 성공");
      return true;
    } else {
      Serial.println("설정 CRC 오류 - 백업에서 복구 시도");
      
      // 백업에서 복구 시도
      if (loadConfigFromAddress(BACKUP_START_ADDR, &config)) {
        if (validateConfigCRC(&config)) {
          Serial.println("백업에서 설정 복구 성공");
          saveSystemConfig(); // 메인 위치에 다시 저장
          return true;
        }
      }
    }
  }
  
  return false;
}

bool loadConfigFromAddress(int address, SystemConfig* cfg) {
  // 시그니처 확인
  uint32_t signature = readUint32FromEEPROM(address);
  if (signature != CONFIG_SIGNATURE) {
    return false;
  }
  
  // 전체 구조체 읽기
  EEPROM.get(address, *cfg);
  
  // 버전 호환성 확인
  if (cfg->version > 1) {
    Serial.println("경고: 설정 버전이 더 높습니다");
  }
  
  return true;
}

bool validateConfigCRC(SystemConfig* cfg) {
  // CRC 계산 (CRC 필드 제외)
  uint16_t calculatedCRC = calculateCRC16((uint8_t*)cfg, 
                                         sizeof(SystemConfig) - sizeof(uint16_t));
  
  return (calculatedCRC == cfg->crc16);
}

void saveSystemConfig() {
  // 쓰기 횟수 증가
  config.writeCount++;
  config.lastModified = millis();
  
  // CRC 계산
  config.crc16 = calculateCRC16((uint8_t*)&config, 
                               sizeof(SystemConfig) - sizeof(uint16_t));
  
  // 메인 위치에 저장
  EEPROM.put(CONFIG_START_ADDR, config);
  
  // 백업 위치에도 저장
  EEPROM.put(BACKUP_START_ADDR, config);
  
  Serial.println("설정 저장 완료");
  
  // 로그 엔트리 추가
  addLogEntry(0, 0, 1001, config.writeCount);
}

void createDefaultConfig() {
  memset(&config, 0, sizeof(SystemConfig));
  
  config.signature = CONFIG_SIGNATURE;
  config.version = 1;
  config.size = sizeof(SystemConfig);
  
  // 센서 기본 설정
  for (int i = 0; i < 8; i++) {
    config.sensors.offset[i] = 0.0;
    config.sensors.gain[i] = 1.0;
    config.sensors.enabled[i] = true;
  }
  config.sensors.sampleRate = 100; // 100ms
  
  // 로깅 기본 설정
  config.logging.enabled = true;
  config.logging.interval = 1000; // 1초
  config.logging.maxFileSize = 1024; // 1MB
  config.logging.compression = false;
  
  // 통신 기본 설정
  strcpy(config.communication.wifiSSID, "");
  strcpy(config.communication.wifiPassword, "");
  config.communication.baudRate = 115200;
  config.communication.bluetoothEnabled = false;
  
  // 알람 기본 설정
  for (int i = 0; i < 8; i++) {
    config.alarms.thresholds[i] = 100.0;
    config.alarms.enabled[i] = false;
  }
  config.alarms.duration = 5000; // 5초
  
  config.writeCount = 0;
  config.lastModified = millis();
  
  Serial.println("기본 설정 생성 완료");
}

bool loadCalibrationData() {
  uint32_t signature = readUint32FromEEPROM(CALIBRATION_START_ADDR);
  
  if (signature != CALIB_SIGNATURE) {
    return false;
  }
  
  EEPROM.get(CALIBRATION_START_ADDR, calibration);
  
  // CRC 검증
  uint16_t calculatedCRC = calculateCRC16((uint8_t*)&calibration, 
                                         sizeof(CalibrationData) - sizeof(uint16_t));
  
  if (calculatedCRC != calibration.crc16) {
    Serial.println("캘리브레이션 데이터 CRC 오류");
    return false;
  }
  
  Serial.println("캘리브레이션 데이터 로드 성공");
  return true;
}

void saveCalibrationData() {
  calibration.crc16 = calculateCRC16((uint8_t*)&calibration, 
                                    sizeof(CalibrationData) - sizeof(uint16_t));
  
  EEPROM.put(CALIBRATION_START_ADDR, calibration);
  
  Serial.println("캘리브레이션 데이터 저장 완료");
}

void createDefaultCalibration() {
  memset(&calibration, 0, sizeof(CalibrationData));
  
  calibration.signature = CALIB_SIGNATURE;
  calibration.version = 1;
  
  // 센서별 기본 캘리브레이션
  for (int i = 0; i < 8; i++) {
    // 선형 보정: y = 1.0*x + 0.0
    calibration.sensors.linearCoeff[i][0] = 1.0; // slope
    calibration.sensors.linearCoeff[i][1] = 0.0; // offset
    
    // 다항식 보정: y = 0*x² + 1*x + 0
    calibration.sensors.polynomial[i][0] = 0.0; // x²
    calibration.sensors.polynomial[i][1] = 1.0; // x
    calibration.sensors.polynomial[i][2] = 0.0; // constant
    
    calibration.sensors.referenceValues[i] = 0.0;
    calibration.sensors.lastCalibrated = 0;
  }
  
  Serial.println("기본 캘리브레이션 데이터 생성 완료");
}

void addLogEntry(uint8_t level, uint8_t module, uint16_t code, uint32_t data) {
  LogEntry entry;
  entry.timestamp = millis();
  entry.level = level;
  entry.module = module;
  entry.code = code;
  entry.data = data;
  
  // 원형 로그 구현
  int logAddress = LOG_START_ADDR + (nextLogIndex * sizeof(LogEntry));
  
  // 로그 영역 크기 체크
  int maxLogEntries = (BACKUP_START_ADDR - LOG_START_ADDR) / sizeof(LogEntry);
  
  if (nextLogIndex >= maxLogEntries) {
    nextLogIndex = 0; // 처음부터 다시 시작
  }
  
  EEPROM.put(logAddress, entry);
  nextLogIndex++;
  
  // 디버그 출력
  Serial.print("로그 추가: Level=");
  Serial.print(level);
  Serial.print(", Module=");
  Serial.print(module);
  Serial.print(", Code=");
  Serial.print(code);
  Serial.print(", Data=");
  Serial.println(data);
}

void handleUserCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "config") {
      printCurrentConfig();
    }
    else if (command == "calib") {
      printCalibrationData();
    }
    else if (command == "logs") {
      printLogEntries();
    }
    else if (command.startsWith("set ")) {
      handleSetCommand(command);
    }
    else if (command.startsWith("cal ")) {
      handleCalibrationCommand(command);
    }
    else if (command == "backup") {
      backupConfiguration();
    }
    else if (command == "restore") {
      restoreConfiguration();
    }
    else if (command == "reset") {
      resetToDefaults();
    }
    else if (command == "info") {
      printSystemInfo();
    }
    else if (command == "help") {
      printHelpMenu();
    }
    else {
      Serial.println("알 수 없는 명령: " + command);
    }
  }
}

void printCurrentConfig() {
  Serial.println("\n=== 현재 시스템 설정 ===");
  Serial.print("버전: ");
  Serial.println(config.version);
  Serial.print("쓰기 횟수: ");
  Serial.println(config.writeCount);
  Serial.print("마지막 수정: ");
  Serial.println(config.lastModified);
  
  Serial.println("\n[센서 설정]");
  Serial.print("샘플링 레이트: ");
  Serial.print(config.sensors.sampleRate);
  Serial.println("ms");
  
  for (int i = 0; i < 8; i++) {
    Serial.print("센서 ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(config.sensors.enabled[i] ? "활성" : "비활성");
    Serial.print(", 오프셋=");
    Serial.print(config.sensors.offset[i], 3);
    Serial.print(", 게인=");
    Serial.println(config.sensors.gain[i], 3);
  }
  
  Serial.println("\n[로깅 설정]");
  Serial.print("활성화: ");
  Serial.println(config.logging.enabled ? "예" : "아니오");
  Serial.print("간격: ");
  Serial.print(config.logging.interval);
  Serial.println("ms");
  Serial.print("최대 파일 크기: ");
  Serial.print(config.logging.maxFileSize);
  Serial.println("KB");
  
  Serial.println("\n[통신 설정]");
  Serial.print("WiFi SSID: ");
  Serial.println(config.communication.wifiSSID);
  Serial.print("시리얼 속도: ");
  Serial.println(config.communication.baudRate);
  Serial.print("블루투스: ");
  Serial.println(config.communication.bluetoothEnabled ? "활성" : "비활성");
  
  Serial.println("=======================\n");
}

void printCalibrationData() {
  Serial.println("\n=== 캘리브레이션 데이터 ===");
  Serial.print("버전: ");
  Serial.println(calibration.version);
  
  for (int i = 0; i < 8; i++) {
    Serial.print("센서 ");
    Serial.print(i);
    Serial.print(": 선형(");
    Serial.print(calibration.sensors.linearCoeff[i][0], 4);
    Serial.print("x + ");
    Serial.print(calibration.sensors.linearCoeff[i][1], 4);
    Serial.print("), 기준값=");
    Serial.println(calibration.sensors.referenceValues[i], 3);
  }
  
  Serial.println("==========================\n");
}

void printLogEntries() {
  Serial.println("\n=== 시스템 로그 ===");
  
  int maxLogEntries = (BACKUP_START_ADDR - LOG_START_ADDR) / sizeof(LogEntry);
  int displayCount = min(10, maxLogEntries); // 최근 10개만 표시
  
  for (int i = 0; i < displayCount; i++) {
    int logIndex = (nextLogIndex - i - 1 + maxLogEntries) % maxLogEntries;
    int logAddress = LOG_START_ADDR + (logIndex * sizeof(LogEntry));
    
    LogEntry entry;
    EEPROM.get(logAddress, entry);
    
    if (entry.timestamp == 0) continue; // 빈 엔트리
    
    Serial.print("Time: ");
    Serial.print(entry.timestamp);
    Serial.print(", Level: ");
    
    switch (entry.level) {
      case 0: Serial.print("INFO"); break;
      case 1: Serial.print("WARN"); break;
      case 2: Serial.print("ERROR"); break;
      default: Serial.print("UNKNOWN"); break;
    }
    
    Serial.print(", Module: ");
    Serial.print(entry.module);
    Serial.print(", Code: ");
    Serial.print(entry.code);
    Serial.print(", Data: ");
    Serial.println(entry.data);
  }
  
  Serial.println("==================\n");
}

void handleSetCommand(String command) {
  // set sensor 0 offset 1.5
  // set logging interval 2000
  
  int firstSpace = command.indexOf(' ', 4);
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  int thirdSpace = command.indexOf(' ', secondSpace + 1);
  
  if (firstSpace == -1 || secondSpace == -1) {
    Serial.println("사용법: set <category> <item> <parameter> <value>");
    return;
  }
  
  String category = command.substring(4, firstSpace);
  String item = command.substring(firstSpace + 1, secondSpace);
  
  if (category == "sensor") {
    int sensorNum = item.toInt();
    if (sensorNum < 0 || sensorNum >= 8) {
      Serial.println("센서 번호는 0-7 범위여야 합니다");
      return;
    }
    
    if (thirdSpace == -1) {
      Serial.println("사용법: set sensor <num> <param> <value>");
      return;
    }
    
    String param = command.substring(secondSpace + 1, thirdSpace);
    float value = command.substring(thirdSpace + 1).toFloat();
    
    if (param == "offset") {
      config.sensors.offset[sensorNum] = value;
      Serial.print("센서 ");
      Serial.print(sensorNum);
      Serial.print(" 오프셋을 ");
      Serial.print(value, 3);
      Serial.println("로 설정");
    }
    else if (param == "gain") {
      config.sensors.gain[sensorNum] = value;
      Serial.print("센서 ");
      Serial.print(sensorNum);
      Serial.print(" 게인을 ");
      Serial.print(value, 3);
      Serial.println("로 설정");
    }
    
    saveSystemConfig();
  }
  else if (category == "logging") {
    if (thirdSpace == -1) {
      String param = command.substring(secondSpace + 1);
      
      if (item == "enable") {
        config.logging.enabled = (param == "true" || param == "1");
        Serial.print("로깅을 ");
        Serial.println(config.logging.enabled ? "활성화" : "비활성화");
      }
    } else {
      String param = command.substring(secondSpace + 1, thirdSpace);
      int value = command.substring(thirdSpace + 1).toInt();
      
      if (param == "interval") {
        config.logging.interval = value;
        Serial.print("로깅 간격을 ");
        Serial.print(value);
        Serial.println("ms로 설정");
      }
    }
    
    saveSystemConfig();
  }
}

void backupConfiguration() {
  // 현재 설정을 백업 영역에 저장 (이미 saveSystemConfig에서 수행됨)
  Serial.println("설정 백업 완료");
  
  addLogEntry(0, 1, 2000, millis());
}

void restoreConfiguration() {
  if (loadConfigFromAddress(BACKUP_START_ADDR, &config)) {
    if (validateConfigCRC(&config)) {
      // 메인 위치에 복원
      EEPROM.put(CONFIG_START_ADDR, config);
      Serial.println("백업에서 설정 복원 완료");
      addLogEntry(0, 1, 2001, millis());
    } else {
      Serial.println("백업 데이터 손상됨");
      addLogEntry(2, 1, 2002, millis());
    }
  } else {
    Serial.println("백업 데이터 없음");
  }
}

void resetToDefaults() {
  Serial.println("기본값으로 재설정 중...");
  
  createDefaultConfig();
  saveSystemConfig();
  
  createDefaultCalibration();
  saveCalibrationData();
  
  Serial.println("기본값 재설정 완료");
  addLogEntry(1, 1, 3000, millis());
}

void monitorEEPROMWear() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck >= 60000) { // 1분마다 체크
    if (config.writeCount > 10000) { // 10,000회 이상 쓰기
      Serial.print("경고: EEPROM 쓰기 횟수가 많습니다 (");
      Serial.print(config.writeCount);
      Serial.println("회)");
      
      addLogEntry(1, 2, 4000, config.writeCount);
    }
    
    lastCheck = millis();
  }
}

void printSystemInfo() {
  Serial.println("\n=== EEPROM 시스템 정보 ===");
  Serial.print("EEPROM 크기: ");
  Serial.print(EEPROM_SIZE);
  Serial.println(" bytes");
  
  Serial.println("메모리 맵:");
  Serial.print("  설정: ");
  Serial.print(CONFIG_START_ADDR);
  Serial.print(" - ");
  Serial.println(CONFIG_START_ADDR + sizeof(SystemConfig) - 1);
  
  Serial.print("  캘리브레이션: ");
  Serial.print(CALIBRATION_START_ADDR);
  Serial.print(" - ");
  Serial.println(CALIBRATION_START_ADDR + sizeof(CalibrationData) - 1);
  
  Serial.print("  로그: ");
  Serial.print(LOG_START_ADDR);
  Serial.print(" - ");
  Serial.println(BACKUP_START_ADDR - 1);
  
  Serial.print("  백업: ");
  Serial.print(BACKUP_START_ADDR);
  Serial.print(" - ");
  Serial.println(BACKUP_START_ADDR + sizeof(SystemConfig) - 1);
  
  Serial.print("사용된 공간: ");
  int usedSpace = sizeof(SystemConfig) * 2 + sizeof(CalibrationData) + 
                  (BACKUP_START_ADDR - LOG_START_ADDR);
  Serial.print(usedSpace);
  Serial.print(" / ");
  Serial.print(EEPROM_SIZE);
  Serial.print(" bytes (");
  Serial.print((usedSpace * 100) / EEPROM_SIZE);
  Serial.println("%)");
  
  Serial.println("===========================\n");
}

void printHelpMenu() {
  Serial.println("\n=== 명령어 도움말 ===");
  Serial.println("config                    - 현재 설정 표시");
  Serial.println("calib                     - 캘리브레이션 데이터 표시");
  Serial.println("logs                      - 시스템 로그 표시");
  Serial.println("set sensor <n> offset <v> - 센서 오프셋 설정");
  Serial.println("set sensor <n> gain <v>   - 센서 게인 설정");
  Serial.println("set logging interval <v>  - 로깅 간격 설정");
  Serial.println("backup                    - 설정 백업");
  Serial.println("restore                   - 백업에서 복원");
  Serial.println("reset                     - 기본값으로 재설정");
  Serial.println("info                      - 시스템 정보");
  Serial.println("help                      - 이 도움말");
  Serial.println("====================\n");
}

// 유틸리티 함수들
uint32_t readUint32FromEEPROM(int address) {
  uint32_t value;
  EEPROM.get(address, value);
  return value;
}

void writeUint32ToEEPROM(int address, uint32_t value) {
  EEPROM.put(address, value);
}

uint16_t calculateCRC16(uint8_t* data, int length) {
  uint16_t crc = 0xFFFF;
  
  for (int i = 0; i < length; i++) {
    crc ^= data[i];
    
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  
  return crc;
}
```

## 🔧 실습 3: 클라우드 동기화 시스템

### WiFi를 이용한 데이터 업로드 및 원격 설정

```cpp
/*
  Cloud_Data_Sync.ino
  ESP32를 이용한 클라우드 데이터 동기화 시스템
  
  특징:
  - 실시간 클라우드 업로드
  - 오프라인 버퍼링
  - 원격 설정 동기화
  - OTA 업데이트 지원
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>
#include <Update.h>

// WiFi 설정
const char* WIFI_SSID = "YourWiFiSSID";
const char* WIFI_PASSWORD = "YourWiFiPassword";

// 클라우드 서비스 설정
const char* CLOUD_SERVER = "https://your-iot-server.com";
const char* API_KEY = "your_api_key_here";
const char* DEVICE_ID = "sensor_node_001";

// 핀 설정
const int SD_CS_PIN = 5;
const int LED_WIFI = 2;
const int LED_SYNC = 4;

// 동기화 설정
const int SYNC_INTERVAL = 30000;      // 30초마다 동기화
const int BATCH_SIZE = 50;            // 한 번에 업로드할 레코드 수
const int MAX_OFFLINE_RECORDS = 1000; // 최대 오프라인 저장 레코드

// 데이터 구조체
struct SensorRecord {
  uint32_t timestamp;
  float temperature;
  float humidity;
  float pressure;
  float batteryVoltage;
  int wifiRSSI;
  uint8_t status;
};

// 시스템 상태
struct SyncStatus {
  bool wifiConnected;
  bool cloudConnected;
  unsigned long totalUploaded;
  unsigned long pendingRecords;
  unsigned long lastSyncTime;
  unsigned long syncErrors;
  String lastError;
  float uploadSpeed;  // records/sec
} syncStatus;

// 큐 관리
const int QUEUE_SIZE = 100;
SensorRecord recordQueue[QUEUE_SIZE];
volatile int queueHead = 0;
volatile int queueTail = 0;
volatile int queueCount = 0;

// NTP 설정
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET = 32400;  // GMT+9 (한국)

void setup() {
  Serial.begin(115200);
  
  // LED 핀 설정
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_SYNC, OUTPUT);
  
  // 상태 초기화
  initializeSyncStatus();
  
  // WiFi 연결
  connectToWiFi();
  
  // NTP 시간 동기화
  if (syncStatus.wifiConnected) {
    configTime(GMT_OFFSET, 0, NTP_SERVER);
    Serial.println("NTP 시간 동기화 완료");
  }
  
  // SD카드 초기화
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD카드 초기화 실패!");
  } else {
    Serial.println("SD카드 초기화 성공");
    
    // 기존 오프라인 데이터 복구
    recoverOfflineData();
  }
  
  // 클라우드 연결 테스트
  testCloudConnection();
  
  // 데이터 수집 타이머 시작
  setupDataCollectionTimer();
  
  Serial.println("클라우드 동기화 시스템 시작");
  printSystemStatus();
}

void loop() {
  // WiFi 연결 상태 확인
  checkWiFiConnection();
  
  // 클라우드 동기화
  if (syncStatus.wifiConnected && 
      millis() - syncStatus.lastSyncTime >= SYNC_INTERVAL) {
    performCloudSync();
  }
  
  // 큐 처리 (SD카드 백업)
  processRecordQueue();
  
  // 상태 LED 업데이트
  updateStatusLEDs();
  
  // 원격 명령 확인
  checkRemoteCommands();
  
  // 시리얼 명령 처리
  handleSerialCommands();
  
  // 상태 출력 (1분마다)
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 60000) {
    printSystemStatus();
    lastStatusPrint = millis();
  }
  
  delay(100);
}

void initializeSyncStatus() {
  syncStatus.wifiConnected = false;
  syncStatus.cloudConnected = false;
  syncStatus.totalUploaded = 0;
  syncStatus.pendingRecords = 0;
  syncStatus.lastSyncTime = 0;
  syncStatus.syncErrors = 0;
  syncStatus.lastError = "";
  syncStatus.uploadSpeed = 0.0;
}

void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi 연결 중");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    syncStatus.wifiConnected = true;
    Serial.println("\nWiFi 연결 성공!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    syncStatus.wifiConnected = false;
    Serial.println("\nWiFi 연결 실패!");
  }
}

void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck >= 10000) { // 10초마다 체크
    bool currentStatus = (WiFi.status() == WL_CONNECTED);
    
    if (currentStatus != syncStatus.wifiConnected) {
      syncStatus.wifiConnected = currentStatus;
      
      if (currentStatus) {
        Serial.println("WiFi 재연결됨");
        configTime(GMT_OFFSET, 0, NTP_SERVER);
      } else {
        Serial.println("WiFi 연결 끊어짐");
        syncStatus.cloudConnected = false;
      }
    }
    
    lastCheck = millis();
  }
}

void testCloudConnection() {
  if (!syncStatus.wifiConnected) {
    syncStatus.cloudConnected = false;
    return;
  }
  
  HTTPClient http;
  http.begin(String(CLOUD_SERVER) + "/api/ping");
  http.addHeader("Authorization", "Bearer " + String(API_KEY));
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    syncStatus.cloudConnected = true;
    Serial.println("클라우드 연결 성공");
  } else {
    syncStatus.cloudConnected = false;
    Serial.print("클라우드 연결 실패: ");
    Serial.println(httpCode);
    syncStatus.lastError = "HTTP " + String(httpCode);
  }
  
  http.end();
}

void setupDataCollectionTimer() {
  // 하드웨어 타이머를 사용한 정확한 데이터 수집
  hw_timer_t* timer = timerBegin(0, 80, true);  // 1MHz 타이머
  timerAttachInterrupt(timer, &collectSensorDataISR, true);
  timerAlarmWrite(timer, 5000000, true);  // 5초마다
  timerAlarmEnable(timer);
  
  Serial.println("데이터 수집 타이머 설정 완료");
}

void IRAM_ATTR collectSensorDataISR() {
  // 인터럽트에서는 최소한의 작업만 수행
  static unsigned long lastCollection = 0;
  
  if (millis() - lastCollection >= 5000) {
    // 플래그 설정으로 메인 루프에서 처리하도록 함
    // (실제 구현에서는 FreeRTOS 큐 사용 권장)
    lastCollection = millis();
  }
}

void collectSensorData() {
  SensorRecord record;
  
  // 현재 시간 (Unix timestamp)
  time_t now;
  time(&now);
  record.timestamp = now;
  
  // 센서 데이터 읽기 (시뮬레이션)
  record.temperature = 25.0 + random(-50, 51) / 10.0;
  record.humidity = 60.0 + random(-200, 201) / 10.0;
  record.pressure = 1013.25 + random(-100, 101) / 10.0;
  record.batteryVoltage = 3.7 + random(-50, 51) / 100.0;
  record.wifiRSSI = WiFi.RSSI();
  
  // 상태 정보
  record.status = (syncStatus.wifiConnected ? 0x01 : 0x00) |
                  (syncStatus.cloudConnected ? 0x02 : 0x00);
  
  // 큐에 추가
  addToQueue(record);
  
  Serial.print("센서 데이터 수집: T=");
  Serial.print(record.temperature, 1);
  Serial.print("°C, H=");
  Serial.print(record.humidity, 1);
  Serial.println("%");
}

void addToQueue(SensorRecord record) {
  if (queueCount >= QUEUE_SIZE) {
    Serial.println("경고: 큐 오버플로우!");
    // 가장 오래된 데이터 제거
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount--;
  }
  
  recordQueue[queueHead] = record;
  queueHead = (queueHead + 1) % QUEUE_SIZE;
  queueCount++;
  
  syncStatus.pendingRecords++;
}

void processRecordQueue() {
  // 큐가 비어있으면 리턴
  if (queueCount == 0) return;
  
  // SD카드에 백업 (오프라인 저장)
  if (queueCount >= 10) {  // 10개씩 배치 처리
    saveRecordsToSD();
  }
}

void saveRecordsToSD() {
  File dataFile = SD.open("/offline_data.json", FILE_APPEND);
  
  if (!dataFile) {
    Serial.println("SD카드 파일 열기 실패");
    return;
  }
  
  int recordsToSave = min(queueCount, 10);
  
  for (int i = 0; i < recordsToSave; i++) {
    SensorRecord record = recordQueue[queueTail];
    
    // JSON 형식으로 저장
    DynamicJsonDocument doc(512);
    doc["timestamp"] = record.timestamp;
    doc["device_id"] = DEVICE_ID;
    doc["temperature"] = record.temperature;
    doc["humidity"] = record.humidity;
    doc["pressure"] = record.pressure;
    doc["battery"] = record.batteryVoltage;
    doc["rssi"] = record.wifiRSSI;
    doc["status"] = record.status;
    
    String jsonString;
    serializeJson(doc, jsonString);
    dataFile.println(jsonString);
    
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount--;
  }
  
  dataFile.close();
  Serial.print("SD카드에 ");
  Serial.print(recordsToSave);
  Serial.println("개 레코드 저장");
}

void performCloudSync() {
  if (!syncStatus.wifiConnected || !syncStatus.cloudConnected) {
    testCloudConnection();
    if (!syncStatus.cloudConnected) {
      syncStatus.lastSyncTime = millis();
      return;
    }
  }
  
  unsigned long startTime = millis();
  int uploadedCount = 0;
  
  // 큐에서 데이터 업로드
  uploadedCount += uploadQueueData();
  
  // SD카드 오프라인 데이터 업로드
  uploadedCount += uploadOfflineData();
  
  // 성능 계산
  unsigned long elapsedTime = millis() - startTime;
  if (elapsedTime > 0) {
    syncStatus.uploadSpeed = (uploadedCount * 1000.0) / elapsedTime;
  }
  
  syncStatus.totalUploaded += uploadedCount;
  syncStatus.lastSyncTime = millis();
  
  if (uploadedCount > 0) {
    Serial.print("클라우드 동기화 완료: ");
    Serial.print(uploadedCount);
    Serial.print("개 레코드, ");
    Serial.print(syncStatus.uploadSpeed, 2);
    Serial.println(" records/sec");
  }
}

int uploadQueueData() {
  if (queueCount == 0) return 0;
  
  int recordsToUpload = min(queueCount, BATCH_SIZE);
  DynamicJsonDocument doc(8192);
  JsonArray dataArray = doc.createNestedArray("data");
  
  // 배치 생성
  for (int i = 0; i < recordsToUpload; i++) {
    JsonObject record = dataArray.createNestedObject();
    SensorRecord data = recordQueue[queueTail];
    
    record["timestamp"] = data.timestamp;
    record["device_id"] = DEVICE_ID;
    record["temperature"] = data.temperature;
    record["humidity"] = data.humidity;
    record["pressure"] = data.pressure;
    record["battery"] = data.batteryVoltage;
    record["rssi"] = data.wifiRSSI;
    record["status"] = data.status;
    
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount--;
  }
  
  // HTTP POST 전송
  String jsonString;
  serializeJson(doc, jsonString);
  
  HTTPClient http;
  http.begin(String(CLOUD_SERVER) + "/api/data");
  http.addHeader("Authorization", "Bearer " + String(API_KEY));
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(jsonString);
  
  if (httpCode == HTTP_CODE_OK) {
    syncStatus.pendingRecords -= recordsToUpload;
    http.end();
    return recordsToUpload;
  } else {
    // 업로드 실패 - 큐에 다시 추가
    for (int i = recordsToUpload - 1; i >= 0; i--) {
      queueTail = (queueTail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
      queueCount++;
    }
    
    syncStatus.syncErrors++;
    syncStatus.lastError = "Upload failed: HTTP " + String(httpCode);
    Serial.println(syncStatus.lastError);
  }
  
  http.end();
  return 0;
}

int uploadOfflineData() {
  File dataFile = SD.open("/offline_data.json");
  if (!dataFile) return 0;
  
  int uploadedCount = 0;
  String line;
  
  // 파일에서 라인별로 읽어서 업로드
  while (dataFile.available() && uploadedCount < BATCH_SIZE) {
    line = dataFile.readStringUntil('\n');
    
    if (line.length() > 0) {
      if (uploadSingleRecord(line)) {
        uploadedCount++;
      } else {
        break; // 업로드 실패시 중단
      }
    }
  }
  
  dataFile.close();
  
  // 업로드된 데이터 제거
  if (uploadedCount > 0) {
    removeUploadedLines(uploadedCount);
  }
  
  return uploadedCount;
}

bool uploadSingleRecord(String jsonData) {
  HTTPClient http;
  http.begin(String(CLOUD_SERVER) + "/api/data/single");
  http.addHeader("Authorization", "Bearer " + String(API_KEY));
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(jsonData);
  bool success = (httpCode == HTTP_CODE_OK);
  
  if (!success) {
    syncStatus.syncErrors++;
  }
  
  http.end();
  return success;
}

void removeUploadedLines(int lineCount) {
  // 업로드된 라인들을 파일에서 제거
  File originalFile = SD.open("/offline_data.json");
  File tempFile = SD.open("/temp_data.json", FILE_WRITE);
  
  if (!originalFile || !tempFile) {
    if (originalFile) originalFile.close();
    if (tempFile) tempFile.close();
    return;
  }
  
  // 처음 lineCount개 라인 건너뛰기
  for (int i = 0; i < lineCount && originalFile.available(); i++) {
    originalFile.readStringUntil('\n');
  }
  
  // 나머지 데이터 복사
  while (originalFile.available()) {
    String line = originalFile.readStringUntil('\n');
    if (line.length() > 0) {
      tempFile.println(line);
    }
  }
  
  originalFile.close();
  tempFile.close();
  
  // 파일 교체
  SD.remove("/offline_data.json");
  SD.rename("/temp_data.json", "/offline_data.json");
}

void recoverOfflineData() {
  File dataFile = SD.open("/offline_data.json");
  if (!dataFile) {
    Serial.println("오프라인 데이터 없음");
    return;
  }
  
  int lineCount = 0;
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    if (line.length() > 0) {
      lineCount++;
    }
  }
  
  dataFile.close();
  
  syncStatus.pendingRecords += lineCount;
  Serial.print("오프라인 데이터 복구: ");
  Serial.print(lineCount);
  Serial.println("개 레코드");
}

void checkRemoteCommands() {
  if (!syncStatus.cloudConnected) return;
  
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 60000) return; // 1분마다
  
  HTTPClient http;
  http.begin(String(CLOUD_SERVER) + "/api/commands/" + String(DEVICE_ID));
  http.addHeader("Authorization", "Bearer " + String(API_KEY));
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, response);
    
    if (doc.containsKey("commands")) {
      JsonArray commands = doc["commands"];
      
      for (JsonObject cmd : commands) {
        String command = cmd["command"];
        String value = cmd["value"];
        
        executeRemoteCommand(command, value);
        
        // 명령 실행 완료 알림
        acknowledgeCommand(cmd["id"]);
      }
    }
  }
  
  http.end();
  lastCheck = millis();
}

void executeRemoteCommand(String command, String value) {
  Serial.print("원격 명령 실행: ");
  Serial.print(command);
  Serial.print(" = ");
  Serial.println(value);
  
  if (command == "sync_interval") {
    // 동기화 간격 변경
    int newInterval = value.toInt();
    if (newInterval >= 10000 && newInterval <= 3600000) {
      // SYNC_INTERVAL = newInterval; // const이므로 실제로는 변수 사용
      Serial.print("동기화 간격 변경: ");
      Serial.println(newInterval);
    }
  }
  else if (command == "restart") {
    Serial.println("원격 재시작 명령 수신");
    delay(1000);
    ESP.restart();
  }
  else if (command == "ota_update") {
    performOTAUpdate(value); // 펌웨어 URL
  }
  else if (command == "clear_data") {
    // 로컬 데이터 삭제
    SD.remove("/offline_data.json");
    Serial.println("로컬 데이터 삭제됨");
  }
}

void acknowledgeCommand(int commandId) {
  HTTPClient http;
  http.begin(String(CLOUD_SERVER) + "/api/commands/" + String(commandId) + "/ack");
  http.addHeader("Authorization", "Bearer " + String(API_KEY));
  http.addHeader("Content-Type", "application/json");
  
  DynamicJsonDocument doc(256);
  doc["device_id"] = DEVICE_ID;
  doc["status"] = "completed";
  doc["timestamp"] = time(nullptr);
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  http.POST(jsonString);
  http.end();
}

void performOTAUpdate(String firmwareUrl) {
  Serial.println("OTA 업데이트 시작: " + firmwareUrl);
  
  HTTPClient http;
  http.begin(firmwareUrl);
  
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    
    if (contentLength > 0) {
      bool canBegin = Update.begin(contentLength);
      
      if (canBegin) {
        WiFiClient* client = http.getStreamPtr();
        size_t written = Update.writeStream(*client);
        
        if (written == contentLength) {
          Serial.println("OTA 업데이트 완료");
          
          if (Update.end()) {
            Serial.println("업데이트 성공 - 재시작합니다");
            ESP.restart();
          } else {
            Serial.println("업데이트 완료 실패");
          }
        } else {
          Serial.println("OTA 쓰기 실패");
        }
      } else {
        Serial.println("OTA 시작 실패");
      }
    }
  }
  
  http.end();
}

void updateStatusLEDs() {
  // WiFi LED
  digitalWrite(LED_WIFI, syncStatus.wifiConnected ? HIGH : LOW);
  
  // 동기화 LED (깜빡임)
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  if (syncStatus.cloudConnected && syncStatus.pendingRecords > 0) {
    if (millis() - lastBlink >= 200) {
      ledState = !ledState;
      digitalWrite(LED_SYNC, ledState);
      lastBlink = millis();
    }
  } else if (syncStatus.cloudConnected) {
    digitalWrite(LED_SYNC, HIGH);
  } else {
    digitalWrite(LED_SYNC, LOW);
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "status") {
      printDetailedStatus();
    }
    else if (command == "sync") {
      performCloudSync();
    }
    else if (command == "test") {
      collectSensorData();
    }
    else if (command == "wifi") {
      Serial.print("WiFi 상태: ");
      Serial.println(WiFi.status());
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI());
    }
    else if (command == "clear") {
      SD.remove("/offline_data.json");
      syncStatus.pendingRecords = 0;
      Serial.println("오프라인 데이터 삭제됨");
    }
    else if (command == "help") {
      printHelpMenu();
    }
  }
}

void printSystemStatus() {
  Serial.println("\n=== 시스템 상태 ===");
  Serial.print("WiFi: ");
  Serial.println(syncStatus.wifiConnected ? "연결됨" : "연결 안됨");
  
  Serial.print("클라우드: ");
  Serial.println(syncStatus.cloudConnected ? "연결됨" : "연결 안됨");
  
  Serial.print("대기 중인 레코드: ");
  Serial.println(syncStatus.pendingRecords);
  
  Serial.print("총 업로드: ");
  Serial.println(syncStatus.totalUploaded);
  
  Serial.print("업로드 속도: ");
  Serial.print(syncStatus.uploadSpeed, 2);
  Serial.println(" records/sec");
  
  Serial.print("동기화 오류: ");
  Serial.println(syncStatus.syncErrors);
  
  if (syncStatus.lastError.length() > 0) {
    Serial.print("마지막 오류: ");
    Serial.println(syncStatus.lastError);
  }
  
  Serial.print("마지막 동기화: ");
  if (syncStatus.lastSyncTime > 0) {
    Serial.print((millis() - syncStatus.lastSyncTime) / 1000);
    Serial.println("초 전");
  } else {
    Serial.println("없음");
  }
  
  Serial.println("==================\n");
}

void printDetailedStatus() {
  printSystemStatus();
  
  Serial.println("=== 큐 상태 ===");
  Serial.print("큐 크기: ");
  Serial.print(queueCount);
  Serial.print(" / ");
  Serial.println(QUEUE_SIZE);
  
  Serial.print("메모리 사용량: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  
  Serial.print("업타임: ");
  Serial.print(millis() / 1000);
  Serial.println("초");
  
  time_t now;
  time(&now);
  Serial.print("현재 시간: ");
  Serial.println(ctime(&now));
  
  Serial.println("===============\n");
}

void printHelpMenu() {
  Serial.println("\n=== 명령어 도움말 ===");
  Serial.println("status  - 시스템 상태");
  Serial.println("sync    - 즉시 동기화");
  Serial.println("test    - 센서 데이터 수집");
  Serial.println("wifi    - WiFi 상태");
  Serial.println("clear   - 오프라인 데이터 삭제");
  Serial.println("help    - 이 도움말");
  Serial.println("====================\n");
}
```

## 📊 저장소 성능 비교 및 최적화

### 성능 벤치마크 결과

```
┌─────────────────────────────────────────────────────────────────┐
│                    저장소 성능 비교표                            │
├─────────────┬─────────────┬─────────────┬─────────────────────┤
│   저장소    │  쓰기 속도   │  읽기 속도   │       특징          │
├─────────────┼─────────────┼─────────────┼─────────────────────┤
│    SRAM     │   1μs/byte  │   0.1μs/byte │ 가장 빠름, 휘발성   │
│   EEPROM    │   3.3ms/byte│   1μs/byte  │ 느림, 비휘발성      │
│   Flash     │   50μs/page │   1μs/byte  │ 페이지 단위 쓰기    │
│  SD Card    │   1ms/block │   0.1ms/block│ 대용량, 블록 단위   │
│  Cloud      │   100ms/req │   50ms/req  │ 네트워크 의존적     │
└─────────────┴─────────────┴─────────────┴─────────────────────┘
```

## 🚀 다음 단계

데이터 로깅 및 저장소를 완전히 마스터했습니다! 다음 강의에서는:

- **Lesson 14**: 전력 관리 및 저전력 설계
- **Lesson 15**: 실전 프로젝트 - IoT 스마트홈 시스템

이제 고성능 데이터 로깅, 클라우드 동기화, 설정 관리 등 실제 상용 제품에서 사용되는 모든 저장소 기술을 활용할 수 있습니다!

---

*아두이노 마스터 과정 - Lesson 13 완료*  
*Created by Arduino Expert Teacher*  
*© 2024 Arduino Complete Guide*