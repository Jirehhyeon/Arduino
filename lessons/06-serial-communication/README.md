# Lesson 06: 시리얼 통신의 이해

## 🎯 학습 목표
- 시리얼 통신의 원리와 종류 이해하기
- Serial 라이브러리 완전 활용하기
- 양방향 데이터 통신 구현하기
- 다양한 시리얼 통신 프로토콜 비교하기
- 실시간 데이터 모니터링 시스템 구축하기

## 📖 1. 시리얼 통신 기초

### 1.1 시리얼 통신이란?
```
시리얼 통신 (Serial Communication):
데이터를 한 비트씩 순차적으로 전송하는 통신 방식

패러럴 vs 시리얼:

패러럴 통신:          시리얼 통신:
┌─ Bit 0             ┌─ Data
├─ Bit 1             │  ┌─┐ ┌─┐ ┌─┐ ┌─┐
├─ Bit 2       vs    │  │0│ │1│ │0│ │1│
├─ Bit 3             │  └─┘ └─┘ └─┘ └─┘
├─ Bit 4             └─ 시간 →
├─ Bit 5
├─ Bit 6
└─ Bit 7

장점: 선 1개만 필요, 장거리 통신 가능
단점: 속도 상대적으로 느림
```

### 1.2 UART (Universal Asynchronous Receiver Transmitter)
```
UART 신호 구조:

시작비트  데이터비트(8bit)  패리티  정지비트
    ↓     ↓  ↓  ↓  ↓  ↓     ↓      ↓
   ┌─┐   ┌─┐┌─┐┌─┐┌─┐┌─┐   ┌─┐    ┌─┐
───┘ └───┘ ┘┘ ┘┘ ┘┘ ┘┘ ┘───┘ └────┘ └───
   START  D0 D1 D2 D3 D4   PARITY  STOP

- 비동기 통신 (클럭 신호 없음)
- 송신(TX)과 수신(RX) 선으로 구성
- 보드레이트로 통신 속도 설정
```

### 1.3 보드레이트 (Baud Rate)
```
일반적인 보드레이트:
300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200

9600 bps 예시:
- 1초에 9600비트 전송
- 1비트당 약 104μs
- 8N1 설정시: 실제 데이터는 약 960바이트/초
```

## 💻 2. Arduino Serial 라이브러리

### 2.1 기본 Serial 함수들
```cpp
// 시리얼 통신 초기화
Serial.begin(9600);           // 9600 bps로 통신 시작

// 데이터 전송
Serial.print("Hello");        // 문자열 전송
Serial.println("World");      // 문자열 + 개행 전송
Serial.write(65);             // 바이트 데이터 전송 (ASCII 'A')

// 데이터 수신
Serial.available();           // 수신 버퍼에 있는 바이트 수
Serial.read();                // 1바이트 읽기
Serial.readString();          // 문자열 읽기
Serial.parseInt();            // 정수 읽기
Serial.parseFloat();          // 실수 읽기

// 버퍼 관리
Serial.flush();               // 송신 버퍼 비우기 (전송 완료까지 대기)
```

### 2.2 기본 시리얼 통신 예제
```cpp
/*
  Serial_Basic.ino
  기본 시리얼 통신 - 에코 시스템
*/

void setup() {
  // 시리얼 통신 시작 (9600 bps)
  Serial.begin(9600);
  
  // 시작 메시지
  Serial.println("===== Arduino Serial Echo System =====");
  Serial.println("입력한 내용이 그대로 출력됩니다.");
  Serial.println("명령어:");
  Serial.println("- 'led on' : LED 켜기");
  Serial.println("- 'led off' : LED 끄기");
  Serial.println("- 'status' : 현재 상태 보기");
  Serial.println("======================================");
  
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // 시리얼 데이터가 있는지 확인
  if (Serial.available() > 0) {
    
    // 문자열 읽기 (개행문자까지)
    String inputString = Serial.readString();
    inputString.trim();  // 앞뒤 공백 제거
    
    // 입력된 내용 에코
    Serial.print("입력받은 내용: ");
    Serial.println(inputString);
    
    // 명령어 처리
    processCommand(inputString);
  }
}

void processCommand(String command) {
  command.toLowerCase();  // 소문자로 변환
  
  if (command == "led on") {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("✅ LED가 켜졌습니다.");
    
  } else if (command == "led off") {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("❌ LED가 꺼졌습니다.");
    
  } else if (command == "status") {
    Serial.println("📊 현재 상태:");
    Serial.print("  - LED: ");
    Serial.println(digitalRead(LED_BUILTIN) ? "ON" : "OFF");
    Serial.print("  - 동작 시간: ");
    Serial.print(millis() / 1000);
    Serial.println("초");
    
  } else {
    Serial.println("❓ 알 수 없는 명령어입니다.");
    Serial.println("사용 가능한 명령어: led on, led off, status");
  }
}
```

## 📊 3. 실시간 데이터 모니터링

### 3.1 센서 데이터 실시간 전송
```cpp
/*
  Data_Monitor.ino
  실시간 센서 데이터 모니터링 시스템
*/

// 센서 핀 정의
const int TEMP_PIN = A0;      // 온도센서 (LM35)
const int LIGHT_PIN = A1;     // 조도센서 (LDR)
const int SOUND_PIN = A2;     // 사운드센서

// 데이터 전송 간격
unsigned long previousMillis = 0;
const long SEND_INTERVAL = 1000;  // 1초마다 전송

// 데이터 형식 설정
enum DataFormat {
  FORMAT_READABLE,    // 사람이 읽기 쉬운 형태
  FORMAT_CSV,         // CSV 형태
  FORMAT_JSON         // JSON 형태
};

DataFormat currentFormat = FORMAT_READABLE;

void setup() {
  Serial.begin(115200);  // 높은 보드레이트 사용
  
  Serial.println("🌡️ 실시간 센서 모니터링 시스템");
  Serial.println("명령어: 'readable', 'csv', 'json', 'help'");
  Serial.println("========================================");
  
  // 기준전압 설정 (더 정확한 측정을 위해)
  analogReference(DEFAULT);
}

void loop() {
  // 명령어 처리
  handleSerialCommands();
  
  // 주기적으로 센서 데이터 전송
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= SEND_INTERVAL) {
    previousMillis = currentMillis;
    sendSensorData();
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();
    command.toLowerCase();
    
    if (command == "readable") {
      currentFormat = FORMAT_READABLE;
      Serial.println("📖 출력 형식: 읽기 쉬운 형태");
      
    } else if (command == "csv") {
      currentFormat = FORMAT_CSV;
      Serial.println("📊 출력 형식: CSV");
      Serial.println("시간(ms),온도(°C),조도,소음,메모리(byte)");
      
    } else if (command == "json") {
      currentFormat = FORMAT_JSON;
      Serial.println("🔗 출력 형식: JSON");
      
    } else if (command == "help") {
      showHelp();
      
    } else {
      Serial.println("❓ 알 수 없는 명령어: " + command);
    }
  }
}

void sendSensorData() {
  // 센서 데이터 읽기
  float temperature = readTemperature();
  int lightLevel = analogRead(LIGHT_PIN);
  int soundLevel = analogRead(SOUND_PIN);
  unsigned long timestamp = millis();
  int freeMemory = getFreeMemory();
  
  // 선택된 형식으로 데이터 전송
  switch (currentFormat) {
    case FORMAT_READABLE:
      sendReadableFormat(timestamp, temperature, lightLevel, soundLevel, freeMemory);
      break;
      
    case FORMAT_CSV:
      sendCSVFormat(timestamp, temperature, lightLevel, soundLevel, freeMemory);
      break;
      
    case FORMAT_JSON:
      sendJSONFormat(timestamp, temperature, lightLevel, soundLevel, freeMemory);
      break;
  }
}

float readTemperature() {
  int sensorValue = analogRead(TEMP_PIN);
  float voltage = (sensorValue / 1023.0) * 5.0;
  return voltage * 100.0;  // LM35: 10mV/°C
}

void sendReadableFormat(unsigned long time, float temp, int light, int sound, int memory) {
  Serial.println("📊 센서 데이터 리포트");
  Serial.print("🕐 시간: ");
  Serial.print(time / 1000);
  Serial.println("초");
  
  Serial.print("🌡️ 온도: ");
  Serial.print(temp, 1);
  Serial.println("°C");
  
  Serial.print("💡 조도: ");
  Serial.print(light);
  Serial.print(" (");
  Serial.print(map(light, 0, 1023, 0, 100));
  Serial.println("%)");
  
  Serial.print("🔊 소음: ");
  Serial.println(sound);
  
  Serial.print("💾 메모리: ");
  Serial.print(memory);
  Serial.println(" bytes");
  
  Serial.println("------------------------");
}

void sendCSVFormat(unsigned long time, float temp, int light, int sound, int memory) {
  Serial.print(time);
  Serial.print(",");
  Serial.print(temp, 2);
  Serial.print(",");
  Serial.print(light);
  Serial.print(",");
  Serial.print(sound);
  Serial.print(",");
  Serial.println(memory);
}

void sendJSONFormat(unsigned long time, float temp, int light, int sound, int memory) {
  Serial.println("{");
  Serial.print("  \"timestamp\": ");
  Serial.print(time);
  Serial.println(",");
  
  Serial.print("  \"temperature\": ");
  Serial.print(temp, 2);
  Serial.println(",");
  
  Serial.print("  \"light\": ");
  Serial.print(light);
  Serial.println(",");
  
  Serial.print("  \"sound\": ");
  Serial.print(sound);
  Serial.println(",");
  
  Serial.print("  \"memory\": ");
  Serial.println(memory);
  Serial.println("}");
}

int getFreeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void showHelp() {
  Serial.println("📚 도움말");
  Serial.println("사용 가능한 명령어:");
  Serial.println("  readable - 읽기 쉬운 형식으로 출력");
  Serial.println("  csv      - CSV 형식으로 출력");
  Serial.println("  json     - JSON 형식으로 출력");
  Serial.println("  help     - 이 도움말 표시");
}
```

## 🌐 4. 고급 시리얼 통신 기법

### 4.1 이진 데이터 전송
```cpp
/*
  Binary_Communication.ino
  이진 데이터를 이용한 효율적인 통신
*/

// 데이터 패킷 구조체
struct SensorPacket {
  uint8_t header;        // 패킷 시작 마커 (0xAA)
  uint16_t timestamp;    // 타임스탬프 (2바이트)
  int16_t temperature;   // 온도 x100 (정수로 저장)
  uint16_t light;        // 조도값
  uint16_t sound;        // 소음값
  uint8_t checksum;      // 체크섬
} __attribute__((packed));  // 패딩 없이 패킹

const uint8_t PACKET_HEADER = 0xAA;

void setup() {
  Serial.begin(115200);
  Serial.println("Binary Communication Test");
  Serial.println("패킷 크기: " + String(sizeof(SensorPacket)) + " bytes");
}

void loop() {
  // 센서 데이터 읽기
  float temp = readTemperature();
  uint16_t light = analogRead(A1);
  uint16_t sound = analogRead(A2);
  
  // 패킷 생성
  SensorPacket packet;
  packet.header = PACKET_HEADER;
  packet.timestamp = millis() / 100;  // 100ms 단위
  packet.temperature = (int16_t)(temp * 100);  // 소수점 2자리까지
  packet.light = light;
  packet.sound = sound;
  packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
  
  // 이진 데이터 전송
  Serial.write((uint8_t*)&packet, sizeof(packet));
  
  delay(500);
}

float readTemperature() {
  int sensorValue = analogRead(A0);
  float voltage = (sensorValue / 1023.0) * 5.0;
  return voltage * 100.0;
}

uint8_t calculateChecksum(uint8_t* data, int length) {
  uint8_t checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i];  // XOR 체크섬
  }
  return checksum;
}
```

### 4.2 프로토콜 파서 구현
```cpp
/*
  Protocol_Parser.ino
  사용자 정의 프로토콜 파서
*/

// 명령어 타입 정의
enum CommandType {
  CMD_SET_LED = 0x01,
  CMD_GET_SENSOR = 0x02,
  CMD_SET_PWM = 0x03,
  CMD_RESET = 0x04
};

// 응답 타입 정의
enum ResponseType {
  RESP_OK = 0x80,
  RESP_ERROR = 0x81,
  RESP_DATA = 0x82
};

// 프로토콜 구조: [STX][LEN][CMD][DATA...][ETX][CHECKSUM]
const uint8_t STX = 0x02;  // Start of Text
const uint8_t ETX = 0x03;  // End of Text

// 수신 버퍼
uint8_t rxBuffer[64];
int rxIndex = 0;
bool packetReady = false;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Protocol Parser Ready");
  Serial.println("Format: [STX][LEN][CMD][DATA...][ETX][CHECKSUM]");
}

void loop() {
  // 시리얼 데이터 수신 및 파싱
  receivePacket();
  
  // 완성된 패킷 처리
  if (packetReady) {
    processPacket();
    packetReady = false;
    rxIndex = 0;
  }
}

void receivePacket() {
  while (Serial.available() > 0 && !packetReady) {
    uint8_t receivedByte = Serial.read();
    
    if (rxIndex == 0 && receivedByte == STX) {
      // 패킷 시작
      rxBuffer[rxIndex++] = receivedByte;
      
    } else if (rxIndex > 0) {
      rxBuffer[rxIndex++] = receivedByte;
      
      // 패킷 길이 확인 (최소 길이: STX + LEN + CMD + ETX + CHECKSUM = 5)
      if (rxIndex >= 2) {
        uint8_t packetLength = rxBuffer[1];
        
        if (rxIndex >= packetLength + 3) {  // STX + LEN + DATA + ETX + CHECKSUM
          // 패킷 완성 확인
          if (rxBuffer[packetLength + 1] == ETX) {
            packetReady = true;
          } else {
            // 잘못된 패킷, 리셋
            rxIndex = 0;
          }
        }
      }
      
      // 버퍼 오버플로우 방지
      if (rxIndex >= sizeof(rxBuffer)) {
        rxIndex = 0;
      }
    }
  }
}

void processPacket() {
  // 체크섬 검증
  uint8_t packetLength = rxBuffer[1];
  uint8_t receivedChecksum = rxBuffer[packetLength + 2];
  uint8_t calculatedChecksum = 0;
  
  for (int i = 0; i < packetLength + 2; i++) {
    calculatedChecksum ^= rxBuffer[i];
  }
  
  if (receivedChecksum != calculatedChecksum) {
    sendResponse(RESP_ERROR, NULL, 0);
    return;
  }
  
  // 명령어 처리
  uint8_t command = rxBuffer[2];
  uint8_t* data = &rxBuffer[3];
  uint8_t dataLength = packetLength - 2;  // CMD와 ETX 제외
  
  switch (command) {
    case CMD_SET_LED:
      handleSetLED(data, dataLength);
      break;
      
    case CMD_GET_SENSOR:
      handleGetSensor(data, dataLength);
      break;
      
    case CMD_SET_PWM:
      handleSetPWM(data, dataLength);
      break;
      
    case CMD_RESET:
      handleReset();
      break;
      
    default:
      sendResponse(RESP_ERROR, NULL, 0);
      break;
  }
}

void handleSetLED(uint8_t* data, uint8_t length) {
  if (length >= 1) {
    digitalWrite(LED_BUILTIN, data[0] ? HIGH : LOW);
    sendResponse(RESP_OK, NULL, 0);
  } else {
    sendResponse(RESP_ERROR, NULL, 0);
  }
}

void handleGetSensor(uint8_t* data, uint8_t length) {
  uint8_t responseData[6];
  
  // 온도 데이터 (2바이트)
  int16_t temp = (int16_t)(readTemperature() * 100);
  responseData[0] = (temp >> 8) & 0xFF;
  responseData[1] = temp & 0xFF;
  
  // 조도 데이터 (2바이트)
  uint16_t light = analogRead(A1);
  responseData[2] = (light >> 8) & 0xFF;
  responseData[3] = light & 0xFF;
  
  // 소음 데이터 (2바이트)
  uint16_t sound = analogRead(A2);
  responseData[4] = (sound >> 8) & 0xFF;
  responseData[5] = sound & 0xFF;
  
  sendResponse(RESP_DATA, responseData, 6);
}

void handleSetPWM(uint8_t* data, uint8_t length) {
  if (length >= 2) {
    uint8_t pin = data[0];
    uint8_t value = data[1];
    
    if (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11) {
      analogWrite(pin, value);
      sendResponse(RESP_OK, NULL, 0);
    } else {
      sendResponse(RESP_ERROR, NULL, 0);
    }
  } else {
    sendResponse(RESP_ERROR, NULL, 0);
  }
}

void handleReset() {
  sendResponse(RESP_OK, NULL, 0);
  delay(100);
  
  // 소프트웨어 리셋 (watchdog 사용)
  asm volatile ("  jmp 0");
}

void sendResponse(uint8_t responseType, uint8_t* data, uint8_t dataLength) {
  uint8_t packetLength = dataLength + 1;  // ResponseType + Data
  
  Serial.write(STX);
  Serial.write(packetLength);
  Serial.write(responseType);
  
  if (data && dataLength > 0) {
    Serial.write(data, dataLength);
  }
  
  Serial.write(ETX);
  
  // 체크섬 계산 및 전송
  uint8_t checksum = STX ^ packetLength ^ responseType;
  for (int i = 0; i < dataLength; i++) {
    checksum ^= data[i];
  }
  checksum ^= ETX;
  
  Serial.write(checksum);
}

float readTemperature() {
  int sensorValue = analogRead(A0);
  float voltage = (sensorValue / 1023.0) * 5.0;
  return voltage * 100.0;
}
```

## 🔧 5. 다중 시리얼 포트 (Arduino Mega)

### 5.1 하드웨어 시리얼 포트 활용
```cpp
/*
  Multiple_Serial.ino
  Arduino Mega의 다중 시리얼 포트 활용
  
  Mega 2560 시리얼 포트:
  - Serial: USB (핀 0, 1)
  - Serial1: 핀 19(RX), 18(TX)
  - Serial2: 핀 17(RX), 16(TX)  
  - Serial3: 핀 15(RX), 14(TX)
*/

void setup() {
  // 모든 시리얼 포트 초기화
  Serial.begin(9600);    // USB 시리얼 (디버그용)
  Serial1.begin(9600);   // GPS 모듈용
  Serial2.begin(9600);   // WiFi 모듈용
  Serial3.begin(9600);   // 블루투스 모듈용
  
  Serial.println("Multiple Serial Ports Ready");
  Serial.println("Serial1: GPS, Serial2: WiFi, Serial3: Bluetooth");
}

void loop() {
  // USB 시리얼에서 명령 받기
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command.startsWith("gps:")) {
      // GPS 모듈에 명령 전송
      String gpsCommand = command.substring(4);
      Serial1.println(gpsCommand);
      Serial.println("GPS에 전송: " + gpsCommand);
      
    } else if (command.startsWith("wifi:")) {
      // WiFi 모듈에 명령 전송
      String wifiCommand = command.substring(5);
      Serial2.println(wifiCommand);
      Serial.println("WiFi에 전송: " + wifiCommand);
      
    } else if (command.startsWith("bt:")) {
      // 블루투스 모듈에 명령 전송
      String btCommand = command.substring(3);
      Serial3.println(btCommand);
      Serial.println("Bluetooth에 전송: " + btCommand);
    }
  }
  
  // 각 시리얼 포트에서 데이터 받기
  handleSerialData();
}

void handleSerialData() {
  // GPS 데이터 처리
  if (Serial1.available()) {
    String gpsData = Serial1.readString();
    Serial.print("GPS: ");
    Serial.println(gpsData);
  }
  
  // WiFi 데이터 처리
  if (Serial2.available()) {
    String wifiData = Serial2.readString();
    Serial.print("WiFi: ");
    Serial.println(wifiData);
  }
  
  // 블루투스 데이터 처리
  if (Serial3.available()) {
    String btData = Serial3.readString();
    Serial.print("Bluetooth: ");
    Serial.println(btData);
  }
}
```

## 📱 6. 블루투스 시리얼 통신

### 6.1 HC-06 블루투스 모듈 제어
```cpp
/*
  Bluetooth_Control.ino
  HC-06 블루투스 모듈을 통한 무선 제어
  
  연결:
  HC-06 VCC → 5V
  HC-06 GND → GND
  HC-06 RXD → Pin 2
  HC-06 TXD → Pin 3
*/

#include <SoftwareSerial.h>

// 소프트웨어 시리얼 설정
SoftwareSerial bluetooth(2, 3);  // RX, TX

// 제어할 핀들
const int LED_PINS[] = {9, 10, 11, 12, 13};
const int NUM_LEDS = 5;
const int SERVO_PIN = 6;
const int BUZZER_PIN = 8;

// 현재 상태
bool ledStates[NUM_LEDS] = {false};
int servoAngle = 90;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  // 핀 설정
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  Serial.println("Bluetooth Control System Ready");
  bluetooth.println("Arduino Connected!");
  
  // 초기 상태 전송
  sendStatus();
}

void loop() {
  // 블루투스에서 명령 받기
  if (bluetooth.available()) {
    String command = bluetooth.readString();
    command.trim();
    processBluetoothCommand(command);
  }
  
  // USB 시리얼에서 디버그 명령 받기
  if (Serial.available()) {
    String debugCommand = Serial.readString();
    debugCommand.trim();
    
    if (debugCommand == "status") {
      sendStatus();
    } else {
      processBluetoothCommand(debugCommand);
    }
  }
}

void processBluetoothCommand(String command) {
  command.toLowerCase();
  
  Serial.println("명령 수신: " + command);
  
  if (command.startsWith("led")) {
    handleLEDCommand(command);
    
  } else if (command.startsWith("servo")) {
    handleServoCommand(command);
    
  } else if (command.startsWith("buzz")) {
    handleBuzzerCommand(command);
    
  } else if (command == "all on") {
    handleAllOn();
    
  } else if (command == "all off") {
    handleAllOff();
    
  } else if (command == "status") {
    sendStatus();
    
  } else if (command == "help") {
    sendHelp();
    
  } else {
    bluetooth.println("Unknown command: " + command);
    bluetooth.println("Type 'help' for available commands");
  }
}

void handleLEDCommand(String command) {
  // "led 1 on", "led 3 off" 형식 파싱
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  
  if (firstSpace != -1 && secondSpace != -1) {
    int ledNum = command.substring(firstSpace + 1, secondSpace).toInt();
    String action = command.substring(secondSpace + 1);
    
    if (ledNum >= 1 && ledNum <= NUM_LEDS) {
      int ledIndex = ledNum - 1;
      
      if (action == "on") {
        digitalWrite(LED_PINS[ledIndex], HIGH);
        ledStates[ledIndex] = true;
        bluetooth.println("LED " + String(ledNum) + " ON");
        
      } else if (action == "off") {
        digitalWrite(LED_PINS[ledIndex], LOW);
        ledStates[ledIndex] = false;
        bluetooth.println("LED " + String(ledNum) + " OFF");
        
      } else {
        bluetooth.println("Invalid action. Use 'on' or 'off'");
      }
    } else {
      bluetooth.println("Invalid LED number. Use 1-" + String(NUM_LEDS));
    }
  } else {
    bluetooth.println("Format: led <number> <on/off>");
  }
}

void handleServoCommand(String command) {
  // "servo 90" 형식 파싱
  int spaceIndex = command.indexOf(' ');
  
  if (spaceIndex != -1) {
    int angle = command.substring(spaceIndex + 1).toInt();
    
    if (angle >= 0 && angle <= 180) {
      servoAngle = angle;
      // 간단한 서보 제어 (라이브러리 없이)
      int pulseWidth = map(angle, 0, 180, 1000, 2000);
      
      for (int i = 0; i < 20; i++) {  // 20번 반복으로 안정화
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulseWidth);
        digitalWrite(SERVO_PIN, LOW);
        delay(18);
      }
      
      bluetooth.println("Servo angle set to " + String(angle) + "°");
    } else {
      bluetooth.println("Invalid angle. Use 0-180");
    }
  } else {
    bluetooth.println("Format: servo <angle>");
  }
}

void handleBuzzerCommand(String command) {
  // "buzz 1000 500" (주파수, 지속시간) 형식
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  
  if (firstSpace != -1) {
    int frequency = command.substring(firstSpace + 1, 
                    secondSpace != -1 ? secondSpace : command.length()).toInt();
    int duration = 200;  // 기본값
    
    if (secondSpace != -1) {
      duration = command.substring(secondSpace + 1).toInt();
    }
    
    tone(BUZZER_PIN, frequency, duration);
    bluetooth.println("Buzzer: " + String(frequency) + "Hz for " + String(duration) + "ms");
  } else {
    bluetooth.println("Format: buzz <frequency> [duration]");
  }
}

void handleAllOn() {
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(LED_PINS[i], HIGH);
    ledStates[i] = true;
  }
  bluetooth.println("All LEDs ON");
}

void handleAllOff() {
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(LED_PINS[i], LOW);
    ledStates[i] = false;
  }
  bluetooth.println("All LEDs OFF");
}

void sendStatus() {
  bluetooth.println("=== System Status ===");
  
  bluetooth.print("LEDs: ");
  for (int i = 0; i < NUM_LEDS; i++) {
    bluetooth.print("LED");
    bluetooth.print(i + 1);
    bluetooth.print(":");
    bluetooth.print(ledStates[i] ? "ON" : "OFF");
    if (i < NUM_LEDS - 1) bluetooth.print(", ");
  }
  bluetooth.println();
  
  bluetooth.println("Servo: " + String(servoAngle) + "°");
  bluetooth.println("Uptime: " + String(millis() / 1000) + "s");
  bluetooth.println("Free Memory: " + String(getFreeMemory()) + " bytes");
  bluetooth.println("==================");
}

void sendHelp() {
  bluetooth.println("=== Available Commands ===");
  bluetooth.println("led <1-5> <on/off> - Control LED");
  bluetooth.println("servo <0-180>      - Set servo angle");  
  bluetooth.println("buzz <freq> [dur]  - Play buzzer");
  bluetooth.println("all on             - Turn all LEDs on");
  bluetooth.println("all off            - Turn all LEDs off");
  bluetooth.println("status             - Show system status");
  bluetooth.println("help               - Show this help");
  bluetooth.println("========================");
}

int getFreeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
```

## ⚠️ 7. 시리얼 통신 문제 해결

### 7.1 일반적인 문제들
```cpp
// 문제 1: 보드레이트 불일치
// 해결: 송신과 수신 측 보드레이트를 동일하게 설정

// 문제 2: 데이터 손실
// 해결: 버퍼 크기 확인 및 흐름 제어
if (Serial.available() > 0) {
  while (Serial.available() > 0) {  // 모든 데이터 읽기
    char c = Serial.read();
    // 처리...
  }
}

// 문제 3: 문자열 끝 처리
String data = Serial.readString();
data.trim();  // 앞뒤 공백/개행 제거

// 문제 4: 타이밍 문제
Serial.flush();  // 전송 완료까지 대기
delay(10);       // 안정화 시간
```

### 7.2 디버깅 기법
```cpp
// 시리얼 데이터 헥스 덤프
void hexDump(uint8_t* data, int length) {
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
    if ((i + 1) % 16 == 0) Serial.println();
  }
  Serial.println();
}

// 통신 통계
struct CommStats {
  unsigned long bytesReceived;
  unsigned long bytesSent;
  unsigned long packetsReceived;
  unsigned long packetsSent;
  unsigned long errors;
};
```

## ✅ 핵심 정리

1. **시리얼 통신**은 아두이노의 가장 기본적이고 중요한 통신 방식입니다.
2. **보드레이트 설정**이 송신과 수신 측에서 일치해야 합니다.
3. **데이터 형식**을 정의하면 효율적이고 안정적인 통신이 가능합니다.
4. **에러 처리**와 **체크섬**을 통해 통신 신뢰성을 높일 수 있습니다.

## 🚀 다음 단계

다음 레슨에서는 인터럽트와 타이머를 배워서 
더 정확한 타이밍 제어와 효율적인 프로그램 구조를 
구현하는 방법을 알아보겠습니다.

## 💻 실습 과제

### 과제 1: 기본 통신
1. 간단한 채팅 프로그램 만들기
2. 센서 데이터를 CSV 파일로 저장할 수 있는 로거 구현
3. 원격 LED 제어 시스템 구축

### 과제 2: 프로토콜 설계
1. 사용자 정의 프로토콜로 다중 기기 제어
2. 이진 데이터 전송으로 효율성 개선
3. 에러 복구 기능이 있는 통신 시스템

### 과제 3: 무선 통신 응용
1. 블루투스로 모바일 앱과 연동
2. WiFi를 이용한 웹 기반 제어 인터페이스
3. 다중 아두이노 간 데이터 중계 시스템

---

[← 이전 레슨: 아날로그 입출력과 PWM](../05-analog-io-pwm/README.md) | [다음 레슨: 인터럽트와 타이머 →](../07-interrupts-timers/README.md)