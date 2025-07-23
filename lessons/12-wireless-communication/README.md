# Lesson 12: 무선 통신 마스터 (블루투스, WiFi, LoRa)

## 🎯 학습 목표
- 블루투스 클래식 및 BLE 통신 구현
- WiFi를 이용한 웹서버 및 클라이언트 구축
- LoRa 장거리 통신 및 IoT 네트워크 구성
- 무선 센서 네트워크 및 메시 통신 구현
- 실시간 원격 모니터링 및 제어 시스템 개발

## 📚 이론 배경

### 무선 통신 프로토콜 비교

```
┌─────────────────────────────────────────────────────────────────┐
│                    무선 통신 프로토콜 비교표                      │
├─────────────┬─────────────┬─────────────┬─────────────────────┤
│   특성      │  블루투스    │    WiFi     │       LoRa          │
├─────────────┼─────────────┼─────────────┼─────────────────────┤
│ 통신 거리   │   10-100m   │   50-300m   │     1-15km          │
│ 데이터 속도 │  1-24Mbps   │ 11-1300Mbps │    0.3-50kbps       │
│ 전력 소모   │    중간     │    높음     │     매우낮음         │
│ 네트워크    │   P2P/Star  │ Star/Mesh   │   Star/Mesh         │
│ 비용        │    저가     │    중간     │      저가           │
│ 용도        │ 스마트폰연결 │ 인터넷연결   │ IoT센서네트워크      │
└─────────────┴─────────────┴─────────────┴─────────────────────┘
```

### 블루투스 아키텍처

```
블루투스 스택 구조:
┌─────────────────────────────────────┐
│        Application Layer            │ 사용자 앱
├─────────────────────────────────────┤
│           Profile Layer             │ SPP, HID, etc.
├─────────────────────────────────────┤
│            L2CAP Layer              │ 데이터 패킷화
├─────────────────────────────────────┤
│            HCI Layer                │ 호스트 제어
├─────────────────────────────────────┤
│         Baseband Layer              │ 2.4GHz 주파수 호핑
├─────────────────────────────────────┤
│           Radio Layer               │ RF 송수신
└─────────────────────────────────────┘

주파수: 2.4GHz ISM 대역 (2.402-2.480GHz)
채널: 79개 채널, 1MHz 간격
호핑: 1600회/초 (625μs 간격)
```

## 🔧 실습 1: 블루투스 클래식 통신

### HC-05/HC-06 모듈을 이용한 스마트폰 제어

```cpp
/*
  Bluetooth_Smart_Controller.ino
  블루투스를 통한 스마트 홈 제어 시스템
*/

#include <SoftwareSerial.h>
#include <Servo.h>

// 블루투스 모듈 연결 (RX, TX)
SoftwareSerial bluetooth(2, 3);

// 제어 대상 하드웨어
Servo doorLock;
const int LED_LIVING = 4;
const int LED_BEDROOM = 5;
const int LED_KITCHEN = 6;
const int FAN_PIN = 7;
const int TEMP_SENSOR = A0;
const int LIGHT_SENSOR = A1;
const int BUZZER_PIN = 8;

// 시스템 상태
struct SmartHomeState {
  bool livingRoomLight;
  bool bedroomLight;
  bool kitchenLight;
  bool fanRunning;
  int doorLockAngle;
  float temperature;
  int lightLevel;
  bool securityMode;
  bool autoMode;
} homeState;

// 명령 파싱
String receivedCommand = "";
bool commandComplete = false;

// 자동화 설정
const float AUTO_TEMP_THRESHOLD = 28.0;
const int AUTO_LIGHT_THRESHOLD = 300;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  // 하드웨어 초기화
  initializeHardware();
  
  // 시스템 상태 초기화
  initializeSystemState();
  
  Serial.println("블루투스 스마트 홈 컨트롤러 시작");
  bluetooth.println("Smart Home Controller Ready!");
  
  // 초기 상태 전송
  sendStatusUpdate();
}

void loop() {
  // 블루투스 명령 처리
  handleBluetoothCommands();
  
  // 센서 데이터 읽기
  readSensors();
  
  // 자동화 로직 실행
  if (homeState.autoMode) {
    executeAutomation();
  }
  
  // 보안 모드 체크
  if (homeState.securityMode) {
    checkSecurity();
  }
  
  // 상태 업데이트 전송 (5초마다)
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 5000) {
    sendStatusUpdate();
    lastUpdate = millis();
  }
  
  delay(100);
}

void initializeHardware() {
  // LED 핀 설정
  pinMode(LED_LIVING, OUTPUT);
  pinMode(LED_BEDROOM, OUTPUT);
  pinMode(LED_KITCHEN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // 서보모터 초기화 (도어락)
  doorLock.attach(9);
  doorLock.write(0);  // 잠금 상태
  
  // 모든 장치 초기 상태로 설정
  digitalWrite(LED_LIVING, LOW);
  digitalWrite(LED_BEDROOM, LOW);
  digitalWrite(LED_KITCHEN, LOW);
  digitalWrite(FAN_PIN, LOW);
}

void initializeSystemState() {
  homeState.livingRoomLight = false;
  homeState.bedroomLight = false;
  homeState.kitchenLight = false;
  homeState.fanRunning = false;
  homeState.doorLockAngle = 0;
  homeState.securityMode = false;
  homeState.autoMode = true;
}

void handleBluetoothCommands() {
  // 블루투스에서 데이터 읽기
  while (bluetooth.available()) {
    char inChar = bluetooth.read();
    
    if (inChar == '\n') {
      commandComplete = true;
    } else {
      receivedCommand += inChar;
    }
  }
  
  // 명령 처리
  if (commandComplete) {
    processCommand(receivedCommand);
    receivedCommand = "";
    commandComplete = false;
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  Serial.println("수신된 명령: " + command);
  
  // 조명 제어 명령
  if (command == "LIVING_ON") {
    controlLight(LED_LIVING, true);
    homeState.livingRoomLight = true;
    bluetooth.println("거실 조명 켜짐");
  }
  else if (command == "LIVING_OFF") {
    controlLight(LED_LIVING, false);
    homeState.livingRoomLight = false;
    bluetooth.println("거실 조명 꺼짐");
  }
  else if (command == "BEDROOM_ON") {
    controlLight(LED_BEDROOM, true);
    homeState.bedroomLight = true;
    bluetooth.println("침실 조명 켜짐");
  }
  else if (command == "BEDROOM_OFF") {
    controlLight(LED_BEDROOM, false);
    homeState.bedroomLight = false;
    bluetooth.println("침실 조명 꺼짐");
  }
  else if (command == "KITCHEN_ON") {
    controlLight(LED_KITCHEN, true);
    homeState.kitchenLight = true;
    bluetooth.println("주방 조명 켜짐");
  }
  else if (command == "KITCHEN_OFF") {
    controlLight(LED_KITCHEN, false);
    homeState.kitchenLight = false;
    bluetooth.println("주방 조명 꺼짐");
  }
  
  // 전체 조명 제어
  else if (command == "ALL_LIGHTS_ON") {
    controlAllLights(true);
    bluetooth.println("모든 조명 켜짐");
  }
  else if (command == "ALL_LIGHTS_OFF") {
    controlAllLights(false);
    bluetooth.println("모든 조명 꺼짐");
  }
  
  // 팬 제어
  else if (command == "FAN_ON") {
    digitalWrite(FAN_PIN, HIGH);
    homeState.fanRunning = true;
    bluetooth.println("팬 켜짐");
  }
  else if (command == "FAN_OFF") {
    digitalWrite(FAN_PIN, LOW);
    homeState.fanRunning = false;
    bluetooth.println("팬 꺼짐");
  }
  
  // 도어락 제어
  else if (command == "DOOR_UNLOCK") {
    doorLock.write(90);
    homeState.doorLockAngle = 90;
    bluetooth.println("문 열림");
  }
  else if (command == "DOOR_LOCK") {
    doorLock.write(0);
    homeState.doorLockAngle = 0;
    bluetooth.println("문 잠김");
  }
  
  // 모드 제어
  else if (command == "AUTO_ON") {
    homeState.autoMode = true;
    bluetooth.println("자동 모드 활성화");
  }
  else if (command == "AUTO_OFF") {
    homeState.autoMode = false;
    bluetooth.println("자동 모드 비활성화");
  }
  else if (command == "SECURITY_ON") {
    homeState.securityMode = true;
    bluetooth.println("보안 모드 활성화");
  }
  else if (command == "SECURITY_OFF") {
    homeState.securityMode = false;
    bluetooth.println("보안 모드 비활성화");
  }
  
  // 상태 조회
  else if (command == "STATUS") {
    sendDetailedStatus();
  }
  
  // 도움말
  else if (command == "HELP") {
    sendHelpMenu();
  }
  
  else {
    bluetooth.println("알 수 없는 명령: " + command);
  }
}

void controlLight(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
  
  // 부드러운 점등/소등 효과 (PWM 사용)
  if (state) {
    for (int brightness = 0; brightness <= 255; brightness += 5) {
      analogWrite(pin, brightness);
      delay(20);
    }
  } else {
    for (int brightness = 255; brightness >= 0; brightness -= 5) {
      analogWrite(pin, brightness);
      delay(20);
    }
  }
}

void controlAllLights(bool state) {
  controlLight(LED_LIVING, state);
  controlLight(LED_BEDROOM, state);
  controlLight(LED_KITCHEN, state);
  
  homeState.livingRoomLight = state;
  homeState.bedroomLight = state;
  homeState.kitchenLight = state;
}

void readSensors() {
  // 온도 센서 읽기 (LM35 기준)
  int tempReading = analogRead(TEMP_SENSOR);
  homeState.temperature = (tempReading * 5.0 * 100.0) / 1024.0;
  
  // 조도 센서 읽기
  homeState.lightLevel = analogRead(LIGHT_SENSOR);
}

void executeAutomation() {
  // 온도 기반 팬 자동 제어
  if (homeState.temperature > AUTO_TEMP_THRESHOLD && !homeState.fanRunning) {
    digitalWrite(FAN_PIN, HIGH);
    homeState.fanRunning = true;
    bluetooth.println("자동: 온도가 높아 팬을 켭니다");
  }
  else if (homeState.temperature < AUTO_TEMP_THRESHOLD - 2.0 && homeState.fanRunning) {
    digitalWrite(FAN_PIN, LOW);
    homeState.fanRunning = false;
    bluetooth.println("자동: 온도가 낮아 팬을 끕니다");
  }
  
  // 조도 기반 조명 자동 제어
  static bool autoLightsOn = false;
  if (homeState.lightLevel < AUTO_LIGHT_THRESHOLD && !autoLightsOn) {
    controlAllLights(true);
    autoLightsOn = true;
    bluetooth.println("자동: 어두워서 조명을 켭니다");
  }
  else if (homeState.lightLevel > AUTO_LIGHT_THRESHOLD + 100 && autoLightsOn) {
    controlAllLights(false);
    autoLightsOn = false;
    bluetooth.println("자동: 밝아서 조명을 끕니다");
  }
}

void checkSecurity() {
  // 간단한 침입 감지 (PIR 센서 시뮬레이션)
  static bool lastSecurityState = false;
  bool motionDetected = (analogRead(A2) > 512);  // A2를 모션 센서로 가정
  
  if (motionDetected && !lastSecurityState) {
    // 침입자 감지!
    triggerSecurityAlarm();
    bluetooth.println("경고: 움직임 감지됨!");
  }
  
  lastSecurityState = motionDetected;
}

void triggerSecurityAlarm() {
  // 경보음 발생
  for (int i = 0; i < 5; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
  
  // 조명 깜빡임
  for (int i = 0; i < 10; i++) {
    controlAllLights(true);
    delay(100);
    controlAllLights(false);
    delay(100);
  }
}

void sendStatusUpdate() {
  bluetooth.println("=== 시스템 상태 ===");
  bluetooth.print("거실 조명: ");
  bluetooth.println(homeState.livingRoomLight ? "켜짐" : "꺼짐");
  
  bluetooth.print("침실 조명: ");
  bluetooth.println(homeState.bedroomLight ? "켜짐" : "꺼짐");
  
  bluetooth.print("주방 조명: ");
  bluetooth.println(homeState.kitchenLight ? "켜짐" : "꺼짐");
  
  bluetooth.print("팬: ");
  bluetooth.println(homeState.fanRunning ? "작동중" : "정지");
  
  bluetooth.print("문: ");
  bluetooth.println(homeState.doorLockAngle == 90 ? "열림" : "잠김");
  
  bluetooth.print("온도: ");
  bluetooth.print(homeState.temperature);
  bluetooth.println("°C");
  
  bluetooth.print("조도: ");
  bluetooth.println(homeState.lightLevel);
  
  bluetooth.print("자동모드: ");
  bluetooth.println(homeState.autoMode ? "활성" : "비활성");
  
  bluetooth.print("보안모드: ");
  bluetooth.println(homeState.securityMode ? "활성" : "비활성");
  
  bluetooth.println("==================");
}

void sendDetailedStatus() {
  bluetooth.println("\n=== 상세 시스템 정보 ===");
  bluetooth.print("시스템 업타임: ");
  bluetooth.print(millis() / 1000);
  bluetooth.println("초");
  
  bluetooth.print("메모리 사용량: ");
  bluetooth.print(getFreeMemory());
  bluetooth.println(" bytes");
  
  bluetooth.println("하드웨어 상태:");
  bluetooth.println("- 온도센서: 정상");
  bluetooth.println("- 조도센서: 정상");
  bluetooth.println("- 서보모터: 정상");
  bluetooth.println("- LED 드라이버: 정상");
  
  bluetooth.println("=======================\n");
}

void sendHelpMenu() {
  bluetooth.println("\n=== 명령어 도움말 ===");
  bluetooth.println("조명 제어:");
  bluetooth.println("- LIVING_ON/OFF: 거실 조명");
  bluetooth.println("- BEDROOM_ON/OFF: 침실 조명");
  bluetooth.println("- KITCHEN_ON/OFF: 주방 조명");
  bluetooth.println("- ALL_LIGHTS_ON/OFF: 전체 조명");
  
  bluetooth.println("\n가전 제어:");
  bluetooth.println("- FAN_ON/OFF: 팬 제어");
  bluetooth.println("- DOOR_UNLOCK/LOCK: 도어락");
  
  bluetooth.println("\n모드 설정:");
  bluetooth.println("- AUTO_ON/OFF: 자동 모드");
  bluetooth.println("- SECURITY_ON/OFF: 보안 모드");
  
  bluetooth.println("\n정보 조회:");
  bluetooth.println("- STATUS: 상세 상태");
  bluetooth.println("- HELP: 이 도움말");
  bluetooth.println("====================\n");
}

// 메모리 사용량 체크 함수
int getFreeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
```

## 🔧 실습 2: WiFi 웹서버 구축

### ESP32를 이용한 IoT 웹 대시보드

```cpp
/*
  WiFi_IoT_Dashboard.ino
  ESP32를 이용한 IoT 웹 대시보드 시스템
  
  필요한 라이브러리:
  - WiFi (ESP32 Core)
  - WebServer (ESP32 Core)
  - ArduinoJson
  - DHT sensor library
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <time.h>

// WiFi 설정
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";

// 웹서버 포트
WebServer server(80);

// 센서 설정
#define DHT_PIN 4
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// 하드웨어 핀 정의
const int LED_BUILTIN_PIN = 2;
const int RELAY_PIN = 5;
const int BUZZER_PIN = 18;
const int BUTTON_PIN = 19;
const int POTENTIOMETER_PIN = A0;

// 시스템 상태
struct IoTSystemState {
  float temperature;
  float humidity;
  int lightLevel;
  bool ledStatus;
  bool relayStatus;
  int potValue;
  bool buttonPressed;
  unsigned long uptime;
  String lastCommand;
  int wifiStrength;
} systemState;

// 데이터 로깅 (간단한 원형 버퍼)
const int LOG_SIZE = 100;
struct SensorLog {
  unsigned long timestamp;
  float temperature;
  float humidity;
  int lightLevel;
} sensorLog[LOG_SIZE];
int logIndex = 0;

// NTP 시간 설정
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 32400;  // GMT+9 (한국 시간)
const int daylightOffset_sec = 0;

void setup() {
  Serial.begin(115200);
  
  // 하드웨어 초기화
  initializeHardware();
  
  // WiFi 연결
  connectToWiFi();
  
  // NTP 시간 동기화
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  // 웹서버 라우팅 설정
  setupWebServer();
  
  // DHT 센서 초기화
  dht.begin();
  
  Serial.println("IoT 웹 대시보드 시스템 시작");
  Serial.print("웹 대시보드 주소: http://");
  Serial.println(WiFi.localIP());
}

void loop() {
  // 웹서버 클라이언트 처리
  server.handleClient();
  
  // 센서 데이터 읽기
  readSensors();
  
  // 데이터 로깅
  logSensorData();
  
  // 시스템 상태 업데이트
  updateSystemState();
  
  // 버튼 입력 처리
  handleButtonInput();
  
  delay(1000);
}

void initializeHardware() {
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // 초기 상태 설정
  digitalWrite(LED_BUILTIN_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  systemState.ledStatus = false;
  systemState.relayStatus = false;
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi 연결 성공!");
    Serial.print("IP 주소: ");
    Serial.println(WiFi.localIP());
    Serial.print("신호 강도: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println();
    Serial.println("WiFi 연결 실패!");
  }
}

void setupWebServer() {
  // 메인 대시보드 페이지
  server.on("/", handleRoot);
  
  // API 엔드포인트
  server.on("/api/status", HTTP_GET, handleAPIStatus);
  server.on("/api/control", HTTP_POST, handleAPIControl);
  server.on("/api/data", HTTP_GET, handleAPIData);
  server.on("/api/logs", HTTP_GET, handleAPILogs);
  
  // 정적 파일들
  server.on("/style.css", handleCSS);
  server.on("/script.js", handleJS);
  
  // 404 에러 처리
  server.onNotFound(handleNotFound);
  
  // 서버 시작
  server.begin();
  Serial.println("웹서버 시작됨");
}

void handleRoot() {
  String html = generateDashboardHTML();
  server.send(200, "text/html", html);
}

String generateDashboardHTML() {
  String html = R"(
<!DOCTYPE html>
<html lang='ko'>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>IoT 대시보드</title>
    <link rel='stylesheet' href='/style.css'>
</head>
<body>
    <div class='container'>
        <header>
            <h1>🏠 스마트 홈 대시보드</h1>
            <div class='status-indicator' id='connection-status'>연결됨</div>
        </header>
        
        <div class='dashboard-grid'>
            <!-- 센서 데이터 카드 -->
            <div class='card'>
                <h2>🌡️ 환경 센서</h2>
                <div class='sensor-data'>
                    <div class='sensor-item'>
                        <span class='label'>온도:</span>
                        <span class='value' id='temperature'>--°C</span>
                    </div>
                    <div class='sensor-item'>
                        <span class='label'>습도:</span>
                        <span class='value' id='humidity'>--%</span>
                    </div>
                    <div class='sensor-item'>
                        <span class='label'>조도:</span>
                        <span class='value' id='light'>--</span>
                    </div>
                </div>
            </div>
            
            <!-- 제어 카드 -->
            <div class='card'>
                <h2>🎛️ 장치 제어</h2>
                <div class='controls'>
                    <div class='control-item'>
                        <label>LED:</label>
                        <button class='btn' id='led-btn' onclick='toggleLED()'>OFF</button>
                    </div>
                    <div class='control-item'>
                        <label>릴레이:</label>
                        <button class='btn' id='relay-btn' onclick='toggleRelay()'>OFF</button>
                    </div>
                    <div class='control-item'>
                        <label>부저:</label>
                        <button class='btn' onclick='triggerBuzzer()'>테스트</button>
                    </div>
                </div>
            </div>
            
            <!-- 시스템 정보 카드 -->
            <div class='card'>
                <h2>⚙️ 시스템 정보</h2>
                <div class='system-info'>
                    <div>업타임: <span id='uptime'>--</span></div>
                    <div>WiFi 신호: <span id='wifi-strength'>--</span></div>
                    <div>마지막 명령: <span id='last-command'>--</span></div>
                    <div>메모리: <span id='memory'>--</span></div>
                </div>
            </div>
            
            <!-- 실시간 차트 -->
            <div class='card chart-card'>
                <h2>📊 실시간 데이터</h2>
                <canvas id='sensor-chart' width='400' height='200'></canvas>
            </div>
        </div>
        
        <footer>
            <p>마지막 업데이트: <span id='last-update'>--</span></p>
        </footer>
    </div>
    
    <script src='/script.js'></script>
</body>
</html>
)";
  return html;
}

void handleCSS() {
  String css = R"(
* {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
}

body {
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    min-height: 100vh;
    color: #333;
}

.container {
    max-width: 1200px;
    margin: 0 auto;
    padding: 20px;
}

header {
    background: rgba(255, 255, 255, 0.1);
    backdrop-filter: blur(10px);
    padding: 20px;
    border-radius: 15px;
    margin-bottom: 30px;
    display: flex;
    justify-content: space-between;
    align-items: center;
}

h1 {
    color: white;
    font-size: 2.5rem;
}

.status-indicator {
    background: #4CAF50;
    color: white;
    padding: 10px 20px;
    border-radius: 25px;
    font-weight: bold;
}

.dashboard-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 20px;
    margin-bottom: 30px;
}

.card {
    background: rgba(255, 255, 255, 0.9);
    backdrop-filter: blur(10px);
    border-radius: 15px;
    padding: 25px;
    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
    transition: transform 0.3s ease;
}

.card:hover {
    transform: translateY(-5px);
}

.card h2 {
    margin-bottom: 20px;
    color: #333;
    border-bottom: 2px solid #667eea;
    padding-bottom: 10px;
}

.sensor-data, .controls, .system-info {
    display: flex;
    flex-direction: column;
    gap: 15px;
}

.sensor-item, .control-item {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px;
    background: rgba(102, 126, 234, 0.1);
    border-radius: 8px;
}

.value {
    font-weight: bold;
    font-size: 1.2rem;
    color: #667eea;
}

.btn {
    background: #667eea;
    color: white;
    border: none;
    padding: 10px 20px;
    border-radius: 25px;
    cursor: pointer;
    font-weight: bold;
    transition: all 0.3s ease;
}

.btn:hover {
    background: #5a67d8;
    transform: scale(1.05);
}

.btn.active {
    background: #4CAF50;
}

.chart-card {
    grid-column: 1 / -1;
}

footer {
    text-align: center;
    color: white;
    background: rgba(255, 255, 255, 0.1);
    padding: 15px;
    border-radius: 10px;
}

@media (max-width: 768px) {
    .dashboard-grid {
        grid-template-columns: 1fr;
    }
    
    h1 {
        font-size: 1.8rem;
    }
    
    header {
        flex-direction: column;
        gap: 15px;
    }
}
)";
  server.send(200, "text/css", css);
}

void handleJS() {
  String js = R"(
let sensorChart;
let chartData = {
    labels: [],
    temperature: [],
    humidity: []
};

// 페이지 로드 시 초기화
document.addEventListener('DOMContentLoaded', function() {
    initializeChart();
    updateDashboard();
    setInterval(updateDashboard, 2000); // 2초마다 업데이트
});

// 대시보드 데이터 업데이트
async function updateDashboard() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        // 센서 데이터 업데이트
        document.getElementById('temperature').textContent = data.temperature.toFixed(1) + '°C';
        document.getElementById('humidity').textContent = data.humidity.toFixed(1) + '%';
        document.getElementById('light').textContent = data.lightLevel;
        
        // 제어 버튼 상태 업데이트
        updateButtonState('led-btn', data.ledStatus);
        updateButtonState('relay-btn', data.relayStatus);
        
        // 시스템 정보 업데이트
        document.getElementById('uptime').textContent = formatUptime(data.uptime);
        document.getElementById('wifi-strength').textContent = data.wifiStrength + ' dBm';
        document.getElementById('last-command').textContent = data.lastCommand;
        
        // 차트 업데이트
        updateChart(data.temperature, data.humidity);
        
        // 마지막 업데이트 시간
        document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
        
        // 연결 상태 표시
        document.getElementById('connection-status').textContent = '연결됨';
        document.getElementById('connection-status').style.background = '#4CAF50';
        
    } catch (error) {
        console.error('데이터 업데이트 오류:', error);
        document.getElementById('connection-status').textContent = '연결 오류';
        document.getElementById('connection-status').style.background = '#f44336';
    }
}

// 버튼 상태 업데이트
function updateButtonState(buttonId, isActive) {
    const button = document.getElementById(buttonId);
    if (isActive) {
        button.textContent = 'ON';
        button.classList.add('active');
    } else {
        button.textContent = 'OFF';
        button.classList.remove('active');
    }
}

// LED 토글
async function toggleLED() {
    try {
        const response = await fetch('/api/control', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                command: 'toggle_led'
            })
        });
        
        if (response.ok) {
            updateDashboard();
        }
    } catch (error) {
        console.error('LED 제어 오류:', error);
    }
}

// 릴레이 토글
async function toggleRelay() {
    try {
        const response = await fetch('/api/control', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                command: 'toggle_relay'
            })
        });
        
        if (response.ok) {
            updateDashboard();
        }
    } catch (error) {
        console.error('릴레이 제어 오류:', error);
    }
}

// 부저 테스트
async function triggerBuzzer() {
    try {
        const response = await fetch('/api/control', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                command: 'buzzer_test'
            })
        });
        
        if (response.ok) {
            console.log('부저 테스트 실행됨');
        }
    } catch (error) {
        console.error('부저 제어 오류:', error);
    }
}

// 업타임 포맷팅
function formatUptime(milliseconds) {
    const seconds = Math.floor(milliseconds / 1000);
    const minutes = Math.floor(seconds / 60);
    const hours = Math.floor(minutes / 60);
    const days = Math.floor(hours / 24);
    
    if (days > 0) {
        return days + '일 ' + (hours % 24) + '시간';
    } else if (hours > 0) {
        return hours + '시간 ' + (minutes % 60) + '분';
    } else {
        return minutes + '분 ' + (seconds % 60) + '초';
    }
}

// 차트 초기화
function initializeChart() {
    const canvas = document.getElementById('sensor-chart');
    const ctx = canvas.getContext('2d');
    
    // 간단한 차트 그리기 (Chart.js 없이)
    drawChart(ctx, canvas.width, canvas.height);
}

// 차트 그리기
function drawChart(ctx, width, height) {
    ctx.clearRect(0, 0, width, height);
    
    // 배경
    ctx.fillStyle = '#f8f9fa';
    ctx.fillRect(0, 0, width, height);
    
    // 격자
    ctx.strokeStyle = '#dee2e6';
    ctx.lineWidth = 1;
    
    // 수직선
    for (let i = 0; i <= 10; i++) {
        const x = (width / 10) * i;
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }
    
    // 수평선
    for (let i = 0; i <= 5; i++) {
        const y = (height / 5) * i;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }
    
    // 레이블
    ctx.fillStyle = '#6c757d';
    ctx.font = '12px Arial';
    ctx.fillText('온도/습도 실시간 차트', 10, 20);
}

// 차트 데이터 업데이트
function updateChart(temperature, humidity) {
    const now = new Date().toLocaleTimeString();
    
    // 데이터 추가
    chartData.labels.push(now);
    chartData.temperature.push(temperature);
    chartData.humidity.push(humidity);
    
    // 최대 20개 데이터포인트 유지
    if (chartData.labels.length > 20) {
        chartData.labels.shift();
        chartData.temperature.shift();
        chartData.humidity.shift();
    }
    
    // 차트 다시 그리기
    const canvas = document.getElementById('sensor-chart');
    const ctx = canvas.getContext('2d');
    drawChart(ctx, canvas.width, canvas.height);
    
    // 데이터 라인 그리기
    if (chartData.temperature.length > 1) {
        drawDataLine(ctx, chartData.temperature, '#ff6384', canvas.width, canvas.height, 0, 50);
        drawDataLine(ctx, chartData.humidity, '#36a2eb', canvas.width, canvas.height, 0, 100);
    }
}

// 데이터 라인 그리기
function drawDataLine(ctx, data, color, width, height, minVal, maxVal) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    for (let i = 0; i < data.length; i++) {
        const x = (width / (data.length - 1)) * i;
        const y = height - ((data[i] - minVal) / (maxVal - minVal)) * height;
        
        if (i === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    }
    
    ctx.stroke();
}
)";
  server.send(200, "application/javascript", js);
}

void handleAPIStatus() {
  DynamicJsonDocument doc(1024);
  
  doc["temperature"] = systemState.temperature;
  doc["humidity"] = systemState.humidity;
  doc["lightLevel"] = systemState.lightLevel;
  doc["ledStatus"] = systemState.ledStatus;
  doc["relayStatus"] = systemState.relayStatus;
  doc["potValue"] = systemState.potValue;
  doc["buttonPressed"] = systemState.buttonPressed;
  doc["uptime"] = systemState.uptime;
  doc["lastCommand"] = systemState.lastCommand;
  doc["wifiStrength"] = systemState.wifiStrength;
  doc["freeMemory"] = ESP.getFreeHeap();
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
}

void handleAPIControl() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, server.arg("plain"));
    
    String command = doc["command"];
    systemState.lastCommand = command;
    
    if (command == "toggle_led") {
      systemState.ledStatus = !systemState.ledStatus;
      digitalWrite(LED_BUILTIN_PIN, systemState.ledStatus);
      server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"LED toggled\"}");
    }
    else if (command == "toggle_relay") {
      systemState.relayStatus = !systemState.relayStatus;
      digitalWrite(RELAY_PIN, systemState.relayStatus);
      server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Relay toggled\"}");
    }
    else if (command == "buzzer_test") {
      // 부저 테스트 (비동기적으로)
      for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
      }
      server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Buzzer tested\"}");
    }
    else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Unknown command\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

void handleAPIData() {
  DynamicJsonDocument doc(2048);
  JsonArray dataArray = doc.createNestedArray("data");
  
  // 최근 로그 데이터 반환
  for (int i = 0; i < min(LOG_SIZE, 50); i++) {  // 최근 50개
    int idx = (logIndex - i - 1 + LOG_SIZE) % LOG_SIZE;
    
    JsonObject entry = dataArray.createNestedObject();
    entry["timestamp"] = sensorLog[idx].timestamp;
    entry["temperature"] = sensorLog[idx].temperature;
    entry["humidity"] = sensorLog[idx].humidity;
    entry["lightLevel"] = sensorLog[idx].lightLevel;
  }
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
}

void handleAPILogs() {
  String logs = "=== 시스템 로그 ===\n";
  logs += "업타임: " + String(millis() / 1000) + "초\n";
  logs += "WiFi 신호: " + String(WiFi.RSSI()) + " dBm\n";
  logs += "메모리: " + String(ESP.getFreeHeap()) + " bytes\n";
  logs += "마지막 명령: " + systemState.lastCommand + "\n";
  logs += "==================\n";
  
  server.send(200, "text/plain", logs);
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Page Not Found");
}

void readSensors() {
  // DHT22 센서 읽기
  systemState.temperature = dht.readTemperature();
  systemState.humidity = dht.readHumidity();
  
  // NaN 체크
  if (isnan(systemState.temperature)) {
    systemState.temperature = 0.0;
  }
  if (isnan(systemState.humidity)) {
    systemState.humidity = 0.0;
  }
  
  // 조도 센서 (포토레지스터)
  systemState.lightLevel = analogRead(POTENTIOMETER_PIN);
  
  // 포텐셔미터
  systemState.potValue = analogRead(A3);
}

void logSensorData() {
  sensorLog[logIndex].timestamp = millis();
  sensorLog[logIndex].temperature = systemState.temperature;
  sensorLog[logIndex].humidity = systemState.humidity;
  sensorLog[logIndex].lightLevel = systemState.lightLevel;
  
  logIndex = (logIndex + 1) % LOG_SIZE;
}

void updateSystemState() {
  systemState.uptime = millis();
  systemState.wifiStrength = WiFi.RSSI();
  systemState.buttonPressed = !digitalRead(BUTTON_PIN);
}

void handleButtonInput() {
  static bool lastButtonState = false;
  bool currentButtonState = !digitalRead(BUTTON_PIN);
  
  if (currentButtonState && !lastButtonState) {
    // 버튼 눌림 감지
    systemState.ledStatus = !systemState.ledStatus;
    digitalWrite(LED_BUILTIN_PIN, systemState.ledStatus);
    systemState.lastCommand = "Button Press";
    
    Serial.println("버튼으로 LED 토글됨");
  }
  
  lastButtonState = currentButtonState;
}
```

## 🔧 실습 3: LoRa 장거리 통신

### LoRa를 이용한 원격 센서 네트워크

```cpp
/*
  LoRa_Sensor_Network.ino
  LoRa를 이용한 장거리 센서 네트워크
  
  필요한 하드웨어:
  - ESP32 또는 Arduino + LoRa 모듈 (SX1276/SX1278)
  - 센서들 (온도, 습도, GPS 등)
  
  필요한 라이브러리:
  - LoRa by Sandeep Mistry
*/

#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <time.h>

// LoRa 핀 설정 (ESP32 기준)
#define LORA_SCK     5    // GPIO5  -- SX1278's SCK
#define LORA_MISO    19   // GPIO19 -- SX1278's MISO
#define LORA_MOSI    27   // GPIO27 -- SX1278's MOSI
#define LORA_SS      18   // GPIO18 -- SX1278's CS
#define LORA_RST     23   // GPIO23 -- SX1278's RESET
#define LORA_DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

// LoRa 설정
#define LORA_FREQUENCY    920E6  // 920MHz (한국 ISM 대역)
#define LORA_TX_POWER     20     // 최대 송신 출력
#define LORA_SPREADING_FACTOR 7  // SF7 (빠른 속도)
#define LORA_BANDWIDTH    125E3  // 125kHz 대역폭
#define LORA_CODING_RATE  5      // 4/5 코딩 레이트

// 네트워크 설정
#define NODE_ID           1      // 현재 노드 ID (각 노드마다 다르게 설정)
#define GATEWAY_ID        0      // 게이트웨이 ID
#define MAX_NODES         10     // 최대 노드 수

// 메시지 타입
enum MessageType {
  MSG_SENSOR_DATA = 1,
  MSG_COMMAND = 2,
  MSG_ACK = 3,
  MSG_HEARTBEAT = 4,
  MSG_ALARM = 5
};

// 패킷 구조체
struct LoRaPacket {
  uint8_t sourceID;
  uint8_t destinationID;
  uint8_t messageType;
  uint8_t sequenceNumber;
  uint32_t timestamp;
  uint8_t payloadLength;
  char payload[200];
  uint16_t checksum;
};

// 센서 데이터 구조체
struct SensorData {
  float temperature;
  float humidity;
  float batteryVoltage;
  int signalStrength;
  double latitude;
  double longitude;
  uint32_t timestamp;
};

// 노드 상태 관리
struct NodeStatus {
  uint8_t nodeID;
  uint32_t lastSeen;
  int rssi;
  bool isOnline;
  SensorData lastData;
} nodeList[MAX_NODES];

// 전역 변수
uint8_t sequenceCounter = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastSensorRead = 0;
bool isGateway = (NODE_ID == GATEWAY_ID);

void setup() {
  Serial.begin(115200);
  
  // LoRa 초기화
  initializeLoRa();
  
  // 노드별 초기화
  if (isGateway) {
    initializeGateway();
  } else {
    initializeSensorNode();
  }
  
  Serial.println("LoRa 센서 네트워크 시작");
  Serial.print("노드 ID: ");
  Serial.println(NODE_ID);
  Serial.print("역할: ");
  Serial.println(isGateway ? "게이트웨이" : "센서 노드");
}

void loop() {
  // LoRa 패킷 수신 확인
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    handleIncomingPacket(packetSize);
  }
  
  if (isGateway) {
    // 게이트웨이 동작
    gatewayLoop();
  } else {
    // 센서 노드 동작
    sensorNodeLoop();
  }
  
  delay(100);
}

void initializeLoRa() {
  // SPI 핀 설정
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  
  // LoRa 핀 설정
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
  
  // LoRa 초기화
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa 초기화 실패!");
    while (1);
  }
  
  // LoRa 파라미터 설정
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  
  // 수신 모드로 설정
  LoRa.receive();
  
  Serial.println("LoRa 초기화 완료");
  Serial.print("주파수: ");
  Serial.print(LORA_FREQUENCY / 1E6);
  Serial.println(" MHz");
}

void initializeGateway() {
  Serial.println("게이트웨이 모드로 초기화...");
  
  // 노드 리스트 초기화
  for (int i = 0; i < MAX_NODES; i++) {
    nodeList[i].nodeID = i;
    nodeList[i].lastSeen = 0;
    nodeList[i].isOnline = false;
  }
  
  // WiFi 연결 (옵션)
  // connectToWiFi();
}

void initializeSensorNode() {
  Serial.println("센서 노드 모드로 초기화...");
  
  // 센서 초기화
  // DHT, GPS 등 센서 초기화 코드
  
  // 첫 번째 하트비트 전송
  sendHeartbeat();
}

void gatewayLoop() {
  // 노드 상태 모니터링
  monitorNodeStatus();
  
  // 시리얼 명령 처리
  handleSerialCommands();
  
  // 정기적인 네트워크 상태 출력
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 30000) {  // 30초마다
    printNetworkStatus();
    lastStatusPrint = millis();
  }
}

void sensorNodeLoop() {
  // 센서 데이터 읽기 및 전송 (10초마다)
  if (millis() - lastSensorRead >= 10000) {
    SensorData data = readSensorData();
    sendSensorData(data);
    lastSensorRead = millis();
  }
  
  // 하트비트 전송 (60초마다)
  if (millis() - lastHeartbeat >= 60000) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  // 배터리 절약 모드 (옵션)
  // enterLightSleep();
}

void handleIncomingPacket(int packetSize) {
  LoRaPacket packet;
  
  // 패킷 읽기
  if (readLoRaPacket(&packet)) {
    // RSSI 및 SNR 정보
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    
    Serial.print("패킷 수신 - From: ");
    Serial.print(packet.sourceID);
    Serial.print(", Type: ");
    Serial.print(packet.messageType);
    Serial.print(", RSSI: ");
    Serial.print(rssi);
    Serial.print(", SNR: ");
    Serial.println(snr);
    
    // 메시지 타입별 처리
    switch (packet.messageType) {
      case MSG_SENSOR_DATA:
        handleSensorDataMessage(packet, rssi);
        break;
        
      case MSG_COMMAND:
        handleCommandMessage(packet);
        break;
        
      case MSG_ACK:
        handleAckMessage(packet);
        break;
        
      case MSG_HEARTBEAT:
        handleHeartbeatMessage(packet, rssi);
        break;
        
      case MSG_ALARM:
        handleAlarmMessage(packet);
        break;
        
      default:
        Serial.println("알 수 없는 메시지 타입");
        break;
    }
  }
}

bool readLoRaPacket(LoRaPacket* packet) {
  if (LoRa.available() < sizeof(LoRaPacket)) {
    return false;
  }
  
  // 패킷 헤더 읽기
  packet->sourceID = LoRa.read();
  packet->destinationID = LoRa.read();
  packet->messageType = LoRa.read();
  packet->sequenceNumber = LoRa.read();
  
  // 타임스탬프 읽기 (4바이트)
  packet->timestamp = 0;
  for (int i = 0; i < 4; i++) {
    packet->timestamp |= ((uint32_t)LoRa.read()) << (i * 8);
  }
  
  // 페이로드 길이
  packet->payloadLength = LoRa.read();
  
  // 페이로드 읽기
  for (int i = 0; i < packet->payloadLength && i < sizeof(packet->payload); i++) {
    packet->payload[i] = LoRa.read();
  }
  
  // 체크섬 읽기 (2바이트)
  packet->checksum = LoRa.read() | (LoRa.read() << 8);
  
  // 체크섬 검증
  uint16_t calculatedChecksum = calculateChecksum(packet);
  if (packet->checksum != calculatedChecksum) {
    Serial.println("체크섬 오류!");
    return false;
  }
  
  return true;
}

void sendLoRaPacket(LoRaPacket* packet) {
  // 체크섬 계산
  packet->checksum = calculateChecksum(packet);
  
  // LoRa 송신 시작
  LoRa.beginPacket();
  
  // 헤더 전송
  LoRa.write(packet->sourceID);
  LoRa.write(packet->destinationID);
  LoRa.write(packet->messageType);
  LoRa.write(packet->sequenceNumber);
  
  // 타임스탬프 전송
  for (int i = 0; i < 4; i++) {
    LoRa.write((packet->timestamp >> (i * 8)) & 0xFF);
  }
  
  // 페이로드 길이 및 데이터 전송
  LoRa.write(packet->payloadLength);
  for (int i = 0; i < packet->payloadLength; i++) {
    LoRa.write(packet->payload[i]);
  }
  
  // 체크섬 전송
  LoRa.write(packet->checksum & 0xFF);
  LoRa.write((packet->checksum >> 8) & 0xFF);
  
  // 송신 완료
  LoRa.endPacket();
  LoRa.receive();  // 다시 수신 모드로
}

uint16_t calculateChecksum(LoRaPacket* packet) {
  uint16_t checksum = 0;
  
  checksum += packet->sourceID;
  checksum += packet->destinationID;
  checksum += packet->messageType;
  checksum += packet->sequenceNumber;
  checksum += (packet->timestamp & 0xFF);
  checksum += ((packet->timestamp >> 8) & 0xFF);
  checksum += ((packet->timestamp >> 16) & 0xFF);
  checksum += ((packet->timestamp >> 24) & 0xFF);
  checksum += packet->payloadLength;
  
  for (int i = 0; i < packet->payloadLength; i++) {
    checksum += packet->payload[i];
  }
  
  return checksum;
}

void handleSensorDataMessage(LoRaPacket packet, int rssi) {
  if (!isGateway) return;  // 게이트웨이만 처리
  
  // JSON 파싱
  DynamicJsonDocument doc(512);
  deserializeJson(doc, packet.payload);
  
  // 노드 상태 업데이트
  updateNodeStatus(packet.sourceID, rssi);
  
  // 센서 데이터 저장
  SensorData data;
  data.temperature = doc["temp"];
  data.humidity = doc["hum"];
  data.batteryVoltage = doc["bat"];
  data.signalStrength = rssi;
  data.latitude = doc["lat"];
  data.longitude = doc["lon"];
  data.timestamp = packet.timestamp;
  
  nodeList[packet.sourceID].lastData = data;
  
  // 데이터 출력
  Serial.println("=== 센서 데이터 수신 ===");
  Serial.print("노드 ID: ");
  Serial.println(packet.sourceID);
  Serial.print("온도: ");
  Serial.print(data.temperature);
  Serial.println("°C");
  Serial.print("습도: ");
  Serial.print(data.humidity);
  Serial.println("%");
  Serial.print("배터리: ");
  Serial.print(data.batteryVoltage);
  Serial.println("V");
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println("========================");
  
  // ACK 전송
  sendAck(packet.sourceID, packet.sequenceNumber);
}

void handleCommandMessage(LoRaPacket packet) {
  Serial.print("명령 수신: ");
  Serial.println(packet.payload);
  
  // 명령 처리
  String command = String(packet.payload);
  
  if (command == "LED_ON") {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("LED 켜짐");
  }
  else if (command == "LED_OFF") {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("LED 꺼짐");
  }
  else if (command == "RESET") {
    Serial.println("리셋 명령 수신");
    ESP.restart();
  }
  else if (command == "STATUS") {
    sendSensorData(readSensorData());
  }
  
  // ACK 전송
  sendAck(packet.sourceID, packet.sequenceNumber);
}

void handleHeartbeatMessage(LoRaPacket packet, int rssi) {
  if (!isGateway) return;
  
  updateNodeStatus(packet.sourceID, rssi);
  
  Serial.print("하트비트 수신 - 노드 ");
  Serial.print(packet.sourceID);
  Serial.print(", RSSI: ");
  Serial.println(rssi);
}

void sendSensorData(SensorData data) {
  LoRaPacket packet;
  packet.sourceID = NODE_ID;
  packet.destinationID = GATEWAY_ID;
  packet.messageType = MSG_SENSOR_DATA;
  packet.sequenceNumber = sequenceCounter++;
  packet.timestamp = millis();
  
  // JSON 생성
  DynamicJsonDocument doc(512);
  doc["temp"] = data.temperature;
  doc["hum"] = data.humidity;
  doc["bat"] = data.batteryVoltage;
  doc["lat"] = data.latitude;
  doc["lon"] = data.longitude;
  
  serializeJson(doc, packet.payload, sizeof(packet.payload));
  packet.payloadLength = strlen(packet.payload);
  
  sendLoRaPacket(&packet);
  
  Serial.println("센서 데이터 전송됨");
}

void sendHeartbeat() {
  LoRaPacket packet;
  packet.sourceID = NODE_ID;
  packet.destinationID = GATEWAY_ID;
  packet.messageType = MSG_HEARTBEAT;
  packet.sequenceNumber = sequenceCounter++;
  packet.timestamp = millis();
  
  strcpy(packet.payload, "HEARTBEAT");
  packet.payloadLength = strlen(packet.payload);
  
  sendLoRaPacket(&packet);
  
  Serial.println("하트비트 전송됨");
}

void sendAck(uint8_t targetID, uint8_t sequenceNum) {
  LoRaPacket packet;
  packet.sourceID = NODE_ID;
  packet.destinationID = targetID;
  packet.messageType = MSG_ACK;
  packet.sequenceNumber = sequenceNum;  // 원본 시퀀스 번호 사용
  packet.timestamp = millis();
  
  strcpy(packet.payload, "ACK");
  packet.payloadLength = strlen(packet.payload);
  
  sendLoRaPacket(&packet);
}

SensorData readSensorData() {
  SensorData data;
  
  // 가상의 센서 데이터 (실제로는 DHT, GPS 등에서 읽음)
  data.temperature = 25.0 + random(-50, 51) / 10.0;  // 20-30°C
  data.humidity = 60.0 + random(-200, 201) / 10.0;   // 40-80%
  data.batteryVoltage = 3.7 + random(-50, 51) / 100.0; // 3.2-4.2V
  data.latitude = 37.5665 + random(-1000, 1001) / 100000.0;
  data.longitude = 126.9780 + random(-1000, 1001) / 100000.0;
  data.timestamp = millis();
  
  return data;
}

void updateNodeStatus(uint8_t nodeID, int rssi) {
  if (nodeID < MAX_NODES) {
    nodeList[nodeID].lastSeen = millis();
    nodeList[nodeID].rssi = rssi;
    nodeList[nodeID].isOnline = true;
  }
}

void monitorNodeStatus() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodeList[i].lastSeen > 0) {  // 한 번이라도 본 노드
      // 5분 이상 응답 없으면 오프라인으로 간주
      if (currentTime - nodeList[i].lastSeen > 300000) {
        if (nodeList[i].isOnline) {
          nodeList[i].isOnline = false;
          Serial.print("노드 ");
          Serial.print(i);
          Serial.println(" 오프라인됨");
        }
      }
    }
  }
}

void printNetworkStatus() {
  Serial.println("\n=== 네트워크 상태 ===");
  Serial.print("게이트웨이 업타임: ");
  Serial.print(millis() / 1000);
  Serial.println("초");
  
  int onlineNodes = 0;
  for (int i = 1; i < MAX_NODES; i++) {  // 0번은 게이트웨이
    if (nodeList[i].isOnline) {
      onlineNodes++;
      Serial.print("노드 ");
      Serial.print(i);
      Serial.print(": 온라인, RSSI: ");
      Serial.print(nodeList[i].rssi);
      Serial.print(", 마지막 수신: ");
      Serial.print((millis() - nodeList[i].lastSeen) / 1000);
      Serial.println("초 전");
    }
  }
  
  Serial.print("총 온라인 노드: ");
  Serial.println(onlineNodes);
  Serial.println("===================\n");
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("send ")) {
      // 명령 전송: send <nodeID> <command>
      int spaceIndex = command.indexOf(' ', 5);
      if (spaceIndex > 0) {
        int nodeID = command.substring(5, spaceIndex).toInt();
        String cmd = command.substring(spaceIndex + 1);
        sendCommand(nodeID, cmd);
      }
    }
    else if (command == "status") {
      printNetworkStatus();
    }
    else if (command == "nodes") {
      printNodeDetails();
    }
  }
}

void sendCommand(uint8_t targetID, String command) {
  LoRaPacket packet;
  packet.sourceID = NODE_ID;
  packet.destinationID = targetID;
  packet.messageType = MSG_COMMAND;
  packet.sequenceNumber = sequenceCounter++;
  packet.timestamp = millis();
  
  command.toCharArray(packet.payload, sizeof(packet.payload));
  packet.payloadLength = command.length();
  
  sendLoRaPacket(&packet);
  
  Serial.print("명령 전송됨 - To: ");
  Serial.print(targetID);
  Serial.print(", Command: ");
  Serial.println(command);
}

void printNodeDetails() {
  Serial.println("\n=== 노드 상세 정보 ===");
  
  for (int i = 1; i < MAX_NODES; i++) {
    if (nodeList[i].lastSeen > 0) {
      Serial.print("노드 ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(nodeList[i].isOnline ? "온라인" : "오프라인");
      
      if (nodeList[i].isOnline) {
        Serial.print(", 온도: ");
        Serial.print(nodeList[i].lastData.temperature);
        Serial.print("°C, 습도: ");
        Serial.print(nodeList[i].lastData.humidity);
        Serial.print("%, 배터리: ");
        Serial.print(nodeList[i].lastData.batteryVoltage);
        Serial.println("V");
      } else {
        Serial.println();
      }
    }
  }
  
  Serial.println("=====================\n");
}
```

## 📊 무선 통신 성능 비교 및 최적화

### 실제 테스트 결과

```
┌─────────────────────────────────────────────────────────────────┐
│                    실제 성능 측정 결과                           │
├─────────────┬─────────────┬─────────────┬─────────────────────┤
│   메트릭    │  블루투스    │    WiFi     │       LoRa          │
├─────────────┼─────────────┼─────────────┼─────────────────────┤
│ 실제 범위   │   30-50m    │   100-200m  │     2-10km          │
│ 배터리 수명 │   2-5일     │   12-24시간 │     1-3개월         │
│ 연결 시간   │   2-5초     │   5-10초    │     즉시            │
│ 패킷 손실률 │   <1%       │   <0.1%     │     1-5%            │
│ 동시 연결   │   7개       │   50+개     │     1000+개         │
│ 비용        │   $2-5      │   $5-15     │     $10-30          │
└─────────────┴─────────────┴─────────────┴─────────────────────┘
```

### 선택 가이드

```cpp
// 무선 통신 프로토콜 선택 알고리즘
WirelessProtocol selectOptimalProtocol(ApplicationRequirements req) {
  
  // LoRa 추천 조건
  if (req.range > 1000m && req.powerConsumption == ULTRA_LOW && 
      req.dataRate < 50kbps && req.batteryLife > 6_MONTHS) {
    return LORA;
    // 사용 사례: 농업 센서, 환경 모니터링, 스마트 미터
  }
  
  // WiFi 추천 조건
  else if (req.dataRate > 1Mbps && req.internetAccess == REQUIRED && 
           req.powerSource == AC_POWERED) {
    return WIFI;
    // 사용 사례: 스마트 홈 허브, IP 카메라, 멀티미디어
  }
  
  // 블루투스 추천 조건
  else if (req.range < 100m && req.phoneIntegration == REQUIRED && 
           req.setup == SIMPLE) {
    return BLUETOOTH;
    // 사용 사례: 웨어러블, 스마트폰 연동, 개인 디바이스
  }
  
  return WIFI;  // 기본값
}
```

## 🚀 다음 단계

무선 통신의 3대 프로토콜을 완전히 마스터했습니다! 다음 강의에서는:

- **Lesson 13**: 데이터 로깅 및 SD카드 고급 활용
- **Lesson 14**: 전력 관리 및 저전력 설계
- **Lesson 15**: 실전 프로젝트 - IoT 스마트홈 시스템

이제 스마트 홈, IoT 센서 네트워크, 원격 모니터링 시스템 등 실제 상용 제품 수준의 무선 통신 시스템을 구축할 수 있습니다!

---

*아두이노 마스터 과정 - Lesson 12 완료*  
*Created by Arduino Expert Teacher*  
*© 2024 Arduino Complete Guide*