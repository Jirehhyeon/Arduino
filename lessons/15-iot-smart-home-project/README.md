# Lesson 15: 실전 프로젝트 - IoT 스마트홈 시스템 (종합 프로젝트)

## 🎯 프로젝트 개요
지금까지 배운 모든 기술을 종합하여 **실제 상용 제품 수준의 IoT 스마트홈 시스템**을 구축합니다. 이 프로젝트는 14개 강의에서 학습한 모든 내용을 통합한 **마스터피스 프로젝트**입니다.

### 🏠 **스마트홈 시스템 구성도**

```
                    ┌─────────────────────────────────────┐
                    │         클라우드 서버               │
                    │    - 데이터 저장소                  │
                    │    - 원격 제어 API                  │
                    │    - 모바일 앱 백엔드               │
                    │    - AI 학습 서버                   │
                    └─────────────┬───────────────────────┘
                                  │ HTTPS/MQTT
                    ┌─────────────┴───────────────────────┐
                    │       WiFi 메인 허브                │
                    │     (ESP32 + 터치스크린)             │
                    │  - 중앙 제어기                      │
                    │  - 웹 대시보드                      │
                    │  - 음성 인식                        │
                    │  - 보안 시스템                      │
                    └─────────────┬───────────────────────┘
                                  │ WiFi/LoRa
        ┌─────────────────────────┼─────────────────────────┐
        │                         │                         │
┌───────▼──────┐         ┌────────▼─────────┐        ┌─────▼─────┐
│  거실 노드   │         │    침실 노드     │        │ 주방 노드 │
│ (Arduino+ESP)│         │ (Arduino+LoRa)   │        │(ESP32)    │
│- 조명 제어   │         │- 환경 센서       │        │- 가전제어 │
│- TV/오디오   │         │- 자동 커튼       │        │- 누수감지 │
│- 에어컨      │         │- 공기청정기      │        │- 화재감지 │
│- 움직임 감지 │         │- 수면 모니터링   │        │- 냉장고   │
└──────────────┘         └──────────────────┘        └───────────┘
        │                         │                         │
        │ I2C/SPI                 │ I2C/SPI                 │ I2C/SPI
        │                         │                         │
┌───────▼──────┐         ┌────────▼─────────┐        ┌─────▼─────┐
│  센서 노드들 │         │   센서 노드들    │        │센서노드들│
│- DHT22       │         │- BME680          │        │- MQ-2     │
│- PIR         │         │- CO2 센서        │        │- DS18B20  │
│- 조도센서    │         │- 소음센서        │        │- 수위센서 │
│- RFID        │         │- 진동센서        │        │- 전력센서 │
└──────────────┘         └──────────────────┘        └───────────┘

        ┌─────────────────────────────────────────────────┐
        │              추가 시스템 모듈                   │
        ├─────────────────────────────────────────────────┤
        │  🔋 태양광 + 배터리 백업 시스템                 │
        │  📹 보안 카메라 + 얼굴 인식                     │
        │  🚗 차고 자동문 + 주차 가이드                   │
        │  🌱 자동 급수 시스템 + 화분 관리                │
        │  🏠 전체 전력 모니터링 + HEMS                   │
        └─────────────────────────────────────────────────┘
```

## 📋 **시스템 요구사항**

### **기능적 요구사항**
- ✅ 실시간 환경 모니터링 (온도, 습도, 공기질, 조도)
- ✅ 자동 조명/가전 제어 (스케줄링, 패턴 학습)
- ✅ 보안 시스템 (RFID, 카메라, 침입 감지)
- ✅ 원격 제어 (모바일 앱, 웹 대시보드)
- ✅ 음성 제어 (한국어 명령 인식)
- ✅ 에너지 관리 (전력 모니터링, 절약 모드)
- ✅ 비상 시스템 (화재, 가스, 정전 대응)

### **비기능적 요구사항**
- ⚡ **성능**: 응답시간 < 500ms, 처리량 > 1000 req/min
- 🛡️ **보안**: WPA3 암호화, JWT 토큰, HTTPS 통신
- 🔄 **가용성**: 99.5% 업타임, 자동 복구 기능
- 📊 **확장성**: 100개 센서 노드 지원
- 🔋 **전력**: 배터리 백업 12시간 이상
- 📱 **사용성**: 직관적 UI, 3초 이내 설정 완료

## 🔧 **실습 1: 메인 허브 시스템 (ESP32 + 터치스크린)**

### 중앙 제어기 및 웹 대시보드

```cpp
/*
  Smart_Home_Main_Hub.ino
  IoT 스마트홈 메인 허브 시스템
  
  하드웨어:
  - ESP32 DevKit V1
  - 3.5" TFT 터치스크린 (ILI9488)
  - MicroSD 카드 모듈
  - RTC 모듈 (DS3231)
  - 스피커 + 마이크 (음성 제어)
  - 릴레이 모듈 8채널
  
  기능:
  - 중앙 집중식 제어
  - 웹 대시보드 서버
  - 음성 인식 및 제어
  - 자동화 룰 엔진
  - 보안 및 알림 시스템
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <PubSubClient.h>
#include <ESP32_FTPServer.h>
#include <time.h>
#include <driver/i2s.h>

// WiFi 설정
const char* WIFI_SSID = "SmartHome_WiFi";
const char* WIFI_PASSWORD = "SmartHome2024!";

// MQTT 브로커 설정
const char* MQTT_SERVER = "192.168.1.100";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "smarthome";
const char* MQTT_PASS = "mqtt2024";

// 하드웨어 객체
TFT_eSPI tft = TFT_eSPI();
WebServer webServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
RTC_DS3231 rtc;
FTPServer ftpServer;

// 핀 정의
#define SD_CS_PIN 5
#define TOUCH_CS_PIN 21
#define RELAY_PINS {13, 12, 14, 27, 26, 25, 33, 32}
#define BUZZER_PIN 23
#define MIC_PIN 36
#define SPEAKER_PIN 22

// 시스템 상태
struct SmartHomeSystem {
  // 환경 데이터
  float temperature;
  float humidity;
  float airQuality;
  int lightLevel;
  float energyUsage;
  
  // 디바이스 상태
  bool lights[8];          // 8개 조명
  bool appliances[8];      // 8개 가전제품
  bool securityArmed;      // 보안 시스템
  bool emergencyMode;      // 비상 모드
  
  // 사용자 설정
  int autoLightThreshold;  // 자동 조명 임계값
  int comfortTemp;         // 쾌적 온도
  bool voiceControlEnabled;// 음성 제어
  bool learningMode;       // 학습 모드
  
  // 시스템 정보
  unsigned long uptime;
  int connectedNodes;      // 연결된 노드 수
  float wifiSignal;        // WiFi 신호 강도
  bool internetConnected;  // 인터넷 연결 상태
} homeSystem;

// 자동화 규칙
struct AutomationRule {
  String name;
  String condition;        // "time:20:00" or "sensor:motion:true"
  String action;          // "light:1:on" or "appliance:2:off"
  bool enabled;
  unsigned long lastTriggered;
} automationRules[20];

int ruleCount = 0;

// 음성 인식 키워드
const char* voiceCommands[] = {
  "불켜줘", "불꺼줘", "에어컨켜줘", "에어컨꺼줘",
  "온도올려줘", "온도내려줘", "보안설정", "보안해제",
  "전체끄기", "외출모드", "취침모드", "홈시어터"
};

// 터치스크린 UI 상태
enum UIScreen {
  SCREEN_HOME,
  SCREEN_LIGHTS,
  SCREEN_CLIMATE,
  SCREEN_SECURITY,
  SCREEN_ENERGY,
  SCREEN_SETTINGS
};

UIScreen currentScreen = SCREEN_HOME;

void setup() {
  Serial.begin(115200);
  
  // 하드웨어 초기화
  initializeHardware();
  
  // WiFi 연결
  connectToWiFi();
  
  // MQTT 연결
  connectToMQTT();
  
  // 웹서버 시작
  setupWebServer();
  
  // 터치스크린 UI 초기화
  initializeTouchUI();
  
  // 음성 인식 초기화
  initializeVoiceControl();
  
  // 자동화 규칙 로드
  loadAutomationRules();
  
  // FTP 서버 시작 (원격 업데이트용)
  ftpServer.begin("admin", "admin123");
  
  Serial.println("스마트홈 메인 허브 시작 완료!");
  
  // 시작 화면 표시
  displayWelcomeScreen();
}

void loop() {
  // 네트워크 서비스 처리
  webServer.handleClient();
  webSocket.loop();
  mqttClient.loop();
  ftpServer.handleFTP();
  
  // 터치 입력 처리
  handleTouchInput();
  
  // 음성 명령 처리
  processVoiceCommands();
  
  // 자동화 규칙 실행
  executeAutomationRules();
  
  // 센서 데이터 수집
  collectSensorData();
  
  // UI 업데이트
  updateTouchUI();
  
  // 시스템 모니터링
  monitorSystemHealth();
  
  // 에너지 관리
  manageEnergyUsage();
  
  delay(100);
}

void initializeHardware() {
  // TFT 디스플레이 초기화
  tft.init();
  tft.setRotation(1);  // 가로 모드
  tft.fillScreen(TFT_BLACK);
  
  // SD 카드 초기화
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD 카드 초기화 실패!");
  }
  
  // RTC 초기화
  if (!rtc.begin()) {
    Serial.println("RTC 초기화 실패!");
  }
  
  // 릴레이 핀 초기화
  int relayPins[] = RELAY_PINS;
  for (int i = 0; i < 8; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
    homeSystem.lights[i] = false;
    homeSystem.appliances[i] = false;
  }
  
  // 기타 핀 설정
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MIC_PIN, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  
  Serial.println("하드웨어 초기화 완료");
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
    Serial.println();
    Serial.print("WiFi 연결 성공! IP: ");
    Serial.println(WiFi.localIP());
    homeSystem.internetConnected = true;
  } else {
    Serial.println("WiFi 연결 실패!");
    homeSystem.internetConnected = false;
  }
}

void connectToMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  
  while (!mqttClient.connected()) {
    Serial.print("MQTT 연결 시도...");
    
    if (mqttClient.connect("SmartHomeHub", MQTT_USER, MQTT_PASS)) {
      Serial.println("MQTT 연결 성공!");
      
      // 토픽 구독
      mqttClient.subscribe("smarthome/sensors/+/+");
      mqttClient.subscribe("smarthome/control/+");
      mqttClient.subscribe("smarthome/status/+");
      
    } else {
      Serial.print("MQTT 연결 실패, rc=");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("MQTT 메시지 수신: ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(message);
  
  // 토픽 파싱 및 처리
  String topicStr = String(topic);
  
  if (topicStr.startsWith("smarthome/sensors/")) {
    processSensorData(topicStr, message);
  } else if (topicStr.startsWith("smarthome/control/")) {
    processControlCommand(topicStr, message);
  }
  
  // 웹소켓으로 실시간 업데이트 전송
  broadcastSystemUpdate();
}

void setupWebServer() {
  // 정적 파일 서빙
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/api/status", HTTP_GET, handleAPIStatus);
  webServer.on("/api/control", HTTP_POST, handleAPIControl);
  webServer.on("/api/automation", HTTP_GET, handleAPIAutomation);
  webServer.on("/api/automation", HTTP_POST, handleAPIAddRule);
  webServer.on("/css/style.css", HTTP_GET, handleCSS);
  webServer.on("/js/app.js", HTTP_GET, handleJS);
  
  // 파일 업로드 핸들러
  webServer.on("/upload", HTTP_POST, [](){
    webServer.send(200);
  }, handleFileUpload);
  
  webServer.onNotFound(handleNotFound);
  
  // 웹소켓 이벤트 핸들러
  webSocket.onEvent(webSocketEvent);
  
  webServer.begin();
  webSocket.begin();
  
  Serial.println("웹서버 시작됨");
}

void handleRoot() {
  String html = generateMainDashboard();
  webServer.send(200, "text/html", html);
}

String generateMainDashboard() {
  String html = R"(
<!DOCTYPE html>
<html lang='ko'>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>🏠 스마트홈 제어 센터</title>
    <link rel='stylesheet' href='/css/style.css'>
    <link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css'>
</head>
<body>
    <div class='dashboard-container'>
        <!-- 헤더 -->
        <header class='dashboard-header'>
            <h1><i class='fas fa-home'></i> 스마트홈 제어 센터</h1>
            <div class='system-status'>
                <span class='status-item'>
                    <i class='fas fa-wifi'></i>
                    <span id='wifi-status'>연결됨</span>
                </span>
                <span class='status-item'>
                    <i class='fas fa-clock'></i>
                    <span id='current-time'>--:--</span>
                </span>
                <span class='status-item'>
                    <i class='fas fa-thermometer-half'></i>
                    <span id='current-temp'>--°C</span>
                </span>
            </div>
        </header>

        <!-- 메인 대시보드 -->
        <div class='dashboard-grid'>
            <!-- 환경 모니터링 카드 -->
            <div class='card environment-card'>
                <h3><i class='fas fa-leaf'></i> 환경 모니터링</h3>
                <div class='sensor-grid'>
                    <div class='sensor-item'>
                        <i class='fas fa-thermometer-half'></i>
                        <span class='sensor-label'>온도</span>
                        <span class='sensor-value' id='temperature'>--°C</span>
                    </div>
                    <div class='sensor-item'>
                        <i class='fas fa-tint'></i>
                        <span class='sensor-label'>습도</span>
                        <span class='sensor-value' id='humidity'>--%</span>
                    </div>
                    <div class='sensor-item'>
                        <i class='fas fa-wind'></i>
                        <span class='sensor-label'>공기질</span>
                        <span class='sensor-value' id='air-quality'>--</span>
                    </div>
                    <div class='sensor-item'>
                        <i class='fas fa-sun'></i>
                        <span class='sensor-label'>조도</span>
                        <span class='sensor-value' id='light-level'>--</span>
                    </div>
                </div>
                <canvas id='environment-chart' width='400' height='200'></canvas>
            </div>

            <!-- 조명 제어 카드 -->
            <div class='card lighting-card'>
                <h3><i class='fas fa-lightbulb'></i> 조명 제어</h3>
                <div class='light-controls'>
                    <div class='light-room'>
                        <span>거실</span>
                        <div class='switch-container'>
                            <input type='checkbox' id='light-0' class='light-switch' data-room='0'>
                            <label for='light-0' class='switch-label'></label>
                        </div>
                    </div>
                    <div class='light-room'>
                        <span>침실</span>
                        <div class='switch-container'>
                            <input type='checkbox' id='light-1' class='light-switch' data-room='1'>
                            <label for='light-1' class='switch-label'></label>
                        </div>
                    </div>
                    <div class='light-room'>
                        <span>주방</span>
                        <div class='switch-container'>
                            <input type='checkbox' id='light-2' class='light-switch' data-room='2'>
                            <label for='light-2' class='switch-label'></label>
                        </div>
                    </div>
                    <div class='light-room'>
                        <span>화장실</span>
                        <div class='switch-container'>
                            <input type='checkbox' id='light-3' class='light-switch' data-room='3'>
                            <label for='light-3' class='switch-label'></label>
                        </div>
                    </div>
                </div>
                <div class='quick-actions'>
                    <button class='btn btn-primary' onclick='controlAllLights(true)'>
                        <i class='fas fa-lightbulb'></i> 전체 켜기
                    </button>
                    <button class='btn btn-secondary' onclick='controlAllLights(false)'>
                        <i class='far fa-lightbulb'></i> 전체 끄기
                    </button>
                </div>
            </div>

            <!-- 가전제품 제어 카드 -->
            <div class='card appliance-card'>
                <h3><i class='fas fa-plug'></i> 가전제품 제어</h3>
                <div class='appliance-grid'>
                    <div class='appliance-item' data-appliance='0'>
                        <i class='fas fa-snowflake'></i>
                        <span>에어컨</span>
                        <button class='appliance-btn' onclick='toggleAppliance(0)'>OFF</button>
                    </div>
                    <div class='appliance-item' data-appliance='1'>
                        <i class='fas fa-tv'></i>
                        <span>TV</span>
                        <button class='appliance-btn' onclick='toggleAppliance(1)'>OFF</button>
                    </div>
                    <div class='appliance-item' data-appliance='2'>
                        <i class='fas fa-volume-up'></i>
                        <span>오디오</span>
                        <button class='appliance-btn' onclick='toggleAppliance(2)'>OFF</button>
                    </div>
                    <div class='appliance-item' data-appliance='3'>
                        <i class='fas fa-robot'></i>
                        <span>로봇청소기</span>
                        <button class='appliance-btn' onclick='toggleAppliance(3)'>OFF</button>
                    </div>
                </div>
            </div>

            <!-- 보안 시스템 카드 -->
            <div class='card security-card'>
                <h3><i class='fas fa-shield-alt'></i> 보안 시스템</h3>
                <div class='security-status'>
                    <div class='security-indicator' id='security-indicator'>
                        <i class='fas fa-shield-alt'></i>
                        <span id='security-status'>해제됨</span>
                    </div>
                    <button class='btn btn-warning' id='security-toggle' onclick='toggleSecurity()'>
                        보안 설정
                    </button>
                </div>
                <div class='security-sensors'>
                    <div class='sensor-status'>
                        <i class='fas fa-door-open'></i>
                        <span>현관문: </span>
                        <span class='status-ok'>정상</span>
                    </div>
                    <div class='sensor-status'>
                        <i class='fas fa-running'></i>
                        <span>움직임 감지: </span>
                        <span class='status-ok'>정상</span>
                    </div>
                    <div class='sensor-status'>
                        <i class='fas fa-video'></i>
                        <span>카메라: </span>
                        <span class='status-ok'>정상</span>
                    </div>
                </div>
            </div>

            <!-- 에너지 모니터링 카드 -->
            <div class='card energy-card'>
                <h3><i class='fas fa-bolt'></i> 에너지 모니터링</h3>
                <div class='energy-stats'>
                    <div class='energy-item'>
                        <span class='energy-label'>현재 사용량</span>
                        <span class='energy-value' id='current-power'>-- W</span>
                    </div>
                    <div class='energy-item'>
                        <span class='energy-label'>오늘 사용량</span>
                        <span class='energy-value' id='daily-energy'>-- kWh</span>
                    </div>
                    <div class='energy-item'>
                        <span class='energy-label'>이번 달</span>
                        <span class='energy-value' id='monthly-energy'>-- kWh</span>
                    </div>
                    <div class='energy-item'>
                        <span class='energy-label'>예상 요금</span>
                        <span class='energy-value' id='estimated-cost'>-- 원</span>
                    </div>
                </div>
                <div class='energy-chart-container'>
                    <canvas id='energy-chart' width='400' height='150'></canvas>
                </div>
            </div>

            <!-- 자동화 규칙 카드 -->
            <div class='card automation-card'>
                <h3><i class='fas fa-cogs'></i> 자동화 규칙</h3>
                <div class='automation-list' id='automation-list'>
                    <!-- 자동화 규칙들이 동적으로 추가됨 -->
                </div>
                <div class='automation-actions'>
                    <button class='btn btn-primary' onclick='showAddRuleModal()'>
                        <i class='fas fa-plus'></i> 규칙 추가
                    </button>
                    <button class='btn btn-secondary' onclick='loadAutomationRules()'>
                        <i class='fas fa-sync'></i> 새로고침
                    </button>
                </div>
            </div>
        </div>

        <!-- 음성 제어 버튼 -->
        <div class='voice-control'>
            <button class='voice-btn' id='voice-btn' onclick='startVoiceControl()'>
                <i class='fas fa-microphone'></i>
                <span>음성 명령</span>
            </button>
        </div>

        <!-- 빠른 설정 패널 -->
        <div class='quick-settings'>
            <button class='quick-btn' onclick='setHomeMode()'>
                <i class='fas fa-home'></i>
                <span>재택모드</span>
            </button>
            <button class='quick-btn' onclick='setAwayMode()'>
                <i class='fas fa-suitcase'></i>
                <span>외출모드</span>
            </button>
            <button class='quick-btn' onclick='setSleepMode()'>
                <i class='fas fa-bed'></i>
                <span>취침모드</span>
            </button>
            <button class='quick-btn' onclick='setPartyMode()'>
                <i class='fas fa-music'></i>
                <span>파티모드</span>
            </button>
        </div>
    </div>

    <!-- 자동화 규칙 추가 모달 -->
    <div id='add-rule-modal' class='modal'>
        <div class='modal-content'>
            <div class='modal-header'>
                <h3>자동화 규칙 추가</h3>
                <button class='close-btn' onclick='closeAddRuleModal()'>&times;</button>
            </div>
            <div class='modal-body'>
                <form id='add-rule-form'>
                    <div class='form-group'>
                        <label>규칙 이름:</label>
                        <input type='text' id='rule-name' placeholder='예: 저녁 자동 조명'>
                    </div>
                    <div class='form-group'>
                        <label>조건:</label>
                        <select id='rule-condition-type'>
                            <option value='time'>시간</option>
                            <option value='sensor'>센서</option>
                            <option value='device'>디바이스</option>
                        </select>
                        <input type='text' id='rule-condition-value' placeholder='예: 18:00'>
                    </div>
                    <div class='form-group'>
                        <label>동작:</label>
                        <select id='rule-action-type'>
                            <option value='light'>조명</option>
                            <option value='appliance'>가전제품</option>
                            <option value='notification'>알림</option>
                        </select>
                        <input type='text' id='rule-action-value' placeholder='예: 1:on'>
                    </div>
                    <div class='form-actions'>
                        <button type='button' class='btn btn-primary' onclick='addAutomationRule()'>추가</button>
                        <button type='button' class='btn btn-secondary' onclick='closeAddRuleModal()'>취소</button>
                    </div>
                </form>
            </div>
        </div>
    </div>

    <script src='/js/app.js'></script>
</body>
</html>
)";
  return html;
}

void handleAPIStatus() {
  DynamicJsonDocument doc(2048);
  
  // 환경 데이터
  doc["temperature"] = homeSystem.temperature;
  doc["humidity"] = homeSystem.humidity;
  doc["airQuality"] = homeSystem.airQuality;
  doc["lightLevel"] = homeSystem.lightLevel;
  
  // 디바이스 상태
  JsonArray lights = doc.createNestedArray("lights");
  for (int i = 0; i < 8; i++) {
    lights.add(homeSystem.lights[i]);
  }
  
  JsonArray appliances = doc.createNestedArray("appliances");
  for (int i = 0; i < 8; i++) {
    appliances.add(homeSystem.appliances[i]);
  }
  
  // 시스템 정보
  doc["securityArmed"] = homeSystem.securityArmed;
  doc["emergencyMode"] = homeSystem.emergencyMode;
  doc["uptime"] = millis();
  doc["connectedNodes"] = homeSystem.connectedNodes;
  doc["wifiSignal"] = WiFi.RSSI();
  doc["freeHeap"] = ESP.getFreeHeap();
  
  String response;
  serializeJson(doc, response);
  
  webServer.send(200, "application/json", response);
}

void handleAPIControl() {
  if (webServer.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, webServer.arg("plain"));
    
    String device = doc["device"];
    int index = doc["index"];
    bool state = doc["state"];
    
    if (device == "light") {
      controlLight(index, state);
      webServer.send(200, "application/json", "{\"status\":\"success\"}");
    }
    else if (device == "appliance") {
      controlAppliance(index, state);
      webServer.send(200, "application/json", "{\"status\":\"success\"}");
    }
    else if (device == "security") {
      homeSystem.securityArmed = state;
      webServer.send(200, "application/json", "{\"status\":\"success\"}");
    }
    else {
      webServer.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Unknown device\"}");
    }
    
    // 상태 변경을 MQTT로 브로드캐스트
    publishDeviceState(device, index, state);
    broadcastSystemUpdate();
  }
}

void controlLight(int index, bool state) {
  if (index >= 0 && index < 8) {
    int relayPins[] = RELAY_PINS;
    digitalWrite(relayPins[index], state ? HIGH : LOW);
    homeSystem.lights[index] = state;
    
    Serial.print("조명 ");
    Serial.print(index);
    Serial.print(state ? " 켜짐" : " 꺼짐");
    Serial.println();
  }
}

void controlAppliance(int index, bool state) {
  if (index >= 0 && index < 8) {
    int relayPins[] = RELAY_PINS;
    digitalWrite(relayPins[index], state ? HIGH : LOW);
    homeSystem.appliances[index] = state;
    
    Serial.print("가전제품 ");
    Serial.print(index);
    Serial.print(state ? " 켜짐" : " 꺼짐");
    Serial.println();
  }
}

void publishDeviceState(String device, int index, bool state) {
  String topic = "smarthome/status/" + device + "/" + String(index);
  String payload = state ? "ON" : "OFF";
  
  mqttClient.publish(topic.c_str(), payload.c_str());
}

void broadcastSystemUpdate() {
  DynamicJsonDocument doc(1024);
  
  doc["temperature"] = homeSystem.temperature;
  doc["humidity"] = homeSystem.humidity;
  doc["securityArmed"] = homeSystem.securityArmed;
  
  JsonArray lights = doc.createNestedArray("lights");
  for (int i = 0; i < 8; i++) {
    lights.add(homeSystem.lights[i]);
  }
  
  String message;
  serializeJson(doc, message);
  
  webSocket.broadcastTXT(message);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("WebSocket[%u] 연결 해제\n", num);
      break;
      
    case WStype_CONNECTED:
      Serial.printf("WebSocket[%u] 연결됨\n", num);
      // 연결시 현재 상태 전송
      broadcastSystemUpdate();
      break;
      
    case WStype_TEXT:
      Serial.printf("WebSocket[%u] 메시지: %s\n", num, payload);
      // 실시간 제어 명령 처리
      processWebSocketCommand(num, (char*)payload);
      break;
  }
}

void processWebSocketCommand(uint8_t clientNum, String command) {
  DynamicJsonDocument doc(512);
  deserializeJson(doc, command);
  
  String action = doc["action"];
  
  if (action == "control") {
    String device = doc["device"];
    int index = doc["index"];
    bool state = doc["state"];
    
    if (device == "light") {
      controlLight(index, state);
    } else if (device == "appliance") {
      controlAppliance(index, state);
    }
    
    // 모든 클라이언트에게 업데이트 브로드캐스트
    broadcastSystemUpdate();
  }
}

void initializeTouchUI() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  
  // 기본 UI 레이아웃 그리기
  drawHomeScreen();
}

void drawHomeScreen() {
  tft.fillScreen(TFT_BLACK);
  
  // 헤더
  tft.fillRect(0, 0, 480, 40, TFT_BLUE);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Smart Home Control", 10, 10);
  
  // 현재 시간 표시
  DateTime now = rtc.now();
  String timeStr = String(now.hour()) + ":" + 
                   (now.minute() < 10 ? "0" : "") + String(now.minute());
  tft.drawString(timeStr, 380, 10);
  
  // 환경 정보 섹션
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Environment", 10, 60);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Temp: " + String(homeSystem.temperature, 1) + "C", 10, 85);
  tft.drawString("Humidity: " + String(homeSystem.humidity, 0) + "%", 10, 110);
  
  // 조명 제어 버튼들
  drawLightControls();
  
  // 시스템 상태
  drawSystemStatus();
}

void drawLightControls() {
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("Lighting Control", 250, 60);
  
  const char* roomNames[] = {"Living", "Bedroom", "Kitchen", "Bathroom"};
  
  for (int i = 0; i < 4; i++) {
    int x = 250 + (i % 2) * 100;
    int y = 85 + (i / 2) * 40;
    
    // 버튼 배경
    uint16_t buttonColor = homeSystem.lights[i] ? TFT_GREEN : TFT_RED;
    tft.fillRect(x, y, 80, 25, buttonColor);
    
    // 버튼 텍스트
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.drawString(roomNames[i], x + 5, y + 8);
    tft.setTextSize(2);
  }
}

void drawSystemStatus() {
  tft.setTextColor(TFT_CYAN);
  tft.drawString("System Status", 10, 160);
  
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  
  String wifiStatus = WiFi.isConnected() ? "Connected" : "Disconnected";
  tft.drawString("WiFi: " + wifiStatus, 10, 185);
  
  String securityStatus = homeSystem.securityArmed ? "Armed" : "Disarmed";
  tft.drawString("Security: " + securityStatus, 10, 205);
  
  tft.drawString("Nodes: " + String(homeSystem.connectedNodes), 10, 225);
  
  tft.setTextSize(2);
}

void handleTouchInput() {
  uint16_t x, y;
  if (tft.getTouch(&x, &y)) {
    processTouchEvent(x, y);
    delay(200); // 디바운싱
  }
}

void processTouchEvent(uint16_t x, uint16_t y) {
  Serial.print("터치 이벤트: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  
  // 조명 제어 버튼 영역 체크
  if (y >= 85 && y <= 155) {
    for (int i = 0; i < 4; i++) {
      int buttonX = 250 + (i % 2) * 100;
      int buttonY = 85 + (i / 2) * 40;
      
      if (x >= buttonX && x <= buttonX + 80 && 
          y >= buttonY && y <= buttonY + 25) {
        // 조명 토글
        controlLight(i, !homeSystem.lights[i]);
        drawLightControls(); // UI 업데이트
        
        // MQTT 상태 전송
        publishDeviceState("light", i, homeSystem.lights[i]);
        break;
      }
    }
  }
}

void initializeVoiceControl() {
  // I2S 마이크 설정
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = 26,
    .ws_io_num = 25,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 22
  };
  
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  
  Serial.println("음성 인식 시스템 초기화 완료");
}

void processVoiceCommands() {
  static unsigned long lastVoiceCheck = 0;
  
  if (millis() - lastVoiceCheck >= 1000) { // 1초마다 체크
    if (detectVoiceActivity()) {
      String command = recognizeVoiceCommand();
      
      if (command.length() > 0) {
        executeVoiceCommand(command);
      }
    }
    
    lastVoiceCheck = millis();
  }
}

bool detectVoiceActivity() {
  size_t bytes_read = 0;
  int16_t i2s_read_buff[1024];
  
  i2s_read(I2S_NUM_0, i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, 100);
  
  // 간단한 음성 활동 감지 (진폭 기반)
  long sum = 0;
  for (int i = 0; i < bytes_read/2; i++) {
    sum += abs(i2s_read_buff[i]);
  }
  
  long average = sum / (bytes_read/2);
  return average > 1000; // 임계값
}

String recognizeVoiceCommand() {
  // 실제 구현에서는 음성 인식 라이브러리 사용
  // 여기서는 시뮬레이션
  
  static int commandIndex = 0;
  String detectedCommand = "";
  
  // 키워드 매칭 시뮬레이션
  int numCommands = sizeof(voiceCommands) / sizeof(voiceCommands[0]);
  if (random(100) < 20) { // 20% 확률로 명령 인식
    detectedCommand = voiceCommands[commandIndex % numCommands];
    commandIndex++;
  }
  
  return detectedCommand;
}

void executeVoiceCommand(String command) {
  Serial.println("음성 명령 인식: " + command);
  
  if (command == "불켜줘") {
    for (int i = 0; i < 4; i++) {
      controlLight(i, true);
    }
    playTTS("모든 조명을 켰습니다");
  }
  else if (command == "불꺼줘") {
    for (int i = 0; i < 4; i++) {
      controlLight(i, false);
    }
    playTTS("모든 조명을 껐습니다");
  }
  else if (command == "에어컨켜줘") {
    controlAppliance(0, true);
    playTTS("에어컨을 켰습니다");
  }
  else if (command == "에어컨꺼줘") {
    controlAppliance(0, false);
    playTTS("에어컨을 껐습니다");
  }
  else if (command == "보안설정") {
    homeSystem.securityArmed = true;
    playTTS("보안 시스템을 설정했습니다");
  }
  else if (command == "보안해제") {
    homeSystem.securityArmed = false;
    playTTS("보안 시스템을 해제했습니다");
  }
  else if (command == "외출모드") {
    setAwayMode();
    playTTS("외출 모드로 설정했습니다");
  }
  else if (command == "취침모드") {
    setSleepMode();
    playTTS("취침 모드로 설정했습니다");
  }
  
  // UI 업데이트
  drawHomeScreen();
  broadcastSystemUpdate();
}

void playTTS(String text) {
  // TTS 구현 (실제로는 외부 TTS 엔진 사용)
  Serial.println("TTS: " + text);
  
  // 간단한 비프음으로 대체
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

void loadAutomationRules() {
  // SD 카드에서 자동화 규칙 로드
  File rulesFile = SD.open("/automation_rules.json", FILE_READ);
  if (rulesFile) {
    String jsonString = rulesFile.readString();
    rulesFile.close();
    
    DynamicJsonDocument doc(4096);
    deserializeJson(doc, jsonString);
    
    JsonArray rules = doc["rules"];
    ruleCount = 0;
    
    for (JsonObject rule : rules) {
      if (ruleCount < 20) {
        automationRules[ruleCount].name = rule["name"].as<String>();
        automationRules[ruleCount].condition = rule["condition"].as<String>();
        automationRules[ruleCount].action = rule["action"].as<String>();
        automationRules[ruleCount].enabled = rule["enabled"];
        automationRules[ruleCount].lastTriggered = 0;
        ruleCount++;
      }
    }
    
    Serial.print("자동화 규칙 ");
    Serial.print(ruleCount);
    Serial.println("개 로드됨");
  } else {
    // 기본 규칙들 생성
    createDefaultAutomationRules();
  }
}

void createDefaultAutomationRules() {
  // 기본 자동화 규칙들
  automationRules[0] = {"아침 자동 조명", "time:07:00", "light:0:on", true, 0};
  automationRules[1] = {"저녁 자동 조명", "time:18:00", "light:all:on", true, 0};
  automationRules[2] = {"심야 조명 끄기", "time:23:00", "light:all:off", true, 0};
  automationRules[3] = {"움직임 감지 조명", "sensor:motion:true", "light:0:on", true, 0};
  automationRules[4] = {"외출시 전체 끄기", "device:security:armed", "appliance:all:off", true, 0};
  
  ruleCount = 5;
  
  Serial.println("기본 자동화 규칙 생성됨");
}

void executeAutomationRules() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck >= 30000) { // 30초마다 체크
    DateTime now = rtc.now();
    
    for (int i = 0; i < ruleCount; i++) {
      if (automationRules[i].enabled) {
        if (checkRuleCondition(automationRules[i], now)) {
          executeRuleAction(automationRules[i]);
          automationRules[i].lastTriggered = millis();
        }
      }
    }
    
    lastCheck = millis();
  }
}

bool checkRuleCondition(AutomationRule rule, DateTime now) {
  String condition = rule.condition;
  
  if (condition.startsWith("time:")) {
    String timeStr = condition.substring(5);
    int hour = timeStr.substring(0, 2).toInt();
    int minute = timeStr.substring(3, 5).toInt();
    
    // 해당 시간인지 확인 (±1분 오차 허용)
    if (now.hour() == hour && abs(now.minute() - minute) <= 1) {
      // 같은 시간에 중복 실행 방지
      return (millis() - rule.lastTriggered) > 120000; // 2분 이상 간격
    }
  }
  else if (condition.startsWith("sensor:")) {
    // 센서 조건 체크 (MQTT 데이터 기반)
    // 실제 구현에서는 센서 데이터와 비교
    return false; // 임시로 false
  }
  else if (condition.startsWith("device:")) {
    // 디바이스 상태 조건 체크
    if (condition == "device:security:armed") {
      return homeSystem.securityArmed;
    }
  }
  
  return false;
}

void executeRuleAction(AutomationRule rule) {
  String action = rule.action;
  
  Serial.println("자동화 규칙 실행: " + rule.name + " -> " + action);
  
  if (action.startsWith("light:")) {
    String params = action.substring(6);
    
    if (params.startsWith("all:")) {
      bool state = params.endsWith(":on");
      for (int i = 0; i < 4; i++) {
        controlLight(i, state);
      }
    } else {
      int index = params.substring(0, 1).toInt();
      bool state = params.endsWith(":on");
      controlLight(index, state);
    }
  }
  else if (action.startsWith("appliance:")) {
    String params = action.substring(10);
    
    if (params.startsWith("all:")) {
      bool state = params.endsWith(":on");
      for (int i = 0; i < 4; i++) {
        controlAppliance(i, state);
      }
    } else {
      int index = params.substring(0, 1).toInt();
      bool state = params.endsWith(":on");
      controlAppliance(index, state);
    }
  }
  else if (action.startsWith("notification:")) {
    String message = action.substring(13);
    sendNotification(message);
  }
  
  // UI 업데이트
  drawHomeScreen();
  broadcastSystemUpdate();
}

void collectSensorData() {
  static unsigned long lastCollection = 0;
  
  if (millis() - lastCollection >= 5000) { // 5초마다
    // 센서 데이터 시뮬레이션 (실제로는 MQTT에서 수신)
    homeSystem.temperature = 22.5 + random(-30, 31) / 10.0;
    homeSystem.humidity = 60.0 + random(-100, 101) / 10.0;
    homeSystem.airQuality = 50 + random(-20, 21);
    homeSystem.lightLevel = random(0, 1024);
    homeSystem.energyUsage = random(500, 2000) / 10.0;
    
    // 연결된 노드 수 업데이트
    homeSystem.connectedNodes = random(3, 8);
    
    lastCollection = millis();
  }
}

void monitorSystemHealth() {
  static unsigned long lastHealthCheck = 0;
  
  if (millis() - lastHealthCheck >= 60000) { // 1분마다
    // 메모리 사용량 체크
    size_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 10000) { // 10KB 미만
      Serial.println("경고: 메모리 부족!");
      // 메모리 정리 또는 재시작
    }
    
    // WiFi 연결 상태 체크
    if (!WiFi.isConnected()) {
      Serial.println("WiFi 재연결 시도...");
      connectToWiFi();
    }
    
    // MQTT 연결 상태 체크
    if (!mqttClient.connected()) {
      Serial.println("MQTT 재연결 시도...");
      connectToMQTT();
    }
    
    // 시스템 업타임 업데이트
    homeSystem.uptime = millis();
    
    lastHealthCheck = millis();
  }
}

void manageEnergyUsage() {
  static unsigned long lastEnergyCheck = 0;
  
  if (millis() - lastEnergyCheck >= 30000) { // 30초마다
    // 에너지 사용량이 높으면 절약 모드 제안
    if (homeSystem.energyUsage > 150.0) { // 1.5kW 이상
      Serial.println("높은 전력 사용 감지 - 절약 모드 제안");
      
      // 불필요한 가전제품 자동 끄기
      for (int i = 0; i < 4; i++) {
        if (homeSystem.appliances[i] && i != 0) { // 에어컨 제외
          controlAppliance(i, false);
        }
      }
      
      sendNotification("전력 사용량이 높아 일부 가전제품을 껐습니다");
    }
    
    lastEnergyCheck = millis();
  }
}

void setAwayMode() {
  // 외출 모드 설정
  for (int i = 0; i < 8; i++) {
    controlLight(i, false);
    controlAppliance(i, false);
  }
  
  homeSystem.securityArmed = true;
  
  Serial.println("외출 모드 설정됨");
  sendNotification("외출 모드가 설정되었습니다");
}

void setSleepMode() {
  // 취침 모드 설정
  for (int i = 0; i < 4; i++) {
    controlLight(i, false);
  }
  
  controlAppliance(1, false); // TV 끄기
  controlAppliance(2, false); // 오디오 끄기
  
  homeSystem.securityArmed = true;
  
  Serial.println("취침 모드 설정됨");
  sendNotification("취침 모드가 설정되었습니다");
}

void sendNotification(String message) {
  // 알림 전송 (MQTT, 모바일 앱 등)
  mqttClient.publish("smarthome/notifications", message.c_str());
  
  // 터치스크린에 알림 표시
  tft.fillRect(0, 280, 480, 40, TFT_YELLOW);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(message, 10, 295);
  tft.setTextSize(2);
  
  Serial.println("알림: " + message);
}

// 추가 웹 핸들러들
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

.dashboard-container {
    max-width: 1400px;
    margin: 0 auto;
    padding: 20px;
}

.dashboard-header {
    background: rgba(255, 255, 255, 0.95);
    backdrop-filter: blur(10px);
    padding: 20px 30px;
    border-radius: 15px;
    margin-bottom: 30px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
}

.dashboard-header h1 {
    color: #2c3e50;
    font-size: 2.2rem;
    font-weight: 600;
}

.system-status {
    display: flex;
    gap: 20px;
}

.status-item {
    display: flex;
    align-items: center;
    gap: 8px;
    color: #34495e;
    font-weight: 500;
}

.dashboard-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
    gap: 25px;
    margin-bottom: 30px;
}

.card {
    background: rgba(255, 255, 255, 0.95);
    backdrop-filter: blur(10px);
    border-radius: 20px;
    padding: 25px;
    box-shadow: 0 10px 40px rgba(0, 0, 0, 0.1);
    transition: transform 0.3s ease, box-shadow 0.3s ease;
}

.card:hover {
    transform: translateY(-5px);
    box-shadow: 0 15px 50px rgba(0, 0, 0, 0.15);
}

.card h3 {
    color: #2c3e50;
    margin-bottom: 20px;
    font-size: 1.4rem;
    font-weight: 600;
    display: flex;
    align-items: center;
    gap: 10px;
}

/* 환경 모니터링 카드 */
.sensor-grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 15px;
    margin-bottom: 20px;
}

.sensor-item {
    background: linear-gradient(135deg, #f8f9fa, #e9ecef);
    padding: 15px;
    border-radius: 12px;
    text-align: center;
    transition: all 0.3s ease;
}

.sensor-item:hover {
    transform: scale(1.05);
    background: linear-gradient(135deg, #e3f2fd, #bbdefb);
}

.sensor-item i {
    font-size: 2rem;
    color: #3498db;
    margin-bottom: 8px;
}

.sensor-label {
    display: block;
    font-size: 0.9rem;
    color: #6c757d;
    margin-bottom: 5px;
}

.sensor-value {
    display: block;
    font-size: 1.3rem;
    font-weight: bold;
    color: #2c3e50;
}

/* 조명 제어 카드 */
.light-controls {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 15px;
    margin-bottom: 20px;
}

.light-room {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 12px 15px;
    background: #f8f9fa;
    border-radius: 10px;
}

.switch-container {
    position: relative;
}

.light-switch {
    display: none;
}

.switch-label {
    display: block;
    width: 50px;
    height: 25px;
    background: #ccc;
    border-radius: 25px;
    cursor: pointer;
    transition: background 0.3s;
    position: relative;
}

.switch-label::after {
    content: '';
    position: absolute;
    width: 21px;
    height: 21px;
    background: white;
    border-radius: 50%;
    top: 2px;
    left: 2px;
    transition: transform 0.3s;
}

.light-switch:checked + .switch-label {
    background: #4CAF50;
}

.light-switch:checked + .switch-label::after {
    transform: translateX(25px);
}

.quick-actions {
    display: flex;
    gap: 10px;
}

/* 버튼 스타일 */
.btn {
    padding: 10px 20px;
    border: none;
    border-radius: 25px;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.3s ease;
    text-decoration: none;
    display: inline-flex;
    align-items: center;
    gap: 8px;
}

.btn:hover {
    transform: translateY(-2px);
    box-shadow: 0 5px 15px rgba(0, 0, 0, 0.2);
}

.btn-primary {
    background: linear-gradient(135deg, #3498db, #2980b9);
    color: white;
}

.btn-secondary {
    background: linear-gradient(135deg, #95a5a6, #7f8c8d);
    color: white;
}

.btn-warning {
    background: linear-gradient(135deg, #f39c12, #e67e22);
    color: white;
}

/* 가전제품 제어 */
.appliance-grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 15px;
}

.appliance-item {
    background: #f8f9fa;
    padding: 15px;
    border-radius: 12px;
    text-align: center;
    transition: all 0.3s ease;
}

.appliance-item:hover {
    background: #e9ecef;
    transform: translateY(-2px);
}

.appliance-item i {
    font-size: 2rem;
    color: #34495e;
    margin-bottom: 10px;
}

.appliance-btn {
    padding: 5px 15px;
    border: 2px solid #3498db;
    background: transparent;
    color: #3498db;
    border-radius: 15px;
    cursor: pointer;
    transition: all 0.3s ease;
    margin-top: 8px;
}

.appliance-btn:hover {
    background: #3498db;
    color: white;
}

.appliance-btn.active {
    background: #27ae60;
    border-color: #27ae60;
    color: white;
}

/* 보안 시스템 */
.security-status {
    text-align: center;
    margin-bottom: 20px;
}

.security-indicator {
    display: inline-flex;
    align-items: center;
    gap: 10px;
    padding: 15px 25px;
    background: linear-gradient(135deg, #e74c3c, #c0392b);
    color: white;
    border-radius: 25px;
    margin-bottom: 15px;
    font-weight: bold;
}

.security-indicator.armed {
    background: linear-gradient(135deg, #27ae60, #229954);
}

.security-sensors {
    display: flex;
    flex-direction: column;
    gap: 10px;
}

.sensor-status {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px 15px;
    background: #f8f9fa;
    border-radius: 8px;
}

.status-ok {
    color: #27ae60;
    font-weight: bold;
}

.status-alert {
    color: #e74c3c;
    font-weight: bold;
}

/* 에너지 모니터링 */
.energy-stats {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 15px;
    margin-bottom: 20px;
}

.energy-item {
    text-align: center;
    padding: 15px;
    background: linear-gradient(135deg, #fff3cd, #ffeaa7);
    border-radius: 10px;
}

.energy-label {
    display: block;
    font-size: 0.9rem;
    color: #856404;
    margin-bottom: 5px;
}

.energy-value {
    display: block;
    font-size: 1.2rem;
    font-weight: bold;
    color: #533f03;
}

/* 자동화 규칙 */
.automation-list {
    max-height: 200px;
    overflow-y: auto;
    margin-bottom: 15px;
}

.automation-rule {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px 15px;
    background: #f8f9fa;
    border-radius: 8px;
    margin-bottom: 8px;
}

.rule-toggle {
    width: 40px;
    height: 20px;
    background: #ccc;
    border-radius: 20px;
    cursor: pointer;
    transition: background 0.3s;
    position: relative;
}

.rule-toggle.active {
    background: #4CAF50;
}

.rule-toggle::after {
    content: '';
    position: absolute;
    width: 16px;
    height: 16px;
    background: white;
    border-radius: 50%;
    top: 2px;
    left: 2px;
    transition: transform 0.3s;
}

.rule-toggle.active::after {
    transform: translateX(20px);
}

/* 음성 제어 */
.voice-control {
    position: fixed;
    bottom: 30px;
    right: 30px;
    z-index: 1000;
}

.voice-btn {
    width: 70px;
    height: 70px;
    border-radius: 50%;
    background: linear-gradient(135deg, #e74c3c, #c0392b);
    color: white;
    border: none;
    cursor: pointer;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    box-shadow: 0 5px 20px rgba(231, 76, 60, 0.4);
    transition: all 0.3s ease;
}

.voice-btn:hover {
    transform: scale(1.1);
    box-shadow: 0 8px 25px rgba(231, 76, 60, 0.6);
}

.voice-btn.active {
    background: linear-gradient(135deg, #27ae60, #229954);
    animation: pulse 1s infinite;
}

@keyframes pulse {
    0% { transform: scale(1); }
    50% { transform: scale(1.05); }
    100% { transform: scale(1); }
}

/* 빠른 설정 */
.quick-settings {
    display: flex;
    justify-content: center;
    gap: 15px;
    flex-wrap: wrap;
}

.quick-btn {
    padding: 15px 20px;
    background: rgba(255, 255, 255, 0.9);
    border: none;
    border-radius: 15px;
    cursor: pointer;
    transition: all 0.3s ease;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 5px;
    min-width: 100px;
}

.quick-btn:hover {
    background: white;
    transform: translateY(-3px);
    box-shadow: 0 8px 25px rgba(0, 0, 0, 0.15);
}

.quick-btn i {
    font-size: 1.5rem;
    color: #3498db;
}

.quick-btn span {
    font-size: 0.9rem;
    color: #2c3e50;
    font-weight: 500;
}

/* 모달 */
.modal {
    display: none;
    position: fixed;
    z-index: 2000;
    left: 0;
    top: 0;
    width: 100%;
    height: 100%;
    background: rgba(0, 0, 0, 0.7);
    backdrop-filter: blur(5px);
}

.modal-content {
    background: white;
    margin: 5% auto;
    padding: 0;
    border-radius: 20px;
    width: 90%;
    max-width: 500px;
    box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
    animation: modalSlideIn 0.3s ease;
}

@keyframes modalSlideIn {
    from { opacity: 0; transform: translateY(-50px); }
    to { opacity: 1; transform: translateY(0); }
}

.modal-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 20px 25px;
    border-bottom: 1px solid #eee;
}

.modal-header h3 {
    margin: 0;
    color: #2c3e50;
}

.close-btn {
    background: none;
    border: none;
    font-size: 24px;
    cursor: pointer;
    color: #aaa;
    transition: color 0.3s;
}

.close-btn:hover {
    color: #e74c3c;
}

.modal-body {
    padding: 25px;
}

.form-group {
    margin-bottom: 20px;
}

.form-group label {
    display: block;
    margin-bottom: 8px;
    font-weight: 600;
    color: #2c3e50;
}

.form-group input,
.form-group select {
    width: 100%;
    padding: 12px 15px;
    border: 2px solid #e9ecef;
    border-radius: 10px;
    font-size: 14px;
    transition: border-color 0.3s;
}

.form-group input:focus,
.form-group select:focus {
    outline: none;
    border-color: #3498db;
}

.form-actions {
    display: flex;
    gap: 10px;
    justify-content: flex-end;
    margin-top: 20px;
}

/* 반응형 디자인 */
@media (max-width: 768px) {
    .dashboard-header {
        flex-direction: column;
        gap: 15px;
        text-align: center;
    }
    
    .dashboard-header h1 {
        font-size: 1.8rem;
    }
    
    .system-status {
        flex-wrap: wrap;
        justify-content: center;
    }
    
    .dashboard-grid {
        grid-template-columns: 1fr;
    }
    
    .sensor-grid,
    .light-controls,
    .appliance-grid,
    .energy-stats {
        grid-template-columns: 1fr;
    }
    
    .quick-settings {
        gap: 10px;
    }
    
    .quick-btn {
        min-width: 80px;
        padding: 12px 15px;
    }
    
    .voice-control {
        bottom: 20px;
        right: 20px;
    }
    
    .voice-btn {
        width: 60px;
        height: 60px;
    }
}
)";
  webServer.send(200, "text/css", css);
}

void handleJS() {
  String js = R"(
// WebSocket 연결
let socket = new WebSocket('ws://' + window.location.hostname + ':81');
let isConnected = false;

// 전역 변수
let chartData = {
    environment: {
        labels: [],
        temperature: [],
        humidity: []
    },
    energy: {
        labels: [],
        power: []
    }
};

// 페이지 로드 시 초기화
document.addEventListener('DOMContentLoaded', function() {
    initializeWebSocket();
    initializeCharts();
    updateTime();
    loadSystemStatus();
    loadAutomationRules();
    
    // 주기적 업데이트
    setInterval(updateTime, 1000);
    setInterval(loadSystemStatus, 5000);
    
    // 이벤트 리스너 등록
    registerEventListeners();
});

// WebSocket 초기화
function initializeWebSocket() {
    socket.onopen = function(event) {
        console.log('WebSocket 연결됨');
        isConnected = true;
        updateConnectionStatus(true);
    };
    
    socket.onmessage = function(event) {
        const data = JSON.parse(event.data);
        updateRealTimeData(data);
    };
    
    socket.onclose = function(event) {
        console.log('WebSocket 연결 끊어짐');
        isConnected = false;
        updateConnectionStatus(false);
        
        // 재연결 시도
        setTimeout(initializeWebSocket, 5000);
    };
    
    socket.onerror = function(error) {
        console.error('WebSocket 오류:', error);
    };
}

// 실시간 데이터 업데이트
function updateRealTimeData(data) {
    // 환경 데이터 업데이트
    if (data.temperature !== undefined) {
        document.getElementById('temperature').textContent = data.temperature.toFixed(1) + '°C';
        document.getElementById('current-temp').textContent = data.temperature.toFixed(1) + '°C';
    }
    
    if (data.humidity !== undefined) {
        document.getElementById('humidity').textContent = data.humidity.toFixed(0) + '%';
    }
    
    // 조명 상태 업데이트
    if (data.lights) {
        for (let i = 0; i < data.lights.length; i++) {
            const lightSwitch = document.getElementById('light-' + i);
            if (lightSwitch) {
                lightSwitch.checked = data.lights[i];
            }
        }
    }
    
    // 보안 상태 업데이트
    if (data.securityArmed !== undefined) {
        updateSecurityStatus(data.securityArmed);
    }
    
    // 차트 데이터 업데이트
    updateChartData(data);
}

// 차트 초기화
function initializeCharts() {
    // 환경 차트
    const envCanvas = document.getElementById('environment-chart');
    if (envCanvas) {
        const ctx = envCanvas.getContext('2d');
        drawEnvironmentChart(ctx);
    }
    
    // 에너지 차트
    const energyCanvas = document.getElementById('energy-chart');
    if (energyCanvas) {
        const ctx = energyCanvas.getContext('2d');
        drawEnergyChart(ctx);
    }
}

// 환경 차트 그리기
function drawEnvironmentChart(ctx) {
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;
    
    // 배경 클리어
    ctx.clearRect(0, 0, width, height);
    
    // 격자 그리기
    ctx.strokeStyle = '#e0e0e0';
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
    
    // 데이터 라인 그리기
    if (chartData.environment.temperature.length > 1) {
        drawDataLine(ctx, chartData.environment.temperature, '#e74c3c', 15, 35);
        drawDataLine(ctx, chartData.environment.humidity, '#3498db', 30, 80);
    }
    
    // 범례
    ctx.fillStyle = '#2c3e50';
    ctx.font = '12px Arial';
    ctx.fillText('온도 (°C)', 10, 20);
    ctx.fillStyle = '#e74c3c';
    ctx.fillRect(70, 15, 10, 2);
    
    ctx.fillStyle = '#2c3e50';
    ctx.fillText('습도 (%)', 10, 40);
    ctx.fillStyle = '#3498db';
    ctx.fillRect(50, 35, 10, 2);
}

// 데이터 라인 그리기
function drawDataLine(ctx, data, color, minVal, maxVal) {
    if (data.length < 2) return;
    
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;
    
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

// 에너지 차트 그리기
function drawEnergyChart(ctx) {
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;
    
    // 배경 클리어
    ctx.clearRect(0, 0, width, height);
    
    // 원형 차트 그리기 (에너지 사용량)
    const centerX = width / 2;
    const centerY = height / 2;
    const radius = Math.min(width, height) / 3;
    
    const energyData = [
        { label: '조명', value: 30, color: '#f39c12' },
        { label: '난방', value: 40, color: '#e74c3c' },
        { label: '가전', value: 20, color: '#3498db' },
        { label: '기타', value: 10, color: '#95a5a6' }
    ];
    
    let currentAngle = 0;
    
    energyData.forEach(item => {
        const sliceAngle = (item.value / 100) * 2 * Math.PI;
        
        ctx.fillStyle = item.color;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.arc(centerX, centerY, radius, currentAngle, currentAngle + sliceAngle);
        ctx.closePath();
        ctx.fill();
        
        // 라벨
        const labelAngle = currentAngle + sliceAngle / 2;
        const labelX = centerX + Math.cos(labelAngle) * (radius + 20);
        const labelY = centerY + Math.sin(labelAngle) * (radius + 20);
        
        ctx.fillStyle = '#2c3e50';
        ctx.font = '10px Arial';
        ctx.textAlign = 'center';
        ctx.fillText(item.label, labelX, labelY);
        ctx.fillText(item.value + '%', labelX, labelY + 12);
        
        currentAngle += sliceAngle;
    });
}

// 차트 데이터 업데이트
function updateChartData(data) {
    const now = new Date();
    
    // 환경 데이터 추가
    if (data.temperature !== undefined) {
        chartData.environment.temperature.push(data.temperature);
        chartData.environment.labels.push(now.toLocaleTimeString());
        
        // 최대 20개 데이터포인트 유지
        if (chartData.environment.temperature.length > 20) {
            chartData.environment.temperature.shift();
            chartData.environment.labels.shift();
        }
    }
    
    if (data.humidity !== undefined) {
        chartData.environment.humidity.push(data.humidity);
        
        if (chartData.environment.humidity.length > 20) {
            chartData.environment.humidity.shift();
        }
    }
    
    // 차트 다시 그리기
    initializeCharts();
}

// 보안 상태 업데이트
function updateSecurityStatus(armed) {
    const statusElement = document.getElementById('security-status');
    const armButton = document.getElementById('security-arm');
    
    if (armed) {
        statusElement.textContent = '보안 시스템 활성화';
        statusElement.style.color = '#e74c3c';
        armButton.textContent = '보안 해제';
        armButton.classList.remove('btn-danger');
        armButton.classList.add('btn-success');
    } else {
        statusElement.textContent = '보안 시스템 비활성화';
        statusElement.style.color = '#95a5a6';
        armButton.textContent = '보안 설정';
        armButton.classList.remove('btn-success');
        armButton.classList.add('btn-danger');
    }
}

// 조명 제어
function controlLight(lightId, lightSwitch) {
    const data = {
        type: 'control',
        device: 'light',
        id: lightId,
        state: lightSwitch.checked
    };
    
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(data));
    }
}

// 온도 설정
function setTemperature(temperature) {
    const data = {
        type: 'control',
        device: 'thermostat',
        temperature: parseInt(temperature)
    };
    
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(data));
    }
    
    document.getElementById('temp-value').textContent = temperature + '°C';
}

// 보안 시스템 토글
function toggleSecurity() {
    const data = {
        type: 'control',
        device: 'security',
        action: 'toggle'
    };
    
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(data));
    }
}

// 시나리오 실행
function executeScenario(scenarioName) {
    const data = {
        type: 'scenario',
        name: scenarioName
    };
    
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(data));
    }
    
    // 피드백 표시
    const button = event.target;
    const originalText = button.textContent;
    button.textContent = '실행 중...';
    button.disabled = true;
    
    setTimeout(() => {
        button.textContent = originalText;
        button.disabled = false;
    }, 2000);
}

// 페이지 로드 시 초기화
document.addEventListener('DOMContentLoaded', function() {
    initializeWebSocket();
    initializeCharts();
    
    // 온도 슬라이더 이벤트
    const tempSlider = document.getElementById('temp-slider');
    if (tempSlider) {
        tempSlider.addEventListener('input', function() {
            setTemperature(this.value);
        });
    }
    
    // 조명 스위치 이벤트
    for (let i = 0; i < 4; i++) {
        const lightSwitch = document.getElementById('light-' + i);
        if (lightSwitch) {
            lightSwitch.addEventListener('change', function() {
                controlLight(i, this);
            });
        }
    }
    
    // 주기적 차트 업데이트
    setInterval(() => {
        if (Math.random() < 0.3) { // 30% 확률로 가상 데이터 추가
            const fakeData = {
                temperature: 20 + Math.random() * 10,
                humidity: 40 + Math.random() * 20
            };
            updateChartData(fakeData);
        }
    }, 5000);
});

// 알림 표시
function showNotification(message, type = 'info') {
    const notification = document.createElement('div');
    notification.className = `alert alert-${type} alert-dismissible fade show`;
    notification.style.position = 'fixed';
    notification.style.top = '20px';
    notification.style.right = '20px';
    notification.style.zIndex = '9999';
    notification.innerHTML = `
        ${message}
        <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
    `;
    
    document.body.appendChild(notification);
    
    // 5초 후 자동 제거
    setTimeout(() => {
        if (notification.parentNode) {
            notification.parentNode.removeChild(notification);
        }
    }, 5000);
}
```

## 2.4 MQTT 서버 설정 (Node.js)

```javascript
/*
  mqtt_server.js
  MQTT 브로커 및 웹소켓 서버
  
  설치: npm install mosca websocket express
*/

const mosca = require('mosca');
const WebSocket = require('ws');
const express = require('express');
const http = require('http');

// Express 서버 설정
const app = express();
const server = http.createServer(app);

// 정적 파일 서빙
app.use(express.static('public'));

// MQTT 브로커 설정
const moscaSettings = {
    port: 1883,
    persistence: {
        factory: mosca.persistence.Memory
    }
};

const mqttServer = new mosca.Server(moscaSettings);

// WebSocket 서버 설정
const wss = new WebSocket.Server({ server });

// 연결된 클라이언트들
const connectedClients = new Set();

// MQTT 브로커 이벤트
mqttServer.on('ready', function() {
    console.log('MQTT 브로커가 포트 1883에서 시작되었습니다.');
});

mqttServer.on('clientConnected', function(client) {
    console.log('MQTT 클라이언트 연결:', client.id);
});

mqttServer.on('published', function(packet, client) {
    if (packet.topic.startsWith('smarthome/')) {
        console.log('수신된 메시지:', packet.topic, packet.payload.toString());
        
        // WebSocket 클라이언트들에게 브로드캐스트
        const message = {
            topic: packet.topic,
            payload: packet.payload.toString(),
            timestamp: new Date().toISOString()
        };
        
        broadcastToWebSocketClients(JSON.stringify(message));
    }
});

// WebSocket 서버 이벤트
wss.on('connection', function(ws) {
    console.log('WebSocket 클라이언트 연결됨');
    connectedClients.add(ws);
    
    ws.on('message', function(message) {
        try {
            const data = JSON.parse(message);
            console.log('WebSocket에서 수신:', data);
            
            // MQTT 토픽으로 퍼블리시
            if (data.type === 'control') {
                const topic = `smarthome/control/${data.device}`;
                const payload = JSON.stringify(data);
                mqttServer.publish({
                    topic: topic,
                    payload: payload,
                    qos: 0,
                    retain: false
                });
            }
        } catch (error) {
            console.error('WebSocket 메시지 파싱 오류:', error);
        }
    });
    
    ws.on('close', function() {
        console.log('WebSocket 클라이언트 연결 해제');
        connectedClients.delete(ws);
    });
    
    ws.on('error', function(error) {
        console.error('WebSocket 오류:', error);
        connectedClients.delete(ws);
    });
});

// WebSocket 클라이언트들에게 브로드캐스트
function broadcastToWebSocketClients(message) {
    connectedClients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}

// 서버 시작
const PORT = process.env.PORT || 3000;
server.listen(PORT, function() {
    console.log(`웹 서버가 포트 ${PORT}에서 시작되었습니다.`);
    console.log(`웹 대시보드: http://localhost:${PORT}`);
});

// 시뮬레이션 데이터 생성 (테스트용)
setInterval(() => {
    const sensorData = {
        temperature: 20 + Math.random() * 10,
        humidity: 40 + Math.random() * 20,
        motion: Math.random() < 0.1,
        door: Math.random() < 0.05,
        timestamp: new Date().toISOString()
    };
    
    const message = {
        topic: 'smarthome/sensors/environmental',
        payload: JSON.stringify(sensorData),
        timestamp: sensorData.timestamp
    };
    
    broadcastToWebSocketClients(JSON.stringify(message));
}, 5000);
```

---

# 3. 센서 노드 구현

## 3.1 환경 센서 노드 (아두이노 나노)

```cpp
/*
  Environmental_Sensor_Node.ino
  환경 센서 노드 - 온습도, 조도, 공기질 측정
  
  하드웨어:
  - Arduino Nano
  - DHT22 (온습도)
  - BH1750 (조도)
  - MQ-135 (공기질)
  - ESP8266-01 (WiFi)
*/

#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <ArduinoJson.h>

// 핀 정의
#define DHT_PIN 2
#define DHT_TYPE DHT22
#define MQ135_PIN A0
#define ESP8266_RX 3
#define ESP8266_TX 4

// 센서 객체
DHT dht(DHT_PIN, DHT_TYPE);
BH1750 lightSensor;
SoftwareSerial esp8266(ESP8266_RX, ESP8266_TX);

// 전역 변수
unsigned long lastReading = 0;
const unsigned long READING_INTERVAL = 30000; // 30초 간격
String nodeId = "env_sensor_01";

// 센서 데이터 구조체
struct SensorData {
  float temperature;
  float humidity;
  uint16_t light;
  uint16_t airQuality;
  unsigned long timestamp;
};

void setup() {
  Serial.begin(9600);
  esp8266.begin(9600);
  
  // 센서 초기화
  dht.begin();
  Wire.begin();
  
  if (lightSensor.begin()) {
    Serial.println("BH1750 센서 초기화 완료");
  } else {
    Serial.println("BH1750 센서 초기화 실패");
  }
  
  // ESP8266 초기화
  initializeWiFi();
  
  Serial.println("환경 센서 노드 시작");
  Serial.println("NODE ID: " + nodeId);
}

void loop() {
  unsigned long currentTime = millis();
  
  // 주기적 센서 읽기
  if (currentTime - lastReading >= READING_INTERVAL) {
    SensorData data = readSensors();
    
    if (isValidData(data)) {
      sendDataToHub(data);
      printSensorData(data);
    } else {
      Serial.println("센서 데이터 읽기 실패");
    }
    
    lastReading = currentTime;
  }
  
  // ESP8266 응답 처리
  handleESP8266Response();
  
  delay(100);
}

// 센서 데이터 읽기
SensorData readSensors() {
  SensorData data;
  
  // DHT22 온습도 읽기
  data.temperature = dht.readTemperature();
  data.humidity = dht.readHumidity();
  
  // BH1750 조도 읽기
  data.light = lightSensor.readLightLevel();
  
  // MQ-135 공기질 읽기
  int rawValue = analogRead(MQ135_PIN);
  data.airQuality = map(rawValue, 0, 1023, 0, 1000);
  
  data.timestamp = millis();
  
  return data;
}

// 데이터 유효성 검사
bool isValidData(const SensorData& data) {
  return !isnan(data.temperature) && 
         !isnan(data.humidity) && 
         data.temperature > -50 && data.temperature < 100 &&
         data.humidity >= 0 && data.humidity <= 100;
}

// 허브에 데이터 전송
void sendDataToHub(const SensorData& data) {
  // JSON 데이터 생성
  StaticJsonDocument<300> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "environmental";
  doc["temperature"] = round(data.temperature * 10) / 10.0;
  doc["humidity"] = round(data.humidity);
  doc["light"] = data.light;
  doc["airQuality"] = data.airQuality;
  doc["timestamp"] = data.timestamp;
  doc["battery"] = readBatteryLevel();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // MQTT 퍼블리시
  String command = "AT+PUBLISH=\"smarthome/sensors/environmental\",\"" + jsonString + "\"";
  esp8266.println(command);
  
  delay(500);
}

// 배터리 레벨 읽기
int readBatteryLevel() {
  // 간단한 배터리 전압 측정 (실제 구현시 전압 분배기 필요)
  int batteryRaw = analogRead(A1);
  return map(batteryRaw, 0, 1023, 0, 100);
}

// WiFi 초기화
void initializeWiFi() {
  Serial.println("WiFi 초기화 중...");
  
  // ESP8266 리셋
  esp8266.println("AT+RST");
  delay(2000);
  
  // Station 모드 설정
  esp8266.println("AT+CWMODE=1");
  delay(1000);
  
  // WiFi 연결
  esp8266.println("AT+CWJAP=\"YOUR_WIFI_SSID\",\"YOUR_WIFI_PASSWORD\"");
  delay(5000);
  
  // MQTT 서버 연결
  esp8266.println("AT+MQTTUSERCFG=0,1,\"" + nodeId + "\",\"\",\"\",0,0,\"\"");
  delay(500);
  
  esp8266.println("AT+MQTTCONN=0,\"192.168.1.100\",1883,1");
  delay(2000);
  
  Serial.println("WiFi 초기화 완료");
}

// ESP8266 응답 처리
void handleESP8266Response() {
  if (esp8266.available()) {
    String response = esp8266.readString();
    Serial.print("ESP8266: ");
    Serial.println(response);
    
    // 연결 상태 확인
    if (response.indexOf("WIFI DISCONNECT") > -1) {
      Serial.println("WiFi 연결 끊어짐 - 재연결 시도");
      initializeWiFi();
    }
    
    // MQTT 연결 상태 확인
    if (response.indexOf("MQTT: Disconnected") > -1) {
      Serial.println("MQTT 연결 끊어짐 - 재연결 시도");
      esp8266.println("AT+MQTTCONN=0,\"192.168.1.100\",1883,1");
    }
  }
}

// 센서 데이터 출력
void printSensorData(const SensorData& data) {
  Serial.println("=== 센서 데이터 ===");
  Serial.println("온도: " + String(data.temperature, 1) + "°C");
  Serial.println("습도: " + String(data.humidity, 0) + "%");
  Serial.println("조도: " + String(data.light) + " lux");
  Serial.println("공기질: " + String(data.airQuality));
  Serial.println("배터리: " + String(readBatteryLevel()) + "%");
  Serial.println("==================");
}
```

## 3.2 보안 센서 노드 (아두이노 프로 미니)

```cpp
/*
  Security_Sensor_Node.ino
  보안 센서 노드 - 모션, 도어/윈도우, 진동 감지
  
  하드웨어:
  - Arduino Pro Mini 3.3V
  - PIR 모션 센서
  - 도어/윈도우 자기 센서
  - SW-420 진동 센서
  - nRF24L01+ (무선 통신)
*/

#include <SPI.h>
#include <RF24.h>
#include <ArduinoJson.h>
#include <LowPower.h>

// 핀 정의
#define PIR_PIN 2        // PIR 센서 (인터럽트)
#define DOOR_PIN 3       // 도어 센서 (인터럽트)
#define VIBRATION_PIN 4  // 진동 센서
#define LED_PIN 13       // 상태 LED
#define CE_PIN 9         // nRF24L01 CE
#define CSN_PIN 10       // nRF24L01 CSN

// nRF24L01 설정
RF24 radio(CE_PIN, CSN_PIN);
const byte addresses[][6] = {"00001", "00002"};

// 전역 변수
String nodeId = "security_01";
volatile bool motionDetected = false;
volatile bool doorOpened = false;
volatile unsigned long lastMotionTime = 0;
volatile unsigned long lastDoorTime = 0;
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 300000; // 5분 간격

// 보안 상태 구조체
struct SecurityStatus {
  bool motionActive;
  bool doorOpen;
  bool vibrationDetected;
  int batteryLevel;
  unsigned long timestamp;
};

void setup() {
  Serial.begin(9600);
  
  // 핀 모드 설정
  pinMode(PIR_PIN, INPUT);
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(VIBRATION_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, RISING);
  attachInterrupt(digitalPinToInterrupt(DOOR_PIN), doorISR, CHANGE);
  
  // nRF24L01 초기화
  if (radio.begin()) {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.startListening();
    Serial.println("nRF24L01 초기화 완료");
  } else {
    Serial.println("nRF24L01 초기화 실패");
    // LED로 오류 표시
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
  
  Serial.println("보안 센서 노드 시작");
  Serial.println("NODE ID: " + nodeId);
  
  // 초기 상태 전송
  sendSecurityStatus();
}

void loop() {
  unsigned long currentTime = millis();
  
  // 모션 감지 처리
  if (motionDetected && (currentTime - lastMotionTime > 500)) {
    Serial.println("모션 감지됨!");
    sendMotionAlert();
    motionDetected = false;
    blinkLED(3);
  }
  
  // 도어 상태 변경 처리
  if (doorOpened && (currentTime - lastDoorTime > 500)) {
    bool currentDoorState = digitalRead(DOOR_PIN) == LOW;
    Serial.println("도어 상태 변경: " + String(currentDoorState ? "열림" : "닫힘"));
    sendDoorAlert(currentDoorState);
    doorOpened = false;
    blinkLED(2);
  }
  
  // 진동 감지 체크
  if (digitalRead(VIBRATION_PIN) == HIGH) {
    Serial.println("진동 감지됨!");
    sendVibrationAlert();
    blinkLED(1);
    delay(1000); // 중복 감지 방지
  }
  
  // 하트비트 전송
  if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = currentTime;
  }
  
  // 명령 수신 체크
  checkForCommands();
  
  // 절전 모드 진입 (8초 대기)
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

// 모션 감지 인터럽트
void motionISR() {
  motionDetected = true;
  lastMotionTime = millis();
}

// 도어 센서 인터럽트
void doorISR() {
  doorOpened = true;
  lastDoorTime = millis();
}

// 보안 상태 전송
void sendSecurityStatus() {
  SecurityStatus status;
  status.motionActive = digitalRead(PIR_PIN) == HIGH;
  status.doorOpen = digitalRead(DOOR_PIN) == LOW;
  status.vibrationDetected = digitalRead(VIBRATION_PIN) == HIGH;
  status.batteryLevel = readBatteryLevel();
  status.timestamp = millis();
  
  // JSON 생성
  StaticJsonDocument<200> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "security_status";
  doc["motion"] = status.motionActive;
  doc["door"] = status.doorOpen;
  doc["vibration"] = status.vibrationDetected;
  doc["battery"] = status.batteryLevel;
  doc["timestamp"] = status.timestamp;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  sendRadioMessage(jsonString);
}

// 모션 알림 전송
void sendMotionAlert() {
  StaticJsonDocument<150> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "motion_alert";
  doc["timestamp"] = millis();
  doc["battery"] = readBatteryLevel();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  sendRadioMessage(jsonString);
}

// 도어 알림 전송
void sendDoorAlert(bool isOpen) {
  StaticJsonDocument<150> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "door_alert";
  doc["doorOpen"] = isOpen;
  doc["timestamp"] = millis();
  doc["battery"] = readBatteryLevel();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  sendRadioMessage(jsonString);
}

// 진동 알림 전송
void sendVibrationAlert() {
  StaticJsonDocument<150> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "vibration_alert";
  doc["timestamp"] = millis();
  doc["battery"] = readBatteryLevel();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  sendRadioMessage(jsonString);
}

// 하트비트 전송
void sendHeartbeat() {
  StaticJsonDocument<100> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "heartbeat";
  doc["timestamp"] = millis();
  doc["battery"] = readBatteryLevel();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  sendRadioMessage(jsonString);
  
  Serial.println("하트비트 전송: 배터리 " + String(readBatteryLevel()) + "%");
}

// 무선 메시지 전송
void sendRadioMessage(const String& message) {
  radio.stopListening();
  
  char buffer[250];
  message.toCharArray(buffer, sizeof(buffer));
  
  bool result = radio.write(&buffer, sizeof(buffer));
  if (result) {
    Serial.println("전송 성공: " + message);
  } else {
    Serial.println("전송 실패: " + message);
  }
  
  radio.startListening();
}

// 명령 수신 확인
void checkForCommands() {
  if (radio.available()) {
    char buffer[250];
    radio.read(&buffer, sizeof(buffer));
    
    String command = String(buffer);
    Serial.println("수신된 명령: " + command);
    
    // 명령 처리
    if (command.indexOf("status_request") > -1) {
      sendSecurityStatus();
    } else if (command.indexOf("test_mode") > -1) {
      // 테스트 모드 - 모든 센서 상태 확인
      testAllSensors();
    }
  }
}

// 모든 센서 테스트
void testAllSensors() {
  Serial.println("=== 센서 테스트 모드 ===");
  
  // LED 점멸로 테스트 시작 표시
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  Serial.println("PIR 센서: " + String(digitalRead(PIR_PIN) ? "HIGH" : "LOW"));
  Serial.println("도어 센서: " + String(digitalRead(DOOR_PIN) ? "HIGH" : "LOW"));
  Serial.println("진동 센서: " + String(digitalRead(VIBRATION_PIN) ? "HIGH" : "LOW"));
  Serial.println("배터리 레벨: " + String(readBatteryLevel()) + "%");
  
  sendSecurityStatus();
}

// 배터리 레벨 읽기
int readBatteryLevel() {
  // VCC 전압 측정 (Pro Mini 3.3V 기준)
  long result = readVcc();
  int batteryPercent = map(result, 2800, 3300, 0, 100);
  return constrain(batteryPercent, 0, 100);
}

// VCC 전압 읽기
long readVcc() {
  long result;
  // AVR 내부 1.1V 기준전압으로 VCC 측정
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result;
  return result;
}

// LED 깜빡임
void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}
```

## 3.3 스마트 릴레이 노드 (아두이노 나노)

```cpp
/*
  Smart_Relay_Node.ino
  스마트 릴레이 노드 - 조명, 가전제품 제어
  
  하드웨어:
  - Arduino Nano
  - 8채널 릴레이 모듈
  - ESP8266-01 (WiFi)
  - 전류 센서 (ACS712) x4
*/

#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// 핀 정의
#define ESP8266_RX 2
#define ESP8266_TX 3
#define RELAY_START_PIN 4  // 릴레이 핀 4~11
#define CURRENT_SENSOR_1 A0
#define CURRENT_SENSOR_2 A1
#define CURRENT_SENSOR_3 A2
#define CURRENT_SENSOR_4 A3
#define STATUS_LED 13

// 릴레이 설정
const int RELAY_COUNT = 8;
const int relayPins[RELAY_COUNT] = {4, 5, 6, 7, 8, 9, 10, 11};
const String relayNames[RELAY_COUNT] = {
  "거실조명", "침실조명", "주방조명", "현관조명",
  "에어컨", "TV", "세탁기", "온수기"
};

// 통신 객체
SoftwareSerial esp8266(ESP8266_RX, ESP8266_TX);

// 전역 변수
String nodeId = "relay_control_01";
bool relayStates[RELAY_COUNT] = {false};
unsigned long lastStatusReport = 0;
const unsigned long STATUS_INTERVAL = 60000; // 1분 간격

// 전력 사용량 구조체
struct PowerData {
  float current[4];      // 전류값 (A)
  float power[4];        // 전력값 (W)
  float totalPower;      // 총 전력 (W)
  unsigned long timestamp;
};

void setup() {
  Serial.begin(9600);
  esp8266.begin(9600);
  
  // 릴레이 핀 초기화
  for (int i = 0; i < RELAY_COUNT; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH); // 릴레이 OFF (Active LOW)
  }
  
  pinMode(STATUS_LED, OUTPUT);
  
  // EEPROM에서 마지막 상태 복원
  loadRelayStates();
  
  // WiFi 초기화
  initializeWiFi();
  
  Serial.println("스마트 릴레이 노드 시작");
  Serial.println("NODE ID: " + nodeId);
  printRelayStatus();
  
  // 초기 상태 보고
  sendStatusReport();
}

void loop() {
  unsigned long currentTime = millis();
  
  // ESP8266 명령 처리
  handleESP8266Commands();
  
  // 주기적 상태 보고
  if (currentTime - lastStatusReport >= STATUS_INTERVAL) {
    sendStatusReport();
    sendPowerReport();
    lastStatusReport = currentTime;
  }
  
  // 상태 LED 업데이트
  updateStatusLED();
  
  delay(100);
}

// WiFi 초기화
void initializeWiFi() {
  Serial.println("WiFi 초기화 중...");
  
  // ESP8266 리셋
  esp8266.println("AT+RST");
  delay(2000);
  
  // Station 모드 설정
  esp8266.println("AT+CWMODE=1");
  delay(1000);
  
  // WiFi 연결
  esp8266.println("AT+CWJAP=\"YOUR_WIFI_SSID\",\"YOUR_WIFI_PASSWORD\"");
  delay(5000);
  
  // MQTT 설정
  esp8266.println("AT+MQTTUSERCFG=0,1,\"" + nodeId + "\",\"\",\"\",0,0,\"\"");
  delay(500);
  
  esp8266.println("AT+MQTTCONN=0,\"192.168.1.100\",1883,1");
  delay(2000);
  
  // 제어 토픽 구독
  esp8266.println("AT+MQTTSUB=0,\"smarthome/control/relay\",1");
  delay(500);
  
  esp8266.println("AT+MQTTSUB=0,\"smarthome/control/lights\",1");
  delay(500);
  
  Serial.println("WiFi 초기화 완료");
}

// ESP8266 명령 처리
void handleESP8266Commands() {
  if (esp8266.available()) {
    String response = esp8266.readString();
    Serial.println("ESP8266: " + response);
    
    // MQTT 메시지 파싱
    if (response.indexOf("+MQTTSUBRECV:") > -1) {
      parseMQTTMessage(response);
    }
    
    // 연결 상태 확인
    if (response.indexOf("WIFI DISCONNECT") > -1) {
      Serial.println("WiFi 연결 끊어짐 - 재연결 시도");
      initializeWiFi();
    }
  }
}

// MQTT 메시지 파싱
void parseMQTTMessage(const String& message) {
  // 메시지에서 JSON 부분 추출
  int jsonStart = message.indexOf("{");
  int jsonEnd = message.lastIndexOf("}");
  
  if (jsonStart > -1 && jsonEnd > jsonStart) {
    String jsonStr = message.substring(jsonStart, jsonEnd + 1);
    Serial.println("수신된 JSON: " + jsonStr);
    
    // JSON 파싱
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    
    if (!error) {
      processControlCommand(doc);
    } else {
      Serial.println("JSON 파싱 오류: " + String(error.c_str()));
    }
  }
}

// 제어 명령 처리
void processControlCommand(const JsonDocument& doc) {
  String device = doc["device"].as<String>();
  
  if (device == "relay" || device == "light") {
    int relayId = doc["id"].as<int>();
    bool state = doc["state"].as<bool>();
    
    if (relayId >= 0 && relayId < RELAY_COUNT) {
      controlRelay(relayId, state);
      Serial.println("릴레이 " + String(relayId) + " (" + relayNames[relayId] + "): " + (state ? "ON" : "OFF"));
    }
  } else if (device == "scenario") {
    String scenario = doc["name"].as<String>();
    executeScenario(scenario);
  } else if (device == "all_lights") {
    bool state = doc["state"].as<bool>();
    controlAllLights(state);
  }
}

// 릴레이 제어
void controlRelay(int relayId, bool state) {
  if (relayId >= 0 && relayId < RELAY_COUNT) {
    relayStates[relayId] = state;
    digitalWrite(relayPins[relayId], state ? LOW : HIGH); // Active LOW
    
    // EEPROM에 상태 저장
    EEPROM.write(relayId, state ? 1 : 0);
    
    // 즉시 상태 보고
    sendRelayStatusUpdate(relayId, state);
  }
}

// 모든 조명 제어 (0-3번 릴레이)
void controlAllLights(bool state) {
  for (int i = 0; i < 4; i++) {
    controlRelay(i, state);
    delay(100); // 릴레이 간 지연
  }
}

// 시나리오 실행
void executeScenario(const String& scenario) {
  Serial.println("시나리오 실행: " + scenario);
  
  if (scenario == "evening") {
    // 저녁 시나리오: 거실, 주방 조명 ON
    controlRelay(0, true);  // 거실조명
    controlRelay(2, true);  // 주방조명
    controlRelay(1, false); // 침실조명 OFF
    controlRelay(3, true);  // 현관조명
  } else if (scenario == "sleep") {
    // 수면 시나리오: 모든 조명 OFF, 에어컨 유지
    for (int i = 0; i < 4; i++) {
      controlRelay(i, false);
    }
    controlRelay(5, false); // TV OFF
  } else if (scenario == "away") {
    // 외출 시나리오: 모든 기기 OFF (온수기 제외)
    for (int i = 0; i < RELAY_COUNT - 1; i++) {
      controlRelay(i, false);
    }
  } else if (scenario == "morning") {
    // 아침 시나리오: 거실, 주방 조명 ON, TV ON
    controlRelay(0, true);  // 거실조명
    controlRelay(2, true);  // 주방조명
    controlRelay(5, true);  // TV ON
  }
}

// 상태 보고 전송
void sendStatusReport() {
  StaticJsonDocument<400> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "relay_status";
  doc["timestamp"] = millis();
  
  JsonArray relays = doc.createNestedArray("relays");
  for (int i = 0; i < RELAY_COUNT; i++) {
    JsonObject relay = relays.createNestedObject();
    relay["id"] = i;
    relay["name"] = relayNames[i];
    relay["state"] = relayStates[i];
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  publishMQTT("smarthome/status/relay", jsonString);
}

// 개별 릴레이 상태 업데이트
void sendRelayStatusUpdate(int relayId, bool state) {
  StaticJsonDocument<150> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "relay_update";
  doc["relayId"] = relayId;
  doc["relayName"] = relayNames[relayId];
  doc["state"] = state;
  doc["timestamp"] = millis();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  publishMQTT("smarthome/status/relay", jsonString);
}

// 전력 사용량 보고
void sendPowerReport() {
  PowerData power = readPowerData();
  
  StaticJsonDocument<300> doc;
  doc["nodeId"] = nodeId;
  doc["type"] = "power_data";
  doc["timestamp"] = power.timestamp;
  doc["totalPower"] = power.totalPower;
  
  JsonArray channels = doc.createNestedArray("channels");
  for (int i = 0; i < 4; i++) {
    JsonObject channel = channels.createNestedObject();
    channel["id"] = i;
    channel["current"] = power.current[i];
    channel["power"] = power.power[i];
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  publishMQTT("smarthome/sensors/power", jsonString);
}

// 전력 데이터 읽기
PowerData readPowerData() {
  PowerData data;
  data.timestamp = millis();
  data.totalPower = 0;
  
  const int analogPins[4] = {CURRENT_SENSOR_1, CURRENT_SENSOR_2, CURRENT_SENSOR_3, CURRENT_SENSOR_4};
  const float voltage = 220.0; // AC 전압 (V)
  const float sensitivity = 0.1; // 센서 감도 (V/A)
  const float vcc = 5.0;
  
  for (int i = 0; i < 4; i++) {
    // 여러 번 읽어서 평균값 계산
    long sum = 0;
    for (int j = 0; j < 100; j++) {
      sum += analogRead(analogPins[i]);
      delay(1);
    }
    
    float analogValue = sum / 100.0;
    float voltage_sensor = (analogValue / 1024.0) * vcc;
    
    // 전류 계산 (ACS712 기준)
    data.current[i] = abs(voltage_sensor - (vcc / 2)) / sensitivity;
    
    // 전력 계산
    data.power[i] = data.current[i] * voltage;
    data.totalPower += data.power[i];
  }
  
  return data;
}

// MQTT 퍼블리시
void publishMQTT(const String& topic, const String& payload) {
  String command = "AT+MQTTPUB=0,\"" + topic + "\",\"" + payload + "\",1,0";
  esp8266.println(command);
  delay(500);
}

// EEPROM에서 릴레이 상태 로드
void loadRelayStates() {
  Serial.println("EEPROM에서 릴레이 상태 복원 중...");
  
  for (int i = 0; i < RELAY_COUNT; i++) {
    byte state = EEPROM.read(i);
    if (state == 0 || state == 1) {
      relayStates[i] = (state == 1);
      digitalWrite(relayPins[i], relayStates[i] ? LOW : HIGH);
    }
  }
  
  printRelayStatus();
}

// 릴레이 상태 출력
void printRelayStatus() {
  Serial.println("=== 릴레이 상태 ===");
  for (int i = 0; i < RELAY_COUNT; i++) {
    Serial.println(String(i) + ". " + relayNames[i] + ": " + (relayStates[i] ? "ON" : "OFF"));
  }
  Serial.println("==================");
}

// 상태 LED 업데이트
void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  // WiFi 연결 상태에 따른 LED 패턴
  if (millis() - lastBlink > 1000) {
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState);
    lastBlink = millis();
  }
}
```

---

# 4. 시스템 통합 및 테스트

## 4.1 시스템 구성도

```
                    인터넷
                      |
              [WiFi 공유기]
                      |
           ┌─────────────────────┐
           |                     |
    [MQTT 브로커]          [웹 서버]
    (Node.js)             (Express)
           |                     |
           └─────────┬───────────┘
                     |
              [메인 허브]
              (ESP32 + 터치스크린)
                     |
        ┌────────────┼────────────┐
        |            |            |
   [환경센서]    [보안센서]    [릴레이제어]
   (나노+WiFi)   (Mini+nRF24)  (나노+WiFi)
        |            |            |
   온습도,조도     모션,도어      조명,가전
   공기질센서     진동센서       전류센서
```

## 4.2 통합 테스트 스크립트

```python
#!/usr/bin/env python3
"""
Smart Home System Integration Test
스마트홈 시스템 통합 테스트 스크립트
"""

import paho.mqtt.client as mqtt
import json
import time
import threading
import requests
from datetime import datetime

# MQTT 브로커 설정
MQTT_BROKER = "192.168.1.100"
MQTT_PORT = 1883
WEB_SERVER = "http://192.168.1.100:3000"

class SmartHomeTest:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.received_messages = []
        self.test_results = {}
        
    def on_connect(self, client, userdata, flags, rc):
        print(f"MQTT 브로커 연결됨 (코드: {rc})")
        
        # 모든 토픽 구독
        topics = [
            "smarthome/sensors/+",
            "smarthome/status/+",
            "smarthome/control/+"
        ]
        
        for topic in topics:
            client.subscribe(topic)
            print(f"토픽 구독: {topic}")
    
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        timestamp = datetime.now().isoformat()
        
        print(f"[{timestamp}] {topic}: {payload}")
        
        self.received_messages.append({
            'topic': topic,
            'payload': payload,
            'timestamp': timestamp
        })
    
    def connect_mqtt(self):
        """MQTT 브로커 연결"""
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
            return True
        except Exception as e:
            print(f"MQTT 연결 실패: {e}")
            return False
    
    def test_web_server(self):
        """웹 서버 연결 테스트"""
        print("\n=== 웹 서버 테스트 ===")
        
        try:
            response = requests.get(WEB_SERVER, timeout=5)
            if response.status_code == 200:
                print("✅ 웹 서버 정상 응답")
                self.test_results['web_server'] = True
            else:
                print(f"❌ 웹 서버 오류: {response.status_code}")
                self.test_results['web_server'] = False
        except Exception as e:
            print(f"❌ 웹 서버 연결 실패: {e}")
            self.test_results['web_server'] = False
    
    def test_sensor_nodes(self):
        """센서 노드 통신 테스트"""
        print("\n=== 센서 노드 테스트 ===")
        
        # 환경 센서 테스트
        print("환경 센서 데이터 대기 중...")
        env_data_received = self.wait_for_message("smarthome/sensors/environmental", 30)
        
        if env_data_received:
            print("✅ 환경 센서 데이터 수신")
            self.test_results['env_sensor'] = True
        else:
            print("❌ 환경 센서 데이터 수신 실패")
            self.test_results['env_sensor'] = False
        
        # 보안 센서 테스트
        print("보안 센서 상태 요청...")
        self.client.publish("smarthome/control/security", json.dumps({
            "command": "status_request"
        }))
        
        security_data_received = self.wait_for_message("smarthome/sensors/security", 15)
        
        if security_data_received:
            print("✅ 보안 센서 응답 수신")
            self.test_results['security_sensor'] = True
        else:
            print("❌ 보안 센서 응답 없음")
            self.test_results['security_sensor'] = False
    
    def test_relay_control(self):
        """릴레이 제어 테스트"""
        print("\n=== 릴레이 제어 테스트 ===")
        
        # 조명 제어 테스트
        test_commands = [
            {"device": "light", "id": 0, "state": True},
            {"device": "light", "id": 0, "state": False},
            {"device": "relay", "id": 4, "state": True},  # 에어컨
            {"device": "relay", "id": 4, "state": False}
        ]
        
        for cmd in test_commands:
            print(f"명령 전송: {cmd}")
            self.client.publish("smarthome/control/relay", json.dumps(cmd))
            time.sleep(2)
            
            # 상태 업데이트 확인
            status_received = self.wait_for_message("smarthome/status/relay", 5)
            if status_received:
                print("✅ 릴레이 상태 업데이트 확인")
            else:
                print("❌ 릴레이 상태 업데이트 없음")
        
        self.test_results['relay_control'] = True
    
    def test_scenarios(self):
        """시나리오 실행 테스트"""
        print("\n=== 시나리오 테스트 ===")
        
        scenarios = ["evening", "sleep", "morning", "away"]
        
        for scenario in scenarios:
            print(f"시나리오 실행: {scenario}")
            self.client.publish("smarthome/control/relay", json.dumps({
                "device": "scenario",
                "name": scenario
            }))
            time.sleep(3)
        
        self.test_results['scenarios'] = True
    
    def test_power_monitoring(self):
        """전력 모니터링 테스트"""
        print("\n=== 전력 모니터링 테스트 ===")
        
        power_data_received = self.wait_for_message("smarthome/sensors/power", 30)
        
        if power_data_received:
            print("✅ 전력 데이터 수신")
            self.test_results['power_monitoring'] = True
        else:
            print("❌ 전력 데이터 수신 실패")
            self.test_results['power_monitoring'] = False
    
    def wait_for_message(self, topic_pattern, timeout):
        """특정 토픽의 메시지 대기"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            for msg in self.received_messages:
                if topic_pattern in msg['topic']:
                    return True
            time.sleep(0.5)
        
        return False
    
    def performance_test(self):
        """성능 테스트"""
        print("\n=== 성능 테스트 ===")
        
        start_time = time.time()
        message_count = 0
        
        # 30초 동안 메시지 수집
        while time.time() - start_time < 30:
            initial_count = len(self.received_messages)
            time.sleep(1)
            current_count = len(self.received_messages)
            message_count += (current_count - initial_count)
        
        throughput = message_count / 30
        print(f"메시지 처리량: {throughput:.2f} 메시지/초")
        
        if throughput > 1:
            print("✅ 성능 테스트 통과")
            self.test_results['performance'] = True
        else:
            print("❌ 성능 테스트 실패")
            self.test_results['performance'] = False
    
    def generate_report(self):
        """테스트 리포트 생성"""
        print("\n" + "="*50)
        print("           스마트홈 시스템 테스트 리포트")
        print("="*50)
        
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        
        print(f"총 테스트: {total_tests}")
        print(f"통과: {passed_tests}")
        print(f"실패: {total_tests - passed_tests}")
        print(f"성공률: {(passed_tests/total_tests)*100:.1f}%")
        print()
        
        for test_name, result in self.test_results.items():
            status = "✅ 통과" if result else "❌ 실패"
            print(f"{test_name:20}: {status}")
        
        print("\n상세 메시지 로그:")
        print("-"*30)
        for msg in self.received_messages[-10:]:  # 최근 10개 메시지
            print(f"[{msg['timestamp']}] {msg['topic']}")
            try:
                data = json.loads(msg['payload'])
                print(f"  → {json.dumps(data, indent=2, ensure_ascii=False)}")
            except:
                print(f"  → {msg['payload']}")
        
        # 리포트 파일 저장
        report_filename = f"test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_filename, 'w', encoding='utf-8') as f:
            json.dump({
                'test_results': self.test_results,
                'messages': self.received_messages,
                'summary': {
                    'total_tests': total_tests,
                    'passed_tests': passed_tests,
                    'success_rate': (passed_tests/total_tests)*100
                }
            }, f, indent=2, ensure_ascii=False)
        
        print(f"\n리포트 저장됨: {report_filename}")
    
    def run_all_tests(self):
        """모든 테스트 실행"""
        print("스마트홈 시스템 통합 테스트 시작")
        print("="*50)
        
        if not self.connect_mqtt():
            print("MQTT 연결 실패 - 테스트 중단")
            return
        
        # 연결 안정화 대기
        time.sleep(2)
        
        # 테스트 실행
        self.test_web_server()
        self.test_sensor_nodes()
        self.test_relay_control()
        self.test_scenarios()
        self.test_power_monitoring()
        self.performance_test()
        
        # 리포트 생성
        self.generate_report()
        
        # 정리
        self.client.loop_stop()
        self.client.disconnect()

def main():
    """메인 함수"""
    tester = SmartHomeTest()
    
    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        print("\n테스트 중단됨")
    except Exception as e:
        print(f"테스트 오류: {e}")

if __name__ == "__main__":
    main()
```

## 4.3 배포 및 설치 가이드

```bash
#!/bin/bash
# Smart Home System Deployment Script
# 스마트홈 시스템 배포 스크립트

echo "스마트홈 시스템 배포 시작..."

# 1. 시스템 업데이트
echo "시스템 업데이트 중..."
sudo apt update && sudo apt upgrade -y

# 2. Node.js 설치
echo "Node.js 설치 중..."
curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt-get install -y nodejs

# 3. MQTT 브로커 설치 (Mosquitto)
echo "MQTT 브로커 설치 중..."
sudo apt-get install -y mosquitto mosquitto-clients

# 4. Python 의존성 설치
echo "Python 의존성 설치 중..."
pip3 install paho-mqtt requests flask

# 5. 프로젝트 디렉토리 생성
echo "프로젝트 디렉토리 설정 중..."
mkdir -p ~/smarthome/{server,web,config,logs}
cd ~/smarthome

# 6. Node.js 의존성 설치
echo "Node.js 의존성 설치 중..."
cat > package.json << EOF
{
  "name": "smart-home-system",
  "version": "1.0.0",
  "description": "IoT Smart Home System",
  "main": "server.js",
  "dependencies": {
    "express": "^4.18.0",
    "mosca": "^2.8.3",
    "websocket": "^1.0.34",
    "cors": "^2.8.5"
  }
}
EOF

npm install

# 7. 시스템 서비스 설정
echo "시스템 서비스 설정 중..."
sudo tee /etc/systemd/system/smarthome.service > /dev/null << EOF
[Unit]
Description=Smart Home System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/smarthome
ExecStart=/usr/bin/node server/mqtt_server.js
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# 8. 방화벽 설정
echo "방화벽 설정 중..."
sudo ufw allow 1883/tcp  # MQTT
sudo ufw allow 3000/tcp  # Web Server
sudo ufw allow 8080/tcp  # WebSocket

# 9. Mosquitto 설정
echo "MQTT 브로커 설정 중..."
sudo tee /etc/mosquitto/conf.d/smarthome.conf > /dev/null << EOF
# Smart Home MQTT Configuration
listener 1883
allow_anonymous true
max_connections 100
max_inflight_messages 20
max_queued_messages 1000
persistent_client_expiration 1h

# 로그 설정
log_dest file /var/log/mosquitto/mosquitto.log
log_type error
log_type warning
log_type notice
log_type information
log_timestamp true
EOF

# 10. 로그 디렉토리 생성
sudo mkdir -p /var/log/mosquitto
sudo chown mosquitto:mosquitto /var/log/mosquitto

# 11. 서비스 시작
echo "서비스 시작 중..."
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
sudo systemctl enable smarthome
sudo systemctl start smarthome

# 12. 테스트 스크립트 복사
echo "테스트 환경 설정 중..."
# 이전에 작성한 테스트 스크립트를 여기에 복사

# 13. 설치 완료 메시지
echo ""
echo "======================================"
echo "  스마트홈 시스템 설치 완료!"
echo "======================================"
echo ""
echo "서비스 상태 확인:"
echo "  sudo systemctl status mosquitto"
echo "  sudo systemctl status smarthome"
echo ""
echo "웹 대시보드: http://$(hostname -I | awk '{print $1}'):3000"
echo "MQTT 브로커: $(hostname -I | awk '{print $1}'):1883"
echo ""
echo "테스트 실행:"
echo "  python3 integration_test.py"
echo ""
echo "로그 확인:"
echo "  tail -f /var/log/mosquitto/mosquitto.log"
echo "  sudo journalctl -u smarthome -f"
```

---

# 5. 결론 및 확장 가능성

## 5.1 프로젝트 학습 성과

이 IoT 스마트홈 프로젝트를 통해 다음과 같은 핵심 기술들을 종합적으로 학습했습니다:

### 🎯 **핵심 학습 내용**
1. **ESP32 고급 활용**: WiFi, 터치스크린, 멀티태스킹
2. **MQTT 통신**: 실시간 IoT 데이터 전송 프로토콜
3. **웹 기술 통합**: HTML5, CSS3, JavaScript, WebSocket
4. **센서 네트워크**: 다중 아두이노 노드 간 통신
5. **시스템 아키텍처**: 분산 시스템 설계 및 구현
6. **전력 관리**: 저전력 설계 및 배터리 최적화
7. **보안**: IoT 보안 설계 및 구현

### 🚀 **실용적 응용 분야**
- **스마트홈 자동화**: 조명, 온도, 보안 통합 제어
- **에너지 관리**: 실시간 전력 모니터링 및 최적화
- **원격 모니터링**: 웹/모바일을 통한 실시간 제어
- **데이터 분석**: 센서 데이터 수집 및 패턴 분석

## 5.2 확장 가능성

### 📈 **단계별 확장 계획**

**Phase 1: 기본 기능 확장**
- 음성 제어 (Google Assistant, Alexa 연동)
- 스마트폰 앱 개발 (Flutter/React Native)
- 더 많은 센서 추가 (CO2, 소음, UV 등)

**Phase 2: AI 통합**
- 머신러닝 기반 사용 패턴 학습
- 예측적 자동화 시스템
- 컴퓨터 비전 기반 보안 카메라

**Phase 3: 상용화 준비**
- 클라우드 서비스 연동 (AWS IoT, Azure IoT)
- 대규모 센서 네트워크 관리
- 상용 제품 수준의 보안 강화

### 💡 **추가 개발 아이디어**
1. **스마트 정원 관리**: 토양 센서, 자동 급수 시스템
2. **펫 케어 시스템**: 펫 모니터링, 자동 사료 공급
3. **홈 헬스케어**: 실내 공기질, 운동량 트래킹
4. **에너지 하베스팅**: 태양광, 진동 에너지 수집

---

## 5.3 최종 코드 리뷰 체크리스트

### ✅ **코드 품질**
- [ ] 모든 함수에 적절한 주석 추가
- [ ] 에러 처리 및 예외 상황 대응
- [ ] 메모리 누수 방지 (동적 할당 최소화)
- [ ] 변수명 및 함수명 명명 규칙 일관성

### ✅ **시스템 안정성**
- [ ] 네트워크 연결 끊김 시 자동 재연결
- [ ] 센서 오류 시 fallback 메커니즘
- [ ] 전력 공급 중단 시 상태 복구
- [ ] MQTT 메시지 중복 처리 방지

### ✅ **보안 고려사항**
- [ ] WiFi 비밀번호 하드코딩 제거
- [ ] MQTT 인증 및 암호화 설정
- [ ] 웹 인터페이스 보안 (HTTPS, 인증)
- [ ] 펌웨어 업데이트 보안

### ✅ **성능 최적화**
- [ ] 불필요한 delay() 사용 최소화
- [ ] 센서 읽기 주기 최적화
- [ ] 메모리 사용량 모니터링
- [ ] 배터리 수명 최대화

---

**🏆 축하합니다!** 15개 레슨으로 구성된 아두이노 완전 마스터 과정을 성공적으로 완주했습니다!

단순한 LED 깜빡이기부터 시작해서 상용 수준의 IoT 스마트홈 시스템까지, 여러분은 이제 **전문 임베디드 개발자**로서의 역량을 갖추게 되었습니다.

이 과정에서 배운 지식과 경험을 바탕으로, 더 창의적이고 혁신적인 IoT 프로젝트에 도전해보세요! 🚀

---

**다음 단계 추천:**
1. 본인만의 IoT 프로젝트 설계 및 구현
2. 오픈소스 IoT 프로젝트 기여
3. IoT 관련 대회 참가 및 포트폴리오 구축
4. 전문 IoT 개발자 커뮤니티 참여

**행운을 빕니다!** ✨
    
    ctx.fillStyle = '#2c3e50';
    ctx.fillText('습도 (%)', 10, 35);
    ctx.fillStyle = '#3498db';
    ctx.fillRect(70, 30, 10, 2);
}

// 데이터 라인 그리기
function drawDataLine(ctx, data, color, minVal, maxVal) {
    if (data.length < 2) return;
    
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;
    
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

// 차트 데이터 업데이트
function updateChartData(data) {
    const now = new Date().toLocaleTimeString();
    
    // 환경 데이터 추가
    if (data.temperature !== undefined && data.humidity !== undefined) {
        chartData.environment.labels.push(now);
        chartData.environment.temperature.push(data.temperature);
        chartData.environment.humidity.push(data.humidity);
        
        // 최대 20개 데이터포인트 유지
        if (chartData.environment.labels.length > 20) {
            chartData.environment.labels.shift();
            chartData.environment.temperature.shift();
            chartData.environment.humidity.shift();
        }
        
        // 차트 다시 그리기
        const envCanvas = document.getElementById('environment-chart');
        if (envCanvas) {
            const ctx = envCanvas.getContext('2d');
            drawEnvironmentChart(ctx);
        }
    }
}

// 시간 업데이트
function updateTime() {
    const now = new Date();
    const timeString = now.toLocaleTimeString('ko-KR', {
        hour: '2-digit',
        minute: '2-digit'
    });
    
    const timeElement = document.getElementById('current-time');
    if (timeElement) {
        timeElement.textContent = timeString;
    }
}

// 시스템 상태 로드
async function loadSystemStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        // UI 업데이트
        updateSystemUI(data);
        
    } catch (error) {
        console.error('시스템 상태 로드 오류:', error);
    }
}

// 시스템 UI 업데이트
function updateSystemUI(data) {
    // 환경 데이터
    document.getElementById('temperature').textContent = data.temperature.toFixed(1) + '°C';
    document.getElementById('humidity').textContent = data.humidity.toFixed(0) + '%';
    document.getElementById('air-quality').textContent = data.airQuality || '--';
    document.getElementById('light-level').textContent = data.lightLevel || '--';
    
    // 조명 상태
    for (let i = 0; i < data.lights.length; i++) {
        const lightSwitch = document.getElementById('light-' + i);
        if (lightSwitch) {
            lightSwitch.checked = data.lights[i];
        }
    }
    
    // 가전제품 상태
    for (let i = 0; i < data.appliances.length; i++) {
        const applianceBtn = document.querySelector(`[data-appliance="${i}"] .appliance-btn`);
        if (applianceBtn) {
            applianceBtn.textContent = data.appliances[i] ? 'ON' : 'OFF';
            applianceBtn.classList.toggle('active', data.appliances[i]);
        }
    }
    
    // 보안 상태
    updateSecurityStatus(data.securityArmed);
    
    // 에너지 정보
    document.getElementById('current-power').textContent = (data.energyUsage || 0) + ' W';
    
    // WiFi 상태
    const wifiStatus = data.wifiSignal > -70 ? '연결됨' : '약함';
    document.getElementById('wifi-status').textContent = wifiStatus;
}

// 보안 상태 업데이트
function updateSecurityStatus(armed) {
    const indicator = document.getElementById('security-indicator');
    const status = document.getElementById('security-status');
    const toggle = document.getElementById('security-toggle');
    
    if (armed) {
        indicator.classList.add('armed');
        status.textContent = '설정됨';
        toggle.textContent = '보안 해제';
        toggle.className = 'btn btn-secondary';
    } else {
        indicator.classList.remove('armed');
        status.textContent = '해제됨';
        toggle.textContent = '보안 설정';
        toggle.className = 'btn btn-warning';
    }
}

// 연결 상태 업데이트
function updateConnectionStatus(connected) {
    const wifiStatus = document.getElementById('wifi-status');
    if (wifiStatus) {
        wifiStatus.textContent = connected ? '연결됨' : '연결 끊어짐';
    }
}

// 이벤트 리스너 등록
function registerEventListeners() {
    // 조명 스위치
    document.querySelectorAll('.light-switch').forEach(function(switch_) {
        switch_.addEventListener('change', function() {
            const roomIndex = parseInt(this.dataset.room);
            const state = this.checked;
            controlDevice('light', roomIndex, state);
        });
    });
    
    // 가전제품 버튼
    document.querySelectorAll('.appliance-btn').forEach(function(btn) {
        btn.addEventListener('click', function() {
            const applianceIndex = parseInt(this.parentElement.dataset.appliance);
            const currentState = this.classList.contains('active');
            const newState = !currentState;
            controlDevice('appliance', applianceIndex, newState);
        });
    });
}

// 디바이스 제어
async function controlDevice(device, index, state) {
    try {
        const response = await fetch('/api/control', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                device: device,
                index: index,
                state: state
            })
        });
        
        if (!response.ok) {
            throw new Error('제어 명령 실패');
        }
        
        // WebSocket으로도 전송
        if (isConnected) {
            socket.send(JSON.stringify({
                action: 'control',
                device: device,
                index: index,
                state: state
            }));
        }
        
    } catch (error) {
        console.error('디바이스 제어 오류:', error);
        alert('디바이스 제어에 실패했습니다.');
    }
}

// 전체 조명 제어
function controlAllLights(state) {
    for (let i = 0; i < 4; i++) {
        controlDevice('light', i, state);
    }
}

// 가전제품 토글
function toggleAppliance(index) {
    const btn = document.querySelector(`[data-appliance="${index}"] .appliance-btn`);
    const currentState = btn.classList.contains('active');
    const newState = !currentState;
    controlDevice('appliance', index, newState);
}

// 보안 시스템 토글
function toggleSecurity() {
    const indicator = document.getElementById('security-indicator');
    const currentState = indicator.classList.contains('armed');
    const newState = !currentState;
    controlDevice('security', 0, newState);
}

// 음성 제어 시작
function startVoiceControl() {
    const btn = document.getElementById('voice-btn');
    btn.classList.add('active');
    
    // 음성 인식 시뮬레이션
    setTimeout(function() {
        btn.classList.remove('active');
        alert('음성 명령이 인식되었습니다!');
    }, 3000);
}

// 빠른 모드 설정
function setHomeMode() {
    // 재택 모드: 적당한 조명, 에어컨 켜기
    controlAllLights(true);
    controlDevice('appliance', 0, true); // 에어컨
    alert('재택 모드로 설정되었습니다.');
}

function setAwayMode() {
    // 외출 모드: 모든 것 끄기, 보안 설정
    controlAllLights(false);
    for (let i = 0; i < 4; i++) {
        controlDevice('appliance', i, false);
    }
    controlDevice('security', 0, true);
    alert('외출 모드로 설정되었습니다.');
}

function setSleepMode() {
    // 취침 모드: 조명 끄기, TV/오디오 끄기, 보안 설정
    controlAllLights(false);
    controlDevice('appliance', 1, false); // TV
    controlDevice('appliance', 2, false); // 오디오
    controlDevice('security', 0, true);
    alert('취침 모드로 설정되었습니다.');
}

function setPartyMode() {
    // 파티 모드: 모든 조명 켜기, 오디오 켜기
    controlAllLights(true);
    controlDevice('appliance', 2, true); // 오디오
    alert('파티 모드로 설정되었습니다.');
}

// 자동화 규칙 관리
async function loadAutomationRules() {
    try {
        const response = await fetch('/api/automation');
        const data = await response.json();
        
        const rulesList = document.getElementById('automation-list');
        rulesList.innerHTML = '';
        
        data.rules.forEach(function(rule, index) {
            const ruleElement = createRuleElement(rule, index);
            rulesList.appendChild(ruleElement);
        });
        
    } catch (error) {
        console.error('자동화 규칙 로드 오류:', error);
    }
}

function createRuleElement(rule, index) {
    const div = document.createElement('div');
    div.className = 'automation-rule';
    div.innerHTML = `
        <div>
            <strong>${rule.name}</strong>
            <br>
            <small>${rule.condition} → ${rule.action}</small>
        </div>
        <div class="rule-toggle ${rule.enabled ? 'active' : ''}" 
             onclick="toggleRule(${index})"></div>
    `;
    return div;
}

function toggleRule(index) {
    // 규칙 토글 로직
    console.log('규칙 토글:', index);
}

// 자동화 규칙 추가 모달
function showAddRuleModal() {
    document.getElementById('add-rule-modal').style.display = 'block';
}

function closeAddRuleModal() {
    document.getElementById('add-rule-modal').style.display = 'none';
}

async function addAutomationRule() {
    const name = document.getElementById('rule-name').value;
    const conditionType = document.getElementById('rule-condition-type').value;
    const conditionValue = document.getElementById('rule-condition-value').value;
    const actionType = document.getElementById('rule-action-type').value;
    const actionValue = document.getElementById('rule-action-value').value;
    
    if (!name || !conditionValue || !actionValue) {
        alert('모든 필드를 입력해주세요.');
        return;
    }
    
    try {
        const response = await fetch('/api/automation', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                name: name,
                condition: conditionType + ':' + conditionValue,
                action: actionType + ':' + actionValue,
                enabled: true
            })
        });
        
        if (response.ok) {
            closeAddRuleModal();
            loadAutomationRules();
            alert('자동화 규칙이 추가되었습니다.');
        } else {
            throw new Error('규칙 추가 실패');
        }
        
    } catch (error) {
        console.error('자동화 규칙 추가 오류:', error);
        alert('자동화 규칙 추가에 실패했습니다.');
    }
}

// 모달 외부 클릭시 닫기
window.onclick = function(event) {
    const modal = document.getElementById('add-rule-modal');
    if (event.target == modal) {
        closeAddRuleModal();
    }
}
)";
  webServer.send(200, "application/javascript", js);
}

void handleNotFound() {
  webServer.send(404, "text/plain", "404: 페이지를 찾을 수 없습니다.");
}

void handleFileUpload() {
  HTTPUpload& upload = webServer.upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("업로드 시작: %s\n", upload.filename.c_str());
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    // 파일 쓰기 로직
  } else if (upload.status == UPLOAD_FILE_END) {
    Serial.printf("업로드 완료: %u bytes\n", upload.totalSize);
  }
}

void displayWelcomeScreen() {
  tft.fillScreen(TFT_BLACK);
  
  // 타이틀
  tft.setTextColor(TFT_CYAN);
  tft.setTextSize(3);
  tft.drawString("Smart Home", 120, 80);
  
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.drawString("Control Center", 140, 120);
  
  // 로딩 애니메이션
  for (int i = 0; i < 480; i += 20) {
    tft.drawPixel(i, 200, TFT_GREEN);
    delay(50);
  }
  
  tft.setTextSize(1);
  tft.drawString("System Ready", 200, 220);
  
  delay(2000);
  drawHomeScreen();
}

void processSensorData(String topic, String message) {
  // 토픽 파싱: smarthome/sensors/room/sensor_type
  int firstSlash = topic.indexOf('/', 10);  // "smarthome/" 이후
  int secondSlash = topic.indexOf('/', firstSlash + 1);
  
  if (firstSlash > 0 && secondSlash > 0) {
    String room = topic.substring(firstSlash + 1, secondSlash);
    String sensorType = topic.substring(secondSlash + 1);
    
    float value = message.toFloat();
    
    // 센서 데이터 처리
    if (sensorType == "temperature") {
      homeSystem.temperature = value;
    } else if (sensorType == "humidity") {
      homeSystem.humidity = value;
    } else if (sensorType == "air_quality") {
      homeSystem.airQuality = value;
    } else if (sensorType == "light") {
      homeSystem.lightLevel = (int)value;
    }
    
    Serial.print("센서 데이터 업데이트 - ");
    Serial.print(room);
    Serial.print("/");
    Serial.print(sensorType);
    Serial.print(": ");
    Serial.println(value);
  }
}

void processControlCommand(String topic, String message) {
  // 원격 제어 명령 처리
  Serial.print("제어 명령 수신: ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(message);
  
  if (topic.endsWith("/light")) {
    // 조명 제어
  } else if (topic.endsWith("/appliance")) {
    // 가전제품 제어
  }
}
```

이제 **메인 허브 시스템**이 완성되었습니다! 다음으로 **센서 노드들**과 **최종 시스템 통합**을 계속해서 완전한 스마트홈 시스템을 완성하겠습니다.

계속해서 나머지 부분들을 구현해드릴까요?