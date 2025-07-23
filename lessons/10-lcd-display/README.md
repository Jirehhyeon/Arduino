# Lesson 10: LCD 디스플레이 제어 마스터

## 🎯 학습 목표
- LCD 동작 원리와 인터페이스 이해
- HD44780 컨트롤러 기반 LCD 제어 방법
- I2C LCD 모듈 사용법 및 배선 최적화
- 커스텀 문자 생성 및 특수 효과 구현
- 실시간 데이터 표시 및 메뉴 시스템 구현

## 📚 이론 배경

### LCD 디스플레이 기본 원리

LCD(Liquid Crystal Display)는 액정의 전기적 특성을 이용해 문자와 도형을 표시하는 장치입니다.

```
HD44780 컨트롤러 구조:
┌─────────────────────────────────────┐
│           LCD 모듈 (16x2)           │
├─────────────────────────────────────┤
│  VSS VDD V0  RS  EN  D4 D5 D6 D7   │
│   │   │   │   │   │   │  │  │  │    │
│  GND +5V 대비 제어 클럭 데이터버스   │
└─────────────────────────────────────┘

핀 기능:
- VSS: 접지 (GND)
- VDD: 전원 (+5V)  
- V0: 대비 조절 (가변저항 또는 PWM)
- RS: 레지스터 선택 (명령/데이터)
- EN: 활성화 신호 (클럭)
- D4~D7: 4비트 데이터 버스
```

### LCD 제어 프로토콜

#### 4비트 모드 통신 프로세스
```cpp
// LCD 명령 전송 과정
void sendCommand(uint8_t command) {
  digitalWrite(RS, LOW);      // 명령 모드
  digitalWrite(EN, HIGH);     // 활성화
  
  // 상위 4비트 전송
  digitalWrite(D4, (command >> 4) & 0x01);
  digitalWrite(D5, (command >> 5) & 0x01);
  digitalWrite(D6, (command >> 6) & 0x01);
  digitalWrite(D7, (command >> 7) & 0x01);
  
  digitalWrite(EN, LOW);      // 클럭 신호
  delayMicroseconds(1);
  
  // 하위 4비트 전송
  digitalWrite(D4, command & 0x01);
  digitalWrite(D5, (command >> 1) & 0x01);
  digitalWrite(D6, (command >> 2) & 0x01);
  digitalWrite(D7, (command >> 3) & 0x01);
  
  digitalWrite(EN, LOW);
  delayMicroseconds(37);      // 명령 처리 대기
}
```

## 🔧 실습 1: 기본 LCD 제어

### 하드웨어 연결

```
아두이노 우노 ←→ 16x2 LCD
┌─────────────┐    ┌─────────────┐
│  디지털 핀  │    │   LCD 핀    │
├─────────────┼────┼─────────────┤
│     2       │────│     RS      │
│     3       │────│     EN      │
│     4       │────│     D4      │
│     5       │────│     D5      │
│     6       │────│     D6      │
│     7       │────│     D7      │
│    5V       │────│    VDD      │
│    GND      │────│    VSS      │
│    A0       │────│     V0      │ (가변저항 통해)
└─────────────┘    └─────────────┘

대비 조절 회로:
     +5V ──┐
           │
         ┌─┴─┐ 10kΩ 가변저항
    V0 ──┤   ├── GND
         └───┘
```

### 기본 LCD 제어 코드

```cpp
/*
  Basic_LCD_Control.ino
  HD44780 컨트롤러 기반 LCD 기본 제어
*/

#include <LiquidCrystal.h>

// LCD 핀 연결 설정 (RS, EN, D4, D5, D6, D7)
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

void setup() {
  // LCD 크기 설정 (컬럼, 행)
  lcd.begin(16, 2);
  
  // 초기 메시지 표시
  lcd.print("Arduino LCD");
  lcd.setCursor(0, 1);
  lcd.print("제어 마스터!");
  
  Serial.begin(9600);
  Serial.println("LCD 초기화 완료");
}

void loop() {
  // 시간 정보 표시
  displayTime();
  delay(1000);
}

void displayTime() {
  static unsigned long previousMillis = 0;
  static int seconds = 0;
  
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    seconds++;
    
    // 두 번째 줄에 시간 표시
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(seconds);
    lcd.print("s   ");  // 이전 숫자 지우기
  }
}
```

## 🔧 실습 2: I2C LCD 모듈 사용

I2C 인터페이스를 사용하면 핀 사용량을 대폭 줄일 수 있습니다.

### I2C LCD 모듈 연결

```
아두이노 우노 ←→ I2C LCD 모듈
┌─────────────┐    ┌─────────────┐
│    핀       │    │  I2C LCD    │
├─────────────┼────┼─────────────┤
│    A4       │────│    SDA      │
│    A5       │────│    SCL      │
│    5V       │────│    VCC      │
│    GND      │────│    GND      │
└─────────────┘    └─────────────┘

I2C LCD 백팩 구조:
┌───────────────────────────────┐
│  PCF8574 I2C 확장 칩          │
│  ┌─────┐  8비트 → LCD 제어    │
│  │     │  P0~P7 핀 매핑:      │
│  │PCF  │  P0: RS              │
│  │8574 │  P1: RW              │
│  │     │  P2: EN              │
│  └─────┘  P3: Backlight      │
│            P4~P7: D4~D7       │
└───────────────────────────────┘
```

### I2C LCD 제어 코드

```cpp
/*
  I2C_LCD_Advanced.ino
  I2C 인터페이스를 통한 LCD 고급 제어
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// I2C LCD 객체 생성 (주소, 컬럼, 행)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 센서 시뮬레이션 변수
float temperature = 25.5;
int humidity = 60;
unsigned long lastUpdate = 0;

void setup() {
  // I2C LCD 초기화
  lcd.init();
  lcd.backlight();
  
  // 시작 화면
  lcd.setCursor(0, 0);
  lcd.print("   Smart Home   ");
  lcd.setCursor(0, 1);
  lcd.print("  Monitoring    ");
  delay(2000);
  
  lcd.clear();
  setupDisplay();
  
  Serial.begin(9600);
  Serial.println("I2C LCD 시스템 시작");
}

void loop() {
  updateSensorData();
  displaySensorData();
  handleSerialInput();
  delay(500);
}

void setupDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("Temp:    ");
  lcd.write(byte(223));  // 도 기호
  lcd.print("C");
  
  lcd.setCursor(0, 1);
  lcd.print("Humid:     %");
}

void updateSensorData() {
  if (millis() - lastUpdate > 2000) {
    // 센서 데이터 시뮬레이션
    temperature += random(-10, 11) / 10.0;
    humidity += random(-3, 4);
    
    // 범위 제한
    temperature = constrain(temperature, 15.0, 35.0);
    humidity = constrain(humidity, 30, 80);
    
    lastUpdate = millis();
  }
}

void displaySensorData() {
  // 온도 표시
  lcd.setCursor(6, 0);
  if (temperature < 10) lcd.print(" ");
  lcd.print(temperature, 1);
  
  // 습도 표시  
  lcd.setCursor(7, 1);
  if (humidity < 10) lcd.print(" ");
  lcd.print(humidity);
}

void handleSerialInput() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "clear") {
      lcd.clear();
      setupDisplay();
    }
    else if (command == "backlight") {
      lcd.backlight();
    }
    else if (command == "nobacklight") {
      lcd.noBacklight();
    }
    else if (command.startsWith("message:")) {
      String msg = command.substring(8);
      displayMessage(msg);
    }
  }
}

void displayMessage(String message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("메시지:");
  lcd.setCursor(0, 1);
  lcd.print(message);
  delay(3000);
  lcd.clear();
  setupDisplay();
}
```

## 🔧 실습 3: 커스텀 문자 및 애니메이션

LCD에서 사용자 정의 문자를 만들어 특별한 효과를 구현할 수 있습니다.

### 커스텀 문자 생성 원리

```cpp
/*
  Custom_Characters.ino
  사용자 정의 문자 생성 및 애니메이션
*/

#include <LiquidCrystal.h>

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

// 커스텀 문자 비트맵 (5x8 픽셀)
byte heart[8] = {
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

byte speaker[8] = {
  0b00001,
  0b00011,
  0b11111,
  0b11111,
  0b11111,
  0b00011,
  0b00001,
  0b00000
};

byte sound1[8] = {
  0b00001,
  0b00011,
  0b00101,
  0b01001,
  0b01001,
  0b00101,
  0b00011,
  0b00001
};

byte sound2[8] = {
  0b00010,
  0b00110,
  0b01010,
  0b10010,
  0b10010,
  0b01010,
  0b00110,
  0b00010
};

byte battery_full[8] = {
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111
};

byte battery_half[8] = {
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte battery_empty[8] = {
  0b01110,
  0b11011,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte degree[8] = {
  0b01100,
  0b10010,
  0b10010,
  0b01100,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

void setup() {
  lcd.begin(16, 2);
  
  // 커스텀 문자 등록 (0~7번)
  lcd.createChar(0, heart);
  lcd.createChar(1, speaker);
  lcd.createChar(2, sound1);
  lcd.createChar(3, sound2);
  lcd.createChar(4, battery_full);
  lcd.createChar(5, battery_half);
  lcd.createChar(6, battery_empty);
  lcd.createChar(7, degree);
  
  // 초기 화면
  lcd.setCursor(0, 0);
  lcd.print("Custom Chars");
  lcd.setCursor(0, 1);
  lcd.print("Animation Demo");
  delay(2000);
  
  Serial.begin(9600);
}

void loop() {
  // 하트 비트 애니메이션
  heartBeatAnimation();
  delay(1000);
  
  // 스피커 애니메이션
  speakerAnimation();
  delay(1000);
  
  // 배터리 애니메이션
  batteryAnimation();
  delay(1000);
  
  // 온도 표시
  temperatureDisplay();
  delay(2000);
}

void heartBeatAnimation() {
  lcd.clear();
  lcd.setCursor(7, 0);
  lcd.print("LOVE");
  
  for (int i = 0; i < 5; i++) {
    lcd.setCursor(7, 1);
    lcd.write(byte(0));  // 하트 표시
    delay(300);
    
    lcd.setCursor(7, 1);
    lcd.print(" ");      // 하트 지우기
    delay(200);
  }
}

void speakerAnimation() {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Sound");
  
  for (int i = 0; i < 8; i++) {
    lcd.setCursor(6, 1);
    lcd.write(byte(1));  // 스피커
    lcd.write(byte(2 + (i % 2)));  // 사운드 웨이브
    delay(250);
  }
}

void batteryAnimation() {
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Battery");
  
  // 방전 애니메이션
  byte batteryLevels[] = {4, 5, 6};  // 만충, 반, 빈
  String labels[] = {"100%", " 50%", "  0%"};
  
  for (int i = 0; i < 3; i++) {
    lcd.setCursor(6, 1);
    lcd.write(byte(batteryLevels[i]));
    lcd.print(" ");
    lcd.print(labels[i]);
    delay(1000);
  }
  
  // 충전 애니메이션
  for (int i = 2; i >= 0; i--) {
    lcd.setCursor(6, 1);
    lcd.write(byte(batteryLevels[i]));
    lcd.print(" ");
    lcd.print(labels[i]);
    delay(500);
  }
}

void temperatureDisplay() {
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Temperature");
  
  // 온도 변화 시뮬레이션
  for (int temp = 20; temp <= 30; temp++) {
    lcd.setCursor(6, 1);
    lcd.print(temp);
    lcd.write(byte(7));  // 도 기호
    lcd.print("C    ");
    delay(200);
  }
}
```

## 🔧 실습 4: 스마트 메뉴 시스템

실제 제품에서 사용되는 메뉴 네비게이션 시스템을 구현해보겠습니다.

```cpp
/*
  Smart_Menu_System.ino
  다층 메뉴 시스템 및 설정 관리
*/

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// 버튼 핀 정의
const int BTN_UP = 8;
const int BTN_DOWN = 9;
const int BTN_SELECT = 10;
const int BTN_BACK = 11;

// 메뉴 구조체
struct MenuItem {
  String name;
  void (*action)();
  MenuItem* parent;
  MenuItem* children;
  int childCount;
};

// 시스템 설정 변수
struct Settings {
  int brightness;
  int contrast;
  bool autoMode;
  int alarmHour;
  int alarmMinute;
} settings;

// 메뉴 상태 변수
int currentMenuIndex = 0;
int menuLevel = 0;
MenuItem* currentMenu;
bool inSettingMode = false;
int settingValue = 0;

// 메뉴 아이템들
MenuItem mainMenu[4];
MenuItem settingsMenu[5];
MenuItem displayMenu[2];

void setup() {
  lcd.init();
  lcd.backlight();
  
  // 버튼 핀 설정
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  
  // EEPROM에서 설정 로드
  loadSettings();
  
  // 메뉴 구조 초기화
  setupMenuStructure();
  
  // 현재 메뉴를 메인 메뉴로 설정
  currentMenu = mainMenu;
  
  // 초기 화면 표시
  displayCurrentMenu();
  
  Serial.begin(9600);
  Serial.println("스마트 메뉴 시스템 시작");
}

void loop() {
  handleButtonInput();
  
  if (!inSettingMode) {
    updateDisplay();
  }
  
  delay(50);  // 버튼 디바운싱
}

void setupMenuStructure() {
  // 메인 메뉴 설정
  mainMenu[0] = {"Status", showStatus, nullptr, nullptr, 0};
  mainMenu[1] = {"Settings", enterSettings, nullptr, settingsMenu, 5};
  mainMenu[2] = {"Monitor", showMonitor, nullptr, nullptr, 0};
  mainMenu[3] = {"About", showAbout, nullptr, nullptr, 0};
  
  // 설정 메뉴
  settingsMenu[0] = {"Display", enterDisplay, mainMenu, displayMenu, 2};
  settingsMenu[1] = {"Auto Mode", toggleAutoMode, mainMenu, nullptr, 0};
  settingsMenu[2] = {"Alarm Hour", setAlarmHour, mainMenu, nullptr, 0};
  settingsMenu[3] = {"Alarm Min", setAlarmMinute, mainMenu, nullptr, 0};
  settingsMenu[4] = {"Save & Exit", saveAndExit, mainMenu, nullptr, 0};
  
  // 디스플레이 메뉴
  displayMenu[0] = {"Brightness", setBrightness, settingsMenu, nullptr, 0};
  displayMenu[1] = {"Contrast", setContrast, settingsMenu, nullptr, 0};
}

void handleButtonInput() {
  static unsigned long lastButtonTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastButtonTime < 200) return;  // 디바운싱
  
  if (digitalRead(BTN_UP) == LOW) {
    if (inSettingMode) {
      settingValue++;
      updateSettingDisplay();
    } else {
      navigateUp();
    }
    lastButtonTime = currentTime;
  }
  
  if (digitalRead(BTN_DOWN) == LOW) {
    if (inSettingMode) {
      settingValue--;
      updateSettingDisplay();
    } else {
      navigateDown();
    }
    lastButtonTime = currentTime;
  }
  
  if (digitalRead(BTN_SELECT) == LOW) {
    if (inSettingMode) {
      confirmSetting();
    } else {
      selectMenuItem();
    }
    lastButtonTime = currentTime;
  }
  
  if (digitalRead(BTN_BACK) == LOW) {
    if (inSettingMode) {
      cancelSetting();
    } else {
      goBack();
    }
    lastButtonTime = currentTime;
  }
}

void navigateUp() {
  currentMenuIndex--;
  if (currentMenuIndex < 0) {
    currentMenuIndex = getMenuSize() - 1;
  }
  displayCurrentMenu();
}

void navigateDown() {
  currentMenuIndex++;
  if (currentMenuIndex >= getMenuSize()) {
    currentMenuIndex = 0;
  }
  displayCurrentMenu();
}

void selectMenuItem() {
  MenuItem* item = &currentMenu[currentMenuIndex];
  
  if (item->action != nullptr) {
    item->action();
  }
  
  if (item->children != nullptr) {
    currentMenu = item->children;
    currentMenuIndex = 0;
    menuLevel++;
    displayCurrentMenu();
  }
}

void goBack() {
  if (menuLevel > 0) {
    currentMenu = currentMenu[0].parent;
    currentMenuIndex = 0;
    menuLevel--;
    displayCurrentMenu();
  }
}

int getMenuSize() {
  if (currentMenu == mainMenu) return 4;
  if (currentMenu == settingsMenu) return 5;
  if (currentMenu == displayMenu) return 2;
  return 0;
}

void displayCurrentMenu() {
  lcd.clear();
  
  // 현재 선택된 항목 표시
  lcd.setCursor(0, 0);
  lcd.print("> ");
  lcd.print(currentMenu[currentMenuIndex].name);
  
  // 다음 항목 표시 (있다면)
  int nextIndex = (currentMenuIndex + 1) % getMenuSize();
  lcd.setCursor(2, 1);
  lcd.print(currentMenu[nextIndex].name);
}

void updateDisplay() {
  // 주기적으로 시간이나 상태 정보 업데이트
}

// 메뉴 액션 함수들
void showStatus() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Status");
  lcd.setCursor(0, 1);
  lcd.print("All OK - ");
  lcd.print(millis() / 1000);
  lcd.print("s");
  delay(2000);
  displayCurrentMenu();
}

void enterSettings() {
  // 설정 메뉴로 진입 (이미 처리됨)
}

void enterDisplay() {
  // 디스플레이 메뉴로 진입 (이미 처리됨)
}

void showMonitor() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Real-time Data");
  
  for (int i = 0; i < 10; i++) {
    lcd.setCursor(0, 1);
    lcd.print("Value: ");
    lcd.print(random(0, 100));
    lcd.print("   ");
    delay(500);
    
    if (digitalRead(BTN_BACK) == LOW) break;
  }
  
  displayCurrentMenu();
}

void showAbout() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Arduino LCD");
  lcd.setCursor(0, 1);
  lcd.print("Menu v1.0");
  delay(2000);
  displayCurrentMenu();
}

void setBrightness() {
  enterSettingMode("Brightness", settings.brightness, 0, 255);
}

void setContrast() {
  enterSettingMode("Contrast", settings.contrast, 0, 100);
}

void toggleAutoMode() {
  settings.autoMode = !settings.autoMode;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Auto Mode:");
  lcd.setCursor(0, 1);
  lcd.print(settings.autoMode ? "ON" : "OFF");
  delay(1000);
  displayCurrentMenu();
}

void setAlarmHour() {
  enterSettingMode("Alarm Hour", settings.alarmHour, 0, 23);
}

void setAlarmMinute() {
  enterSettingMode("Alarm Min", settings.alarmMinute, 0, 59);
}

void saveAndExit() {
  saveSettings();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Settings Saved!");
  delay(1000);
  
  // 메인 메뉴로 돌아가기
  currentMenu = mainMenu;
  currentMenuIndex = 0;
  menuLevel = 0;
  displayCurrentMenu();
}

void enterSettingMode(String name, int currentValue, int minVal, int maxVal) {
  inSettingMode = true;
  settingValue = currentValue;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(name + ":");
  updateSettingDisplay();
}

void updateSettingDisplay() {
  lcd.setCursor(0, 1);
  lcd.print("Value: ");
  lcd.print(settingValue);
  lcd.print("   ");
}

void confirmSetting() {
  // 현재 설정 중인 항목에 따라 값 저장
  String currentItemName = currentMenu[currentMenuIndex].name;
  
  if (currentItemName == "Brightness") {
    settings.brightness = settingValue;
  } else if (currentItemName == "Contrast") {
    settings.contrast = settingValue;
  } else if (currentItemName == "Alarm Hour") {
    settings.alarmHour = settingValue;
  } else if (currentItemName == "Alarm Min") {
    settings.alarmMinute = settingValue;
  }
  
  inSettingMode = false;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Value Set!");
  delay(1000);
  displayCurrentMenu();
}

void cancelSetting() {
  inSettingMode = false;
  displayCurrentMenu();
}

void loadSettings() {
  // EEPROM에서 설정 로드
  EEPROM.get(0, settings);
  
  // 기본값 설정 (첫 실행시)
  if (settings.brightness > 255) {
    settings.brightness = 128;
    settings.contrast = 50;
    settings.autoMode = true;
    settings.alarmHour = 8;
    settings.alarmMinute = 0;
  }
}

void saveSettings() {
  EEPROM.put(0, settings);
}
```

## 🔧 실습 5: 고급 LCD 활용 - 실시간 모니터링 시스템

```cpp
/*
  Advanced_LCD_Monitor.ino
  실시간 데이터 모니터링 및 그래프 표시
*/

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // 20x4 LCD 사용

// 센서 핀 정의
const int TEMP_SENSOR = A0;
const int LIGHT_SENSOR = A1;
const int SOUND_SENSOR = A2;

// 데이터 저장 배열
float tempHistory[20];
int lightHistory[20];
int soundHistory[20];
int historyIndex = 0;

// 시스템 상태
struct SystemStatus {
  float temperature;
  int lightLevel;
  int soundLevel;
  unsigned long uptime;
  bool alarmActive;
  String systemMode;
} status;

// 알림 임계값
const float TEMP_HIGH = 30.0;
const float TEMP_LOW = 15.0;
const int SOUND_HIGH = 800;
const int LIGHT_LOW = 100;

// 커스텀 문자 (막대그래프용)
byte bar1[8] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
byte bar2[8] = {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
byte bar3[8] = {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C};
byte bar4[8] = {0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E};
byte bar5[8] = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};

void setup() {
  lcd.init();
  lcd.backlight();
  
  // 커스텀 문자 등록
  lcd.createChar(1, bar1);
  lcd.createChar(2, bar2);
  lcd.createChar(3, bar3);
  lcd.createChar(4, bar4);
  lcd.createChar(5, bar5);
  
  // 시스템 초기화
  status.systemMode = "MONITOR";
  status.alarmActive = false;
  
  // 초기 화면
  lcd.setCursor(0, 0);
  lcd.print("Environmental");
  lcd.setCursor(0, 1);
  lcd.print("Monitor System");
  lcd.setCursor(0, 2);
  lcd.print("Version 2.0");
  lcd.setCursor(0, 3);
  lcd.print("Initializing...");
  
  delay(3000);
  lcd.clear();
  
  Serial.begin(9600);
  Serial.println("고급 LCD 모니터링 시스템 시작");
}

void loop() {
  readSensors();
  updateHistory();
  displayMainScreen();
  checkAlarms();
  
  delay(1000);
}

void readSensors() {
  // 온도 센서 읽기 (LM35 기준)
  int tempReading = analogRead(TEMP_SENSOR);
  status.temperature = (tempReading * 5.0 * 100.0) / 1024.0;
  
  // 조도 센서 읽기
  status.lightLevel = analogRead(LIGHT_SENSOR);
  
  // 사운드 센서 읽기
  status.soundLevel = analogRead(SOUND_SENSOR);
  
  // 업타임 업데이트
  status.uptime = millis() / 1000;
}

void updateHistory() {
  tempHistory[historyIndex] = status.temperature;
  lightHistory[historyIndex] = status.lightLevel;
  soundHistory[historyIndex] = status.soundLevel;
  
  historyIndex = (historyIndex + 1) % 20;
}

void displayMainScreen() {
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(status.temperature, 1);
  lcd.print("C L:");
  lcd.print(map(status.lightLevel, 0, 1023, 0, 100));
  lcd.print("% S:");
  lcd.print(map(status.soundLevel, 0, 1023, 0, 100));
  lcd.print("%");
  
  // 두 번째 줄: 막대그래프
  lcd.setCursor(0, 1);
  displayBarGraph(status.temperature, 15.0, 35.0, 10);
  lcd.setCursor(11, 1);
  displayBarGraph(status.lightLevel, 0, 1023, 9);
  
  // 세 번째 줄: 상태 정보
  lcd.setCursor(0, 2);
  lcd.print("Mode: ");
  lcd.print(status.systemMode);
  if (status.alarmActive) {
    lcd.print(" [ALARM]");
  } else {
    lcd.print("        ");
  }
  
  // 네 번째 줄: 업타임
  lcd.setCursor(0, 3);
  lcd.print("Uptime: ");
  displayUptime(status.uptime);
}

void displayBarGraph(float value, float minVal, float maxVal, int width) {
  int bars = map(value, minVal, maxVal, 0, width * 5);
  int fullBars = bars / 5;
  int partialBar = bars % 5;
  
  // 완전한 막대 표시
  for (int i = 0; i < fullBars; i++) {
    lcd.write(byte(5));
  }
  
  // 부분 막대 표시
  if (partialBar > 0 && fullBars < width) {
    lcd.write(byte(partialBar));
    fullBars++;
  }
  
  // 빈 공간 채우기
  for (int i = fullBars; i < width; i++) {
    lcd.print(" ");
  }
}

void displayUptime(unsigned long seconds) {
  int days = seconds / 86400;
  int hours = (seconds % 86400) / 3600;
  int minutes = (seconds % 3600) / 60;
  
  if (days > 0) {
    lcd.print(days);
    lcd.print("d ");
  }
  
  if (hours < 10) lcd.print("0");
  lcd.print(hours);
  lcd.print(":");
  
  if (minutes < 10) lcd.print("0");
  lcd.print(minutes);
}

void checkAlarms() {
  bool newAlarmState = false;
  
  // 온도 알람
  if (status.temperature > TEMP_HIGH || status.temperature < TEMP_LOW) {
    newAlarmState = true;
  }
  
  // 소음 알람
  if (status.soundLevel > SOUND_HIGH) {
    newAlarmState = true;
  }
  
  // 조도 알람
  if (status.lightLevel < LIGHT_LOW) {
    newAlarmState = true;
  }
  
  // 알람 상태 변경시
  if (newAlarmState != status.alarmActive) {
    status.alarmActive = newAlarmState;
    
    if (status.alarmActive) {
      displayAlarmScreen();
    }
  }
}

void displayAlarmScreen() {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("ALARM!");
  
  lcd.setCursor(0, 1);
  if (status.temperature > TEMP_HIGH) {
    lcd.print("High Temperature!");
  } else if (status.temperature < TEMP_LOW) {
    lcd.print("Low Temperature!");
  } else if (status.soundLevel > SOUND_HIGH) {
    lcd.print("High Noise Level!");
  } else if (status.lightLevel < LIGHT_LOW) {
    lcd.print("Low Light Level!");
  }
  
  lcd.setCursor(0, 2);
  lcd.print("Check Environment");
  lcd.setCursor(0, 3);
  lcd.print("Press any key...");
  
  // 3초 후 원래 화면으로
  delay(3000);
  lcd.clear();
}
```

## 📊 성능 최적화 및 고급 기법

### LCD 성능 향상 기법

```cpp
// 1. 부분 업데이트 기법
void updatePartialDisplay() {
  static String lastTemp = "";
  static String lastHumid = "";
  
  String currentTemp = String(temperature, 1);
  String currentHumid = String(humidity);
  
  // 온도가 변경된 경우만 업데이트
  if (currentTemp != lastTemp) {
    lcd.setCursor(6, 0);
    lcd.print(currentTemp);
    lcd.print("   ");  // 이전 값 지우기
    lastTemp = currentTemp;
  }
  
  // 습도가 변경된 경우만 업데이트
  if (currentHumid != lastHumid) {
    lcd.setCursor(7, 1);
    lcd.print(currentHumid);
    lcd.print("   ");
    lastHumid = currentHumid;
  }
}

// 2. 메모리 효율적인 스크롤 텍스트
void scrollText(String text, int row, int startCol, int width) {
  static int scrollPos = 0;
  static unsigned long lastScroll = 0;
  
  if (millis() - lastScroll > 300) {  // 스크롤 속도 조절
    lcd.setCursor(startCol, row);
    
    if (text.length() <= width) {
      lcd.print(text);
    } else {
      String displayText = text.substring(scrollPos, scrollPos + width);
      if (displayText.length() < width) {
        displayText += text.substring(0, width - displayText.length());
      }
      lcd.print(displayText);
      
      scrollPos = (scrollPos + 1) % text.length();
    }
    
    lastScroll = millis();
  }
}

// 3. 고속 숫자 업데이트
void fastNumberUpdate(int value, int x, int y, int digits) {
  lcd.setCursor(x, y);
  
  // 자릿수 맞춤 (예: 3자리면 001, 010, 100)
  if (digits == 3) {
    if (value < 100) lcd.print("0");
    if (value < 10) lcd.print("0");
  } else if (digits == 2) {
    if (value < 10) lcd.print("0");
  }
  
  lcd.print(value);
}
```

## 🔧 문제 해결 가이드

### 일반적인 LCD 문제들

1. **LCD에 아무것도 표시되지 않는 경우**
   - 전원 연결 확인 (VDD = +5V, VSS = GND)
   - 대비 조절 (V0 핀에 가변저항 연결)
   - 코드에서 `lcd.begin(16, 2)` 확인

2. **문자가 깨져서 나오는 경우**
   - 4비트/8비트 모드 설정 확인
   - 배선 재점검 (특히 D4~D7)
   - 전원 전압 안정성 확인

3. **I2C LCD가 동작하지 않는 경우**
   - I2C 주소 확인 (`i2cscanner` 예제 사용)
   - SDA/SCL 핀 연결 확인
   - 풀업 저항 필요 (보통 내장)

### 성능 최적화 팁

```cpp
// 효율적인 LCD 사용법
class EfficientLCD {
private:
  LiquidCrystal_I2C* lcd;
  String displayBuffer[4];  // 화면 버퍼
  bool needUpdate[4];       // 업데이트 플래그
  
public:
  EfficientLCD(LiquidCrystal_I2C* display) : lcd(display) {
    for (int i = 0; i < 4; i++) {
      displayBuffer[i] = "";
      needUpdate[i] = true;
    }
  }
  
  void setText(int row, String text) {
    if (displayBuffer[row] != text) {
      displayBuffer[row] = text;
      needUpdate[row] = true;
    }
  }
  
  void update() {
    for (int i = 0; i < 4; i++) {
      if (needUpdate[i]) {
        lcd->setCursor(0, i);
        lcd->print(displayBuffer[i]);
        // 라인 끝 클리어
        for (int j = displayBuffer[i].length(); j < 20; j++) {
          lcd->print(" ");
        }
        needUpdate[i] = false;
      }
    }
  }
};
```

## 🚀 다음 단계

이제 LCD 디스플레이 제어를 완전히 마스터했습니다! 다음 강의에서는:

- **Lesson 11**: I2C와 SPI 통신 프로토콜
- **Lesson 12**: 무선 통신 (블루투스, WiFi)
- **Lesson 13**: 데이터 로깅 및 SD카드
- **Lesson 14**: 전력 관리 및 저전력 설계
- **Lesson 15**: 실전 프로젝트 - IoT 스마트홈

LCD를 활용한 다양한 프로젝트를 직접 만들어보세요. 사용자 인터페이스가 있는 모든 임베디드 시스템의 기초가 됩니다!

---

*아두이노 마스터 과정 - Lesson 10 완료*  
*Created by Arduino Expert Teacher*  
*© 2024 Arduino Complete Guide*