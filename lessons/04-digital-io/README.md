# Lesson 04: 디지털 입출력 제어

## 🎯 학습 목표
- 디지털 입력과 출력의 개념 이해하기
- 푸시 버튼으로 입력 받기
- digitalRead() 함수 사용법 익히기
- 풀업/풀다운 저항의 원리 이해하기
- 여러 입출력을 조합한 시스템 만들기

## 📖 1. 디지털 입출력의 이해

### 1.1 디지털 신호란?
디지털 신호는 **HIGH(1)** 또는 **LOW(0)** 두 가지 상태만 가지는 신호입니다.

```
디지털 신호의 특징:
┌─────┐     ┌─────┐     HIGH (5V, 논리 1)
│     │     │     │
│     │     │     │
└─────┘     └─────┘     LOW (0V, 논리 0)

시간 →

장점: 노이즈에 강함, 정확한 판단
단점: 중간값 표현 불가
```

### 1.2 입력과 출력의 구분

#### 출력 (OUTPUT)
- 아두이노가 외부로 신호를 보내는 것
- LED, 부저, 모터 등을 제어
- `digitalWrite(핀, HIGH/LOW)` 사용

#### 입력 (INPUT)
- 외부에서 아두이노로 신호를 받는 것
- 버튼, 스위치, 센서 등에서 신호 수신
- `digitalRead(핀)` 사용

## 🔘 2. 푸시 버튼 입력

### 2.1 푸시 버튼의 구조
```
    푸시 버튼 내부 구조
    
    누르지 않은 상태:        눌렀을 때:
    ┌─ A ──┐ ┌─ B ──┐      ┌─ A ──┐ ┌─ B ──┐
    │      │ │      │      │      │ │      │
    │  ●   │ │   ●  │      │  ●───┼─┼───●  │
    │      │ │      │      │      │ │      │
    └─ C ──┘ └─ D ──┘      └─ C ──┘ └─ D ──┘
    
    A-C, B-D 항상 연결됨     A-B, C-D도 연결됨
```

### 2.2 기본 버튼 회로 (외부 풀다운)
```
아두이노               브레드보드
┌─────────┐           ┌─────────────────────┐
│         │           │                     │
│    5V ●─┼─────────────● ┌─────┐           │
│         │           │   │버튼 │           │
│         │           │   └──┬──┘           │
│     2 ●─┼─────────────●────┘              │
│         │           │ ↑                   │
│         │           │ │  10kΩ             │
│   GND ●─┼───────────────●─────────────────● │
│         │           │                     │
└─────────┘           └─────────────────────┘

버튼을 누르지 않으면: LOW (0V)
버튼을 누르면: HIGH (5V)
```

### 2.3 내부 풀업 저항 사용 (권장)
```
아두이노만으로 간단한 연결:
┌─────────┐
│    2 ●──┼────┐
│         │    │ 푸시버튼
│   GND ●─┼────┘
│         │
└─────────┘

pinMode(2, INPUT_PULLUP); 사용
버튼을 누르지 않으면: HIGH
버튼을 누르면: LOW (반대 논리)
```

## 💻 3. 기본 버튼 제어 코드

### 3.1 외부 풀다운 저항 방식
```cpp
/*
  Button_External_Pulldown.ino
  외부 풀다운 저항을 사용한 버튼 입력
*/

const int BUTTON_PIN = 2;  // 버튼이 연결된 핀
const int LED_PIN = 13;    // LED가 연결된 핀

void setup() {
  pinMode(BUTTON_PIN, INPUT);   // 버튼 핀을 입력으로 설정
  pinMode(LED_PIN, OUTPUT);     // LED 핀을 출력으로 설정
  
  Serial.begin(9600);
  Serial.println("버튼 테스트 시작!");
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);  // 버튼 상태 읽기
  
  if (buttonState == HIGH) {
    digitalWrite(LED_PIN, HIGH);  // 버튼을 누르면 LED 켜기
    Serial.println("버튼 눌림 - LED ON");
  } else {
    digitalWrite(LED_PIN, LOW);   // 버튼을 떼면 LED 끄기
    Serial.println("버튼 떼임 - LED OFF");
  }
  
  delay(100);  // 디바운싱을 위한 짧은 지연
}
```

### 3.2 내부 풀업 저항 방식 (권장)
```cpp
/*
  Button_Internal_Pullup.ino
  내부 풀업 저항을 사용한 버튼 입력 (권장 방식)
*/

const int BUTTON_PIN = 2;
const int LED_PIN = 13;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // 내부 풀업 저항 사용
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("내부 풀업 버튼 테스트 시작!");
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);
  
  // 내부 풀업에서는 논리가 반대 (누르면 LOW)
  if (buttonState == LOW) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("버튼 눌림 - LED ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("버튼 떼임 - LED OFF");
  }
  
  delay(100);
}
```

## ⚡ 4. 디바운싱 (Debouncing)

### 4.1 채터링 현상이란?
```
버튼을 한 번 눌렀을 때 실제 신호:
           이상적인 신호    실제 신호 (채터링)
           ┌────────┐      ┌─┐┌┐┌─────┐
           │        │      │ ││││     │
    ───────┘        └───   └─┘└┘└─────┘
    
    → 한 번 누른 것이 여러 번으로 인식됨
```

### 4.2 소프트웨어 디바운싱
```cpp
/*
  Button_Debouncing.ino
  디바운싱을 적용한 버튼 제어
*/

const int BUTTON_PIN = 2;
const int LED_PIN = 13;

// 디바운싱 변수들
int lastButtonState = HIGH;      // 이전 버튼 상태
int currentButtonState = HIGH;   // 현재 버튼 상태
unsigned long lastDebounceTime = 0;  // 마지막 토글 시간
const unsigned long DEBOUNCE_DELAY = 50;  // 디바운스 지연 시간 (ms)

// LED 상태 관리
bool ledState = false;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("디바운싱 버튼 테스트 시작!");
}

void loop() {
  // 현재 버튼 상태 읽기
  int reading = digitalRead(BUTTON_PIN);
  
  // 버튼 상태가 변경되었다면 디바운스 타이머 리셋
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // 디바운스 시간이 지났다면
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // 버튼 상태가 실제로 변경되었다면
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      // 버튼이 눌렸다면 (HIGH → LOW)
      if (currentButtonState == LOW) {
        ledState = !ledState;  // LED 상태 토글
        digitalWrite(LED_PIN, ledState);
        Serial.print("버튼 클릭! LED 상태: ");
        Serial.println(ledState ? "ON" : "OFF");
      }
    }
  }
  
  lastButtonState = reading;
}
```

## 🎮 5. 다중 입출력 제어

### 5.1 3개 버튼으로 3개 LED 제어
```cpp
/*
  Multiple_Button_LED.ino
  여러 버튼으로 여러 LED 제어
*/

// 핀 정의
const int BUTTON_PINS[] = {2, 3, 4};    // 버튼 핀들
const int LED_PINS[] = {11, 12, 13};    // LED 핀들
const int NUM_BUTTONS = 3;              // 버튼 개수

// 상태 관리 배열
bool ledStates[] = {false, false, false};
int lastButtonStates[] = {HIGH, HIGH, HIGH};
unsigned long lastDebounceTimes[] = {0, 0, 0};
const unsigned long DEBOUNCE_DELAY = 50;

void setup() {
  // 버튼과 LED 핀 설정
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    pinMode(LED_PINS[i], OUTPUT);
  }
  
  Serial.begin(9600);
  Serial.println("다중 버튼-LED 제어 시작!");
}

void loop() {
  // 각 버튼 확인
  for (int i = 0; i < NUM_BUTTONS; i++) {
    int reading = digitalRead(BUTTON_PINS[i]);
    
    // 디바운싱 처리
    if (reading != lastButtonStates[i]) {
      lastDebounceTimes[i] = millis();
    }
    
    if ((millis() - lastDebounceTimes[i]) > DEBOUNCE_DELAY) {
      if (reading == LOW && lastButtonStates[i] == HIGH) {
        // 버튼이 눌렸을 때
        ledStates[i] = !ledStates[i];
        digitalWrite(LED_PINS[i], ledStates[i]);
        
        Serial.print("버튼 ");
        Serial.print(i + 1);
        Serial.print(" 클릭! LED ");
        Serial.print(i + 1);
        Serial.print(" 상태: ");
        Serial.println(ledStates[i] ? "ON" : "OFF");
      }
    }
    
    lastButtonStates[i] = reading;
  }
}
```

### 5.2 버튼으로 LED 패턴 제어
```cpp
/*
  Button_Pattern_Control.ino
  버튼으로 LED 패턴 변경하기
*/

const int BUTTON_PIN = 2;
const int LED_PINS[] = {9, 10, 11, 12, 13};
const int NUM_LEDS = 5;

int currentPattern = 0;
const int NUM_PATTERNS = 4;

// 디바운싱 변수
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// 패턴 제어 변수
unsigned long previousMillis = 0;
int patternStep = 0;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }
  
  Serial.begin(9600);
  Serial.println("LED 패턴 제어 시작!");
  Serial.println("버튼을 눌러 패턴을 변경하세요.");
}

void loop() {
  handleButton();
  runPattern();
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading == LOW) {
      currentPattern = (currentPattern + 1) % NUM_PATTERNS;
      patternStep = 0;  // 패턴 스텝 리셋
      clearAllLEDs();
      
      Serial.print("패턴 변경: ");
      Serial.println(currentPattern);
    }
  }
  
  lastButtonState = reading;
}

void runPattern() {
  unsigned long currentMillis = millis();
  
  switch (currentPattern) {
    case 0:  // 순차 점등
      if (currentMillis - previousMillis >= 200) {
        clearAllLEDs();
        digitalWrite(LED_PINS[patternStep], HIGH);
        patternStep = (patternStep + 1) % NUM_LEDS;
        previousMillis = currentMillis;
      }
      break;
      
    case 1:  // 전체 깜빡임
      if (currentMillis - previousMillis >= 500) {
        for (int i = 0; i < NUM_LEDS; i++) {
          digitalWrite(LED_PINS[i], patternStep % 2);
        }
        patternStep++;
        previousMillis = currentMillis;
      }
      break;
      
    case 2:  // 양쪽에서 중앙으로
      if (currentMillis - previousMillis >= 300) {
        clearAllLEDs();
        if (patternStep < NUM_LEDS / 2 + 1) {
          digitalWrite(LED_PINS[patternStep], HIGH);
          digitalWrite(LED_PINS[NUM_LEDS - 1 - patternStep], HIGH);
          patternStep++;
        } else {
          patternStep = 0;
        }
        previousMillis = currentMillis;
      }
      break;
      
    case 3:  // 랜덤 패턴
      if (currentMillis - previousMillis >= 150) {
        for (int i = 0; i < NUM_LEDS; i++) {
          digitalWrite(LED_PINS[i], random(0, 2));
        }
        previousMillis = currentMillis;
      }
      break;
  }
}

void clearAllLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(LED_PINS[i], LOW);
  }
}
```

## 🔧 6. 회로 연결 가이드

### 6.1 단일 버튼 + LED 회로
```
Arduino Uno                브레드보드
┌──────────┐              ┌─────────────────────────┐
│          │              │  a  b  c  d  e  f  g  h │
│     2 ●──┼──────────────│5 ●                     │
│          │              │  ↑                     │
│          │              │  │ 푸시버튼             │
│   GND ●──┼──────────────│10●                     │
│          │              │                         │
│    13 ●──┼──────────────│15●━━━━●                 │
│          │              │      220Ω              │
│          │              │        ↓               │
│          │              │20      ●  LED(+)       │
│          │              │        ↓               │
│   GND ●──┼──────────────│25      ●  LED(-)       │
│          │              │                         │
└──────────┘              └─────────────────────────┘
```

### 6.2 다중 버튼 + LED 회로
```
3개 버튼 + 3개 LED 연결도:

버튼들:               LED들:
2번 핀 ── 버튼1       11번 핀 ── 저항 ── LED1(빨강)
3번 핀 ── 버튼2       12번 핀 ── 저항 ── LED2(노랑)  
4번 핀 ── 버튼3       13번 핀 ── 저항 ── LED3(초록)

모든 버튼의 다른쪽 → GND
모든 LED의 (-) → GND
```

## 🛠️ 7. 주요 함수 정리

### 7.1 디지털 입력 함수
```cpp
// 핀 모드 설정
pinMode(핀번호, INPUT);        // 일반 입력
pinMode(핀번호, INPUT_PULLUP); // 내부 풀업 저항 사용

// 디지털 값 읽기
int value = digitalRead(핀번호);
// 반환값: HIGH(1) 또는 LOW(0)

// 시간 관련 함수
unsigned long time = millis();  // 프로그램 시작 후 경과 시간(ms)
delay(시간);                    // 지정 시간만큼 대기
```

### 7.2 논리 연산자
```cpp
// 비교 연산자
==  // 같음
!=  // 다름
>   // 크다
<   // 작다

// 논리 연산자
&&  // AND (그리고)
||  // OR (또는)
!   // NOT (부정)

// 예제
if (buttonState == LOW && ledState == false) {
  // 버튼이 눌렸고 LED가 꺼져있을 때
}
```

## ⚠️ 8. 문제 해결 가이드

### 8.1 자주 발생하는 문제들

| 문제 | 원인 | 해결방법 |
|-----|------|----------|
| 버튼이 반응하지 않음 | 연결 불량, 핀 설정 오류 | 연결 재확인, pinMode 확인 |
| 한 번 눌렀는데 여러 번 인식 | 채터링 현상 | 디바운싱 코드 적용 |
| 버튼 안눌러도 LED 켜짐 | 플로팅 상태 | 풀업/풀다운 저항 사용 |
| 논리가 반대로 동작 | 풀업 저항 사용시 | 조건문에서 LOW 확인 |

### 8.2 디버깅 팁
```cpp
// 시리얼 모니터로 상태 확인
Serial.print("버튼 상태: ");
Serial.println(digitalRead(BUTTON_PIN));

// 변수 값 모니터링  
Serial.print("LED 상태: ");
Serial.println(ledState ? "ON" : "OFF");

// 시간 측정
Serial.print("경과 시간: ");
Serial.println(millis());
```

## ✅ 핵심 정리

1. **디지털 입력**은 digitalRead()로 HIGH/LOW 값을 읽습니다.
2. **풀업 저항**은 플로팅 상태를 방지하여 안정적인 입력을 제공합니다.
3. **디바운싱**은 버튼의 채터링 현상을 해결하는 필수 기법입니다.
4. **배열과 반복문**을 사용하면 다중 입출력을 효율적으로 제어할 수 있습니다.

## 🚀 다음 단계

다음 레슨에서는 아날로그 입출력과 PWM을 배워서 
더 정교한 제어와 다양한 센서 활용법을 익혀보겠습니다.

## 💻 실습 과제

### 과제 1: 기본 버튼 제어
1. 버튼 2개로 각각 다른 LED 제어하기
2. 한 버튼으로 LED ON/OFF, 다른 버튼으로 깜빡임 속도 조절
3. 디바운싱 적용해서 정확한 입력 처리하기

### 과제 2: 고급 패턴 제어
1. 버튼 길게 누르기와 짧게 누르기 구분하기
2. 더블 클릭 감지 기능 구현하기
3. 버튼 조합으로 다양한 LED 패턴 만들기

### 과제 3: 실생활 응용
1. 디지털 주사위 만들기 (버튼 누르면 1-6 LED로 표시)
2. 간단한 모스 부호 입력기 (길게/짧게 눌러서 입력)
3. 반응속도 테스트 게임 (LED 켜지면 빨리 버튼 누르기)

---

[← 이전 레슨: LED 깜빡이기](../03-first-program-blink/README.md) | [다음 레슨: 아날로그 입출력과 PWM →](../05-analog-io-pwm/README.md)