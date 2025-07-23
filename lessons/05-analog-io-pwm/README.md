# Lesson 05: 아날로그 입출력과 PWM

## 🎯 학습 목표
- 아날로그와 디지털의 차이점 이해하기
- analogRead()로 아날로그 센서 값 읽기
- PWM을 이용한 아날로그 출력 구현하기
- 가변저항, 조도센서, 온도센서 활용하기
- LED 밝기 조절과 모터 속도 제어하기

## 📖 1. 아날로그 vs 디지털

### 1.1 개념 비교
```
디지털 신호:                아날로그 신호:
┌─────┐   ┌─────┐          ╭─╮
│  1  │   │  1  │         ╱   ╲    ╭─╮
│     │   │     │        ╱     ╲  ╱   ╲
└─────┘   └─────┘       ╱       ╲╱     ╲
  0       0             0                 ╲
                                          ╲___

특징:                     특징:
- 0 또는 1 (HIGH/LOW)     - 연속적인 값 변화
- 노이즈에 강함           - 실세계와 유사
- 정확한 값               - 부드러운 제어 가능
```

### 1.2 아두이노에서의 처리
- **아날로그 입력**: 0~5V → 0~1023 (10비트 ADC)
- **아날로그 출력**: PWM으로 유사 아날로그 구현
- **해상도**: 입력 10비트, 출력 8비트

## 📊 2. 아날로그 입력 (analogRead)

### 2.1 ADC (Analog-to-Digital Converter)
```
실제 전압  →  ADC  →  디지털 값
0V        →       →  0
1.25V     →       →  255
2.5V      →       →  512
3.75V     →       →  767
5V        →       →  1023

변환 공식:
디지털값 = (입력전압 / 5V) × 1023
입력전압 = (디지털값 / 1023) × 5V
```

### 2.2 아날로그 핀 사용법
```cpp
// 기본 문법
int value = analogRead(A0);  // A0 핀에서 아날로그 값 읽기

// 전압으로 변환
float voltage = (value / 1023.0) * 5.0;

// 백분율로 변환  
int percentage = map(value, 0, 1023, 0, 100);
```

### 2.3 가변저항 예제
```cpp
/*
  Potentiometer_Basic.ino
  가변저항으로 LED 밝기 조절하기
*/

const int POT_PIN = A0;     // 가변저항 핀
const int LED_PIN = 9;      // PWM 지원 핀

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("가변저항 LED 밝기 조절 시작!");
}

void loop() {
  // 가변저항 값 읽기 (0~1023)
  int potValue = analogRead(POT_PIN);
  
  // PWM 값으로 변환 (0~255)
  int ledBrightness = map(potValue, 0, 1023, 0, 255);
  
  // LED 밝기 조절
  analogWrite(LED_PIN, ledBrightness);
  
  // 시리얼 모니터 출력
  Serial.print("가변저항 값: ");
  Serial.print(potValue);
  Serial.print(" → LED 밝기: ");
  Serial.print(ledBrightness);
  Serial.print(" (");
  Serial.print(map(potValue, 0, 1023, 0, 100));
  Serial.println("%)");
  
  delay(100);
}
```

## ⚡ 3. PWM (Pulse Width Modulation)

### 3.1 PWM 원리
```
PWM 신호의 구조:

Duty Cycle 25%:          Duty Cycle 50%:          Duty Cycle 75%:
┌─┐   ┌─┐   ┌─┐         ┌──┐  ┌──┐  ┌──┐        ┌───┐ ┌───┐ ┌───┐
│ │   │ │   │ │         │  │  │  │  │  │        │   │ │   │ │   │
└─┘   └─┘   └─┘         └──┘  └──┘  └──┘        └───┘ └───┘ └───┘
평균 전압: 1.25V         평균 전압: 2.5V         평균 전압: 3.75V

analogWrite(pin, 64);    analogWrite(pin, 128);   analogWrite(pin, 192);
```

### 3.2 PWM 핀과 주파수
```
Arduino Uno PWM 핀:
- 3, 5, 6, 9, 10, 11번 핀
- 기본 주파수: 490Hz (6, 5번은 980Hz)
- 분해능: 8비트 (0~255)

사용법:
analogWrite(핀번호, 0~255);
```

### 3.3 RGB LED 색상 제어
```cpp
/*
  RGB_Color_Control.ino
  RGB LED로 다양한 색상 만들기
*/

const int RED_PIN = 9;
const int GREEN_PIN = 10;
const int BLUE_PIN = 11;

// 색상 배열 (R, G, B)
int colors[][3] = {
  {255, 0, 0},    // 빨강
  {0, 255, 0},    // 초록
  {0, 0, 255},    // 파랑
  {255, 255, 0},  // 노랑
  {255, 0, 255},  // 자홍
  {0, 255, 255},  // 청록
  {255, 255, 255}, // 흰색
  {255, 128, 0},  // 주황
  {128, 0, 128},  // 보라
  {255, 192, 203}  // 분홍
};

const int NUM_COLORS = sizeof(colors) / sizeof(colors[0]);

void setup() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("RGB LED 색상 제어 시작!");
}

void loop() {
  // 각 색상을 순차적으로 표시
  for (int i = 0; i < NUM_COLORS; i++) {
    setColor(colors[i][0], colors[i][1], colors[i][2]);
    
    Serial.print("색상 ");
    Serial.print(i + 1);
    Serial.print(": R=");
    Serial.print(colors[i][0]);
    Serial.print(", G=");
    Serial.print(colors[i][1]);
    Serial.print(", B=");
    Serial.println(colors[i][2]);
    
    delay(1000);
  }
  
  // 색상 페이드 효과
  rainbowFade();
}

void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

void rainbowFade() {
  Serial.println("무지개 페이드 효과 시작!");
  
  for (int i = 0; i < 255; i++) {
    // 무지개 색상 계산
    int red = (sin(i * 0.024) + 1) * 127;
    int green = (sin(i * 0.024 + 2) + 1) * 127;
    int blue = (sin(i * 0.024 + 4) + 1) * 127;
    
    setColor(red, green, blue);
    delay(20);
  }
}
```

## 🌡️ 4. 아날로그 센서 활용

### 4.1 조도센서 (LDR)
```cpp
/*
  Light_Sensor.ino
  조도센서로 자동 조명 제어
*/

const int LDR_PIN = A0;
const int LED_PIN = 9;

// 조도 임계값 설정
const int DARK_THRESHOLD = 300;   // 어두움 기준값
const int BRIGHT_THRESHOLD = 700; // 밝음 기준값

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("자동 조명 시스템 시작!");
}

void loop() {
  int lightLevel = analogRead(LDR_PIN);
  
  // 조도에 따른 LED 밝기 자동 조절
  int ledBrightness;
  
  if (lightLevel < DARK_THRESHOLD) {
    // 어두우면 LED 밝게
    ledBrightness = map(lightLevel, 0, DARK_THRESHOLD, 255, 128);
    ledBrightness = constrain(ledBrightness, 128, 255);
  } else if (lightLevel > BRIGHT_THRESHOLD) {
    // 밝으면 LED 끄기
    ledBrightness = 0;
  } else {
    // 중간 밝기는 점진적 조절
    ledBrightness = map(lightLevel, DARK_THRESHOLD, BRIGHT_THRESHOLD, 128, 0);
  }
  
  analogWrite(LED_PIN, ledBrightness);
  
  // 상태 출력
  Serial.print("조도: ");
  Serial.print(lightLevel);
  Serial.print(" → LED 밝기: ");
  Serial.print(ledBrightness);
  Serial.print(" (");
  if (lightLevel < DARK_THRESHOLD) {
    Serial.println("어두움)");
  } else if (lightLevel > BRIGHT_THRESHOLD) {
    Serial.println("밝음)");
  } else {
    Serial.println("보통)");
  }
  
  delay(200);
}
```

### 4.2 온도센서 (LM35)
```cpp
/*
  Temperature_Sensor.ino
  LM35 온도센서로 온도 측정하기
*/

const int TEMP_PIN = A1;
const int FAN_PIN = 6;        // 팬 제어용 PWM 핀
const int HEATER_PIN = 7;     // 히터 제어용 디지털 핀

// 온도 제어 설정
const float TARGET_TEMP = 25.0;      // 목표 온도 (°C)
const float TEMP_TOLERANCE = 2.0;    // 온도 허용 오차

void setup() {
  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("온도 제어 시스템 시작!");
  Serial.print("목표 온도: ");
  Serial.print(TARGET_TEMP);
  Serial.println("°C");
}

void loop() {
  // LM35 온도 읽기
  int sensorValue = analogRead(TEMP_PIN);
  float voltage = (sensorValue / 1023.0) * 5.0;
  float temperature = voltage * 100.0;  // LM35: 10mV/°C
  
  // PID 제어 대신 간단한 온오프 제어
  float tempError = temperature - TARGET_TEMP;
  
  if (tempError > TEMP_TOLERANCE) {
    // 너무 뜨거우면 팬 켜기
    int fanSpeed = map(abs(tempError), 0, 20, 100, 255);
    fanSpeed = constrain(fanSpeed, 0, 255);
    analogWrite(FAN_PIN, fanSpeed);
    digitalWrite(HEATER_PIN, LOW);
    
    Serial.print("냉각 모드 - 팬 속도: ");
    Serial.println(fanSpeed);
    
  } else if (tempError < -TEMP_TOLERANCE) {
    // 너무 차가우면 히터 켜기
    analogWrite(FAN_PIN, 0);
    digitalWrite(HEATER_PIN, HIGH);
    
    Serial.println("가열 모드 - 히터 ON");
    
  } else {
    // 적정 온도면 모두 끄기
    analogWrite(FAN_PIN, 0);
    digitalWrite(HEATER_PIN, LOW);
    
    Serial.println("온도 적정 - 시스템 대기");
  }
  
  // 온도 정보 출력
  Serial.print("현재 온도: ");
  Serial.print(temperature, 1);
  Serial.print("°C (오차: ");
  Serial.print(tempError, 1);
  Serial.println("°C)");
  
  delay(1000);
}
```

## 🎵 5. 소리와 음악

### 5.1 아날로그 방식 톤 생성
```cpp
/*
  Analog_Music.ino
  PWM을 이용한 음악 재생
*/

const int SPEAKER_PIN = 9;
const int POT_PIN = A0;

// 음계 주파수 (Hz)
const int NOTE_C4 = 262;
const int NOTE_D4 = 294;
const int NOTE_E4 = 330;
const int NOTE_F4 = 349;
const int NOTE_G4 = 392;
const int NOTE_A4 = 440;
const int NOTE_B4 = 494;
const int NOTE_C5 = 523;

// 도레미파솔라시도 멜로디
int melody[] = {
  NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4,
  NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5
};

const int MELODY_LENGTH = sizeof(melody) / sizeof(melody[0]);

void setup() {
  pinMode(SPEAKER_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("아날로그 음악 재생기 시작!");
}

void loop() {
  int potValue = analogRead(POT_PIN);
  
  // 가변저항으로 음계 선택
  int noteIndex = map(potValue, 0, 1023, 0, MELODY_LENGTH - 1);
  int frequency = melody[noteIndex];
  
  // PWM으로 톤 생성
  generateTone(SPEAKER_PIN, frequency, 200);
  
  Serial.print("음계: ");
  Serial.print(noteIndex + 1);
  Serial.print(" (");
  Serial.print(frequency);
  Serial.println("Hz)");
  
  delay(250);
}

void generateTone(int pin, int frequency, int duration) {
  long period = 1000000L / frequency;
  long pulse = period / 2;
  
  for (long i = 0; i < duration * 1000L; i += period) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(pin, LOW);
    delayMicroseconds(pulse);
  }
}
```

## 🚗 6. 모터 속도 제어

### 6.1 DC 모터 PWM 제어
```cpp
/*
  DC_Motor_Control.ino
  DC 모터의 속도와 방향 제어
*/

// L298N 모터 드라이버 핀 연결
const int MOTOR_SPEED_PIN = 9;    // ENA 핀 (PWM)
const int MOTOR_DIR1_PIN = 7;     // IN1 핀
const int MOTOR_DIR2_PIN = 8;     // IN2 핀
const int POT_PIN = A0;           // 속도 조절 가변저항

void setup() {
  pinMode(MOTOR_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_DIR1_PIN, OUTPUT);
  pinMode(MOTOR_DIR2_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("DC 모터 제어 시작!");
  Serial.println("가변저항으로 속도와 방향을 조절하세요.");
}

void loop() {
  int potValue = analogRead(POT_PIN);
  
  // 중앙값(512) 기준으로 방향과 속도 결정
  int centerValue = 512;
  int deadZone = 50;  // 정지 구간
  
  if (potValue > centerValue + deadZone) {
    // 전진
    int speed = map(potValue, centerValue + deadZone, 1023, 0, 255);
    motorForward(speed);
    
    Serial.print("전진 - 속도: ");
    Serial.println(speed);
    
  } else if (potValue < centerValue - deadZone) {
    // 후진
    int speed = map(potValue, 0, centerValue - deadZone, 255, 0);
    motorBackward(speed);
    
    Serial.print("후진 - 속도: ");
    Serial.println(speed);
    
  } else {
    // 정지
    motorStop();
    Serial.println("정지");
  }
  
  delay(100);
}

void motorForward(int speed) {
  digitalWrite(MOTOR_DIR1_PIN, HIGH);
  digitalWrite(MOTOR_DIR2_PIN, LOW);
  analogWrite(MOTOR_SPEED_PIN, speed);
}

void motorBackward(int speed) {
  digitalWrite(MOTOR_DIR1_PIN, LOW);
  digitalWrite(MOTOR_DIR2_PIN, HIGH);
  analogWrite(MOTOR_SPEED_PIN, speed);
}

void motorStop() {
  digitalWrite(MOTOR_DIR1_PIN, LOW);
  digitalWrite(MOTOR_DIR2_PIN, LOW);
  analogWrite(MOTOR_SPEED_PIN, 0);
}
```

## 📊 7. 데이터 로깅과 시각화

### 7.1 센서 데이터 수집
```cpp
/*
  Data_Logger.ino
  여러 센서 데이터를 동시에 수집하고 기록
*/

const int TEMP_PIN = A0;
const int LIGHT_PIN = A1;
const int SOUND_PIN = A2;
const int POT_PIN = A3;

unsigned long previousMillis = 0;
const long SAMPLE_INTERVAL = 1000;  // 1초마다 샘플링

void setup() {
  Serial.begin(9600);
  Serial.println("다중 센서 데이터 로거 시작!");
  Serial.println("시간(ms),온도(°C),조도,소음,가변저항");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // 지정된 간격마다 데이터 수집
  if (currentMillis - previousMillis >= SAMPLE_INTERVAL) {
    previousMillis = currentMillis;
    
    // 각 센서 데이터 읽기
    float temperature = readTemperature();
    int lightLevel = analogRead(LIGHT_PIN);
    int soundLevel = analogRead(SOUND_PIN);
    int potValue = analogRead(POT_PIN);
    
    // CSV 형식으로 데이터 출력
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(temperature, 2);
    Serial.print(",");
    Serial.print(lightLevel);
    Serial.print(",");
    Serial.print(soundLevel);
    Serial.print(",");
    Serial.println(potValue);
    
    // 간단한 통계 계산
    static float tempSum = 0;
    static int sampleCount = 0;
    
    tempSum += temperature;
    sampleCount++;
    
    if (sampleCount % 10 == 0) {  // 10초마다 평균 출력
      Serial.print("# 평균 온도 (최근 10초): ");
      Serial.print(tempSum / 10, 2);
      Serial.println("°C");
      tempSum = 0;
    }
  }
  
  // 실시간 경고 시스템
  checkAlerts();
}

float readTemperature() {
  int sensorValue = analogRead(TEMP_PIN);
  float voltage = (sensorValue / 1023.0) * 5.0;
  return voltage * 100.0;  // LM35 온도 변환
}

void checkAlerts() {
  static unsigned long lastAlert = 0;
  
  if (millis() - lastAlert > 5000) {  // 5초마다 경고 확인
    float temp = readTemperature();
    int light = analogRead(LIGHT_PIN);
    int sound = analogRead(SOUND_PIN);
    
    // 경고 조건 확인
    if (temp > 30.0) {
      Serial.println("⚠️ 경고: 온도가 너무 높습니다!");
    }
    if (light < 100) {
      Serial.println("⚠️ 경고: 조도가 너무 낮습니다!");
    }
    if (sound > 800) {
      Serial.println("⚠️ 경고: 소음이 너무 큽니다!");
    }
    
    lastAlert = millis();
  }
}
```

## 🔧 8. 회로 연결 가이드

### 8.1 기본 아날로그 센서 연결
```
가변저항 연결:
┌─────────────┐
│   Arduino   │     가변저항
│             │     ┌─────┐
│    5V ●─────┼─────┤  1  │
│             │     │     │
│    A0 ●─────┼─────┤  2  │ (중앙핀)
│             │     │     │
│   GND ●─────┼─────┤  3  │
│             │     └─────┘
└─────────────┘

조도센서 연결:
5V ── LDR ── A0 ── 10kΩ저항 ── GND
```

### 8.2 PWM 출력 연결
```
LED 밝기 조절:
Arduino 9번핀 ── 220Ω저항 ── LED(+) ── LED(-) ── GND

RGB LED 연결:
Arduino 9번핀 ── 220Ω ── LED 빨강(+)
Arduino 10번핀 ── 220Ω ── LED 초록(+)
Arduino 11번핀 ── 220Ω ── LED 파랑(+)
LED 공통(-) ── GND
```

## 🛠️ 9. 주요 함수 정리

### 9.1 아날로그 입출력 함수
```cpp
// 아날로그 입력
int value = analogRead(A0);        // 0~1023 값 반환

// 아날로그 출력 (PWM)
analogWrite(9, 255);               // 0~255 값 출력

// 값 변환 함수
int mapped = map(value, 0, 1023, 0, 255);  // 범위 변환
int limited = constrain(value, 0, 255);    // 범위 제한

// 수학 함수
float voltage = (value / 1023.0) * 5.0;   // 전압 변환
```

### 9.2 유용한 수학 함수
```cpp
// 최대/최소값
int maximum = max(a, b);
int minimum = min(a, b);

// 절대값
int absolute = abs(value);

// 제곱근
float squareRoot = sqrt(value);

// 삼각함수 (라디안)
float sine = sin(angle);
float cosine = cos(angle);
```

## ✅ 핵심 정리

1. **아날로그 입력**은 연속적인 값을 읽을 수 있어 센서 활용에 필수입니다.
2. **PWM**은 디지털 신호로 아날로그 출력을 구현하는 핵심 기술입니다.
3. **map() 함수**는 서로 다른 범위의 값을 변환할 때 매우 유용합니다.
4. **센서 보정**과 **필터링**은 정확한 측정을 위해 중요합니다.

## 🚀 다음 단계

이제 기본적인 입출력을 모두 마스터했습니다!
다음 레슨에서는 시리얼 통신을 배워서 더 복잡한 데이터 처리와 
외부 장치와의 통신 방법을 알아보겠습니다.

## 💻 실습 과제

### 과제 1: 센서 조합
1. 온도와 습도를 동시에 측정하는 시스템 만들기
2. 조도센서로 자동 커튼 제어 시스템 구현
3. 소음 측정기 만들어서 데시벨 표시하기

### 과제 2: PWM 활용
1. RGB LED로 무지개 색상 자동 변화 구현
2. 서보모터를 PWM으로 정밀 각도 제어
3. DC모터 부드러운 가속/감속 알고리즘 구현

### 과제 3: 통합 프로젝트  
1. 스마트 화분 (온도, 습도, 조도 측정 + 자동 물주기)
2. 홈 오토메이션 시스템 (여러 센서 + LED + 모터)
3. 날씨 모니터링 스테이션 (데이터 수집 + 시각화)

---

[← 이전 레슨: 디지털 입출력](../04-digital-io/README.md) | [다음 레슨: 시리얼 통신 →](../06-serial-communication/README.md)