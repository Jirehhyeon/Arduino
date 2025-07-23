# Lesson 03: 첫 번째 프로그램 - LED 깜빡이기

## 🎯 학습 목표
- 아두이노 프로그램의 기본 구조 이해하기
- digitalWrite() 함수로 LED 제어하기
- delay() 함수로 시간 제어하기
- 외부 LED 연결하고 제어하기

## 📖 1. 아두이노 프로그램의 기본 구조

### 1.1 스케치(Sketch)란?
아두이노에서는 프로그램을 **스케치(Sketch)**라고 부릅니다. 모든 스케치는 두 개의 필수 함수를 포함해야 합니다.

### 1.2 기본 구조
```cpp
// 전역 변수 선언 영역

void setup() {
  // 초기화 코드
  // 프로그램 시작시 한 번만 실행
}

void loop() {
  // 메인 코드
  // setup() 실행 후 계속 반복 실행
}
```

### 1.3 실행 흐름
```
전원 ON → setup() 실행 (1번) → loop() 실행 → loop() 실행 → loop() 실행 → ...
                                     ↑                    ↓
                                     ←────────────────────←
```

## 💡 2. 내장 LED 제어하기 (Blink)

### 2.1 하드웨어 이해
- Arduino Uno의 **13번 핀**에는 내장 LED가 연결되어 있습니다
- 이 LED는 **'L'**이라고 표시되어 있습니다
- HIGH(5V) 신호를 보내면 켜지고, LOW(0V) 신호를 보내면 꺼집니다

### 2.2 기본 Blink 코드
```cpp
/*
  Blink - LED 깜빡이기
  내장 LED를 1초 간격으로 켜고 끄는 프로그램
*/

void setup() {
  // 13번 핀을 출력 모드로 설정
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);   // LED 켜기 (5V 출력)
  delay(1000);              // 1000ms = 1초 대기
  digitalWrite(13, LOW);    // LED 끄기 (0V 출력)
  delay(1000);              // 1초 대기
}
```

### 2.3 코드 개선하기
```cpp
// LED_BUILTIN은 내장 LED 핀 번호를 나타내는 상수
const int ledPin = LED_BUILTIN;  // 대부분의 보드에서 13

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
}
```

## 🔧 3. 주요 함수 상세 설명

### 3.1 pinMode(pin, mode)
핀의 동작 모드를 설정합니다.

```cpp
// 문법
pinMode(핀번호, 모드);

// 모드 종류
OUTPUT      // 출력 모드 (0V 또는 5V 출력)
INPUT       // 입력 모드 (외부 신호 읽기)
INPUT_PULLUP // 내부 풀업 저항 사용 입력 모드

// 예제
pinMode(13, OUTPUT);    // 13번 핀을 출력으로 설정
pinMode(7, INPUT);      // 7번 핀을 입력으로 설정
```

### 3.2 digitalWrite(pin, value)
디지털 핀에 HIGH(5V) 또는 LOW(0V)를 출력합니다.

```cpp
// 문법
digitalWrite(핀번호, 값);

// 값 종류
HIGH  // 5V 출력 (LED 켜기)
LOW   // 0V 출력 (LED 끄기)
1     // HIGH와 동일
0     // LOW와 동일

// 예제
digitalWrite(13, HIGH);  // 13번 핀에 5V 출력
digitalWrite(13, LOW);   // 13번 핀에 0V 출력
```

### 3.3 delay(ms)
프로그램 실행을 일시 정지합니다.

```cpp
// 문법
delay(밀리초);

// 시간 단위 변환
1초 = 1000밀리초
0.5초 = 500밀리초
0.1초 = 100밀리초

// 예제
delay(1000);   // 1초 대기
delay(500);    // 0.5초 대기
delay(100);    // 0.1초 대기
```

## 🔌 4. 외부 LED 연결하기

### 4.1 필요한 부품
- LED 1개 (색상 무관)
- 저항 1개 (220Ω ~ 1kΩ)
- 점퍼 와이어 2개
- 브레드보드

### 4.2 LED의 구조
```
     LED 구조
    ───┬───
       │
    ┌──┴──┐
    │ LED │
    └─┬─┬─┘
      │ │
    긴쪽 짧은쪽
    (+)  (-)
   애노드 캐소드
```

### 4.3 회로 연결
```
Arduino Uno                     브레드보드
                               
   13 ─────────────────┐
                       │
                      ┌┴┐
                      │R│ 220Ω
                      └┬┘
                       │
                      LED
                       │
   GND ────────────────┘
```

### 4.4 연결 순서
1. LED의 긴 다리(+)를 브레드보드에 꽂기
2. LED의 짧은 다리(-)를 다른 열에 꽂기
3. 저항의 한쪽을 LED 긴 다리와 같은 열에 연결
4. 저항의 다른 쪽에 점퍼 와이어 연결 → 아두이노 13번 핀
5. LED 짧은 다리에 점퍼 와이어 연결 → 아두이노 GND

### 4.5 저항을 사용하는 이유
```
전압 = 전류 × 저항 (옴의 법칙)

LED 없이 직접 연결시:
- 과전류가 흘러 LED가 타버림
- 아두이노 핀도 손상될 수 있음

저항 계산:
R = (Vs - Vf) / If
R = (5V - 2V) / 0.02A = 150Ω
→ 안전을 위해 220Ω 이상 사용 권장
```

## 🎮 5. 다양한 Blink 패턴 만들기

### 5.1 빠르게 깜빡이기
```cpp
void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  delay(100);  // 0.1초
  digitalWrite(13, LOW);
  delay(100);  // 0.1초
}
```

### 5.2 SOS 신호 패턴
```cpp
// SOS 모스 부호: ··· --- ···
// 짧은 신호(S): 3번, 긴 신호(O): 3번

const int ledPin = 13;
const int dotDelay = 200;    // 점(·) 지속 시간

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // S (···)
  for(int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(dotDelay);
    digitalWrite(ledPin, LOW);
    delay(dotDelay);
  }
  
  delay(dotDelay * 2);  // 문자 간 간격
  
  // O (---)
  for(int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(dotDelay * 3);  // 대시는 점의 3배
    digitalWrite(ledPin, LOW);
    delay(dotDelay);
  }
  
  delay(dotDelay * 2);
  
  // S (···)
  for(int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(dotDelay);
    digitalWrite(ledPin, LOW);
    delay(dotDelay);
  }
  
  delay(dotDelay * 6);  // 단어 간 간격
}
```

### 5.3 페이드 효과 흉내내기 (디지털 방식)
```cpp
void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  // 점점 빠르게
  for(int delayTime = 1000; delayTime > 0; delayTime -= 100) {
    digitalWrite(13, HIGH);
    delay(delayTime);
    digitalWrite(13, LOW);
    delay(delayTime);
  }
  
  // 점점 느리게
  for(int delayTime = 100; delayTime <= 1000; delayTime += 100) {
    digitalWrite(13, HIGH);
    delay(delayTime);
    digitalWrite(13, LOW);
    delay(delayTime);
  }
}
```

## 🛠️ 6. 문제 해결 가이드

### 6.1 LED가 켜지지 않을 때

| 문제 | 확인사항 | 해결방법 |
|-----|---------|---------|
| LED 극성 | 긴 다리가 +인지 확인 | LED 방향 바꾸기 |
| 연결 상태 | 모든 연결이 확실한지 | 연결 다시 확인 |
| 저항값 | 저항이 너무 큰지 | 220Ω~1kΩ 사용 |
| 핀 번호 | 코드와 연결 핀 일치 | 핀 번호 확인 |
| LED 고장 | LED 자체 문제 | 다른 LED로 교체 |

### 6.2 업로드 오류
```
// 오류: 'OUTPUT' was not declared in this scope
→ 대소문자 확인 (OUTPUT이 맞음, Output 아님)

// 오류: expected ';' before '}'
→ 세미콜론(;) 빠뜨림

// 오류: 'digitalwrite' was not declared
→ digitalWrite (W 대문자)
```

## 📊 7. delay() 없이 LED 제어하기 (고급)

### 7.1 millis() 함수 활용
```cpp
const int ledPin = 13;
unsigned long previousMillis = 0;
const long interval = 1000;
int ledState = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // LED 상태 토글
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    
    digitalWrite(ledPin, ledState);
  }
  
  // 여기에 다른 코드 실행 가능
  // delay()와 달리 프로그램이 멈추지 않음
}
```

### 7.2 장점
- 프로그램이 멈추지 않음
- 여러 작업을 동시에 처리 가능
- 더 복잡한 타이밍 제어 가능

## ✅ 핵심 정리

1. **모든 아두이노 프로그램**은 setup()과 loop() 함수가 필요합니다.
2. **pinMode()**로 핀의 입출력 모드를 설정합니다.
3. **digitalWrite()**로 디지털 핀에 HIGH/LOW를 출력합니다.
4. **delay()**는 프로그램을 일시 정지시킵니다.
5. **LED 연결시** 반드시 저항을 사용해야 합니다.

## 🚀 다음 단계

다음 레슨에서는 버튼을 사용한 입력 제어와 
여러 개의 LED를 제어하는 방법을 배워보겠습니다.

## 💻 실습 과제

### 과제 1: 다양한 깜빡임 패턴
1. LED가 빠르게 2번, 느리게 1번 깜빡이는 패턴 만들기
2. 심장박동처럼 깜빡이는 패턴 만들기
3. 점점 빨라졌다가 느려지는 패턴 만들기

### 과제 2: 여러 개의 LED 제어
1. 3개의 LED를 각각 다른 핀에 연결
2. 순차적으로 켜지는 패턴 만들기
3. 신호등처럼 동작하는 프로그램 작성

### 과제 3: 코드 최적화
1. 반복되는 코드를 for 루프로 개선
2. 함수를 만들어 코드 재사용성 높이기
3. #define을 사용해 상수 정의하기

## 🎨 창의적 프로젝트 아이디어

1. **크리스마스 트리 LED**: 여러 LED로 반짝이는 효과
2. **자동차 방향지시등**: 좌우 깜빡임 패턴
3. **음악 비트 표시**: 리듬에 맞춰 깜빡이기
4. **경고등 시스템**: 상황별 다른 패턴
5. **모스 부호 송신기**: 문자를 모스 부호로 변환

---

[← 이전 레슨: IDE 설치](../02-arduino-ide-setup/README.md) | [다음 레슨: 디지털 입출력 →](../04-digital-io/README.md)