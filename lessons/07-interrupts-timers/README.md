# Lesson 07: 인터럽트와 타이머

## 🎯 학습 목표
- 인터럽트의 개념과 동작 원리 이해하기
- 외부 인터럽트와 타이머 인터럽트 활용하기
- 정확한 타이밍이 필요한 시스템 구현하기
- 멀티태스킹 개념을 아두이노에 적용하기
- 실시간 시스템의 기초 익히기

## 📖 1. 인터럽트 기초 개념

### 1.1 인터럽트란?
```
일반적인 프로그램 실행:
main() → loop() → delay() → loop() → delay() → ...
         ↓                    ↓
     센서 확인            센서 확인
     (폴링 방식)          (폴링 방식)

인터럽트 방식:
main() → loop() → 다른작업 → loop() → 다른작업 → ...
              ↗                 ↗
         인터럽트!          인터럽트!
         즉시 처리          즉시 처리

장점:
✅ 실시간 응답 가능
✅ CPU 효율성 향상  
✅ 정확한 타이밍 보장
✅ 멀티태스킹 가능
```

### 1.2 아두이노 인터럽트 종류
```
📍 외부 인터럽트 (External Interrupt)
- Arduino Uno: 2번, 3번 핀 (INT0, INT1)
- 트리거: RISING, FALLING, CHANGE, LOW

⏰ 타이머 인터럽트 (Timer Interrupt)  
- Timer0: delay(), millis() 사용
- Timer1: 16비트, 정밀한 타이밍
- Timer2: 8비트, PWM 및 톤 생성

📡 기타 인터럽트
- 시리얼 통신, ADC 완료, 핀 변화 등
```

### 1.3 인터럽트 우선순위 (Arduino Uno)
```
우선순위 높음 → 낮음

1. Reset (최고 우선순위)
2. External Interrupt Request 0 (INT0) - Pin 2
3. External Interrupt Request 1 (INT1) - Pin 3  
4. Pin Change Interrupt Request 0-2 (PCINT)
5. Watchdog Time-out Interrupt
6. Timer2 Compare Match A/B
7. Timer2 Overflow
8. Timer1 Capture Event
9. Timer1 Compare Match A/B  
10. Timer1 Overflow
11. Timer0 Compare Match A/B
12. Timer0 Overflow
13. SPI Serial Transfer Complete
14. USART Receive Complete
15. USART Data Register Empty
16. USART Transmit Complete
17. ADC Conversion Complete
```

## ⚡ 2. 외부 인터럽트 활용

### 2.1 기본 외부 인터럽트
```cpp
/*
  External_Interrupt_Basic.ino
  외부 인터럽트 기초 - 버튼으로 LED 제어
*/

// 인터럽트 핀 (Arduino Uno: 2, 3번만 가능)
const int INTERRUPT_PIN = 2;
const int LED_PIN = 13;

// 인터럽트에서 사용할 변수 (volatile 필수!)
volatile bool ledState = false;
volatile unsigned long lastInterruptTime = 0;

void setup() {
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("외부 인터럽트 테스트 시작!");
  
  // 인터럽트 설정
  // attachInterrupt(인터럽트번호, 콜백함수, 트리거모드)
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), buttonPressed, FALLING);
  
  Serial.println("버튼을 눌러보세요 (Pin 2)");
}

void loop() {
  // 메인 루프는 다른 작업 수행
  Serial.print("메인 루프 실행 중... LED 상태: ");
  Serial.println(ledState ? "ON" : "OFF");
  
  // LED 상태 반영
  digitalWrite(LED_PIN, ledState);
  
  // 1초 대기 (인터럽트는 이 시간에도 동작!)
  delay(1000);
}

// 인터럽트 서비스 루틴 (ISR)
void buttonPressed() {
  // 디바운싱: 50ms 이내의 연속 인터럽트 무시  
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 50) {
    
    ledState = !ledState;  // LED 상태 토글
    lastInterruptTime = currentTime;
    
    // 주의: ISR에서는 Serial.print() 사용 지양
    // (여기서는 데모를 위해 사용)
  }
}
```

### 2.2 인터럽트 트리거 모드별 예제
```cpp
/*
  Interrupt_Trigger_Modes.ino
  다양한 인터럽트 트리거 모드 테스트
*/

const int INTERRUPT_PIN = 2;
volatile int interruptCount = 0;
volatile char lastTrigger = ' ';

void setup() {
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  
  Serial.println("인터럽트 트리거 모드 테스트");
  Serial.println("명령어: R(RISING), F(FALLING), C(CHANGE), L(LOW)");
  
  // 기본값: FALLING 모드
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, FALLING);
  Serial.println("현재 모드: FALLING");
}

void loop() {
  // 시리얼 명령 처리
  if (Serial.available()) {
    char command = Serial.read();
    
    // 기존 인터럽트 해제
    detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
    
    switch (command) {
      case 'R':
      case 'r':
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, RISING);
        Serial.println("모드 변경: RISING (LOW → HIGH)");
        lastTrigger = 'R';
        break;
        
      case 'F':  
      case 'f':
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, FALLING);
        Serial.println("모드 변경: FALLING (HIGH → LOW)");
        lastTrigger = 'F';
        break;
        
      case 'C':
      case 'c':
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, CHANGE);
        Serial.println("모드 변경: CHANGE (모든 변화)");
        lastTrigger = 'C';
        break;
        
      case 'L':
      case 'l':
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, LOW);
        Serial.println("모드 변경: LOW (LOW 상태 지속)");
        lastTrigger = 'L';
        break;
    }
  }
  
  // 상태 출력
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    Serial.print("인터럽트 발생 횟수: ");
    Serial.print(interruptCount);
    Serial.print(" (모드: ");
    Serial.print(lastTrigger);
    Serial.print(", 핀 상태: ");
    Serial.print(digitalRead(INTERRUPT_PIN) ? "HIGH" : "LOW");
    Serial.println(")");
    lastPrint = millis();
  }
}

void handleInterrupt() {
  interruptCount++;
}
```

## ⏰ 3. 타이머 인터럽트

### 3.1 Timer1 인터럽트 기초
```cpp
/*
  Timer_Interrupt_Basic.ino
  Timer1을 이용한 정밀한 주기 제어
*/

#include <TimerOne.h>  // TimerOne 라이브러리 필요

const int LED_PIN = 13;
volatile bool ledState = false;
volatile unsigned long timerCount = 0;

// 타이머 주기 설정 (마이크로초)
const long TIMER_PERIOD = 500000;  // 500ms = 0.5초

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  
  Serial.println("Timer1 인터럽트 테스트");
  Serial.print("타이머 주기: ");
  Serial.print(TIMER_PERIOD / 1000);
  Serial.println("ms");
  
  // Timer1 초기화 및 시작
  Timer1.initialize(TIMER_PERIOD);
  Timer1.attachInterrupt(timerCallback);
  
  Serial.println("타이머 시작!");
}

void loop() {
  // 메인 루프는 독립적으로 동작
  Serial.print("메인 루프 - 타이머 호출 횟수: ");
  Serial.print(timerCount);
  Serial.print(", LED: ");
  Serial.println(ledState ? "ON" : "OFF");
  
  // 다른 작업 수행 (타이머와 독립적)
  delay(1500);  // 1.5초 대기
}

// 타이머 인터럽트 서비스 루틴
void timerCallback() {
  timerCount++;
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
}
```

### 3.2 정밀한 주파수 생성기
```cpp
/*
  Precision_Frequency_Generator.ino
  타이머 인터럽트를 이용한 정밀 주파수 생성
*/

const int OUTPUT_PIN = 9;
volatile bool outputState = false;

// 주파수 설정 (Hz)
float targetFrequency = 1000.0;  // 1kHz

void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  Serial.begin(9600);
  
  Serial.println("정밀 주파수 생성기");
  Serial.println("명령: 주파수 입력 (예: 1000)");
  
  setupTimer1(targetFrequency);
}

void loop() {
  // 시리얼로 주파수 변경
  if (Serial.available()) {
    float newFreq = Serial.parseFloat();
    
    if (newFreq > 0.1 && newFreq <= 8000000) {  // 0.1Hz ~ 8MHz
      targetFrequency = newFreq;
      setupTimer1(targetFrequency);
      
      Serial.print("주파수 변경: ");
      Serial.print(targetFrequency, 2);
      Serial.println(" Hz");
    } else {
      Serial.println("유효 범위: 0.1 ~ 8000000 Hz");
    }
  }
  
  // 주파수 측정 및 출력
  static unsigned long lastMeasure = 0;
  if (millis() - lastMeasure > 5000) {  // 5초마다
    measureFrequency();
    lastMeasure = millis();
  }
}

void setupTimer1(float frequency) {
  // Timer1 정지
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  // 주파수 계산
  // 토글 주파수 = 목표주파수 * 2 (HIGH/LOW 변화)
  float toggleFreq = frequency * 2.0;
  
  // OCR1A 값 계산 (16MHz 클럭 기준)
  long ocrValue = (16000000.0 / toggleFreq) - 1;
  
  // 프리스케일러 선택
  int prescaler = 1;
  if (ocrValue > 65535) {
    prescaler = 8;
    ocrValue = (16000000.0 / (8.0 * toggleFreq)) - 1;
  }
  if (ocrValue > 65535) {
    prescaler = 64;
    ocrValue = (16000000.0 / (64.0 * toggleFreq)) - 1;
  }
  if (ocrValue > 65535) {
    prescaler = 256;
    ocrValue = (16000000.0 / (256.0 * toggleFreq)) - 1;
  }
  if (ocrValue > 65535) {
    prescaler = 1024;
    ocrValue = (16000000.0 / (1024.0 * toggleFreq)) - 1;
  }
  
  OCR1A = ocrValue;
  
  // CTC 모드, 인터럽트 활성화
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);  // CTC 모드
  
  // 프리스케일러 설정
  switch (prescaler) {
    case 1:    TCCR1B |= (1 << CS10); break;
    case 8:    TCCR1B |= (1 << CS11); break;
    case 64:   TCCR1B |= (1 << CS11) | (1 << CS10); break;
    case 256:  TCCR1B |= (1 << CS12); break;
    case 1024: TCCR1B |= (1 << CS12) | (1 << CS10); break;
  }
  
  TIMSK1 |= (1 << OCIE1A);  // 인터럽트 활성화
  sei();  // 전역 인터럽트 활성화
  
  Serial.print("타이머 설정: OCR1A=");
  Serial.print(ocrValue);
  Serial.print(", 프리스케일러=");
  Serial.println(prescaler);
}

// Timer1 Compare A 인터럽트
ISR(TIMER1_COMPA_vect) {
  outputState = !outputState;
  digitalWrite(OUTPUT_PIN, outputState);
}

void measureFrequency() {
  // 간단한 주파수 측정 (실제론 더 정교한 방법 필요)
  unsigned long startTime = micros();
  int transitions = 0;
  bool lastState = digitalRead(OUTPUT_PIN);
  
  // 1000번의 상태 변화 측정
  while (transitions < 1000) {
    bool currentState = digitalRead(OUTPUT_PIN);
    if (currentState != lastState) {
      transitions++;
      lastState = currentState;
    }
  }
  
  unsigned long endTime = micros();
  float measuredPeriod = (endTime - startTime) / 1000000.0;  // 초 단위
  float measuredFrequency = 500.0 / measuredPeriod;  // 500번의 사이클
  
  Serial.print("측정된 주파수: ");
  Serial.print(measuredFrequency, 2);
  Serial.print(" Hz (설정: ");
  Serial.print(targetFrequency, 2);
  Serial.println(" Hz)");
}
```

## 🔄 4. 멀티태스킹 시뮬레이션

### 4.1 협력적 멀티태스킹
```cpp
/*
  Cooperative_Multitasking.ino
  인터럽트를 이용한 간단한 멀티태스킹 시뮬레이션
*/

#include <TimerOne.h>

// 태스크 구조체
struct Task {
  void (*function)();      // 실행할 함수
  unsigned long period;    // 실행 주기 (ms)
  unsigned long lastRun;   // 마지막 실행 시간
  bool enabled;            // 활성화 여부
  const char* name;        // 태스크 이름
};

// 태스크 정의
Task tasks[] = {
  {blinkLED,     500,  0, true,  "LED Blink"},
  {readSensors,  1000, 0, true,  "Sensor Read"},
  {sendData,     2000, 0, true,  "Data Send"},
  {checkButtons, 50,   0, true,  "Button Check"},
  {updateDisplay,250,  0, true,  "Display Update"}
};

const int NUM_TASKS = sizeof(tasks) / sizeof(tasks[0]);

// 시스템 상태
volatile unsigned long systemTick = 0;
volatile bool taskSchedulerRunning = false;

// 하드웨어 핀
const int LED_PIN = 13;
const int BUTTON_PIN = 2;
const int TEMP_PIN = A0;
const int LIGHT_PIN = A1;

// 전역 변수
bool ledState = false;
float temperature = 0;
int lightLevel = 0;
bool buttonPressed = false;
int displayCounter = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("멀티태스킹 시뮬레이터 시작!");
  
  // 1ms 타이머 인터럽트 설정
  Timer1.initialize(1000);  // 1ms
  Timer1.attachInterrupt(systemTickISR);
  
  taskSchedulerRunning = true;
  Serial.println("태스크 스케줄러 활성화");
  
  // 태스크 목록 출력
  Serial.println("\n활성 태스크:");
  for (int i = 0; i < NUM_TASKS; i++) {
    Serial.print("- ");
    Serial.print(tasks[i].name);
    Serial.print(" (");
    Serial.print(tasks[i].period);
    Serial.println("ms)");
  }
  Serial.println();
}

void loop() {
  // 태스크 스케줄러 실행
  if (taskSchedulerRunning) {
    runTaskScheduler();
  }
  
  // 명령어 처리
  handleCommands();
}

// 1ms 시스템 틱 인터럽트
void systemTickISR() {
  systemTick++;
}

void runTaskScheduler() {
  unsigned long currentTime = systemTick;
  
  for (int i = 0; i < NUM_TASKS; i++) {
    if (tasks[i].enabled && 
        (currentTime - tasks[i].lastRun >= tasks[i].period)) {
      
      // 태스크 실행
      unsigned long startTime = micros();
      tasks[i].function();
      unsigned long executionTime = micros() - startTime;
      
      tasks[i].lastRun = currentTime;
      
      // 실행 시간이 길면 경고
      if (executionTime > 1000) {  // 1ms 이상
        Serial.print("⚠️ ");
        Serial.print(tasks[i].name);
        Serial.print(" 태스크 실행 시간: ");
        Serial.print(executionTime);
        Serial.println("μs");
      }
    }
  }
}

// 개별 태스크 함수들
void blinkLED() {
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
}

void readSensors() {
  // 센서 데이터 읽기
  int tempRaw = analogRead(TEMP_PIN);
  temperature = (tempRaw / 1023.0) * 5.0 * 100.0;  // LM35
  
  lightLevel = analogRead(LIGHT_PIN);
}

void sendData() {
  Serial.print("📊 센서 데이터 - 온도: ");
  Serial.print(temperature, 1);
  Serial.print("°C, 조도: ");
  Serial.print(lightLevel);
  Serial.print(", 시스템 틱: ");
  Serial.println(systemTick);
}

void checkButtons() {
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    buttonPressed = true;
    Serial.println("🔘 버튼 눌림 감지!");
  }
  
  lastButtonState = currentButtonState;
}

void updateDisplay() {
  displayCounter++;
  
  // 매 4번째 호출마다 (1초마다) 상태 출력
  if (displayCounter % 4 == 0) {
    Serial.print("💻 디스플레이: LED=");
    Serial.print(ledState ? "ON" : "OFF");
    Serial.print(", 버튼=");
    Serial.print(buttonPressed ? "PRESSED" : "NORMAL");
    Serial.print(", 업타임=");
    Serial.print(systemTick / 1000);
    Serial.println("s");
    
    buttonPressed = false;  // 상태 리셋
  }
}

void handleCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toLowerCase();
    
    if (command == "status") {
      showSystemStatus();
      
    } else if (command == "pause") {
      taskSchedulerRunning = false;
      Serial.println("⏸️ 태스크 스케줄러 일시정지");
      
    } else if (command == "resume") {
      taskSchedulerRunning = true;
      Serial.println("▶️ 태스크 스케줄러 재시작");
      
    } else if (command.startsWith("disable ")) {
      int taskIndex = command.substring(8).toInt();
      if (taskIndex >= 0 && taskIndex < NUM_TASKS) {
        tasks[taskIndex].enabled = false;
        Serial.print("❌ 태스크 비활성화: ");
        Serial.println(tasks[taskIndex].name);
      }
      
    } else if (command.startsWith("enable ")) {
      int taskIndex = command.substring(7).toInt();
      if (taskIndex >= 0 && taskIndex < NUM_TASKS) {
        tasks[taskIndex].enabled = true;
        Serial.print("✅ 태스크 활성화: ");
        Serial.println(tasks[taskIndex].name);
      }
      
    } else if (command == "help") {
      showHelp();
    }
  }
}

void showSystemStatus() {
  Serial.println("🖥️ 시스템 상태");
  Serial.print("시스템 틱: ");
  Serial.println(systemTick);
  Serial.print("업타임: ");
  Serial.print(systemTick / 1000);
  Serial.println("초");
  Serial.print("스케줄러: ");
  Serial.println(taskSchedulerRunning ? "실행중" : "정지");
  
  Serial.println("\n📋 태스크 상태:");
  for (int i = 0; i < NUM_TASKS; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(tasks[i].name);
    Serial.print(" (");
    Serial.print(tasks[i].period);
    Serial.print("ms) - ");
    Serial.println(tasks[i].enabled ? "활성" : "비활성");
  }
  Serial.println();
}

void showHelp() {
  Serial.println("📚 사용 가능한 명령어:");
  Serial.println("status        - 시스템 상태 표시");
  Serial.println("pause         - 스케줄러 일시정지");
  Serial.println("resume        - 스케줄러 재시작");
  Serial.println("disable <n>   - n번 태스크 비활성화");
  Serial.println("enable <n>    - n번 태스크 활성화");
  Serial.println("help          - 이 도움말");
}
```

## 🎵 5. 정밀한 사운드 생성

### 5.1 다중 톤 생성기
```cpp
/*
  Multi_Tone_Generator.ino
  타이머 인터럽트를 이용한 다중 음계 동시 재생
*/

// 음계 주파수 정의
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

// 스피커 핀
const int SPEAKER1_PIN = 9;
const int SPEAKER2_PIN = 10;
const int SPEAKER3_PIN = 11;

// 톤 생성을 위한 변수
volatile unsigned long timer1Counter = 0;
volatile unsigned long timer2Counter = 0;

volatile int tone1Frequency = 0;
volatile int tone2Frequency = 0;
volatile int tone3Frequency = 0;

volatile bool tone1State = false;
volatile bool tone2State = false;
volatile bool tone3State = false;

volatile unsigned long tone1Period = 0;
volatile unsigned long tone2Period = 0;
volatile unsigned long tone3Period = 0;

void setup() {
  pinMode(SPEAKER1_PIN, OUTPUT);
  pinMode(SPEAKER2_PIN, OUTPUT);
  pinMode(SPEAKER3_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("다중 톤 생성기");
  Serial.println("명령어: play <톤1_주파수> <톤2_주파수> <톤3_주파수>");
  Serial.println("예: play 440 554 659 (A4 화음)");
  Serial.println("stop - 모든 톤 정지");
  
  // Timer2를 사용하여 고속 인터럽트 설정 (31.25kHz)
  setupTimer2();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command.startsWith("play ")) {
      parseAndPlayTones(command);
      
    } else if (command == "stop") {
      stopAllTones();
      Serial.println("모든 톤 정지");
      
    } else if (command == "chord") {
      playChordProgression();
      
    } else if (command == "melody") {
      playMelody();
    }
  }
}

void setupTimer2() {
  // Timer2 설정 (8비트 타이머)
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  
  // CTC 모드, 프리스케일러 1 (16MHz / 256 = 62.5kHz)
  OCR2A = 255;  // TOP 값
  TCCR2A |= (1 << WGM21);  // CTC 모드
  TCCR2B |= (1 << CS20);   // 프리스케일러 1
  TIMSK2 |= (1 << OCIE2A); // 인터럽트 활성화
  
  sei();  // 전역 인터럽트 활성화
}

// Timer2 인터럽트 (약 62.5kHz)
ISR(TIMER2_COMPA_vect) {
  timer1Counter++;
  timer2Counter++;
  
  // 톤 1 생성
  if (tone1Frequency > 0 && tone1Period > 0) {
    if (timer1Counter >= tone1Period) {
      tone1State = !tone1State;
      digitalWrite(SPEAKER1_PIN, tone1State);
      timer1Counter = 0;
    }
  }
  
  // 톤 2 생성 (독립적)
  if (tone2Frequency > 0 && tone2Period > 0) {
    static unsigned long tone2Counter = 0;
    tone2Counter++;
    if (tone2Counter >= tone2Period) {
      tone2State = !tone2State;
      digitalWrite(SPEAKER2_PIN, tone2State);
      tone2Counter = 0;
    }
  }
  
  // 톤 3 생성 (독립적)  
  if (tone3Frequency > 0 && tone3Period > 0) {
    static unsigned long tone3Counter = 0;
    tone3Counter++;
    if (tone3Counter >= tone3Period) {
      tone3State = !tone3State;
      digitalWrite(SPEAKER3_PIN, tone3State);
      tone3Counter = 0;
    }
  }
}

void parseAndPlayTones(String command) {
  // "play 440 554 659" 형식 파싱
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  int thirdSpace = command.indexOf(' ', secondSpace + 1);
  
  if (firstSpace != -1 && secondSpace != -1 && thirdSpace != -1) {
    int freq1 = command.substring(firstSpace + 1, secondSpace).toInt();
    int freq2 = command.substring(secondSpace + 1, thirdSpace).toInt();
    int freq3 = command.substring(thirdSpace + 1).toInt();
    
    playTones(freq1, freq2, freq3);
    
    Serial.print("재생 중: ");
    Serial.print(freq1);
    Serial.print("Hz, ");
    Serial.print(freq2);
    Serial.print("Hz, ");
    Serial.print(freq3);
    Serial.println("Hz");
  } else {
    Serial.println("형식: play <freq1> <freq2> <freq3>");
  }
}

void playTones(int freq1, int freq2, int freq3) {
  // 주파수를 타이머 주기로 변환
  // 62500Hz / (2 * frequency) = 토글 주기
  
  tone1Frequency = freq1;
  tone2Frequency = freq2;
  tone3Frequency = freq3;
  
  if (freq1 > 0) {
    tone1Period = 62500 / (2 * freq1);
  } else {
    tone1Period = 0;
  }
  
  if (freq2 > 0) {
    tone2Period = 62500 / (2 * freq2);
  } else {
    tone2Period = 0;
  }
  
  if (freq3 > 0) {
    tone3Period = 62500 / (2 * freq3);
  } else {
    tone3Period = 0;
  }
}

void stopAllTones() {
  tone1Frequency = 0;
  tone2Frequency = 0;
  tone3Frequency = 0;
  
  digitalWrite(SPEAKER1_PIN, LOW);
  digitalWrite(SPEAKER2_PIN, LOW);
  digitalWrite(SPEAKER3_PIN, LOW);
}

void playChordProgression() {
  Serial.println("화음 진행 연주 시작...");
  
  // C 메이저 코드 (C-E-G)
  playTones(NOTE_C4, NOTE_E4, NOTE_G4);
  delay(1000);
  
  // F 메이저 코드 (F-A-C)
  playTones(NOTE_F4, NOTE_A4, NOTE_C5);
  delay(1000);
  
  // G 메이저 코드 (G-B-D)
  playTones(NOTE_G4, NOTE_B4, NOTE_D4 * 2);  // D5
  delay(1000);
  
  // C 메이저 코드 (마무리)
  playTones(NOTE_C4, NOTE_E4, NOTE_G4);
  delay(1000);
  
  stopAllTones();
  Serial.println("화음 진행 완료");
}

void playMelody() {
  Serial.println("멜로디 연주 시작...");
  
  int melody[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5};
  int noteDuration = 500;
  
  for (int i = 0; i < 8; i++) {
    playTones(melody[i], 0, 0);  // 첫 번째 스피커만 사용
    delay(noteDuration);
  }
  
  stopAllTones();
  Serial.println("멜로디 완료");
}
```

## ⚠️ 6. 인터럽트 사용시 주의사항

### 6.1 ISR 작성 규칙
```cpp
// ✅ 올바른 ISR 작성법
void correctISR() {
  // 1. 최대한 짧고 빠르게
  digitalState = !digitalState;
  
  // 2. volatile 변수만 수정
  volatile int counter;  // 전역에서 선언
  counter++;
  
  // 3. 간단한 연산만
  simpleFlag = true;
}

// ❌ 잘못된 ISR 사용법  
void wrongISR() {
  Serial.println("ISR 실행!");  // 시리얼 통신 금지!
  delay(100);                   // delay() 사용 금지!
  int result = sqrt(value);     // 복잡한 연산 금지!
  malloc(100);                  // 동적 메모리 할당 금지!
}

// ISR에서 사용할 변수는 반드시 volatile로 선언
volatile bool interruptFlag = false;
volatile unsigned long interruptTime = 0;
volatile int interruptCount = 0;
```

### 6.2 인터럽트 충돌 해결
```cpp
// 크리티컬 섹션 보호
void protectedFunction() {
  noInterrupts();  // 인터럽트 비활성화
  
  // 중요한 작업 (원자적 실행 필요)
  criticalVariable++;
  complexCalculation();
  
  interrupts();    // 인터럽트 재활성화
}

// 특정 인터럽트만 제어
void selectiveInterruptControl() {
  // Timer1 인터럽트만 비활성화
  TIMSK1 &= ~(1 << OCIE1A);
  
  // 중요한 작업
  timerRelatedWork();
  
  // Timer1 인터럽트 재활성화
  TIMSK1 |= (1 << OCIE1A);
}
```

## ✅ 핵심 정리

1. **인터럽트**는 실시간 응답과 정확한 타이밍 제어에 필수적입니다.
2. **ISR은 짧고 빠르게** 작성하며, volatile 변수만 사용해야 합니다.
3. **타이머 인터럽트**로 정밀한 주기 제어와 멀티태스킹이 가능합니다.
4. **인터럽트 우선순위**를 이해하고 충돌을 방지해야 합니다.

## 🚀 다음 단계

다음 레슨에서는 다양한 센서들을 심화적으로 활용하여
실제 측정 시스템과 IoT 기기를 구축하는 방법을 
배워보겠습니다.

## 💻 실습 과제

### 과제 1: 기본 인터럽트
1. 외부 인터럽트로 정확한 속도 측정 시스템 구현
2. 타이머 인터럽트로 정밀한 PWM 신호 생성
3. 인터럽트 기반 디지털 스톱워치 제작

### 과제 2: 고급 응용
1. 다중 인터럽트를 이용한 엔코더 읽기
2. 인터럽트 기반 실시간 FFT 분석기
3. 협력적 멀티태스킹 OS 프레임워크 구현

### 과제 3: 실전 프로젝트
1. 인터럽트 기반 데이터 로거 (정확한 타이밍)
2. 실시간 모터 제어 시스템 (PID + 인터럽트)
3. 다채널 사운드 시스템 (실시간 믹싱)

---

[← 이전 레슨: 시리얼 통신](../06-serial-communication/README.md) | [다음 레슨: 센서 활용 심화 →](../08-advanced-sensors/README.md)