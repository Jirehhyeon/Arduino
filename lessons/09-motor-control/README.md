# Lesson 09: 모터 제어 마스터

## 🎯 학습 목표
- 다양한 모터의 종류와 특성 완전 이해하기
- DC 모터, 서보 모터, 스테퍼 모터 전문가 수준 제어
- 모터 드라이버와 전력 관리 시스템 구축하기
- PID 제어를 이용한 정밀 위치/속도 제어 구현하기
- 실제 로봇 시스템에 적용 가능한 모터 제어 기술 마스터

## 📖 1. 모터의 종류와 특성

### 1.1 모터 분류 체계
```
⚡ 전기 모터 대분류

🔄 DC 모터 (Direct Current Motor)
├── 브러시 DC 모터 (Brushed DC)
│   ├── 영구자석형 (PMDC)
│   └── 권선형 (Wound Field)
└── 브러시리스 DC 모터 (BLDC)
    ├── 센서리스 (Sensorless)
    └── 홀센서형 (Hall Sensor)

🎯 서보 모터 (Servo Motor)  
├── 아날로그 서보 (Analog)
├── 디지털 서보 (Digital)
└── 연속회전 서보 (Continuous)

⚙️ 스테퍼 모터 (Stepper Motor)
├── 유니폴라 (Unipolar)
├── 바이폴라 (Bipolar)  
└── 하이브리드 (Hybrid)

🌊 AC 모터 (Alternating Current)
├── 유도 모터 (Induction)
└── 동기 모터 (Synchronous)
```

### 1.2 모터 특성 비교표
```cpp
/*
  모터별 특성 비교

  특성           DC모터    서보모터   스테퍼모터   BLDC
  ────────────────────────────────────────────────────
  제어 복잡도     ⭐        ⭐⭐       ⭐⭐⭐      ⭐⭐⭐⭐
  위치 정확도     ⭐        ⭐⭐⭐⭐   ⭐⭐⭐⭐⭐   ⭐⭐⭐
  속도 제어       ⭐⭐⭐⭐   ⭐⭐⭐     ⭐⭐        ⭐⭐⭐⭐⭐
  토크            ⭐⭐⭐     ⭐⭐⭐     ⭐⭐⭐⭐     ⭐⭐⭐⭐
  효율성          ⭐⭐      ⭐⭐⭐     ⭐⭐⭐      ⭐⭐⭐⭐⭐
  소음            ⭐⭐      ⭐⭐⭐     ⭐⭐⭐⭐     ⭐⭐⭐⭐⭐
  가격            ⭐⭐⭐⭐⭐  ⭐⭐⭐     ⭐⭐⭐      ⭐⭐
  수명            ⭐⭐⭐     ⭐⭐⭐⭐   ⭐⭐⭐⭐⭐   ⭐⭐⭐⭐⭐
  
  용도:
  DC모터     - 팬, 펌프, 간단한 구동
  서보모터   - 로봇 관절, 카메라 짐벌
  스테퍼모터 - 3D프린터, CNC, 정밀위치
  BLDC      - 드론, 전기차, 고성능 시스템
*/

// 모터 선택 가이드
struct MotorSelection {
  String application;
  String recommendedType;
  String reason;
};

MotorSelection motorGuide[] = {
  {"로봇 바퀴", "DC모터 + 엔코더", "연속회전, 속도제어 중요"},
  {"로봇 관절", "서보모터", "정확한 각도 제어 필요"},
  {"3D프린터 축", "스테퍼모터", "정밀한 위치 제어"},
  {"드론 프로펠러", "BLDC모터", "고효율, 고속회전"},
  {"카메라 짐벌", "서보모터", "부드러운 각도 조절"},
  {"컨베이어벨트", "DC모터", "일정 속도 연속 구동"}
};
```

## 🔄 2. DC 모터 완전 정복

### 2.1 DC 모터 기본 제어
```cpp
/*
  DC_Motor_Basic_Control.ino
  DC 모터 기본 제어 - L298N 드라이버 사용
  
  연결:
  L298N ENA → Pin 9 (PWM)
  L298N IN1 → Pin 7
  L298N IN2 → Pin 8
  L298N ENB → Pin 10 (PWM)
  L298N IN3 → Pin 4
  L298N IN4 → Pin 5
*/

// 모터 A (좌측)
const int MOTOR_A_SPEED = 9;   // ENA (PWM)
const int MOTOR_A_DIR1 = 7;    // IN1
const int MOTOR_A_DIR2 = 8;    // IN2

// 모터 B (우측)
const int MOTOR_B_SPEED = 10;  // ENB (PWM)
const int MOTOR_B_DIR1 = 4;    // IN3
const int MOTOR_B_DIR2 = 5;    // IN4

// 제어 변수
int motorASpeed = 0;   // -255 ~ +255
int motorBSpeed = 0;   // -255 ~ +255

void setup() {
  Serial.begin(9600);
  Serial.println("🔄 DC 모터 제어 시스템 시작!");
  
  // 모터 핀 설정
  pinMode(MOTOR_A_SPEED, OUTPUT);
  pinMode(MOTOR_A_DIR1, OUTPUT);
  pinMode(MOTOR_A_DIR2, OUTPUT);
  pinMode(MOTOR_B_SPEED, OUTPUT);
  pinMode(MOTOR_B_DIR1, OUTPUT);
  pinMode(MOTOR_B_DIR2, OUTPUT);
  
  // 초기 상태: 정지
  stopMotors();
  
  Serial.println("명령어:");
  Serial.println("w: 전진, s: 후진, a: 좌회전, d: 우회전");
  Serial.println("q: 정지, +: 속도증가, -: 속도감소");
  Serial.println("숫자 1-9: 직접 속도 설정");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    handleCommand(command);
  }
  
  // 현재 상태 출력 (2초마다)
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 2000) {
    printStatus();
    lastStatus = millis();
  }
}

void handleCommand(char cmd) {
  static int currentSpeed = 150; // 기본 속도
  
  switch (cmd) {
    case 'w': // 전진
      motorASpeed = currentSpeed;
      motorBSpeed = currentSpeed;
      Serial.println("⬆️ 전진");
      break;
      
    case 's': // 후진
      motorASpeed = -currentSpeed;
      motorBSpeed = -currentSpeed;
      Serial.println("⬇️ 후진");
      break;
      
    case 'a': // 좌회전 (제자리)
      motorASpeed = -currentSpeed;
      motorBSpeed = currentSpeed;
      Serial.println("↪️ 좌회전");
      break;
      
    case 'd': // 우회전 (제자리)
      motorASpeed = currentSpeed;
      motorBSpeed = -currentSpeed;
      Serial.println("↩️ 우회전");
      break;
      
    case 'q': // 정지
      motorASpeed = 0;
      motorBSpeed = 0;
      Serial.println("⏹️ 정지");
      break;
      
    case '+': // 속도 증가
      currentSpeed = min(currentSpeed + 25, 255);
      Serial.print("⬆️ 속도 증가: ");
      Serial.println(currentSpeed);
      break;
      
    case '-': // 속도 감소
      currentSpeed = max(currentSpeed - 25, 50);
      Serial.print("⬇️ 속도 감소: ");
      Serial.println(currentSpeed);
      break;
      
    case '1': case '2': case '3': case '4': case '5':
    case '6': case '7': case '8': case '9':
      currentSpeed = map(cmd - '0', 1, 9, 50, 255);
      Serial.print("🎛️ 속도 설정: ");
      Serial.println(currentSpeed);
      break;
      
    default:
      Serial.println("❓ 알 수 없는 명령어");
      return;
  }
  
  // 모터 구동
  driveMotors(motorASpeed, motorBSpeed);
}

void driveMotors(int speedA, int speedB) {
  // 모터 A 제어
  if (speedA > 0) {
    digitalWrite(MOTOR_A_DIR1, HIGH);
    digitalWrite(MOTOR_A_DIR2, LOW);
    analogWrite(MOTOR_A_SPEED, speedA);
  } else if (speedA < 0) {
    digitalWrite(MOTOR_A_DIR1, LOW);
    digitalWrite(MOTOR_A_DIR2, HIGH);
    analogWrite(MOTOR_A_SPEED, -speedA);
  } else {
    digitalWrite(MOTOR_A_DIR1, LOW);
    digitalWrite(MOTOR_A_DIR2, LOW);
    analogWrite(MOTOR_A_SPEED, 0);
  }
  
  // 모터 B 제어
  if (speedB > 0) {
    digitalWrite(MOTOR_B_DIR1, HIGH);
    digitalWrite(MOTOR_B_DIR2, LOW);
    analogWrite(MOTOR_B_SPEED, speedB);
  } else if (speedB < 0) {
    digitalWrite(MOTOR_B_DIR1, LOW);
    digitalWrite(MOTOR_B_DIR2, HIGH);
    analogWrite(MOTOR_B_SPEED, -speedB);
  } else {
    digitalWrite(MOTOR_B_DIR1, LOW);
    digitalWrite(MOTOR_B_DIR2, LOW);
    analogWrite(MOTOR_B_SPEED, 0);
  }
}

void stopMotors() {
  driveMotors(0, 0);
}

void printStatus() {
  Serial.println("📊 모터 상태:");
  Serial.print("  좌측 모터: ");
  Serial.print(motorASpeed);
  Serial.print(" | 우측 모터: ");
  Serial.println(motorBSpeed);
  
  // 전류 소모 측정 (개념적)
  float estimatedCurrent = (abs(motorASpeed) + abs(motorBSpeed)) * 0.01;
  Serial.print("  예상 전류: ");
  Serial.print(estimatedCurrent, 2);
  Serial.println("A");
}
```

### 2.2 엔코더를 이용한 속도 피드백 제어
```cpp
/*
  DC_Motor_Encoder_Control.ino
  엔코더 피드백을 이용한 정밀 DC 모터 제어
  
  하드웨어:
  - DC 모터 + 엔코더 (예: JGA25-371)
  - L298N 모터 드라이버
  
  연결:
  엔코더 A상 → Pin 2 (인터럽트)
  엔코더 B상 → Pin 3 (인터럽트)
*/

#include <PID_v1.h>

// 모터 제어 핀
const int MOTOR_SPEED = 9;
const int MOTOR_DIR1 = 7;
const int MOTOR_DIR2 = 8;

// 엔코더 핀 (인터럽트 핀 사용)
const int ENCODER_A = 2;
const int ENCODER_B = 3;

// 엔코더 변수 (volatile - 인터럽트에서 사용)
volatile long encoderCount = 0;
volatile bool lastA = false;
volatile bool lastB = false;

// PID 제어 변수
double setpoint = 0;    // 목표 속도 (RPM)
double input = 0;       // 현재 속도 (RPM)
double output = 0;      // PID 출력 (모터 PWM)

// PID 계수 (튜닝 필요)
double Kp = 2.0, Ki = 0.1, Kd = 0.05;
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// 속도 계산 변수
unsigned long lastTime = 0;
long lastEncoderCount = 0;
const int ENCODER_PPR = 1200;  // 엔코더 펄스/회전 (모터에 따라 다름)

void setup() {
  Serial.begin(115200);
  Serial.println("🎯 엔코더 피드백 DC 모터 제어");
  
  // 모터 핀 설정
  pinMode(MOTOR_SPEED, OUTPUT);
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  
  // 엔코더 핀 설정
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);
  
  // PID 설정
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-255, 255);
  motorPID.SetSampleTime(20); // 20ms = 50Hz
  
  Serial.println("명령어: 숫자입력으로 목표 RPM 설정 (예: 100)");
  Serial.println("s: 정지, i: PID 정보 표시");
}

void loop() {
  // 시리얼 명령 처리
  handleSerialCommands();
  
  // 속도 계산 (100ms마다)
  static unsigned long lastSpeedCalc = 0;
  if (millis() - lastSpeedCalc >= 100) {
    calculateSpeed();
    lastSpeedCalc = millis();
  }
  
  // PID 제어 실행
  if (motorPID.Compute()) {
    // PID 출력을 모터에 적용
    applyMotorOutput();
  }
  
  // 상태 출력 (500ms마다)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    printStatus();
    lastPrint = millis();
  }
}

void encoderISR() {
  // 엔코더 A, B 상태 읽기
  bool currentA = digitalRead(ENCODER_A);
  bool currentB = digitalRead(ENCODER_B);
  
  // 쿼드러처 디코딩
  if (currentA != lastA) {
    if (currentA == currentB) {
      encoderCount--;  // 역방향
    } else {
      encoderCount++;  // 정방향
    }
  }
  
  if (currentB != lastB) {
    if (currentA == currentB) {
      encoderCount++;  // 정방향
    } else {
      encoderCount--;  // 역방향
    }
  }
  
  lastA = currentA;
  lastB = currentB;
}

void calculateSpeed() {
  unsigned long currentTime = millis();
  long currentCount = encoderCount;
  
  if (currentTime - lastTime > 0) {
    // RPM 계산
    long pulseChange = currentCount - lastEncoderCount;
    float timeChange = (currentTime - lastTime) / 1000.0; // 초 단위
    
    // RPM = (펄스변화 / 시간) * (60초/분) / (펄스/회전)
    input = (pulseChange / timeChange) * 60.0 / ENCODER_PPR;
    
    lastTime = currentTime;
    lastEncoderCount = currentCount;
  }
}

void applyMotorOutput() {
  int motorPWM = (int)output;
  
  if (motorPWM > 0) {
    // 정방향 회전
    digitalWrite(MOTOR_DIR1, HIGH);
    digitalWrite(MOTOR_DIR2, LOW);
    analogWrite(MOTOR_SPEED, motorPWM);
    
  } else if (motorPWM < 0) {
    // 역방향 회전
    digitalWrite(MOTOR_DIR1, LOW);
    digitalWrite(MOTOR_DIR2, HIGH);
    analogWrite(MOTOR_SPEED, -motorPWM);
    
  } else {
    // 정지
    digitalWrite(MOTOR_DIR1, LOW);
    digitalWrite(MOTOR_DIR2, LOW);
    analogWrite(MOTOR_SPEED, 0);
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command == "s") {
      setpoint = 0;
      Serial.println("⏹️ 모터 정지");
      
    } else if (command == "i") {
      Serial.println("📊 PID 정보:");
      Serial.print("  Kp: "); Serial.println(Kp);
      Serial.print("  Ki: "); Serial.println(Ki);
      Serial.print("  Kd: "); Serial.println(Kd);
      
    } else if (command.startsWith("kp")) {
      Kp = command.substring(2).toFloat();
      motorPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Kp 설정: "); Serial.println(Kp);
      
    } else if (command.startsWith("ki")) {
      Ki = command.substring(2).toFloat();
      motorPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Ki 설정: "); Serial.println(Ki);
      
    } else if (command.startsWith("kd")) {
      Kd = command.substring(2).toFloat();
      motorPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Kd 설정: "); Serial.println(Kd);
      
    } else {
      // 숫자로 RPM 설정
      float targetRPM = command.toFloat();
      if (targetRPM != 0 || command == "0") {
        setpoint = targetRPM;
        Serial.print("🎯 목표 RPM: ");
        Serial.println(setpoint);
      } else {
        Serial.println("❓ 알 수 없는 명령어");
      }
    }
  }
}

void printStatus() {
  Serial.print("RPM - 목표: ");
  Serial.print(setpoint, 1);
  Serial.print(", 현재: ");
  Serial.print(input, 1);
  Serial.print(", PWM: ");
  Serial.print((int)output);
  Serial.print(", 엔코더: ");
  Serial.println(encoderCount);
}
```

## 🎯 3. 서보 모터 정밀 제어

### 3.1 서보 모터 기본 원리와 고급 제어
```cpp
/*
  Servo_Advanced_Control.ino
  서보 모터 고급 제어 - 부드러운 움직임과 정밀 제어
  
  서보 모터 연결:
  빨강선(+) → 5V (또는 외부 전원)
  갈색선(-) → GND  
  주황선(신호) → Pin 9
*/

#include <Servo.h>

Servo myServo;

// 서보 제어 변수
struct ServoController {
  int currentPosition;      // 현재 위치 (0-180도)
  int targetPosition;       // 목표 위치
  float speed;              // 이동 속도 (도/초)
  unsigned long lastUpdate; // 마지막 업데이트 시간
  bool isMoving;            // 움직임 상태
  
  // 부드러운 움직임을 위한 변수
  float acceleration;       // 가속도 (도/초²)
  float currentSpeed;       // 현재 속도
  float maxSpeed;           // 최대 속도
  
  // 위치 제한
  int minPosition;
  int maxPosition;
};

ServoController servo = {90, 90, 30.0, 0, false, 100.0, 0, 60.0, 0, 180};

// 사전 정의된 위치들
struct PresetPosition {
  String name;
  int angle;
  float speed;
};

PresetPosition presets[] = {
  {"home", 90, 30},
  {"left", 0, 45},
  {"right", 180, 45},
  {"up", 45, 20},
  {"down", 135, 20}
};

const int NUM_PRESETS = sizeof(presets) / sizeof(presets[0]);

void setup() {
  Serial.begin(9600);
  Serial.println("🎯 서보 모터 고급 제어 시스템");
  
  myServo.attach(9);
  myServo.write(servo.currentPosition);
  
  Serial.println("명령어:");
  Serial.println("숫자 0-180: 각도 설정");
  Serial.println("s30: 속도 30도/초로 설정");
  Serial.println("preset: home, left, right, up, down");
  Serial.println("smooth: 부드러운 움직임 토글");
  Serial.println("sweep: 자동 스윕 모드");
  
  showPresets();
}

void loop() {
  // 시리얼 명령 처리
  handleCommands();
  
  // 서보 위치 업데이트
  updateServoPosition();
  
  // 상태 출력 (1초마다)
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 1000) {
    printStatus();
    lastStatus = millis();
  }
}

void handleCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toLowerCase();
    
    if (command.startsWith("s")) {
      // 속도 설정 (예: s30)
      float newSpeed = command.substring(1).toFloat();
      if (newSpeed > 0 && newSpeed <= 180) {
        servo.speed = newSpeed;
        servo.maxSpeed = newSpeed;
        Serial.print("🚀 속도 설정: ");
        Serial.print(newSpeed);
        Serial.println("도/초");
      } else {
        Serial.println("❌ 유효 범위: 1-180 도/초");
      }
      
    } else if (command == "smooth") {
      // 부드러운 움직임 토글
      static bool smoothMode = false;
      smoothMode = !smoothMode;
      servo.acceleration = smoothMode ? 100.0 : 1000.0;
      Serial.print("🌊 부드러운 움직임: ");
      Serial.println(smoothMode ? "활성화" : "비활성화");
      
    } else if (command == "sweep") {
      // 자동 스윕 모드
      Serial.println("🔄 자동 스윕 시작 (5초간)");
      performSweep();
      
    } else if (command == "stop") {
      // 즉시 정지
      servo.targetPosition = servo.currentPosition;
      servo.currentSpeed = 0;
      servo.isMoving = false;
      Serial.println("⏹️ 즉시 정지");
      
    } else if (isPresetCommand(command)) {
      // 프리셋 위치 이동
      executePreset(command);
      
    } else {
      // 각도 설정
      int angle = command.toInt();
      if (angle >= servo.minPosition && angle <= servo.maxPosition) {
        moveToPosition(angle);
        Serial.print("📍 목표 위치: ");
        Serial.print(angle);
        Serial.println("도");
      } else {
        Serial.print("❌ 유효 범위: ");
        Serial.print(servo.minPosition);
        Serial.print("-");
        Serial.print(servo.maxPosition);
        Serial.println("도");
      }
    }
  }
}

void updateServoPosition() {
  if (!servo.isMoving) return;
  
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - servo.lastUpdate) / 1000.0; // 초 단위
  servo.lastUpdate = currentTime;
  
  if (deltaTime > 0.1) deltaTime = 0.1; // 최대 0.1초로 제한
  
  int positionDiff = servo.targetPosition - servo.currentPosition;
  
  if (abs(positionDiff) <= 1) {
    // 목표 위치 도달
    servo.currentPosition = servo.targetPosition;
    servo.currentSpeed = 0;
    servo.isMoving = false;
    myServo.write(servo.currentPosition);
    Serial.println("✅ 목표 위치 도달");
    return;
  }
  
  // 가속도 기반 속도 제어
  float targetSpeed = (positionDiff > 0) ? servo.maxSpeed : -servo.maxSpeed;
  
  // 감속 구간 계산
  float brakingDistance = (servo.currentSpeed * servo.currentSpeed) / (2 * servo.acceleration);
  if (abs(positionDiff) <= brakingDistance) {
    targetSpeed = (positionDiff > 0) ? 1 : -1; // 최소 속도로 감속
  }
  
  // 가속도 적용
  if (servo.currentSpeed < targetSpeed) {
    servo.currentSpeed += servo.acceleration * deltaTime;
    if (servo.currentSpeed > targetSpeed) servo.currentSpeed = targetSpeed;
  } else if (servo.currentSpeed > targetSpeed) {
    servo.currentSpeed -= servo.acceleration * deltaTime;
    if (servo.currentSpeed < targetSpeed) servo.currentSpeed = targetSpeed;
  }
  
  // 위치 업데이트
  float positionChange = servo.currentSpeed * deltaTime;
  servo.currentPosition += (int)positionChange;
  
  // 범위 제한
  servo.currentPosition = constrain(servo.currentPosition, 
                                   servo.minPosition, servo.maxPosition);
  
  // 서보 위치 설정
  myServo.write(servo.currentPosition);
}

void moveToPosition(int position) {
  servo.targetPosition = constrain(position, servo.minPosition, servo.maxPosition);
  servo.isMoving = true;
  servo.lastUpdate = millis();
}

bool isPresetCommand(String command) {
  for (int i = 0; i < NUM_PRESETS; i++) {
    if (command == presets[i].name) {
      return true;
    }
  }
  return false;
}

void executePreset(String command) {
  for (int i = 0; i < NUM_PRESETS; i++) {
    if (command == presets[i].name) {
      servo.maxSpeed = presets[i].speed;
      moveToPosition(presets[i].angle);
      Serial.print("🎯 프리셋 '");
      Serial.print(presets[i].name);
      Serial.print("' 실행: ");
      Serial.print(presets[i].angle);
      Serial.println("도");
      return;
    }
  }
}

void performSweep() {
  unsigned long sweepStart = millis();
  bool direction = true; // true: 증가, false: 감소
  
  while (millis() - sweepStart < 5000) {
    if (!servo.isMoving) {
      if (direction) {
        if (servo.currentPosition >= servo.maxPosition - 10) {
          direction = false;
          moveToPosition(servo.minPosition);
        } else {
          moveToPosition(servo.maxPosition);
        }
      } else {
        if (servo.currentPosition <= servo.minPosition + 10) {
          direction = true;
          moveToPosition(servo.maxPosition);
        } else {
          moveToPosition(servo.minPosition);
        }
      }
    }
    
    updateServoPosition();
    delay(20);
  }
  
  // 중앙 위치로 복귀
  moveToPosition(90);
  Serial.println("🏠 중앙 위치로 복귀");
}

void showPresets() {
  Serial.println("📋 사용 가능한 프리셋:");
  for (int i = 0; i < NUM_PRESETS; i++) {
    Serial.print("  ");
    Serial.print(presets[i].name);
    Serial.print(": ");
    Serial.print(presets[i].angle);
    Serial.print("도 (");
    Serial.print(presets[i].speed);
    Serial.println("도/초)");
  }
}

void printStatus() {
  Serial.print("📊 위치: ");
  Serial.print(servo.currentPosition);
  Serial.print("도 → ");
  Serial.print(servo.targetPosition);
  Serial.print("도, 속도: ");
  Serial.print(servo.currentSpeed, 1);
  Serial.print("도/초");
  
  if (servo.isMoving) {
    Serial.println(" [이동중]");
  } else {
    Serial.println(" [정지]");
  }
}
```

## ⚙️ 4. 스테퍼 모터 정밀 제어

### 4.1 스테퍼 모터 마이크로스테핑 제어
```cpp
/*
  Stepper_Microstep_Control.ino
  스테퍼 모터 마이크로스테핑 제어 - A4988 드라이버 사용
  
  연결:
  A4988 STEP → Pin 2
  A4988 DIR  → Pin 3
  A4988 EN   → Pin 4
  A4988 MS1  → Pin 5
  A4988 MS2  → Pin 6  
  A4988 MS3  → Pin 7
*/

#include <AccelStepper.h>

// 스테퍼 모터 핀 정의
const int STEP_PIN = 2;
const int DIR_PIN = 3;
const int ENABLE_PIN = 4;

// 마이크로스테핑 핀
const int MS1_PIN = 5;
const int MS2_PIN = 6;
const int MS3_PIN = 7;

// 스테퍼 모터 객체 생성
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// 마이크로스테핑 설정
enum MicrostepMode {
  FULL_STEP = 0,      // 1 step
  HALF_STEP = 1,      // 1/2 step
  QUARTER_STEP = 2,   // 1/4 step
  EIGHTH_STEP = 3,    // 1/8 step
  SIXTEENTH_STEP = 4  // 1/16 step
};

// 마이크로스테핑 설정 테이블
bool microstepSettings[5][3] = {
  //MS1, MS2, MS3
  {0, 0, 0},  // Full step
  {1, 0, 0},  // Half step
  {0, 1, 0},  // Quarter step
  {1, 1, 0},  // Eighth step
  {1, 1, 1}   // Sixteenth step
};

String microstepNames[5] = {
  "Full (1/1)", "Half (1/2)", "Quarter (1/4)", 
  "Eighth (1/8)", "Sixteenth (1/16)"
};

// 모터 특성
const int STEPS_PER_REV = 200;  // 1.8도 스테퍼 모터
int currentMicrostep = SIXTEENTH_STEP;
int stepsPerRevolution = STEPS_PER_REV * 16; // 16 마이크로스텝

// 제어 변수
bool motorEnabled = true;
long targetPosition = 0;
float currentSpeed = 800;    // steps/sec
float acceleration = 1000;   // steps/sec²

void setup() {
  Serial.begin(115200);
  Serial.println("⚙️ 스테퍼 모터 마이크로스테핑 제어");
  
  // 핀 설정
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  
  // 스테퍼 모터 설정
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(acceleration);
  stepper.setSpeed(currentSpeed);
  
  // 초기 마이크로스테핑 설정
  setMicrostepMode(currentMicrostep);
  enableMotor(true);
  
  Serial.println("명령어:");
  Serial.println("r<각도>: 상대 회전 (예: r90, r-180)");
  Serial.println("a<각도>: 절대 위치 (예: a45)");
  Serial.println("s<속도>: 속도 설정 (예: s1000)");
  Serial.println("m<모드>: 마이크로스테핑 0-4");
  Serial.println("home: 원점 복귀");
  Serial.println("enable/disable: 모터 활성화/비활성화");
  Serial.println("info: 상태 정보");
  
  showMicrostepModes();
}

void loop() {
  // 시리얼 명령 처리
  handleCommands();
  
  // 스테퍼 모터 실행
  stepper.run();
  
  // 위치 도달 확인
  static bool wasRunning = false;
  bool isRunning = stepper.isRunning();
  
  if (wasRunning && !isRunning) {
    Serial.println("✅ 목표 위치 도달");
    printCurrentStatus();
  }
  wasRunning = isRunning;
  
  // 주기적 상태 출력 (5초마다, 움직이지 않을 때만)
  static unsigned long lastStatus = 0;
  if (!isRunning && millis() - lastStatus > 5000) {
    printCurrentStatus();
    lastStatus = millis();
  }
}

void handleCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toLowerCase();
    
    if (command.startsWith("r")) {
      // 상대 회전
      float angle = command.substring(1).toFloat();
      if (angle != 0) {
        rotateRelative(angle);
      }
      
    } else if (command.startsWith("a")) {
      // 절대 위치
      float angle = command.substring(1).toFloat();
      rotateAbsolute(angle);
      
    } else if (command.startsWith("s")) {
      // 속도 설정
      float speed = command.substring(1).toFloat();
      if (speed > 0 && speed <= 2000) {
        currentSpeed = speed;
        stepper.setMaxSpeed(speed);
        Serial.print("🚀 속도 설정: ");
        Serial.print(speed);
        Serial.println(" steps/sec");
      } else {
        Serial.println("❌ 유효 범위: 1-2000 steps/sec");
      }
      
    } else if (command.startsWith("m")) {
      // 마이크로스테핑 모드
      int mode = command.substring(1).toInt();
      if (mode >= 0 && mode <= 4) {
        setMicrostepMode(mode);
      } else {
        Serial.println("❌ 유효 범위: 0-4");
      }
      
    } else if (command == "home") {
      // 원점 복귀
      rotateAbsolute(0);
      Serial.println("🏠 원점 복귀");
      
    } else if (command == "enable") {
      enableMotor(true);
      
    } else if (command == "disable") {
      enableMotor(false);
      
    } else if (command == "stop") {
      stepper.stop();
      Serial.println("⏹️ 긴급 정지");
      
    } else if (command == "info") {
      showDetailedInfo();
      
    } else if (command == "help") {
      showHelp();
      
    } else {
      Serial.println("❓ 알 수 없는 명령어 (help 입력)");
    }
  }
}

void rotateRelative(float degrees) {
  long steps = degreesToSteps(degrees);
  targetPosition = stepper.currentPosition() + steps;
  stepper.moveTo(targetPosition);
  
  Serial.print("🔄 상대 회전: ");
  Serial.print(degrees);
  Serial.print("도 (");
  Serial.print(steps);
  Serial.println(" steps)");
}

void rotateAbsolute(float degrees) {
  long steps = degreesToSteps(degrees);
  targetPosition = steps;
  stepper.moveTo(targetPosition);
  
  Serial.print("📍 절대 위치: ");
  Serial.print(degrees);
  Serial.print("도 (");
  Serial.print(steps);
  Serial.println(" steps)");
}

long degreesToSteps(float degrees) {
  return (long)(degrees * stepsPerRevolution / 360.0);
}

float stepsToDegrees(long steps) {
  return (float)steps * 360.0 / stepsPerRevolution;
}

void setMicrostepMode(int mode) {
  if (mode < 0 || mode > 4) return;
  
  currentMicrostep = mode;
  
  // MS1, MS2, MS3 핀 설정
  digitalWrite(MS1_PIN, microstepSettings[mode][0]);
  digitalWrite(MS2_PIN, microstepSettings[mode][1]);
  digitalWrite(MS3_PIN, microstepSettings[mode][2]);
  
  // steps per revolution 업데이트
  int multiplier = 1;
  switch (mode) {
    case FULL_STEP: multiplier = 1; break;
    case HALF_STEP: multiplier = 2; break;
    case QUARTER_STEP: multiplier = 4; break;
    case EIGHTH_STEP: multiplier = 8; break;
    case SIXTEENTH_STEP: multiplier = 16; break;
  }
  
  stepsPerRevolution = STEPS_PER_REV * multiplier;
  
  Serial.print("⚙️ 마이크로스테핑: ");
  Serial.print(microstepNames[mode]);
  Serial.print(" (");
  Serial.print(stepsPerRevolution);
  Serial.println(" steps/rev)");
}

void enableMotor(bool enable) {
  motorEnabled = enable;
  digitalWrite(ENABLE_PIN, !enable); // A4988은 LOW가 활성화
  
  Serial.print("🔌 모터: ");
  Serial.println(enable ? "활성화" : "비활성화");
}

void showMicrostepModes() {
  Serial.println("📋 마이크로스테핑 모드:");
  for (int i = 0; i < 5; i++) {
    Serial.print("  ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(microstepNames[i]);
  }
}

void printCurrentStatus() {
  float currentAngle = stepsToDegrees(stepper.currentPosition());
  float targetAngle = stepsToDegrees(targetPosition);
  
  Serial.println("📊 현재 상태:");
  Serial.print("  위치: ");
  Serial.print(currentAngle, 2);
  Serial.print("도 (");
  Serial.print(stepper.currentPosition());
  Serial.println(" steps)");
  
  if (stepper.isRunning()) {
    Serial.print("  목표: ");
    Serial.print(targetAngle, 2);
    Serial.print("도 (");
    Serial.print(targetPosition);
    Serial.println(" steps)");
    
    Serial.print("  속도: ");
    Serial.print(stepper.speed(), 1);
    Serial.println(" steps/sec");
  }
  
  Serial.print("  마이크로스테핑: ");
  Serial.println(microstepNames[currentMicrostep]);
  
  Serial.print("  모터: ");
  Serial.println(motorEnabled ? "활성화" : "비활성화");
}

void showDetailedInfo() {
  Serial.println("🔍 상세 정보:");
  Serial.print("  모터 타입: ");
  Serial.print(STEPS_PER_REV);
  Serial.println(" steps/rev (1.8도/step)");
  
  Serial.print("  현재 해상도: ");
  Serial.print(360.0 / stepsPerRevolution, 4);
  Serial.println("도/step");
  
  Serial.print("  최대 속도: ");
  Serial.print(stepper.maxSpeed());
  Serial.println(" steps/sec");
  
  Serial.print("  가속도: ");
  Serial.print(acceleration);
  Serial.println(" steps/sec²");
  
  printCurrentStatus();
}

void showHelp() {
  Serial.println("📚 명령어 도움말:");
  Serial.println("  r<각도>  - 상대 회전 (r90: 90도 시계방향)");
  Serial.println("  a<각도>  - 절대 위치 (a180: 180도 위치)");
  Serial.println("  s<속도>  - 속도 설정 (s500: 500 steps/sec)");
  Serial.println("  m<0-4>   - 마이크로스테핑 모드");
  Serial.println("  home     - 원점 복귀");
  Serial.println("  enable   - 모터 활성화");
  Serial.println("  disable  - 모터 비활성화");
  Serial.println("  stop     - 긴급 정지");
  Serial.println("  info     - 상세 상태 정보");
  Serial.println("  help     - 이 도움말");
}
```

## 🤖 5. 로봇 팔 제어 시스템

### 5.1 다관절 로봇 팔 통합 제어
```cpp
/*
  Robot_Arm_Control.ino
  6축 로봇 팔 통합 제어 시스템
  
  구성:
  - Base 회전: 스테퍼 모터
  - Shoulder: 서보 모터 (큰 토크)
  - Elbow: 서보 모터
  - Wrist Pitch: 서보 모터
  - Wrist Roll: 서보 모터  
  - Gripper: 서보 모터
*/

#include <Servo.h>
#include <AccelStepper.h>

// 서보 모터 객체
Servo shoulderServo;
Servo elbowServo;
Servo wristPitchServo;
Servo wristRollServo;
Servo gripperServo;

// 스테퍼 모터 (Base)
AccelStepper baseStepper(AccelStepper::DRIVER, 2, 3);

// 로봇 팔 구조체
struct RobotArm {
  // 현재 각도
  float baseAngle;        // 0-360도
  float shoulderAngle;    // 0-180도
  float elbowAngle;       // 0-180도
  float wristPitchAngle;  // 0-180도
  float wristRollAngle;   // 0-180도
  float gripperAngle;     // 0-90도 (0=열림, 90=닫힘)
  
  // 목표 각도
  float targetBase;
  float targetShoulder;
  float targetElbow;
  float targetWristPitch;
  float targetWristRoll;
  float targetGripper;
  
  // 이동 속도 (도/초)
  float baseSpeed;
  float servoSpeed;
  
  // 상태
  bool isMoving;
  unsigned long moveStartTime;
  unsigned long moveDuration;
};

RobotArm arm;

// 사전 정의된 포지션
struct RobotPosition {
  String name;
  float base, shoulder, elbow, wristPitch, wristRoll, gripper;
};

RobotPosition positions[] = {
  {"home",    0,   90,  90,  90,  90,  45},
  {"rest",    0,  150, 150,  90,  90,   0},
  {"pickup", 45,   60,  30,  60,  90,   0},
  {"place",  90,   80,  60,  45,  90,  90},
  {"wave",   0,    45,  45, 135,   0,  45}
};

const int NUM_POSITIONS = sizeof(positions) / sizeof(positions[0]);

void setup() {
  Serial.begin(115200);
  Serial.println("🤖 6축 로봇 팔 제어 시스템");
  
  // 서보 모터 연결
  shoulderServo.attach(6);
  elbowServo.attach(7);
  wristPitchServo.attach(8);
  wristRollServo.attach(9);
  gripperServo.attach(10);
  
  // 스테퍼 모터 설정
  baseStepper.setMaxSpeed(800);
  baseStepper.setAcceleration(400);
  
  // 초기 위치 설정
  initializeArm();
  
  Serial.println("명령어:");
  Serial.println("pos <name>: 사전 정의 위치로 이동");
  Serial.println("move <joint> <angle>: 개별 관절 제어");
  Serial.println("speed <value>: 이동 속도 설정");
  Serial.println("sequence: 데모 시퀀스 실행");
  Serial.println("current: 현재 위치 출력");
  
  showPositions();
}

void loop() {
  // 시리얼 명령 처리
  handleCommands();
  
  // 스테퍼 모터 실행
  baseStepper.run();
  
  // 서보 모터 부드러운 움직임 처리
  updateServoMovement();
  
  // 상태 출력 (5초마다)
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000 && !arm.isMoving) {
    printCurrentPosition();
    lastStatus = millis();
  }
}

void initializeArm() {
  // 초기 위치 설정 (home 포지션)
  arm.baseAngle = 0;
  arm.shoulderAngle = 90;
  arm.elbowAngle = 90;
  arm.wristPitchAngle = 90;
  arm.wristRollAngle = 90;
  arm.gripperAngle = 45;
  
  // 목표 위치도 동일하게 설정
  arm.targetBase = arm.baseAngle;
  arm.targetShoulder = arm.shoulderAngle;
  arm.targetElbow = arm.elbowAngle;
  arm.targetWristPitch = arm.wristPitchAngle;
  arm.targetWristRoll = arm.wristRollAngle;
  arm.targetGripper = arm.gripperAngle;
  
  // 속도 설정
  arm.baseSpeed = 90;    // 90도/초
  arm.servoSpeed = 45;   // 45도/초
  
  // 서보 모터 초기 위치 설정
  shoulderServo.write(arm.shoulderAngle);
  elbowServo.write(arm.elbowAngle);
  wristPitchServo.write(arm.wristPitchAngle);
  wristRollServo.write(arm.wristRollAngle);
  gripperServo.write(arm.gripperAngle);
  
  arm.isMoving = false;
  
  Serial.println("✅ 로봇 팔 초기화 완료 (Home 위치)");
}

void handleCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toLowerCase();
    
    if (command.startsWith("pos ")) {
      // 사전 정의 위치로 이동
      String posName = command.substring(4);
      moveToPosition(posName);
      
    } else if (command.startsWith("move ")) {
      // 개별 관절 제어
      handleJointMove(command);
      
    } else if (command.startsWith("speed ")) {
      // 속도 설정
      float speed = command.substring(6).toFloat();
      if (speed > 0 && speed <= 180) {
        arm.servoSpeed = speed;
        arm.baseSpeed = speed * 2; // Base는 조금 더 빠르게
        Serial.print("🚀 속도 설정: ");
        Serial.print(speed);
        Serial.println("도/초");
      } else {
        Serial.println("❌ 유효 범위: 1-180 도/초");
      }
      
    } else if (command == "sequence") {
      // 데모 시퀀스 실행
      executeSequence();
      
    } else if (command == "current") {
      // 현재 위치 출력
      printCurrentPosition();
      
    } else if (command == "stop") {
      // 모든 움직임 정지
      stopAllMovement();
      
    } else if (command == "home") {
      // 홈 위치로 이동
      moveToPosition("home");
      
    } else {
      Serial.println("❓ 알 수 없는 명령어");
    }
  }
}

void handleJointMove(String command) {
  // "move shoulder 45" 형식 파싱
  int firstSpace = command.indexOf(' ', 5);
  if (firstSpace == -1) return;
  
  String joint = command.substring(5, firstSpace);
  float angle = command.substring(firstSpace + 1).toFloat();
  
  if (joint == "base") {
    moveJoint("base", angle);
  } else if (joint == "shoulder") {
    moveJoint("shoulder", angle);
  } else if (joint == "elbow") {
    moveJoint("elbow", angle);
  } else if (joint == "wrist_pitch" || joint == "pitch") {
    moveJoint("wrist_pitch", angle);
  } else if (joint == "wrist_roll" || joint == "roll") {
    moveJoint("wrist_roll", angle);
  } else if (joint == "gripper") {
    moveJoint("gripper", angle);
  } else {
    Serial.println("❌ 유효 관절: base, shoulder, elbow, wrist_pitch, wrist_roll, gripper");
  }
}

void moveJoint(String joint, float angle) {
  if (joint == "base") {
    arm.targetBase = constrain(angle, 0, 360);
    Serial.print("🔄 Base → ");
    Serial.print(arm.targetBase);
    Serial.println("도");
    
  } else if (joint == "shoulder") {
    arm.targetShoulder = constrain(angle, 0, 180);
    Serial.print("💪 Shoulder → ");
    Serial.print(arm.targetShoulder);
    Serial.println("도");
    
  } else if (joint == "elbow") {
    arm.targetElbow = constrain(angle, 0, 180);
    Serial.print("⭕ Elbow → ");
    Serial.print(arm.targetElbow);
    Serial.println("도");
    
  } else if (joint == "wrist_pitch") {
    arm.targetWristPitch = constrain(angle, 0, 180);
    Serial.print("📐 Wrist Pitch → ");
    Serial.print(arm.targetWristPitch);
    Serial.println("도");
    
  } else if (joint == "wrist_roll") {
    arm.targetWristRoll = constrain(angle, 0, 180);
    Serial.print("🔄 Wrist Roll → ");
    Serial.print(arm.targetWristRoll);
    Serial.println("도");
    
  } else if (joint == "gripper") {
    arm.targetGripper = constrain(angle, 0, 90);
    Serial.print("✋ Gripper → ");
    Serial.print(arm.targetGripper);
    Serial.println("도");
  }
  
  startMovement();
}

void moveToPosition(String posName) {
  for (int i = 0; i < NUM_POSITIONS; i++) {
    if (positions[i].name == posName) {
      arm.targetBase = positions[i].base;
      arm.targetShoulder = positions[i].shoulder;
      arm.targetElbow = positions[i].elbow;
      arm.targetWristPitch = positions[i].wristPitch;
      arm.targetWristRoll = positions[i].wristRoll;
      arm.targetGripper = positions[i].gripper;
      
      startMovement();
      
      Serial.print("🎯 위치 '");
      Serial.print(posName);
      Serial.println("'로 이동 시작");
      
      return;
    }
  }
  
  Serial.print("❌ 알 수 없는 위치: ");
  Serial.println(posName);
}

void startMovement() {
  arm.isMoving = true;
  arm.moveStartTime = millis();
  
  // Base 스테퍼 모터 이동 시작
  long baseSteps = (long)(arm.targetBase * 800 / 360); // 800 steps = 360도
  baseStepper.moveTo(baseSteps);
  
  // 예상 이동 시간 계산 (가장 긴 이동 시간 기준)
  float maxAngleChange = 0;
  maxAngleChange = max(maxAngleChange, abs(arm.targetShoulder - arm.shoulderAngle));
  maxAngleChange = max(maxAngleChange, abs(arm.targetElbow - arm.elbowAngle));
  maxAngleChange = max(maxAngleChange, abs(arm.targetWristPitch - arm.wristPitchAngle));
  maxAngleChange = max(maxAngleChange, abs(arm.targetWristRoll - arm.wristRollAngle));
  maxAngleChange = max(maxAngleChange, abs(arm.targetGripper - arm.gripperAngle));
  
  arm.moveDuration = (unsigned long)(maxAngleChange / arm.servoSpeed * 1000);
  
  Serial.print("⏱️ 예상 이동 시간: ");
  Serial.print(arm.moveDuration / 1000.0, 1);
  Serial.println("초");
}

void updateServoMovement() {
  if (!arm.isMoving) return;
  
  unsigned long elapsed = millis() - arm.moveStartTime;
  
  if (elapsed >= arm.moveDuration) {
    // 이동 완료
    arm.shoulderAngle = arm.targetShoulder;
    arm.elbowAngle = arm.targetElbow;
    arm.wristPitchAngle = arm.targetWristPitch;
    arm.wristRollAngle = arm.targetWristRoll;
    arm.gripperAngle = arm.targetGripper;
    arm.baseAngle = arm.targetBase;
    
    // 최종 위치 설정
    shoulderServo.write(arm.shoulderAngle);
    elbowServo.write(arm.elbowAngle);
    wristPitchServo.write(arm.wristPitchAngle);
    wristRollServo.write(arm.wristRollAngle);
    gripperServo.write(arm.gripperAngle);
    
    arm.isMoving = false;
    Serial.println("✅ 목표 위치 도달 완료");
    
  } else {
    // 진행률 계산
    float progress = (float)elapsed / arm.moveDuration;
    
    // 각 관절의 현재 위치 계산 (선형 보간)
    float currentShoulder = arm.shoulderAngle + (arm.targetShoulder - arm.shoulderAngle) * progress;
    float currentElbow = arm.elbowAngle + (arm.targetElbow - arm.elbowAngle) * progress;
    float currentWristPitch = arm.wristPitchAngle + (arm.targetWristPitch - arm.wristPitchAngle) * progress;
    float currentWristRoll = arm.wristRollAngle + (arm.targetWristRoll - arm.wristRollAngle) * progress;
    float currentGripper = arm.gripperAngle + (arm.targetGripper - arm.gripperAngle) * progress;
    
    // 서보 모터 위치 업데이트
    shoulderServo.write(currentShoulder);
    elbowServo.write(currentElbow);
    wristPitchServo.write(currentWristPitch);
    wristRollServo.write(currentWristRoll);
    gripperServo.write(currentGripper);
  }
}

void executeSequence() {
  Serial.println("🎬 데모 시퀀스 시작!");
  
  String sequence[] = {"home", "pickup", "place", "wave", "rest", "home"};
  int delays[] = {2000, 3000, 3000, 2000, 2000, 2000}; // 각 위치별 대기 시간
  
  for (int i = 0; i < 6; i++) {
    Serial.print("단계 ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(sequence[i]);
    
    moveToPosition(sequence[i]);
    
    // 이동 완료까지 대기
    while (arm.isMoving || baseStepper.isRunning()) {
      baseStepper.run();
      updateServoMovement();
      delay(20);
    }
    
    delay(delays[i]);
  }
  
  Serial.println("🎉 데모 시퀀스 완료!");
}

void stopAllMovement() {
  arm.isMoving = false;
  baseStepper.stop();
  
  // 현재 위치를 목표 위치로 설정
  arm.targetBase = arm.baseAngle;
  arm.targetShoulder = arm.shoulderAngle;
  arm.targetElbow = arm.elbowAngle;
  arm.targetWristPitch = arm.wristPitchAngle;
  arm.targetWristRoll = arm.wristRollAngle;
  arm.targetGripper = arm.gripperAngle;
  
  Serial.println("⏹️ 모든 움직임 정지");
}

void showPositions() {
  Serial.println("📋 사용 가능한 위치:");
  for (int i = 0; i < NUM_POSITIONS; i++) {
    Serial.print("  ");
    Serial.print(positions[i].name);
    Serial.print(": Base=");
    Serial.print(positions[i].base);
    Serial.print("° Shoulder=");
    Serial.print(positions[i].shoulder);
    Serial.print("° Elbow=");
    Serial.print(positions[i].elbow);
    Serial.print("° Pitch=");
    Serial.print(positions[i].wristPitch);
    Serial.print("° Roll=");
    Serial.print(positions[i].wristRoll);
    Serial.print("° Gripper=");
    Serial.print(positions[i].gripper);
    Serial.println("°");
  }
}

void printCurrentPosition() {
  Serial.println("📊 현재 로봇 팔 위치:");
  Serial.print("  Base: ");
  Serial.print(arm.baseAngle, 1);
  Serial.println("°");
  Serial.print("  Shoulder: ");
  Serial.print(arm.shoulderAngle, 1);
  Serial.println("°");
  Serial.print("  Elbow: ");
  Serial.print(arm.elbowAngle, 1);
  Serial.println("°");
  Serial.print("  Wrist Pitch: ");
  Serial.print(arm.wristPitchAngle, 1);
  Serial.println("°");
  Serial.print("  Wrist Roll: ");
  Serial.print(arm.wristRollAngle, 1);
  Serial.println("°");
  Serial.print("  Gripper: ");
  Serial.print(arm.gripperAngle, 1);
  Serial.println("°");
  
  if (arm.isMoving) {
    Serial.println("  상태: 이동 중");
  } else {
    Serial.println("  상태: 정지");
  }
}
```

## ✅ 핵심 정리

1. **DC 모터**는 연속 회전과 속도 제어에 최적화되어 있습니다.
2. **서보 모터**는 정확한 각도 제어가 필요한 용도에 사용됩니다.
3. **스테퍼 모터**는 정밀한 위치 제어와 오픈루프 제어가 가능합니다.
4. **엔코더 피드백**과 **PID 제어**로 정밀도를 크게 향상시킬 수 있습니다.
5. **마이크로스테핑**으로 스테퍼 모터의 해상도와 부드러움을 개선할 수 있습니다.

## 🚀 다음 단계

다음 레슨에서는 LCD 디스플레이 제어를 배워서 
시각적 피드백과 사용자 인터페이스를 구현하는 
방법을 알아보겠습니다.

## 💻 실습 과제

### 과제 1: 기본 모터 제어
1. 2륜 구동 로봇 원격 조종 시스템 구축
2. 엔코더를 이용한 정확한 거리 이동 구현
3. 서보 모터로 카메라 팬-틸트 시스템 제작

### 과제 2: 고급 제어 시스템
1. PID 제어를 이용한 라인 트레이싱 로봇
2. 스테퍼 모터 기반 2축 CNC 플로터
3. 다관절 로봇 팔 역기구학 구현

### 과제 3: 실전 응용 프로젝트
1. 자율 주행 AGV (Automated Guided Vehicle)
2. 델타 로봇 3D 프린터 시스템
3. 6축 로봇 팔 물체 분류 시스템

---

[← 이전 레슨: 센서 활용 심화](../08-advanced-sensors/README.md) | [다음 레슨: LCD 디스플레이 제어 →](../10-lcd-display/README.md)