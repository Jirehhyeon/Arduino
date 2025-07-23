# Lesson 02: Arduino IDE 설치 및 환경 설정

## 🎯 학습 목표
- Arduino IDE 2.0 설치하기
- IDE 인터페이스 이해하기
- 보드와 포트 설정하기
- 첫 번째 스케치 업로드 테스트하기

## 📖 1. Arduino IDE란?

### 1.1 정의
Arduino IDE(Integrated Development Environment)는 아두이노 프로그래밍을 위한 **통합 개발 환경**입니다.

### 1.2 주요 기능
- 코드 편집기 (문법 하이라이팅, 자동 완성)
- 컴파일러 (스케치를 기계어로 변환)
- 업로더 (컴파일된 코드를 보드에 전송)
- 시리얼 모니터 (보드와 통신)
- 라이브러리 매니저 (추가 기능 설치)

## 💾 2. Arduino IDE 설치하기

### 2.1 시스템 요구사항
- **Windows**: Windows 10 이상
- **macOS**: macOS 10.14 이상
- **Linux**: 64비트 배포판
- **RAM**: 최소 4GB (8GB 권장)
- **저장공간**: 최소 500MB

### 2.2 Windows 설치 과정

#### Step 1: 다운로드
1. [Arduino 공식 사이트](https://www.arduino.cc/en/software) 접속
2. "Download Arduino IDE" 클릭
3. Windows 버전 선택 (Win 10 and newer, 64 bits)
4. "JUST DOWNLOAD" 클릭 (기부는 선택사항)

#### Step 2: 설치
```
1. 다운로드된 arduino-ide_2.x.x_Windows_64bit.exe 실행
2. 설치 마법사 시작
   - License Agreement → "I Agree"
   - Installation Options → 모두 체크
   - Install Location → 기본값 사용 권장
   - "Install" 클릭
3. 설치 완료 후 "Close" 클릭
```

#### Step 3: 드라이버 설치
```
설치 중 나타나는 드라이버 설치 창:
- "이 드라이버 소프트웨어를 설치하시겠습니까?" → "설치"
- Arduino USB Driver
- Genuino USB Driver
- Arduino USB Driver (CDC ACM)
```

### 2.3 macOS 설치 과정

```bash
# 1. DMG 파일 다운로드 후 더블클릭
# 2. Arduino IDE를 Applications 폴더로 드래그
# 3. 첫 실행시 "개발자를 확인할 수 없습니다" 메시지가 나타나면:
#    시스템 환경설정 → 보안 및 개인정보 보호 → "확인 없이 열기" 클릭
```

### 2.4 Linux 설치 과정

```bash
# AppImage 방식 (권장)
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.x.x_Linux_64bit.AppImage
chmod +x arduino-ide_2.x.x_Linux_64bit.AppImage
./arduino-ide_2.x.x_Linux_64bit.AppImage

# 사용자를 dialout 그룹에 추가 (시리얼 포트 접근 권한)
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인 필요
```

## 🖥️ 3. Arduino IDE 인터페이스 둘러보기

### 3.1 메인 화면 구성

```
┌─────────────────────────────────────────────────────┐
│ File Edit Sketch Tools Help                         │ ← 메뉴바
├─────────────────────────────────────────────────────┤
│ ✓ □ → ⚙ 🔍                                         │ ← 툴바
├─────────────────────────────────────────────────────┤
│ sketch_name.ino                                     │ ← 탭
├─────────────────────────────────────────────────────┤
│ void setup() {                                      │
│   // 초기화 코드                                     │ ← 코드 에디터
│ }                                                   │
│                                                     │
│ void loop() {                                       │
│   // 반복 실행 코드                                  │
│ }                                                   │
├─────────────────────────────────────────────────────┤
│ 아두이노 Uno on COM3                                │ ← 상태바
├─────────────────────────────────────────────────────┤
│ Output | Serial Monitor | Serial Plotter            │ ← 출력 창
│ Compiling sketch...                                 │
└─────────────────────────────────────────────────────┘
```

### 3.2 주요 버튼 설명

| 버튼 | 이름 | 단축키 | 기능 |
|-----|------|--------|------|
| ✓ | Verify | Ctrl+R | 코드 컴파일 (오류 검사) |
| → | Upload | Ctrl+U | 컴파일 후 보드에 업로드 |
| □ | Debug | - | 디버깅 모드 (지원 보드만) |
| 📁 | Open | Ctrl+O | 스케치 파일 열기 |
| 💾 | Save | Ctrl+S | 스케치 저장 |
| 🔍 | Serial Monitor | Ctrl+Shift+M | 시리얼 통신 창 열기 |

### 3.3 주요 메뉴 기능

#### File 메뉴
- **New Sketch**: 새 스케치 생성
- **Open**: 기존 스케치 열기
- **Examples**: 예제 코드 모음
- **Save As**: 다른 이름으로 저장
- **Preferences**: 환경 설정

#### Tools 메뉴
- **Board**: 아두이노 보드 선택
- **Port**: 통신 포트 선택
- **Programmer**: 프로그래머 선택
- **Manage Libraries**: 라이브러리 관리
- **Board Manager**: 보드 패키지 관리

## ⚙️ 4. 아두이노 보드 연결 및 설정

### 4.1 보드 연결
1. USB 케이블로 아두이노와 컴퓨터 연결
2. 전원 LED(ON) 확인
3. 장치 관리자에서 포트 확인

### 4.2 보드 선택
```
1. Tools → Board → Arduino AVR Boards → Arduino Uno
2. 또는 상태바에서 "Select Board" 클릭
3. "Arduino Uno" 검색 후 선택
```

### 4.3 포트 선택

#### Windows
```
Tools → Port → COM3 (Arduino Uno)
* COM 번호는 시스템마다 다를 수 있음
```

#### macOS
```
Tools → Port → /dev/cu.usbmodem14201 (Arduino Uno)
```

#### Linux
```
Tools → Port → /dev/ttyACM0 (Arduino Uno)
또는 /dev/ttyUSB0
```

### 4.4 포트 문제 해결

#### Windows
```cmd
# 장치 관리자에서 확인
1. Win + X → 장치 관리자
2. "포트(COM & LPT)" 확장
3. "Arduino Uno (COMx)" 확인
4. 없다면 드라이버 재설치
```

#### Linux
```bash
# 포트 권한 문제 해결
ls -l /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0
# 또는 사용자를 dialout 그룹에 추가
sudo usermod -a -G dialout $USER
```

## 🧪 5. 첫 번째 업로드 테스트

### 5.1 Blink 예제 열기
```
File → Examples → 01.Basics → Blink
```

### 5.2 코드 이해하기
```cpp
// LED가 연결된 핀 번호
int ledPin = 13;  // 대부분의 아두이노 보드는 13번 핀에 내장 LED 있음

void setup() {
  // 13번 핀을 출력 모드로 설정
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);  // LED 켜기
  delay(1000);                 // 1초 대기
  digitalWrite(ledPin, LOW);   // LED 끄기
  delay(1000);                 // 1초 대기
}
```

### 5.3 업로드 과정
1. **Verify** (✓) 버튼 클릭 → 컴파일
2. **Upload** (→) 버튼 클릭 → 업로드
3. 업로드 중 TX/RX LED 깜빡임 확인
4. "Done uploading" 메시지 확인
5. 보드의 13번 핀 LED가 1초 간격으로 깜빡이는지 확인

### 5.4 업로드 문제 해결

#### 일반적인 오류와 해결방법

| 오류 메시지 | 원인 | 해결 방법 |
|-----------|------|----------|
| "Port not found" | 포트 미선택/인식 오류 | 포트 재선택, USB 재연결 |
| "Access denied" | 포트 권한 문제 | 관리자 권한 실행, 권한 설정 |
| "Programmer not responding" | 보드 선택 오류 | 올바른 보드 선택 |
| "Sketch too big" | 메모리 부족 | 코드 최적화, 큰 보드 사용 |

## 🛠️ 6. IDE 환경 설정

### 6.1 기본 설정 (File → Preferences)

```
Editor:
□ Display line numbers          ← 체크 권장
□ Enable code folding          ← 체크 권장
Font size: 14                   ← 가독성에 맞게 조정
Theme: Dark                     ← 개인 취향

Compile:
☑ Show verbose output during: compilation  ← 디버깅시 유용
☑ Show verbose output during: upload       ← 문제 해결시 필요

Network:
□ Use proxy                    ← 필요시 설정
```

### 6.2 추가 보드 매니저 URL
```
File → Preferences → Additional Boards Manager URLs:
ESP8266: http://arduino.esp8266.com/stable/package_esp8266com_index.json
ESP32: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

### 6.3 라이브러리 설치
```
Tools → Manage Libraries...
1. 검색창에 라이브러리 이름 입력
2. 원하는 라이브러리 선택
3. "Install" 클릭
4. 의존성 있으면 "Install All" 선택
```

## 📊 7. 시리얼 모니터 사용하기

### 7.1 시리얼 모니터 열기
- 메뉴: Tools → Serial Monitor
- 단축키: Ctrl+Shift+M
- 툴바: 돋보기(🔍) 아이콘

### 7.2 기본 시리얼 통신 예제
```cpp
void setup() {
  Serial.begin(9600);  // 시리얼 통신 시작 (9600 bps)
  Serial.println("Arduino IDE 설정 완료!");
}

void loop() {
  Serial.print("실행 시간: ");
  Serial.print(millis() / 1000);  // 초 단위로 출력
  Serial.println(" 초");
  delay(1000);
}
```

### 7.3 시리얼 모니터 설정
- **Baud Rate**: 9600 (코드와 일치해야 함)
- **Line Ending**: "Both NL & CR"
- **Autoscroll**: 체크 (자동 스크롤)

## ✅ 핵심 정리

1. **Arduino IDE 2.0**은 향상된 UI와 자동완성 기능을 제공합니다.
2. **보드와 포트 설정**이 올바르게 되어야 업로드가 가능합니다.
3. **시리얼 모니터**는 디버깅과 데이터 확인에 필수적입니다.
4. **Examples**를 통해 다양한 기능을 학습할 수 있습니다.

## 🚀 다음 단계

다음 레슨에서는 실제로 LED를 제어하는 첫 번째 프로그램을 작성하고,
코드의 구조와 기본 함수들을 자세히 배워보겠습니다.

## 💻 실습 과제

1. Arduino IDE 설치 후 Blink 예제 성공적으로 업로드하기
2. Blink 예제의 delay 값을 변경하여 깜빡이는 속도 조절해보기
3. 시리얼 모니터로 "Hello, Arduino!" 출력해보기
4. File → Examples 메뉴에서 다른 예제들 살펴보기

## 🔧 문제 해결 팁

### IDE가 시작되지 않을 때
```bash
# Windows
- 바이러스 백신 예외 추가
- 관리자 권한으로 실행
- C:\Users\[사용자명]\AppData\Local\Arduino15 폴더 삭제 후 재설치

# Linux
- ./arduino-ide_*.AppImage --no-sandbox
```

### 보드가 인식되지 않을 때
1. 다른 USB 포트 시도
2. 다른 USB 케이블 시도 (데이터 케이블인지 확인)
3. 장치 관리자에서 드라이버 업데이트
4. 보드의 리셋 버튼을 누른 상태에서 USB 연결

---

[← 이전 레슨: 아두이노 소개](../01-arduino-introduction/README.md) | [다음 레슨: LED 깜빡이기 →](../03-first-program-blink/README.md)