# Arduino 오픈소스 기여 완벽 가이드

## 목차
1. [오픈소스 기여의 가치](#오픈소스-기여의-가치)
2. [시작하기 전 준비사항](#시작하기-전-준비사항)
3. [첫 기여를 위한 프로젝트 찾기](#첫-기여를-위한-프로젝트-찾기)
4. [기여 프로세스 단계별 가이드](#기여-프로세스-단계별-가이드)
5. [Arduino 관련 주요 오픈소스 프로젝트](#arduino-관련-주요-오픈소스-프로젝트)
6. [기여 유형별 가이드](#기여-유형별-가이드)
7. [PR(Pull Request) 작성법](#pr-작성법)
8. [오픈소스 에티켓](#오픈소스-에티켓)
9. [나만의 오픈소스 프로젝트 시작하기](#나만의-오픈소스-프로젝트-시작하기)
10. [오픈소스 포트폴리오 구축](#오픈소스-포트폴리오-구축)

---

## 오픈소스 기여의 가치

### 개인적 성장
- **실무 경험**: 대규모 프로젝트 참여 경험
- **코드 리뷰**: 전문가들의 피드백
- **네트워킹**: 글로벌 개발자 커뮤니티
- **포트폴리오**: 공개된 기여 내역

### 기술적 성장
- **Best Practices**: 업계 표준 학습
- **협업 도구**: Git, CI/CD, 이슈 트래킹
- **코드 품질**: 높은 수준의 코드 작성
- **문제 해결**: 실제 사용자 문제 해결

### 커리어 발전
- **가시성**: 잠재적 고용주에게 노출
- **신뢰성**: 검증된 개발 능력
- **리더십**: 메인테이너로 성장 가능
- **기회**: 컨퍼런스 발표, 멘토링

---

## 시작하기 전 준비사항

### 1. 개발 환경 설정

#### Git 설정
```bash
# 사용자 정보 설정
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 유용한 별칭 설정
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.st status

# 에디터 설정
git config --global core.editor "code --wait"  # VS Code
```

#### SSH 키 설정
```bash
# SSH 키 생성
ssh-keygen -t ed25519 -C "your.email@example.com"

# SSH 에이전트 시작
eval "$(ssh-agent -s)"

# SSH 키 추가
ssh-add ~/.ssh/id_ed25519

# GitHub에 공개키 추가
cat ~/.ssh/id_ed25519.pub
# GitHub Settings > SSH and GPG keys에 추가
```

### 2. 필수 도구 설치

#### 개발 도구
```bash
# Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# PlatformIO
pip install platformio

# 코드 품질 도구
npm install -g cpplint
pip install pre-commit
```

#### 문서화 도구
```bash
# Doxygen (C++ 문서화)
sudo apt-get install doxygen graphviz

# Markdown 린터
npm install -g markdownlint-cli
```

### 3. 기본 스킬 체크리스트

- [ ] Git 브랜칭 전략 이해
- [ ] Pull Request 프로세스 이해
- [ ] 이슈 트래킹 시스템 사용법
- [ ] 코드 리뷰 기본 원칙
- [ ] 라이선스 기초 지식
- [ ] CI/CD 파이프라인 이해

---

## 첫 기여를 위한 프로젝트 찾기

### 1. 초보자 친화적 레이블

GitHub에서 다음 레이블들을 검색:
- `good first issue`
- `beginner friendly`
- `help wanted`
- `first-timers-only`
- `up for grabs`

### 2. 프로젝트 선택 기준

#### 활발한 프로젝트 확인
```python
# 프로젝트 활성도 체크 스크립트
import requests
from datetime import datetime, timedelta

def check_project_activity(owner, repo):
    # 최근 커밋 확인
    commits_url = f"https://api.github.com/repos/{owner}/{repo}/commits"
    commits = requests.get(commits_url).json()
    
    if commits:
        last_commit = datetime.fromisoformat(commits[0]['commit']['author']['date'].replace('Z', '+00:00'))
        days_since_commit = (datetime.now(timezone.utc) - last_commit).days
        
        if days_since_commit < 30:
            print(f"✅ 활발함: 마지막 커밋 {days_since_commit}일 전")
        else:
            print(f"⚠️ 비활발: 마지막 커밋 {days_since_commit}일 전")
    
    # 이슈 응답 시간 확인
    issues_url = f"https://api.github.com/repos/{owner}/{repo}/issues?state=closed"
    issues = requests.get(issues_url).json()
    
    response_times = []
    for issue in issues[:10]:  # 최근 10개 이슈
        created = datetime.fromisoformat(issue['created_at'].replace('Z', '+00:00'))
        if issue['closed_at']:
            closed = datetime.fromisoformat(issue['closed_at'].replace('Z', '+00:00'))
            response_times.append((closed - created).days)
    
    if response_times:
        avg_response = sum(response_times) / len(response_times)
        print(f"📊 평균 이슈 해결 시간: {avg_response:.1f}일")

# 사용 예
check_project_activity("arduino", "Arduino")
```

### 3. 추천 시작 프로젝트

#### 입문자용
1. **Arduino Documentation**
   - 문서 개선, 번역
   - 예제 코드 추가

2. **Arduino Libraries**
   - 버그 수정
   - 예제 추가
   - 문서 개선

3. **Community Projects**
   - 작은 규모
   - 친절한 멘토링

---

## 기여 프로세스 단계별 가이드

### 1. Fork와 Clone

```bash
# 1. GitHub에서 Fork 버튼 클릭

# 2. Fork한 저장소 clone
git clone git@github.com:YOUR_USERNAME/PROJECT_NAME.git
cd PROJECT_NAME

# 3. Upstream 저장소 추가
git remote add upstream git@github.com:ORIGINAL_OWNER/PROJECT_NAME.git

# 4. 확인
git remote -v
```

### 2. 브랜치 생성

```bash
# Upstream 최신 상태 가져오기
git fetch upstream
git checkout main
git merge upstream/main

# 기능 브랜치 생성
git checkout -b feature/add-sensor-support
# 또는 버그 수정
git checkout -b fix/memory-leak-issue-123
```

### 3. 개발 작업

#### 코드 작성 체크리스트
- [ ] 프로젝트 코딩 스타일 준수
- [ ] 테스트 작성
- [ ] 문서 업데이트
- [ ] 변경사항 최소화
- [ ] 커밋 메시지 규칙 준수

#### 예제: Arduino 라이브러리 개선
```cpp
// 개선 전: 하드코딩된 값
class Sensor {
    int pin = 2;  // 고정된 핀
public:
    void begin() {
        pinMode(pin, INPUT);
    }
};

// 개선 후: 유연한 설정
class Sensor {
private:
    uint8_t _pin;
    uint32_t _sampleRate;
    
public:
    Sensor(uint8_t pin = 2) : _pin(pin), _sampleRate(1000) {}
    
    void begin() {
        pinMode(_pin, INPUT);
    }
    
    void setSampleRate(uint32_t rate) {
        _sampleRate = rate;
    }
};
```

### 4. 테스트

#### 유닛 테스트 작성
```cpp
// test/SensorTest.cpp
#include <Arduino.h>
#include <unity.h>
#include "Sensor.h"

void test_sensor_initialization() {
    Sensor sensor(A0);
    sensor.begin();
    
    TEST_ASSERT_EQUAL(INPUT, getPinMode(A0));
}

void test_sample_rate_setting() {
    Sensor sensor;
    sensor.setSampleRate(500);
    
    TEST_ASSERT_EQUAL(500, sensor.getSampleRate());
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_sensor_initialization);
    RUN_TEST(test_sample_rate_setting);
    UNITY_END();
}

void loop() {}
```

#### 통합 테스트
```bash
# PlatformIO 테스트 실행
pio test

# Arduino CLI 테스트
arduino-cli compile --fqbn arduino:avr:uno
arduino-cli upload --port COM3 --fqbn arduino:avr:uno
```

### 5. 커밋

#### 좋은 커밋 메시지 작성
```bash
# 형식: <type>(<scope>): <subject>
# 
# <body>
# 
# <footer>

# 예제
git commit -m "feat(sensor): Add configurable sample rate

- Added setSampleRate() method to allow dynamic configuration
- Sample rate can now be adjusted from 1Hz to 1000Hz
- Default rate remains 1000Hz for backward compatibility

Fixes #123"
```

#### 커밋 타입
- `feat`: 새로운 기능
- `fix`: 버그 수정
- `docs`: 문서 수정
- `style`: 코드 스타일 변경
- `refactor`: 리팩토링
- `test`: 테스트 추가/수정
- `chore`: 빌드, 패키지 매니저 등

### 6. Push와 PR 생성

```bash
# 변경사항 push
git push origin feature/add-sensor-support

# GitHub에서 Pull Request 생성
```

---

## Arduino 관련 주요 오픈소스 프로젝트

### 1. 공식 Arduino 프로젝트

#### Arduino Core
- **저장소**: https://github.com/arduino/Arduino
- **언어**: C++, Java
- **기여 영역**: 코어 기능, IDE 개선, 버그 수정

#### Arduino CLI
- **저장소**: https://github.com/arduino/arduino-cli
- **언어**: Go
- **기여 영역**: CLI 기능, 자동화 도구

#### Arduino Libraries
- **저장소**: https://github.com/arduino-libraries
- **언어**: C++
- **기여 영역**: 라이브러리 개선, 예제 추가

### 2. 인기 서드파티 프로젝트

#### PlatformIO
```cpp
// platformio.ini 예제 기여
[env:arduino_uno]
platform = atmelavr
board = uno
framework = arduino
lib_deps = 
    Wire
    SPI
    adafruit/Adafruit Sensor@^1.1.4

; 커스텀 빌드 플래그
build_flags = 
    -D VERSION=1.2.3
    -D DEBUG=1

; 업로드 설정
upload_port = COM3
upload_speed = 115200
```

#### ESPHome
```yaml
# ESPHome 설정 예제 기여
esphome:
  name: smart-sensor
  platform: ESP32
  board: esp32dev

wifi:
  ssid: "MyNetwork"
  password: "MyPassword"

sensor:
  - platform: dht
    pin: GPIO23
    temperature:
      name: "Room Temperature"
    humidity:
      name: "Room Humidity"
    update_interval: 60s

# 커스텀 컴포넌트 추가
custom_component:
- lambda: |-
    auto my_sensor = new MyCustomSensor();
    return {my_sensor};
```

#### FastLED
```cpp
// FastLED 효과 기여 예제
class FireEffect : public LEDEffect {
private:
    CRGBPalette16 heatPalette;
    uint8_t heat[NUM_LEDS];
    
public:
    FireEffect() {
        heatPalette = CRGBPalette16(
            CRGB::Black, CRGB::Red, CRGB::Orange, CRGB::Yellow
        );
    }
    
    void update() override {
        // 열 시뮬레이션
        for (int i = 0; i < NUM_LEDS; i++) {
            heat[i] = qsub8(heat[i], random8(0, ((55 * 10) / NUM_LEDS) + 2));
        }
        
        // 열 상승
        for (int k = NUM_LEDS - 1; k >= 2; k--) {
            heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
        }
        
        // 새로운 불꽃
        if (random8() < 120) {
            int y = random8(7);
            heat[y] = qadd8(heat[y], random8(160, 255));
        }
        
        // 열을 색상으로 변환
        for (int j = 0; j < NUM_LEDS; j++) {
            leds[j] = ColorFromPalette(heatPalette, heat[j]);
        }
    }
};
```

### 3. 하드웨어 관련 프로젝트

#### Marlin (3D 프린터 펌웨어)
```cpp
// Marlin 기능 추가 예제
class AutoBedLeveling {
private:
    float z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
    
public:
    void probe_bed() {
        for (int x = 0; x < GRID_MAX_POINTS_X; x++) {
            for (int y = 0; y < GRID_MAX_POINTS_Y; y++) {
                // 프로브 위치로 이동
                do_blocking_move_to_xy(
                    MESH_MIN_X + x * MESH_X_DIST,
                    MESH_MIN_Y + y * MESH_Y_DIST
                );
                
                // Z 높이 측정
                z_values[x][y] = probe_at_point();
                
                // 사용자 피드백
                SERIAL_ECHOLNPAIR("Point ", x, ",", y, " Z=", z_values[x][y]);
            }
        }
        
        // 메시 보정 적용
        apply_leveling_correction();
    }
};
```

#### GRBL (CNC 컨트롤러)
```cpp
// GRBL 개선 예제
class SpindleControl {
private:
    uint16_t current_rpm;
    uint16_t target_rpm;
    
public:
    void update() {
        // 부드러운 가속/감속
        if (current_rpm < target_rpm) {
            current_rpm += min(SPINDLE_ACCEL_RATE, target_rpm - current_rpm);
        } else if (current_rpm > target_rpm) {
            current_rpm -= min(SPINDLE_DECEL_RATE, current_rpm - target_rpm);
        }
        
        // PWM 출력
        uint8_t pwm_value = map(current_rpm, 0, SPINDLE_MAX_RPM, 0, 255);
        analogWrite(SPINDLE_PWM_PIN, pwm_value);
        
        // 안전 체크
        if (current_rpm > 0 && !coolant_is_on()) {
            enable_coolant();
        }
    }
};
```

---

## 기여 유형별 가이드

### 1. 버그 수정

#### 버그 찾기
```cpp
// 메모리 누수 예제
class DataLogger {
    char* buffer;  // 문제: 소멸자 없음
    
public:
    DataLogger() {
        buffer = new char[1024];
    }
    
    // 수정: 소멸자 추가
    ~DataLogger() {
        delete[] buffer;
    }
};
```

#### 버그 리포트 작성
```markdown
## Bug Report

### Description
메모리 누수가 DataLogger 클래스에서 발생합니다.

### Steps to Reproduce
1. DataLogger 인스턴스 생성
2. 함수 종료 시 메모리 해제 안됨
3. 반복 시 메모리 부족

### Expected Behavior
객체 소멸 시 할당된 메모리가 해제되어야 함

### Actual Behavior
메모리가 해제되지 않아 누수 발생

### Environment
- Arduino: Uno
- IDE: 1.8.19
- Library: DataLogger v1.2.3

### Proposed Fix
```cpp
~DataLogger() {
    delete[] buffer;
}
```
```

### 2. 기능 추가

#### 제안서 작성
```markdown
## Feature Proposal: Add Timeout to Sensor Reading

### Motivation
센서 응답이 없을 때 무한 대기하는 문제 해결

### Proposed API
```cpp
class Sensor {
public:
    // 기존 API (호환성 유지)
    int read();
    
    // 새로운 API
    int read(uint32_t timeout_ms);
    bool readWithTimeout(int& value, uint32_t timeout_ms);
};
```

### Implementation Plan
1. 타임아웃 파라미터 추가
2. millis() 기반 시간 체크
3. 타임아웃 시 에러 코드 반환

### Backward Compatibility
기존 read() 함수는 유지하여 호환성 보장
```

#### 구현 예제
```cpp
bool Sensor::readWithTimeout(int& value, uint32_t timeout_ms) {
    uint32_t start_time = millis();
    
    while (!dataAvailable()) {
        if (millis() - start_time >= timeout_ms) {
            return false;  // 타임아웃
        }
        yield();  // ESP8266/ESP32 호환성
    }
    
    value = readData();
    return true;
}
```

### 3. 문서화

#### README 개선
```markdown
# Awesome Sensor Library

![Build Status](https://img.shields.io/github/workflow/status/user/repo/CI)
![License](https://img.shields.io/github/license/user/repo)
![Version](https://img.shields.io/github/v/release/user/repo)

## Features
- 🚀 High-speed data acquisition
- 🔧 Easy configuration
- 📊 Built-in data filtering
- 🔌 Multiple sensor support

## Quick Start

### Installation

#### Arduino IDE
1. Open Library Manager
2. Search for "Awesome Sensor"
3. Click Install

#### PlatformIO
```ini
lib_deps = 
    awesome-sensor
```

### Basic Usage
```cpp
#include <AwesomeSensor.h>

AwesomeSensor sensor(A0);

void setup() {
    Serial.begin(115200);
    sensor.begin();
    sensor.setFilterLevel(MEDIUM);
}

void loop() {
    float value = sensor.read();
    Serial.println(value);
    delay(100);
}
```

## API Reference

### Constructor
```cpp
AwesomeSensor(uint8_t pin, uint8_t type = TYPE_ANALOG)
```

### Methods
| Method | Description | Return |
|--------|-------------|--------|
| `begin()` | Initialize sensor | void |
| `read()` | Read sensor value | float |
| `setFilterLevel(level)` | Set filter strength | void |

## Examples

### Temperature Monitoring
```cpp
// examples/TemperatureMonitor/TemperatureMonitor.ino
```

### Data Logging
```cpp
// examples/DataLogger/DataLogger.ino
```

## Contributing
See [CONTRIBUTING.md](CONTRIBUTING.md)

## License
MIT License - see [LICENSE](LICENSE)
```

#### API 문서 생성
```cpp
/**
 * @file AwesomeSensor.h
 * @brief High-performance sensor library for Arduino
 * @author Your Name
 * @version 1.0.0
 * @date 2024-01-01
 */

#ifndef AWESOME_SENSOR_H
#define AWESOME_SENSOR_H

/**
 * @class AwesomeSensor
 * @brief Main sensor interface class
 * 
 * This class provides a unified interface for various sensor types
 * with built-in filtering and calibration capabilities.
 * 
 * @code
 * AwesomeSensor sensor(A0);
 * sensor.begin();
 * float value = sensor.read();
 * @endcode
 */
class AwesomeSensor {
public:
    /**
     * @brief Sensor types enumeration
     */
    enum SensorType {
        TYPE_ANALOG,    ///< Analog voltage sensor
        TYPE_DIGITAL,   ///< Digital sensor
        TYPE_I2C,       ///< I2C sensor
        TYPE_SPI        ///< SPI sensor
    };
    
    /**
     * @brief Filter levels
     */
    enum FilterLevel {
        NONE,    ///< No filtering
        LOW,     ///< Light filtering
        MEDIUM,  ///< Medium filtering
        HIGH     ///< Heavy filtering
    };
    
    /**
     * @brief Construct a new sensor object
     * 
     * @param pin Sensor pin number
     * @param type Sensor type (default: TYPE_ANALOG)
     */
    AwesomeSensor(uint8_t pin, SensorType type = TYPE_ANALOG);
    
    /**
     * @brief Initialize the sensor
     * 
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool begin();
    
    /**
     * @brief Read sensor value
     * 
     * @return float Sensor reading
     * @throws SensorException if read fails
     */
    float read();
    
    /**
     * @brief Set filter level
     * 
     * @param level Filter strength
     */
    void setFilterLevel(FilterLevel level);
};

#endif // AWESOME_SENSOR_H
```

### 4. 테스트 추가

#### 유닛 테스트
```cpp
// test/test_sensor/test_sensor.cpp
#include <Arduino.h>
#include <unity.h>
#include "AwesomeSensor.h"

// 모의 하드웨어
void setUp() {
    // 테스트 환경 설정
}

void tearDown() {
    // 테스트 환경 정리
}

void test_sensor_initialization() {
    AwesomeSensor sensor(A0);
    TEST_ASSERT_TRUE(sensor.begin());
}

void test_sensor_read_range() {
    AwesomeSensor sensor(A0);
    sensor.begin();
    
    float value = sensor.read();
    TEST_ASSERT_FLOAT_WITHIN(1023.0, value, 0.0);
}

void test_filter_levels() {
    AwesomeSensor sensor(A0);
    sensor.begin();
    
    // 각 필터 레벨 테스트
    for (int level = 0; level <= 3; level++) {
        sensor.setFilterLevel(static_cast<AwesomeSensor::FilterLevel>(level));
        float value = sensor.read();
        TEST_ASSERT_GREATER_OR_EQUAL(0.0, value);
    }
}

void test_error_handling() {
    AwesomeSensor sensor(255);  // 잘못된 핀
    TEST_ASSERT_FALSE(sensor.begin());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    RUN_TEST(test_sensor_initialization);
    RUN_TEST(test_sensor_read_range);
    RUN_TEST(test_filter_levels);
    RUN_TEST(test_error_handling);
    
    return UNITY_END();
}
```

#### 통합 테스트
```yaml
# .github/workflows/test.yml
name: CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        board: [uno, mega, esp32, esp8266]
    
    steps:
    - uses: actions/checkout@v2
    
    - name: Cache PlatformIO
      uses: actions/cache@v2
      with:
        path: ~/.platformio
        key: ${{ runner.os }}-pio
    
    - name: Install PlatformIO
      run: |
        python -m pip install --upgrade pip
        pip install platformio
    
    - name: Run tests
      run: |
        pio test -e ${{ matrix.board }}
    
    - name: Build examples
      run: |
        for example in examples/*/; do
          pio ci --board=${{ matrix.board }} $example/*.ino
        done
```

---

## PR 작성법

### 1. PR 템플릿

```markdown
## Description
Brief description of what this PR does.

## Type of Change
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update

## How Has This Been Tested?
- [ ] Test A: Description
- [ ] Test B: Description

## Checklist
- [ ] My code follows the style guidelines of this project
- [ ] I have performed a self-review of my own code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to the documentation
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix is effective or that my feature works
- [ ] New and existing unit tests pass locally with my changes

## Screenshots (if appropriate)
```

### 2. 좋은 PR 예제

```markdown
## Add timeout support to sensor reading

### Description
This PR adds timeout functionality to prevent infinite blocking when sensors don't respond.

### Motivation
Fixes #123 - Application hangs when sensor disconnected

### Changes
- Added `readWithTimeout()` method
- Maintained backward compatibility with existing `read()` method
- Added comprehensive unit tests
- Updated documentation and examples

### Test Results
```
Running tests...
test/test_sensor/test_timeout.cpp:25:test_read_with_timeout	[PASSED]
test/test_sensor/test_timeout.cpp:35:test_timeout_expiry	[PASSED]
test/test_sensor/test_timeout.cpp:45:test_backward_compatibility	[PASSED]

==================== SUMMARY ====================
3 Tests 0 Failures 0 Ignored
OK
```

### Performance Impact
- No impact on existing `read()` method
- Timeout checking adds ~2μs overhead

### Example Usage
```cpp
int value;
if (sensor.readWithTimeout(value, 1000)) {
    Serial.println(value);
} else {
    Serial.println("Sensor timeout!");
}
```
```

---

## 오픈소스 에티켓

### 1. 커뮤니케이션

#### 이슈 작성
```markdown
<!-- 좋은 예 -->
## Issue: Sensor readings incorrect at high temperatures

### Environment
- Board: Arduino Uno
- Sensor: DHT22
- Library version: 1.4.0
- Temperature: >40°C

### Problem
Sensor returns -999 when ambient temperature exceeds 40°C.

### Expected Behavior
Should return actual temperature up to 80°C as per DHT22 specs.

### Code to Reproduce
```cpp
DHT dht(2, DHT22);
void setup() {
    Serial.begin(9600);
    dht.begin();
}
void loop() {
    float t = dht.readTemperature();
    Serial.println(t);  // Returns -999 at >40°C
    delay(2000);
}
```

### Possible Cause
Timing issue in data reading at high temperatures?
```

#### 코드 리뷰 피드백
```cpp
// 건설적인 피드백 예
// "이 부분은 메모리 누수가 발생할 수 있습니다. 
// delete[]를 사용하여 메모리를 해제하는 것이 좋겠습니다."

char* buffer = new char[size];
// ... 사용
// delete[] buffer; // 추가 필요

// 긍정적 피드백도 중요
// "효율적인 알고리즘 선택이네요! 👍"
```

### 2. 라이선스 이해

#### 주요 오픈소스 라이선스
| 라이선스 | 상업적 사용 | 수정 | 배포 | 특허 보호 | 소스 공개 의무 |
|---------|------------|------|------|----------|---------------|
| MIT | ✅ | ✅ | ✅ | ❌ | ❌ |
| Apache 2.0 | ✅ | ✅ | ✅ | ✅ | ❌ |
| GPL v3 | ✅ | ✅ | ✅ | ✅ | ✅ |
| LGPL v3 | ✅ | ✅ | ✅ | ✅ | 부분적 |

### 3. 기여자 행동 강령

#### 존중과 포용
- 다양한 배경의 기여자 존중
- 건설적인 비판과 피드백
- 인내심을 가지고 설명

#### 전문성
- 시간 약속 지키기
- 품질 높은 코드 제출
- 문서화 중요성 인식

---

## 나만의 오픈소스 프로젝트 시작하기

### 1. 프로젝트 구조

```
my-arduino-library/
├── src/
│   ├── MyLibrary.h
│   └── MyLibrary.cpp
├── examples/
│   ├── Basic/
│   │   └── Basic.ino
│   └── Advanced/
│       └── Advanced.ino
├── test/
│   └── test_main.cpp
├── docs/
│   ├── API.md
│   └── TUTORIAL.md
├── .github/
│   ├── workflows/
│   │   └── ci.yml
│   └── ISSUE_TEMPLATE/
│       ├── bug_report.md
│       └── feature_request.md
├── library.properties
├── library.json
├── platformio.ini
├── LICENSE
├── README.md
└── CONTRIBUTING.md
```

### 2. 필수 파일

#### library.properties
```properties
name=MyAwesomeLibrary
version=1.0.0
author=Your Name <your.email@example.com>
maintainer=Your Name <your.email@example.com>
sentence=A brief description of the library.
paragraph=A more detailed description of the library.
category=Sensors
url=https://github.com/yourusername/MyAwesomeLibrary
architectures=*
includes=MyAwesomeLibrary.h
```

#### library.json (PlatformIO)
```json
{
  "name": "MyAwesomeLibrary",
  "version": "1.0.0",
  "description": "A brief description of the library",
  "keywords": "arduino, sensor, awesome",
  "repository": {
    "type": "git",
    "url": "https://github.com/yourusername/MyAwesomeLibrary.git"
  },
  "authors": [
    {
      "name": "Your Name",
      "email": "your.email@example.com",
      "maintainer": true
    }
  ],
  "license": "MIT",
  "homepage": "https://github.com/yourusername/MyAwesomeLibrary",
  "dependencies": {
    "Wire": "~1.0"
  },
  "frameworks": "arduino",
  "platforms": "*"
}
```

### 3. CI/CD 설정

#### GitHub Actions
```yaml
# .github/workflows/release.yml
name: Release

on:
  push:
    tags:
      - 'v*'

jobs:
  release:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v2
    
    - name: Create Release
      uses: actions/create-release@v1
      env:
        GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
      with:
        tag_name: ${{ github.ref }}
        release_name: Release ${{ github.ref }}
        draft: false
        prerelease: false
    
    - name: Publish to Arduino Library Manager
      run: |
        # Arduino library submission process
        echo "Submit at https://github.com/arduino/library-registry"
    
    - name: Publish to PlatformIO
      run: |
        pip install platformio
        pio package publish
```

### 4. 홍보와 커뮤니티 구축

#### README 배지
```markdown
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)
![PlatformIO](https://img.shields.io/badge/PlatformIO-FF7F00?style=for-the-badge&logo=PlatformIO&logoColor=white)
![License](https://img.shields.io/github/license/yourusername/repo?style=for-the-badge)
![Downloads](https://img.shields.io/github/downloads/yourusername/repo/total?style=for-the-badge)
![Stars](https://img.shields.io/github/stars/yourusername/repo?style=for-the-badge)
```

#### 커뮤니티 채널
- Discord/Slack 채널 생성
- 포럼 스레드 시작
- 소셜 미디어 활용
- 블로그 포스트 작성

---

## 오픈소스 포트폴리오 구축

### 1. GitHub 프로필 최적화

#### README.md
```markdown
### Hi there 👋 I'm [Your Name]

🔧 **Embedded Systems Developer** specializing in Arduino and IoT

#### 🚀 Featured Projects

| Project | Description | Stars | Language |
|---------|-------------|-------|----------|
| [SensorLib](link) | High-performance sensor library | ⭐ 245 | C++ |
| [IoT-Framework](link) | Complete IoT solution | ⭐ 189 | C++/Python |
| [Motor-Control](link) | Advanced motor control | ⭐ 156 | C++ |

#### 📊 GitHub Stats

![Your GitHub stats](https://github-readme-stats.vercel.app/api?username=yourusername&show_icons=true&theme=radical)

#### 🛠️ Tech Stack

![Arduino](https://img.shields.io/badge/-Arduino-00979D?style=flat&logo=Arduino&logoColor=white)
![C++](https://img.shields.io/badge/-C++-00599C?style=flat&logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/-Python-3776AB?style=flat&logo=Python&logoColor=white)

#### 📫 How to reach me

- 📧 Email: your.email@example.com
- 💼 LinkedIn: [yourprofile](link)
- 🐦 Twitter: [@yourhandle](link)
```

### 2. 기여 통계 시각화

```python
# 기여 통계 생성 스크립트
import requests
import matplotlib.pyplot as plt
from datetime import datetime, timedelta

def visualize_contributions(username):
    # GitHub API로 기여 데이터 가져오기
    contributions = get_github_contributions(username)
    
    # 시각화
    plt.figure(figsize=(12, 6))
    
    # 월별 기여도
    plt.subplot(1, 2, 1)
    plot_monthly_contributions(contributions)
    
    # 언어별 분포
    plt.subplot(1, 2, 2)
    plot_language_distribution(contributions)
    
    plt.tight_layout()
    plt.savefig('contributions.png')

def generate_contribution_report(username):
    """기여 리포트 생성"""
    report = f"""
    # Open Source Contribution Report - {username}
    
    ## Summary
    - Total Contributions: {total_contributions}
    - Projects Contributed: {project_count}
    - Languages Used: {languages}
    
    ## Major Contributions
    {format_major_contributions()}
    
    ## Impact Metrics
    - Code Added: {lines_added}
    - Code Removed: {lines_removed}
    - Issues Resolved: {issues_closed}
    - PRs Merged: {prs_merged}
    """
    
    return report
```

### 3. 포트폴리오 웹사이트

```html
<!-- portfolio.html -->
<!DOCTYPE html>
<html>
<head>
    <title>Arduino Developer Portfolio</title>
    <style>
        .project-card {
            border: 1px solid #ddd;
            padding: 20px;
            margin: 10px;
            border-radius: 8px;
        }
        .tech-tag {
            display: inline-block;
            padding: 5px 10px;
            margin: 2px;
            background: #00979D;
            color: white;
            border-radius: 15px;
            font-size: 12px;
        }
    </style>
</head>
<body>
    <header>
        <h1>Arduino & IoT Developer</h1>
        <p>Open Source Contributor | Embedded Systems Specialist</p>
    </header>
    
    <section id="projects">
        <h2>Open Source Projects</h2>
        
        <div class="project-card">
            <h3>🚀 Advanced Sensor Library</h3>
            <p>High-performance sensor interface library with 500+ stars</p>
            <div class="tech-tags">
                <span class="tech-tag">C++</span>
                <span class="tech-tag">Arduino</span>
                <span class="tech-tag">PlatformIO</span>
            </div>
            <div class="metrics">
                <span>⭐ 523</span>
                <span>🍴 89</span>
                <span>👁️ 1.2k</span>
            </div>
        </div>
    </section>
    
    <section id="contributions">
        <h2>Contribution Graph</h2>
        <canvas id="contributionChart"></canvas>
    </section>
    
    <script>
        // 기여도 차트 렌더링
        renderContributionChart();
    </script>
</body>
</html>
```

### 4. 인증과 뱃지

#### 획득 가능한 뱃지
- **Arctic Code Vault Contributor**: 2020 GitHub Archive Program
- **Pull Shark**: 2개 이상의 병합된 PR
- **Galaxy Brain**: 토론에서 답변 채택
- **Starstruck**: 프로젝트 16개 이상 스타

#### 커스텀 뱃지 생성
```markdown
![Custom Badge](https://img.shields.io/badge/Arduino_Expert-Level_5-brightgreen?style=for-the-badge&logo=arduino)
![Contributions](https://img.shields.io/badge/Contributions-1000+-blue?style=for-the-badge)
![Projects](https://img.shields.io/badge/Projects-25+-orange?style=for-the-badge)
```

---

## 마무리

오픈소스 기여는 단순히 코드를 작성하는 것 이상의 의미를 가집니다. 전 세계 개발자들과 협력하여 더 나은 소프트웨어를 만들고, 지식을 공유하며, 함께 성장하는 과정입니다.

### 시작하기 위한 체크리스트
- [ ] GitHub 계정 설정 완료
- [ ] 첫 번째 프로젝트 선택
- [ ] 개발 환경 구축
- [ ] 첫 이슈 또는 PR 작성
- [ ] 커뮤니티 참여

### 장기적 목표
1. **단기 (3개월)**: 5개 이상의 PR 머지
2. **중기 (6개월)**: 자신의 라이브러리 공개
3. **장기 (1년)**: 주요 프로젝트 메인테이너

오픈소스는 여러분의 기술적 성장과 커리어 발전에 큰 도움이 될 것입니다. 작은 기여부터 시작하여 점차 영향력을 확대해 나가세요!

**Happy Contributing! 🎉**