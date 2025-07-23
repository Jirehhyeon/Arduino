# 🔧 아두이노 트러블슈팅 완전 가이드

아두이노 프로젝트에서 자주 발생하는 문제들과 해결 방법을 정리한 실무 중심 가이드입니다.

---

## 📋 목차

1. [하드웨어 문제 해결](#1-하드웨어-문제-해결)
2. [소프트웨어 문제 해결](#2-소프트웨어-문제-해결)
3. [통신 문제 해결](#3-통신-문제-해결)
4. [전력 관련 문제](#4-전력-관련-문제)
5. [센서 문제 진단](#5-센서-문제-진단)
6. [성능 최적화](#6-성능-최적화)
7. [디버깅 도구 활용](#7-디버깅-도구-활용)

---

# 1. 하드웨어 문제 해결

## 🔌 연결 및 배선 문제

### 문제 1: 아두이노가 PC에서 인식되지 않음

**증상:**
- 장치 관리자에서 아두이노가 보이지 않음
- IDE에서 포트가 표시되지 않음
- COM 포트 오류

**진단 및 해결:**

```cpp
// 하드웨어 진단 코드
void diagnosticHardware() {
    Serial.begin(9600);
    
    // 1. 기본 동작 확인
    Serial.println("=== Hardware Diagnostic ===");
    Serial.print("Arduino Type: ");
    #ifdef ARDUINO_AVR_UNO
        Serial.println("Arduino Uno");
    #elif defined(ARDUINO_ESP32_DEV)
        Serial.println("ESP32");
    #elif defined(ARDUINO_AVR_NANO)
        Serial.println("Arduino Nano");
    #endif
    
    // 2. 전원 상태 확인
    Serial.print("Supply Voltage: ");
    Serial.print(readVcc());
    Serial.println(" mV");
    
    // 3. 핀 상태 테스트
    testAllPins();
    
    // 4. 메모리 상태
    Serial.print("Free SRAM: ");
    Serial.print(freeRam());
    Serial.println(" bytes");
}

// VCC 전압 측정 (Arduino Uno/Nano)
long readVcc() {
    long result;
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    result = ADCL;
    result |= ADCH << 8;
    result = 1125300L / result;
    return result;
}

// 사용 가능한 SRAM 확인
int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// 모든 핀 상태 테스트
void testAllPins() {
    Serial.println("\n--- Pin Status Test ---");
    
    for (int pin = 2; pin <= 13; pin++) {
        pinMode(pin, INPUT_PULLUP);
        delay(10);
        int state = digitalRead(pin);
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.print(": ");
        Serial.println(state ? "HIGH" : "LOW");
    }
    
    // 아날로그 핀 테스트
    Serial.println("\n--- Analog Pin Test ---");
    for (int pin = A0; pin <= A5; pin++) {
        int value = analogRead(pin);
        Serial.print("A");
        Serial.print(pin - A0);
        Serial.print(": ");
        Serial.println(value);
    }
}
```

**해결 방법:**

1. **드라이버 문제:**
   ```bash
   # Windows
   - CH340 드라이버 설치 (중국산 아두이노)
   - FTDI 드라이버 설치 (정품 아두이노)
   
   # macOS
   brew install --cask silicon-labs-vcp-driver
   
   # Linux
   sudo usermod -a -G dialout $USER
   sudo udevadm control --reload-rules
   ```

2. **USB 케이블 문제:**
   - 데이터 통신 지원 케이블 사용 확인
   - 다른 USB 포트 시도
   - 케이블 교체

3. **부트로더 문제:**
   ```cpp
   // ISP 프로그래머로 부트로더 재설치
   // Arduino IDE: Tools > Burn Bootloader
   ```

### 문제 2: 핀이 제대로 동작하지 않음

**증상:**
- 디지털 핀이 항상 HIGH 또는 LOW
- 아날로그 읽기값이 이상함
- PWM이 동작하지 않음

**진단 코드:**

```cpp
class PinDiagnostic {
private:
    struct PinInfo {
        int pinNumber;
        String pinType;
        bool isWorking;
        String errorMsg;
    };
    
    PinInfo pinStatus[20];
    int pinCount = 0;
    
public:
    void runCompleteDiagnostic() {
        Serial.println("=== Complete Pin Diagnostic ===");
        
        testDigitalPins();
        testAnalogPins();
        testPWMPins();
        testInterruptPins();
        
        generateReport();
    }
    
private:
    void testDigitalPins() {
        Serial.println("\n1. Digital Pin Test");
        
        for (int pin = 2; pin <= 13; pin++) {
            bool passed = testDigitalPin(pin);
            recordPinStatus(pin, "DIGITAL", passed, 
                          passed ? "OK" : "FAILED");
        }
    }
    
    bool testDigitalPin(int pin) {
        // 출력 테스트
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delay(10);
        
        // 입력 테스트로 확인
        pinMode(pin, INPUT);
        bool highTest = digitalRead(pin);
        
        // LOW 테스트
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        delay(10);
        
        pinMode(pin, INPUT);
        bool lowTest = !digitalRead(pin);
        
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.print(": HIGH=");
        Serial.print(highTest ? "OK" : "FAIL");
        Serial.print(", LOW=");
        Serial.println(lowTest ? "OK" : "FAIL");
        
        return highTest && lowTest;
    }
    
    void testAnalogPins() {
        Serial.println("\n2. Analog Pin Test");
        
        for (int pin = A0; pin <= A5; pin++) {
            bool passed = testAnalogPin(pin);
            recordPinStatus(pin, "ANALOG", passed,
                          passed ? "OK" : "NOISY/STUCK");
        }
    }
    
    bool testAnalogPin(int pin) {
        // 여러 번 읽어서 노이즈 체크
        int readings[10];
        for (int i = 0; i < 10; i++) {
            readings[i] = analogRead(pin);
            delay(10);
        }
        
        // 표준편차 계산
        float mean = 0;
        for (int i = 0; i < 10; i++) {
            mean += readings[i];
        }
        mean /= 10;
        
        float variance = 0;
        for (int i = 0; i < 10; i++) {
            variance += pow(readings[i] - mean, 2);
        }
        float stdDev = sqrt(variance / 10);
        
        Serial.print("A");
        Serial.print(pin - A0);
        Serial.print(": Mean=");
        Serial.print(mean);
        Serial.print(", StdDev=");
        Serial.println(stdDev);
        
        // 노이즈가 너무 크면 문제
        return stdDev < 10;
    }
    
    void testPWMPins() {
        Serial.println("\n3. PWM Pin Test");
        
        int pwmPins[] = {3, 5, 6, 9, 10, 11};
        
        for (int i = 0; i < 6; i++) {
            bool passed = testPWMPin(pwmPins[i]);
            recordPinStatus(pwmPins[i], "PWM", passed,
                          passed ? "OK" : "PWM_FAILED");
        }
    }
    
    bool testPWMPin(int pin) {
        // PWM 출력 테스트
        analogWrite(pin, 128); // 50% duty cycle
        delay(100);
        
        // 다른 핀으로 측정 (간접 테스트)
        // 실제로는 오실로스코프나 멀티미터 필요
        
        Serial.print("PWM Pin ");
        Serial.print(pin);
        Serial.println(": Testing...");
        
        // 간단한 검증 (실제 구현에서는 더 정교한 테스트 필요)
        return true;
    }
    
    void generateReport() {
        Serial.println("\n=== Diagnostic Report ===");
        
        int workingPins = 0;
        for (int i = 0; i < pinCount; i++) {
            Serial.print("Pin ");
            Serial.print(pinStatus[i].pinNumber);
            Serial.print(" (");
            Serial.print(pinStatus[i].pinType);
            Serial.print("): ");
            Serial.println(pinStatus[i].errorMsg);
            
            if (pinStatus[i].isWorking) workingPins++;
        }
        
        Serial.print("\nWorking pins: ");
        Serial.print(workingPins);
        Serial.print("/");
        Serial.println(pinCount);
        
        float successRate = (float)workingPins / pinCount * 100;
        Serial.print("Success rate: ");
        Serial.print(successRate);
        Serial.println("%");
    }
    
    void recordPinStatus(int pin, String type, bool working, String msg) {
        if (pinCount < 20) {
            pinStatus[pinCount].pinNumber = pin;
            pinStatus[pinCount].pinType = type;
            pinStatus[pinCount].isWorking = working;
            pinStatus[pinCount].errorMsg = msg;
            pinCount++;
        }
    }
};
```

### 문제 3: 브레드보드 연결 문제

**일반적인 실수들:**

```cpp
// 브레드보드 연결 검증 코드
class BreadboardTester {
public:
    void testConnections() {
        Serial.println("=== Breadboard Connection Test ===");
        
        // 전원 레일 테스트
        testPowerRails();
        
        // 연결 무결성 테스트
        testConnectionIntegrity();
        
        // 접촉 불량 감지
        detectLooseConnections();
    }
    
private:
    void testPowerRails() {
        Serial.println("\n1. Power Rail Test");
        
        // 5V 레일 테스트 (디지털 핀으로)
        pinMode(13, OUTPUT);
        digitalWrite(13, HIGH);
        delay(100);
        
        // 3.3V 레일 테스트
        float voltage3v3 = analogRead(A0) * (5.0 / 1023.0);
        Serial.print("Power rail voltage: ");
        Serial.println(voltage3v3);
        
        if (voltage3v3 < 4.8 || voltage3v3 > 5.2) {
            Serial.println("WARNING: Power rail voltage abnormal");
        }
    }
    
    void testConnectionIntegrity() {
        Serial.println("\n2. Connection Integrity Test");
        
        // 저항을 통한 연결 테스트
        for (int testPin = 2; testPin <= 7; testPin++) {
            if (testConnectionWithResistor(testPin)) {
                Serial.print("Pin ");
                Serial.print(testPin);
                Serial.println(": Connection OK");
            } else {
                Serial.print("Pin ");
                Serial.print(testPin);
                Serial.println(": Connection FAILED");
            }
        }
    }
    
    bool testConnectionWithResistor(int pin) {
        // 풀업 저항 테스트
        pinMode(pin, INPUT_PULLUP);
        delay(10);
        bool pullupTest = digitalRead(pin);
        
        // 풀다운 테스트
        pinMode(pin, INPUT);
        delay(10);
        bool pulldownTest = !digitalRead(pin);
        
        return pullupTest && pulldownTest;
    }
    
    void detectLooseConnections() {
        Serial.println("\n3. Loose Connection Detection");
        
        // 여러 번 읽어서 일관성 확인
        for (int pin = A0; pin <= A3; pin++) {
            int readings[5];
            bool stable = true;
            
            for (int i = 0; i < 5; i++) {
                readings[i] = analogRead(pin);
                delay(50);
            }
            
            // 편차 확인
            for (int i = 1; i < 5; i++) {
                if (abs(readings[i] - readings[0]) > 20) {
                    stable = false;
                    break;
                }
            }
            
            Serial.print("A");
            Serial.print(pin - A0);
            Serial.print(": ");
            Serial.println(stable ? "Stable" : "Unstable (loose connection?)");
        }
    }
};
```

**해결 방법:**
1. **점퍼 와이어 확인:**
   - 손상된 와이어 교체
   - 연결부 청소
   - 브레드보드 교체

2. **브레드보드 품질:**
   - 고품질 브레드보드 사용
   - 접촉점 청소
   - 오래된 브레드보드 교체

---

# 2. 소프트웨어 문제 해결

## 💻 컴파일 및 업로드 오류

### 문제 1: 컴파일 오류

**일반적인 오류와 해결:**

```cpp
// 일반적인 C++ 오류 예방 코드 패턴

class CommonErrorPrevention {
public:
    // 1. 변수 초기화 누락 방지
    void properVariableInit() {
        // BAD
        // int sensor_value;
        // float temperature;
        
        // GOOD
        int sensor_value = 0;
        float temperature = 0.0;
        bool is_initialized = false;
        
        // 배열 초기화
        int readings[10] = {0}; // 모든 요소를 0으로 초기화
    }
    
    // 2. 배열 경계 검사
    void safeArrayAccess() {
        const int ARRAY_SIZE = 10;
        int data[ARRAY_SIZE];
        
        // 안전한 배열 접근
        for (int i = 0; i < ARRAY_SIZE; i++) {
            data[i] = i * 2;
        }
        
        // 경계 검사 함수
        int safeRead(int index) {
            if (index >= 0 && index < ARRAY_SIZE) {
                return data[index];
            }
            Serial.println("Array index out of bounds!");
            return -1;
        }
    }
    
    // 3. 메모리 관리
    void memoryManagement() {
        // String 객체 남용 방지
        // BAD: String message = "Temperature: " + String(temp) + " C";
        
        // GOOD: 고정 크기 문자 배열 사용
        char message[50];
        float temp = 25.5;
        snprintf(message, sizeof(message), "Temperature: %.1f C", temp);
        Serial.println(message);
    }
    
    // 4. 타입 안전성
    void typeSafety() {
        // 명시적 캐스팅
        int analog_val = analogRead(A0);
        float voltage = (float)analog_val * 5.0 / 1023.0;
        
        // 상수 사용
        const int LED_PIN = 13;
        const float TEMP_OFFSET = 0.5;
        
        // enum 사용
        enum SystemState {
            INIT,
            RUNNING,
            ERROR,
            SHUTDOWN
        };
        
        SystemState current_state = INIT;
    }
    
    // 5. 함수 반환값 검사
    bool safeFunctionCall() {
        if (!initializeSensor()) {
            Serial.println("Sensor initialization failed");
            return false;
        }
        
        float reading = 0;
        if (readSensorSafe(&reading)) {
            Serial.print("Reading: ");
            Serial.println(reading);
            return true;
        } else {
            Serial.println("Sensor reading failed");
            return false;
        }
    }
    
private:
    bool initializeSensor() {
        // 센서 초기화 로직
        delay(100);
        return true; // 성공 시 true 반환
    }
    
    bool readSensorSafe(float* result) {
        if (result == nullptr) return false;
        
        int raw = analogRead(A0);
        if (raw < 0 || raw > 1023) {
            return false;
        }
        
        *result = raw * 5.0 / 1023.0;
        return true;
    }
};
```

**컴파일 오류 해결 체크리스트:**

```cpp
// 컴파일 오류 자동 검사 도구
class CompileErrorChecker {
public:
    void checkCommonIssues() {
        Serial.println("=== Compile Error Prevention Check ===");
        
        // 1. 라이브러리 의존성 확인
        checkLibraryIncludes();
        
        // 2. 문법 검사
        checkSyntaxIssues();
        
        // 3. 메모리 사용량 확인
        checkMemoryUsage();
        
        // 4. 핀 정의 검사
        checkPinDefinitions();
    }
    
private:
    void checkLibraryIncludes() {
        Serial.println("\n1. Library Include Check");
        
        #ifdef DHT_H
            Serial.println("✓ DHT library included");
        #else
            Serial.println("✗ DHT library not found");
        #endif
        
        #ifdef WiFi_h
            Serial.println("✓ WiFi library included");
        #else
            Serial.println("✗ WiFi library not found");
        #endif
    }
    
    void checkSyntaxIssues() {
        Serial.println("\n2. Syntax Check");
        
        // 괄호 매칭 확인 (컴파일 타임에 체크됨)
        int test_brackets = (1 + (2 * 3)); // OK
        
        // 세미콜론 체크
        int test_semicolon = 42; // OK
        
        Serial.println("✓ Basic syntax OK");
    }
    
    void checkMemoryUsage() {
        Serial.println("\n3. Memory Usage Check");
        
        extern int __heap_start, *__brkval;
        int ram_size = 2048; // Arduino Uno
        int used_ram = ram_size - freeRam();
        
        Serial.print("RAM usage: ");
        Serial.print(used_ram);
        Serial.print("/");
        Serial.print(ram_size);
        Serial.print(" (");
        Serial.print((float)used_ram / ram_size * 100);
        Serial.println("%)");
        
        if (used_ram > ram_size * 0.8) {
            Serial.println("⚠️ High RAM usage detected");
        }
    }
    
    void checkPinDefinitions() {
        Serial.println("\n4. Pin Definition Check");
        
        // 중복 핀 사용 검사
        int used_pins[] = {2, 3, 4, 13, A0};
        int pin_count = sizeof(used_pins) / sizeof(used_pins[0]);
        
        for (int i = 0; i < pin_count; i++) {
            for (int j = i + 1; j < pin_count; j++) {
                if (used_pins[i] == used_pins[j]) {
                    Serial.print("⚠️ Pin ");
                    Serial.print(used_pins[i]);
                    Serial.println(" used multiple times");
                }
            }
        }
        
        Serial.println("✓ Pin definition check complete");
    }
    
    int freeRam() {
        extern int __heap_start, *__brkval;
        int v;
        return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
    }
};
```

### 문제 2: 업로드 실패

**증상:**
- "avrdude: stk500_recv(): programmer is not responding"
- "espcomm_upload_mem failed"
- 업로드 중 중단

**해결 코드:**

```cpp
// 업로드 문제 진단 및 해결
class UploadDiagnostic {
public:
    void diagnoseUploadIssues() {
        Serial.println("=== Upload Diagnostic ===");
        
        // 1. 부트로더 확인
        checkBootloader();
        
        // 2. 시리얼 포트 상태
        checkSerialPort();
        
        // 3. 보드 설정 확인
        checkBoardSettings();
        
        // 4. 자동 리셋 확인
        checkAutoReset();
    }
    
private:
    void checkBootloader() {
        Serial.println("\n1. Bootloader Check");
        
        // 부트로더 시그니처 확인
        Serial.println("Checking bootloader signature...");
        
        // 현재 실행 중이라면 부트로더가 정상
        Serial.println("✓ Bootloader is functional (code is running)");
        
        // 리셋 후 부트로더 진입 시간 체크
        Serial.println("Note: Bootloader timeout is typically 2 seconds");
    }
    
    void checkSerialPort() {
        Serial.println("\n2. Serial Port Check");
        
        // 보드레이트 테스트
        Serial.println("Testing baud rates...");
        
        int baudRates[] = {9600, 57600, 115200};
        for (int i = 0; i < 3; i++) {
            Serial.print("Baud rate ");
            Serial.print(baudRates[i]);
            Serial.println(": Testing...");
            // 실제로는 각 보드레이트로 테스트해야 함
        }
        
        Serial.println("✓ Serial communication working");
    }
    
    void checkBoardSettings() {
        Serial.println("\n3. Board Settings Check");
        
        // 보드 타입 확인
        #ifdef ARDUINO_AVR_UNO
            Serial.println("Board: Arduino Uno");
            Serial.println("Processor: ATmega328P");
        #elif defined(ARDUINO_ESP32_DEV)
            Serial.println("Board: ESP32 Dev Module");
            Serial.println("Processor: ESP32");
        #elif defined(ARDUINO_AVR_NANO)
            Serial.println("Board: Arduino Nano");
            Serial.println("Processor: ATmega328P");
        #endif
        
        Serial.println("✓ Board configuration detected");
    }
    
    void checkAutoReset() {
        Serial.println("\n4. Auto-Reset Check");
        
        // DTR/RTS 신호 확인 (간접적)
        Serial.println("Auto-reset circuit status: ");
        Serial.println("- Check capacitor on RESET line (100nF)");
        Serial.println("- Verify DTR connection");
        Serial.println("- Ensure no manual reset during upload");
    }
};
```

**수동 업로드 방법:**

```cpp
// 수동 업로드를 위한 부트로더 모드 진입
void enterBootloaderMode() {
    Serial.println("Manual upload procedure:");
    Serial.println("1. Hold RESET button");
    Serial.println("2. Click Upload in IDE");
    Serial.println("3. Release RESET when 'Uploading...' appears");
    
    // ESP32의 경우
    #ifdef ARDUINO_ESP32_DEV
    Serial.println("ESP32 manual upload:");
    Serial.println("1. Hold BOOT button");
    Serial.println("2. Press and release RESET");
    Serial.println("3. Release BOOT button");
    Serial.println("4. Start upload");
    #endif
}
```

---

# 3. 통신 문제 해결

## 📡 시리얼 통신 문제

### 문제 1: 시리얼 통신 불안정

**증상:**
- 데이터 손실
- 문자 깨짐
- 간헐적 통신 중단

**해결 코드:**

```cpp
class SerialDiagnostic {
private:
    unsigned long lastRxTime = 0;
    unsigned long lastTxTime = 0;
    int rxErrors = 0;
    int txErrors = 0;
    
public:
    void initializeSerial() {
        Serial.begin(9600);
        while (!Serial) {
            ; // 시리얼 포트 연결 대기
        }
        
        Serial.println("=== Serial Communication Diagnostic ===");
        runSerialTests();
    }
    
    void runSerialTests() {
        // 1. 보드레이트 테스트
        testBaudRates();
        
        // 2. 데이터 무결성 테스트
        testDataIntegrity();
        
        // 3. 버퍼 오버플로우 테스트
        testBufferOverflow();
        
        // 4. 지연 시간 측정
        measureLatency();
    }
    
private:
    void testBaudRates() {
        Serial.println("\n1. Baud Rate Test");
        
        int baudRates[] = {9600, 19200, 38400, 57600, 115200};
        
        for (int i = 0; i < 5; i++) {
            Serial.print("Testing ");
            Serial.print(baudRates[i]);
            Serial.println(" baud...");
            
            // 실제 테스트에서는 보드레이트 변경 후 확인
            // Serial.end();
            // Serial.begin(baudRates[i]);
            
            bool success = testBaudRate(baudRates[i]);
            Serial.println(success ? "✓ OK" : "✗ FAILED");
        }
    }
    
    bool testBaudRate(int baudRate) {
        // 테스트 패턴 전송
        String testPattern = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
        
        unsigned long startTime = millis();
        Serial.println(testPattern);
        
        // 에코백 대기 (실제로는 외부 장치에서 에코)
        delay(100);
        
        unsigned long elapsed = millis() - startTime;
        
        // 전송 시간이 예상 범위 내인지 확인
        float expectedTime = (float)(testPattern.length() * 8) / baudRate * 1000;
        
        return (elapsed >= expectedTime * 0.8 && elapsed <= expectedTime * 1.5);
    }
    
    void testDataIntegrity() {
        Serial.println("\n2. Data Integrity Test");
        
        // 체크섬을 포함한 데이터 전송 테스트
        for (int i = 0; i < 10; i++) {
            sendChecksummedData(i);
            delay(100);
        }
    }
    
    void sendChecksummedData(int data) {
        uint8_t checksum = calculateChecksum((uint8_t*)&data, sizeof(data));
        
        Serial.print("DATA:");
        Serial.print(data);
        Serial.print(",CHK:");
        Serial.println(checksum, HEX);
    }
    
    uint8_t calculateChecksum(uint8_t* data, int length) {
        uint8_t checksum = 0;
        for (int i = 0; i < length; i++) {
            checksum ^= data[i];
        }
        return checksum;
    }
    
    void testBufferOverflow() {
        Serial.println("\n3. Buffer Overflow Test");
        
        // 대량 데이터 전송으로 버퍼 오버플로우 테스트
        const int TEST_SIZE = 1000;
        
        Serial.println("Sending large data block...");
        
        for (int i = 0; i < TEST_SIZE; i++) {
            Serial.print(i);
            Serial.print(",");
            
            // 너무 빠른 전송 방지
            if (i % 50 == 0) {
                delay(10);
            }
        }
        
        Serial.println("\nLarge data transmission complete");
    }
    
    void measureLatency() {
        Serial.println("\n4. Latency Measurement");
        
        unsigned long totalLatency = 0;
        int measurements = 10;
        
        for (int i = 0; i < measurements; i++) {
            unsigned long startTime = micros();
            
            Serial.print("PING");
            Serial.flush(); // 전송 완료 대기
            
            unsigned long latency = micros() - startTime;
            totalLatency += latency;
            
            Serial.print(" - Latency: ");
            Serial.print(latency);
            Serial.println(" μs");
            
            delay(100);
        }
        
        Serial.print("Average latency: ");
        Serial.print(totalLatency / measurements);
        Serial.println(" μs");
    }
};
```

### 문제 2: I2C 통신 오류

**증상:**
- 장치 응답 없음
- 데이터 손상
- 버스 충돌

**해결 코드:**

```cpp
#include <Wire.h>

class I2CDiagnostic {
private:
    int devicesFound = 0;
    uint8_t deviceAddresses[128];
    
public:
    void diagnoseI2C() {
        Serial.println("=== I2C Bus Diagnostic ===");
        
        Wire.begin();
        
        // 1. 버스 스캔
        scanI2CBus();
        
        // 2. 신호 품질 테스트
        testSignalQuality();
        
        // 3. 통신 속도 테스트
        testCommunicationSpeed();
        
        // 4. 오류 복구 테스트
        testErrorRecovery();
    }
    
private:
    void scanI2CBus() {
        Serial.println("\n1. I2C Bus Scan");
        Serial.println("Scanning I2C bus...");
        
        devicesFound = 0;
        
        for (uint8_t address = 1; address < 127; address++) {
            Wire.beginTransmission(address);
            uint8_t error = Wire.endTransmission();
            
            if (error == 0) {
                Serial.print("Device found at address 0x");
                if (address < 16) Serial.print("0");
                Serial.print(address, HEX);
                Serial.println();
                
                deviceAddresses[devicesFound] = address;
                devicesFound++;
            } else if (error == 4) {
                Serial.print("Unknown error at address 0x");
                if (address < 16) Serial.print("0");
                Serial.println(address, HEX);
            }
        }
        
        Serial.print("Total devices found: ");
        Serial.println(devicesFound);
        
        if (devicesFound == 0) {
            Serial.println("⚠️ No I2C devices found!");
            diagnoseI2CProblem();
        }
    }
    
    void diagnoseI2CProblem() {
        Serial.println("\nI2C Problem Diagnosis:");
        Serial.println("1. Check pull-up resistors (4.7kΩ on SDA and SCL)");
        Serial.println("2. Verify wiring: SDA to A4, SCL to A5 (Uno)");
        Serial.println("3. Check power supply to I2C devices");
        Serial.println("4. Ensure common ground connection");
        Serial.println("5. Check for short circuits");
    }
    
    void testSignalQuality() {
        Serial.println("\n2. Signal Quality Test");
        
        if (devicesFound == 0) {
            Serial.println("No devices to test");
            return;
        }
        
        uint8_t testAddress = deviceAddresses[0];
        int successCount = 0;
        int totalTests = 100;
        
        for (int i = 0; i < totalTests; i++) {
            Wire.beginTransmission(testAddress);
            uint8_t error = Wire.endTransmission();
            
            if (error == 0) {
                successCount++;
            }
            
            delay(10);
        }
        
        float successRate = (float)successCount / totalTests * 100;
        Serial.print("Signal quality: ");
        Serial.print(successRate);
        Serial.println("%");
        
        if (successRate < 95) {
            Serial.println("⚠️ Poor signal quality detected");
            Serial.println("- Check connections");
            Serial.println("- Reduce cable length");
            Serial.println("- Check for interference");
        }
    }
    
    void testCommunicationSpeed() {
        Serial.println("\n3. Communication Speed Test");
        
        if (devicesFound == 0) return;
        
        // 다양한 클럭 속도 테스트
        long clockSpeeds[] = {100000, 400000, 1000000}; // 100kHz, 400kHz, 1MHz
        
        for (int i = 0; i < 3; i++) {
            Wire.setClock(clockSpeeds[i]);
            
            Serial.print("Testing ");
            Serial.print(clockSpeeds[i] / 1000);
            Serial.print(" kHz: ");
            
            bool success = testAtSpeed(deviceAddresses[0]);
            Serial.println(success ? "✓ OK" : "✗ FAILED");
            
            delay(100);
        }
        
        // 기본 속도로 복원
        Wire.setClock(100000);
    }
    
    bool testAtSpeed(uint8_t address) {
        int successCount = 0;
        
        for (int i = 0; i < 10; i++) {
            Wire.beginTransmission(address);
            uint8_t error = Wire.endTransmission();
            
            if (error == 0) {
                successCount++;
            }
        }
        
        return successCount >= 8; // 80% 성공률
    }
    
    void testErrorRecovery() {
        Serial.println("\n4. Error Recovery Test");
        
        // 버스 리셋 테스트
        resetI2CBus();
        
        // 재스캔
        Serial.println("Rescanning after reset...");
        int newDeviceCount = quickScan();
        
        if (newDeviceCount == devicesFound) {
            Serial.println("✓ Bus recovery successful");
        } else {
            Serial.println("⚠️ Bus recovery issue detected");
        }
    }
    
    void resetI2CBus() {
        // 소프트웨어 리셋
        Wire.end();
        delay(100);
        Wire.begin();
        
        // 하드웨어 리셋이 필요한 경우 추가 구현
        Serial.println("I2C bus reset performed");
    }
    
    int quickScan() {
        int count = 0;
        
        for (uint8_t address = 1; address < 127; address++) {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0) {
                count++;
            }
        }
        
        return count;
    }
};
```

### 문제 3: WiFi 연결 문제 (ESP32/ESP8266)

```cpp
#ifdef ESP32
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

class WiFiDiagnostic {
private:
    String ssid = "YOUR_SSID";
    String password = "YOUR_PASSWORD";
    int connectionAttempts = 0;
    
public:
    void diagnoseWiFi() {
        Serial.println("=== WiFi Diagnostic ===");
        
        // 1. WiFi 모듈 상태 확인
        checkWiFiModule();
        
        // 2. 네트워크 스캔
        scanNetworks();
        
        // 3. 연결 테스트
        testConnection();
        
        // 4. 신호 강도 모니터링
        monitorSignalStrength();
        
        // 5. 인터넷 연결 테스트
        testInternetConnection();
    }
    
private:
    void checkWiFiModule() {
        Serial.println("\n1. WiFi Module Check");
        
        #ifdef ESP32
        Serial.println("Platform: ESP32");
        Serial.print("WiFi Mode: ");
        Serial.println(WiFi.getMode());
        #elif defined(ESP8266)
        Serial.println("Platform: ESP8266");
        #endif
        
        // MAC 주소 확인
        Serial.print("MAC Address: ");
        Serial.println(WiFi.macAddress());
        
        // 모듈 상태
        if (WiFi.status() == WL_NO_SHIELD) {
            Serial.println("⚠️ WiFi module not detected");
        } else {
            Serial.println("✓ WiFi module detected");
        }
    }
    
    void scanNetworks() {
        Serial.println("\n2. Network Scan");
        Serial.println("Scanning for networks...");
        
        int networkCount = WiFi.scanNetworks();
        
        if (networkCount == 0) {
            Serial.println("No networks found");
        } else {
            Serial.print(networkCount);
            Serial.println(" networks found:");
            
            for (int i = 0; i < networkCount; i++) {
                Serial.print(i + 1);
                Serial.print(": ");
                Serial.print(WiFi.SSID(i));
                Serial.print(" (");
                Serial.print(WiFi.RSSI(i));
                Serial.print(" dBm) ");
                Serial.print(getEncryptionType(WiFi.encryptionType(i)));
                Serial.println();
            }
        }
        
        // 메모리 정리
        WiFi.scanDelete();
    }
    
    String getEncryptionType(int type) {
        switch (type) {
            case WIFI_AUTH_OPEN: return "Open";
            case WIFI_AUTH_WEP: return "WEP";
            case WIFI_AUTH_WPA_PSK: return "WPA";
            case WIFI_AUTH_WPA2_PSK: return "WPA2";
            case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
            default: return "Unknown";
        }
    }
    
    void testConnection() {
        Serial.println("\n3. Connection Test");
        
        Serial.print("Connecting to ");
        Serial.println(ssid);
        
        WiFi.begin(ssid.c_str(), password.c_str());
        
        unsigned long startTime = millis();
        connectionAttempts = 0;
        
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) {
            delay(1000);
            Serial.print(".");
            connectionAttempts++;
            
            // 상태 확인
            printConnectionStatus();
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\n✓ Connected successfully!");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            Serial.print("Gateway: ");
            Serial.println(WiFi.gatewayIP());
            Serial.print("DNS: ");
            Serial.println(WiFi.dnsIP());
        } else {
            Serial.println("\n✗ Connection failed");
            diagnoseConnectionFailure();
        }
    }
    
    void printConnectionStatus() {
        switch (WiFi.status()) {
            case WL_IDLE_STATUS:
                Serial.print(" [IDLE]");
                break;
            case WL_NO_SSID_AVAIL:
                Serial.print(" [NO_SSID]");
                break;
            case WL_SCAN_COMPLETED:
                Serial.print(" [SCAN_COMPLETED]");
                break;
            case WL_CONNECTED:
                Serial.print(" [CONNECTED]");
                break;
            case WL_CONNECT_FAILED:
                Serial.print(" [CONNECT_FAILED]");
                break;
            case WL_CONNECTION_LOST:
                Serial.print(" [CONNECTION_LOST]");
                break;
            case WL_DISCONNECTED:
                Serial.print(" [DISCONNECTED]");
                break;
            default:
                Serial.print(" [UNKNOWN]");
                break;
        }
    }
    
    void diagnoseConnectionFailure() {
        Serial.println("\nConnection Failure Diagnosis:");
        
        switch (WiFi.status()) {
            case WL_NO_SSID_AVAIL:
                Serial.println("- SSID not found");
                Serial.println("- Check network name spelling");
                Serial.println("- Ensure network is broadcasting");
                break;
                
            case WL_CONNECT_FAILED:
                Serial.println("- Authentication failed");
                Serial.println("- Check password");
                Serial.println("- Verify encryption type");
                break;
                
            case WL_CONNECTION_LOST:
                Serial.println("- Connection lost");
                Serial.println("- Check signal strength");
                Serial.println("- Network may be unstable");
                break;
                
            default:
                Serial.println("- Unknown connection issue");
                Serial.println("- Try resetting WiFi module");
                break;
        }
        
        // 자동 복구 시도
        attemptRecovery();
    }
    
    void attemptRecovery() {
        Serial.println("\nAttempting recovery...");
        
        // WiFi 리셋
        WiFi.disconnect();
        delay(1000);
        
        // 재연결 시도
        WiFi.begin(ssid.c_str(), password.c_str());
        
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
            delay(500);
            Serial.print(".");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\n✓ Recovery successful");
        } else {
            Serial.println("\n✗ Recovery failed");
        }
    }
    
    void monitorSignalStrength() {
        if (WiFi.status() != WL_CONNECTED) return;
        
        Serial.println("\n4. Signal Strength Monitoring");
        
        for (int i = 0; i < 10; i++) {
            int rssi = WiFi.RSSI();
            Serial.print("RSSI: ");
            Serial.print(rssi);
            Serial.print(" dBm (");
            Serial.print(getSignalQuality(rssi));
            Serial.println(")");
            
            delay(1000);
        }
    }
    
    String getSignalQuality(int rssi) {
        if (rssi > -50) return "Excellent";
        else if (rssi > -60) return "Good";
        else if (rssi > -70) return "Fair";
        else if (rssi > -80) return "Poor";
        else return "Very Poor";
    }
    
    void testInternetConnection() {
        if (WiFi.status() != WL_CONNECTED) return;
        
        Serial.println("\n5. Internet Connection Test");
        
        WiFiClient client;
        
        if (client.connect("www.google.com", 80)) {
            Serial.println("✓ Internet connection OK");
            client.stop();
        } else {
            Serial.println("✗ No internet access");
            Serial.println("- Check router internet connection");
            Serial.println("- Verify DNS settings");
        }
    }
};
```

이제 성능 최적화 가이드를 작성하겠습니다.

<function_calls>
<invoke name="TodoWrite">
<parameter name="todos">[{"content": "\uace0\uae09 \ud655\uc7a5 \uac00\uc774\ub4dc \uc791\uc131 - \uc0c1\uc6a9\ud654 \uc900\ube44", "status": "completed", "priority": "high", "id": "23"}, {"content": "\uc2e4\ubb34 \ud504\ub85c\uc81d\ud2b8 \uc544\uc774\ub514\uc5b4 \uceec\ub809\uc158 \uc0dd\uc131", "status": "completed", "priority": "medium", "id": "24"}, {"content": "\ud2b8\ub7ec\ube14\uc288\ud305 \uac00\uc774\ub4dc \uc791\uc131", "status": "completed", "priority": "medium", "id": "25"}, {"content": "\uc131\ub2a5 \ucd5c\uc801\ud654 \uac00\uc774\ub4dc \uc791\uc131", "status": "in_progress", "priority": "medium", "id": "26"}]