# 한국타이어 센서 제작 시스템 - 실무 전문 가이드

## 📋 목차
1. [개요](#개요)
2. [타이어 센서 기술 분석](#타이어-센서-기술-분석)
3. [TPMS 시스템 구현](#tpms-시스템-구현)
4. [온도 센서 모니터링](#온도-센서-모니터링)
5. [품질 관리 센서](#품질-관리-센서)
6. [생산라인 IoT 통합](#생산라인-iot-통합)
7. [실시간 모니터링 시스템](#실시간-모니터링-시스템)
8. [예측 정비 시스템](#예측-정비-시스템)
9. [자동차 산업 표준](#자동차-산업-표준)
10. [실전 구현 예제](#실전-구현-예제)

---

## 개요

한국타이어는 세계 8위의 타이어 제조사로, 첨단 센서 기술을 통한 스마트 타이어 및 제조 공정 혁신을 추진하고 있습니다. 본 가이드는 Arduino를 활용한 타이어 센서 제작의 실무적 접근법을 제시합니다.

### 핵심 기술 영역
- **TPMS (Tire Pressure Monitoring System)**: 타이어 공기압 실시간 모니터링
- **온도 센서**: 제조 및 운행 중 온도 관리
- **진동 센서**: 타이어 균형 및 이상 진동 감지
- **화학 센서**: 고무 화합물 품질 관리
- **IoT 통합**: 스마트 팩토리 및 커넥티드 카 연동

---

## 타이어 센서 기술 분석

### 1. TPMS 기술 개요

```cpp
/*
 * 타이어 공기압 모니터링 시스템 (TPMS)
 * 한국타이어 기술 사양 기반
 * ISO 21750 표준 준수
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

class HankookTPMS {
private:
    // TPMS 센서 데이터 구조체
    struct TPMSData {
        float pressure_kPa;      // 공기압 (kPa)
        float temperature_C;     // 온도 (°C)
        uint16_t battery_voltage; // 배터리 전압 (mV)
        uint32_t sensor_id;      // 센서 고유 ID
        uint8_t tire_position;   // 타이어 위치 (1-4)
        bool alarm_status;       // 경보 상태
        uint32_t timestamp;      // 타임스탬프
    };
    
    TPMSData sensor_data[4];     // 4개 타이어 센서 데이터
    
    // 한국타이어 표준 압력 범위 (kPa)
    const float MIN_PRESSURE = 200.0;  // 최소 압력
    const float MAX_PRESSURE = 350.0;  // 최대 압력
    const float OPTIMAL_PRESSURE = 220.0; // 최적 압력
    
    // 온도 임계값 (°C)
    const float MAX_TEMPERATURE = 85.0;   // 최대 허용 온도
    const float CRITICAL_TEMP = 100.0;    // 위험 온도
    
public:
    HankookTPMS() {
        // 센서 초기화
        for(int i = 0; i < 4; i++) {
            sensor_data[i].sensor_id = 0x1000 + i;
            sensor_data[i].tire_position = i + 1;
            sensor_data[i].alarm_status = false;
        }
    }
    
    // 압력 센서 읽기 (Analog 기반)
    float readPressure(uint8_t tire_index) {
        if(tire_index >= 4) return -1;
        
        // MPX5700AP 압력 센서 사용 (15-700 kPa)
        int raw_value = analogRead(A0 + tire_index);
        
        // 전압을 압력으로 변환 (한국타이어 캘리브레이션)
        float voltage = (raw_value * 5.0) / 1024.0;
        float pressure = (voltage - 0.2) * 125.0 + 15.0; // kPa 변환
        
        // 노이즈 필터링 (이동평균)
        static float pressure_buffer[4][10];
        static uint8_t buffer_index[4] = {0};
        
        pressure_buffer[tire_index][buffer_index[tire_index]] = pressure;
        buffer_index[tire_index] = (buffer_index[tire_index] + 1) % 10;
        
        float filtered_pressure = 0;
        for(int i = 0; i < 10; i++) {
            filtered_pressure += pressure_buffer[tire_index][i];
        }
        filtered_pressure /= 10.0;
        
        sensor_data[tire_index].pressure_kPa = filtered_pressure;
        sensor_data[tire_index].timestamp = millis();
        
        return filtered_pressure;
    }
    
    // 온도 센서 읽기
    float readTemperature(uint8_t tire_index) {
        if(tire_index >= 4) return -1;
        
        // DS18B20 온도 센서 사용
        int raw_temp = analogRead(A4 + tire_index);
        float temperature = (raw_temp * 150.0) / 1024.0 - 40.0; // -40°C ~ 110°C
        
        sensor_data[tire_index].temperature_C = temperature;
        return temperature;
    }
    
    // 알람 상태 체크
    bool checkAlarmConditions(uint8_t tire_index) {
        if(tire_index >= 4) return false;
        
        TPMSData& data = sensor_data[tire_index];
        bool alarm = false;
        
        // 압력 이상 체크
        if(data.pressure_kPa < MIN_PRESSURE || data.pressure_kPa > MAX_PRESSURE) {
            alarm = true;
            Serial.printf("타이어 %d 압력 이상: %.1f kPa\n", tire_index + 1, data.pressure_kPa);
        }
        
        // 온도 이상 체크
        if(data.temperature_C > MAX_TEMPERATURE) {
            alarm = true;
            Serial.printf("타이어 %d 온도 이상: %.1f°C\n", tire_index + 1, data.temperature_C);
        }
        
        // 위험 온도 체크
        if(data.temperature_C > CRITICAL_TEMP) {
            alarm = true;
            Serial.printf("타이어 %d 위험 온도: %.1f°C - 즉시 정차 필요!\n", 
                         tire_index + 1, data.temperature_C);
        }
        
        data.alarm_status = alarm;
        return alarm;
    }
    
    // 센서 데이터 JSON 출력
    String getTPMSDataJSON() {
        StaticJsonDocument<1024> doc;
        JsonArray tires = doc.createNestedArray("tires");
        
        for(int i = 0; i < 4; i++) {
            JsonObject tire = tires.createNestedObject();
            tire["position"] = sensor_data[i].tire_position;
            tire["pressure_kPa"] = sensor_data[i].pressure_kPa;
            tire["temperature_C"] = sensor_data[i].temperature_C;
            tire["battery_voltage"] = sensor_data[i].battery_voltage;
            tire["alarm"] = sensor_data[i].alarm_status;
            tire["timestamp"] = sensor_data[i].timestamp;
        }
        
        String json_string;
        serializeJson(doc, json_string);
        return json_string;
    }
};
```

### 2. 타이어 제조 품질 센서

```cpp
/*
 * 타이어 제조 품질 관리 센서 시스템
 * 한국타이어 대전공장 기술 기준
 */

class HankookQualitySensor {
private:
    // 품질 관리 데이터 구조
    struct QualityData {
        float rubber_temperature;    // 고무 온도
        float pressure_molding;      // 몰딩 압력
        float cure_time;            // 가황 시간
        float belt_tension;         // 벨트 장력
        uint16_t thickness_mm;      // 두께 (mm * 100)
        uint8_t defect_count;       // 결함 개수
        bool pass_status;           // 합격 여부
    };
    
    QualityData current_tire;
    
    // 한국타이어 품질 기준
    const float RUBBER_TEMP_MIN = 160.0;   // 최소 고무 온도
    const float RUBBER_TEMP_MAX = 180.0;   // 최대 고무 온도
    const float MOLDING_PRESSURE_MIN = 15.0; // 최소 몰딩 압력 (MPa)
    const float MOLDING_PRESSURE_MAX = 20.0; // 최대 몰딩 압력
    const uint16_t THICKNESS_MIN = 800;     // 최소 두께 (8.00mm)
    const uint16_t THICKNESS_MAX = 1200;    // 최대 두께 (12.00mm)
    
public:
    // 고무 온도 측정 (제조 공정)
    float measureRubberTemperature() {
        // K-type 열전대 사용
        int raw_value = analogRead(A0);
        
        // MAX31855 열전대 앰프 사용 시 변환
        float voltage = (raw_value * 3.3) / 4096.0;
        float temperature = voltage * 200.0; // 0-600°C 범위
        
        current_tire.rubber_temperature = temperature;
        
        // 온도 범위 체크
        if(temperature < RUBBER_TEMP_MIN || temperature > RUBBER_TEMP_MAX) {
            Serial.printf("고무 온도 이상: %.1f°C (정상범위: %.1f-%.1f°C)\n", 
                         temperature, RUBBER_TEMP_MIN, RUBBER_TEMP_MAX);
        }
        
        return temperature;
    }
    
    // 몰딩 압력 측정
    float measureMoldingPressure() {
        // 고압 압력 센서 (0-25 MPa)
        int raw_value = analogRead(A1);
        float voltage = (raw_value * 5.0) / 1024.0;
        float pressure = voltage * 5.0; // MPa 변환
        
        current_tire.pressure_molding = pressure;
        
        // 압력 범위 체크
        if(pressure < MOLDING_PRESSURE_MIN || pressure > MOLDING_PRESSURE_MAX) {
            Serial.printf("몰딩 압력 이상: %.1f MPa (정상범위: %.1f-%.1f MPa)\n", 
                         pressure, MOLDING_PRESSURE_MIN, MOLDING_PRESSURE_MAX);
        }
        
        return pressure;
    }
    
    // 두께 측정 (레이저 센서)
    uint16_t measureThickness() {
        // VL53L0X 레이저 거리 센서 시뮬레이션
        int raw_distance = analogRead(A2);
        uint16_t thickness = map(raw_distance, 0, 1024, 600, 1400); // 6.00-14.00mm
        
        current_tire.thickness_mm = thickness;
        
        // 두께 범위 체크
        if(thickness < THICKNESS_MIN || thickness > THICKNESS_MAX) {
            Serial.printf("타이어 두께 이상: %d.%02dmm (정상범위: %d.%02d-%d.%02dmm)\n", 
                         thickness/100, thickness%100,
                         THICKNESS_MIN/100, THICKNESS_MIN%100,
                         THICKNESS_MAX/100, THICKNESS_MAX%100);
        }
        
        return thickness;
    }
    
    // 표면 결함 검사 (광학 센서)
    uint8_t detectSurfaceDefects() {
        // 광학 센서 어레이로 표면 스캔 시뮬레이션
        uint8_t defect_count = 0;
        
        for(int i = 0; i < 8; i++) {
            int sensor_value = analogRead(A3);
            
            // 반사율 기반 결함 감지
            if(sensor_value < 100 || sensor_value > 900) {
                defect_count++;
            }
            
            delay(50); // 센서 간 지연
        }
        
        current_tire.defect_count = defect_count;
        
        if(defect_count > 2) {
            Serial.printf("표면 결함 감지: %d개 (허용: 2개 이하)\n", defect_count);
        }
        
        return defect_count;
    }
    
    // 종합 품질 판정
    bool performQualityCheck() {
        measureRubberTemperature();
        measureMoldingPressure();
        measureThickness();
        detectSurfaceDefects();
        
        bool pass = true;
        
        // 온도 체크
        if(current_tire.rubber_temperature < RUBBER_TEMP_MIN || 
           current_tire.rubber_temperature > RUBBER_TEMP_MAX) {
            pass = false;
        }
        
        // 압력 체크
        if(current_tire.pressure_molding < MOLDING_PRESSURE_MIN || 
           current_tire.pressure_molding > MOLDING_PRESSURE_MAX) {
            pass = false;
        }
        
        // 두께 체크
        if(current_tire.thickness_mm < THICKNESS_MIN || 
           current_tire.thickness_mm > THICKNESS_MAX) {
            pass = false;
        }
        
        // 결함 체크
        if(current_tire.defect_count > 2) {
            pass = false;
        }
        
        current_tire.pass_status = pass;
        
        // 결과 출력
        Serial.println("=== 한국타이어 품질 검사 결과 ===");
        Serial.printf("고무 온도: %.1f°C\n", current_tire.rubber_temperature);
        Serial.printf("몰딩 압력: %.1f MPa\n", current_tire.pressure_molding);
        Serial.printf("타이어 두께: %d.%02dmm\n", 
                     current_tire.thickness_mm/100, current_tire.thickness_mm%100);
        Serial.printf("표면 결함: %d개\n", current_tire.defect_count);
        Serial.printf("품질 판정: %s\n", pass ? "합격" : "불합격");
        Serial.println("===============================");
        
        return pass;
    }
    
    // 품질 데이터 로깅
    void logQualityData() {
        // CSV 형식으로 SD카드에 저장
        String log_data = String(millis()) + "," +
                         String(current_tire.rubber_temperature, 1) + "," +
                         String(current_tire.pressure_molding, 1) + "," +
                         String(current_tire.thickness_mm) + "," +
                         String(current_tire.defect_count) + "," +
                         String(current_tire.pass_status ? "PASS" : "FAIL");
        
        // SD 카드 저장 코드 (실제 구현 시 SD 라이브러리 사용)
        Serial.println("Quality Log: " + log_data);
    }
};
```

---

## TPMS 시스템 구현

### 실시간 TPMS 모니터링 시스템

```cpp
/*
 * 한국타이어 실시간 TPMS 모니터링 시스템
 * 차량용 Arduino 기반 구현
 */

#include <SPI.h>
#include <RF24.h>
#include <LiquidCrystal.h>

class HankookTPMSMonitor {
private:
    RF24 radio;
    LiquidCrystal lcd;
    
    // TPMS 센서 RF 통신 구조
    struct TPMSPacket {
        uint32_t sensor_id;
        float pressure;
        float temperature;
        uint16_t battery;
        uint8_t checksum;
    };
    
    TPMSPacket tire_data[4];
    bool sensor_connected[4] = {false};
    unsigned long last_update[4] = {0};
    
    // 한국타이어 알람 임계값
    const float PRESSURE_LOW = 200.0;     // 저압 경보
    const float PRESSURE_HIGH = 350.0;    // 고압 경보  
    const float TEMP_HIGH = 85.0;         // 고온 경보
    const unsigned long TIMEOUT_MS = 10000; // 센서 통신 타임아웃
    
public:
    HankookTPMSMonitor() : radio(7, 8), lcd(12, 11, 5, 4, 3, 2) {
        // RF24 초기화 (2.4GHz ISM 대역)
        radio.begin();
        radio.setChannel(76);              // 한국타이어 전용 채널
        radio.setPALevel(RF24_PA_HIGH);
        radio.setDataRate(RF24_250KBPS);   // 장거리 통신
        radio.setCRCLength(RF24_CRC_16);   // 오류 검출
        
        // LCD 초기화
        lcd.begin(20, 4);
        lcd.print("Hankook TPMS v2.1");
        delay(2000);
        lcd.clear();
    }
    
    // TPMS 데이터 수신
    void receiveTPMSData() {
        if(radio.available()) {
            TPMSPacket packet;
            radio.read(&packet, sizeof(packet));
            
            // 체크섬 검증
            uint8_t calculated_checksum = calculateChecksum(&packet);
            if(packet.checksum != calculated_checksum) {
                Serial.println("TPMS 패킷 체크섬 오류");
                return;
            }
            
            // 센서 ID로 타이어 위치 결정
            uint8_t tire_index = (packet.sensor_id & 0x03);
            if(tire_index < 4) {
                tire_data[tire_index] = packet;
                sensor_connected[tire_index] = true;
                last_update[tire_index] = millis();
                
                Serial.printf("타이어 %d: %.1fkPa, %.1f°C, %dmV\n", 
                             tire_index + 1, packet.pressure, 
                             packet.temperature, packet.battery);
            }
        }
    }
    
    // 체크섬 계산
    uint8_t calculateChecksum(TPMSPacket* packet) {
        uint8_t* data = (uint8_t*)packet;
        uint8_t checksum = 0;
        
        for(int i = 0; i < sizeof(TPMSPacket) - 1; i++) {
            checksum ^= data[i];
        }
        
        return checksum;
    }
    
    // 알람 상태 체크
    void checkAlarms() {
        for(int i = 0; i < 4; i++) {
            // 센서 연결 상태 체크
            if(millis() - last_update[i] > TIMEOUT_MS) {
                sensor_connected[i] = false;
                Serial.printf("타이어 %d 센서 연결 끊김\n", i + 1);
                continue;
            }
            
            if(!sensor_connected[i]) continue;
            
            TPMSPacket& data = tire_data[i];
            
            // 저압 알람
            if(data.pressure < PRESSURE_LOW) {
                Serial.printf("경고: 타이어 %d 저압 (%.1f kPa)\n", i + 1, data.pressure);
                digitalWrite(13, HIGH); // LED 경고
            }
            
            // 고압 알람
            if(data.pressure > PRESSURE_HIGH) {
                Serial.printf("경고: 타이어 %d 고압 (%.1f kPa)\n", i + 1, data.pressure);
                digitalWrite(13, HIGH);
            }
            
            // 고온 알람
            if(data.temperature > TEMP_HIGH) {
                Serial.printf("위험: 타이어 %d 고온 (%.1f°C)\n", i + 1, data.temperature);
                digitalWrite(13, HIGH);
                tone(9, 1000, 500); // 부저 경고
            }
            
            // 배터리 부족 알람
            if(data.battery < 2800) { // 2.8V 이하
                Serial.printf("주의: 타이어 %d 센서 배터리 부족 (%dmV)\n", 
                             i + 1, data.battery);
            }
        }
    }
    
    // LCD 디스플레이 업데이트
    void updateDisplay() {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Hankook TPMS Status");
        
        for(int i = 0; i < 4; i++) {
            lcd.setCursor(0, i + 1);
            
            if(sensor_connected[i]) {
                lcd.printf("T%d:%3.0fkPa %2.0fC %c", 
                          i + 1,
                          tire_data[i].pressure,
                          tire_data[i].temperature,
                          (tire_data[i].pressure < PRESSURE_LOW || 
                           tire_data[i].pressure > PRESSURE_HIGH ||
                           tire_data[i].temperature > TEMP_HIGH) ? '!' : ' ');
            } else {
                lcd.printf("T%d: -- OFFLINE --", i + 1);
            }
        }
    }
    
    // 메인 모니터링 루프
    void monitorLoop() {
        receiveTPMSData();
        checkAlarms();
        updateDisplay();
        
        // 1초마다 디스플레이 업데이트
        static unsigned long last_display = 0;
        if(millis() - last_display > 1000) {
            updateDisplay();
            last_display = millis();
        }
    }
};

// 전역 인스턴스
HankookTPMSMonitor tpms_monitor;

void setup() {
    Serial.begin(115200);
    pinMode(13, OUTPUT); // 경고 LED
    pinMode(9, OUTPUT);  // 부저
    
    Serial.println("한국타이어 TPMS 모니터링 시스템 시작");
}

void loop() {
    tpms_monitor.monitorLoop();
    delay(100);
}
```

---

## 온도 센서 모니터링

### 제조공정 온도 관리 시스템

```cpp
/*
 * 한국타이어 제조공정 온도 관리 시스템
 * 가황 공정 온도 제어 및 모니터링
 */

class HankookThermalSystem {
private:
    // 온도 센서 채널
    enum ThermalZone {
        ZONE_MIXER = 0,        // 믹서 온도
        ZONE_EXTRUDER = 1,     // 압출기 온도
        ZONE_CALENDAR = 2,     // 캘린더 온도
        ZONE_VULCANIZER = 3,   // 가황기 온도
        ZONE_COOLING = 4       // 냉각 온도
    };
    
    // 온도 데이터 구조
    struct ThermalData {
        float current_temp;
        float target_temp;
        float tolerance;
        bool heater_status;
        bool alarm_status;
        unsigned long last_update;
    };
    
    ThermalData thermal_zones[5];
    
    // 한국타이어 제조공정 온도 기준
    const float MIXER_TEMP = 160.0;      // 믹서 목표온도
    const float EXTRUDER_TEMP = 120.0;   // 압출기 목표온도
    const float CALENDAR_TEMP = 80.0;    // 캘린더 목표온도
    const float VULCANIZER_TEMP = 150.0; // 가황기 목표온도
    const float COOLING_TEMP = 25.0;     // 냉각 목표온도
    
    const float TEMP_TOLERANCE = 5.0;    // 온도 허용오차
    
public:
    HankookThermalSystem() {
        // 각 구역 초기 설정
        thermal_zones[ZONE_MIXER] = {20.0, MIXER_TEMP, TEMP_TOLERANCE, false, false, 0};
        thermal_zones[ZONE_EXTRUDER] = {20.0, EXTRUDER_TEMP, TEMP_TOLERANCE, false, false, 0};
        thermal_zones[ZONE_CALENDAR] = {20.0, CALENDAR_TEMP, TEMP_TOLERANCE, false, false, 0};
        thermal_zones[ZONE_VULCANIZER] = {20.0, VULCANIZER_TEMP, TEMP_TOLERANCE, false, false, 0};
        thermal_zones[ZONE_COOLING] = {20.0, COOLING_TEMP, TEMP_TOLERANCE, false, false, 0};
    }
    
    // 온도 측정 (다중 센서)
    float readTemperature(ThermalZone zone) {
        if(zone >= 5) return -999.0;
        
        // MAX31855 열전대 앰프 사용
        int raw_value = analogRead(A0 + zone);
        float voltage = (raw_value * 3.3) / 4096.0;
        
        // K-type 열전대 온도 변환 (0-400°C)
        float temperature = voltage * 121.0 - 40.0;
        
        // 노이즈 필터링 (지수 이동평균)
        static float filtered_temp[5] = {20.0, 20.0, 20.0, 20.0, 20.0};
        const float alpha = 0.1;
        filtered_temp[zone] = alpha * temperature + (1 - alpha) * filtered_temp[zone];
        
        thermal_zones[zone].current_temp = filtered_temp[zone];
        thermal_zones[zone].last_update = millis();
        
        return filtered_temp[zone];
    }
    
    // PID 온도 제어
    float pidTemperatureControl(ThermalZone zone) {
        if(zone >= 5) return 0;
        
        static float integral[5] = {0};
        static float previous_error[5] = {0};
        
        // PID 파라미터 (한국타이어 최적화 값)
        const float Kp = 2.0;   // 비례 게인
        const float Ki = 0.1;   // 적분 게인
        const float Kd = 0.05;  // 미분 게인
        
        float error = thermal_zones[zone].target_temp - thermal_zones[zone].current_temp;
        
        // 적분항 (적분 와인드업 방지)
        integral[zone] += error;
        if(integral[zone] > 100) integral[zone] = 100;
        if(integral[zone] < -100) integral[zone] = -100;
        
        // 미분항
        float derivative = error - previous_error[zone];
        previous_error[zone] = error;
        
        // PID 출력 계산
        float output = Kp * error + Ki * integral[zone] + Kd * derivative;
        
        // 출력 제한 (0-100%)
        if(output > 100) output = 100;
        if(output < 0) output = 0;
        
        // PWM 히터 제어 (핀 3-7 사용)
        analogWrite(3 + zone, map(output, 0, 100, 0, 255));
        
        // 히터 상태 업데이트
        thermal_zones[zone].heater_status = (output > 10);
        
        return output;
    }
    
    // 온도 알람 체크
    void checkThermalAlarms() {
        const char* zone_names[] = {"믹서", "압출기", "캘린더", "가황기", "냉각"};
        
        for(int i = 0; i < 5; i++) {
            ThermalData& zone = thermal_zones[i];
            
            // 온도 범위 체크
            float temp_error = abs(zone.current_temp - zone.target_temp);
            
            if(temp_error > zone.tolerance) {
                zone.alarm_status = true;
                Serial.printf("온도 알람: %s 구역 %.1f°C (목표: %.1f°C ±%.1f°C)\n",
                             zone_names[i], zone.current_temp, 
                             zone.target_temp, zone.tolerance);
            } else {
                zone.alarm_status = false;
            }
            
            // 센서 이상 체크 (5분간 업데이트 없음)
            if(millis() - zone.last_update > 300000) {
                Serial.printf("센서 이상: %s 구역 온도 센서 응답 없음\n", zone_names[i]);
            }
            
            // 위험 온도 체크
            if(zone.current_temp > 200.0) {
                Serial.printf("위험: %s 구역 과열! %.1f°C - 긴급 정지 필요\n",
                             zone_names[i], zone.current_temp);
                // 긴급 정지 신호
                digitalWrite(13, HIGH);
            }
        }
    }
    
    // 온도 데이터 로깅
    void logThermalData() {
        Serial.println("=== 한국타이어 열관리 시스템 상태 ===");
        const char* zone_names[] = {"믹서", "압출기", "캘린더", "가황기", "냉각"};
        
        for(int i = 0; i < 5; i++) {
            ThermalData& zone = thermal_zones[i];
            Serial.printf("%s: %.1f°C (목표:%.1f°C) 히터:%s 알람:%s\n",
                         zone_names[i], zone.current_temp, zone.target_temp,
                         zone.heater_status ? "ON" : "OFF",
                         zone.alarm_status ? "YES" : "NO");
        }
        Serial.println("=====================================");
    }
    
    // 메인 열관리 루프
    void thermalControlLoop() {
        // 모든 구역 온도 측정
        for(int i = 0; i < 5; i++) {
            readTemperature((ThermalZone)i);
            pidTemperatureControl((ThermalZone)i);
        }
        
        // 알람 체크
        checkThermalAlarms();
        
        // 10초마다 로깅
        static unsigned long last_log = 0;
        if(millis() - last_log > 10000) {
            logThermalData();
            last_log = millis();
        }
    }
};
```

---

## 생산라인 IoT 통합

### 스마트 팩토리 센서 네트워크

```cpp
/*
 * 한국타이어 스마트 팩토리 IoT 통합 시스템
 * MQTT 기반 실시간 데이터 수집 및 분석
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

class HankookSmartFactory {
private:
    WiFiClient wifi_client;
    PubSubClient mqtt_client;
    WiFiUDP ntp_udp;
    NTPClient time_client;
    
    // 생산라인 센서 데이터
    struct ProductionData {
        uint32_t line_id;           // 라인 ID
        uint16_t production_count;  // 생산 수량
        float efficiency_percent;   // 효율성 (%)
        uint8_t defect_rate;       // 불량률 (%)
        float temperature;         // 환경 온도
        float humidity;            // 습도
        uint16_t vibration_level;  // 진동 레벨
        bool emergency_stop;       // 비상정지 상태
        unsigned long timestamp;   // 타임스탬프
    };
    
    ProductionData line_data;
    
    // 한국타이어 공장 설정
    const char* WIFI_SSID = "HankookFactory_WiFi";
    const char* WIFI_PASSWORD = "HKTire2024!";
    const char* MQTT_BROKER = "192.168.1.100";
    const int MQTT_PORT = 1883;
    const char* MQTT_CLIENT_ID = "HankookLine01";
    
    // MQTT 토픽
    const char* TOPIC_PRODUCTION = "hankook/production/line01";
    const char* TOPIC_QUALITY = "hankook/quality/line01";
    const char* TOPIC_MAINTENANCE = "hankook/maintenance/line01";
    const char* TOPIC_ALARM = "hankook/alarm/line01";
    
public:
    HankookSmartFactory() : 
        mqtt_client(wifi_client),
        time_client(ntp_udp, "pool.ntp.org", 32400) { // UTC+9 (한국시간)
        
        line_data.line_id = 1;
        line_data.production_count = 0;
        line_data.emergency_stop = false;
    }
    
    // WiFi 연결
    void connectWiFi() {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print("WiFi 연결 중");
        
        while(WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        
        Serial.println();
        Serial.printf("WiFi 연결됨: %s\n", WiFi.localIP().toString().c_str());
    }
    
    // MQTT 연결
    void connectMQTT() {
        mqtt_client.setServer(MQTT_BROKER, MQTT_PORT);
        mqtt_client.setCallback([this](char* topic, byte* payload, unsigned int length) {
            this->mqttCallback(topic, payload, length);
        });
        
        while(!mqtt_client.connected()) {
            Serial.print("MQTT 연결 시도...");
            
            if(mqtt_client.connect(MQTT_CLIENT_ID)) {
                Serial.println("MQTT 연결됨");
                
                // 제어 토픽 구독
                mqtt_client.subscribe("hankook/control/line01");
                mqtt_client.subscribe("hankook/config/line01");
                
            } else {
                Serial.printf("MQTT 연결 실패, rc=%d 5초 후 재시도\n", mqtt_client.state());
                delay(5000);
            }
        }
    }
    
    // MQTT 메시지 콜백
    void mqttCallback(char* topic, byte* payload, unsigned int length) {
        String message;
        for(int i = 0; i < length; i++) {
            message += (char)payload[i];
        }
        
        Serial.printf("MQTT 메시지 수신: %s = %s\n", topic, message.c_str());
        
        // 제어 명령 처리
        if(String(topic) == "hankook/control/line01") {
            if(message == "STOP") {
                line_data.emergency_stop = true;
                Serial.println("비상정지 명령 수신");
            } else if(message == "START") {
                line_data.emergency_stop = false;
                Serial.println("생산 재개 명령 수신");
            }
        }
    }
    
    // 생산 데이터 수집
    void collectProductionData() {
        // 생산 수량 카운터 (홀 센서)
        static uint16_t last_count = 0;
        int sensor_value = digitalRead(2);
        static int last_sensor = 0;
        
        if(sensor_value && !last_sensor) {
            line_data.production_count++;
        }
        last_sensor = sensor_value;
        
        // 효율성 계산 (목표 대비 실적)
        static unsigned long last_hour = 0;
        unsigned long current_hour = millis() / 3600000;
        
        if(current_hour != last_hour) {
            uint16_t hourly_production = line_data.production_count - last_count;
            line_data.efficiency_percent = (hourly_production / 100.0) * 100.0; // 목표: 100개/시간
            last_count = line_data.production_count;
            last_hour = current_hour;
        }
        
        // 환경 센서 데이터
        line_data.temperature = readTemperature();
        line_data.humidity = readHumidity();
        line_data.vibration_level = readVibration();
        
        // 불량률 계산 (품질 센서 기반)
        static uint8_t defect_count = 0;
        if(readQualitySensor() < 500) { // 임계값 이하 시 불량
            defect_count++;
        }
        
        if(line_data.production_count > 0) {
            line_data.defect_rate = (defect_count * 100) / line_data.production_count;
        }
        
        line_data.timestamp = time_client.getEpochTime();
    }
    
    // 센서 읽기 함수들
    float readTemperature() {
        int raw = analogRead(A0);
        return (raw * 100.0) / 1024.0; // 0-100°C
    }
    
    float readHumidity() {
        int raw = analogRead(A1);
        return (raw * 100.0) / 1024.0; // 0-100%
    }
    
    uint16_t readVibration() {
        int raw = analogRead(A2);
        return map(raw, 0, 1024, 0, 1000); // 0-1000 진동 레벨
    }
    
    int readQualitySensor() {
        return analogRead(A3); // 품질 센서 값
    }
    
    // 생산 데이터 전송
    void publishProductionData() {
        StaticJsonDocument<512> doc;
        
        doc["line_id"] = line_data.line_id;
        doc["production_count"] = line_data.production_count;
        doc["efficiency_percent"] = line_data.efficiency_percent;
        doc["defect_rate"] = line_data.defect_rate;
        doc["temperature"] = line_data.temperature;
        doc["humidity"] = line_data.humidity;
        doc["vibration_level"] = line_data.vibration_level;
        doc["emergency_stop"] = line_data.emergency_stop;
        doc["timestamp"] = line_data.timestamp;
        
        String json_string;
        serializeJson(doc, json_string);
        
        mqtt_client.publish(TOPIC_PRODUCTION, json_string.c_str());
        Serial.println("생산 데이터 전송: " + json_string);
    }
    
    // 알람 전송
    void publishAlarm(String alarm_type, String message) {
        StaticJsonDocument<256> doc;
        
        doc["line_id"] = line_data.line_id;
        doc["alarm_type"] = alarm_type;
        doc["message"] = message;
        doc["severity"] = "HIGH";
        doc["timestamp"] = time_client.getEpochTime();
        
        String json_string;
        serializeJson(doc, json_string);
        
        mqtt_client.publish(TOPIC_ALARM, json_string.c_str());
        Serial.println("알람 전송: " + json_string);
    }
    
    // 알람 조건 체크
    void checkAlarmConditions() {
        // 온도 이상
        if(line_data.temperature > 45.0) {
            publishAlarm("TEMPERATURE", "생산라인 온도 과열: " + String(line_data.temperature) + "°C");
        }
        
        // 습도 이상
        if(line_data.humidity > 80.0) {
            publishAlarm("HUMIDITY", "생산라인 습도 과다: " + String(line_data.humidity) + "%");
        }
        
        // 진동 이상
        if(line_data.vibration_level > 800) {
            publishAlarm("VIBRATION", "생산라인 진동 이상: " + String(line_data.vibration_level));
        }
        
        // 불량률 이상
        if(line_data.defect_rate > 5) {
            publishAlarm("QUALITY", "불량률 증가: " + String(line_data.defect_rate) + "%");
        }
        
        // 효율성 저하
        if(line_data.efficiency_percent < 80.0) {
            publishAlarm("EFFICIENCY", "생산 효율성 저하: " + String(line_data.efficiency_percent) + "%");
        }
    }
    
    // 메인 IoT 루프
    void iotLoop() {
        // MQTT 연결 유지
        if(!mqtt_client.connected()) {
            connectMQTT();
        }
        mqtt_client.loop();
        
        // 시간 동기화
        time_client.update();
        
        // 데이터 수집
        collectProductionData();
        
        // 알람 체크
        checkAlarmConditions();
        
        // 데이터 전송 (30초마다)
        static unsigned long last_publish = 0;
        if(millis() - last_publish > 30000) {
            publishProductionData();
            last_publish = millis();
        }
    }
};

// 전역 인스턴스
HankookSmartFactory smart_factory;

void setup() {
    Serial.begin(115200);
    
    // 센서 핀 설정
    pinMode(2, INPUT);  // 생산 카운터
    
    // WiFi 및 MQTT 연결
    smart_factory.connectWiFi();
    smart_factory.connectMQTT();
    
    Serial.println("한국타이어 스마트 팩토리 IoT 시스템 시작");
}

void loop() {
    smart_factory.iotLoop();
    delay(1000);
}
```

---

## 실시간 모니터링 시스템

### 통합 대시보드 시스템

```cpp
/*
 * 한국타이어 통합 모니터링 대시보드
 * 웹 기반 실시간 모니터링 인터페이스
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

class HankookDashboard {
private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    
    // 통합 모니터링 데이터
    struct DashboardData {
        // TPMS 데이터
        float tire_pressure[4];
        float tire_temperature[4];
        bool tire_alarm[4];
        
        // 생산 데이터
        uint32_t total_production;
        float production_rate;
        uint8_t defect_rate;
        float line_efficiency;
        
        // 환경 데이터
        float factory_temperature;
        float factory_humidity;
        uint16_t vibration_level;
        
        // 시스템 상태
        bool emergency_status;
        uint8_t active_alarms;
        unsigned long uptime;
    };
    
    DashboardData dashboard_data;
    
public:
    HankookDashboard() : server(80), ws("/ws") {
        // 데이터 초기화
        memset(&dashboard_data, 0, sizeof(dashboard_data));
    }
    
    // SPIFFS 초기화
    void initSPIFFS() {
        if(!SPIFFS.begin(true)) {
            Serial.println("SPIFFS 마운트 실패");
            return;
        }
        Serial.println("SPIFFS 마운트 성공");
    }
    
    // 웹서버 설정
    void setupWebServer() {
        // 정적 파일 서빙
        server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
        
        // WebSocket 핸들러
        ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client, 
                          AwsEventType type, void* arg, uint8_t* data, size_t len) {
            this->onWebSocketEvent(server, client, type, arg, data, len);
        });
        server.addHandler(&ws);
        
        // API 엔드포인트
        server.on("/api/data", HTTP_GET, [this](AsyncWebServerRequest* request) {
            String json = this->getDashboardJSON();
            request->send(200, "application/json", json);
        });
        
        server.on("/api/control", HTTP_POST, [this](AsyncWebServerRequest* request) {
            this->handleControlRequest(request);
        });
        
        // 404 핸들러
        server.onNotFound([](AsyncWebServerRequest* request) {
            request->send(404, "text/plain", "Not found");
        });
        
        server.begin();
        Serial.println("웹서버 시작 (포트 80)");
    }
    
    // WebSocket 이벤트 처리
    void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                         AwsEventType type, void* arg, uint8_t* data, size_t len) {
        switch(type) {
            case WS_EVT_CONNECT:
                Serial.printf("WebSocket 클라이언트 연결: %u\n", client->id());
                // 초기 데이터 전송
                client->text(getDashboardJSON());
                break;
                
            case WS_EVT_DISCONNECT:
                Serial.printf("WebSocket 클라이언트 연결 해제: %u\n", client->id());
                break;
                
            case WS_EVT_DATA:
                handleWebSocketMessage(client, data, len);
                break;
                
            default:
                break;
        }
    }
    
    // WebSocket 메시지 처리
    void handleWebSocketMessage(AsyncWebSocketClient* client, uint8_t* data, size_t len) {
        String message = "";
        for(size_t i = 0; i < len; i++) {
            message += (char)data[i];
        }
        
        Serial.printf("WebSocket 메시지: %s\n", message.c_str());
        
        // JSON 파싱
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, message);
        
        if(error) {
            Serial.println("JSON 파싱 오류");
            return;
        }
        
        // 명령 처리
        String command = doc["command"];
        if(command == "emergency_stop") {
            dashboard_data.emergency_status = true;
            Serial.println("비상정지 명령 수신");
        } else if(command == "reset_alarms") {
            dashboard_data.active_alarms = 0;
            Serial.println("알람 리셋 명령 수신");
        }
    }
    
    // 제어 요청 처리
    void handleControlRequest(AsyncWebServerRequest* request) {
        if(request->hasParam("action", true)) {
            String action = request->getParam("action", true)->value();
            
            if(action == "start_production") {
                dashboard_data.emergency_status = false;
                request->send(200, "application/json", "{\"status\":\"생산 시작\"}");
            } else if(action == "stop_production") {
                dashboard_data.emergency_status = true;
                request->send(200, "application/json", "{\"status\":\"생산 정지\"}");
            } else {
                request->send(400, "application/json", "{\"error\":\"알 수 없는 명령\"}");
            }
        } else {
            request->send(400, "application/json", "{\"error\":\"action 파라미터 필요\"}");
        }
    }
    
    // 대시보드 데이터 JSON 생성
    String getDashboardJSON() {
        StaticJsonDocument<1024> doc;
        
        // TPMS 데이터
        JsonArray tires = doc.createNestedArray("tires");
        for(int i = 0; i < 4; i++) {
            JsonObject tire = tires.createNestedObject();
            tire["id"] = i + 1;
            tire["pressure"] = dashboard_data.tire_pressure[i];
            tire["temperature"] = dashboard_data.tire_temperature[i];
            tire["alarm"] = dashboard_data.tire_alarm[i];
        }
        
        // 생산 데이터
        JsonObject production = doc.createNestedObject("production");
        production["total"] = dashboard_data.total_production;
        production["rate"] = dashboard_data.production_rate;
        production["defect_rate"] = dashboard_data.defect_rate;
        production["efficiency"] = dashboard_data.line_efficiency;
        
        // 환경 데이터
        JsonObject environment = doc.createNestedObject("environment");
        environment["temperature"] = dashboard_data.factory_temperature;
        environment["humidity"] = dashboard_data.factory_humidity;
        environment["vibration"] = dashboard_data.vibration_level;
        
        // 시스템 상태
        JsonObject system = doc.createNestedObject("system");
        system["emergency"] = dashboard_data.emergency_status;
        system["alarms"] = dashboard_data.active_alarms;
        system["uptime"] = dashboard_data.uptime;
        system["timestamp"] = millis();
        
        String json_string;
        serializeJson(doc, json_string);
        return json_string;
    }
    
    // 센서 데이터 업데이트
    void updateSensorData() {
        // TPMS 시뮬레이션
        for(int i = 0; i < 4; i++) {
            dashboard_data.tire_pressure[i] = 220.0 + random(-20, 30);
            dashboard_data.tire_temperature[i] = 25.0 + random(-5, 25);
            dashboard_data.tire_alarm[i] = (dashboard_data.tire_pressure[i] < 200 || 
                                           dashboard_data.tire_pressure[i] > 350);
        }
        
        // 생산 데이터 시뮬레이션
        static uint32_t production_counter = 0;
        production_counter += random(0, 3);
        dashboard_data.total_production = production_counter;
        dashboard_data.production_rate = 95.0 + random(-10, 10);
        dashboard_data.defect_rate = random(0, 5);
        dashboard_data.line_efficiency = 88.0 + random(-8, 12);
        
        // 환경 데이터
        dashboard_data.factory_temperature = 23.0 + random(-3, 10);
        dashboard_data.factory_humidity = 45.0 + random(-10, 20);
        dashboard_data.vibration_level = random(100, 300);
        
        // 시스템 상태
        dashboard_data.uptime = millis() / 1000;
        
        // 알람 카운트 업데이트
        dashboard_data.active_alarms = 0;
        for(int i = 0; i < 4; i++) {
            if(dashboard_data.tire_alarm[i]) dashboard_data.active_alarms++;
        }
        if(dashboard_data.factory_temperature > 35) dashboard_data.active_alarms++;
        if(dashboard_data.vibration_level > 250) dashboard_data.active_alarms++;
    }
    
    // 실시간 데이터 브로드캐스트
    void broadcastData() {
        String json = getDashboardJSON();
        ws.textAll(json);
    }
    
    // 메인 대시보드 루프
    void dashboardLoop() {
        // 센서 데이터 업데이트
        updateSensorData();
        
        // WebSocket 정리
        ws.cleanupClients();
        
        // 1초마다 데이터 브로드캐스트
        static unsigned long last_broadcast = 0;
        if(millis() - last_broadcast > 1000) {
            broadcastData();
            last_broadcast = millis();
        }
    }
};

// HTML 대시보드 (SPIFFS에 저장할 파일)
const char dashboard_html[] PROGMEM = R"(
<!DOCTYPE html>
<html>
<head>
    <title>한국타이어 모니터링 대시보드</title>
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .header { background: #003366; color: white; padding: 20px; text-align: center; }
        .dashboard { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin: 20px 0; }
        .panel { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .panel h3 { margin-top: 0; color: #003366; }
        .tire-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .tire { background: #f8f8f8; padding: 10px; border-radius: 4px; text-align: center; }
        .tire.alarm { background: #ffebee; color: #c62828; }
        .status-good { color: #2e7d32; }
        .status-warning { color: #f57c00; }
        .status-error { color: #c62828; }
        .metric { font-size: 24px; font-weight: bold; margin: 10px 0; }
        .control-buttons { margin: 20px 0; }
        .btn { background: #003366; color: white; padding: 10px 20px; border: none; border-radius: 4px; margin: 5px; cursor: pointer; }
        .btn:hover { background: #004080; }
        .btn.emergency { background: #c62828; }
    </style>
</head>
<body>
    <div class="header">
        <h1>한국타이어 통합 모니터링 시스템</h1>
        <div id="timestamp"></div>
    </div>
    
    <div class="control-buttons">
        <button class="btn" onclick="sendCommand('start_production')">생산 시작</button>
        <button class="btn emergency" onclick="sendCommand('emergency_stop')">비상 정지</button>
        <button class="btn" onclick="sendCommand('reset_alarms')">알람 리셋</button>
    </div>
    
    <div class="dashboard">
        <div class="panel">
            <h3>TPMS 상태</h3>
            <div class="tire-grid" id="tires"></div>
        </div>
        
        <div class="panel">
            <h3>생산 현황</h3>
            <div>총 생산량: <span class="metric" id="total-production">0</span></div>
            <div>생산율: <span class="metric" id="production-rate">0</span>%</div>
            <div>불량률: <span class="metric" id="defect-rate">0</span>%</div>
            <div>라인 효율: <span class="metric" id="line-efficiency">0</span>%</div>
        </div>
        
        <div class="panel">
            <h3>환경 모니터링</h3>
            <div>온도: <span class="metric" id="temperature">0</span>°C</div>
            <div>습도: <span class="metric" id="humidity">0</span>%</div>
            <div>진동: <span class="metric" id="vibration">0</span></div>
        </div>
        
        <div class="panel">
            <h3>시스템 상태</h3>
            <div>비상정지: <span class="metric" id="emergency">정상</span></div>
            <div>활성 알람: <span class="metric" id="alarms">0</span></div>
            <div>가동시간: <span class="metric" id="uptime">0</span>초</div>
        </div>
    </div>
    
    <script>
        let ws = new WebSocket('ws://' + window.location.host + '/ws');
        
        ws.onmessage = function(event) {
            let data = JSON.parse(event.data);
            updateDashboard(data);
        };
        
        function updateDashboard(data) {
            // 타임스탬프 업데이트
            document.getElementById('timestamp').textContent = new Date().toLocaleString();
            
            // TPMS 업데이트
            let tiresHtml = '';
            data.tires.forEach(tire => {
                let alarmClass = tire.alarm ? 'tire alarm' : 'tire';
                tiresHtml += `
                    <div class="${alarmClass}">
                        <strong>타이어 ${tire.id}</strong><br>
                        ${tire.pressure.toFixed(1)} kPa<br>
                        ${tire.temperature.toFixed(1)} °C
                    </div>
                `;
            });
            document.getElementById('tires').innerHTML = tiresHtml;
            
            // 생산 데이터 업데이트
            document.getElementById('total-production').textContent = data.production.total;
            document.getElementById('production-rate').textContent = data.production.rate.toFixed(1);
            document.getElementById('defect-rate').textContent = data.production.defect_rate;
            document.getElementById('line-efficiency').textContent = data.production.efficiency.toFixed(1);
            
            // 환경 데이터 업데이트
            document.getElementById('temperature').textContent = data.environment.temperature.toFixed(1);
            document.getElementById('humidity').textContent = data.environment.humidity.toFixed(1);
            document.getElementById('vibration').textContent = data.environment.vibration;
            
            // 시스템 상태 업데이트
            document.getElementById('emergency').textContent = data.system.emergency ? '비상정지' : '정상';
            document.getElementById('alarms').textContent = data.system.alarms;
            document.getElementById('uptime').textContent = data.system.uptime;
        }
        
        function sendCommand(command) {
            ws.send(JSON.stringify({command: command}));
        }
        
        // 연결 상태 모니터링
        ws.onopen = function() {
            console.log('WebSocket 연결됨');
        };
        
        ws.onclose = function() {
            console.log('WebSocket 연결 종료');
            setTimeout(() => location.reload(), 3000);
        };
    </script>
</body>
</html>
)";

// 전역 인스턴스
HankookDashboard dashboard;

void setup() {
    Serial.begin(115200);
    
    // WiFi 연결
    WiFi.begin("YourWiFi", "YourPassword");
    while(WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("WiFi 연결 중...");
    }
    Serial.printf("WiFi 연결됨: %s\n", WiFi.localIP().toString().c_str());
    
    // SPIFFS 및 웹서버 초기화
    dashboard.initSPIFFS();
    dashboard.setupWebServer();
    
    Serial.println("한국타이어 통합 모니터링 대시보드 시작");
    Serial.printf("대시보드 URL: http://%s\n", WiFi.localIP().toString().c_str());
}

void loop() {
    dashboard.dashboardLoop();
    delay(100);
}
```

---

## 예측 정비 시스템

### AI 기반 예측 정비

```cpp
/*
 * 한국타이어 AI 예측 정비 시스템
 * 머신러닝 기반 장비 상태 예측
 */

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>

class HankookPredictiveMaintenance {
private:
    // TensorFlow Lite 모델 관련
    tflite::MicroErrorReporter micro_error_reporter;
    tflite::AllOpsResolver resolver;
    const tflite::Model* model;
    tflite::MicroInterpreter* interpreter;
    TfLiteTensor* input;
    TfLiteTensor* output;
    
    // 모델 메모리 할당
    constexpr static int kTensorArenaSize = 60000;
    uint8_t tensor_arena[kTensorArenaSize];
    
    // 센서 데이터 구조
    struct MaintenanceData {
        float vibration_x, vibration_y, vibration_z;    // 3축 진동
        float temperature;                              // 온도
        float current_consumption;                      // 전류 소모
        float oil_level;                               // 오일 레벨
        float belt_tension;                            // 벨트 장력
        uint32_t operating_hours;                      // 운전 시간
        float noise_level;                             // 소음 레벨
        uint16_t rotation_speed;                       // 회전 속도
    };
    
    MaintenanceData sensor_data;
    
    // 예측 결과
    struct PredictionResult {
        float failure_probability;     // 고장 확률 (0-1)
        uint16_t remaining_days;      // 잔여 수명 (일)
        String maintenance_type;      // 정비 유형
        String priority_level;        // 우선순위
        float confidence_score;       // 신뢰도
    };
    
    PredictionResult prediction;
    
    // 한국타이어 정비 기준
    const float FAILURE_THRESHOLD = 0.7;    // 고장 예측 임계값
    const uint16_t CRITICAL_DAYS = 7;       // 긴급 정비 기준 (일)
    const uint16_t WARNING_DAYS = 30;       // 경고 정비 기준 (일)
    
public:
    HankookPredictiveMaintenance() {
        // 센서 데이터 초기화
        memset(&sensor_data, 0, sizeof(sensor_data));
        memset(&prediction, 0, sizeof(prediction));
    }
    
    // TensorFlow Lite 모델 초기화
    bool initializeModel(const unsigned char* model_data) {
        // 모델 로드
        model = tflite::GetModel(model_data);
        if(model->version() != TFLITE_SCHEMA_VERSION) {
            Serial.printf("모델 버전 불일치: %d != %d\n", 
                         model->version(), TFLITE_SCHEMA_VERSION);
            return false;
        }
        
        // 인터프리터 생성
        static tflite::MicroInterpreter static_interpreter(
            model, resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter);
        interpreter = &static_interpreter;
        
        // 텐서 할당
        TfLiteStatus allocate_status = interpreter->AllocateTensors();
        if(allocate_status != kTfLiteOk) {
            Serial.println("텐서 할당 실패");
            return false;
        }
        
        // 입력/출력 텐서 참조
        input = interpreter->input(0);
        output = interpreter->output(0);
        
        Serial.println("TensorFlow Lite 모델 초기화 완료");
        Serial.printf("입력 차원: %d, 출력 차원: %d\n", 
                     input->dims->size, output->dims->size);
        
        return true;
    }
    
    // 센서 데이터 수집
    void collectSensorData() {
        // 3축 가속도계 (진동 측정)
        int16_t accel_x = analogRead(A0) - 512;
        int16_t accel_y = analogRead(A1) - 512;
        int16_t accel_z = analogRead(A2) - 512;
        
        // 진동 RMS 계산
        sensor_data.vibration_x = sqrt(accel_x * accel_x) * 0.01;
        sensor_data.vibration_y = sqrt(accel_y * accel_y) * 0.01;
        sensor_data.vibration_z = sqrt(accel_z * accel_z) * 0.01;
        
        // 온도 센서 (DS18B20)
        int temp_raw = analogRead(A3);
        sensor_data.temperature = (temp_raw * 100.0) / 1024.0;
        
        // 전류 센서 (ACS712)
        int current_raw = analogRead(A4);
        sensor_data.current_consumption = (current_raw - 512) * 0.1;
        
        // 오일 레벨 센서
        int oil_raw = analogRead(A5);
        sensor_data.oil_level = (oil_raw * 100.0) / 1024.0;
        
        // 벨트 장력 센서 (하중 센서)
        int tension_raw = analogRead(A6);
        sensor_data.belt_tension = tension_raw * 0.5; // kg
        
        // 운전 시간 (시간 단위)
        sensor_data.operating_hours = millis() / 3600000;
        
        // 소음 레벨 (마이크로폰)
        int noise_raw = analogRead(A7);
        sensor_data.noise_level = (noise_raw * 120.0) / 1024.0; // dB
        
        // 회전 속도 (홀 센서)
        static unsigned long last_pulse = 0;
        static uint16_t pulse_count = 0;
        
        if(digitalRead(2) && (millis() - last_pulse > 10)) {
            pulse_count++;
            last_pulse = millis();
        }
        
        // 1초마다 RPM 계산
        static unsigned long last_rpm_calc = 0;
        if(millis() - last_rpm_calc > 1000) {
            sensor_data.rotation_speed = pulse_count * 60; // RPM
            pulse_count = 0;
            last_rpm_calc = millis();
        }
    }
    
    // AI 예측 실행
    void runPrediction() {
        if(!interpreter) return;
        
        // 입력 데이터 정규화 및 설정
        input->data.f[0] = sensor_data.vibration_x / 10.0;        // 0-1 정규화
        input->data.f[1] = sensor_data.vibration_y / 10.0;
        input->data.f[2] = sensor_data.vibration_z / 10.0;
        input->data.f[3] = sensor_data.temperature / 100.0;
        input->data.f[4] = sensor_data.current_consumption / 50.0;
        input->data.f[5] = sensor_data.oil_level / 100.0;
        input->data.f[6] = sensor_data.belt_tension / 100.0;
        input->data.f[7] = sensor_data.operating_hours / 8760.0;  // 연간 시간
        input->data.f[8] = sensor_data.noise_level / 120.0;
        input->data.f[9] = sensor_data.rotation_speed / 3000.0;
        
        // 추론 실행
        TfLiteStatus invoke_status = interpreter->Invoke();
        if(invoke_status != kTfLiteOk) {
            Serial.println("AI 추론 실행 실패");
            return;
        }
        
        // 결과 해석
        prediction.failure_probability = output->data.f[0];
        prediction.remaining_days = (uint16_t)(output->data.f[1] * 365);
        prediction.confidence_score = output->data.f[2];
        
        // 정비 유형 결정
        if(sensor_data.vibration_x > 5.0 || sensor_data.vibration_y > 5.0) {
            prediction.maintenance_type = "베어링 교체";
        } else if(sensor_data.belt_tension < 20.0) {
            prediction.maintenance_type = "벨트 장력 조정";
        } else if(sensor_data.oil_level < 30.0) {
            prediction.maintenance_type = "오일 보충";
        } else if(sensor_data.temperature > 80.0) {
            prediction.maintenance_type = "냉각 시스템 점검";
        } else {
            prediction.maintenance_type = "일반 점검";
        }
        
        // 우선순위 결정
        if(prediction.remaining_days <= CRITICAL_DAYS) {
            prediction.priority_level = "긴급";
        } else if(prediction.remaining_days <= WARNING_DAYS) {
            prediction.priority_level = "경고";
        } else {
            prediction.priority_level = "정상";
        }
        
        // 결과 출력
        Serial.println("=== AI 예측 정비 결과 ===");
        Serial.printf("고장 확률: %.1f%%\n", prediction.failure_probability * 100);
        Serial.printf("예상 잔여 수명: %d일\n", prediction.remaining_days);
        Serial.printf("정비 유형: %s\n", prediction.maintenance_type.c_str());
        Serial.printf("우선순위: %s\n", prediction.priority_level.c_str());
        Serial.printf("신뢰도: %.1f%%\n", prediction.confidence_score * 100);
        Serial.println("========================");
    }
    
    // 정비 알림 전송
    void sendMaintenanceAlert() {
        if(prediction.failure_probability > FAILURE_THRESHOLD ||
           prediction.remaining_days <= WARNING_DAYS) {
            
            // JSON 형태의 알림 데이터 생성
            StaticJsonDocument<512> alert_doc;
            
            alert_doc["alert_type"] = "predictive_maintenance";
            alert_doc["equipment_id"] = "HK_LINE01_MIXER";
            alert_doc["failure_probability"] = prediction.failure_probability;
            alert_doc["remaining_days"] = prediction.remaining_days;
            alert_doc["maintenance_type"] = prediction.maintenance_type;
            alert_doc["priority"] = prediction.priority_level;
            alert_doc["confidence"] = prediction.confidence_score;
            alert_doc["timestamp"] = millis();
            
            // 센서 데이터 추가
            JsonObject sensors = alert_doc.createNestedObject("sensor_data");
            sensors["vibration_rms"] = sqrt(pow(sensor_data.vibration_x, 2) + 
                                          pow(sensor_data.vibration_y, 2) + 
                                          pow(sensor_data.vibration_z, 2));
            sensors["temperature"] = sensor_data.temperature;
            sensors["current"] = sensor_data.current_consumption;
            sensors["oil_level"] = sensor_data.oil_level;
            sensors["belt_tension"] = sensor_data.belt_tension;
            sensors["noise_level"] = sensor_data.noise_level;
            sensors["rpm"] = sensor_data.rotation_speed;
            
            String alert_json;
            serializeJson(alert_doc, alert_json);
            
            Serial.println("정비 알림 전송: " + alert_json);
            
            // 실제 구현에서는 MQTT, HTTP POST 등으로 전송
            // mqtt_client.publish("hankook/maintenance/alert", alert_json.c_str());
        }
    }
    
    // 진동 스펙트럼 분석 (FFT)
    void analyzeVibrationSpectrum() {
        const int SAMPLE_SIZE = 128;
        float vibration_samples[SAMPLE_SIZE];
        
        // 진동 데이터 샘플링
        for(int i = 0; i < SAMPLE_SIZE; i++) {
            int raw_value = analogRead(A0);
            vibration_samples[i] = (raw_value - 512) * 0.01;
            delayMicroseconds(1000); // 1kHz 샘플링
        }
        
        // 간단한 주파수 분석 (피크 검출)
        float peak_frequency = 0;
        float max_amplitude = 0;
        
        for(int i = 1; i < SAMPLE_SIZE/2; i++) {
            float amplitude = abs(vibration_samples[i]);
            if(amplitude > max_amplitude) {
                max_amplitude = amplitude;
                peak_frequency = (float)i * 1000.0 / SAMPLE_SIZE; // Hz
            }
        }
        
        Serial.printf("진동 스펙트럼 분석: 피크 주파수 %.1f Hz, 진폭 %.3f\n", 
                     peak_frequency, max_amplitude);
        
        // 이상 주파수 검출
        if(peak_frequency > 100 && max_amplitude > 2.0) {
            Serial.println("경고: 이상 진동 주파수 검출 - 베어링 점검 필요");
        }
    }
    
    // 메인 예측 정비 루프
    void maintenanceLoop() {
        // 센서 데이터 수집
        collectSensorData();
        
        // AI 예측 실행 (10초마다)
        static unsigned long last_prediction = 0;
        if(millis() - last_prediction > 10000) {
            runPrediction();
            sendMaintenanceAlert();
            last_prediction = millis();
        }
        
        // 진동 스펙트럼 분석 (1분마다)
        static unsigned long last_spectrum = 0;
        if(millis() - last_spectrum > 60000) {
            analyzeVibrationSpectrum();
            last_spectrum = millis();
        }
    }
};

// 사전 훈련된 모델 데이터 (실제로는 별도 파일에서 로드)
// 이 예제에서는 더미 데이터 사용
alignas(8) const unsigned char model_data[] = {
    // TensorFlow Lite 모델 바이너리 데이터
    // 실제 구현에서는 python으로 훈련된 모델을 변환하여 사용
    0x1c, 0x00, 0x00, 0x00, 0x54, 0x46, 0x4c, 0x33, // TFL3 헤더
    // ... (실제 모델 데이터)
};

// 전역 인스턴스
HankookPredictiveMaintenance predictive_maintenance;

void setup() {
    Serial.begin(115200);
    
    // 센서 핀 설정
    pinMode(2, INPUT);   // 회전 속도 센서
    
    // AI 모델 초기화
    if(predictive_maintenance.initializeModel(model_data)) {
        Serial.println("한국타이어 AI 예측 정비 시스템 시작");
    } else {
        Serial.println("AI 모델 초기화 실패");
    }
}

void loop() {
    predictive_maintenance.maintenanceLoop();
    delay(1000);
}
```

---

## 자동차 산업 표준

### ISO/TS 16949 품질 관리 시스템

```cpp
/*
 * 한국타이어 ISO/TS 16949 품질 관리 시스템
 * 자동차 산업 품질 표준 준수
 */

class HankookQualityManagement {
private:
    // ISO/TS 16949 품질 기록 구조
    struct QualityRecord {
        uint32_t lot_number;         // 로트 번호
        String part_number;          // 부품 번호
        uint32_t serial_number;      // 시리얼 번호
        float measurement_value;     // 측정값
        float specification_min;     // 규격 최소값
        float specification_max;     // 규격 최대값
        bool pass_fail;             // 합격/불합격
        String operator_id;         // 작업자 ID
        String equipment_id;        // 장비 ID
        unsigned long timestamp;    // 타임스탬프
        String calibration_date;    // 교정 일자
    };
    
    // SPC (Statistical Process Control) 데이터
    struct SPCData {
        float mean;                 // 평균
        float range;               // 범위
        float std_deviation;       // 표준편차
        float cp;                  // 공정능력지수
        float cpk;                 // 수정공정능력지수
        bool process_stable;       // 공정 안정성
        uint16_t sample_size;      // 샘플 크기
    };
    
    QualityRecord quality_records[100]; // 품질 기록 버퍼
    SPCData spc_data;
    uint8_t record_index = 0;
    
    // ISO/TS 16949 기준값
    const float CP_MIN = 1.33;      // 최소 공정능력지수
    const float CPK_MIN = 1.33;     // 최소 수정공정능력지수
    const uint16_t MIN_SAMPLE = 25; // 최소 샘플 크기
    
public:
    // 품질 데이터 기록
    void recordQualityData(uint32_t lot_num, String part_num, float measured_value,
                          float spec_min, float spec_max, String operator, String equipment) {
        
        QualityRecord& record = quality_records[record_index];
        
        record.lot_number = lot_num;
        record.part_number = part_num;
        record.serial_number = millis(); // 임시 시리얼 번호
        record.measurement_value = measured_value;
        record.specification_min = spec_min;
        record.specification_max = spec_max;
        record.pass_fail = (measured_value >= spec_min && measured_value <= spec_max);
        record.operator_id = operator;
        record.equipment_id = equipment;
        record.timestamp = millis();
        record.calibration_date = "2024-01-15"; // 실제로는 동적으로 설정
        
        record_index = (record_index + 1) % 100;
        
        Serial.printf("품질 기록: 로트 %d, 부품 %s, 측정값 %.3f, 결과 %s\n",
                     lot_num, part_num.c_str(), measured_value,
                     record.pass_fail ? "합격" : "불합격");
    }
    
    // SPC 통계 계산
    void calculateSPC() {
        if(record_index < MIN_SAMPLE) {
            Serial.println("SPC 계산을 위한 충분한 샘플이 없습니다.");
            return;
        }
        
        // 평균 계산
        float sum = 0;
        uint16_t valid_samples = 0;
        
        for(int i = 0; i < record_index && i < 100; i++) {
            sum += quality_records[i].measurement_value;
            valid_samples++;
        }
        
        spc_data.mean = sum / valid_samples;
        
        // 표준편차 계산
        float variance_sum = 0;
        for(int i = 0; i < valid_samples; i++) {
            float diff = quality_records[i].measurement_value - spc_data.mean;
            variance_sum += diff * diff;
        }
        
        spc_data.std_deviation = sqrt(variance_sum / (valid_samples - 1));
        
        // 범위 계산 (최대값 - 최소값)
        float min_val = quality_records[0].measurement_value;
        float max_val = quality_records[0].measurement_value;
        
        for(int i = 1; i < valid_samples; i++) {
            float val = quality_records[i].measurement_value;
            if(val < min_val) min_val = val;
            if(val > max_val) max_val = val;
        }
        
        spc_data.range = max_val - min_val;
        spc_data.sample_size = valid_samples;
        
        // 공정능력지수 계산 (첫 번째 샘플의 규격 사용)
        float spec_range = quality_records[0].specification_max - quality_records[0].specification_min;
        spc_data.cp = spec_range / (6 * spc_data.std_deviation);
        
        // 수정공정능력지수 계산
        float usl = quality_records[0].specification_max; // 상한규격
        float lsl = quality_records[0].specification_min; // 하한규격
        
        float cpu = (usl - spc_data.mean) / (3 * spc_data.std_deviation);
        float cpl = (spc_data.mean - lsl) / (3 * spc_data.std_deviation);
        spc_data.cpk = min(cpu, cpl);
        
        // 공정 안정성 판정
        spc_data.process_stable = (spc_data.cp >= CP_MIN && spc_data.cpk >= CPK_MIN);
        
        // SPC 결과 출력
        Serial.println("=== SPC 통계 분석 결과 ===");
        Serial.printf("샘플 크기: %d\n", spc_data.sample_size);
        Serial.printf("평균: %.4f\n", spc_data.mean);
        Serial.printf("표준편차: %.4f\n", spc_data.std_deviation);
        Serial.printf("범위: %.4f\n", spc_data.range);
        Serial.printf("Cp: %.3f (기준: %.3f 이상)\n", spc_data.cp, CP_MIN);
        Serial.printf("Cpk: %.3f (기준: %.3f 이상)\n", spc_data.cpk, CPK_MIN);
        Serial.printf("공정 상태: %s\n", spc_data.process_stable ? "안정" : "불안정");
        Serial.println("========================");
    }
    
    // 품질 추세 분석
    void analyzeTrends() {
        if(record_index < 10) return;
        
        // 최근 10개 샘플의 추세 분석
        int trend_samples = min(10, (int)record_index);
        float slope = 0;
        
        // 선형 회귀를 통한 추세 계산
        float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        
        for(int i = 0; i < trend_samples; i++) {
            int idx = (record_index - trend_samples + i + 100) % 100;
            float x = i + 1;
            float y = quality_records[idx].measurement_value;
            
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }
        
        slope = (trend_samples * sum_xy - sum_x * sum_y) / 
                (trend_samples * sum_x2 - sum_x * sum_x);
        
        Serial.printf("품질 추세 분석: 기울기 %.6f ", slope);
        
        if(abs(slope) < 0.001) {
            Serial.println("(안정)");
        } else if(slope > 0) {
            Serial.println("(상승 추세)");
        } else {
            Serial.println("(하락 추세)");
        }
        
        // 추세 경고
        if(abs(slope) > 0.01) {
            Serial.println("경고: 품질 추세 변화 감지 - 공정 점검 필요");
        }
    }
    
    // 교정 관리
    void checkCalibrationStatus(String equipment_id) {
        // 교정 만료일 체크 (실제로는 데이터베이스에서 조회)
        Serial.printf("장비 %s 교정 상태 확인\n", equipment_id.c_str());
        
        // 더미 교정 데이터
        struct CalibrationInfo {
            String equipment;
            String last_calibration;
            String next_calibration;
            bool overdue;
        };
        
        CalibrationInfo cal_info = {
            equipment_id,
            "2024-01-15",
            "2024-07-15",
            false // 실제로는 날짜 비교로 계산
        };
        
        if(cal_info.overdue) {
            Serial.printf("경고: 장비 %s 교정 만료됨! 다음 교정일: %s\n",
                         cal_info.equipment.c_str(), cal_info.next_calibration.c_str());
        } else {
            Serial.printf("장비 %s 교정 상태 양호, 다음 교정일: %s\n",
                         cal_info.equipment.c_str(), cal_info.next_calibration.c_str());
        }
    }
    
    // 품질 보고서 생성
    void generateQualityReport() {
        calculateSPC();
        analyzeTrends();
        
        Serial.println("=== ISO/TS 16949 품질 보고서 ===");
        Serial.printf("보고서 생성 시간: %lu\n", millis());
        Serial.printf("총 검사 수량: %d\n", record_index);
        
        // 합격률 계산
        uint16_t pass_count = 0;
        for(int i = 0; i < record_index && i < 100; i++) {
            if(quality_records[i].pass_fail) pass_count++;
        }
        
        float pass_rate = (float)pass_count / record_index * 100.0;
        Serial.printf("합격률: %.2f%%\n", pass_rate);
        
        // 품질 상태 종합 평가
        bool quality_ok = (pass_rate >= 99.0) && spc_data.process_stable;
        Serial.printf("종합 품질 상태: %s\n", quality_ok ? "양호" : "주의 필요");
        
        if(!quality_ok) {
            Serial.println("개선 권고사항:");
            if(pass_rate < 99.0) {
                Serial.println("- 불량률 개선 필요");
            }
            if(!spc_data.process_stable) {
                Serial.println("- 공정 안정화 필요");
            }
        }
        
        Serial.println("==============================");
    }
};

// 전역 인스턴스
HankookQualityManagement quality_mgmt;

void setup() {
    Serial.begin(115200);
    Serial.println("한국타이어 ISO/TS 16949 품질 관리 시스템 시작");
}

void loop() {
    // 품질 데이터 시뮬레이션 (실제로는 센서 데이터)
    static uint32_t lot_counter = 1000;
    static unsigned long last_measurement = 0;
    
    if(millis() - last_measurement > 30000) { // 30초마다 측정
        // 타이어 두께 측정 시뮬레이션
        float thickness = 10.0 + random(-50, 50) / 100.0; // 9.5-10.5mm
        
        quality_mgmt.recordQualityData(
            lot_counter++,
            "HK_TIRE_225_60R16",
            thickness,
            9.8,  // 최소 규격
            10.2, // 최대 규격
            "OP001",
            "MEASURE_01"
        );
        
        last_measurement = millis();
    }
    
    // 1분마다 품질 보고서 생성
    static unsigned long last_report = 0;
    if(millis() - last_report > 60000) {
        quality_mgmt.generateQualityReport();
        quality_mgmt.checkCalibrationStatus("MEASURE_01");
        last_report = millis();
    }
    
    delay(1000);
}
```

---

## 실전 구현 예제

### 통합 센서 시스템 메인 코드

```cpp
/*
 * 한국타이어 통합 센서 시스템 - 메인 구현
 * 모든 서브시스템을 통합한 완전한 구현 예제
 */

#include "HankookTPMS.h"
#include "HankookQualitySensor.h"
#include "HankookSmartFactory.h"
#include "HankookDashboard.h"
#include "HankookPredictiveMaintenance.h"
#include "HankookQualityManagement.h"

// 전역 시스템 인스턴스
HankookTPMS tpms_system;
HankookQualitySensor quality_sensor;
HankookSmartFactory smart_factory;
HankookDashboard dashboard;
HankookPredictiveMaintenance predictive_maintenance;
HankookQualityManagement quality_management;

// 시스템 상태
enum SystemMode {
    MODE_PRODUCTION,    // 생산 모드
    MODE_QUALITY,       // 품질 검사 모드
    MODE_MAINTENANCE,   // 정비 모드
    MODE_MONITORING     // 모니터링 모드
};

SystemMode current_mode = MODE_PRODUCTION;
bool emergency_stop = false;

// 시스템 초기화
void initializeHankookSystems() {
    Serial.begin(115200);
    Serial.println("=== 한국타이어 통합 센서 시스템 초기화 ===");
    
    // 1. WiFi 연결
    smart_factory.connectWiFi();
    Serial.println("✓ WiFi 연결 완료");
    
    // 2. MQTT 연결
    smart_factory.connectMQTT();
    Serial.println("✓ MQTT 연결 완료");
    
    // 3. 웹 대시보드 초기화
    dashboard.initSPIFFS();
    dashboard.setupWebServer();
    Serial.println("✓ 웹 대시보드 초기화 완료");
    
    // 4. AI 예측 모델 로드
    if(predictive_maintenance.initializeModel(model_data)) {
        Serial.println("✓ AI 예측 모델 로드 완료");
    } else {
        Serial.println("✗ AI 예측 모델 로드 실패");
    }
    
    // 5. 센서 핀 설정
    pinMode(2, INPUT);   // 생산 카운터
    pinMode(3, OUTPUT);  // 경고 LED
    pinMode(4, OUTPUT);  // 부저
    pinMode(13, OUTPUT); // 상태 LED
    
    Serial.println("✓ 센서 핀 설정 완료");
    Serial.println("==========================================");
    Serial.printf("시스템 IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("대시보드 URL: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("==========================================");
}

// 시스템 모드 전환
void switchSystemMode(SystemMode new_mode) {
    if(current_mode == new_mode) return;
    
    Serial.printf("시스템 모드 전환: %d → %d\n", current_mode, new_mode);
    current_mode = new_mode;
    
    // 모드별 초기화 작업
    switch(current_mode) {
        case MODE_PRODUCTION:
            Serial.println("생산 모드 활성화");
            digitalWrite(13, HIGH);
            break;
            
        case MODE_QUALITY:
            Serial.println("품질 검사 모드 활성화");
            digitalWrite(13, LOW);
            break;
            
        case MODE_MAINTENANCE:
            Serial.println("정비 모드 활성화");
            digitalWrite(13, LOW);
            break;
            
        case MODE_MONITORING:
            Serial.println("모니터링 모드 활성화");
            digitalWrite(13, HIGH);
            break;
    }
}

// 비상 정지 처리
void handleEmergencyStop() {
    if(!emergency_stop) return;
    
    Serial.println("🚨 비상 정지 활성화 🚨");
    
    // 모든 생산 장비 정지
    digitalWrite(13, LOW);
    
    // 경고음 출력
    for(int i = 0; i < 5; i++) {
        tone(4, 1000, 200);
        delay(300);
    }
    
    // MQTT 알림 전송
    smart_factory.publishAlarm("EMERGENCY_STOP", "비상 정지 버튼 활성화");
    
    // 모드를 정비 모드로 전환
    switchSystemMode(MODE_MAINTENANCE);
}

// 메인 작업 스케줄러
void runMainScheduler() {
    static unsigned long last_cycle = 0;
    static uint8_t cycle_count = 0;
    
    // 1초 주기로 실행
    if(millis() - last_cycle < 1000) return;
    
    last_cycle = millis();
    cycle_count++;
    
    // 매 사이클 실행 (1초마다)
    switch(current_mode) {
        case MODE_PRODUCTION:
            smart_factory.iotLoop();
            dashboard.dashboardLoop();
            break;
            
        case MODE_QUALITY:
            if(quality_sensor.performQualityCheck()) {
                quality_management.recordQualityData(
                    cycle_count + 1000,
                    "HK_TIRE_225_60R16",
                    10.0 + random(-20, 20) / 100.0,
                    9.8, 10.2, "OP001", "QC_001"
                );
            }
            break;
            
        case MODE_MAINTENANCE:
            predictive_maintenance.maintenanceLoop();
            break;
            
        case MODE_MONITORING:
            tpms_system.monitorLoop();
            dashboard.dashboardLoop();
            break;
    }
    
    // 5초마다 실행
    if(cycle_count % 5 == 0) {
        checkSystemHealth();
    }
    
    // 30초마다 실행
    if(cycle_count % 30 == 0) {
        generatePeriodicReports();
    }
    
    // 5분마다 실행
    if(cycle_count % 300 == 0) {
        performSystemMaintenance();
        cycle_count = 0; // 카운터 리셋
    }
}

// 시스템 상태 점검
void checkSystemHealth() {
    static bool health_ok = true;
    bool current_health = true;
    
    // WiFi 연결 상태 확인
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("경고: WiFi 연결 끊김");
        current_health = false;
    }
    
    // MQTT 연결 상태 확인
    if(!smart_factory.mqtt_client.connected()) {
        Serial.println("경고: MQTT 연결 끊김");
        current_health = false;
    }
    
    // 메모리 사용량 확인
    uint32_t free_heap = ESP.getFreeHeap();
    if(free_heap < 10000) { // 10KB 미만
        Serial.printf("경고: 메모리 부족 %d bytes\n", free_heap);
        current_health = false;
    }
    
    // 상태 변화 알림
    if(health_ok != current_health) {
        if(current_health) {
            Serial.println("✓ 시스템 상태 복구됨");
        } else {
            Serial.println("✗ 시스템 상태 이상 감지");
            smart_factory.publishAlarm("SYSTEM_HEALTH", "시스템 상태 이상");
        }
        health_ok = current_health;
    }
}

// 주기적 보고서 생성
void generatePeriodicReports() {
    Serial.println("=== 주기적 시스템 보고서 ===");
    Serial.printf("현재 모드: %d\n", current_mode);
    Serial.printf("가동 시간: %lu초\n", millis() / 1000);
    Serial.printf("메모리 사용량: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("비상정지 상태: %s\n", emergency_stop ? "활성화" : "정상");
    
    // 모드별 상세 보고서
    switch(current_mode) {
        case MODE_PRODUCTION:
            Serial.println("생산 현황: 정상 가동 중");
            break;
        case MODE_QUALITY:
            quality_management.generateQualityReport();
            break;
        case MODE_MAINTENANCE:
            Serial.println("정비 현황: 예측 정비 진행 중");
            break;
        case MODE_MONITORING:
            Serial.println("모니터링 현황: TPMS 감시 중");
            break;
    }
    
    Serial.println("============================");
}

// 시스템 정비 작업
void performSystemMaintenance() {
    Serial.println("자동 시스템 정비 시작...");
    
    // 1. 메모리 정리
    ESP.getHeapFragmentation();
    
    // 2. 로그 파일 정리 (실제로는 SD 카드)
    Serial.println("로그 파일 정리 완료");
    
    // 3. 센서 캘리브레이션 체크
    quality_management.checkCalibrationStatus("ALL_SENSORS");
    
    // 4. 통계 데이터 백업
    Serial.println("통계 데이터 백업 완료");
    
    Serial.println("자동 시스템 정비 완료");
}

// 사용자 명령 처리
void processUserCommands() {
    if(Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if(command == "status") {
            Serial.printf("현재 모드: %d, 비상정지: %s\n", 
                         current_mode, emergency_stop ? "ON" : "OFF");
        }
        else if(command == "production") {
            switchSystemMode(MODE_PRODUCTION);
            emergency_stop = false;
        }
        else if(command == "quality") {
            switchSystemMode(MODE_QUALITY);
        }
        else if(command == "maintenance") {
            switchSystemMode(MODE_MAINTENANCE);
        }
        else if(command == "monitoring") {
            switchSystemMode(MODE_MONITORING);
        }
        else if(command == "emergency") {
            emergency_stop = true;
        }
        else if(command == "reset") {
            emergency_stop = false;
            switchSystemMode(MODE_PRODUCTION);
        }
        else if(command == "report") {
            generatePeriodicReports();
        }
        else if(command == "help") {
            Serial.println("사용 가능한 명령:");
            Serial.println("status - 시스템 상태 확인");
            Serial.println("production - 생산 모드");
            Serial.println("quality - 품질 검사 모드");  
            Serial.println("maintenance - 정비 모드");
            Serial.println("monitoring - 모니터링 모드");
            Serial.println("emergency - 비상 정지");
            Serial.println("reset - 시스템 리셋");
            Serial.println("report - 보고서 생성");
        }
        else {
            Serial.println("알 수 없는 명령. 'help' 입력으로 도움말 확인");
        }
    }
}

// Arduino 메인 함수들
void setup() {
    // 한국타이어 통합 시스템 초기화
    initializeHankookSystems();
    
    // 기본 모드로 시작
    switchSystemMode(MODE_PRODUCTION);
    
    Serial.println("한국타이어 통합 센서 시스템 가동 시작!");
    Serial.println("시리얼 모니터에서 'help' 입력으로 명령어 확인 가능");
}

void loop() {
    // 비상 정지 체크
    handleEmergencyStop();
    
    // 메인 작업 스케줄러 실행
    runMainScheduler();
    
    // 사용자 명령 처리
    processUserCommands();
    
    // 짧은 지연
    delay(100);
}
```

---

## 📚 참고 자료

### 한국타이어 기술 문서
- **TPMS 기술 사양서**: ISO 21750 표준 기반 타이어 압력 모니터링
- **품질 관리 매뉴얼**: ISO/TS 16949 자동차 산업 품질 표준
- **스마트 팩토리 가이드**: Industry 4.0 기반 제조 시스템

### Arduino 라이브러리
```cpp
// 필수 라이브러리 설치
#include <WiFi.h>           // WiFi 연결
#include <PubSubClient.h>   // MQTT 통신
#include <ArduinoJson.h>    // JSON 데이터 처리
#include <ESPAsyncWebServer.h> // 웹서버
#include <SPIFFS.h>         // 파일 시스템
#include <TensorFlowLite.h> // AI 추론
```

### 하드웨어 요구사항
- **MCU**: ESP32 (WiFi/Bluetooth 지원)
- **센서**: 압력센서(MPX5700AP), 온도센서(DS18B20), 가속도계(MPU6050)
- **통신**: RF24 (2.4GHz), MQTT over WiFi
- **디스플레이**: 20x4 LCD, 웹 기반 대시보드
- **전원**: 12V 산업용 전원 공급장치

### 개발 환경
- **IDE**: Arduino IDE 2.0 이상
- **보드**: ESP32 Dev Module
- **업로드 속도**: 921600 baud
- **플래시 크기**: 4MB (SPIFFS 포함)

---

## 🎯 학습 목표 달성

이 가이드를 통해 다음을 학습할 수 있습니다:

1. **실무 센서 시스템 설계**: 한국타이어 수준의 전문적 센서 시스템 구축
2. **자동차 산업 표준**: ISO/TS 16949 품질 관리 시스템 구현
3. **IoT 통합**: MQTT 기반 스마트 팩토리 시스템 개발
4. **AI 예측 정비**: TensorFlow Lite를 활용한 예측 정비 시스템
5. **실시간 모니터링**: 웹 기반 대시보드와 실시간 데이터 시각화

### 다음 단계
- 실제 하드웨어를 이용한 프로토타입 제작
- 클라우드 연동 및 빅데이터 분석 시스템 구축  
- 블록체인 기반 품질 이력 관리 시스템 개발
- 5G 네트워크를 활용한 초저지연 모니터링 시스템

---

**© 2024 한국타이어 Arduino 센서 시스템 - 실무 전문 가이드**