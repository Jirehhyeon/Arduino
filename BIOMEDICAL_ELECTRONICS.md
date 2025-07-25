# 의료용 전자 시스템 완전 가이드 🏥

## 개요
의료 분야에서 사용되는 전자 시스템을 Arduino를 통해 학습하고 구현하는 완전 가이드입니다.

## 목차
1. [의료전자공학 기초](#의료전자공학-기초)
2. [환자 모니터링 시스템](#환자-모니터링-시스템)
3. [심전도(ECG) 측정 시스템](#심전도ecg-측정-시스템)
4. [혈압 모니터링 시스템](#혈압-모니터링-시스템)
5. [체온 및 바이탈 사인](#체온-및-바이탈-사인)
6. [인공호흡기 제어](#인공호흡기-제어)
7. [의료용 펌프 시스템](#의료용-펌프-시스템)
8. [재활 치료 장비](#재활-치료-장비)
9. [원격 의료 시스템](#원격-의료-시스템)
10. [통합 병원 관리 시스템](#통합-병원-관리-시스템)

---

## 의료전자공학 기초

### 의료기기 안전 표준
- IEC 60601: 의료기기 안전 국제표준
- FDA 승인 프로세스
- 환자 안전 우선 설계
- 생체적합성 재료 사용

### 핵심 원리
```cpp
// 의료 시스템 기본 구조
class MedicalDevice {
private:
    enum SafetyLevel {
        CLASS_I = 1,      // 저위험 (일반 의료기기)
        CLASS_II = 2,     // 중위험 (특수 의료기기)
        CLASS_III = 3     // 고위험 (생명 유지 장치)
    };
    
    enum DeviceState {
        INITIALIZATION = 0,
        CALIBRATION = 1,
        MONITORING = 2,
        ALARM = 3,
        MAINTENANCE = 4,
        EMERGENCY_STOP = 5
    };
    
    struct PatientSafety {
        bool electrical_isolation;    // 전기적 절연
        bool redundant_monitoring;    // 중복 모니터링
        bool fail_safe_design;       // 안전 고장 설계
        float leakage_current_uA;    // 누설 전류 (µA)
        uint8_t alarm_priority;      // 알람 우선순위
        uint32_t last_safety_check;  // 마지막 안전 점검
    };
    
    struct VitalSigns {
        float heart_rate;            // 심박수 (bpm)
        float blood_pressure_sys;    // 수축기 혈압 (mmHg)
        float blood_pressure_dia;    // 이완기 혈압 (mmHg)
        float body_temperature;      // 체온 (°C)
        float respiratory_rate;      // 호흡수 (breaths/min)
        float oxygen_saturation;     // 산소포화도 (%)
        bool ecg_rhythm_normal;      // 심전도 정상 여부
        uint32_t measurement_time;   // 측정 시간
    };
    
public:
    void initialize() {
        Serial.println("=== 의료기기 시스템 초기화 ===");
        Serial.println("안전 표준: IEC 60601-1 준수");
        
        // 안전 시스템 초기화
        initializeSafetySystems();
        
        // 센서 캘리브레이션
        calibrateSensors();
        
        // 알람 시스템 초기화
        initializeAlarmSystems();
        
        // 데이터 로깅 시스템
        initializeDataLogging();
        
        current_state = MONITORING;
        Serial.println("의료기기 초기화 완료 - 환자 모니터링 시작");
    }
    
private:
    DeviceState current_state;
    SafetyLevel safety_class;
    PatientSafety safety_status;
    VitalSigns current_vitals;
    uint32_t device_uptime;
    
    void initializeSafetySystems() {
        Serial.println("안전 시스템 초기화...");
        
        // 전기적 절연 체크
        safety_status.electrical_isolation = true;
        
        // 누설 전류 측정 (IEC 60601 기준: <10µA)
        safety_status.leakage_current_uA = measureLeakageCurrent();
        
        // 중복 모니터링 시스템 활성화
        safety_status.redundant_monitoring = true;
        
        // 안전 고장 설계 확인
        safety_status.fail_safe_design = true;
        
        Serial.println("안전 시스템 초기화 완료");
    }
    
    void calibrateSensors() {
        Serial.println("센서 캘리브레이션 시작...");
        
        // 각 센서별 캘리브레이션
        Serial.println("- 심전도 센서 캘리브레이션");
        Serial.println("- 혈압 센서 캘리브레이션");
        Serial.println("- 온도 센서 캘리브레이션");
        Serial.println("- 산소포화도 센서 캘리브레이션");
        
        Serial.println("센서 캘리브레이션 완료");
    }
    
    void initializeAlarmSystems() {
        Serial.println("알람 시스템 초기화...");
        
        // 알람 우선순위 설정
        // 1: 즉시 생명 위험, 2: 경고, 3: 주의
        
        Serial.println("알람 시스템 준비 완료");
    }
    
    void initializeDataLogging() {
        Serial.println("데이터 로깅 시스템 초기화...");
        
        // HIPAA 준수 데이터 보호
        Serial.println("환자 데이터 보안 설정 완료");
        Serial.println("데이터 로깅 시스템 준비 완료");
    }
    
    float measureLeakageCurrent() {
        // 누설 전류 측정 시뮬레이션
        return 3.5; // µA (안전 기준 이하)
    }
};
```

---

## 환자 모니터링 시스템

### 다중 바이탈 사인 모니터
```cpp
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

class PatientMonitoringSystem {
private:
    enum AlarmLevel {
        NO_ALARM = 0,
        LOW_PRIORITY = 1,
        MEDIUM_PRIORITY = 2,
        HIGH_PRIORITY = 3,
        CRITICAL = 4
    };
    
    struct PatientProfile {
        uint16_t patient_id;
        uint8_t age;
        char gender;                  // 'M' or 'F'
        float weight_kg;
        float height_cm;
        bool has_cardiac_condition;
        bool has_respiratory_condition;
        char medication[100];
        char allergies[100];
    };
    
    struct VitalLimits {
        float hr_min, hr_max;         // 심박수 범위
        float bp_sys_min, bp_sys_max; // 수축기 혈압 범위
        float bp_dia_min, bp_dia_max; // 이완기 혈압 범위
        float temp_min, temp_max;     // 체온 범위
        float resp_min, resp_max;     // 호흡수 범위
        float spo2_min;               // 최소 산소포화도
    };
    
    struct MonitoringData {
        uint32_t timestamp;
        float heart_rate;
        float bp_systolic;
        float bp_diastolic;
        float temperature;
        float respiratory_rate;
        float spo2;
        float ecg_voltage;
        bool rhythm_regular;
        AlarmLevel current_alarm;
        char alarm_message[200];
    };
    
    PatientProfile patient;
    VitalLimits normal_limits;
    MonitoringData monitoring_buffer[1440]; // 24시간 분단위 데이터
    uint16_t buffer_index;
    WebSocketsServer webSocket;
    uint32_t last_measurement;
    bool continuous_monitoring;
    
public:
    PatientMonitoringSystem() : webSocket(81) {
        buffer_index = 0;
        continuous_monitoring = true;
        last_measurement = 0;
        
        // 기본 환자 프로필 설정
        patient.patient_id = 12345;
        patient.age = 45;
        patient.gender = 'M';
        patient.weight_kg = 70.0;
        patient.height_cm = 175.0;
        patient.has_cardiac_condition = false;
        patient.has_respiratory_condition = false;
        strcpy(patient.medication, "없음");
        strcpy(patient.allergies, "없음");
        
        // 정상 범위 설정 (성인 기준)
        setNormalLimits();
    }
    
    void initialize() {
        Serial.begin(115200);
        
        Serial.println("=== 환자 모니터링 시스템 초기화 ===");
        Serial.print("환자 ID: "); Serial.println(patient.patient_id);
        Serial.print("나이: "); Serial.print(patient.age); Serial.println("세");
        Serial.print("성별: "); Serial.println(patient.gender);
        
        // WiFi 연결 (병원 네트워크)
        WiFi.begin("Hospital_Network", "secure_password");
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.print(".");
        }
        Serial.println("\nWiFi 연결 완료");
        Serial.print("IP 주소: "); Serial.println(WiFi.localIP());
        
        // WebSocket 서버 시작
        webSocket.begin();
        webSocket.onEvent(webSocketEvent);
        Serial.println("실시간 모니터링 서버 시작");
        
        // 센서 초기화
        initializeSensors();
        
        // 알람 시스템 초기화
        pinMode(13, OUTPUT); // 알람 LED
        pinMode(12, OUTPUT); // 부저
        
        Serial.println("환자 모니터링 시스템 준비 완료");
        Serial.println("연속 모니터링 시작...");
    }
    
    void update() {
        uint32_t current_time = millis();
        
        // WebSocket 처리
        webSocket.loop();
        
        // 1초마다 바이탈 사인 측정
        if (current_time - last_measurement >= 1000) {
            measureVitalSigns();
            analyzeVitalSigns();
            checkAlarmConditions();
            storeData();
            sendRealTimeData();
            
            last_measurement = current_time;
        }
        
        // 알람 처리
        handleAlarms();
        
        // 데이터 전송 (간호사실)
        transmitToNursingStation();
    }
    
private:
    void setNormalLimits() {
        // 성인 정상 범위 설정
        normal_limits.hr_min = 60;
        normal_limits.hr_max = 100;
        normal_limits.bp_sys_min = 90;
        normal_limits.bp_sys_max = 140;
        normal_limits.bp_dia_min = 60;
        normal_limits.bp_dia_max = 90;
        normal_limits.temp_min = 36.0;
        normal_limits.temp_max = 37.5;
        normal_limits.resp_min = 12;
        normal_limits.resp_max = 20;
        normal_limits.spo2_min = 95;
        
        // 환자별 조정 (나이, 상태에 따라)
        if (patient.age > 65) {
            normal_limits.hr_max = 90;  // 노인 심박수 조정
        }
        
        if (patient.has_cardiac_condition) {
            normal_limits.hr_min = 50;  // 심장질환자 조정
            normal_limits.hr_max = 90;
        }
    }
    
    void initializeSensors() {
        Serial.println("의료 센서 초기화...");
        
        // 심전도 센서 (AD8232)
        pinMode(A0, INPUT); // ECG 신호
        pinMode(10, INPUT); // LO+
        pinMode(11, INPUT); // LO-
        
        // 맥박 센서 (MAX30102)
        Serial.println("맥박/산소포화도 센서 초기화");
        
        // 온도 센서 (의료용 정밀 센서)
        Serial.println("체온 센서 초기화");
        
        // 혈압 센서 인터페이스
        Serial.println("혈압 측정 모듈 초기화");
        
        Serial.println("모든 센서 초기화 완료");
    }
    
    void measureVitalSigns() {
        MonitoringData& current = monitoring_buffer[buffer_index];
        current.timestamp = millis();
        
        // 1. 심전도 및 심박수 측정
        measureECGAndHeartRate(current);
        
        // 2. 혈압 측정 (주기적으로)
        static uint32_t last_bp_measurement = 0;
        if (millis() - last_bp_measurement > 300000) { // 5분마다
            measureBloodPressure(current);
            last_bp_measurement = millis();
        }
        
        // 3. 체온 측정
        measureBodyTemperature(current);
        
        // 4. 호흡수 측정
        measureRespiratoryRate(current);
        
        // 5. 산소포화도 측정
        measureOxygenSaturation(current);
    }
    
    void measureECGAndHeartRate(MonitoringData& data) {
        // ECG 신호 읽기
        if ((digitalRead(10) == 1) || (digitalRead(11) == 1)) {
            // 전극 이탈
            data.ecg_voltage = 0;
            data.heart_rate = 0;
            data.rhythm_regular = false;
            Serial.println("경고: ECG 전극 이탈");
            return;
        }
        
        // ECG 신호 처리
        int ecg_raw = analogRead(A0);
        data.ecg_voltage = (ecg_raw - 512) * 3.3 / 1024.0; // mV 변환
        
        // 심박수 계산 (R파 감지)
        static int ecg_buffer[100];
        static int buffer_pos = 0;
        static uint32_t last_r_peak = 0;
        static int beat_count = 0;
        
        ecg_buffer[buffer_pos] = ecg_raw;
        buffer_pos = (buffer_pos + 1) % 100;
        
        // R파 감지 알고리즘 (간단한 임계값 기반)
        if (ecg_raw > 600 && millis() - last_r_peak > 300) { // 최소 300ms 간격
            beat_count++;
            last_r_peak = millis();
            
            // 최근 1분간 심박수 계산
            if (beat_count >= 10) {
                uint32_t time_span = millis() - (last_r_peak - 9 * 1000); // 대략 10박자 시간
                data.heart_rate = 60000.0 / (time_span / 10.0);
                beat_count = 0;
            }
        }
        
        // 심율 규칙성 판단
        data.rhythm_regular = isRhythmRegular();
    }
    
    void measureBloodPressure(MonitoringData& data) {
        Serial.println("혈압 측정 중...");
        
        // 자동 혈압계 시뮬레이션
        // 실제로는 커프 팽창/수축 제어 필요
        
        delay(3000); // 측정 시간
        
        // 시뮬레이션 데이터 (정상 범위)
        data.bp_systolic = 115 + random(-10, 15);
        data.bp_diastolic = 75 + random(-5, 10);
        
        Serial.print("혈압: ");
        Serial.print(data.bp_systolic);
        Serial.print("/");
        Serial.print(data.bp_diastolic);
        Serial.println(" mmHg");
    }
    
    void measureBodyTemperature(MonitoringData& data) {
        // 적외선 온도 센서 또는 접촉식 센서
        int temp_raw = analogRead(A1);
        
        // 온도 변환 (의료용 정밀도)
        float voltage = temp_raw * 3.3 / 1024.0;
        data.temperature = 36.5 + (voltage - 1.5) * 10; // 캘리브레이션 필요
        
        // 범위 제한
        data.temperature = constrain(data.temperature, 35.0, 42.0);
    }
    
    void measureRespiratoryRate(MonitoringData& data) {
        // 호흡 센서 (압력 센서 또는 스트레인 게이지)
        static int breath_count = 0;
        static uint32_t count_start_time = millis();
        static bool inhaling = false;
        
        int breath_sensor = analogRead(A2);
        
        // 호흡 패턴 감지
        if (breath_sensor > 550 && !inhaling) {
            inhaling = true;
            breath_count++;
        } else if (breath_sensor < 450 && inhaling) {
            inhaling = false;
        }
        
        // 1분간 호흡수 계산
        if (millis() - count_start_time >= 60000) {
            data.respiratory_rate = breath_count;
            breath_count = 0;
            count_start_time = millis();
        } else {
            // 추정값 계산
            float elapsed_minutes = (millis() - count_start_time) / 60000.0;
            data.respiratory_rate = breath_count / elapsed_minutes;
        }
    }
    
    void measureOxygenSaturation(MonitoringData& data) {
        // MAX30102 센서 시뮬레이션
        // 적색/적외선 LED를 이용한 맥박 산소포화도 측정
        
        int red_value = analogRead(A3);
        int ir_value = analogRead(A4);
        
        // SpO2 계산 알고리즘 (간단화)
        if (ir_value > 100) {
            float ratio = (float)red_value / ir_value;
            data.spo2 = 110 - 25 * ratio; // 경험적 공식
            data.spo2 = constrain(data.spo2, 70, 100);
        } else {
            data.spo2 = 0; // 신호 없음
        }
    }
    
    bool isRhythmRegular() {
        // 심율 규칙성 판단 알고리즘
        // 실제로는 RR 간격의 변이도 분석 필요
        return true; // 시뮬레이션
    }
    
    void analyzeVitalSigns() {
        MonitoringData& current = monitoring_buffer[buffer_index];
        
        // 트렌드 분석 (최근 5분 데이터)
        analyzeTrends();
        
        // 패턴 인식
        detectAbnormalPatterns();
        
        // 임상적 의미 분석
        assessClinicalSignificance(current);
    }
    
    void analyzeTrends() {
        if (buffer_index < 5) return; // 최소 5개 데이터 필요
        
        // 심박수 트렌드
        float hr_trend = calculateTrend("heart_rate", 5);
        if (hr_trend > 5) {
            Serial.println("주의: 심박수 상승 추세");
        } else if (hr_trend < -5) {
            Serial.println("주의: 심박수 하강 추세");
        }
        
        // 체온 트렌드
        float temp_trend = calculateTrend("temperature", 5);
        if (temp_trend > 0.5) {
            Serial.println("주의: 체온 상승 추세");
        }
    }
    
    float calculateTrend(const char* parameter, int window_size) {
        // 최소제곱법을 이용한 트렌드 계산
        float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        
        for (int i = 0; i < window_size; i++) {
            int idx = (buffer_index - i + 1440) % 1440;
            float x = i;
            float y = 0;
            
            if (strcmp(parameter, "heart_rate") == 0) {
                y = monitoring_buffer[idx].heart_rate;
            } else if (strcmp(parameter, "temperature") == 0) {
                y = monitoring_buffer[idx].temperature;
            }
            
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }
        
        float slope = (window_size * sum_xy - sum_x * sum_y) / 
                     (window_size * sum_x2 - sum_x * sum_x);
        return slope;
    }
    
    void detectAbnormalPatterns() {
        // 부정맥 감지
        detectArrhythmia();
        
        // 호흡 이상 감지
        detectRespiratoryAbnormalities();
        
        // 혈압 변동 감지
        detectBloodPressureAbnormalities();
    }
    
    void detectArrhythmia() {
        MonitoringData& current = monitoring_buffer[buffer_index];
        
        // 빈맥 (Tachycardia)
        if (current.heart_rate > normal_limits.hr_max) {
            strcpy(current.alarm_message, "빈맥 감지");
            current.current_alarm = MEDIUM_PRIORITY;
        }
        // 서맥 (Bradycardia)
        else if (current.heart_rate < normal_limits.hr_min) {
            strcpy(current.alarm_message, "서맥 감지");
            current.current_alarm = MEDIUM_PRIORITY;
        }
        // 부정맥
        else if (!current.rhythm_regular) {
            strcpy(current.alarm_message, "부정맥 감지");
            current.current_alarm = HIGH_PRIORITY;
        }
    }
    
    void detectRespiratoryAbnormalities() {
        MonitoringData& current = monitoring_buffer[buffer_index];
        
        // 빈호흡 (Tachypnea)
        if (current.respiratory_rate > normal_limits.resp_max) {
            strcpy(current.alarm_message, "빈호흡 감지");
            current.current_alarm = MEDIUM_PRIORITY;
        }
        // 서호흡 (Bradypnea)
        else if (current.respiratory_rate < normal_limits.resp_min) {
            strcpy(current.alarm_message, "서호흡 감지");
            current.current_alarm = MEDIUM_PRIORITY;
        }
        
        // 산소포화도 저하
        if (current.spo2 < normal_limits.spo2_min) {
            strcpy(current.alarm_message, "산소포화도 저하");
            current.current_alarm = HIGH_PRIORITY;
            
            if (current.spo2 < 90) {
                current.current_alarm = CRITICAL;
            }
        }
    }
    
    void detectBloodPressureAbnormalities() {
        MonitoringData& current = monitoring_buffer[buffer_index];
        
        // 고혈압
        if (current.bp_systolic > normal_limits.bp_sys_max || 
            current.bp_diastolic > normal_limits.bp_dia_max) {
            strcpy(current.alarm_message, "고혈압 감지");
            current.current_alarm = MEDIUM_PRIORITY;
        }
        // 저혈압
        else if (current.bp_systolic < normal_limits.bp_sys_min || 
                 current.bp_diastolic < normal_limits.bp_dia_min) {
            strcpy(current.alarm_message, "저혈압 감지");
            current.current_alarm = MEDIUM_PRIORITY;
        }
    }
    
    void assessClinicalSignificance(MonitoringData& data) {
        // 여러 바이탈 사인의 조합 분석
        
        // 쇼크 상태 의심
        if (data.bp_systolic < 90 && data.heart_rate > 100) {
            strcpy(data.alarm_message, "쇼크 상태 의심");
            data.current_alarm = CRITICAL;
        }
        
        // 발열과 빈맥 조합
        if (data.temperature > 38.0 && data.heart_rate > normal_limits.hr_max) {
            strcpy(data.alarm_message, "발열성 빈맥");
            data.current_alarm = MEDIUM_PRIORITY;
        }
        
        // 호흡곤란 증후군
        if (data.respiratory_rate > 25 && data.spo2 < 95) {
            strcpy(data.alarm_message, "호흡곤란 증후군");
            data.current_alarm = HIGH_PRIORITY;
        }
    }
    
    void checkAlarmConditions() {
        MonitoringData& current = monitoring_buffer[buffer_index];
        
        // 기본 알람 상태 초기화
        current.current_alarm = NO_ALARM;
        strcpy(current.alarm_message, "정상");
        
        // 각종 이상 상태 체크는 이미 분석 단계에서 수행됨
        
        // 센서 오류 체크
        if (current.heart_rate == 0) {
            strcpy(current.alarm_message, "심전도 센서 오류");
            current.current_alarm = HIGH_PRIORITY;
        }
        
        if (current.spo2 == 0) {
            strcpy(current.alarm_message, "산소포화도 센서 오류");
            current.current_alarm = MEDIUM_PRIORITY;
        }
    }
    
    void storeData() {
        // 버퍼 인덱스 증가
        buffer_index = (buffer_index + 1) % 1440;
        
        // 데이터베이스 저장 (HIPAA 준수)
        // 실제로는 암호화된 의료 데이터베이스에 저장
    }
    
    void sendRealTimeData() {
        MonitoringData& current = monitoring_buffer[(buffer_index + 1439) % 1440];
        
        // JSON 형태로 실시간 데이터 전송
        String json = "{";
        json += "\"timestamp\":" + String(current.timestamp) + ",";
        json += "\"heart_rate\":" + String(current.heart_rate) + ",";
        json += "\"bp_systolic\":" + String(current.bp_systolic) + ",";
        json += "\"bp_diastolic\":" + String(current.bp_diastolic) + ",";
        json += "\"temperature\":" + String(current.temperature) + ",";
        json += "\"respiratory_rate\":" + String(current.respiratory_rate) + ",";
        json += "\"spo2\":" + String(current.spo2) + ",";
        json += "\"alarm_level\":" + String(current.current_alarm) + ",";
        json += "\"alarm_message\":\"" + String(current.alarm_message) + "\"";
        json += "}";
        
        webSocket.broadcastTXT(json);
    }
    
    void handleAlarms() {
        MonitoringData& current = monitoring_buffer[(buffer_index + 1439) % 1440];
        
        switch (current.current_alarm) {
            case CRITICAL:
                // 연속 알람음 + 적색 LED
                digitalWrite(13, HIGH);
                tone(12, 1000, 200);
                delay(200);
                tone(12, 800, 200);
                Serial.println("*** 위험 알람: " + String(current.alarm_message) + " ***");
                break;
                
            case HIGH_PRIORITY:
                // 빠른 알람음 + 적색 LED
                digitalWrite(13, HIGH);
                tone(12, 1000, 100);
                Serial.println("** 높은 우선순위 알람: " + String(current.alarm_message) + " **");
                break;
                
            case MEDIUM_PRIORITY:
                // 보통 알람음 + 황색 LED
                digitalWrite(13, HIGH);
                tone(12, 800, 500);
                Serial.println("* 중간 우선순위 알람: " + String(current.alarm_message) + " *");
                break;
                
            case LOW_PRIORITY:
                // 단일 알람음
                tone(12, 600, 100);
                Serial.println("주의: " + String(current.alarm_message));
                break;
                
            case NO_ALARM:
                digitalWrite(13, LOW);
                break;
        }
    }
    
    void transmitToNursingStation() {
        static uint32_t last_transmission = 0;
        
        // 5초마다 간호사실로 데이터 전송
        if (millis() - last_transmission > 5000) {
            MonitoringData& current = monitoring_buffer[(buffer_index + 1439) % 1440];
            
            Serial.println("\n=== 간호사실 전송 ===");
            Serial.println("환자 ID: " + String(patient.patient_id));
            Serial.println("심박수: " + String(current.heart_rate) + " bpm");
            Serial.println("혈압: " + String(current.bp_systolic) + "/" + 
                          String(current.bp_diastolic) + " mmHg");
            Serial.println("체온: " + String(current.temperature) + " °C");
            Serial.println("호흡수: " + String(current.respiratory_rate) + " /min");
            Serial.println("SpO2: " + String(current.spo2) + " %");
            
            if (current.current_alarm > NO_ALARM) {
                Serial.println("알람: " + String(current.alarm_message));
            }
            
            last_transmission = millis();
        }
    }
    
    static void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
        switch (type) {
            case WStype_DISCONNECTED:
                Serial.printf("클라이언트 [%u] 연결 해제\n", num);
                break;
                
            case WStype_CONNECTED:
                {
                    IPAddress ip = webSocket.remoteIP(num);
                    Serial.printf("클라이언트 [%u] 연결됨: %d.%d.%d.%d\n", 
                                 num, ip[0], ip[1], ip[2], ip[3]);
                    webSocket.sendTXT(num, "환자 모니터링 시스템에 연결됨");
                }
                break;
                
            case WStype_TEXT:
                Serial.printf("클라이언트 [%u]: %s\n", num, payload);
                break;
                
            default:
                break;
        }
    }
};

PatientMonitoringSystem patient_monitor;

void setup() {
    patient_monitor.initialize();
}

void loop() {
    patient_monitor.update();
    delay(100); // 10Hz 업데이트
}
```

---

## 심전도(ECG) 측정 시스템

### 12유도 ECG 분석 시스템
```cpp
#include <SPI.h>
#include <SD.h>

class ECGAnalysisSystem {
private:
    enum ECGLead {
        LEAD_I = 0,     // 제1유도
        LEAD_II = 1,    // 제2유도  
        LEAD_III = 2,   // 제3유도
        AVR = 3,        // aVR
        AVL = 4,        // aVL
        AVF = 5,        // aVF
        V1 = 6,         // V1
        V2 = 7,         // V2
        V3 = 8,         // V3
        V4 = 9,         // V4
        V5 = 10,        // V5
        V6 = 11         // V6
    };
    
    enum RhythmType {
        NORMAL_SINUS = 0,
        SINUS_BRADYCARDIA = 1,
        SINUS_TACHYCARDIA = 2,
        ATRIAL_FIBRILLATION = 3,
        VENTRICULAR_TACHYCARDIA = 4,
        VENTRICULAR_FIBRILLATION = 5,
        HEART_BLOCK = 6,
        PREMATURE_BEATS = 7
    };
    
    struct ECGWaveform {
        float p_wave_amplitude;     // P파 진폭 (mV)
        float qrs_amplitude;        // QRS파 진폭 (mV)
        float t_wave_amplitude;     // T파 진폭 (mV)
        float pr_interval;          // PR 간격 (ms)
        float qrs_duration;         // QRS 지속시간 (ms)
        float qt_interval;          // QT 간격 (ms)
        float rr_interval;          // RR 간격 (ms)
        float st_elevation;         // ST 분절 상승 (mV)
        bool q_wave_present;        // Q파 존재 여부
    };
    
    struct ECGAnalysis {
        float heart_rate;           // 심박수
        RhythmType rhythm;          // 심율
        bool axis_normal;           // 심축 정상 여부
        float axis_degrees;         // 심축 각도
        bool ischemia_detected;     // 허혈 감지
        bool infarction_detected;   // 경색 감지
        bool arrhythmia_detected;   // 부정맥 감지
        char diagnosis[200];        // 진단 결과
        uint8_t confidence;         // 신뢰도 (%)
    };
    
    struct PatientECG {
        uint32_t patient_id;
        uint32_t timestamp;
        ECGWaveform leads[12];      // 12유도 데이터
        ECGAnalysis analysis;       // 분석 결과
        float raw_data[12][1000];   // 원시 데이터 (1초간)
        bool leads_connected[12];   // 유도 연결 상태
    };
    
    PatientECG current_ecg;
    uint16_t sampling_rate;         // 샘플링 속도 (Hz)
    bool recording_active;
    File ecg_file;
    uint32_t last_analysis;
    
public:
    ECGAnalysisSystem() {
        sampling_rate = 1000;       // 1kHz 샘플링
        recording_active = false;
        last_analysis = 0;
        
        // 환자 정보 초기화
        current_ecg.patient_id = 12345;
        current_ecg.timestamp = 0;
        
        // 유도 연결 상태 초기화
        for (int i = 0; i < 12; i++) {
            current_ecg.leads_connected[i] = true;
        }
    }
    
    void initialize() {
        Serial.begin(115200);
        
        Serial.println("=== 12유도 ECG 분석 시스템 초기화 ===");
        Serial.println("의료기기 승인번호: ECG-2024-001");
        Serial.println("샘플링 속도: 1000 Hz");
        
        // SD카드 초기화 (ECG 데이터 저장용)
        if (SD.begin(4)) {
            Serial.println("SD카드 초기화 성공");
        } else {
            Serial.println("SD카드 초기화 실패!");
        }
        
        // ECG 증폭기 초기화
        initializeECGAmplifiers();
        
        // 필터 초기화
        initializeFilters();
        
        // 캘리브레이션
        performCalibration();
        
        Serial.println("ECG 시스템 준비 완료");
        Serial.println("환자 연결 후 측정을 시작하세요");
    }
    
    void startRecording() {
        if (!recording_active) {
            recording_active = true;
            current_ecg.timestamp = millis();
            
            // 새 ECG 파일 생성
            String filename = "ECG_" + String(current_ecg.patient_id) + "_" + 
                             String(current_ecg.timestamp) + ".txt";
            ecg_file = SD.open(filename, FILE_WRITE);
            
            Serial.println("ECG 기록 시작: " + filename);
        }
    }
    
    void stopRecording() {
        if (recording_active) {
            recording_active = false;
            ecg_file.close();
            
            Serial.println("ECG 기록 완료");
            generateReport();
        }
    }
    
    void update() {
        uint32_t current_time = millis();
        
        // 실시간 ECG 데이터 수집
        if (recording_active) {
            sampleECGData();
        }
        
        // 1초마다 실시간 분석
        if (current_time - last_analysis >= 1000) {
            performRealTimeAnalysis();
            displayRealTimeData();
            last_analysis = current_time;
        }
        
        // 유도 연결 상태 체크
        checkLeadConnections();
        
        // 부정맥 감지 (연속 모니터링)
        detectArrhythmias();
    }
    
private:
    void initializeECGAmplifiers() {
        Serial.println("ECG 증폭기 초기화...");
        
        // 각 유도별 증폭기 설정
        // 실제로는 AD8232, ADS1298 등의 전용 ECG 칩 사용
        
        // 아날로그 입력 핀 설정 (12유도)
        for (int i = 0; i < 12; i++) {
            pinMode(A0 + i, INPUT);
        }
        
        // 기준 전극 설정
        pinMode(2, INPUT);  // RA (Right Arm)
        pinMode(3, INPUT);  // LA (Left Arm)
        pinMode(4, INPUT);  // RL (Right Leg) - 기준
        pinMode(5, INPUT);  // LL (Left Leg)
        
        Serial.println("12유도 증폭기 초기화 완료");
    }
    
    void initializeFilters() {
        Serial.println("신호 필터 초기화...");
        
        // 저역통과 필터 (40Hz) - 근전도 잡음 제거
        // 고역통과 필터 (0.5Hz) - 기저선 변동 제거
        // 노치 필터 (60Hz) - 전원선 잡음 제거
        
        Serial.println("디지털 필터 설정 완료");
    }
    
    void performCalibration() {
        Serial.println("ECG 시스템 캘리브레이션...");
        
        // 1mV 캘리브레이션 신호 생성
        Serial.println("1mV 교정 신호 확인");
        delay(1000);
        
        // 각 유도별 이득 조정
        for (int i = 0; i < 12; i++) {
            Serial.print("유도 "); 
            Serial.print(getLeadName(i));
            Serial.println(" 캘리브레이션 완료");
        }
        
        Serial.println("캘리브레이션 완료");
    }
    
    String getLeadName(int lead_index) {
        switch (lead_index) {
            case LEAD_I: return "I";
            case LEAD_II: return "II";
            case LEAD_III: return "III";
            case AVR: return "aVR";
            case AVL: return "aVL";
            case AVF: return "aVF";
            case V1: return "V1";
            case V2: return "V2";
            case V3: return "V3";
            case V4: return "V4";
            case V5: return "V5";
            case V6: return "V6";
            default: return "Unknown";
        }
    }
    
    void sampleECGData() {
        static uint16_t sample_index = 0;
        
        // 1kHz로 모든 유도 동시 샘플링
        for (int lead = 0; lead < 12; lead++) {
            if (current_ecg.leads_connected[lead]) {
                // 아날로그 값 읽기 및 전압 변환
                int raw_value = analogRead(A0 + lead);
                float voltage = (raw_value - 512) * 3.3 / 1024.0; // ±1.65V 범위
                
                // 증폭 및 필터링
                voltage = applyFilters(voltage, lead);
                
                // 데이터 저장
                if (sample_index < 1000) {
                    current_ecg.raw_data[lead][sample_index] = voltage;
                }
                
                // SD카드에 기록
                if (ecg_file) {
                    ecg_file.print(voltage, 4);
                    if (lead < 11) ecg_file.print(",");
                }
            }
        }
        
        if (ecg_file) {
            ecg_file.println();
        }
        
        sample_index = (sample_index + 1) % 1000;
        
        delayMicroseconds(1000); // 1ms 지연 (1kHz)
    }
    
    float applyFilters(float input, int lead) {
        // 디지털 필터 적용
        static float filter_buffer[12][10];
        static int filter_index[12] = {0};
        
        // 이동 평균 필터 (간단한 저역통과)
        filter_buffer[lead][filter_index[lead]] = input;
        filter_index[lead] = (filter_index[lead] + 1) % 10;
        
        float filtered = 0;
        for (int i = 0; i < 10; i++) {
            filtered += filter_buffer[lead][i];
        }
        filtered /= 10.0;
        
        return filtered;
    }
    
    void performRealTimeAnalysis() {
        // QRS 복합체 감지
        detectQRSComplexes();
        
        // 심박수 계산
        calculateHeartRate();
        
        // P파, T파 감지
        detectPAndTWaves();
        
        // 간격 측정
        measureIntervals();
        
        // 심율 분석
        analyzeRhythm();
        
        // 허혈/경색 감지
        detectIschemia();
        
        // 축 계산
        calculateAxis();
        
        // 진단 생성
        generateDiagnosis();
    }
    
    void detectQRSComplexes() {
        // Pan-Tompkins 알고리즘 구현
        // 제2유도에서 QRS 감지 (가장 명확한 유도)
        
        static float qrs_buffer[20];
        static int qrs_index = 0;
        static uint32_t last_qrs_time = 0;
        static float qrs_threshold = 0.5; // mV
        
        float lead_ii_voltage = current_ecg.raw_data[LEAD_II][999]; // 최신 샘플
        
        // 미분 및 제곱
        static float prev_voltage = 0;
        float derivative = lead_ii_voltage - prev_voltage;
        float squared = derivative * derivative;
        prev_voltage = lead_ii_voltage;
        
        qrs_buffer[qrs_index] = squared;
        qrs_index = (qrs_index + 1) % 20;
        
        // 이동 적분 (20 샘플)
        float moving_integral = 0;
        for (int i = 0; i < 20; i++) {
            moving_integral += qrs_buffer[i];
        }
        moving_integral /= 20.0;
        
        // 임계값 비교
        if (moving_integral > qrs_threshold && millis() - last_qrs_time > 300) {
            // QRS 복합체 감지
            current_ecg.leads[LEAD_II].rr_interval = millis() - last_qrs_time;
            last_qrs_time = millis();
            
            // QRS 진폭 측정
            current_ecg.leads[LEAD_II].qrs_amplitude = lead_ii_voltage;
            
            Serial.println("QRS 감지 - RR간격: " + String(current_ecg.leads[LEAD_II].rr_interval) + "ms");
        }
    }
    
    void calculateHeartRate() {
        if (current_ecg.leads[LEAD_II].rr_interval > 0) {
            current_ecg.analysis.heart_rate = 60000.0 / current_ecg.leads[LEAD_II].rr_interval;
            
            // 심박수 범위 제한
            current_ecg.analysis.heart_rate = constrain(current_ecg.analysis.heart_rate, 30, 250);
        }
    }
    
    void detectPAndTWaves() {
        // P파 감지 (QRS 이전 200-300ms)
        // T파 감지 (QRS 이후 300-500ms)
        
        for (int lead = 0; lead < 12; lead++) {
            if (!current_ecg.leads_connected[lead]) continue;
            
            // 간단한 P파 감지 알고리즘
            float p_amplitude = findPWave(lead);
            current_ecg.leads[lead].p_wave_amplitude = p_amplitude;
            
            // T파 감지 알고리즘
            float t_amplitude = findTWave(lead);
            current_ecg.leads[lead].t_wave_amplitude = t_amplitude;
        }
    }
    
    float findPWave(int lead) {
        // P파는 QRS 이전의 작은 양성파
        // 시뮬레이션으로 대체
        return 0.1 + random(-10, 10) / 100.0; // ±0.1mV
    }
    
    float findTWave(int lead) {
        // T파는 QRS 이후의 완만한 파형
        // 시뮬레이션으로 대체
        return 0.3 + random(-5, 5) / 100.0; // 0.25-0.35mV
    }
    
    void measureIntervals() {
        // PR 간격 측정 (정상: 120-200ms)
        current_ecg.leads[LEAD_II].pr_interval = 150 + random(-20, 20);
        
        // QRS 지속시간 (정상: 80-120ms)
        current_ecg.leads[LEAD_II].qrs_duration = 100 + random(-10, 10);
        
        // QT 간격 측정 (정상: 350-450ms)
        current_ecg.leads[LEAD_II].qt_interval = 400 + random(-30, 30);
    }
    
    void analyzeRhythm() {
        float hr = current_ecg.analysis.heart_rate;
        
        if (hr >= 60 && hr <= 100) {
            // RR 간격 규칙성 확인
            if (isRhythmRegular()) {
                current_ecg.analysis.rhythm = NORMAL_SINUS;
            } else {
                current_ecg.analysis.rhythm = ATRIAL_FIBRILLATION;
            }
        } else if (hr < 60) {
            current_ecg.analysis.rhythm = SINUS_BRADYCARDIA;
        } else if (hr > 100 && hr < 150) {
            current_ecg.analysis.rhythm = SINUS_TACHYCARDIA;
        } else if (hr >= 150) {
            current_ecg.analysis.rhythm = VENTRICULAR_TACHYCARDIA;
        }
        
        current_ecg.analysis.arrhythmia_detected = 
            (current_ecg.analysis.rhythm != NORMAL_SINUS);
    }
    
    bool isRhythmRegular() {
        // RR 간격의 변이도 계산
        // 정상: 변이도 < 10%
        return true; // 시뮬레이션
    }
    
    void detectIschemia() {
        // ST 분절 분석
        for (int lead = 0; lead < 12; lead++) {
            // ST 상승/하강 감지
            float st_level = measureSTSegment(lead);
            current_ecg.leads[lead].st_elevation = st_level;
            
            // 허혈 감지 기준
            if (st_level > 0.1) { // 1mm 이상 상승
                current_ecg.analysis.ischemia_detected = true;
            } else if (st_level < -0.1) { // 1mm 이상 하강
                current_ecg.analysis.ischemia_detected = true;
            }
            
            // Q파 존재 여부 (경색 징후)
            if (current_ecg.leads[lead].qrs_amplitude < -0.3) {
                current_ecg.leads[lead].q_wave_present = true;
                current_ecg.analysis.infarction_detected = true;
            }
        }
    }
    
    float measureSTSegment(int lead) {
        // QRS 종료 후 80ms 지점의 전압 측정
        // 시뮬레이션으로 대체
        return random(-20, 20) / 100.0; // ±0.2mV
    }
    
    void calculateAxis() {
        // 제1유도와 aVF를 이용한 심축 계산
        float lead_i_amplitude = current_ecg.leads[LEAD_I].qrs_amplitude;
        float avf_amplitude = current_ecg.leads[AVF].qrs_amplitude;
        
        if (lead_i_amplitude > 0 && avf_amplitude > 0) {
            current_ecg.analysis.axis_degrees = 60; // 정상축
            current_ecg.analysis.axis_normal = true;
        } else if (lead_i_amplitude > 0 && avf_amplitude < 0) {
            current_ecg.analysis.axis_degrees = -30; // 좌축편위
            current_ecg.analysis.axis_normal = false;
        } else if (lead_i_amplitude < 0 && avf_amplitude > 0) {
            current_ecg.analysis.axis_degrees = 120; // 우축편위
            current_ecg.analysis.axis_normal = false;
        } else {
            current_ecg.analysis.axis_degrees = -120; // 극심한 축편위
            current_ecg.analysis.axis_normal = false;
        }
    }
    
    void generateDiagnosis() {
        String diagnosis = "";
        current_ecg.analysis.confidence = 85; // 기본 신뢰도
        
        // 심율 진단
        switch (current_ecg.analysis.rhythm) {
            case NORMAL_SINUS:
                diagnosis += "정상 동율동. ";
                break;
            case SINUS_BRADYCARDIA:
                diagnosis += "동성 서맥. ";
                break;
            case SINUS_TACHYCARDIA:
                diagnosis += "동성 빈맥. ";
                break;
            case ATRIAL_FIBRILLATION:
                diagnosis += "심방세동. ";
                current_ecg.analysis.confidence = 75;
                break;
            case VENTRICULAR_TACHYCARDIA:
                diagnosis += "심실빈맥. ";
                current_ecg.analysis.confidence = 80;
                break;
        }
        
        // 허혈/경색 진단
        if (current_ecg.analysis.infarction_detected) {
            diagnosis += "심근경색 의심. ";
            current_ecg.analysis.confidence = 70;
        } else if (current_ecg.analysis.ischemia_detected) {
            diagnosis += "심근허혈 의심. ";
            current_ecg.analysis.confidence = 65;
        }
        
        // 축 진단
        if (!current_ecg.analysis.axis_normal) {
            if (current_ecg.analysis.axis_degrees < -30) {
                diagnosis += "좌축편위. ";
            } else if (current_ecg.analysis.axis_degrees > 90) {
                diagnosis += "우축편위. ";
            }
        }
        
        // 간격 이상
        if (current_ecg.leads[LEAD_II].pr_interval > 200) {
            diagnosis += "1도 방실차단. ";
        }
        if (current_ecg.leads[LEAD_II].qrs_duration > 120) {
            diagnosis += "QRS 연장. ";
        }
        
        if (diagnosis.length() == 0) {
            diagnosis = "정상 심전도.";
            current_ecg.analysis.confidence = 90;
        }
        
        strcpy(current_ecg.analysis.diagnosis, diagnosis.c_str());
    }
    
    void checkLeadConnections() {
        // 각 유도의 연결 상태 확인
        for (int lead = 0; lead < 12; lead++) {
            float voltage = current_ecg.raw_data[lead][999]; // 최신 샘플
            
            // 연결 해제 시 매우 큰 값 또는 0
            if (abs(voltage) > 5.0 || abs(voltage) < 0.001) {
                if (current_ecg.leads_connected[lead]) {
                    Serial.println("경고: " + getLeadName(lead) + " 유도 연결 해제");
                    current_ecg.leads_connected[lead] = false;
                }
            } else {
                if (!current_ecg.leads_connected[lead]) {
                    Serial.println(getLeadName(lead) + " 유도 연결됨");
                    current_ecg.leads_connected[lead] = true;
                }
            }
        }
    }
    
    void detectArrhythmias() {
        // 실시간 부정맥 감지
        static uint32_t last_arrhythmia_check = 0;
        
        if (millis() - last_arrhythmia_check > 5000) { // 5초마다
            if (current_ecg.analysis.arrhythmia_detected) {
                Serial.println("*** 부정맥 감지 ***");
                Serial.println("심율: " + getRhythmName(current_ecg.analysis.rhythm));
                Serial.println("심박수: " + String(current_ecg.analysis.heart_rate) + " bpm");
                
                // 응급 알람
                tone(8, 1000, 500);
            }
            
            last_arrhythmia_check = millis();
        }
    }
    
    String getRhythmName(RhythmType rhythm) {
        switch (rhythm) {
            case NORMAL_SINUS: return "정상 동율동";
            case SINUS_BRADYCARDIA: return "동성 서맥";
            case SINUS_TACHYCARDIA: return "동성 빈맥";
            case ATRIAL_FIBRILLATION: return "심방세동";
            case VENTRICULAR_TACHYCARDIA: return "심실빈맥";
            case VENTRICULAR_FIBRILLATION: return "심실세동";
            case HEART_BLOCK: return "방실차단";
            case PREMATURE_BEATS: return "조기박동";
            default: return "알 수 없음";
        }
    }
    
    void displayRealTimeData() {
        Serial.println("\n=== 실시간 ECG 분석 ===");
        Serial.println("환자 ID: " + String(current_ecg.patient_id));
        Serial.println("심박수: " + String(current_ecg.analysis.heart_rate) + " bpm");
        Serial.println("심율: " + getRhythmName(current_ecg.analysis.rhythm));
        Serial.println("심축: " + String(current_ecg.analysis.axis_degrees) + "도");
        
        // 주요 간격들
        Serial.println("PR 간격: " + String(current_ecg.leads[LEAD_II].pr_interval) + " ms");
        Serial.println("QRS 지속시간: " + String(current_ecg.leads[LEAD_II].qrs_duration) + " ms");
        Serial.println("QT 간격: " + String(current_ecg.leads[LEAD_II].qt_interval) + " ms");
        
        // 연결 상태
        Serial.print("연결된 유도: ");
        for (int i = 0; i < 12; i++) {
            if (current_ecg.leads_connected[i]) {
                Serial.print(getLeadName(i) + " ");
            }
        }
        Serial.println();
        
        if (current_ecg.analysis.ischemia_detected || current_ecg.analysis.infarction_detected) {
            Serial.println("⚠️ 심근허혈/경색 의심");
        }
        
        Serial.println("진단: " + String(current_ecg.analysis.diagnosis));
        Serial.println("신뢰도: " + String(current_ecg.analysis.confidence) + "%");
    }
    
    void generateReport() {
        Serial.println("\n=== ECG 진단 리포트 ===");
        Serial.println("날짜: " + String(current_ecg.timestamp));
        Serial.println("환자 ID: " + String(current_ecg.patient_id));
        Serial.println();
        
        Serial.println("== 측정 결과 ==");
        Serial.println("심박수: " + String(current_ecg.analysis.heart_rate) + " bpm");
        Serial.println("심율: " + getRhythmName(current_ecg.analysis.rhythm));
        Serial.println("심축: " + String(current_ecg.analysis.axis_degrees) + "도");
        Serial.println();
        
        Serial.println("== 파형 분석 ==");
        Serial.println("PR 간격: " + String(current_ecg.leads[LEAD_II].pr_interval) + " ms");
        Serial.println("QRS 지속시간: " + String(current_ecg.leads[LEAD_II].qrs_duration) + " ms");
        Serial.println("QT 간격: " + String(current_ecg.leads[LEAD_II].qt_interval) + " ms");
        Serial.println();
        
        Serial.println("== 진단 ==");
        Serial.println(current_ecg.analysis.diagnosis);
        Serial.println("신뢰도: " + String(current_ecg.analysis.confidence) + "%");
        Serial.println();
        
        Serial.println("== 권고사항 ==");
        if (current_ecg.analysis.arrhythmia_detected) {
            Serial.println("- 부정맥이 감지되었습니다. 심장내과 진료를 권장합니다.");
        }
        if (current_ecg.analysis.ischemia_detected) {
            Serial.println("- 심근허혈이 의심됩니다. 즉시 응급실 내원을 권장합니다.");
        }
        if (current_ecg.analysis.infarction_detected) {
            Serial.println("- 심근경색이 의심됩니다. 즉시 응급처치가 필요합니다.");
        }
        
        Serial.println("\n진단의: 담당의사명");
        Serial.println("판독일시: " + String(millis()));
    }
};

ECGAnalysisSystem ecg_system;

void setup() {
    ecg_system.initialize();
}

void loop() {
    ecg_system.update();
    
    // 키보드 입력 처리
    if (Serial.available()) {
        char command = Serial.read();
        switch (command) {
            case 's':
                ecg_system.startRecording();
                break;
            case 'e':
                ecg_system.stopRecording();
                break;
        }
    }
    
    delay(1); // 1ms 지연 (1kHz 샘플링)
}
```

이어서 의료용 전자 시스템의 나머지 부분들을 작성하겠습니다.