# 자동차 전자 시스템 완전 가이드 - Arduino 기반 실무 구현

## 📋 목차
1. [개요](#개요)
2. [CAN 버스 시스템](#can-버스-시스템)
3. [ECU 시뮬레이션](#ecu-시뮬레이션)
4. [OBD-II 진단 시스템](#obd-ii-진단-시스템)
5. [자동차 센서 네트워크](#자동차-센서-네트워크)
6. [ADAS 시스템](#adas-시스템)
7. [차량 내 인포테인먼트](#차량-내-인포테인먼트)
8. [전동화 시스템](#전동화-시스템)
9. [사이버 보안](#사이버-보안)
10. [자율주행 기초](#자율주행-기초)

---

## 개요

현대 자동차는 100개 이상의 ECU(Electronic Control Unit)와 수백 개의 센서로 이루어진 복잡한 전자 시스템입니다. 본 가이드는 Arduino를 활용하여 자동차 전자 시스템의 핵심 기술들을 구현하고 학습할 수 있는 실무 중심의 완전한 가이드를 제공합니다.

### 자동차 전자 시스템 아키텍처
- **파워트레인**: 엔진/모터 제어, 변속기 제어
- **섀시**: ABS, ESC, 조향 시스템
- **바디**: 조명, 도어, 시트 제어
- **인포테인먼트**: 오디오, 내비게이션, 커넥티비티
- **ADAS**: 충돌 방지, 차선 유지, 어댑티브 크루즈

---

## CAN 버스 시스템

### CAN (Controller Area Network) 기초 구현

```cpp
/*
 * 자동차 CAN 버스 시스템 구현
 * ISO 11898 표준 준수
 * 다중 ECU 간 실시간 통신
 */

#include <mcp_can.h>
#include <SPI.h>

// CAN 버스 설정
#define CAN0_INT 2          // MCP2515 인터럽트 핀
#define CAN0_CS 10          // MCP2515 CS 핀
MCP_CAN CAN0(CAN0_CS);

// 자동차 CAN ID 정의 (SAE J1939 기반)
#define CAN_ID_ENGINE_RPM       0x0C0      // 엔진 RPM
#define CAN_ID_VEHICLE_SPEED    0x0D0      // 차량 속도
#define CAN_ID_COOLANT_TEMP     0x0E0      // 냉각수 온도
#define CAN_ID_FUEL_LEVEL       0x0F0      // 연료량
#define CAN_ID_THROTTLE_POS     0x100      // 스로틀 위치
#define CAN_ID_BRAKE_PRESSURE   0x110      // 브레이크 압력
#define CAN_ID_STEERING_ANGLE   0x120      // 조향각
#define CAN_ID_ABS_STATUS       0x130      // ABS 상태
#define CAN_ID_AIRBAG_STATUS    0x140      // 에어백 상태
#define CAN_ID_DOOR_STATUS      0x150      // 도어 상태

class AutomotiveCAN {
private:
    // 차량 데이터 구조체
    struct VehicleData {
        uint16_t engine_rpm;        // 엔진 RPM (0-8000)
        uint8_t vehicle_speed;      // 차량 속도 (0-255 km/h)
        int8_t coolant_temp;        // 냉각수 온도 (-40 to 215°C)
        uint8_t fuel_level;         // 연료량 (0-100%)
        uint8_t throttle_position;  // 스로틀 위치 (0-100%)
        uint16_t brake_pressure;    // 브레이크 압력 (0-1000 kPa)
        int16_t steering_angle;     // 조향각 (-540 to 540도)
        bool abs_active;            // ABS 작동 상태
        bool airbag_fault;          // 에어백 결함
        uint8_t door_status;        // 도어 상태 (비트마스크)
        unsigned long timestamp;    // 타임스탬프
    };
    
    VehicleData vehicle_data;
    bool can_initialized = false;
    
    // CAN 메시지 통계
    uint32_t messages_sent = 0;
    uint32_t messages_received = 0;
    uint32_t errors_detected = 0;
    
public:
    AutomotiveCAN() {
        // 차량 데이터 초기화
        memset(&vehicle_data, 0, sizeof(vehicle_data));
        vehicle_data.coolant_temp = 90; // 정상 온도
        vehicle_data.fuel_level = 75;   // 75% 연료
    }
    
    // CAN 버스 초기화
    bool initializeCAN() {
        Serial.println("CAN 버스 초기화 중...");
        
        // MCP2515 초기화 (250kbps)
        if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
            Serial.println("✓ CAN 버스 초기화 성공 (250kbps)");
            can_initialized = true;
        } else {
            Serial.println("✗ CAN 버스 초기화 실패");
            return false;
        }
        
        // 인터럽트 설정
        pinMode(CAN0_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(CAN0_INT), 
                       []() { /* CAN 인터럽트 핸들러 */ }, FALLING);
        
        // Normal 모드로 전환
        CAN0.setMode(MCP_NORMAL);
        
        // CAN 필터 설정 (엔진 관련 메시지만 수신)
        CAN0.init_Mask(0, 0, 0x700);           // 마스크 0
        CAN0.init_Filt(0, 0, 0x0C0);           // 필터 0: 엔진 RPM
        CAN0.init_Filt(1, 0, 0x0D0);           // 필터 1: 차량 속도
        
        Serial.println("CAN 필터 설정 완료");
        return true;
    }
    
    // 엔진 데이터 전송
    void sendEngineData() {
        // 엔진 RPM 전송
        uint8_t rpm_data[8] = {0};
        rpm_data[0] = (vehicle_data.engine_rpm >> 8) & 0xFF;
        rpm_data[1] = vehicle_data.engine_rpm & 0xFF;
        rpm_data[2] = vehicle_data.throttle_position;
        rpm_data[3] = vehicle_data.coolant_temp + 40; // offset
        
        if(sendCANMessage(CAN_ID_ENGINE_RPM, rpm_data, 4)) {
            Serial.printf("엔진 데이터 전송: RPM=%d, 스로틀=%d%%, 온도=%d°C\n",
                         vehicle_data.engine_rpm, vehicle_data.throttle_position, 
                         vehicle_data.coolant_temp);
        }
        
        // 연료량 전송
        uint8_t fuel_data[8] = {0};
        fuel_data[0] = vehicle_data.fuel_level;
        
        sendCANMessage(CAN_ID_FUEL_LEVEL, fuel_data, 1);
    }
    
    // 차량 동역학 데이터 전송
    void sendVehicleDynamics() {
        // 차량 속도 전송
        uint8_t speed_data[8] = {0};
        speed_data[0] = vehicle_data.vehicle_speed;
        
        sendCANMessage(CAN_ID_VEHICLE_SPEED, speed_data, 1);
        
        // 조향각 전송
        uint8_t steering_data[8] = {0};
        steering_data[0] = (vehicle_data.steering_angle >> 8) & 0xFF;
        steering_data[1] = vehicle_data.steering_angle & 0xFF;
        
        sendCANMessage(CAN_ID_STEERING_ANGLE, steering_data, 2);
        
        // 브레이크 압력 전송
        uint8_t brake_data[8] = {0};
        brake_data[0] = (vehicle_data.brake_pressure >> 8) & 0xFF;
        brake_data[1] = vehicle_data.brake_pressure & 0xFF;
        
        sendCANMessage(CAN_ID_BRAKE_PRESSURE, brake_data, 2);
    }
    
    // 안전 시스템 데이터 전송
    void sendSafetyData() {
        // ABS 상태 전송
        uint8_t abs_data[8] = {0};
        abs_data[0] = vehicle_data.abs_active ? 0x01 : 0x00;
        abs_data[1] = vehicle_data.vehicle_speed; // 참조 속도
        
        sendCANMessage(CAN_ID_ABS_STATUS, abs_data, 2);
        
        // 에어백 상태 전송
        uint8_t airbag_data[8] = {0};
        airbag_data[0] = vehicle_data.airbag_fault ? 0xFF : 0x00;
        
        sendCANMessage(CAN_ID_AIRBAG_STATUS, airbag_data, 1);
        
        // 도어 상태 전송
        uint8_t door_data[8] = {0};
        door_data[0] = vehicle_data.door_status;
        
        sendCANMessage(CAN_ID_DOOR_STATUS, door_data, 1);
    }
    
    // CAN 메시지 전송
    bool sendCANMessage(uint32_t can_id, uint8_t* data, uint8_t length) {
        if(!can_initialized) return false;
        
        byte send_status = CAN0.sendMsgBuf(can_id, 0, length, data);
        
        if(send_status == CAN_OK) {
            messages_sent++;
            return true;
        } else {
            errors_detected++;
            Serial.printf("CAN 전송 오류: ID=0x%X, 상태=%d\n", can_id, send_status);
            return false;
        }
    }
    
    // CAN 메시지 수신
    void receiveCANMessages() {
        if(!digitalRead(CAN0_INT)) {
            uint32_t rx_id;
            uint8_t rx_len;
            uint8_t rx_data[8];
            
            if(CAN0.readMsgBuf(&rx_id, &rx_len, rx_data) == CAN_OK) {
                messages_received++;
                processReceivedMessage(rx_id, rx_data, rx_len);
            }
        }
    }
    
    // 수신 메시지 처리
    void processReceivedMessage(uint32_t can_id, uint8_t* data, uint8_t length) {
        switch(can_id) {
            case CAN_ID_ENGINE_RPM:
                if(length >= 2) {
                    uint16_t received_rpm = (data[0] << 8) | data[1];
                    Serial.printf("수신: 엔진 RPM = %d\n", received_rpm);
                }
                break;
                
            case CAN_ID_VEHICLE_SPEED:
                if(length >= 1) {
                    Serial.printf("수신: 차량 속도 = %d km/h\n", data[0]);
                }
                break;
                
            case CAN_ID_BRAKE_PRESSURE:
                if(length >= 2) {
                    uint16_t brake_pressure = (data[0] << 8) | data[1];
                    Serial.printf("수신: 브레이크 압력 = %d kPa\n", brake_pressure);
                }
                break;
                
            default:
                Serial.printf("수신: 알 수 없는 CAN ID = 0x%X\n", can_id);
                break;
        }
    }
    
    // 차량 데이터 시뮬레이션
    void simulateVehicleData() {
        static unsigned long last_update = 0;
        static bool engine_running = false;
        
        if(millis() - last_update < 100) return; // 100ms 주기
        
        last_update = millis();
        
        // 엔진 시뮬레이션
        if(Serial.available()) {
            char command = Serial.read();
            if(command == 's') { // 시동
                engine_running = !engine_running;
                Serial.printf("엔진 %s\n", engine_running ? "시동" : "정지");
            }
        }
        
        if(engine_running) {
            // 엔진 RPM (idle: 800, max: 6000)
            static uint16_t target_rpm = 800;
            int throttle_input = analogRead(A0); // 스로틀 입력
            target_rpm = map(throttle_input, 0, 1023, 800, 6000);
            
            // RPM 부드러운 변화
            if(vehicle_data.engine_rpm < target_rpm) {
                vehicle_data.engine_rpm += 50;
            } else if(vehicle_data.engine_rpm > target_rpm) {
                vehicle_data.engine_rpm -= 100;
            }
            
            // 스로틀 위치
            vehicle_data.throttle_position = map(throttle_input, 0, 1023, 0, 100);
            
            // 차량 속도 (RPM 기반)
            vehicle_data.vehicle_speed = map(vehicle_data.engine_rpm, 800, 6000, 0, 180);
            
            // 냉각수 온도 (엔진 부하에 따라 변화)
            if(vehicle_data.engine_rpm > 3000) {
                vehicle_data.coolant_temp = min(105, vehicle_data.coolant_temp + 1);
            } else {
                vehicle_data.coolant_temp = max(85, vehicle_data.coolant_temp - 1);
            }
            
            // 연료 소모 (고부하 시 더 많이 소모)
            static unsigned long last_fuel_update = 0;
            if(millis() - last_fuel_update > 10000) { // 10초마다
                if(vehicle_data.engine_rpm > 3000) {
                    vehicle_data.fuel_level = max(0, vehicle_data.fuel_level - 1);
                }
                last_fuel_update = millis();
            }
        } else {
            vehicle_data.engine_rpm = 0;
            vehicle_data.vehicle_speed = 0;
            vehicle_data.throttle_position = 0;
        }
        
        // 브레이크 시뮬레이션
        int brake_input = analogRead(A1);
        vehicle_data.brake_pressure = map(brake_input, 0, 1023, 0, 1000);
        
        // ABS 시뮬레이션 (급제동 시 작동)
        vehicle_data.abs_active = (vehicle_data.brake_pressure > 800 && 
                                  vehicle_data.vehicle_speed > 30);
        
        // 조향각 시뮬레이션
        int steering_input = analogRead(A2);
        vehicle_data.steering_angle = map(steering_input, 0, 1023, -540, 540);
        
        // 도어 상태 시뮬레이션
        vehicle_data.door_status = digitalRead(3) | (digitalRead(4) << 1) |
                                  (digitalRead(5) << 2) | (digitalRead(6) << 3);
        
        vehicle_data.timestamp = millis();
    }
    
    // CAN 네트워크 진단
    void performCANDiagnostics() {
        Serial.println("=== CAN 네트워크 진단 ===");
        Serial.printf("전송 메시지: %lu\n", messages_sent);
        Serial.printf("수신 메시지: %lu\n", messages_received);
        Serial.printf("오류 발생: %lu\n", errors_detected);
        
        // 버스 점유율 계산
        float bus_utilization = (float)(messages_sent + messages_received) / 
                               (millis() / 1000.0) * 100.0;
        Serial.printf("버스 점유율: %.2f%%\n", bus_utilization);
        
        // 에러율 계산
        float error_rate = (float)errors_detected / 
                          (messages_sent + messages_received) * 100.0;
        Serial.printf("에러율: %.2f%%\n", error_rate);
        
        // 경고 조건 체크
        if(error_rate > 5.0) {
            Serial.println("⚠️ 높은 에러율 감지 - CAN 버스 점검 필요");
        }
        
        if(bus_utilization > 80.0) {
            Serial.println("⚠️ 높은 버스 점유율 - 메시지 주기 조정 필요");
        }
        
        Serial.println("=======================");
    }
    
    // 메인 CAN 루프
    void canMainLoop() {
        // 차량 데이터 시뮬레이션
        simulateVehicleData();
        
        // CAN 메시지 수신
        receiveCANMessages();
        
        // 100ms마다 데이터 전송
        static unsigned long last_send = 0;
        if(millis() - last_send > 100) {
            sendEngineData();
            sendVehicleDynamics();
            sendSafetyData();
            last_send = millis();
        }
        
        // 10초마다 진단
        static unsigned long last_diag = 0;
        if(millis() - last_diag > 10000) {
            performCANDiagnostics();
            last_diag = millis();
        }
    }
};
```

---

## ECU 시뮬레이션

### 엔진 제어 모듈 (ECM) 시뮬레이션

```cpp
/*
 * 엔진 제어 모듈 (ECM) 시뮬레이션
 * 연료 분사, 점화 타이밍, 공연비 제어
 */

class EngineControlModule {
private:
    // 엔진 파라미터
    struct EngineParameters {
        uint16_t rpm;                   // 엔진 RPM
        float throttle_position;        // 스로틀 위치 (%)
        float air_flow_rate;           // 공기 유량 (g/s)
        float fuel_injection_time;     // 연료 분사 시간 (ms)
        float ignition_advance;        // 점화 진각 (도)
        float air_fuel_ratio;          // 공연비 (AFR)
        float manifold_pressure;       // 흡기 매니폴드 압력 (kPa)
        float coolant_temperature;     // 냉각수 온도 (°C)
        float intake_air_temp;         // 흡기 온도 (°C)
        bool knock_detected;           // 노킹 감지
        uint8_t engine_load;           // 엔진 부하 (%)
    };
    
    EngineParameters engine_params;
    
    // 제어 맵 (2D 룩업 테이블)
    const float fuel_map[8][8] = {
        // RPM: 800, 1200, 1600, 2000, 2400, 2800, 3200, 3600
        {2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0}, // Load: 10%
        {3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5}, // Load: 20%
        {3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0}, // Load: 30%
        {4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5}, // Load: 40%
        {4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0}, // Load: 50%
        {5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5}, // Load: 60%
        {5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0}, // Load: 70%
        {6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5}  // Load: 80%
    };
    
    const float ignition_map[8][8] = {
        // 점화 진각 맵 (도)
        {10, 12, 14, 16, 18, 20, 22, 24}, // Load: 10%
        {8,  10, 12, 14, 16, 18, 20, 22}, // Load: 20%
        {6,  8,  10, 12, 14, 16, 18, 20}, // Load: 30%
        {4,  6,  8,  10, 12, 14, 16, 18}, // Load: 40%
        {2,  4,  6,  8,  10, 12, 14, 16}, // Load: 50%
        {0,  2,  4,  6,  8,  10, 12, 14}, // Load: 60%
        {-2, 0,  2,  4,  6,  8,  10, 12}, // Load: 70%
        {-4, -2, 0,  2,  4,  6,  8,  10}  // Load: 80%
    };
    
    // PID 제어기
    struct PIDController {
        float kp, ki, kd;
        float setpoint;
        float integral;
        float previous_error;
        float output_min, output_max;
    };
    
    PIDController afr_controller; // 공연비 제어기
    
public:
    EngineControlModule() {
        // 엔진 파라미터 초기화
        memset(&engine_params, 0, sizeof(engine_params));
        engine_params.coolant_temperature = 90.0;
        engine_params.intake_air_temp = 25.0;
        engine_params.air_fuel_ratio = 14.7; // 이론 공연비
        
        // PID 제어기 초기화
        afr_controller = {1.0, 0.1, 0.05, 14.7, 0, 0, 0.5, 2.0};
    }
    
    // 센서 데이터 읽기
    void readSensors() {
        // RPM 센서 (홀 센서)
        static unsigned long last_pulse = 0;
        static uint16_t pulse_count = 0;
        
        if(digitalRead(2) && (millis() - last_pulse > 5)) {
            pulse_count++;
            last_pulse = millis();
        }
        
        static unsigned long last_rpm_calc = 0;
        if(millis() - last_rpm_calc > 1000) {
            engine_params.rpm = pulse_count * 30; // RPM 계산
            pulse_count = 0;
            last_rpm_calc = millis();
        }
        
        // 스로틀 위치 센서 (TPS)
        int tps_raw = analogRead(A0);
        engine_params.throttle_position = map(tps_raw, 0, 1023, 0, 100);
        
        // 공기 유량 센서 (MAF)
        int maf_raw = analogRead(A1);
        engine_params.air_flow_rate = map(maf_raw, 0, 1023, 0, 400) / 10.0;
        
        // 흡기 매니폴드 압력 센서 (MAP)
        int map_raw = analogRead(A2);
        engine_params.manifold_pressure = map(map_raw, 0, 1023, 20, 105);
        
        // 냉각수 온도 센서 (ECT)
        int ect_raw = analogRead(A3);
        engine_params.coolant_temperature = map(ect_raw, 0, 1023, -40, 150);
        
        // 흡기 온도 센서 (IAT)
        int iat_raw = analogRead(A4);
        engine_params.intake_air_temp = map(iat_raw, 0, 1023, -40, 100);
        
        // 노크 센서
        int knock_raw = analogRead(A5);
        engine_params.knock_detected = (knock_raw > 800);
        
        // 엔진 부하 계산
        if(engine_params.rpm > 0) {
            engine_params.engine_load = (engine_params.air_flow_rate / 
                                        (engine_params.rpm / 100.0)) * 10;
            engine_params.engine_load = constrain(engine_params.engine_load, 0, 100);
        }
    }
    
    // 룩업 테이블에서 값 찾기
    float lookupTable(const float table[8][8], float rpm, float load) {
        // RPM 인덱스 계산
        int rpm_index = constrain((rpm - 800) / 400, 0, 7);
        int load_index = constrain(load / 12.5, 0, 7);
        
        // 선형 보간
        float rpm_frac = ((rpm - 800) / 400.0) - rpm_index;
        float load_frac = (load / 12.5) - load_index;
        
        rpm_frac = constrain(rpm_frac, 0, 1);
        load_frac = constrain(load_frac, 0, 1);
        
        // 4점 보간
        float v00 = table[load_index][rpm_index];
        float v01 = table[load_index][min(rpm_index + 1, 7)];
        float v10 = table[min(load_index + 1, 7)][rpm_index];
        float v11 = table[min(load_index + 1, 7)][min(rpm_index + 1, 7)];
        
        float v0 = v00 + rpm_frac * (v01 - v00);
        float v1 = v10 + rpm_frac * (v11 - v10);
        
        return v0 + load_frac * (v1 - v0);
    }
    
    // 연료 분사 제어
    void calculateFuelInjection() {
        // 기본 연료량 (룩업 테이블)
        float base_fuel = lookupTable(fuel_map, engine_params.rpm, 
                                     engine_params.engine_load);
        
        // 온도 보정
        float temp_correction = 1.0;
        if(engine_params.coolant_temperature < 60) {
            temp_correction = 1.3; // 냉시동 증량
        } else if(engine_params.coolant_temperature > 100) {
            temp_correction = 1.1; // 고온 증량
        }
        
        // 공기 밀도 보정
        float air_density_correction = (engine_params.manifold_pressure / 100.0) *
                                      (298.0 / (engine_params.intake_air_temp + 273.0));
        
        // 최종 연료 분사 시간 계산
        engine_params.fuel_injection_time = base_fuel * temp_correction * 
                                           air_density_correction;
        
        // 공연비 피드백 제어
        float afr_error = 14.7 - engine_params.air_fuel_ratio;
        afr_controller.integral += afr_error;
        float afr_derivative = afr_error - afr_controller.previous_error;
        
        float afr_correction = afr_controller.kp * afr_error +
                              afr_controller.ki * afr_controller.integral +
                              afr_controller.kd * afr_derivative;
        
        afr_correction = constrain(afr_correction, 
                                  afr_controller.output_min, 
                                  afr_controller.output_max);
        
        engine_params.fuel_injection_time *= (1.0 + afr_correction);
        afr_controller.previous_error = afr_error;
        
        // PWM으로 인젝터 제어 (핀 9)
        int pwm_value = map(engine_params.fuel_injection_time * 100, 0, 1000, 0, 255);
        analogWrite(9, pwm_value);
    }
    
    // 점화 타이밍 제어
    void calculateIgnitionTiming() {
        // 기본 점화 진각 (룩업 테이블)
        float base_advance = lookupTable(ignition_map, engine_params.rpm,
                                        engine_params.engine_load);
        
        // 노킹 보정
        static float knock_retard = 0;
        if(engine_params.knock_detected) {
            knock_retard += 2.0; // 2도씩 지각
            knock_retard = min(knock_retard, 15.0); // 최대 15도
            Serial.println("노킹 감지 - 점화 시기 지각");
        } else {
            knock_retard = max(0, knock_retard - 0.5); // 서서히 복원
        }
        
        // 온도 보정
        float temp_correction = 0;
        if(engine_params.coolant_temperature < 60) {
            temp_correction = -5.0; // 냉시동 시 지각
        }
        
        // 최종 점화 진각 계산
        engine_params.ignition_advance = base_advance - knock_retard + temp_correction;
        engine_params.ignition_advance = constrain(engine_params.ignition_advance, 
                                                  -10, 35);
        
        // 점화 코일 제어 (핀 10)
        // 실제로는 크랭크샤프트 위치에 따라 정확한 타이밍으로 제어
        digitalWrite(10, HIGH);
        delayMicroseconds(3000); // 코일 충전 시간
        digitalWrite(10, LOW);   // 점화
    }
    
    // 아이들 속도 제어
    void controlIdleSpeed() {
        const uint16_t target_idle_rpm = 800;
        
        if(engine_params.throttle_position < 5 && // 아이들 상태
           engine_params.rpm < 1200) {
            
            int rpm_error = target_idle_rpm - engine_params.rpm;
            
            // 아이들 에어 컨트롤 밸브 (IACV) 제어
            static int iacv_position = 128; // 중간 위치
            
            if(rpm_error > 50) {
                iacv_position += 5; // 공기량 증가
            } else if(rpm_error < -50) {
                iacv_position -= 5; // 공기량 감소
            }
            
            iacv_position = constrain(iacv_position, 50, 200);
            analogWrite(11, iacv_position); // PWM 제어
            
            Serial.printf("아이들 제어: 목표=%d, 현재=%d, IACV=%d\n",
                         target_idle_rpm, engine_params.rpm, iacv_position);
        }
    }
    
    // 배기가스 재순환 (EGR) 제어
    void controlEGR() {
        float egr_rate = 0;
        
        // 중간 부하에서 EGR 작동
        if(engine_params.engine_load > 30 && engine_params.engine_load < 70 &&
           engine_params.rpm > 1500 && engine_params.rpm < 3000 &&
           engine_params.coolant_temperature > 70) {
            
            egr_rate = map(engine_params.engine_load, 30, 70, 5, 15) / 100.0;
        }
        
        // EGR 밸브 제어 (핀 12)
        int egr_pwm = egr_rate * 255;
        analogWrite(12, egr_pwm);
    }
    
    // 진단 코드 생성
    void generateDiagnosticCodes() {
        static uint16_t dtc_codes[10] = {0}; // 진단 코드 배열
        static uint8_t dtc_count = 0;
        
        // P0100 - 질량 공기 유량 센서 회로 이상
        if(engine_params.air_flow_rate < 0.1 && engine_params.rpm > 1000) {
            dtc_codes[dtc_count++] = 0x0100;
            Serial.println("DTC P0100: MAF 센서 이상");
        }
        
        // P0300 - 다기통 실화 감지
        if(engine_params.rpm > 0 && engine_params.rpm % 100 > 50) { // 시뮬레이션
            dtc_codes[dtc_count++] = 0x0300;
            Serial.println("DTC P0300: 실화 감지");
        }
        
        // P0325 - 노크 센서 회로 이상
        if(engine_params.knock_detected && engine_params.engine_load < 20) {
            dtc_codes[dtc_count++] = 0x0325;
            Serial.println("DTC P0325: 노크 센서 이상");
        }
        
        dtc_count = min(dtc_count, 10);
    }
    
    // ECM 상태 출력
    void printECMStatus() {
        Serial.println("=== 엔진 제어 모듈 상태 ===");
        Serial.printf("RPM: %d\n", engine_params.rpm);
        Serial.printf("스로틀 위치: %.1f%%\n", engine_params.throttle_position);
        Serial.printf("엔진 부하: %d%%\n", engine_params.engine_load);
        Serial.printf("공기 유량: %.1f g/s\n", engine_params.air_flow_rate);
        Serial.printf("연료 분사: %.2f ms\n", engine_params.fuel_injection_time);
        Serial.printf("점화 진각: %.1f도\n", engine_params.ignition_advance);
        Serial.printf("공연비: %.1f:1\n", engine_params.air_fuel_ratio);
        Serial.printf("냉각수 온도: %.1f°C\n", engine_params.coolant_temperature);
        Serial.printf("노킹: %s\n", engine_params.knock_detected ? "감지" : "정상");
        Serial.println("===========================");
    }
    
    // 메인 ECM 루프
    void ecmMainLoop() {
        // 센서 데이터 읽기
        readSensors();
        
        // 제어 알고리즘 실행
        calculateFuelInjection();
        calculateIgnitionTiming();
        controlIdleSpeed();
        controlEGR();
        
        // 진단 코드 생성
        generateDiagnosticCodes();
        
        // 상태 출력 (5초마다)
        static unsigned long last_print = 0;
        if(millis() - last_print > 5000) {
            printECMStatus();
            last_print = millis();
        }
    }
};
```

---

## OBD-II 진단 시스템

### ISO 15765 기반 OBD-II 구현

```cpp
/*
 * OBD-II 진단 시스템 구현
 * ISO 15765-2 (CAN 기반), SAE J1979 표준
 */

#include <mcp_can.h>

class OBDIISystem {
private:
    MCP_CAN* can_bus;
    
    // OBD-II 표준 PID 정의
    enum OBD_PID {
        PID_ENGINE_RPM = 0x0C,
        PID_VEHICLE_SPEED = 0x0D,
        PID_COOLANT_TEMP = 0x05,
        PID_INTAKE_TEMP = 0x0F,
        PID_THROTTLE_POS = 0x11,
        PID_FUEL_LEVEL = 0x2F,
        PID_ENGINE_LOAD = 0x04,
        PID_MAF_RATE = 0x10,
        PID_O2_SENSOR = 0x14,
        PID_FUEL_TRIM_ST = 0x06,
        PID_FUEL_TRIM_LT = 0x07,
        PID_INTAKE_PRESSURE = 0x0B
    };
    
    // OBD-II 서비스 모드
    enum OBD_SERVICE {
        SERVICE_01 = 0x01,  // 현재 데이터 요청
        SERVICE_02 = 0x02,  // 프리즈 프레임 데이터
        SERVICE_03 = 0x03,  // 진단 코드 읽기
        SERVICE_04 = 0x04,  // 진단 코드 삭제
        SERVICE_05 = 0x05,  // O2 센서 모니터링
        SERVICE_06 = 0x06,  // 온보드 모니터링
        SERVICE_07 = 0x07,  // 지속적 진단 코드
        SERVICE_08 = 0x08,  // 온보드 시스템 제어
        SERVICE_09 = 0x09,  // 차량 정보 요청
        SERVICE_0A = 0x0A   // 영구 진단 코드
    };
    
    // 진단 코드 구조체
    struct DiagnosticCode {
        uint16_t code;
        String description;
        bool active;
        bool pending;
        bool permanent;
        unsigned long timestamp;
    };
    
    DiagnosticCode dtc_list[20];
    uint8_t dtc_count = 0;
    
    // OBD-II 데이터
    struct OBDData {
        uint16_t engine_rpm;
        uint8_t vehicle_speed;
        int8_t coolant_temp;
        int8_t intake_temp;
        uint8_t throttle_position;
        uint8_t fuel_level;
        uint8_t engine_load;
        uint16_t maf_rate;
        float o2_voltage;
        int8_t fuel_trim_short;
        int8_t fuel_trim_long;
        uint8_t intake_pressure;
    };
    
    OBDData obd_data;
    
    // ISO-TP (ISO 15765-2) 변수
    uint8_t iso_tp_buffer[4096];
    uint16_t iso_tp_length = 0;
    bool iso_tp_receiving = false;
    
public:
    OBDIISystem(MCP_CAN* can) : can_bus(can) {
        // OBD 데이터 초기화
        memset(&obd_data, 0, sizeof(obd_data));
        obd_data.coolant_temp = 90;
        obd_data.intake_temp = 25;
        obd_data.fuel_level = 75;
        
        // 샘플 진단 코드 설정
        addDiagnosticCode(0x0100, "질량 공기 유량 센서 회로 이상", false);
        addDiagnosticCode(0x0300, "다기통 실화 감지", false);
        addDiagnosticCode(0x0325, "노크 센서 회로 이상", false);
    }
    
    // 진단 코드 추가
    void addDiagnosticCode(uint16_t code, String description, bool active) {
        if(dtc_count < 20) {
            dtc_list[dtc_count].code = code;
            dtc_list[dtc_count].description = description;
            dtc_list[dtc_count].active = active;
            dtc_list[dtc_count].pending = false;
            dtc_list[dtc_count].permanent = false;
            dtc_list[dtc_count].timestamp = millis();
            dtc_count++;
        }
    }
    
    // ISO-TP 단일 프레임 전송
    void sendSingleFrame(uint8_t* data, uint8_t length) {
        uint8_t frame[8] = {0};
        frame[0] = 0x00 | (length & 0x0F); // 단일 프레임 + 길이
        
        for(int i = 0; i < length && i < 7; i++) {
            frame[i + 1] = data[i];
        }
        
        can_bus->sendMsgBuf(0x7E8, 0, 8, frame); // ECU 응답 ID
        
        Serial.print("OBD 응답 전송: ");
        for(int i = 0; i < 8; i++) {
            Serial.printf("%02X ", frame[i]);
        }
        Serial.println();
    }
    
    // ISO-TP 다중 프레임 전송
    void sendMultiFrame(uint8_t* data, uint16_t length) {
        if(length <= 7) {
            sendSingleFrame(data, length);
            return;
        }
        
        // 첫 번째 프레임 (FF)
        uint8_t first_frame[8];
        first_frame[0] = 0x10 | ((length >> 8) & 0x0F); // FF + 상위 길이
        first_frame[1] = length & 0xFF;                   // 하위 길이
        
        for(int i = 0; i < 6; i++) {
            first_frame[i + 2] = data[i];
        }
        
        can_bus->sendMsgBuf(0x7E8, 0, 8, first_frame);
        
        // 연속 프레임들 (CF)
        uint8_t sequence = 1;
        uint16_t offset = 6;
        
        while(offset < length) {
            uint8_t cf_frame[8] = {0};
            cf_frame[0] = 0x20 | (sequence & 0x0F); // CF + 시퀀스
            
            for(int i = 0; i < 7 && offset < length; i++) {
                cf_frame[i + 1] = data[offset++];
            }
            
            can_bus->sendMsgBuf(0x7E8, 0, 8, cf_frame);
            sequence = (sequence + 1) & 0x0F;
            
            delay(10); // CF 간격
        }
    }
    
    // PID 데이터 처리
    uint32_t getPIDData(uint8_t pid) {
        switch(pid) {
            case PID_ENGINE_RPM:
                return obd_data.engine_rpm * 4; // 0.25 RPM/bit
                
            case PID_VEHICLE_SPEED:
                return obd_data.vehicle_speed;
                
            case PID_COOLANT_TEMP:
                return obd_data.coolant_temp + 40; // -40°C offset
                
            case PID_INTAKE_TEMP:
                return obd_data.intake_temp + 40;
                
            case PID_THROTTLE_POS:
                return map(obd_data.throttle_position, 0, 100, 0, 255);
                
            case PID_FUEL_LEVEL:
                return map(obd_data.fuel_level, 0, 100, 0, 255);
                
            case PID_ENGINE_LOAD:
                return map(obd_data.engine_load, 0, 100, 0, 255);
                
            case PID_MAF_RATE:
                return obd_data.maf_rate * 100; // 0.01 g/s/bit
                
            case PID_O2_SENSOR:
                return (uint32_t)(obd_data.o2_voltage * 200); // 0.005V/bit
                
            case PID_FUEL_TRIM_ST:
                return (obd_data.fuel_trim_short + 100) * 128 / 100;
                
            case PID_FUEL_TRIM_LT:
                return (obd_data.fuel_trim_long + 100) * 128 / 100;
                
            case PID_INTAKE_PRESSURE:
                return obd_data.intake_pressure;
                
            default:
                return 0;
        }
    }
    
    // 서비스 01: 현재 데이터 요청
    void handleService01(uint8_t pid) {
        Serial.printf("서비스 01 PID %02X 처리\n", pid);
        
        uint8_t response[6];
        response[0] = 0x41;  // 긍정 응답
        response[1] = pid;   // 요청된 PID
        
        uint32_t pid_data = getPIDData(pid);
        uint8_t data_length = 2;
        
        // PID별 데이터 길이 설정
        switch(pid) {
            case PID_ENGINE_RPM:
                response[2] = (pid_data >> 8) & 0xFF;
                response[3] = pid_data & 0xFF;
                data_length = 4;
                break;
                
            case PID_MAF_RATE:
                response[2] = (pid_data >> 8) & 0xFF;
                response[3] = pid_data & 0xFF;
                data_length = 4;
                break;
                
            case PID_O2_SENSOR:
                response[2] = (pid_data >> 8) & 0xFF;
                response[3] = pid_data & 0xFF;
                data_length = 4;
                break;
                
            default:
                response[2] = pid_data & 0xFF;
                data_length = 3;
                break;
        }
        
        sendSingleFrame(response, data_length);
    }
    
    // 서비스 03: 진단 코드 읽기
    void handleService03() {
        Serial.println("서비스 03: 진단 코드 읽기");
        
        // 활성 DTC 개수 계산
        uint8_t active_count = 0;
        for(int i = 0; i < dtc_count; i++) {
            if(dtc_list[i].active) active_count++;
        }
        
        uint8_t response[64];
        response[0] = 0x43;  // 긍정 응답
        response[1] = active_count;
        
        uint8_t response_length = 2;
        
        // 활성 DTC 추가
        for(int i = 0; i < dtc_count && response_length < 62; i++) {
            if(dtc_list[i].active) {
                // DTC를 OBD-II 형식으로 변환
                uint16_t dtc = dtc_list[i].code;
                
                // 첫 번째 바이트: 시스템 타입 + 상위 2비트
                uint8_t first_byte = ((dtc >> 8) & 0x3F);
                if(dtc >= 0x0000 && dtc <= 0x3FFF) {
                    first_byte |= 0x00; // P 코드
                } else if(dtc >= 0x4000 && dtc <= 0x7FFF) {
                    first_byte |= 0x40; // C 코드
                } else if(dtc >= 0x8000 && dtc <= 0xBFFF) {
                    first_byte |= 0x80; // B 코드
                } else {
                    first_byte |= 0xC0; // U 코드
                }
                
                response[response_length++] = first_byte;
                response[response_length++] = dtc & 0xFF;
                
                Serial.printf("DTC P%04X: %s\n", dtc, dtc_list[i].description.c_str());
            }
        }
        
        if(response_length <= 8) {
            sendSingleFrame(response, response_length);
        } else {
            sendMultiFrame(response, response_length);
        }
    }
    
    // 서비스 04: 진단 코드 삭제
    void handleService04() {
        Serial.println("서비스 04: 진단 코드 삭제");
        
        // 모든 DTC를 비활성화
        for(int i = 0; i < dtc_count; i++) {
            dtc_list[i].active = false;
            dtc_list[i].pending = false;
        }
        
        uint8_t response[1] = {0x44}; // 긍정 응답
        sendSingleFrame(response, 1);
        
        Serial.println("모든 진단 코드가 삭제되었습니다.");
    }
    
    // 서비스 09: 차량 정보 요청
    void handleService09(uint8_t iid) {
        Serial.printf("서비스 09 IID %02X 처리\n", iid);
        
        uint8_t response[32];
        response[0] = 0x49;  // 긍정 응답
        response[1] = iid;   // 요청된 IID
        
        uint8_t response_length = 2;
        
        switch(iid) {
            case 0x02: // VIN (차량 식별 번호)
                {
                    String vin = "KMHXX00XXXX000000"; // 샘플 VIN
                    response[2] = 0x01; // 메시지 개수
                    for(int i = 0; i < 17 && i < 29; i++) {
                        response[3 + i] = vin[i];
                    }
                    response_length = 20;
                }
                break;
                
            case 0x04: // 교정 식별
                {
                    String cal_id = "12345678";
                    for(int i = 0; i < 8; i++) {
                        response[2 + i] = cal_id[i];
                    }
                    response_length = 10;
                }
                break;
                
            case 0x06: // CVN (교정 검증 번호)
                response[2] = 0x12;
                response[3] = 0x34;
                response[4] = 0x56;
                response[5] = 0x78;
                response_length = 6;
                break;
                
            default:
                // 지원하지 않는 IID
                uint8_t nack[2] = {0x7F, 0x09};
                sendSingleFrame(nack, 2);
                return;
        }
        
        if(response_length <= 8) {
            sendSingleFrame(response, response_length);
        } else {
            sendMultiFrame(response, response_length);
        }
    }
    
    // OBD-II 요청 처리
    void processOBDRequest(uint32_t can_id, uint8_t* data, uint8_t length) {
        if(can_id != 0x7DF && can_id != 0x7E0) return; // OBD 요청 ID 확인
        
        // ISO-TP 처리
        uint8_t pci = data[0] & 0xF0; // Protocol Control Information
        
        if(pci == 0x00) { // 단일 프레임
            uint8_t sf_length = data[0] & 0x0F;
            
            if(sf_length >= 2) {
                uint8_t service = data[1];
                uint8_t pid_iid = (sf_length > 2) ? data[2] : 0;
                
                Serial.printf("OBD 요청: 서비스 %02X, PID/IID %02X\n", service, pid_iid);
                
                switch(service) {
                    case SERVICE_01:
                        handleService01(pid_iid);
                        break;
                        
                    case SERVICE_03:
                        handleService03();
                        break;
                        
                    case SERVICE_04:
                        handleService04();
                        break;
                        
                    case SERVICE_09:
                        handleService09(pid_iid);
                        break;
                        
                    default:
                        // 지원하지 않는 서비스
                        uint8_t nack[3] = {0x7F, service, 0x11}; // Service Not Supported
                        sendSingleFrame(nack, 3);
                        Serial.printf("지원하지 않는 서비스: %02X\n", service);
                        break;
                }
            }
        }
    }
    
    // 차량 데이터 업데이트 (시뮬레이션)
    void updateVehicleData() {
        static unsigned long last_update = 0;
        if(millis() - last_update < 100) return;
        
        last_update = millis();
        
        // 엔진 RPM 시뮬레이션
        int throttle_input = analogRead(A0);
        obd_data.engine_rpm = map(throttle_input, 0, 1023, 800, 6000);
        
        // 차량 속도 (RPM 기반)
        obd_data.vehicle_speed = map(obd_data.engine_rpm, 800, 6000, 0, 180);
        
        // 스로틀 위치
        obd_data.throttle_position = map(throttle_input, 0, 1023, 0, 100);
        
        // 엔진 부하
        obd_data.engine_load = map(throttle_input, 0, 1023, 10, 90);
        
        // MAF 센서
        obd_data.maf_rate = map(throttle_input, 0, 1023, 3, 300);
        
        // O2 센서 (공연비 기반)
        obd_data.o2_voltage = 0.1 + (obd_data.engine_load / 100.0) * 0.8;
        
        // 연료 트림
        obd_data.fuel_trim_short = random(-10, 10);
        obd_data.fuel_trim_long = random(-5, 5);
        
        // 흡기 압력
        obd_data.intake_pressure = map(obd_data.engine_load, 0, 100, 30, 100);
        
        // DTC 시뮬레이션
        static unsigned long last_dtc_check = 0;
        if(millis() - last_dtc_check > 5000) {
            // 랜덤 DTC 활성화
            if(random(0, 100) < 5) { // 5% 확률
                int dtc_index = random(0, dtc_count);
                dtc_list[dtc_index].active = true;
                Serial.printf("DTC P%04X 활성화: %s\n", 
                             dtc_list[dtc_index].code, 
                             dtc_list[dtc_index].description.c_str());
            }
            last_dtc_check = millis();
        }
    }
    
    // OBD-II 메인 루프
    void obdMainLoop() {
        updateVehicleData();
        
        // CAN 메시지 확인
        if(!digitalRead(CAN0_INT)) {
            uint32_t rx_id;
            uint8_t rx_len;
            uint8_t rx_data[8];
            
            if(can_bus->readMsgBuf(&rx_id, &rx_len, rx_data) == CAN_OK) {
                processOBDRequest(rx_id, rx_data, rx_len);
            }
        }
    }
};
```

---

## 자동차 센서 네트워크

### 통합 센서 관리 시스템

```cpp
/*
 * 자동차 통합 센서 관리 시스템
 * 다중 센서 데이터 융합 및 실시간 처리
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Wire.h>
#include <MPU6050.h>

class AutomotiveSensorNetwork {
private:
    // 센서 인스턴스
    OneWire oneWire;
    DallasTemperature dallas_temp;
    DHT dht;
    MPU6050 mpu;
    
    // 센서 데이터 구조체
    struct SensorData {
        // 환경 센서
        float ambient_temperature;     // 외부 온도
        float cabin_temperature;       // 실내 온도
        float humidity;               // 습도
        float atmospheric_pressure;    // 대기압
        
        // 엔진 센서
        float coolant_temperature;     // 냉각수 온도
        float oil_temperature;         // 오일 온도
        float oil_pressure;           // 오일 압력
        float fuel_temperature;        // 연료 온도
        
        // 동역학 센서
        float accelerometer[3];        // 가속도 (X, Y, Z)
        float gyroscope[3];           // 각속도 (X, Y, Z)
        float magnetometer[3];        // 자기장 (X, Y, Z)
        
        // 압력 센서
        float tire_pressure[4];        // 타이어 압력
        float brake_pressure;          // 브레이크 압력
        float boost_pressure;          // 부스트 압력
        
        // 위치/속도 센서
        float wheel_speed[4];          // 휠 속도
        float steering_angle;          // 조향각
        float suspension_travel[4];    // 서스펜션 이동량
        
        // 전기 센서
        float battery_voltage;         // 배터리 전압
        float alternator_current;      // 알터네이터 전류
        float fuel_level;             // 연료량
        
        // 품질 지표
        bool sensor_health[32];        // 센서 상태
        uint8_t data_quality;         // 데이터 품질 (0-100)
        unsigned long timestamp;       // 타임스탬프
    };
    
    SensorData sensor_data;
    
    // 센서 융합 필터
    struct KalmanFilter {
        float Q; // 프로세스 노이즈
        float R; // 측정 노이즈
        float P; // 추정 오차
        float K; // 칼만 게인
        float x; // 추정값
    };
    
    KalmanFilter speed_filter;
    KalmanFilter acceleration_filter;
    
    // 센서 보정 데이터
    struct CalibrationData {
        float offset;
        float scale;
        bool calibrated;
    };
    
    CalibrationData sensor_calibration[32];
    
public:
    AutomotiveSensorNetwork() : 
        oneWire(7), 
        dallas_temp(&oneWire),
        dht(8, DHT22),
        speed_filter({0.01, 0.1, 1, 0, 0}),
        acceleration_filter({0.01, 0.5, 1, 0, 0}) {
        
        // 센서 데이터 초기화
        memset(&sensor_data, 0, sizeof(sensor_data));
        
        // 센서 상태 초기화 (모든 센서 양호)
        for(int i = 0; i < 32; i++) {
            sensor_data.sensor_health[i] = true;
            sensor_calibration[i] = {0.0, 1.0, false};
        }
        
        sensor_data.data_quality = 100;
    }
    
    // 전체 센서 초기화
    void initializeSensors() {
        Serial.println("자동차 센서 네트워크 초기화...");
        
        // Dallas 온도 센서 초기화
        dallas_temp.begin();
        int temp_sensors = dallas_temp.getDeviceCount();
        Serial.printf("Dallas 온도 센서: %d개 발견\n", temp_sensors);
        
        // DHT 센서 초기화
        dht.begin();
        Serial.println("DHT 환경 센서 초기화 완료");
        
        // MPU6050 IMU 초기화
        Wire.begin();
        mpu.initialize();
        if(mpu.testConnection()) {
            Serial.println("MPU6050 IMU 연결 확인");
            calibrateIMU();
        } else {
            Serial.println("MPU6050 IMU 연결 실패");
            sensor_data.sensor_health[10] = false;
        }
        
        // 아날로그 센서 핀 설정
        for(int i = A0; i <= A15; i++) {
            pinMode(i, INPUT);
        }
        
        Serial.println("센서 네트워크 초기화 완료");
    }
    
    // IMU 센서 보정
    void calibrateIMU() {
        Serial.println("IMU 센서 보정 중... (5초간 정지 상태 유지)");
        
        float accel_offset[3] = {0};
        float gyro_offset[3] = {0};
        
        const int samples = 1000;
        
        for(int i = 0; i < samples; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            
            accel_offset[0] += ax;
            accel_offset[1] += ay;
            accel_offset[2] += az;
            gyro_offset[0] += gx;
            gyro_offset[1] += gy;
            gyro_offset[2] += gz;
            
            delay(5);
        }
        
        // 평균 계산
        for(int i = 0; i < 3; i++) {
            accel_offset[i] /= samples;
            gyro_offset[i] /= samples;
        }
        
        // 중력 보정 (Z축에서 1g 제거)
        accel_offset[2] -= 16384; // 1g = 16384 (2g 풀스케일)
        
        // 보정값 저장
        sensor_calibration[10].offset = accel_offset[0];
        sensor_calibration[11].offset = accel_offset[1];
        sensor_calibration[12].offset = accel_offset[2];
        sensor_calibration[13].offset = gyro_offset[0];
        sensor_calibration[14].offset = gyro_offset[1];
        sensor_calibration[15].offset = gyro_offset[2];
        
        for(int i = 10; i <= 15; i++) {
            sensor_calibration[i].calibrated = true;
        }
        
        Serial.println("IMU 센서 보정 완료");
    }
    
    // 온도 센서 읽기
    void readTemperatureSensors() {
        // Dallas 온도 센서 (엔진 관련)
        dallas_temp.requestTemperatures();
        
        if(dallas_temp.getDeviceCount() > 0) {
            sensor_data.coolant_temperature = dallas_temp.getTempCByIndex(0);
            sensor_data.sensor_health[0] = (sensor_data.coolant_temperature != DEVICE_DISCONNECTED_C);
        }
        
        if(dallas_temp.getDeviceCount() > 1) {
            sensor_data.oil_temperature = dallas_temp.getTempCByIndex(1);
            sensor_data.sensor_health[1] = (sensor_data.oil_temperature != DEVICE_DISCONNECTED_C);
        }
        
        // DHT 센서 (환경)
        sensor_data.ambient_temperature = dht.readTemperature();
        sensor_data.humidity = dht.readHumidity();
        
        sensor_data.sensor_health[2] = !isnan(sensor_data.ambient_temperature);
        sensor_data.sensor_health[3] = !isnan(sensor_data.humidity);
        
        // 실내 온도 (NTC 써미스터)
        int ntc_raw = analogRead(A0);
        float ntc_voltage = (ntc_raw * 5.0) / 1024.0;
        float ntc_resistance = (10000.0 * ntc_voltage) / (5.0 - ntc_voltage);
        
        // Steinhart-Hart 방정식
        float temp_k = 1.0 / (0.001129148 + 0.000234125 * log(ntc_resistance) + 
                              0.0000000876741 * pow(log(ntc_resistance), 3));
        sensor_data.cabin_temperature = temp_k - 273.15;
        
        sensor_data.sensor_health[4] = (sensor_data.cabin_temperature > -40 && 
                                       sensor_data.cabin_temperature < 100);
    }
    
    // IMU 센서 읽기
    void readIMUSensors() {
        if(!sensor_data.sensor_health[10]) return;
        
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // 보정값 적용 및 단위 변환
        sensor_data.accelerometer[0] = (ax - sensor_calibration[10].offset) / 16384.0; // g
        sensor_data.accelerometer[1] = (ay - sensor_calibration[11].offset) / 16384.0;
        sensor_data.accelerometer[2] = (az - sensor_calibration[12].offset) / 16384.0;
        
        sensor_data.gyroscope[0] = (gx - sensor_calibration[13].offset) / 131.0; // deg/s
        sensor_data.gyroscope[1] = (gy - sensor_calibration[14].offset) / 131.0;
        sensor_data.gyroscope[2] = (gz - sensor_calibration[15].offset) / 131.0;
        
        // 칼만 필터 적용 (가속도)
        float total_accel = sqrt(pow(sensor_data.accelerometer[0], 2) + 
                               pow(sensor_data.accelerometer[1], 2) + 
                               pow(sensor_data.accelerometer[2], 2));
        
        // 필터 업데이트
        acceleration_filter.P += acceleration_filter.Q;
        acceleration_filter.K = acceleration_filter.P / (acceleration_filter.P + acceleration_filter.R);
        acceleration_filter.x += acceleration_filter.K * (total_accel - acceleration_filter.x);
        acceleration_filter.P *= (1 - acceleration_filter.K);
    }
    
    // 압력 센서 읽기
    void readPressureSensors() {
        // 타이어 압력 센서 (4개)
        for(int i = 0; i < 4; i++) {
            int pressure_raw = analogRead(A4 + i);
            float pressure_voltage = (pressure_raw * 5.0) / 1024.0;
            
            // MPX5700AP 압력 센서 (15-700 kPa)
            sensor_data.tire_pressure[i] = (pressure_voltage - 0.2) * 125.0 + 15.0;
            
            // 센서 상태 체크
            sensor_data.sensor_health[16 + i] = (sensor_data.tire_pressure[i] > 0 && 
                                               sensor_data.tire_pressure[i] < 800);
        }
        
        // 브레이크 압력
        int brake_raw = analogRead(A8);
        float brake_voltage = (brake_raw * 5.0) / 1024.0;
        sensor_data.brake_pressure = brake_voltage * 200.0; // 0-1000 kPa
        sensor_data.sensor_health[20] = (sensor_data.brake_pressure >= 0);
        
        // 부스트 압력 (터보)
        int boost_raw = analogRead(A9);
        float boost_voltage = (boost_raw * 5.0) / 1024.0;
        sensor_data.boost_pressure = (boost_voltage - 2.5) * 40.0; // -100 to +100 kPa
        sensor_data.sensor_health[21] = true;
    }
    
    // 휠 속도 센서 읽기
    void readWheelSpeedSensors() {
        // 홀 센서 기반 휠 속도 측정
        static unsigned long wheel_pulse_time[4] = {0};
        static unsigned long last_wheel_pulse[4] = {0};
        
        for(int i = 0; i < 4; i++) {
            if(digitalRead(22 + i) && (millis() - last_wheel_pulse[i] > 10)) {
                wheel_pulse_time[i] = millis() - last_wheel_pulse[i];
                last_wheel_pulse[i] = millis();
                
                // 속도 계산 (휠 둘레: 2m, 펄스 per 회전: 48)
                if(wheel_pulse_time[i] > 0) {
                    float rpm = 60000.0 / (wheel_pulse_time[i] * 48);
                    sensor_data.wheel_speed[i] = rpm * 2.0 * 0.06; // km/h
                }
                
                sensor_data.sensor_health[22 + i] = true;
            }
            
            // 타임아웃 체크 (1초간 펄스 없으면 정지)
            if(millis() - last_wheel_pulse[i] > 1000) {
                sensor_data.wheel_speed[i] = 0;
            }
        }
    }
    
    // 조향각 센서 읽기
    void readSteeringSensor() {
        // 로터리 엔코더 또는 홀 센서
        int steering_raw = analogRead(A10);
        sensor_data.steering_angle = map(steering_raw, 0, 1023, -540, 540); // 도
        
        sensor_data.sensor_health[26] = true;
    }
    
    // 전기 시스템 센서 읽기
    void readElectricalSensors() {
        // 배터리 전압 (분압 회로)
        int battery_raw = analogRead(A11);
        sensor_data.battery_voltage = (battery_raw * 15.0) / 1024.0; // 0-15V 범위
        sensor_data.sensor_health[27] = (sensor_data.battery_voltage > 10.0 && 
                                        sensor_data.battery_voltage < 16.0);
        
        // 알터네이터 전류 (홀 센서)
        int current_raw = analogRead(A12);
        sensor_data.alternator_current = (current_raw - 512) * 0.1; // ±50A
        sensor_data.sensor_health[28] = true;
        
        // 연료량 센서
        int fuel_raw = analogRead(A13);
        sensor_data.fuel_level = map(fuel_raw, 0, 1023, 0, 100); // %
        sensor_data.sensor_health[29] = true;
    }
    
    // 센서 융합 및 검증
    void performSensorFusion() {
        uint8_t healthy_sensors = 0;
        
        // 센서 상태 카운트
        for(int i = 0; i < 32; i++) {
            if(sensor_data.sensor_health[i]) {
                healthy_sensors++;
            }
        }
        
        // 데이터 품질 계산
        sensor_data.data_quality = (healthy_sensors * 100) / 32;
        
        // 크로스 체크 (다중 센서 검증)
        // 예: 휠 속도와 GPS 속도 비교
        float avg_wheel_speed = 0;
        uint8_t valid_wheels = 0;
        
        for(int i = 0; i < 4; i++) {
            if(sensor_data.sensor_health[22 + i]) {
                avg_wheel_speed += sensor_data.wheel_speed[i];
                valid_wheels++;
            }
        }
        
        if(valid_wheels > 0) {
            avg_wheel_speed /= valid_wheels;
            
            // 속도 필터 업데이트
            speed_filter.P += speed_filter.Q;
            speed_filter.K = speed_filter.P / (speed_filter.P + speed_filter.R);
            speed_filter.x += speed_filter.K * (avg_wheel_speed - speed_filter.x);
            speed_filter.P *= (1 - speed_filter.K);
        }
        
        // 이상치 검출
        detectAnomalies();
    }
    
    // 이상치 검출
    void detectAnomalies() {
        // 온도 이상
        if(sensor_data.coolant_temperature > 110.0) {
            Serial.println("경고: 냉각수 과열!");
        }
        
        // 압력 이상
        for(int i = 0; i < 4; i++) {
            if(sensor_data.tire_pressure[i] < 180.0) {
                Serial.printf("경고: 타이어 %d 저압 (%.1f kPa)\n", i+1, sensor_data.tire_pressure[i]);
            }
        }
        
        // 전압 이상
        if(sensor_data.battery_voltage < 11.5) {
            Serial.printf("경고: 배터리 저전압 (%.1fV)\n", sensor_data.battery_voltage);
        } else if(sensor_data.battery_voltage > 14.8) {
            Serial.printf("경고: 과충전 (%.1fV)\n", sensor_data.battery_voltage);
        }
        
        // 가속도 이상 (충돌 감지)
        float total_g = sqrt(pow(sensor_data.accelerometer[0], 2) + 
                           pow(sensor_data.accelerometer[1], 2) + 
                           pow(sensor_data.accelerometer[2], 2));
        
        if(total_g > 4.0) {
            Serial.printf("경고: 높은 가속도 감지 (%.1fG) - 충돌 가능성\n", total_g);
        }
    }
    
    // 센서 데이터 출력
    void printSensorData() {
        Serial.println("=== 자동차 센서 네트워크 상태 ===");
        Serial.printf("데이터 품질: %d%%\n", sensor_data.data_quality);
        
        Serial.println("--- 온도 센서 ---");
        Serial.printf("냉각수: %.1f°C, 오일: %.1f°C, 외부: %.1f°C, 실내: %.1f°C\n",
                     sensor_data.coolant_temperature, sensor_data.oil_temperature,
                     sensor_data.ambient_temperature, sensor_data.cabin_temperature);
        
        Serial.println("--- 압력 센서 ---");
        Serial.printf("타이어: %.1f %.1f %.1f %.1f kPa, 브레이크: %.1f kPa\n",
                     sensor_data.tire_pressure[0], sensor_data.tire_pressure[1],
                     sensor_data.tire_pressure[2], sensor_data.tire_pressure[3],
                     sensor_data.brake_pressure);
        
        Serial.println("--- 동역학 센서 ---");
        Serial.printf("가속도: %.2f %.2f %.2f G, 각속도: %.1f %.1f %.1f °/s\n",
                     sensor_data.accelerometer[0], sensor_data.accelerometer[1], sensor_data.accelerometer[2],
                     sensor_data.gyroscope[0], sensor_data.gyroscope[1], sensor_data.gyroscope[2]);
        
        Serial.println("--- 휠 속도 ---");
        Serial.printf("휠: %.1f %.1f %.1f %.1f km/h, 조향각: %.1f°\n",
                     sensor_data.wheel_speed[0], sensor_data.wheel_speed[1],
                     sensor_data.wheel_speed[2], sensor_data.wheel_speed[3],
                     sensor_data.steering_angle);
        
        Serial.println("--- 전기 시스템 ---");
        Serial.printf("배터리: %.1fV, 전류: %.1fA, 연료: %d%%\n",
                     sensor_data.battery_voltage, sensor_data.alternator_current,
                     (int)sensor_data.fuel_level);
        
        Serial.println("================================");
    }
    
    // 메인 센서 루프
    void sensorMainLoop() {
        // 센서 데이터 읽기
        readTemperatureSensors();
        readIMUSensors();
        readPressureSensors();
        readWheelSpeedSensors();
        readSteeringSensor();
        readElectricalSensors();
        
        // 센서 융합 및 검증
        performSensorFusion();
        
        // 타임스탬프 업데이트
        sensor_data.timestamp = millis();
        
        // 5초마다 데이터 출력
        static unsigned long last_print = 0;
        if(millis() - last_print > 5000) {
            printSensorData();
            last_print = millis();
        }
    }
    
    // 센서 데이터 접근자
    const SensorData& getSensorData() const {
        return sensor_data;
    }
};
```

계속해서 ADAS 시스템과 나머지 섹션들을 구현하겠습니다...

---

## ADAS 시스템

### 첨단 운전자 보조 시스템 구현

```cpp
/*
 * ADAS (Advanced Driver Assistance Systems) 구현
 * 충돌 방지, 차선 유지, 어댑티브 크루즈 컨트롤
 */

#include <Servo.h>
#include <NewPing.h>

class ADASSystem {
private:
    // ADAS 센서 인스턴스
    NewPing front_sonar;
    NewPing rear_sonar;
    NewPing left_sonar;
    NewPing right_sonar;
    
    Servo steering_servo;
    Servo brake_servo;
    Servo throttle_servo;
    
    // ADAS 상태 구조체
    struct ADASState {
        // 충돌 방지 시스템 (Forward Collision Warning)
        bool fcw_active;
        float front_distance;
        float relative_speed;
        float time_to_collision;
        
        // 자동 비상 제동 (Automatic Emergency Braking)
        bool aeb_active;
        uint8_t brake_force;
        
        // 어댑티브 크루즈 컨트롤 (Adaptive Cruise Control)
        bool acc_active;
        uint8_t target_speed;
        uint8_t current_speed;
        float following_distance;
        
        // 차선 유지 보조 (Lane Keeping Assist)
        bool lka_active;
        float lane_center_offset;
        float steering_correction;
        
        // 사각지대 모니터링 (Blind Spot Monitoring)
        bool bsm_left_active;
        bool bsm_right_active;
        float left_distance;
        float right_distance;
        
        // 주차 보조 (Parking Assist)
        bool parking_mode;
        float parking_space_length;
        uint8_t parking_step;
        
        // 시스템 상태
        bool system_enabled;
        uint8_t warning_level; // 0: 정상, 1: 주의, 2: 경고, 3: 위험
        unsigned long last_update;
    };
    
    ADASState adas_state;
    
    // PID 제어기 (크루즈 컨트롤용)
    struct PIDController {
        float kp, ki, kd;
        float setpoint;
        float integral;
        float previous_error;
        float output_min, output_max;
    };
    
    PIDController speed_controller;
    PIDController lane_controller;
    
    // 칼만 필터 (거리 추정용)
    struct KalmanFilter {
        float Q, R, P, K, x;
    };
    
    KalmanFilter distance_filter;
    
public:
    ADASSystem() : 
        front_sonar(30, 31, 400),  // Trig, Echo, Max distance
        rear_sonar(32, 33, 400),
        left_sonar(34, 35, 400),
        right_sonar(36, 37, 400),
        speed_controller({2.0, 0.1, 0.5, 60, 0, 0, 0, 100}),
        lane_controller({1.5, 0.05, 0.3, 0, 0, 0, -30, 30}),
        distance_filter({0.1, 1.0, 1.0, 0, 100}) {
        
        // ADAS 상태 초기화
        memset(&adas_state, 0, sizeof(adas_state));
        adas_state.system_enabled = true;
        adas_state.target_speed = 60; // 60 km/h 기본값
        adas_state.following_distance = 30.0; // 30m 기본 추종거리
    }
    
    // ADAS 시스템 초기화
    void initializeADAS() {
        Serial.println("ADAS 시스템 초기화...");
        
        // 서보 모터 초기화
        steering_servo.attach(9);
        brake_servo.attach(10);
        throttle_servo.attach(11);
        
        // 초기 위치 설정
        steering_servo.write(90);  // 중앙
        brake_servo.write(0);      // 브레이크 해제
        throttle_servo.write(90);  // 중립
        
        // 카메라 모듈 초기화 (시뮬레이션)
        pinMode(A14, INPUT); // 좌측 차선 센서
        pinMode(A15, INPUT); // 우측 차선 센서
        
        // LED 경고등 초기화
        pinMode(LED_BUILTIN, OUTPUT);
        pinMode(12, OUTPUT); // FCW 경고등
        pinMode(13, OUTPUT); // AEB 경고등
        
        Serial.println("ADAS 시스템 초기화 완료");
    }
    
    // 전방 충돌 경고 시스템
    void processForwardCollisionWarning() {
        // 전방 거리 측정
        float raw_distance = front_sonar.ping_cm();
        
        if(raw_distance == 0) raw_distance = 400; // 측정 한계
        
        // 칼만 필터 적용
        distance_filter.P += distance_filter.Q;
        distance_filter.K = distance_filter.P / (distance_filter.P + distance_filter.R);
        distance_filter.x += distance_filter.K * (raw_distance - distance_filter.x);
        distance_filter.P *= (1 - distance_filter.K);
        
        adas_state.front_distance = distance_filter.x;
        
        // 상대속도 계산 (이전 거리와 비교)
        static float previous_distance = 400;
        static unsigned long previous_time = 0;
        
        unsigned long current_time = millis();
        float time_delta = (current_time - previous_time) / 1000.0; // 초
        
        if(time_delta > 0.1) { // 100ms마다 계산
            adas_state.relative_speed = (previous_distance - adas_state.front_distance) / time_delta;
            previous_distance = adas_state.front_distance;
            previous_time = current_time;
        }
        
        // TTC (Time To Collision) 계산
        if(adas_state.relative_speed > 0) {
            adas_state.time_to_collision = adas_state.front_distance / adas_state.relative_speed;
        } else {
            adas_state.time_to_collision = 999; // 무한대
        }
        
        // FCW 활성화 조건
        adas_state.fcw_active = (adas_state.time_to_collision < 2.5 && 
                                adas_state.front_distance < 50.0 &&
                                adas_state.current_speed > 30);
        
        // 경고 레벨 설정
        if(adas_state.time_to_collision < 1.0) {
            adas_state.warning_level = 3; // 위험
        } else if(adas_state.time_to_collision < 2.0) {
            adas_state.warning_level = 2; // 경고
        } else if(adas_state.time_to_collision < 2.5) {
            adas_state.warning_level = 1; // 주의
        } else {
            adas_state.warning_level = 0; // 정상
        }
        
        // 경고등 제어
        digitalWrite(12, adas_state.fcw_active);
        
        if(adas_state.fcw_active) {
            Serial.printf("FCW 활성: 전방거리 %.1fm, TTC %.1fs\n", 
                         adas_state.front_distance, adas_state.time_to_collision);
        }
    }
    
    // 자동 비상 제동 시스템
    void processAutomaticEmergencyBraking() {
        // AEB 활성화 조건 (더 위급한 상황)
        adas_state.aeb_active = (adas_state.time_to_collision < 1.5 && 
                                adas_state.front_distance < 30.0 &&
                                adas_state.current_speed > 20);
        
        if(adas_state.aeb_active) {
            // 제동력 계산 (TTC에 반비례)
            if(adas_state.time_to_collision < 0.5) {
                adas_state.brake_force = 100; // 최대 제동
            } else {
                adas_state.brake_force = map(adas_state.time_to_collision * 10, 5, 15, 100, 30);
            }
            
            // 브레이크 서보 제어
            brake_servo.write(adas_state.brake_force);
            
            // 스로틀 차단
            throttle_servo.write(0);
            
            // 경고등 점멸
            digitalWrite(13, (millis() / 200) % 2);
            
            Serial.printf("AEB 활성: 제동력 %d%%, TTC %.1fs\n", 
                         adas_state.brake_force, adas_state.time_to_collision);
        } else {
            adas_state.brake_force = 0;
            digitalWrite(13, LOW);
        }
    }
    
    // 어댑티브 크루즈 컨트롤
    void processAdaptiveCruiseControl() {
        if(!adas_state.acc_active) return;
        
        // 목표 속도 설정 (앞차와의 거리 기반)
        uint8_t adjusted_target_speed = adas_state.target_speed;
        
        if(adas_state.front_distance < adas_state.following_distance) {
            // 앞차가 너무 가까우면 속도 감소
            float speed_reduction = (adas_state.following_distance - adas_state.front_distance) / 
                                   adas_state.following_distance * 30; // 최대 30km/h 감속
            adjusted_target_speed = max(20, adas_state.target_speed - (int)speed_reduction);
        }
        
        // PID 제어기로 속도 제어
        float speed_error = adjusted_target_speed - adas_state.current_speed;
        
        speed_controller.integral += speed_error;
        speed_controller.integral = constrain(speed_controller.integral, -100, 100);
        
        float speed_derivative = speed_error - speed_controller.previous_error;
        speed_controller.previous_error = speed_error;
        
        float throttle_output = speed_controller.kp * speed_error +
                               speed_controller.ki * speed_controller.integral +
                               speed_controller.kd * speed_derivative;
        
        throttle_output = constrain(throttle_output, 0, 100);
        
        // 스로틀 제어 (AEB가 비활성화된 경우만)
        if(!adas_state.aeb_active) {
            throttle_servo.write(map(throttle_output, 0, 100, 90, 180));
        }
        
        Serial.printf("ACC: 목표속도 %d km/h, 현재속도 %d km/h, 스로틀 %.1f%%\n",
                     adjusted_target_speed, adas_state.current_speed, throttle_output);
    }
    
    // 차선 유지 보조 시스템
    void processLaneKeepingAssist() {
        if(!adas_state.lka_active) return;
        
        // 차선 센서 읽기 (카메라 대신 간단한 센서 사용)
        int left_lane_sensor = analogRead(A14);
        int right_lane_sensor = analogRead(A15);
        
        // 차선 중앙으로부터의 오프셋 계산
        float left_distance = map(left_lane_sensor, 0, 1023, 0, 200); // cm
        float right_distance = map(right_lane_sensor, 0, 1023, 0, 200);
        
        // 차선 폭 3.5m 가정
        float lane_width = 350; // cm
        float expected_center = lane_width / 2;
        
        adas_state.lane_center_offset = (left_distance - expected_center);
        
        // PID 제어기로 조향 보정
        float lane_error = -adas_state.lane_center_offset; // 오프셋의 반대 방향으로 보정
        
        lane_controller.integral += lane_error;
        lane_controller.integral = constrain(lane_controller.integral, -1000, 1000);
        
        float lane_derivative = lane_error - lane_controller.previous_error;
        lane_controller.previous_error = lane_error;
        
        adas_state.steering_correction = lane_controller.kp * lane_error +
                                        lane_controller.ki * lane_controller.integral +
                                        lane_controller.kd * lane_derivative;
        
        adas_state.steering_correction = constrain(adas_state.steering_correction, -30, 30);
        
        // 조향 제어 (위험 상황이 아닌 경우만)
        if(adas_state.warning_level < 2) {
            int steering_angle = 90 + adas_state.steering_correction;
            steering_servo.write(constrain(steering_angle, 60, 120));
        }
        
        // 차선 이탈 경고
        if(abs(adas_state.lane_center_offset) > 50) {
            digitalWrite(LED_BUILTIN, (millis() / 300) % 2); // 점멸
            
            if(abs(adas_state.lane_center_offset) > 80) {
                Serial.println("경고: 차선 이탈 위험!");
            }
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
        
        Serial.printf("LKA: 오프셋 %.1fcm, 조향보정 %.1f도\n",
                     adas_state.lane_center_offset, adas_state.steering_correction);
    }
    
    // 사각지대 모니터링 시스템
    void processBlindSpotMonitoring() {
        // 좌우측 거리 측정
        adas_state.left_distance = left_sonar.ping_cm();
        adas_state.right_distance = right_sonar.ping_cm();
        
        if(adas_state.left_distance == 0) adas_state.left_distance = 400;
        if(adas_state.right_distance == 0) adas_state.right_distance = 400;
        
        // 사각지대 활성화 조건 (50cm ~ 3m 범위)
        adas_state.bsm_left_active = (adas_state.left_distance > 50 && 
                                     adas_state.left_distance < 300 &&
                                     adas_state.current_speed > 30);
        
        adas_state.bsm_right_active = (adas_state.right_distance > 50 && 
                                      adas_state.right_distance < 300 &&
                                      adas_state.current_speed > 30);
        
        // 경고 출력
        if(adas_state.bsm_left_active) {
            Serial.printf("BSM: 좌측 사각지대 차량 감지 (%.1fm)\n", adas_state.left_distance/100.0);
        }
        
        if(adas_state.bsm_right_active) {
            Serial.printf("BSM: 우측 사각지대 차량 감지 (%.1fm)\n", adas_state.right_distance/100.0);
        }
    }
    
    // 자동 주차 보조 시스템
    void processParkingAssist() {
        if(!adas_state.parking_mode) return;
        
        static unsigned long parking_start_time = 0;
        
        switch(adas_state.parking_step) {
            case 0: // 주차 공간 탐지
                {
                    adas_state.parking_space_length = right_sonar.ping_cm();
                    
                    if(adas_state.parking_space_length > 600) { // 6m 이상 공간
                        adas_state.parking_step = 1;
                        parking_start_time = millis();
                        Serial.println("주차: 적절한 공간 발견, 후진 시작");
                    }
                }
                break;
                
            case 1: // 후진 및 조향
                {
                    // 후진
                    throttle_servo.write(60); // 천천히 후진
                    
                    // 조향각 계산 (45도 조향)
                    steering_servo.write(135);
                    
                    if(millis() - parking_start_time > 3000) { // 3초 후진
                        adas_state.parking_step = 2;
                        parking_start_time = millis();
                    }
                }
                break;
                
            case 2: // 조향 반대로 돌려 차량 정렬
                {
                    steering_servo.write(45); // 반대 조향
                    
                    if(millis() - parking_start_time > 2000) { // 2초
                        adas_state.parking_step = 3;
                    }
                }
                break;
                
            case 3: // 주차 완료
                {
                    throttle_servo.write(90); // 정지
                    steering_servo.write(90); // 조향 중앙
                    
                    adas_state.parking_mode = false;
                    adas_state.parking_step = 0;
                    
                    Serial.println("주차: 자동 주차 완료");
                }
                break;
        }
    }
    
    // 차량 속도 업데이트 (외부에서 호출)
    void updateVehicleSpeed(uint8_t speed) {
        adas_state.current_speed = speed;
    }
    
    // ADAS 기능 토글
    void toggleACC() {
        adas_state.acc_active = !adas_state.acc_active;
        Serial.printf("ACC %s\n", adas_state.acc_active ? "활성화" : "비활성화");
    }
    
    void toggleLKA() {
        adas_state.lka_active = !adas_state.lka_active;
        Serial.printf("LKA %s\n", adas_state.lka_active ? "활성화" : "비활성화");
    }
    
    void startParkingAssist() {
        adas_state.parking_mode = true;
        adas_state.parking_step = 0;
        Serial.println("자동 주차 보조 시작");
    }
    
    // 크루즈 컨트롤 속도 설정
    void setCruiseSpeed(uint8_t speed) {
        adas_state.target_speed = constrain(speed, 30, 130);
        Serial.printf("크루즈 목표 속도: %d km/h\n", adas_state.target_speed);
    }
    
    // ADAS 상태 출력
    void printADASStatus() {
        Serial.println("=== ADAS 시스템 상태 ===");
        Serial.printf("시스템 활성화: %s\n", adas_state.system_enabled ? "YES" : "NO");
        Serial.printf("경고 레벨: %d\n", adas_state.warning_level);
        Serial.printf("현재 속도: %d km/h\n", adas_state.current_speed);
        
        Serial.println("--- 충돌 방지 ---");
        Serial.printf("FCW: %s, AEB: %s\n", 
                     adas_state.fcw_active ? "활성" : "비활성",
                     adas_state.aeb_active ? "활성" : "비활성");
        Serial.printf("전방 거리: %.1fm, TTC: %.1fs\n",
                     adas_state.front_distance, adas_state.time_to_collision);
        
        Serial.println("--- 크루즈 컨트롤 ---");
        Serial.printf("ACC: %s, 목표속도: %d km/h\n",
                     adas_state.acc_active ? "활성" : "비활성",
                     adas_state.target_speed);
        
        Serial.println("--- 차선 유지 ---");
        Serial.printf("LKA: %s, 오프셋: %.1fcm\n",
                     adas_state.lka_active ? "활성" : "비활성",
                     adas_state.lane_center_offset);
        
        Serial.println("--- 사각지대 ---");
        Serial.printf("좌측: %s (%.1fm), 우측: %s (%.1fm)\n",
                     adas_state.bsm_left_active ? "감지" : "정상",
                     adas_state.left_distance/100.0,
                     adas_state.bsm_right_active ? "감지" : "정상",
                     adas_state.right_distance/100.0);
        
        Serial.println("========================");
    }
    
    // 메인 ADAS 루프
    void adasMainLoop() {
        if(!adas_state.system_enabled) return;
        
        // 각 ADAS 기능 처리
        processForwardCollisionWarning();
        processAutomaticEmergencyBraking();
        processAdaptiveCruiseControl();
        processLaneKeepingAssist();
        processBlindSpotMonitoring();
        processParkingAssist();
        
        // 타임스탬프 업데이트
        adas_state.last_update = millis();
        
        // 상태 출력 (10초마다)
        static unsigned long last_print = 0;
        if(millis() - last_print > 10000) {
            printADASStatus();
            last_print = millis();
        }
    }
    
    // ADAS 상태 접근자
    const ADASState& getADASState() const {
        return adas_state;
    }
};
```

이제 GitHub에 업로드하겠습니다!
