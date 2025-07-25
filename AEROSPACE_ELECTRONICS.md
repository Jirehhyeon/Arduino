# 항공우주 전자 시스템 완전 가이드 🚀

## 개요
항공우주 산업에서 사용되는 전자 시스템을 Arduino를 통해 학습하고 구현하는 완전 가이드입니다.

## 목차
1. [항공전자공학 기초](#항공전자공학-기초)
2. [드론 비행 제어 시스템](#드론-비행-제어-시스템)
3. [위성 통신 시스템](#위성-통신-시스템)
4. [항공기 센서 시스템](#항공기-센서-시스템)
5. [로켓 제어 시스템](#로켓-제어-시스템)
6. [항공우주 데이터 획득](#항공우주-데이터-획득)
7. [무인항공기(UAV) 시스템](#무인항공기uav-시스템)
8. [우주선 생명유지 시스템](#우주선-생명유지-시스템)
9. [지상 관제 시스템](#지상-관제-시스템)
10. [통합 항공우주 시스템](#통합-항공우주-시스템)

---

## 항공전자공학 기초

### 기본 개념
- 항공전자공학(Avionics): Aviation + Electronics
- 극한 환경에서의 안정성 요구
- 실시간 시스템 설계
- 안전성과 중복성(Redundancy)

### 핵심 원리
```cpp
// 항공우주 시스템 기본 구조
class AerospaceSystem {
private:
    enum SystemState {
        INITIALIZATION = 0,
        STANDBY = 1,
        OPERATIONAL = 2,
        EMERGENCY = 3,
        SHUTDOWN = 4
    };
    
    enum SafetyLevel {
        SAFETY_CRITICAL = 1,   // 생명에 직결
        SAFETY_RELATED = 2,    // 안전에 관련
        NON_SAFETY = 3         // 비안전
    };
    
    struct SystemHealth {
        bool power_ok;
        bool communication_ok;
        bool sensors_ok;
        float temperature_C;
        float pressure_Pa;
        uint8_t error_count;
        uint32_t uptime_seconds;
    };
    
public:
    void initialize() {
        Serial.println("=== 항공우주 시스템 초기화 ===");
        Serial.println("시스템 자가진단 시작...");
        
        // 중요 시스템 순차 초기화
        initializePowerSystems();
        initializeCommunication();
        initializeSensors();
        initializeActuators();
        initializeSafetySystems();
        
        current_state = OPERATIONAL;
        Serial.println("시스템 초기화 완료 - 운용 준비");
    }
    
private:
    SystemState current_state;
    SystemHealth health_status;
    uint32_t last_health_check;
    
    void initializePowerSystems() {
        Serial.println("전력 시스템 초기화...");
        // 이중화 전력 시스템 체크
    }
    
    void initializeCommunication() {
        Serial.println("통신 시스템 초기화...");
        // 다중 통신 채널 설정
    }
    
    void initializeSensors() {
        Serial.println("센서 시스템 초기화...");
        // 센서 배열 초기화 및 캘리브레이션
    }
    
    void initializeActuators() {
        Serial.println("구동기 시스템 초기화...");
        // 서보, 모터 등 구동기 초기화
    }
    
    void initializeSafetySystems() {
        Serial.println("안전 시스템 초기화...");
        // 비상 시스템 및 안전장치 초기화
    }
};
```

---

## 드론 비행 제어 시스템

### PID 기반 자세 제어
```cpp
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

class DroneFlightController {
private:
    struct PIDController {
        float kp, ki, kd;
        float previous_error;
        float integral;
        float max_output;
        
        float calculate(float setpoint, float measured_value, float dt) {
            float error = setpoint - measured_value;
            
            // 비례항
            float proportional = kp * error;
            
            // 적분항
            integral += error * dt;
            float integral_term = ki * integral;
            
            // 미분항
            float derivative = (error - previous_error) / dt;
            float derivative_term = kd * derivative;
            
            previous_error = error;
            
            float output = proportional + integral_term + derivative_term;
            return constrain(output, -max_output, max_output);
        }
    };
    
    struct FlightData {
        float roll, pitch, yaw;          // 현재 자세 (도)
        float roll_rate, pitch_rate, yaw_rate; // 각속도 (도/초)
        float altitude;                   // 고도 (m)
        float velocity_x, velocity_y, velocity_z; // 속도 (m/s)
        bool gps_lock;                   // GPS 상태
        uint8_t satellite_count;         // 위성 수
        float battery_voltage;           // 배터리 전압
    };
    
    enum FlightMode {
        STABILIZE = 0,    // 안정화 모드
        ALT_HOLD = 1,     // 고도 유지
        LOITER = 2,       // 위치 유지
        RTL = 3,          // 자동 귀환
        AUTO = 4,         // 자동 비행
        LAND = 5          // 착륙
    };
    
    MPU6050 mpu;
    Servo motor1, motor2, motor3, motor4; // 쿼드콥터 모터
    
    PIDController roll_pid = {2.0, 0.1, 0.5, 0, 0, 400};
    PIDController pitch_pid = {2.0, 0.1, 0.5, 0, 0, 400};
    PIDController yaw_pid = {1.5, 0.05, 0.3, 0, 0, 200};
    PIDController altitude_pid = {3.0, 0.2, 1.0, 0, 0, 500};
    
    FlightData flight_data;
    FlightMode current_mode;
    uint32_t last_update;
    
public:
    void initialize() {
        Serial.begin(115200);
        Wire.begin();
        
        Serial.println("=== 드론 비행 제어 시스템 초기화 ===");
        
        // MPU6050 초기화
        if (mpu.begin()) {
            Serial.println("IMU 센서 초기화 성공");
            mpu.calibrateGyro();
            mpu.setThreshold(3);
        } else {
            Serial.println("IMU 센서 초기화 실패!");
            return;
        }
        
        // 모터 초기화
        motor1.attach(3);  // 전면 좌측
        motor2.attach(5);  // 전면 우측
        motor3.attach(6);  // 후면 좌측
        motor4.attach(9);  // 후면 우측
        
        // 모터 안전 초기화 (최소 신호)
        for (int i = 0; i < 4; i++) {
            setMotorSpeed(i, 1000); // 최소 PWM 신호
        }
        delay(2000); // ESC 캘리브레이션 대기
        
        current_mode = STABILIZE;
        last_update = millis();
        
        Serial.println("드론 시스템 초기화 완료");
        Serial.println("비행 준비 상태");
    }
    
    void update() {
        uint32_t current_time = millis();
        float dt = (current_time - last_update) / 1000.0;
        last_update = current_time;
        
        // 센서 데이터 읽기
        readSensors();
        
        // 비행 모드에 따른 제어
        switch (current_mode) {
            case STABILIZE:
                stabilizeMode(dt);
                break;
            case ALT_HOLD:
                altitudeHoldMode(dt);
                break;
            case LOITER:
                loiterMode(dt);
                break;
            case RTL:
                returnToLaunchMode(dt);
                break;
            case AUTO:
                autoMode(dt);
                break;
            case LAND:
                landMode(dt);
                break;
        }
        
        // 모터 출력 및 안전 체크
        updateMotors();
        
        // 텔레메트리 출력
        sendTelemetry();
    }
    
private:
    void readSensors() {
        Vector normAccel = mpu.readNormalizeAccel();
        Vector normGyro = mpu.readNormalizeGyro();
        
        // 자세 계산 (상보 필터)
        float accel_roll = atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0 / PI;
        float accel_pitch = atan2(-normAccel.XAxis, 
                                 sqrt(normAccel.YAxis * normAccel.YAxis + 
                                      normAccel.ZAxis * normAccel.ZAxis)) * 180.0 / PI;
        
        float alpha = 0.98; // 상보 필터 계수
        flight_data.roll = alpha * (flight_data.roll + normGyro.XAxis * 0.01) + 
                          (1 - alpha) * accel_roll;
        flight_data.pitch = alpha * (flight_data.pitch + normGyro.YAxis * 0.01) + 
                           (1 - alpha) * accel_pitch;
        flight_data.yaw += normGyro.ZAxis * 0.01;
        
        flight_data.roll_rate = normGyro.XAxis;
        flight_data.pitch_rate = normGyro.YAxis;
        flight_data.yaw_rate = normGyro.ZAxis;
        
        // 배터리 전압 모니터링
        flight_data.battery_voltage = analogRead(A0) * (5.0 / 1023.0) * 4.0; // 분압비 고려
    }
    
    void stabilizeMode(float dt) {
        // 목표 자세 (RC 입력으로부터)
        float target_roll = 0;    // RC 채널에서 읽어온 값
        float target_pitch = 0;   // RC 채널에서 읽어온 값
        float target_yaw_rate = 0; // RC 채널에서 읽어온 값
        
        // PID 제어 계산
        float roll_output = roll_pid.calculate(target_roll, flight_data.roll, dt);
        float pitch_output = pitch_pid.calculate(target_pitch, flight_data.pitch, dt);
        float yaw_output = yaw_pid.calculate(target_yaw_rate, flight_data.yaw_rate, dt);
        
        // 기본 스로틀 (RC에서 읽어온 값)
        int base_throttle = 1200; // RC 스로틀 입력
        
        // 모터 믹싱 (쿼드콥터 X 형태)
        int motor1_speed = base_throttle - roll_output + pitch_output - yaw_output;
        int motor2_speed = base_throttle + roll_output + pitch_output + yaw_output;
        int motor3_speed = base_throttle - roll_output - pitch_output + yaw_output;
        int motor4_speed = base_throttle + roll_output - pitch_output - yaw_output;
        
        // 모터 속도 제한
        motor1_speed = constrain(motor1_speed, 1000, 2000);
        motor2_speed = constrain(motor2_speed, 1000, 2000);
        motor3_speed = constrain(motor3_speed, 1000, 2000);
        motor4_speed = constrain(motor4_speed, 1000, 2000);
        
        // 모터 출력
        setMotorSpeed(0, motor1_speed);
        setMotorSpeed(1, motor2_speed);
        setMotorSpeed(2, motor3_speed);
        setMotorSpeed(3, motor4_speed);
    }
    
    void altitudeHoldMode(float dt) {
        // 고도 센서 데이터 읽기 (바로미터, 초음파 센서 등)
        float current_altitude = readAltitude();
        float target_altitude = 10.0; // 목표 고도 10m
        
        float altitude_correction = altitude_pid.calculate(target_altitude, current_altitude, dt);
        
        // 기본 자세 제어 + 고도 제어
        stabilizeMode(dt);
        
        // 모든 모터에 고도 보정값 추가
        adjustAllMotors(altitude_correction);
    }
    
    void loiterMode(float dt) {
        // GPS 기반 위치 유지
        if (flight_data.gps_lock) {
            // GPS 위치 제어 로직
            altitudeHoldMode(dt);
        } else {
            // GPS 신호 없으면 고도 유지만
            altitudeHoldMode(dt);
        }
    }
    
    void returnToLaunchMode(float dt) {
        // 이륙 지점으로 자동 귀환
        Serial.println("자동 귀환 모드 실행 중...");
        // GPS 기반 웨이포인트 네비게이션
    }
    
    void autoMode(float dt) {
        // 미션 플래너에 따른 자동 비행
        Serial.println("자동 비행 모드 실행 중...");
        // 웨이포인트 기반 자동 비행
    }
    
    void landMode(float dt) {
        // 자동 착륙 시퀀스
        Serial.println("자동 착륙 진행 중...");
        // 천천히 하강하며 착륙
    }
    
    void setMotorSpeed(int motor_index, int speed) {
        switch (motor_index) {
            case 0: motor1.writeMicroseconds(speed); break;
            case 1: motor2.writeMicroseconds(speed); break;
            case 2: motor3.writeMicroseconds(speed); break;
            case 3: motor4.writeMicroseconds(speed); break;
        }
    }
    
    void updateMotors() {
        // 안전 체크
        if (flight_data.battery_voltage < 10.5) { // 리포 배터리 최소 전압
            emergencyLanding();
            return;
        }
        
        // 모터 페일세이프
        if (millis() - last_update > 500) { // 500ms 이상 신호 없음
            emergencyLanding();
            return;
        }
    }
    
    void emergencyLanding() {
        Serial.println("비상 착륙 모드 활성화!");
        current_mode = LAND;
        
        // 모든 모터 최소 출력으로 설정
        for (int i = 0; i < 4; i++) {
            setMotorSpeed(i, 1000);
        }
    }
    
    float readAltitude() {
        // 바로미터 센서에서 고도 읽기
        // 여기서는 시뮬레이션 값 반환
        return 5.0 + sin(millis() / 1000.0) * 0.5; // 시뮬레이션
    }
    
    void adjustAllMotors(float adjustment) {
        // 모든 모터에 동일한 보정값 적용
        // (실제 구현에서는 각 모터의 현재 값을 읽어서 조정)
    }
    
    void sendTelemetry() {
        if (millis() % 100 == 0) { // 10Hz로 텔레메트리 전송
            Serial.print("Roll: "); Serial.print(flight_data.roll);
            Serial.print(" Pitch: "); Serial.print(flight_data.pitch);
            Serial.print(" Yaw: "); Serial.print(flight_data.yaw);
            Serial.print(" Alt: "); Serial.print(flight_data.altitude);
            Serial.print(" Batt: "); Serial.print(flight_data.battery_voltage);
            Serial.print("V Mode: ");
            
            switch (current_mode) {
                case STABILIZE: Serial.println("STABILIZE"); break;
                case ALT_HOLD: Serial.println("ALT_HOLD"); break;
                case LOITER: Serial.println("LOITER"); break;
                case RTL: Serial.println("RTL"); break;
                case AUTO: Serial.println("AUTO"); break;
                case LAND: Serial.println("LAND"); break;
            }
        }
    }
};

DroneFlightController drone;

void setup() {
    drone.initialize();
}

void loop() {
    drone.update();
    delay(10); // 100Hz 제어 루프
}
```

---

## 위성 통신 시스템

### 지상국 통신 시뮬레이터
```cpp
#include <SoftwareSerial.h>
#include <WiFi.h>

class SatelliteComSystem {
private:
    enum CommState {
        ACQUIRING = 0,     // 신호 획득 중
        TRACKING = 1,      // 위성 추적 중
        COMMUNICATING = 2, // 통신 중
        LOST_SIGNAL = 3    // 신호 손실
    };
    
    struct SatelliteData {
        uint16_t satellite_id;
        float elevation_deg;      // 앙각 (도)
        float azimuth_deg;        // 방위각 (도)
        float range_km;           // 거리 (km)
        float doppler_shift_hz;   // 도플러 편이 (Hz)
        int8_t signal_strength;   // 신호 강도 (dBm)
        bool data_valid;          // 데이터 유효성
        uint32_t last_contact;    // 마지막 접촉 시간
    };
    
    struct GroundStationData {
        float latitude;           // 지상국 위도
        float longitude;          // 지상국 경도
        float altitude_m;         // 지상국 고도
        float antenna_azimuth;    // 안테나 방위각
        float antenna_elevation;  // 안테나 앙각
        bool auto_tracking;       // 자동 추적 모드
        uint32_t pass_start_time; // 패스 시작 시간
        uint32_t pass_end_time;   // 패스 종료 시간
    };
    
    SoftwareSerial radioSerial;
    CommState current_state;
    SatelliteData satellites[10]; // 최대 10개 위성 추적
    GroundStationData ground_station;
    uint8_t active_satellite_count;
    uint32_t last_telemetry_time;
    
public:
    SatelliteComSystem() : radioSerial(2, 3) {
        current_state = ACQUIRING;
        active_satellite_count = 0;
        
        // 지상국 위치 설정 (예: 서울)
        ground_station.latitude = 37.5665;
        ground_station.longitude = 126.9780;
        ground_station.altitude_m = 50.0;
        ground_station.auto_tracking = true;
    }
    
    void initialize() {
        Serial.begin(115200);
        radioSerial.begin(9600);
        
        Serial.println("=== 위성 통신 시스템 초기화 ===");
        Serial.println("지상국 위치: 서울 (37.57°N, 126.98°E)");
        
        // 안테나 제어 핀 설정
        pinMode(5, OUTPUT); // 방위각 모터 제어
        pinMode(6, OUTPUT); // 앙각 모터 제어
        
        // 초기 안테나 위치 설정
        setAntennaPosition(0, 0);
        
        Serial.println("위성 신호 스캔 시작...");
        scanForSatellites();
        
        Serial.println("위성 통신 시스템 준비 완료");
    }
    
    void update() {
        uint32_t current_time = millis();
        
        // 위성 추적 업데이트
        updateSatelliteTracking();
        
        // 안테나 위치 업데이트
        if (ground_station.auto_tracking) {
            updateAntennaTracking();
        }
        
        // 통신 상태 관리
        manageCommState();
        
        // 데이터 송수신
        handleCommunication();
        
        // 텔레메트리 출력 (1초마다)
        if (current_time - last_telemetry_time > 1000) {
            sendTelemetry();
            last_telemetry_time = current_time;
        }
    }
    
private:
    void scanForSatellites() {
        Serial.println("위성 스캔 중...");
        
        // 시뮬레이션: 몇 개의 위성 발견
        for (int i = 0; i < 3; i++) {
            satellites[i].satellite_id = 40000 + i; // 아마추어 위성 ID
            satellites[i].elevation_deg = random(10, 80);
            satellites[i].azimuth_deg = random(0, 360);
            satellites[i].range_km = random(500, 2000);
            satellites[i].signal_strength = random(-120, -80);
            satellites[i].data_valid = true;
            satellites[i].last_contact = millis();
            
            Serial.print("위성 발견: ID "); 
            Serial.print(satellites[i].satellite_id);
            Serial.print(", 앙각: "); 
            Serial.print(satellites[i].elevation_deg);
            Serial.print("°, 방위각: "); 
            Serial.print(satellites[i].azimuth_deg);
            Serial.print("°, 신호강도: "); 
            Serial.print(satellites[i].signal_strength);
            Serial.println(" dBm");
        }
        
        active_satellite_count = 3;
        current_state = TRACKING;
    }
    
    void updateSatelliteTracking() {
        float time_factor = millis() / 10000.0; // 시간 스케일링
        
        for (int i = 0; i < active_satellite_count; i++) {
            // 위성 궤도 시뮬레이션 (단순화된 모델)
            satellites[i].azimuth_deg += 0.1; // 천천히 이동
            if (satellites[i].azimuth_deg > 360) {
                satellites[i].azimuth_deg -= 360;
            }
            
            // 앙각 변화 (포물선 궤도)
            float pass_progress = fmod(time_factor, 10.0) / 10.0; // 10초 주기
            satellites[i].elevation_deg = 80 * sin(pass_progress * PI);
            
            // 도플러 효과 계산
            float relative_velocity = sin(pass_progress * PI * 2) * 7000; // m/s
            satellites[i].doppler_shift_hz = relative_velocity * 145.8e6 / 3e8; // 145.8MHz 기준
            
            // 거리 계산
            satellites[i].range_km = 800 + 400 * sin(pass_progress * PI);
            
            // 신호 강도 (거리와 앙각에 따라 변화)
            float path_loss = 20 * log10(satellites[i].range_km) + 
                             20 * log10(145.8) - 147.55; // 자유공간 손실
            satellites[i].signal_strength = -100 - path_loss + 
                                          10 * sin(satellites[i].elevation_deg * PI / 180);
            
            // 신호 유효성 체크
            satellites[i].data_valid = (satellites[i].elevation_deg > 5) && 
                                     (satellites[i].signal_strength > -130);
        }
    }
    
    void updateAntennaTracking() {
        // 가장 좋은 위성 선택
        int best_satellite = selectBestSatellite();
        
        if (best_satellite >= 0) {
            float target_az = satellites[best_satellite].azimuth_deg;
            float target_el = satellites[best_satellite].elevation_deg;
            
            // 안테나 위치 조정
            setAntennaPosition(target_az, target_el);
            
            ground_station.antenna_azimuth = target_az;
            ground_station.antenna_elevation = target_el;
        }
    }
    
    int selectBestSatellite() {
        int best_index = -1;
        float best_score = -1000;
        
        for (int i = 0; i < active_satellite_count; i++) {
            if (!satellites[i].data_valid) continue;
            
            // 점수 계산: 신호강도 + 앙각 보너스
            float score = satellites[i].signal_strength + 
                         satellites[i].elevation_deg * 0.5;
            
            if (score > best_score) {
                best_score = score;
                best_index = i;
            }
        }
        
        return best_index;
    }
    
    void setAntennaPosition(float azimuth, float elevation) {
        // 방위각 모터 제어 (PWM)
        int az_pwm = map(azimuth, 0, 360, 0, 255);
        analogWrite(5, az_pwm);
        
        // 앙각 모터 제어 (PWM)
        int el_pwm = map(elevation, 0, 90, 0, 255);
        analogWrite(6, el_pwm);
    }
    
    void manageCommState() {
        int best_satellite = selectBestSatellite();
        
        if (best_satellite >= 0) {
            float signal_strength = satellites[best_satellite].signal_strength;
            
            if (signal_strength > -100) {
                current_state = COMMUNICATING;
            } else if (signal_strength > -120) {
                current_state = TRACKING;
            } else {
                current_state = ACQUIRING;
            }
        } else {
            current_state = LOST_SIGNAL;
        }
    }
    
    void handleCommunication() {
        if (current_state == COMMUNICATING) {
            // 위성과 데이터 송수신
            if (radioSerial.available()) {
                String received_data = radioSerial.readString();
                Serial.print("위성으로부터 수신: ");
                Serial.println(received_data);
                
                // 응답 전송
                String response = "지상국 응답: " + String(millis());
                radioSerial.println(response);
            }
            
            // 주기적으로 비콘 전송
            if (millis() % 5000 == 0) {
                String beacon = "지상국 비콘 - " + String(millis() / 1000) + "초";
                radioSerial.println(beacon);
                Serial.println("비콘 전송: " + beacon);
            }
        }
    }
    
    void sendTelemetry() {
        Serial.println("\n=== 위성 통신 시스템 상태 ===");
        Serial.print("통신 상태: ");
        
        switch (current_state) {
            case ACQUIRING:
                Serial.println("신호 획득 중");
                break;
            case TRACKING:
                Serial.println("위성 추적 중");
                break;
            case COMMUNICATING:
                Serial.println("통신 중");
                break;
            case LOST_SIGNAL:
                Serial.println("신호 손실");
                break;
        }
        
        Serial.print("안테나 위치: 방위각 ");
        Serial.print(ground_station.antenna_azimuth);
        Serial.print("°, 앙각 ");
        Serial.print(ground_station.antenna_elevation);
        Serial.println("°");
        
        Serial.println("\n추적 중인 위성:");
        for (int i = 0; i < active_satellite_count; i++) {
            if (satellites[i].data_valid) {
                Serial.print("  위성 ");
                Serial.print(satellites[i].satellite_id);
                Serial.print(": 앙각 ");
                Serial.print(satellites[i].elevation_deg, 1);
                Serial.print("°, 신호 ");
                Serial.print(satellites[i].signal_strength);
                Serial.print(" dBm, 도플러 ");
                Serial.print(satellites[i].doppler_shift_hz, 0);
                Serial.println(" Hz");
            }
        }
        Serial.println();
    }
};

SatelliteComSystem satcom;

void setup() {
    satcom.initialize();
}

void loop() {
    satcom.update();
    delay(100); // 10Hz 업데이트
}
```

---

## 항공기 센서 시스템

### 다중 센서 융합 시스템
```cpp
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>

class AircraftSensorSystem {
private:
    enum SensorType {
        ACCELEROMETER = 0,
        GYROSCOPE = 1,
        MAGNETOMETER = 2,
        BAROMETER = 3,
        TEMPERATURE = 4,
        HUMIDITY = 5,
        PITOT_TUBE = 6,
        GPS = 7,
        RADAR_ALTIMETER = 8
    };
    
    struct SensorData {
        // 관성 센서
        float accel_x, accel_y, accel_z;     // 가속도 (m/s²)
        float gyro_x, gyro_y, gyro_z;        // 각속도 (rad/s)
        float mag_x, mag_y, mag_z;           // 자기장 (µT)
        
        // 항공 센서
        float barometric_pressure;           // 기압 (hPa)
        float barometric_altitude;           // 기압 고도 (m)
        float temperature;                   // 온도 (°C)
        float humidity;                      // 습도 (%)
        float airspeed;                      // 대기속도 (m/s)
        float groundspeed;                   // 지상속도 (m/s)
        
        // 위치 센서
        float gps_latitude;                  // GPS 위도
        float gps_longitude;                 // GPS 경도
        float gps_altitude;                  // GPS 고도 (m)
        float radar_altitude;                // 레이더 고도 (m)
        
        // 계산된 값
        float pitch, roll, yaw;              // 자세 (도)
        float vertical_speed;                // 수직속도 (m/s)
        float heading;                       // 헤딩 (도)
        
        // 센서 상태
        bool sensors_healthy[9];             // 각 센서 상태
        uint32_t last_update_time;           // 마지막 업데이트 시간
    };
    
    struct FlightEnvelope {
        float max_speed;                     // 최대 속도 (m/s)
        float max_altitude;                  // 최대 고도 (m)
        float max_g_force;                   // 최대 G force
        float min_temperature;               // 최소 온도 (°C)
        float max_temperature;               // 최대 온도 (°C)
        float stall_speed;                   // 실속 속도 (m/s)
    };
    
    DHT dht;
    Adafruit_BMP280 bmp;
    SensorData sensor_data;
    FlightEnvelope flight_envelope;
    uint32_t last_sensor_read;
    float kalman_filter_x[6]; // 위치, 속도, 가속도 상태
    
public:
    AircraftSensorSystem() : dht(7, DHT22) {
        // 비행 엔벨로프 설정 (일반적인 경항공기)
        flight_envelope.max_speed = 70.0;      // 252 km/h
        flight_envelope.max_altitude = 3000.0;  // 3km
        flight_envelope.max_g_force = 4.0;      // 4G
        flight_envelope.min_temperature = -20.0;
        flight_envelope.max_temperature = 50.0;
        flight_envelope.stall_speed = 15.0;     // 54 km/h
    }
    
    void initialize() {
        Serial.begin(115200);
        Wire.begin();
        
        Serial.println("=== 항공기 센서 시스템 초기화 ===");
        
        // 온습도 센서 초기화
        dht.begin();
        
        // 기압 센서 초기화
        if (bmp.begin()) {
            Serial.println("BMP280 기압 센서 초기화 성공");
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                           Adafruit_BMP280::SAMPLING_X2,
                           Adafruit_BMP280::SAMPLING_X16,
                           Adafruit_BMP280::FILTER_X16,
                           Adafruit_BMP280::STANDBY_MS_500);
        } else {
            Serial.println("BMP280 초기화 실패!");
            sensor_data.sensors_healthy[BAROMETER] = false;
        }
        
        // 아날로그 센서 핀 설정
        pinMode(A0, INPUT); // 피토 튜브 압력 센서
        pinMode(A1, INPUT); // 레이더 고도계
        pinMode(A2, INPUT); // 연료량 센서
        
        // 센서 상태 초기화
        for (int i = 0; i < 9; i++) {
            sensor_data.sensors_healthy[i] = true;
        }
        
        // 칼만 필터 초기화
        initializeKalmanFilter();
        
        Serial.println("모든 센서 초기화 완료");
        Serial.println("비행 데이터 모니터링 시작");
    }
    
    void update() {
        uint32_t current_time = millis();
        float dt = (current_time - last_sensor_read) / 1000.0;
        last_sensor_read = current_time;
        
        // 모든 센서 데이터 읽기
        readAllSensors();
        
        // 센서 융합 및 상태 추정
        performSensorFusion(dt);
        
        // 비행 엔벨로프 체크
        checkFlightEnvelope();
        
        // 센서 건강도 모니터링
        monitorSensorHealth();
        
        // 데이터 로깅 및 텔레메트리
        logFlightData();
        
        sensor_data.last_update_time = current_time;
    }
    
private:
    void readAllSensors() {
        // 1. 관성 센서 (시뮬레이션)
        sensor_data.accel_x = (analogRead(A3) - 512) * 0.02; // ±10g 범위
        sensor_data.accel_y = (analogRead(A4) - 512) * 0.02;
        sensor_data.accel_z = (analogRead(A5) - 512) * 0.02 + 9.81; // 중력 포함
        
        sensor_data.gyro_x = (analogRead(A6) - 512) * 0.01; // ±5 rad/s
        sensor_data.gyro_y = (analogRead(A7) - 512) * 0.01;
        sensor_data.gyro_z = (analogRead(A8) - 512) * 0.01;
        
        // 2. 환경 센서
        if (sensor_data.sensors_healthy[TEMPERATURE]) {
            sensor_data.temperature = dht.readTemperature();
            sensor_data.humidity = dht.readHumidity();
            
            if (isnan(sensor_data.temperature) || isnan(sensor_data.humidity)) {
                sensor_data.sensors_healthy[TEMPERATURE] = false;
                sensor_data.sensors_healthy[HUMIDITY] = false;
            }
        }
        
        // 3. 기압 센서
        if (sensor_data.sensors_healthy[BAROMETER]) {
            sensor_data.barometric_pressure = bmp.readPressure() / 100.0; // hPa
            sensor_data.barometric_altitude = bmp.readAltitude(1013.25); // 해수면 기준
        }
        
        // 4. 피토 튜브 (대기속도)
        float pitot_raw = analogRead(A0);
        float dynamic_pressure = (pitot_raw / 1023.0) * 500.0; // Pa
        sensor_data.airspeed = sqrt(2 * dynamic_pressure / 1.225); // 해수면 공기밀도
        
        // 5. 레이더 고도계
        float radar_raw = analogRead(A1);
        sensor_data.radar_altitude = (radar_raw / 1023.0) * 500.0; // 0-500m 범위
        
        // 6. GPS 데이터 (시뮬레이션)
        simulateGPSData();
    }
    
    void simulateGPSData() {
        // GPS 데이터 시뮬레이션
        float time_sec = millis() / 1000.0;
        
        sensor_data.gps_latitude = 37.5665 + sin(time_sec * 0.001) * 0.001;
        sensor_data.gps_longitude = 126.9780 + cos(time_sec * 0.001) * 0.001;
        sensor_data.gps_altitude = 1000 + sin(time_sec * 0.01) * 100;
        
        // 지상속도 계산
        static float prev_lat = sensor_data.gps_latitude;
        static float prev_lon = sensor_data.gps_longitude;
        static uint32_t prev_time = millis();
        
        uint32_t current_time = millis();
        float dt = (current_time - prev_time) / 1000.0;
        
        if (dt > 0) {
            float dlat = (sensor_data.gps_latitude - prev_lat) * 111000; // m
            float dlon = (sensor_data.gps_longitude - prev_lon) * 111000 * 
                        cos(sensor_data.gps_latitude * PI / 180); // m
            
            sensor_data.groundspeed = sqrt(dlat*dlat + dlon*dlon) / dt;
        }
        
        prev_lat = sensor_data.gps_latitude;
        prev_lon = sensor_data.gps_longitude;
        prev_time = current_time;
    }
    
    void performSensorFusion(float dt) {
        // 상보 필터를 사용한 자세 추정
        float accel_roll = atan2(sensor_data.accel_y, sensor_data.accel_z) * 180.0 / PI;
        float accel_pitch = atan2(-sensor_data.accel_x, 
                                 sqrt(sensor_data.accel_y*sensor_data.accel_y + 
                                      sensor_data.accel_z*sensor_data.accel_z)) * 180.0 / PI;
        
        float alpha = 0.98; // 상보 필터 계수
        sensor_data.roll = alpha * (sensor_data.roll + sensor_data.gyro_x * 180.0/PI * dt) + 
                          (1 - alpha) * accel_roll;
        sensor_data.pitch = alpha * (sensor_data.pitch + sensor_data.gyro_y * 180.0/PI * dt) + 
                           (1 - alpha) * accel_pitch;
        sensor_data.yaw += sensor_data.gyro_z * 180.0/PI * dt;
        
        // 수직속도 계산 (기압 고도 미분)
        static float prev_altitude = sensor_data.barometric_altitude;
        sensor_data.vertical_speed = (sensor_data.barometric_altitude - prev_altitude) / dt;
        prev_altitude = sensor_data.barometric_altitude;
        
        // 헤딩 계산 (자기 나침반)
        sensor_data.heading = atan2(sensor_data.mag_y, sensor_data.mag_x) * 180.0 / PI;
        if (sensor_data.heading < 0) sensor_data.heading += 360;
        
        // 칼만 필터 업데이트
        updateKalmanFilter(dt);
    }
    
    void initializeKalmanFilter() {
        // 간단한 1차원 칼만 필터 (고도 추정용)
        for (int i = 0; i < 6; i++) {
            kalman_filter_x[i] = 0;
        }
    }
    
    void updateKalmanFilter(float dt) {
        // 고도 추정을 위한 칼만 필터
        // 상태: [위치, 속도, 가속도]
        // 측정: 기압 고도, GPS 고도, 레이더 고도
        
        // 예측 단계 (간단화된 모델)
        kalman_filter_x[0] += kalman_filter_x[1] * dt; // 위치 += 속도 * dt
        kalman_filter_x[1] += kalman_filter_x[2] * dt; // 속도 += 가속도 * dt
        
        // 업데이트 단계 (측정값 융합)
        float weight_baro = 0.4;
        float weight_gps = 0.3;
        float weight_radar = 0.3;
        
        float fused_altitude = weight_baro * sensor_data.barometric_altitude +
                              weight_gps * sensor_data.gps_altitude +
                              weight_radar * sensor_data.radar_altitude;
        
        kalman_filter_x[0] = 0.8 * kalman_filter_x[0] + 0.2 * fused_altitude;
    }
    
    void checkFlightEnvelope() {
        bool envelope_violation = false;
        String warnings = "";
        
        // 속도 체크
        if (sensor_data.airspeed > flight_envelope.max_speed) {
            warnings += "경고: 최대 속도 초과! ";
            envelope_violation = true;
        }
        if (sensor_data.airspeed < flight_envelope.stall_speed) {
            warnings += "경고: 실속 속도 접근! ";
            envelope_violation = true;
        }
        
        // 고도 체크
        if (sensor_data.barometric_altitude > flight_envelope.max_altitude) {
            warnings += "경고: 최대 고도 초과! ";
            envelope_violation = true;
        }
        
        // G-force 체크
        float total_g = sqrt(sensor_data.accel_x*sensor_data.accel_x + 
                            sensor_data.accel_y*sensor_data.accel_y + 
                            sensor_data.accel_z*sensor_data.accel_z) / 9.81;
        if (total_g > flight_envelope.max_g_force) {
            warnings += "경고: 과도한 G-force! ";
            envelope_violation = true;
        }
        
        // 온도 체크
        if (sensor_data.temperature < flight_envelope.min_temperature || 
            sensor_data.temperature > flight_envelope.max_temperature) {
            warnings += "경고: 온도 범위 초과! ";
            envelope_violation = true;
        }
        
        if (envelope_violation) {
            Serial.println(warnings);
            // 경고음, LED 등 알림 시스템 활성화
            digitalWrite(13, HIGH); // 경고 LED
        } else {
            digitalWrite(13, LOW);
        }
    }
    
    void monitorSensorHealth() {
        // 센서 건강도 체크 로직
        uint32_t current_time = millis();
        
        // 온도 센서 체크
        if (isnan(sensor_data.temperature) || isnan(sensor_data.humidity)) {
            sensor_data.sensors_healthy[TEMPERATURE] = false;
            sensor_data.sensors_healthy[HUMIDITY] = false;
        }
        
        // 기압 센서 체크
        if (sensor_data.barometric_pressure < 500 || sensor_data.barometric_pressure > 1100) {
            sensor_data.sensors_healthy[BAROMETER] = false;
        }
        
        // GPS 체크 (위치 변화 확인)
        static float prev_gps_lat = sensor_data.gps_latitude;
        static uint32_t last_gps_change = current_time;
        
        if (abs(sensor_data.gps_latitude - prev_gps_lat) > 0.0001) {
            last_gps_change = current_time;
            prev_gps_lat = sensor_data.gps_latitude;
        }
        
        if (current_time - last_gps_change > 5000) { // 5초간 변화 없음
            sensor_data.sensors_healthy[GPS] = false;
        }
    }
    
    void logFlightData() {
        // 비행 데이터 로깅 (1초마다)
        static uint32_t last_log_time = 0;
        uint32_t current_time = millis();
        
        if (current_time - last_log_time > 1000) {
            Serial.println("\n=== 비행 데이터 ===");
            
            // 자세 정보
            Serial.print("자세 - Roll: "); Serial.print(sensor_data.roll, 1);
            Serial.print("°, Pitch: "); Serial.print(sensor_data.pitch, 1);
            Serial.print("°, Yaw: "); Serial.print(sensor_data.yaw, 1); Serial.println("°");
            
            // 속도 및 고도
            Serial.print("속도 - 대기속도: "); Serial.print(sensor_data.airspeed, 1);
            Serial.print(" m/s, 지상속도: "); Serial.print(sensor_data.groundspeed, 1);
            Serial.print(" m/s, 수직속도: "); Serial.print(sensor_data.vertical_speed, 1); 
            Serial.println(" m/s");
            
            Serial.print("고도 - 기압: "); Serial.print(sensor_data.barometric_altitude, 1);
            Serial.print(" m, GPS: "); Serial.print(sensor_data.gps_altitude, 1);
            Serial.print(" m, 레이더: "); Serial.print(sensor_data.radar_altitude, 1); 
            Serial.println(" m");
            
            // 환경 정보
            Serial.print("환경 - 온도: "); Serial.print(sensor_data.temperature, 1);
            Serial.print("°C, 습도: "); Serial.print(sensor_data.humidity, 1);
            Serial.print("%, 기압: "); Serial.print(sensor_data.barometric_pressure, 1); 
            Serial.println(" hPa");
            
            // 위치 정보
            Serial.print("위치 - 위도: "); Serial.print(sensor_data.gps_latitude, 6);
            Serial.print("°, 경도: "); Serial.print(sensor_data.gps_longitude, 6);
            Serial.print("°, 헤딩: "); Serial.print(sensor_data.heading, 1); Serial.println("°");
            
            // 센서 상태
            Serial.print("센서 상태: ");
            for (int i = 0; i < 9; i++) {
                Serial.print(sensor_data.sensors_healthy[i] ? "O" : "X");
                Serial.print(" ");
            }
            Serial.println();
            
            last_log_time = current_time;
        }
    }
};

AircraftSensorSystem aircraft_sensors;

void setup() {
    aircraft_sensors.initialize();
}

void loop() {
    aircraft_sensors.update();
    delay(50); // 20Hz 업데이트
}
```

---

## 로켓 제어 시스템

### 다단 로켓 발사 시퀀스
```cpp
class RocketControlSystem {
private:
    enum LaunchPhase {
        PRELAUNCH = 0,    // 발사 전 점검
        IGNITION = 1,     // 점화
        LIFTOFF = 2,      // 이륙
        FIRST_STAGE = 3,  // 1단 연소
        SEPARATION = 4,   // 단분리
        SECOND_STAGE = 5, // 2단 연소
        PAYLOAD_DEPLOY = 6, // 탑재체 분리
        RECOVERY = 7      // 회수
    };
    
    enum EmergencyMode {
        NOMINAL = 0,      // 정상
        ABORT = 1,        // 발사 중단
        DESTRUCT = 2      // 자폭
    };
    
    struct RocketData {
        // 위치 및 속도
        float altitude;           // 고도 (m)
        float velocity;           // 속도 (m/s)
        float acceleration;       // 가속도 (m/s²)
        float downrange;         // 수평 거리 (m)
        
        // 자세
        float pitch, roll, yaw;   // 자세 (도)
        float pitch_rate, roll_rate, yaw_rate; // 각속도 (도/s)
        
        // 추진 시스템
        float thrust;            // 추력 (N)
        float mass;              // 총 질량 (kg)
        float fuel_mass;         // 연료 질량 (kg)
        float oxidizer_mass;     // 산화제 질량 (kg)
        
        // 환경
        float ambient_pressure;   // 대기압 (Pa)
        float ambient_temp;      // 대기온도 (K)
        float dynamic_pressure;  // 동압 (Pa)
        float mach_number;       // 마하수
        
        // 시스템 상태
        bool engine_running;     // 엔진 가동 상태
        bool guidance_active;    // 유도 시스템 활성
        bool telemetry_active;   // 텔레메트리 활성
        uint32_t mission_time;   // 미션 시간 (초)
    };
    
    struct LaunchSequence {
        uint32_t sequence_time;  // 시퀀스 시간 (초)
        String description;      // 설명
        bool executed;           // 실행 여부
    };
    
    RocketData rocket;
    LaunchPhase current_phase;
    EmergencyMode emergency_status;
    uint32_t launch_time;
    bool launch_enabled;
    
    // 발사 시퀀스 타이머
    LaunchSequence launch_sequence[20] = {
        {-600, "T-10분: 최종 시스템 점검", false},
        {-300, "T-5분: 발사대 클리어런스 확인", false},
        {-120, "T-2분: 내부 전력으로 전환", false},
        {-60, "T-1분: 최종 GO/NO-GO 결정", false},
        {-10, "T-10초: 엔진 점화 시퀀스 시작", false},
        {-3, "T-3초: 엔진 점화", false},
        {0, "T-0초: 리프트오프!", false},
        {5, "T+5초: 타워 클리어", false},
        {20, "T+20초: 롤 프로그램 시작", false},
        {60, "T+1분: 최대 동압 통과", false},
        {120, "T+2분: 1단 연소 종료", false},
        {125, "T+2분5초: 1단 분리", false},
        {130, "T+2분10초: 2단 점화", false},
        {300, "T+5분: 2단 연소 종료", false},
        {320, "T+5분20초: 페어링 분리", false},
        {600, "T+10분: 탑재체 분리", false},
        {900, "T+15분: 1단 착륙 시작", false},
        {1200, "T+20분: 1단 착륙 완료", false},
        {1800, "T+30분: 미션 완료", false}
    };
    
public:
    RocketControlSystem() {
        current_phase = PRELAUNCH;
        emergency_status = NOMINAL;
        launch_enabled = false;
        launch_time = 0;
        
        // 초기 로켓 데이터
        rocket.altitude = 0;
        rocket.velocity = 0;
        rocket.acceleration = 0;
        rocket.mass = 5000;        // 총 5톤
        rocket.fuel_mass = 2000;   // 연료 2톤
        rocket.oxidizer_mass = 1500; // 산화제 1.5톤
        rocket.thrust = 0;
        rocket.engine_running = false;
        rocket.guidance_active = false;
        rocket.telemetry_active = true;
    }
    
    void initialize() {
        Serial.begin(115200);
        
        Serial.println("=== 로켓 제어 시스템 초기화 ===");
        Serial.println("다단 로켓 발사 제어 시스템 v2.1");
        
        // 하드웨어 초기화
        pinMode(2, OUTPUT);  // 1단 엔진 점화
        pinMode(3, OUTPUT);  // 2단 엔진 점화
        pinMode(4, OUTPUT);  // 단분리 장치
        pinMode(5, OUTPUT);  // 페어링 분리
        pinMode(6, OUTPUT);  // 탑재체 분리
        pinMode(7, OUTPUT);  // 비상 자폭 장치
        pinMode(8, INPUT);   // 발사 승인 버튼
        pinMode(9, INPUT);   // 비상 중단 버튼
        
        // 안전 장치 초기화
        digitalWrite(7, LOW); // 자폭 장치 안전
        
        // 센서 초기화
        initializeSensors();
        
        // 시스템 자가진단
        if (performSystemCheck()) {
            Serial.println("모든 시스템 정상 - 발사 준비 완료");
            launch_enabled = true;
        } else {
            Serial.println("시스템 오류 - 발사 불가");
            launch_enabled = false;
        }
        
        Serial.println("발사 승인 대기 중...");
    }
    
    void update() {
        uint32_t current_time = millis();
        
        // 사용자 입력 확인
        checkUserInput();
        
        // 발사 시퀀스 처리
        if (launch_enabled && digitalRead(8) == HIGH) {
            if (launch_time == 0) {
                launch_time = current_time;
                Serial.println("\n발사 시퀀스 시작!");
            }
            
            processLaunchSequence(current_time);
        }
        
        // 로켓 물리 시뮬레이션
        if (current_phase >= LIFTOFF) {
            updateRocketPhysics();
        }
        
        // 비상 상황 모니터링
        checkEmergencyConditions();
        
        // 텔레메트리 출력
        sendTelemetry();
    }
    
private:
    void initializeSensors() {
        Serial.println("센서 시스템 초기화...");
        // 가속도계, 자이로스코프, GPS, 기압계 등 초기화
        Serial.println("IMU 센서 초기화 완료");
        Serial.println("GPS 수신기 초기화 완료");
        Serial.println("기압 센서 초기화 완료");
        Serial.println("온도 센서 초기화 완료");
    }
    
    bool performSystemCheck() {
        Serial.println("\n=== 시스템 자가진단 ===");
        
        // 추진 시스템 체크
        Serial.print("추진 시스템 체크... ");
        delay(500);
        Serial.println("OK");
        
        // 유도 시스템 체크
        Serial.print("유도 시스템 체크... ");
        delay(500);
        Serial.println("OK");
        
        // 통신 시스템 체크
        Serial.print("통신 시스템 체크... ");
        delay(500);
        Serial.println("OK");
        
        // 안전 시스템 체크
        Serial.print("안전 시스템 체크... ");
        delay(500);
        Serial.println("OK");
        
        // 연료 시스템 체크
        Serial.print("연료 시스템 체크... ");
        delay(500);
        Serial.println("OK");
        
        Serial.println("모든 시스템 정상");
        return true;
    }
    
    void checkUserInput() {
        // 비상 중단 버튼 체크
        if (digitalRead(9) == HIGH) {
            if (emergency_status == NOMINAL) {
                emergency_status = ABORT;
                Serial.println("*** 비상 중단 명령 수신 ***");
                executeAbort();
            }
        }
    }
    
    void processLaunchSequence(uint32_t current_time) {
        int32_t elapsed_time = (current_time - launch_time) / 1000; // 초 단위
        int32_t T = elapsed_time - 600; // T-10분부터 시작
        
        // 발사 시퀀스 이벤트 처리
        for (int i = 0; i < 20; i++) {
            if (!launch_sequence[i].executed && T >= launch_sequence[i].sequence_time) {
                Serial.print(launch_sequence[i].description);
                Serial.println();
                
                executeSequenceEvent(i);
                launch_sequence[i].executed = true;
            }
        }
        
        // 발사 단계 업데이트
        updateLaunchPhase(T);
    }
    
    void executeSequenceEvent(int sequence_index) {
        switch (sequence_index) {
            case 4: // T-10초: 엔진 점화 시퀀스
                Serial.println("엔진 점화 시퀀스 시작");
                break;
                
            case 5: // T-3초: 엔진 점화
                digitalWrite(2, HIGH); // 1단 엔진 점화
                rocket.engine_running = true;
                rocket.thrust = 75000; // 75kN 추력
                Serial.println("1단 엔진 점화!");
                break;
                
            case 6: // T-0초: 리프트오프
                current_phase = LIFTOFF;
                Serial.println("*** 리프트오프! ***");
                break;
                
            case 8: // T+20초: 롤 프로그램
                Serial.println("롤 프로그램 실행");
                rocket.guidance_active = true;
                break;
                
            case 10: // T+2분: 1단 연소 종료
                digitalWrite(2, LOW);
                rocket.engine_running = false;
                rocket.thrust = 0;
                current_phase = SEPARATION;
                Serial.println("1단 연소 종료");
                break;
                
            case 11: // T+2분5초: 1단 분리
                digitalWrite(4, HIGH); // 분리 신호
                delay(100);
                digitalWrite(4, LOW);
                Serial.println("1단 분리 완료");
                break;
                
            case 12: // T+2분10초: 2단 점화
                digitalWrite(3, HIGH);
                rocket.engine_running = true;
                rocket.thrust = 25000; // 25kN 추력
                current_phase = SECOND_STAGE;
                Serial.println("2단 엔진 점화!");
                break;
                
            case 13: // T+5분: 2단 연소 종료
                digitalWrite(3, LOW);
                rocket.engine_running = false;
                rocket.thrust = 0;
                Serial.println("2단 연소 종료");
                break;
                
            case 14: // T+5분20초: 페어링 분리
                digitalWrite(5, HIGH);
                delay(100);
                digitalWrite(5, LOW);
                Serial.println("페어링 분리 완료");
                break;
                
            case 15: // T+10분: 탑재체 분리
                digitalWrite(6, HIGH);
                delay(100);
                digitalWrite(6, LOW);
                current_phase = PAYLOAD_DEPLOY;
                Serial.println("탑재체 분리 완료");
                break;
                
            case 18: // T+30분: 미션 완료
                current_phase = RECOVERY;
                Serial.println("*** 미션 완료! ***");
                break;
        }
    }
    
    void updateLaunchPhase(int32_t T) {
        if (T < 0) {
            current_phase = PRELAUNCH;
        } else if (T >= 0 && T < 5) {
            current_phase = IGNITION;
        } else if (T >= 5 && T < 120) {
            current_phase = FIRST_STAGE;
        } else if (T >= 120 && T < 130) {
            current_phase = SEPARATION;
        } else if (T >= 130 && T < 300) {
            current_phase = SECOND_STAGE;
        } else if (T >= 600) {
            current_phase = PAYLOAD_DEPLOY;
        }
    }
    
    void updateRocketPhysics() {
        static uint32_t last_physics_update = millis();
        uint32_t current_time = millis();
        float dt = (current_time - last_physics_update) / 1000.0;
        last_physics_update = current_time;
        
        if (dt > 0.1) return; // 너무 큰 dt는 무시
        
        // 중력 가속도 (고도에 따라 변화)
        float gravity = 9.81 * pow(6371000.0 / (6371000.0 + rocket.altitude), 2);
        
        // 대기 밀도 (고도에 따라 변화)
        float air_density = 1.225 * exp(-rocket.altitude / 8000.0);
        
        // 항력 계산
        float drag_coefficient = 0.3;
        float reference_area = 1.0; // m²
        float drag_force = 0.5 * air_density * rocket.velocity * rocket.velocity * 
                          drag_coefficient * reference_area;
        
        // 총 가속도 계산
        float net_force = rocket.thrust - rocket.mass * gravity - drag_force;
        rocket.acceleration = net_force / rocket.mass;
        
        // 속도 및 위치 업데이트
        rocket.velocity += rocket.acceleration * dt;
        rocket.altitude += rocket.velocity * dt;
        
        // 연료 소모 (엔진 가동 시)
        if (rocket.engine_running) {
            float fuel_flow_rate = 50.0; // kg/s
            rocket.fuel_mass -= fuel_flow_rate * dt;
            rocket.mass = 1500 + rocket.fuel_mass + rocket.oxidizer_mass; // 건조 질량 1.5톤
            
            if (rocket.fuel_mass <= 0) {
                rocket.fuel_mass = 0;
                rocket.engine_running = false;
                rocket.thrust = 0;
            }
        }
        
        // 마하수 계산
        float sound_speed = sqrt(1.4 * 287 * (288.15 - 0.0065 * rocket.altitude));
        rocket.mach_number = rocket.velocity / sound_speed;
        
        // 동압 계산
        rocket.dynamic_pressure = 0.5 * air_density * rocket.velocity * rocket.velocity;
        
        rocket.mission_time = (millis() - launch_time) / 1000;
    }
    
    void checkEmergencyConditions() {
        // 고도 안전 범위 체크
        if (rocket.altitude > 100000) { // 100km 이상
            if (emergency_status == NOMINAL) {
                Serial.println("경고: 최대 고도 초과");
            }
        }
        
        // 속도 안전 범위 체크
        if (rocket.velocity > 8000) { // 8km/s 이상
            if (emergency_status == NOMINAL) {
                Serial.println("경고: 최대 속도 초과");
            }
        }
        
        // 연료 잔량 체크
        if (rocket.fuel_mass < 50 && rocket.engine_running) {
            Serial.println("경고: 연료 부족");
        }
        
        // 최대 동압 체크
        if (rocket.dynamic_pressure > 35000) { // 35kPa 이상
            Serial.println("경고: 최대 동압 초과");
        }
    }
    
    void executeAbort() {
        Serial.println("=== 발사 중단 시퀀스 실행 ===");
        
        // 모든 엔진 정지
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        rocket.engine_running = false;
        rocket.thrust = 0;
        
        // 비상 분리 시퀀스
        if (rocket.altitude > 1000) {
            digitalWrite(6, HIGH); // 탑재체 분리
            Serial.println("비상 탑재체 분리");
        }
        
        current_phase = RECOVERY;
        Serial.println("발사 중단 완료");
    }
    
    void sendTelemetry() {
        static uint32_t last_telemetry = 0;
        uint32_t current_time = millis();
        
        if (current_time - last_telemetry > 1000) { // 1초마다
            Serial.println("\n=== 로켓 텔레메트리 ===");
            
            // 미션 시간
            Serial.print("미션 시간: T+");
            Serial.print(rocket.mission_time);
            Serial.println("초");
            
            // 위치 및 속도
            Serial.print("고도: "); Serial.print(rocket.altitude, 0); Serial.println(" m");
            Serial.print("속도: "); Serial.print(rocket.velocity, 1); Serial.println(" m/s");
            Serial.print("가속도: "); Serial.print(rocket.acceleration, 2); Serial.println(" m/s²");
            Serial.print("마하수: "); Serial.print(rocket.mach_number, 2); Serial.println();
            
            // 추진 시스템
            Serial.print("추력: "); Serial.print(rocket.thrust, 0); Serial.println(" N");
            Serial.print("연료: "); Serial.print(rocket.fuel_mass, 0); Serial.println(" kg");
            Serial.print("총 질량: "); Serial.print(rocket.mass, 0); Serial.println(" kg");
            
            // 상태
            Serial.print("단계: ");
            switch (current_phase) {
                case PRELAUNCH: Serial.println("발사 전"); break;
                case IGNITION: Serial.println("점화"); break;
                case LIFTOFF: Serial.println("이륙"); break;
                case FIRST_STAGE: Serial.println("1단 연소"); break;
                case SEPARATION: Serial.println("단분리"); break;
                case SECOND_STAGE: Serial.println("2단 연소"); break;
                case PAYLOAD_DEPLOY: Serial.println("탑재체 분리"); break;
                case RECOVERY: Serial.println("회수"); break;
            }
            
            Serial.print("엔진 상태: ");
            Serial.println(rocket.engine_running ? "가동" : "정지");
            
            last_telemetry = current_time;
        }
    }
};

RocketControlSystem rocket_control;

void setup() {
    rocket_control.initialize();
}

void loop() {
    rocket_control.update();
    delay(50); // 20Hz 업데이트
}
```

---

## 마무리

이 항공우주 전자 시스템 가이드는 Arduino를 사용하여 항공우주 분야의 핵심 기술들을 학습하고 구현할 수 있도록 구성되었습니다.

### 학습 로드맵
1. **기초**: 드론 비행 제어 시스템부터 시작
2. **중급**: 항공기 센서 시스템으로 다중 센서 융합 학습
3. **고급**: 위성 통신 및 로켓 제어 시스템으로 복합 시스템 구현

### 추가 프로젝트
- 성층권 기구(High Altitude Balloon) 제어
- 위성 궤도 추적 시스템
- 항공 교통 관제 시뮬레이터
- 우주 정거장 도킹 시뮬레이터

### 안전 주의사항
- 실제 비행체 테스트 시 안전 규정 준수
- 무선 주파수 사용 시 허가 필요
- 고고도 실험 시 항공청 신고 필요

이 가이드를 통해 항공우주 엔지니어링의 기초를 다지고, 실제 산업에서 사용되는 기술들을 이해할 수 있을 것입니다.

---

**작성자**: Arduino 임베디드 전문 강사  
**최종 수정**: 2024년  
**난이도**: 고급 ⭐⭐⭐⭐⭐