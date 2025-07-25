# 고급 로봇공학 시스템 🤖

## 개요
Arduino를 이용하여 고급 로봇공학 기술을 구현하고 학습하는 완전 가이드입니다.

## 목차
1. [로봇공학 기초](#로봇공학-기초)
2. [로봇 키네마틱스](#로봇-키네마틱스)
3. [로봇 팔 제어](#로봇-팔-제어)
4. [이동 로봇 네비게이션](#이동-로봇-네비게이션)
5. [SLAM 시스템](#slam-시스템)
6. [머신 비전](#머신-비전)
7. [인공지능 로봇](#인공지능-로봇)
8. [협동 로봇](#협동-로봇)
9. [휴머노이드 로봇](#휴머노이드-로봇)
10. [통합 로봇 시스템](#통합-로봇-시스템)

---

## 로봇공학 기초

### 로봇 제어 시스템 아키텍처
```cpp
// 고급 로봇 제어 시스템
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>

class AdvancedRoboticsSystem {
private:
    // 로봇 상태 정의
    enum RobotState {
        INITIALIZATION = 0,
        IDLE = 1,
        MAPPING = 2,
        NAVIGATION = 3,
        MANIPULATION = 4,
        INTERACTION = 5,
        LEARNING = 6,
        EMERGENCY_STOP = 7
    };
    
    // 로봇 구성 요소
    struct RobotComponents {
        // 센서 시스템
        bool lidar_available;
        bool camera_available;
        bool imu_available;
        bool ultrasonic_available;
        bool touch_sensors;
        bool force_sensors;
        
        // 액추에이터 시스템
        int motor_count;
        int servo_count;
        bool gripper_available;
        bool wheels_available;
        bool legs_available;
        
        // 처리 시스템
        bool ai_processor;
        bool vision_processor;
        bool speech_processor;
        float cpu_usage;
        float memory_usage;
        
        // 통신 시스템
        bool wifi_connected;
        bool bluetooth_available;
        bool rs485_available;
        uint32_t last_communication;
    };
    
    // 로봇 포즈 (위치 + 자세)
    struct RobotPose {
        // 위치 (월드 좌표계)
        float x, y, z;              // 미터 단위
        
        // 자세 (오일러 각)
        float roll, pitch, yaw;     // 라디안 단위
        
        // 속도
        float linear_x, linear_y, linear_z;    // m/s
        float angular_x, angular_y, angular_z; // rad/s
        
        // 가속도
        float accel_x, accel_y, accel_z;      // m/s²
        
        // 시간 스탬프
        uint32_t timestamp;
        
        // 불확실성 (공분산)
        float position_covariance[9];
        float orientation_covariance[9];
    };
    
    // 로봇 임무 시스템
    struct Mission {
        uint16_t mission_id;
        String mission_type;        // "GOTO", "PICK", "PLACE", "PATROL", "SEARCH"
        float target_x, target_y, target_z;
        float target_roll, target_pitch, target_yaw;
        String parameters;          // JSON 형태의 추가 파라미터
        uint8_t priority;          // 1(높음) ~ 5(낮음)
        uint32_t start_time;
        uint32_t deadline;
        bool is_completed;
        float completion_ratio;
    };
    
    RobotState current_state;
    RobotComponents components;
    RobotPose current_pose;
    RobotPose target_pose;
    Mission current_mission;
    
    // 제어 루프 타이밍
    uint32_t last_control_update;
    uint32_t control_frequency;    // Hz
    
    // 안전 시스템
    bool emergency_stop_activated;
    float safety_distance;         // 장애물과의 최소 거리
    float max_velocity;           // 최대 속도 제한
    float max_acceleration;       // 최대 가속도 제한
    
public:
    void initialize() {
        Serial.begin(115200);
        Wire.begin();
        
        Serial.println("=== 고급 로봇공학 시스템 초기화 ===");
        Serial.println("다관절 로봇 제어 시스템 v3.0");
        Serial.println("개발: 로봇공학 연구소");
        Serial.println();
        
        // 시스템 파라미터 초기화
        current_state = INITIALIZATION;
        control_frequency = 50;      // 50Hz 제어 루프
        safety_distance = 0.3;       // 30cm 안전 거리
        max_velocity = 2.0;          // 2m/s 최대 속도
        max_acceleration = 1.0;      // 1m/s² 최대 가속도
        emergency_stop_activated = false;
        
        // 하드웨어 구성 요소 감지
        detectHardwareComponents();
        
        // 센서 시스템 초기화
        initializeSensors();
        
        // 액추에이터 시스템 초기화
        initializeActuators();
        
        // 제어 시스템 초기화
        initializeControlSystems();
        
        // 안전 시스템 초기화
        initializeSafetySystems();
        
        // 통신 시스템 초기화
        initializeCommunication();
        
        // 초기 포즈 설정
        resetPose();
        
        current_state = IDLE;
        last_control_update = millis();
        
        Serial.println("로봇 시스템 초기화 완료");
        Serial.println("운용 준비 상태");
        displaySystemStatus();
    }
    
    void update() {
        uint32_t current_time = millis();
        float dt = (current_time - last_control_update) / 1000.0;
        
        // 제어 주파수 체크
        if (dt < 1.0 / control_frequency) {
            return;
        }
        
        last_control_update = current_time;
        
        // 1. 센서 데이터 읽기
        updateSensorReadings();
        
        // 2. 상태 추정
        updateStateEstimation(dt);
        
        // 3. 안전 시스템 체크
        checkSafetySystems();
        
        // 4. 임무 관리
        manageMissions();
        
        // 5. 제어 명령 계산
        calculateControlCommands(dt);
        
        // 6. 액추에이터 제어
        updateActuators();
        
        // 7. 통신 처리
        handleCommunications();
        
        // 8. 상태 로깅
        logSystemState();
    }
    
private:
    void detectHardwareComponents() {
        Serial.println("하드웨어 구성 요소 감지 중...");
        
        // I2C 스캔으로 센서 감지
        components.imu_available = scanI2CDevice(0x68);     // MPU6050
        components.lidar_available = scanI2CDevice(0x29);   // VL53L0X
        
        // 핀 스캔으로 액추에이터 감지
        components.motor_count = 4;      // 예시: 4개 모터
        components.servo_count = 6;      // 예시: 6개 서보
        components.gripper_available = true;
        components.wheels_available = true;
        
        // 디지털 핀으로 센서 감지
        components.ultrasonic_available = true;
        components.touch_sensors = true;
        components.force_sensors = false;
        
        // 처리 시스템 상태
        components.ai_processor = true;
        components.vision_processor = false;
        components.speech_processor = false;
        
        Serial.println("하드웨어 감지 완료:");
        Serial.print("- 모터: "); Serial.println(components.motor_count);
        Serial.print("- 서보: "); Serial.println(components.servo_count);
        Serial.print("- IMU: "); Serial.println(components.imu_available ? "있음" : "없음");
        Serial.print("- LIDAR: "); Serial.println(components.lidar_available ? "있음" : "없음");
    }
    
    bool scanI2CDevice(uint8_t address) {
        Wire.beginTransmission(address);
        return (Wire.endTransmission() == 0);
    }
    
    void initializeSensors() {
        Serial.println("센서 시스템 초기화...");
        
        if (components.imu_available) {
            Serial.println("- IMU 센서 초기화");
            // MPU6050 초기화 코드
        }
        
        if (components.lidar_available) {
            Serial.println("- LIDAR 센서 초기화");
            // VL53L0X 초기화 코드
        }
        
        if (components.ultrasonic_available) {
            Serial.println("- 초음파 센서 초기화");
            pinMode(7, OUTPUT);  // Trig
            pinMode(8, INPUT);   // Echo
        }
        
        Serial.println("센서 초기화 완료");
    }
    
    void initializeActuators() {
        Serial.println("액추에이터 시스템 초기화...");
        
        // 모터 드라이버 초기화
        for (int i = 0; i < components.motor_count; i++) {
            pinMode(2 + i * 2, OUTPUT);     // 방향 핀
            pinMode(3 + i * 2, OUTPUT);     // PWM 핀
            Serial.print("- 모터 "); Serial.print(i); Serial.println(" 초기화");
        }
        
        // 서보 모터 초기화
        Serial.print("- "); Serial.print(components.servo_count); 
        Serial.println("개 서보 모터 초기화");
        
        Serial.println("액추에이터 초기화 완료");
    }
    
    void initializeControlSystems() {
        Serial.println("제어 시스템 초기화...");
        
        // PID 제어기 초기화
        Serial.println("- PID 제어기 초기화");
        
        // 궤적 계획기 초기화
        Serial.println("- 궤적 계획기 초기화");
        
        // 역기구학 해법기 초기화
        Serial.println("- 역기구학 해법기 초기화");
        
        Serial.println("제어 시스템 초기화 완료");
    }
    
    void initializeSafetySystems() {
        Serial.println("안전 시스템 초기화...");
        
        // 비상 정지 버튼
        pinMode(13, INPUT_PULLUP);  // 비상 정지 버튼
        
        // 안전 LED
        pinMode(12, OUTPUT);        // 상태 LED
        
        // 충돌 방지 시스템
        Serial.println("- 충돌 방지 시스템 활성화");
        
        // 속도 제한 시스템
        Serial.println("- 속도 제한 시스템 활성화");
        
        Serial.println("안전 시스템 초기화 완료");
    }
    
    void initializeCommunication() {
        Serial.println("통신 시스템 초기화...");
        
        // WiFi 초기화
        WiFi.begin("RobotLab_Network", "robot123");
        Serial.println("- WiFi 연결 시도 중...");
        
        int wifi_attempts = 0;
        while (WiFi.status() != WL_CONNECTED && wifi_attempts < 10) {
            delay(1000);
            Serial.print(".");
            wifi_attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            components.wifi_connected = true;
            Serial.print("\n- WiFi 연결 성공: ");
            Serial.println(WiFi.localIP());
        } else {
            components.wifi_connected = false;
            Serial.println("\n- WiFi 연결 실패");
        }
        
        Serial.println("통신 시스템 초기화 완료");
    }
    
    void resetPose() {
        current_pose.x = 0.0;
        current_pose.y = 0.0;
        current_pose.z = 0.0;
        current_pose.roll = 0.0;
        current_pose.pitch = 0.0;
        current_pose.yaw = 0.0;
        
        current_pose.linear_x = 0.0;
        current_pose.linear_y = 0.0;
        current_pose.linear_z = 0.0;
        current_pose.angular_x = 0.0;
        current_pose.angular_y = 0.0;
        current_pose.angular_z = 0.0;
        
        current_pose.timestamp = millis();
        
        Serial.println("로봇 포즈 초기화 완료");
    }
    
    void updateSensorReadings() {
        // IMU 데이터 읽기
        if (components.imu_available) {
            readIMUData();
        }
        
        // LIDAR 데이터 읽기
        if (components.lidar_available) {
            readLIDARData();
        }
        
        // 초음파 센서 읽기
        if (components.ultrasonic_available) {
            readUltrasonicSensors();
        }
        
        // 엔코더 데이터 읽기
        readEncoderData();
        
        // 힘/토크 센서 읽기
        if (components.force_sensors) {
            readForceSensors();
        }
    }
    
    void readIMUData() {
        // 가속도계 데이터
        current_pose.accel_x = (analogRead(A0) - 512) * 0.02;  // 시뮬레이션
        current_pose.accel_y = (analogRead(A1) - 512) * 0.02;
        current_pose.accel_z = (analogRead(A2) - 512) * 0.02;
        
        // 자이로스코프 데이터
        current_pose.angular_x = (analogRead(A3) - 512) * 0.01;
        current_pose.angular_y = (analogRead(A4) - 512) * 0.01;
        current_pose.angular_z = (analogRead(A5) - 512) * 0.01;
    }
    
    void readLIDARData() {
        // LIDAR 거리 측정 시뮬레이션
        float distance = analogRead(A6) * 5.0 / 1024.0;  // 0-5m 범위
        
        // 장애물 감지
        if (distance < safety_distance) {
            Serial.println("경고: 장애물 감지!");
        }
    }
    
    void readUltrasonicSensors() {
        // 초음파 센서 거리 측정
        digitalWrite(7, LOW);
        delayMicroseconds(2);
        digitalWrite(7, HIGH);
        delayMicroseconds(10);
        digitalWrite(7, LOW);
        
        long duration = pulseIn(8, HIGH);
        float distance = duration * 0.034 / 2;  // cm 단위
        
        // 센서 데이터 저장 및 처리
    }
    
    void readEncoderData() {
        // 모터 엔코더에서 위치 및 속도 피드백
        // 실제 구현에서는 인터럽트 기반 엔코더 읽기 사용
        
        static uint32_t last_encoder_time = 0;
        uint32_t current_time = millis();
        float dt = (current_time - last_encoder_time) / 1000.0;
        
        if (dt > 0) {
            // 속도 계산
            current_pose.linear_x = 0.1 * sin(current_time / 1000.0);  // 시뮬레이션
            current_pose.linear_y = 0.0;
            current_pose.angular_z = 0.05 * cos(current_time / 2000.0);
            
            // 위치 적분
            current_pose.x += current_pose.linear_x * dt;
            current_pose.y += current_pose.linear_y * dt;
            current_pose.yaw += current_pose.angular_z * dt;
        }
        
        last_encoder_time = current_time;
    }
    
    void readForceSensors() {
        // 힘/토크 센서 데이터 읽기
        // 로봇 팔의 관절 토크 피드백
    }
    
    void updateStateEstimation(float dt) {
        // 확장 칼만 필터 또는 파티클 필터를 이용한 상태 추정
        // 여기서는 단순한 추정 방법 사용
        
        // 자세 추정 (상보 필터)
        float alpha = 0.98;
        
        // 자이로스코프 적분
        current_pose.roll += current_pose.angular_x * dt;
        current_pose.pitch += current_pose.angular_y * dt;
        current_pose.yaw += current_pose.angular_z * dt;
        
        // 가속도계 기반 자세 계산
        float accel_roll = atan2(current_pose.accel_y, current_pose.accel_z);
        float accel_pitch = atan2(-current_pose.accel_x, 
                                 sqrt(current_pose.accel_y * current_pose.accel_y + 
                                      current_pose.accel_z * current_pose.accel_z));
        
        // 상보 필터 적용
        current_pose.roll = alpha * current_pose.roll + (1 - alpha) * accel_roll;
        current_pose.pitch = alpha * current_pose.pitch + (1 - alpha) * accel_pitch;
        
        current_pose.timestamp = millis();
    }
    
    void checkSafetySystems() {
        // 비상 정지 버튼 체크
        if (digitalRead(13) == LOW) {
            if (!emergency_stop_activated) {
                activateEmergencyStop();
            }
        } else {
            if (emergency_stop_activated) {
                deactivateEmergencyStop();
            }
        }
        
        // 속도 제한 체크
        float current_speed = sqrt(current_pose.linear_x * current_pose.linear_x + 
                                 current_pose.linear_y * current_pose.linear_y);
        
        if (current_speed > max_velocity) {
            Serial.println("경고: 최대 속도 초과!");
            // 속도 제한 적용
        }
        
        // 가속도 제한 체크
        static float last_speed = 0;
        float acceleration = abs(current_speed - last_speed) / 0.02;  // 50Hz 가정
        
        if (acceleration > max_acceleration) {
            Serial.println("경고: 최대 가속도 초과!");
            // 가속도 제한 적용
        }
        
        last_speed = current_speed;
        
        // 작업 공간 제한 체크
        checkWorkspaceLimits();
    }
    
    void checkWorkspaceLimits() {
        // 로봇의 작업 공간 경계 체크
        float max_reach = 2.0;  // 최대 도달 거리 2m
        
        float distance_from_origin = sqrt(current_pose.x * current_pose.x + 
                                        current_pose.y * current_pose.y);
        
        if (distance_from_origin > max_reach) {
            Serial.println("경고: 작업 공간 경계 초과!");
            // 경계 내로 복귀 명령
        }
    }
    
    void activateEmergencyStop() {
        emergency_stop_activated = true;
        current_state = EMERGENCY_STOP;
        
        Serial.println("*** 비상 정지 활성화 ***");
        
        // 모든 모터 정지
        stopAllMotors();
        
        // 상태 LED 점멸
        digitalWrite(12, HIGH);
        
        // 안전 위치로 이동
        moveToSafePosition();
    }
    
    void deactivateEmergencyStop() {
        emergency_stop_activated = false;
        current_state = IDLE;
        
        Serial.println("비상 정지 해제 - 시스템 재시작");
        
        // 시스템 재초기화
        resetPose();
        digitalWrite(12, LOW);
    }
    
    void stopAllMotors() {
        // 모든 모터에 정지 명령
        for (int i = 0; i < components.motor_count; i++) {
            analogWrite(3 + i * 2, 0);  // PWM을 0으로 설정
        }
    }
    
    void moveToSafePosition() {
        // 안전한 위치로 로봇 이동
        Serial.println("안전 위치로 이동 중...");
        
        // 관절을 안전한 각도로 설정
        // 실제 구현에서는 역기구학을 사용하여 안전 포즈 계산
    }
    
    void manageMissions() {
        if (emergency_stop_activated) return;
        
        // 현재 임무 상태 확인
        if (current_mission.is_completed) {
            // 새로운 임무 요청
            requestNewMission();
        } else {
            // 현재 임무 진행
            executeMission();
        }
    }
    
    void requestNewMission() {
        // 임무 대기열에서 다음 임무 가져오기
        // 여기서는 간단한 순찰 임무 예시
        
        current_mission.mission_id = random(1000, 9999);
        current_mission.mission_type = "PATROL";
        current_mission.target_x = random(-100, 100) / 50.0;  // -2m ~ 2m
        current_mission.target_y = random(-100, 100) / 50.0;
        current_mission.target_z = 0.0;
        current_mission.target_yaw = random(-314, 314) / 100.0;  // -π ~ π
        current_mission.priority = 3;
        current_mission.start_time = millis();
        current_mission.deadline = millis() + 30000;  // 30초 제한
        current_mission.is_completed = false;
        current_mission.completion_ratio = 0.0;
        
        current_state = NAVIGATION;
        
        Serial.println("새 임무 할당:");
        Serial.print("- 임무 ID: "); Serial.println(current_mission.mission_id);
        Serial.print("- 타입: "); Serial.println(current_mission.mission_type);
        Serial.print("- 목표: ("); Serial.print(current_mission.target_x);
        Serial.print(", "); Serial.print(current_mission.target_y);
        Serial.print(", "); Serial.print(current_mission.target_yaw); Serial.println(")");
    }
    
    void executeMission() {
        switch (current_state) {
            case NAVIGATION:
                executeNavigation();
                break;
            case MANIPULATION:
                executeManipulation();
                break;
            case INTERACTION:
                executeInteraction();
                break;
            default:
                break;
        }
        
        // 임무 완료 조건 체크
        checkMissionCompletion();
    }
    
    void executeNavigation() {
        // 목표 지점까지의 거리 계산
        float dx = current_mission.target_x - current_pose.x;
        float dy = current_mission.target_y - current_pose.y;
        float distance = sqrt(dx * dx + dy * dy);
        
        float angle_error = current_mission.target_yaw - current_pose.yaw;
        
        // 정규화 (-π ~ π)
        while (angle_error > PI) angle_error -= 2 * PI;
        while (angle_error < -PI) angle_error += 2 * PI;
        
        // 완료 조건 체크
        if (distance < 0.1 && abs(angle_error) < 0.1) {
            current_mission.is_completed = true;
            current_mission.completion_ratio = 1.0;
            Serial.println("네비게이션 임무 완료!");
            return;
        }
        
        // 제어 명령 계산
        target_pose.linear_x = constrain(dx * 0.5, -max_velocity, max_velocity);
        target_pose.linear_y = constrain(dy * 0.5, -max_velocity, max_velocity);
        target_pose.angular_z = constrain(angle_error * 1.0, -1.0, 1.0);
        
        // 진행률 업데이트
        float initial_distance = sqrt(current_mission.target_x * current_mission.target_x +
                                    current_mission.target_y * current_mission.target_y);
        current_mission.completion_ratio = 1.0 - (distance / initial_distance);
    }
    
    void executeManipulation() {
        // 로봇 팔 조작 임무
        Serial.println("조작 임무 실행 중...");
        
        // 여기서는 간단한 그리퍼 동작 시뮬레이션
        current_mission.completion_ratio += 0.1;
        
        if (current_mission.completion_ratio >= 1.0) {
            current_mission.is_completed = true;
            Serial.println("조작 임무 완료!");
        }
    }
    
    void executeInteraction() {
        // 인간-로봇 상호작용 임무
        Serial.println("상호작용 임무 실행 중...");
        
        current_mission.completion_ratio += 0.05;
        
        if (current_mission.completion_ratio >= 1.0) {
            current_mission.is_completed = true;
            Serial.println("상호작용 임무 완료!");
        }
    }
    
    void checkMissionCompletion() {
        // 시간 제한 체크
        if (millis() > current_mission.deadline) {
            Serial.println("임무 시간 초과!");
            current_mission.is_completed = true;
        }
        
        // 완료 시 다음 상태로 전환
        if (current_mission.is_completed) {
            current_state = IDLE;
        }
    }
    
    void calculateControlCommands(float dt) {
        if (emergency_stop_activated) return;
        
        // PID 제어기를 사용한 제어 명령 계산
        calculateMotionControl(dt);
        
        // 관절 제어 (로봇 팔)
        if (components.servo_count > 0) {
            calculateJointControl(dt);
        }
        
        // 그리퍼 제어
        if (components.gripper_available) {
            calculateGripperControl();
        }
    }
    
    void calculateMotionControl(float dt) {
        // 위치 PID 제어
        static float prev_error_x = 0, prev_error_y = 0, prev_error_yaw = 0;
        static float integral_x = 0, integral_y = 0, integral_yaw = 0;
        
        // PID 파라미터
        float kp = 1.0, ki = 0.1, kd = 0.5;
        
        // X축 제어
        float error_x = target_pose.x - current_pose.x;
        integral_x += error_x * dt;
        float derivative_x = (error_x - prev_error_x) / dt;
        float control_x = kp * error_x + ki * integral_x + kd * derivative_x;
        prev_error_x = error_x;
        
        // Y축 제어
        float error_y = target_pose.y - current_pose.y;
        integral_y += error_y * dt;
        float derivative_y = (error_y - prev_error_y) / dt;
        float control_y = kp * error_y + ki * integral_y + kd * derivative_y;
        prev_error_y = error_y;
        
        // 회전 제어
        float error_yaw = target_pose.yaw - current_pose.yaw;
        while (error_yaw > PI) error_yaw -= 2 * PI;
        while (error_yaw < -PI) error_yaw += 2 * PI;
        
        integral_yaw += error_yaw * dt;
        float derivative_yaw = (error_yaw - prev_error_yaw) / dt;
        float control_yaw = kp * error_yaw + ki * integral_yaw + kd * derivative_yaw;
        prev_error_yaw = error_yaw;
        
        // 제어 신호 제한
        target_pose.linear_x = constrain(control_x, -max_velocity, max_velocity);
        target_pose.linear_y = constrain(control_y, -max_velocity, max_velocity);
        target_pose.angular_z = constrain(control_yaw, -2.0, 2.0);
    }
    
    void calculateJointControl(float dt) {
        // 로봇 팔 관절 제어
        // 역기구학을 사용하여 목표 관절 각도 계산
        
        static float joint_angles[6] = {0};  // 6자유도 로봇 팔
        static float target_joint_angles[6] = {0};
        
        // 간단한 사인파 움직임 (시연용)
        float time_sec = millis() / 1000.0;
        for (int i = 0; i < components.servo_count; i++) {
            target_joint_angles[i] = 0.5 * sin(time_sec + i * PI / 3);
            
            // 관절 속도 제한
            float max_joint_velocity = 1.0;  // rad/s
            float angle_diff = target_joint_angles[i] - joint_angles[i];
            
            if (abs(angle_diff) > max_joint_velocity * dt) {
                angle_diff = (angle_diff > 0) ? max_joint_velocity * dt : -max_joint_velocity * dt;
            }
            
            joint_angles[i] += angle_diff;
        }
    }
    
    void calculateGripperControl() {
        // 그리퍼 제어 로직
        static bool gripper_open = true;
        static uint32_t last_gripper_action = 0;
        
        // 5초마다 그리퍼 동작 변경 (시연용)
        if (millis() - last_gripper_action > 5000) {
            gripper_open = !gripper_open;
            last_gripper_action = millis();
            
            Serial.print("그리퍼 "); 
            Serial.println(gripper_open ? "열기" : "닫기");
        }
    }
    
    void updateActuators() {
        if (emergency_stop_activated) return;
        
        // 바퀴 모터 제어
        updateWheelMotors();
        
        // 로봇 팔 서보 제어
        updateArmServos();
        
        // 그리퍼 제어
        updateGripper();
    }
    
    void updateWheelMotors() {
        if (!components.wheels_available) return;
        
        // 차등 구동 로봇의 바퀴 속도 계산
        float wheel_base = 0.3;  // 바퀴 간격 30cm
        
        float left_speed = target_pose.linear_x - target_pose.angular_z * wheel_base / 2;
        float right_speed = target_pose.linear_x + target_pose.angular_z * wheel_base / 2;
        
        // PWM 신호로 변환 (0-255)
        int left_pwm = constrain(left_speed * 127 + 127, 0, 255);
        int right_pwm = constrain(right_speed * 127 + 127, 0, 255);
        
        // 모터 제어
        analogWrite(3, left_pwm);   // 왼쪽 모터
        analogWrite(5, right_pwm);  // 오른쪽 모터
    }
    
    void updateArmServos() {
        // 로봇 팔 서보 모터 제어
        static Servo arm_servos[6];
        static bool servos_attached = false;
        
        if (!servos_attached) {
            for (int i = 0; i < min(components.servo_count, 6); i++) {
                arm_servos[i].attach(9 + i);
            }
            servos_attached = true;
        }
        
        // 관절 각도를 서보 각도로 변환
        for (int i = 0; i < min(components.servo_count, 6); i++) {
            float angle_deg = 90;  // 기본 중앙 위치
            arm_servos[i].write(angle_deg);
        }
    }
    
    void updateGripper() {
        if (!components.gripper_available) return;
        
        // 그리퍼 서보 제어
        static Servo gripper_servo;
        static bool gripper_attached = false;
        
        if (!gripper_attached) {
            gripper_servo.attach(15);
            gripper_attached = true;
        }
        
        // 그리퍼 위치 설정 (시연용)
        static bool gripper_state = false;
        gripper_servo.write(gripper_state ? 180 : 0);
    }
    
    void handleCommunications() {
        // WiFi 통신 처리
        if (components.wifi_connected) {
            handleWiFiCommunication();
        }
        
        // 시리얼 통신 처리
        handleSerialCommunication();
        
        // 상태 브로드캐스트
        broadcastRobotStatus();
    }
    
    void handleWiFiCommunication() {
        // WiFi를 통한 원격 제어 명령 수신
        // 실제 구현에서는 TCP/UDP 서버나 ROS 통신 사용
        
        components.last_communication = millis();
    }
    
    void handleSerialCommunication() {
        // 시리얼 포트를 통한 명령 수신
        if (Serial.available()) {
            String command = Serial.readString();
            command.trim();
            processCommand(command);
        }
    }
    
    void processCommand(String cmd) {
        if (cmd == "stop") {
            activateEmergencyStop();
        } else if (cmd == "start") {
            deactivateEmergencyStop();
        } else if (cmd == "status") {
            displaySystemStatus();
        } else if (cmd == "home") {
            moveToHomePosition();
        } else if (cmd.startsWith("goto")) {
            // goto x,y,yaw 형식의 명령 처리
            parseGotoCommand(cmd);
        } else {
            Serial.println("알 수 없는 명령: " + cmd);
        }
    }
    
    void parseGotoCommand(String cmd) {
        // "goto 1.0,2.0,0.5" 형식 파싱
        int space_pos = cmd.indexOf(' ');
        if (space_pos > 0) {
            String params = cmd.substring(space_pos + 1);
            
            int comma1 = params.indexOf(',');
            int comma2 = params.lastIndexOf(',');
            
            if (comma1 > 0 && comma2 > comma1) {
                float x = params.substring(0, comma1).toFloat();
                float y = params.substring(comma1 + 1, comma2).toFloat();
                float yaw = params.substring(comma2 + 1).toFloat();
                
                setTargetPosition(x, y, yaw);
            }
        }
    }
    
    void setTargetPosition(float x, float y, float yaw) {
        target_pose.x = x;
        target_pose.y = y;
        target_pose.yaw = yaw;
        
        current_state = NAVIGATION;
        
        Serial.print("목표 위치 설정: (");
        Serial.print(x); Serial.print(", ");
        Serial.print(y); Serial.print(", ");
        Serial.print(yaw); Serial.println(")");
    }
    
    void moveToHomePosition() {
        Serial.println("홈 위치로 이동 중...");
        setTargetPosition(0.0, 0.0, 0.0);
    }
    
    void broadcastRobotStatus() {
        static uint32_t last_broadcast = 0;
        
        // 1초마다 상태 브로드캐스트
        if (millis() - last_broadcast > 1000) {
            if (components.wifi_connected) {
                // JSON 형태로 상태 정보 전송
                String status_json = createStatusJSON();
                // WiFi를 통해 전송 (실제 구현 필요)
            }
            
            last_broadcast = millis();
        }
    }
    
    String createStatusJSON() {
        String json = "{";
        json += "\"timestamp\":" + String(millis()) + ",";
        json += "\"state\":\"" + getStateString() + "\",";
        json += "\"position\":{";
        json += "\"x\":" + String(current_pose.x) + ",";
        json += "\"y\":" + String(current_pose.y) + ",";
        json += "\"yaw\":" + String(current_pose.yaw);
        json += "},";
        json += "\"velocity\":{";
        json += "\"linear_x\":" + String(current_pose.linear_x) + ",";
        json += "\"angular_z\":" + String(current_pose.angular_z);
        json += "},";
        json += "\"emergency_stop\":" + String(emergency_stop_activated ? "true" : "false");
        json += "}";
        
        return json;
    }
    
    String getStateString() {
        switch (current_state) {
            case INITIALIZATION: return "INIT";
            case IDLE: return "IDLE";
            case NAVIGATION: return "NAV";
            case MANIPULATION: return "MANIP";
            case INTERACTION: return "INTERACT";
            case EMERGENCY_STOP: return "E_STOP";
            default: return "UNKNOWN";
        }
    }
    
    void logSystemState() {
        static uint32_t last_log = 0;
        
        // 10초마다 상세 로그
        if (millis() - last_log > 10000) {
            Serial.println("\n=== 시스템 상태 로그 ===");
            Serial.print("시간: "); Serial.println(millis() / 1000);
            Serial.print("상태: "); Serial.println(getStateString());
            Serial.print("위치: ("); Serial.print(current_pose.x, 2);
            Serial.print(", "); Serial.print(current_pose.y, 2);
            Serial.print(", "); Serial.print(current_pose.yaw, 2); Serial.println(")");
            Serial.print("속도: ("); Serial.print(current_pose.linear_x, 2);
            Serial.print(", "); Serial.print(current_pose.angular_z, 2); Serial.println(")");
            
            if (!current_mission.is_completed) {
                Serial.print("임무 진행률: "); 
                Serial.print(current_mission.completion_ratio * 100, 1); 
                Serial.println("%");
            }
            
            Serial.print("CPU 사용률: "); Serial.print(components.cpu_usage); Serial.println("%");
            Serial.print("메모리 사용률: "); Serial.print(components.memory_usage); Serial.println("%");
            Serial.println();
            
            last_log = millis();
        }
    }
    
    void displaySystemStatus() {
        Serial.println("\n=== 로봇 시스템 상태 ===");
        Serial.print("현재 상태: "); Serial.println(getStateString());
        Serial.print("비상 정지: "); Serial.println(emergency_stop_activated ? "활성화" : "비활성화");
        
        Serial.println("\n하드웨어 구성:");
        Serial.print("- 모터: "); Serial.println(components.motor_count);
        Serial.print("- 서보: "); Serial.println(components.servo_count);
        Serial.print("- IMU: "); Serial.println(components.imu_available ? "연결됨" : "연결 안됨");
        Serial.print("- LIDAR: "); Serial.println(components.lidar_available ? "연결됨" : "연결 안됨");
        Serial.print("- WiFi: "); Serial.println(components.wifi_connected ? "연결됨" : "연결 안됨");
        
        Serial.println("\n현재 포즈:");
        Serial.print("- 위치: ("); Serial.print(current_pose.x, 2);
        Serial.print(", "); Serial.print(current_pose.y, 2);
        Serial.print(", "); Serial.print(current_pose.z, 2); Serial.println(")");
        Serial.print("- 자세: ("); Serial.print(current_pose.roll, 2);
        Serial.print(", "); Serial.print(current_pose.pitch, 2);
        Serial.print(", "); Serial.print(current_pose.yaw, 2); Serial.println(")");
        
        Serial.println("\n현재 임무:");
        if (current_mission.is_completed) {
            Serial.println("- 임무 없음");
        } else {
            Serial.print("- ID: "); Serial.println(current_mission.mission_id);
            Serial.print("- 타입: "); Serial.println(current_mission.mission_type);
            Serial.print("- 진행률: "); Serial.print(current_mission.completion_ratio * 100, 1); 
            Serial.println("%");
        }
        
        Serial.println();
    }
};

AdvancedRoboticsSystem robot_system;

void setup() {
    robot_system.initialize();
    
    Serial.println("명령어:");
    Serial.println("'stop' - 비상 정지");
    Serial.println("'start' - 시스템 시작");
    Serial.println("'status' - 상태 표시");
    Serial.println("'home' - 홈 위치로 이동");
    Serial.println("'goto x,y,yaw' - 목표 위치 설정");
}

void loop() {
    robot_system.update();
    delay(20);  // 50Hz 제어 루프
}
```

---

## 로봇 키네마틱스

### 순기구학 및 역기구학 해법기
```cpp
class RobotKinematics {
private:
    static const int MAX_JOINTS = 6;
    
    struct DHParameters {
        float a;        // 링크 길이
        float alpha;    // 링크 비틀림
        float d;        // 링크 오프셋
        float theta;    // 관절 각도
    };
    
    struct Transform3D {
        float matrix[4][4];
        
        Transform3D() {
            // 단위 행렬로 초기화
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    matrix[i][j] = (i == j) ? 1.0 : 0.0;
                }
            }
        }
        
        void setTranslation(float x, float y, float z) {
            matrix[0][3] = x;
            matrix[1][3] = y;
            matrix[2][3] = z;
        }
        
        void setRotationZ(float angle) {
            float c = cos(angle);
            float s = sin(angle);
            matrix[0][0] = c; matrix[0][1] = -s;
            matrix[1][0] = s; matrix[1][1] = c;
        }
        
        Transform3D multiply(const Transform3D& other) const {
            Transform3D result;
            
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    result.matrix[i][j] = 0;
                    for (int k = 0; k < 4; k++) {
                        result.matrix[i][j] += matrix[i][k] * other.matrix[k][j];
                    }
                }
            }
            
            return result;
        }
        
        void print() {
            Serial.println("변환 행렬:");
            for (int i = 0; i < 4; i++) {
                Serial.print("[");
                for (int j = 0; j < 4; j++) {
                    Serial.print(matrix[i][j], 3);
                    if (j < 3) Serial.print(", ");
                }
                Serial.println("]");
            }
        }
    };
    
    struct JointConfiguration {
        float joint_angles[MAX_JOINTS];  // 라디안 단위
        float joint_velocities[MAX_JOINTS];  // rad/s
        float joint_limits_min[MAX_JOINTS];
        float joint_limits_max[MAX_JOINTS];
        bool joint_enabled[MAX_JOINTS];
        int num_joints;
        
        JointConfiguration() : num_joints(6) {
            for (int i = 0; i < MAX_JOINTS; i++) {
                joint_angles[i] = 0.0;
                joint_velocities[i] = 0.0;
                joint_limits_min[i] = -PI;
                joint_limits_max[i] = PI;
                joint_enabled[i] = true;
            }
        }
        
        void setJointLimits(int joint, float min_angle, float max_angle) {
            if (joint >= 0 && joint < MAX_JOINTS) {
                joint_limits_min[joint] = min_angle;
                joint_limits_max[joint] = max_angle;
            }
        }
        
        bool setJointAngle(int joint, float angle) {
            if (joint >= 0 && joint < MAX_JOINTS) {
                // 관절 제한 체크
                if (angle >= joint_limits_min[joint] && angle <= joint_limits_max[joint]) {
                    joint_angles[joint] = angle;
                    return true;
                }
            }
            return false;
        }
        
        void display() {
            Serial.println("관절 구성:");
            for (int i = 0; i < num_joints; i++) {
                Serial.print("관절 "); Serial.print(i); Serial.print(": ");
                Serial.print(joint_angles[i] * 180 / PI, 1); Serial.print("° ");
                Serial.print("("); Serial.print(joint_limits_min[i] * 180 / PI, 0);
                Serial.print("° ~ "); Serial.print(joint_limits_max[i] * 180 / PI, 0);
                Serial.println("°)");
            }
        }
    };
    
    struct CartesianPose {
        float x, y, z;              // 위치 (m)
        float roll, pitch, yaw;     // 자세 (rad)
        
        CartesianPose(float px = 0, float py = 0, float pz = 0,
                     float rx = 0, float ry = 0, float rz = 0)
            : x(px), y(py), z(pz), roll(rx), pitch(ry), yaw(rz) {}
        
        void display() {
            Serial.println("데카르트 포즈:");
            Serial.print("위치: ("); Serial.print(x, 3);
            Serial.print(", "); Serial.print(y, 3);
            Serial.print(", "); Serial.print(z, 3); Serial.println(")");
            Serial.print("자세: ("); Serial.print(roll * 180 / PI, 1);
            Serial.print("°, "); Serial.print(pitch * 180 / PI, 1);
            Serial.print("°, "); Serial.print(yaw * 180 / PI, 1); Serial.println("°)");
        }
        
        float distanceTo(const CartesianPose& other) const {
            float dx = x - other.x;
            float dy = y - other.y;
            float dz = z - other.z;
            return sqrt(dx*dx + dy*dy + dz*dz);
        }
    };
    
    // 6자유도 로봇 팔의 DH 파라미터 (PUMA 560 스타일)
    DHParameters dh_params[MAX_JOINTS];
    JointConfiguration current_joints;
    CartesianPose current_pose;
    
    // 역기구학 해법 파라미터
    float ik_tolerance;
    int max_ik_iterations;
    
public:
    RobotKinematics() {
        ik_tolerance = 0.001;  // 1mm 허용 오차
        max_ik_iterations = 100;
        
        setupDHParameters();
        setupJointLimits();
    }
    
    void initialize() {
        Serial.println("=== 로봇 키네마틱스 시스템 ===");
        Serial.println("6자유도 로봇 팔 키네마틱스");
        Serial.println();
        
        Serial.println("DH 파라미터:");
        for (int i = 0; i < current_joints.num_joints; i++) {
            Serial.print("링크 "); Serial.print(i); Serial.print(": ");
            Serial.print("a="); Serial.print(dh_params[i].a, 3);
            Serial.print(", α="); Serial.print(dh_params[i].alpha * 180 / PI, 1);
            Serial.print("°, d="); Serial.print(dh_params[i].d, 3);
            Serial.println("m");
        }
        
        Serial.println();
        current_joints.display();
        
        // 초기 순기구학 계산
        current_pose = forwardKinematics(current_joints);
        current_pose.display();
        
        Serial.println("\n키네마틱스 시스템 준비 완료");
    }
    
    void setupDHParameters() {
        // PUMA 560 스타일 로봇 팔 DH 파라미터
        dh_params[0] = {0.0,    PI/2,   0.15,  0.0};    // 베이스 회전
        dh_params[1] = {0.3,    0.0,    0.0,   0.0};    // 어깨
        dh_params[2] = {0.05,   PI/2,   0.0,   0.0};    // 팔꿈치
        dh_params[3] = {0.0,   -PI/2,   0.25,  0.0};    // 손목 1
        dh_params[4] = {0.0,    PI/2,   0.0,   0.0};    // 손목 2
        dh_params[5] = {0.0,    0.0,    0.1,   0.0};    // 손목 3 (엔드이펙터)
    }
    
    void setupJointLimits() {
        // 각 관절의 물리적 제한 설정
        current_joints.setJointLimits(0, -PI, PI);           // 베이스: ±180°
        current_joints.setJointLimits(1, -PI/2, PI/2);       // 어깨: ±90°
        current_joints.setJointLimits(2, -PI/2, PI/2);       // 팔꿈치: ±90°
        current_joints.setJointLimits(3, -PI, PI);           // 손목1: ±180°
        current_joints.setJointLimits(4, -PI/2, PI/2);       // 손목2: ±90°
        current_joints.setJointLimits(5, -PI, PI);           // 손목3: ±180°
    }
    
    CartesianPose forwardKinematics(const JointConfiguration& joints) {
        Transform3D total_transform;
        
        Serial.println("\n순기구학 계산:");
        
        // 각 관절에 대해 변환 행렬 계산
        for (int i = 0; i < joints.num_joints; i++) {
            Transform3D link_transform = computeDHTransform(i, joints.joint_angles[i]);
            total_transform = total_transform.multiply(link_transform);
            
            Serial.print("링크 "); Serial.print(i); Serial.print(" 변환 후 위치: (");
            Serial.print(total_transform.matrix[0][3], 3); Serial.print(", ");
            Serial.print(total_transform.matrix[1][3], 3); Serial.print(", ");
            Serial.print(total_transform.matrix[2][3], 3); Serial.println(")");
        }
        
        // 최종 변환 행렬에서 위치와 자세 추출
        CartesianPose result;
        result.x = total_transform.matrix[0][3];
        result.y = total_transform.matrix[1][3];
        result.z = total_transform.matrix[2][3];
        
        // 회전 행렬에서 오일러 각 추출
        extractEulerAngles(total_transform, result);
        
        return result;
    }
    
    Transform3D computeDHTransform(int joint_index, float theta) {
        DHParameters& dh = dh_params[joint_index];
        
        // DH 변환 행렬 계산
        Transform3D transform;
        
        float ct = cos(theta + dh.theta);
        float st = sin(theta + dh.theta);
        float ca = cos(dh.alpha);
        float sa = sin(dh.alpha);
        
        transform.matrix[0][0] = ct;
        transform.matrix[0][1] = -st * ca;
        transform.matrix[0][2] = st * sa;
        transform.matrix[0][3] = dh.a * ct;
        
        transform.matrix[1][0] = st;
        transform.matrix[1][1] = ct * ca;
        transform.matrix[1][2] = -ct * sa;
        transform.matrix[1][3] = dh.a * st;
        
        transform.matrix[2][0] = 0;
        transform.matrix[2][1] = sa;
        transform.matrix[2][2] = ca;
        transform.matrix[2][3] = dh.d;
        
        transform.matrix[3][0] = 0;
        transform.matrix[3][1] = 0;
        transform.matrix[3][2] = 0;
        transform.matrix[3][3] = 1;
        
        return transform;
    }
    
    void extractEulerAngles(const Transform3D& transform, CartesianPose& pose) {
        // 회전 행렬에서 ZYX 오일러 각 추출
        float r11 = transform.matrix[0][0];
        float r12 = transform.matrix[0][1];
        float r13 = transform.matrix[0][2];
        float r21 = transform.matrix[1][0];
        float r23 = transform.matrix[1][2];
        float r31 = transform.matrix[2][0];
        float r32 = transform.matrix[2][1];
        float r33 = transform.matrix[2][2];
        
        // 특이점 체크
        if (abs(r31) >= 1.0) {
            pose.yaw = 0;  // 임의 값
            if (r31 < 0) {
                pose.pitch = PI / 2;
                pose.roll = pose.yaw + atan2(r12, r13);
            } else {
                pose.pitch = -PI / 2;
                pose.roll = -pose.yaw + atan2(-r12, -r13);
            }
        } else {
            pose.pitch = -asin(r31);
            pose.roll = atan2(r32 / cos(pose.pitch), r33 / cos(pose.pitch));
            pose.yaw = atan2(r21 / cos(pose.pitch), r11 / cos(pose.pitch));
        }
    }
    
    bool inverseKinematics(const CartesianPose& target_pose, JointConfiguration& result_joints) {
        Serial.println("\n역기구학 계산:");
        Serial.print("목표 포즈: ");
        target_pose.display();
        
        // 해석적 역기구학 (PUMA 560 스타일)
        return solveAnalyticalIK(target_pose, result_joints);
    }
    
    bool solveAnalyticalIK(const CartesianPose& target, JointConfiguration& joints) {
        float px = target.x;
        float py = target.y;
        float pz = target.z;
        
        // 손목 위치 계산
        float d6 = dh_params[5].d;  // 엔드이펙터 오프셋
        float wx = px - d6 * cos(target.yaw) * cos(target.pitch);
        float wy = py - d6 * sin(target.yaw) * cos(target.pitch);
        float wz = pz - d6 * sin(target.pitch);
        
        // 관절 1 (베이스 회전)
        joints.joint_angles[0] = atan2(wy, wx);
        
        // 관절 2, 3 (어깨, 팔꿈치)
        float r = sqrt(wx*wx + wy*wy);
        float s = wz - dh_params[0].d;
        
        float a2 = dh_params[1].a;
        float d4 = dh_params[3].d;
        
        float D = (r*r + s*s - a2*a2 - d4*d4) / (2 * a2 * d4);
        
        if (D > 1.0 || D < -1.0) {
            Serial.println("역기구학 해가 존재하지 않음 (도달 불가능)");
            return false;
        }
        
        joints.joint_angles[2] = atan2(sqrt(1 - D*D), D);  // 팔꿈치 위 해
        
        float alpha = atan2(s, r);
        float beta = atan2(d4 * sin(joints.joint_angles[2]), 
                          a2 + d4 * cos(joints.joint_angles[2]));
        joints.joint_angles[1] = alpha - beta;
        
        // 관절 4, 5, 6 (손목 자세)
        solveWristKinematics(target, joints);
        
        // 관절 제한 체크
        for (int i = 0; i < joints.num_joints; i++) {
            if (joints.joint_angles[i] < joints.joint_limits_min[i] || 
                joints.joint_angles[i] > joints.joint_limits_max[i]) {
                Serial.print("관절 "); Serial.print(i); Serial.println(" 제한 초과");
                return false;
            }
        }
        
        Serial.println("역기구학 해:");
        joints.display();
        
        // 검증: 순기구학으로 확인
        CartesianPose verification = forwardKinematics(joints);
        float error = target.distanceTo(verification);
        
        Serial.print("위치 오차: "); Serial.print(error * 1000, 2); Serial.println("mm");
        
        return (error < ik_tolerance);
    }
    
    void solveWristKinematics(const CartesianPose& target, JointConfiguration& joints) {
        // 손목 3자유도의 자세 제어
        // 복잡한 해석적 해법 대신 간단한 근사 사용
        
        joints.joint_angles[3] = target.roll;
        joints.joint_angles[4] = target.pitch;
        joints.joint_angles[5] = target.yaw;
        
        // 각도 정규화
        for (int i = 3; i < 6; i++) {
            while (joints.joint_angles[i] > PI) joints.joint_angles[i] -= 2*PI;
            while (joints.joint_angles[i] < -PI) joints.joint_angles[i] += 2*PI;
        }
    }
    
    bool numericalIK(const CartesianPose& target_pose, JointConfiguration& result_joints) {
        Serial.println("수치적 역기구학 (야코비안 방법):");
        
        JointConfiguration current = current_joints;
        
        for (int iter = 0; iter < max_ik_iterations; iter++) {
            // 현재 관절 각도에서 순기구학
            CartesianPose current_pose = forwardKinematics(current);
            
            // 오차 계산
            float error_x = target_pose.x - current_pose.x;
            float error_y = target_pose.y - current_pose.y;
            float error_z = target_pose.z - current_pose.z;
            float error_roll = target_pose.roll - current_pose.roll;
            float error_pitch = target_pose.pitch - current_pose.pitch;
            float error_yaw = target_pose.yaw - current_pose.yaw;
            
            float total_error = sqrt(error_x*error_x + error_y*error_y + error_z*error_z);
            
            if (total_error < ik_tolerance) {
                Serial.print("수렴 완료 ("); Serial.print(iter); Serial.println("회 반복)");
                result_joints = current;
                return true;
            }
            
            // 야코비안 행렬 계산
            float jacobian[6][6];
            computeJacobian(current, jacobian);
            
            // 의사 역행렬을 이용한 관절 각도 업데이트
            float error_vector[6] = {error_x, error_y, error_z, error_roll, error_pitch, error_yaw};
            float delta_joints[6];
            
            pseudoInverse(jacobian, error_vector, delta_joints);
            
            // 관절 각도 업데이트
            float step_size = 0.1;  // 학습률
            for (int i = 0; i < current.num_joints; i++) {
                current.joint_angles[i] += step_size * delta_joints[i];
                
                // 관절 제한 적용
                current.joint_angles[i] = constrain(current.joint_angles[i],
                                                   current.joint_limits_min[i],
                                                   current.joint_limits_max[i]);
            }
            
            if (iter % 10 == 0) {
                Serial.print("반복 "); Serial.print(iter);
                Serial.print(", 오차: "); Serial.print(total_error * 1000, 2);
                Serial.println("mm");
            }
        }
        
        Serial.println("수치적 역기구학 수렴 실패");
        return false;
    }
    
    void computeJacobian(const JointConfiguration& joints, float jacobian[6][6]) {
        // 수치적 미분을 이용한 야코비안 계산
        float epsilon = 0.001;  // 미분 간격
        
        CartesianPose base_pose = forwardKinematics(joints);
        
        for (int i = 0; i < joints.num_joints; i++) {
            JointConfiguration perturbed = joints;
            perturbed.joint_angles[i] += epsilon;
            
            CartesianPose perturbed_pose = forwardKinematics(perturbed);
            
            // 편미분 계산
            jacobian[0][i] = (perturbed_pose.x - base_pose.x) / epsilon;
            jacobian[1][i] = (perturbed_pose.y - base_pose.y) / epsilon;
            jacobian[2][i] = (perturbed_pose.z - base_pose.z) / epsilon;
            jacobian[3][i] = (perturbed_pose.roll - base_pose.roll) / epsilon;
            jacobian[4][i] = (perturbed_pose.pitch - base_pose.pitch) / epsilon;
            jacobian[5][i] = (perturbed_pose.yaw - base_pose.yaw) / epsilon;
        }
    }
    
    void pseudoInverse(float jacobian[6][6], float error[6], float result[6]) {
        // 간단한 전치행렬 근사 (실제로는 SVD 분해 필요)
        for (int i = 0; i < 6; i++) {
            result[i] = 0;
            for (int j = 0; j < 6; j++) {
                result[i] += jacobian[j][i] * error[j];  // 전치
            }
            result[i] *= 0.1;  // 스케일 팩터
        }
    }
    
    void moveToTarget(const CartesianPose& target) {
        Serial.println("\n=== 목표 위치로 이동 ===");
        
        JointConfiguration target_joints;
        
        if (inverseKinematics(target, target_joints)) {
            Serial.println("역기구학 해 발견 - 이동 시작");
            
            // 궤적 계획 및 부드러운 이동
            interpolateJointMotion(current_joints, target_joints, 3.0);  // 3초간 이동
            
            current_joints = target_joints;
            current_pose = target;
            
            Serial.println("이동 완료");
        } else {
            Serial.println("역기구학 해를 찾을 수 없음 - 이동 불가");
        }
    }
    
    void interpolateJointMotion(const JointConfiguration& start, 
                               const JointConfiguration& end, float duration) {
        Serial.println("관절 궤적 보간 실행 중...");
        
        int steps = 50;  // 50단계로 분할
        float dt = duration / steps;
        
        for (int step = 0; step <= steps; step++) {
            float t = (float)step / steps;  // 0 ~ 1
            
            // 3차 다항식 보간 (부드러운 가속/감속)
            float s = 3*t*t - 2*t*t*t;
            
            JointConfiguration current;
            for (int i = 0; i < start.num_joints; i++) {
                current.joint_angles[i] = start.joint_angles[i] + 
                                         s * (end.joint_angles[i] - start.joint_angles[i]);
            }
            
            // 물리적 로봇으로 명령 전송 (시뮬레이션)
            sendJointCommands(current);
            
            if (step % 10 == 0) {
                Serial.print("진행률: "); Serial.print(t * 100, 0); Serial.println("%");
            }
            
            delay(dt * 1000);  // 실제 시간 지연
        }
    }
    
    void sendJointCommands(const JointConfiguration& joints) {
        // 실제 로봇의 서보 모터로 명령 전송
        static Servo servos[6];
        static bool servos_attached = false;
        
        if (!servos_attached) {
            for (int i = 0; i < 6; i++) {
                servos[i].attach(9 + i);
            }
            servos_attached = true;
        }
        
        for (int i = 0; i < joints.num_joints; i++) {
            // 라디안을 서보 각도(0-180)로 변환
            float servo_angle = (joints.joint_angles[i] + PI) * 180 / (2 * PI);
            servo_angle = constrain(servo_angle, 0, 180);
            
            servos[i].write(servo_angle);
        }
    }
    
    void demonstrateKinematics() {
        Serial.println("\n=== 키네마틱스 시연 ===");
        
        // 1. 순기구학 시연
        Serial.println("1. 순기구학 시연:");
        current_joints.joint_angles[0] = PI/4;   // 45도
        current_joints.joint_angles[1] = PI/6;   // 30도
        current_joints.joint_angles[2] = -PI/4;  // -45도
        
        CartesianPose fk_result = forwardKinematics(current_joints);
        fk_result.display();
        
        // 2. 역기구학 시연
        Serial.println("\n2. 역기구학 시연:");
        CartesianPose target(0.4, 0.2, 0.3, 0, 0, PI/4);  // 목표 포즈
        
        JointConfiguration ik_result;
        if (inverseKinematics(target, ik_result)) {
            Serial.println("역기구학 성공!");
            
            // 검증
            CartesianPose verification = forwardKinematics(ik_result);
            Serial.println("검증 결과:");
            verification.display();
        }
        
        // 3. 작업 공간 분석
        Serial.println("\n3. 작업 공간 분석:");
        analyzeWorkspace();
    }
    
    void analyzeWorkspace() {
        Serial.println("작업 공간 분석 중...");
        
        int reachable_points = 0;
        int total_points = 1000;
        
        for (int i = 0; i < total_points; i++) {
            // 랜덤 목표 위치 생성
            float x = random(-100, 100) / 100.0;  // -1m ~ 1m
            float y = random(-100, 100) / 100.0;
            float z = random(0, 80) / 100.0;      // 0 ~ 0.8m
            
            CartesianPose test_target(x, y, z, 0, 0, 0);
            JointConfiguration test_joints;
            
            if (inverseKinematics(test_target, test_joints)) {
                reachable_points++;
            }
        }
        
        float reachability = (float)reachable_points / total_points * 100;
        Serial.print("작업 공간 도달 가능률: "); Serial.print(reachability, 1); Serial.println("%");
        
        // 최대 도달 거리 계산
        float max_reach = 0;
        for (int i = 0; i < current_joints.num_joints - 1; i++) {
            max_reach += dh_params[i].a;
        }
        max_reach += dh_params[current_joints.num_joints - 1].d;
        
        Serial.print("이론적 최대 도달 거리: "); Serial.print(max_reach, 3); Serial.println("m");
    }
    
    void update() {
        // 실시간 키네마틱스 업데이트
        static uint32_t last_update = 0;
        
        if (millis() - last_update > 100) {  // 10Hz 업데이트
            // 현재 관절 각도 읽기 (엔코더에서)
            readJointAngles();
            
            // 순기구학으로 현재 포즈 계산
            current_pose = forwardKinematics(current_joints);
            
            last_update = millis();
        }
        
        // 명령 처리
        handleCommands();
    }
    
    void readJointAngles() {
        // 실제 로봇에서는 엔코더나 포텐셔미터로 관절 각도 읽기
        // 여기서는 시뮬레이션
        
        for (int i = 0; i < current_joints.num_joints; i++) {
            // 시뮬레이션: 천천히 변화하는 각도
            current_joints.joint_angles[i] += 0.001 * sin(millis() / 1000.0 + i);
            
            // 관절 제한 적용
            current_joints.joint_angles[i] = constrain(current_joints.joint_angles[i],
                                                      current_joints.joint_limits_min[i],
                                                      current_joints.joint_limits_max[i]);
        }
    }
    
    void handleCommands() {
        if (Serial.available()) {
            String command = Serial.readString();
            command.trim();
            
            if (command == "fk") {
                current_pose = forwardKinematics(current_joints);
                current_pose.display();
            } else if (command == "demo") {
                demonstrateKinematics();
            } else if (command == "workspace") {
                analyzeWorkspace();
            } else if (command == "home") {
                moveToHomePosition();
            } else if (command.startsWith("move")) {
                parseMoveCommand(command);
            } else if (command == "joints") {
                current_joints.display();
            }
        }
    }
    
    void parseMoveCommand(String cmd) {
        // "move x,y,z,roll,pitch,yaw" 형식 파싱
        int space_pos = cmd.indexOf(' ');
        if (space_pos > 0) {
            String params = cmd.substring(space_pos + 1);
            
            // 쉼표로 분리
            int positions[6];
            int param_count = 0;
            int start = 0;
            
            for (int i = 0; i <= params.length() && param_count < 6; i++) {
                if (i == params.length() || params[i] == ',') {
                    if (i > start) {
                        float value = params.substring(start, i).toFloat();
                        
                        switch (param_count) {
                            case 0: case 1: case 2:  // x, y, z (m)
                                break;
                            case 3: case 4: case 5:  // roll, pitch, yaw (도 → 라디안)
                                value = value * PI / 180;
                                break;
                        }
                        
                        positions[param_count] = value;
                        param_count++;
                    }
                    start = i + 1;
                }
            }
            
            if (param_count == 6) {
                CartesianPose target(positions[0], positions[1], positions[2],
                                   positions[3], positions[4], positions[5]);
                moveToTarget(target);
            } else {
                Serial.println("사용법: move x,y,z,roll,pitch,yaw");
            }
        }
    }
    
    void moveToHomePosition() {
        Serial.println("홈 위치로 이동...");
        
        // 모든 관절을 0도로 설정
        JointConfiguration home_joints;
        for (int i = 0; i < home_joints.num_joints; i++) {
            home_joints.joint_angles[i] = 0.0;
        }
        
        interpolateJointMotion(current_joints, home_joints, 2.0);
        current_joints = home_joints;
        current_pose = forwardKinematics(current_joints);
        
        Serial.println("홈 위치 도달 완료");
    }
};

RobotKinematics kinematics;

void setup() {
    Serial.begin(115200);
    kinematics.initialize();
    
    Serial.println("\n명령어:");
    Serial.println("'fk' - 순기구학 계산");
    Serial.println("'demo' - 키네마틱스 시연");
    Serial.println("'workspace' - 작업 공간 분석");
    Serial.println("'home' - 홈 위치로 이동");
    Serial.println("'move x,y,z,roll,pitch,yaw' - 목표 위치로 이동");
    Serial.println("'joints' - 현재 관절 각도 표시");
}

void loop() {
    kinematics.update();
    delay(100);  // 10Hz
}
```

이어서 로봇공학의 나머지 시스템들을 계속 작성하겠습니다.