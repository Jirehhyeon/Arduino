# 양자 컴퓨팅 시뮬레이션 시스템 🔬

## 개요
Arduino를 이용하여 양자 컴퓨팅의 기본 원리를 시뮬레이션하고 학습하는 완전 가이드입니다.

## 목차
1. [양자 컴퓨팅 기초](#양자-컴퓨팅-기초)
2. [큐비트 시뮬레이션](#큐비트-시뮬레이션)
3. [양자 게이트 구현](#양자-게이트-구현)
4. [양자 얽힘 시뮬레이션](#양자-얽힘-시뮬레이션)
5. [양자 알고리즘 구현](#양자-알고리즘-구현)
6. [양자 암호화 시스템](#양자-암호화-시스템)
7. [양자 오류 정정](#양자-오류-정정)
8. [양자 텔레포테이션](#양자-텔레포테이션)
9. [양자 시뮬레이터](#양자-시뮬레이터)
10. [통합 양자 컴퓨팅 랩](#통합-양자-컴퓨팅-랩)

---

## 양자 컴퓨팅 기초

### 양자역학 기본 원리
- 중첩(Superposition): 큐비트가 0과 1을 동시에 가질 수 있는 상태
- 얽힘(Entanglement): 큐비트들 간의 상관관계
- 측정(Measurement): 큐비트 상태의 확률적 붕괴
- 간섭(Interference): 확률 진폭의 상호작용

### 기본 구조
```cpp
// 양자 컴퓨팅 시뮬레이션 기본 클래스
#include <math.h>
#include <complex.h>

class QuantumComputingLab {
private:
    // 복소수 구조체 (C++에서 complex 대신 사용)
    struct Complex {
        float real;
        float imag;
        
        Complex(float r = 0, float i = 0) : real(r), imag(i) {}
        
        Complex operator+(const Complex& other) const {
            return Complex(real + other.real, imag + other.imag);
        }
        
        Complex operator-(const Complex& other) const {
            return Complex(real - other.real, imag - other.imag);
        }
        
        Complex operator*(const Complex& other) const {
            return Complex(real * other.real - imag * other.imag,
                          real * other.imag + imag * other.real);
        }
        
        float magnitude() const {
            return sqrt(real * real + imag * imag);
        }
        
        float phase() const {
            return atan2(imag, real);
        }
        
        Complex conjugate() const {
            return Complex(real, -imag);
        }
    };
    
    // 큐비트 상태 표현
    struct Qubit {
        Complex alpha;  // |0⟩ 상태의 확률 진폭
        Complex beta;   // |1⟩ 상태의 확률 진폭
        bool measured;  // 측정 여부
        int measured_value; // 측정 결과 (0 또는 1)
        
        Qubit() : alpha(1, 0), beta(0, 0), measured(false), measured_value(-1) {}
        
        // 확률 계산
        float probability_0() const {
            return alpha.magnitude() * alpha.magnitude();
        }
        
        float probability_1() const {
            return beta.magnitude() * beta.magnitude();
        }
        
        // 정규화
        void normalize() {
            float norm = sqrt(probability_0() + probability_1());
            if (norm > 0) {
                alpha.real /= norm;
                alpha.imag /= norm;
                beta.real /= norm;
                beta.imag /= norm;
            }
        }
        
        // 측정
        int measure() {
            if (measured) return measured_value;
            
            float prob_0 = probability_0();
            float random_val = random(0, 1000) / 1000.0;
            
            if (random_val < prob_0) {
                alpha = Complex(1, 0);
                beta = Complex(0, 0);
                measured_value = 0;
            } else {
                alpha = Complex(0, 0);
                beta = Complex(1, 0);
                measured_value = 1;
            }
            
            measured = true;
            return measured_value;
        }
        
        // 상태 초기화
        void reset() {
            alpha = Complex(1, 0);
            beta = Complex(0, 0);
            measured = false;
            measured_value = -1;
        }
    };
    
    // 양자 레지스터 (다중 큐비트)
    struct QuantumRegister {
        static const int MAX_QUBITS = 8;
        Qubit qubits[MAX_QUBITS];
        int num_qubits;
        Complex state_vector[256]; // 2^8 = 256 가지 상태
        bool using_state_vector;
        
        QuantumRegister(int n = 1) : num_qubits(min(n, MAX_QUBITS)), using_state_vector(false) {
            // 모든 큐비트를 |0⟩ 상태로 초기화
            for (int i = 0; i < num_qubits; i++) {
                qubits[i].reset();
            }
            
            // 상태 벡터 모드가 필요한 경우
            if (num_qubits > 2) {
                using_state_vector = true;
                initializeStateVector();
            }
        }
        
        void initializeStateVector() {
            int total_states = 1 << num_qubits; // 2^num_qubits
            for (int i = 0; i < total_states; i++) {
                state_vector[i] = (i == 0) ? Complex(1, 0) : Complex(0, 0);
            }
        }
        
        // 특정 큐비트 측정
        int measureQubit(int index) {
            if (index >= num_qubits) return -1;
            return qubits[index].measure();
        }
        
        // 모든 큐비트 측정
        String measureAll() {
            String result = "";
            for (int i = 0; i < num_qubits; i++) {
                result += String(measureQubit(i));
            }
            return result;
        }
        
        // 확률 분포 계산
        float getProbability(String bitstring) {
            if (bitstring.length() != num_qubits) return 0;
            
            if (using_state_vector) {
                int state_index = 0;
                for (int i = 0; i < num_qubits; i++) {
                    if (bitstring[i] == '1') {
                        state_index |= (1 << i);
                    }
                }
                return state_vector[state_index].magnitude() * 
                       state_vector[state_index].magnitude();
            } else {
                // 개별 큐비트 확률의 곱
                float prob = 1.0;
                for (int i = 0; i < num_qubits; i++) {
                    if (bitstring[i] == '0') {
                        prob *= qubits[i].probability_0();
                    } else {
                        prob *= qubits[i].probability_1();
                    }
                }
                return prob;
            }
        }
    };
    
public:
    void initialize() {
        Serial.begin(115200);
        randomSeed(analogRead(0));
        
        Serial.println("=== 양자 컴퓨팅 시뮬레이션 랩 ===");
        Serial.println("Arduino 기반 양자 컴퓨팅 교육 시스템");
        Serial.println("개발: 양자컴퓨팅 연구소");
        Serial.println();
        
        // 양자역학 기본 상수들
        Serial.println("양자역학 기본 원리:");
        Serial.println("- 플랑크 상수: 6.626 × 10⁻³⁴ J·s");
        Serial.println("- 중첩의 원리: |ψ⟩ = α|0⟩ + β|1⟩");
        Serial.println("- 확률 조건: |α|² + |β|² = 1");
        Serial.println("- 측정 후 상태 붕괴");
        Serial.println();
        
        Serial.println("시스템 초기화 완료");
        Serial.println("양자 실험을 시작합니다...");
    }
    
    void demonstrateBasicConcepts() {
        Serial.println("\n=== 양자 컴퓨팅 기본 개념 시연 ===");
        
        // 1. 큐비트 상태 시연
        demonstrateQubitStates();
        
        // 2. 중첩 상태 시연
        demonstrateSuperposition();
        
        // 3. 측정의 확률적 특성
        demonstrateProbabilisticMeasurement();
        
        // 4. 양자 간섭
        demonstrateQuantumInterference();
    }
    
private:
    void demonstrateQubitStates() {
        Serial.println("\n--- 큐비트 기본 상태 ---");
        
        Qubit qubit;
        
        // |0⟩ 상태
        Serial.println("초기 상태 |0⟩:");
        Serial.print("확률 P(0) = "); Serial.println(qubit.probability_0());
        Serial.print("확률 P(1) = "); Serial.println(qubit.probability_1());
        
        // |1⟩ 상태로 설정
        qubit.alpha = Complex(0, 0);
        qubit.beta = Complex(1, 0);
        Serial.println("\n|1⟩ 상태:");
        Serial.print("확률 P(0) = "); Serial.println(qubit.probability_0());
        Serial.print("확률 P(1) = "); Serial.println(qubit.probability_1());
        
        // 중첩 상태 |+⟩ = (|0⟩ + |1⟩)/√2
        qubit.alpha = Complex(1/sqrt(2), 0);
        qubit.beta = Complex(1/sqrt(2), 0);
        Serial.println("\n중첩 상태 |+⟩:");
        Serial.print("확률 P(0) = "); Serial.println(qubit.probability_0());
        Serial.print("확률 P(1) = "); Serial.println(qubit.probability_1());
    }
    
    void demonstrateSuperposition() {
        Serial.println("\n--- 중첩 상태 시연 ---");
        
        Qubit qubit;
        // 하다마드 게이트 적용: |0⟩ → (|0⟩ + |1⟩)/√2
        qubit.alpha = Complex(1/sqrt(2), 0);
        qubit.beta = Complex(1/sqrt(2), 0);
        
        Serial.println("하다마드 게이트 적용 후:");
        Serial.print("α = "); Serial.print(qubit.alpha.real); 
        Serial.print(" + "); Serial.print(qubit.alpha.imag); Serial.println("i");
        Serial.print("β = "); Serial.print(qubit.beta.real); 
        Serial.print(" + "); Serial.print(qubit.beta.imag); Serial.println("i");
        
        // 10번 측정하여 확률적 특성 확인
        Serial.println("\n10번 측정 결과:");
        for (int i = 0; i < 10; i++) {
            Qubit test_qubit = qubit; // 복사본으로 측정
            int result = test_qubit.measure();
            Serial.print(result); Serial.print(" ");
        }
        Serial.println();
    }
    
    void demonstrateProbabilisticMeasurement() {
        Serial.println("\n--- 확률적 측정 시연 ---");
        
        // 100번 측정하여 확률 분포 확인
        int count_0 = 0, count_1 = 0;
        
        for (int i = 0; i < 100; i++) {
            Qubit qubit;
            qubit.alpha = Complex(0.6, 0);  // 60% 확률로 |0⟩
            qubit.beta = Complex(0.8, 0);   // 80% 확률로 |1⟩
            qubit.normalize();
            
            int result = qubit.measure();
            if (result == 0) count_0++;
            else count_1++;
        }
        
        Serial.println("100번 측정 결과:");
        Serial.print("|0⟩ 측정: "); Serial.print(count_0); Serial.println("번");
        Serial.print("|1⟩ 측정: "); Serial.print(count_1); Serial.println("번");
        Serial.print("이론적 확률 P(0) = "); 
        
        Qubit test_qubit;
        test_qubit.alpha = Complex(0.6, 0);
        test_qubit.beta = Complex(0.8, 0);
        test_qubit.normalize();
        Serial.println(test_qubit.probability_0());
    }
    
    void demonstrateQuantumInterference() {
        Serial.println("\n--- 양자 간섭 시연 ---");
        
        // 간섭 상태: |ψ⟩ = (|0⟩ - |1⟩)/√2
        Qubit qubit;
        qubit.alpha = Complex(1/sqrt(2), 0);
        qubit.beta = Complex(-1/sqrt(2), 0);  // 음의 위상
        
        Serial.println("간섭 상태 |−⟩ = (|0⟩ - |1⟩)/√2");
        Serial.print("α = "); Serial.print(qubit.alpha.real);
        Serial.print(" + "); Serial.print(qubit.alpha.imag); Serial.println("i");
        Serial.print("β = "); Serial.print(qubit.beta.real);
        Serial.print(" + "); Serial.print(qubit.beta.imag); Serial.println("i");
        
        Serial.print("확률 P(0) = "); Serial.println(qubit.probability_0());
        Serial.print("확률 P(1) = "); Serial.println(qubit.probability_1());
        
        // 위상이 측정 확률에 미치는 영향
        Serial.println("위상은 측정 확률에 직접 영향을 주지 않지만,");
        Serial.println("다른 큐비트와의 상호작용에서 중요한 역할을 합니다.");
    }
};
```

---

## 큐비트 시뮬레이션

### 단일 큐비트 시뮬레이터
```cpp
class QubitSimulator {
private:
    struct QubitState {
        float alpha_real, alpha_imag;  // |0⟩ 진폭
        float beta_real, beta_imag;    // |1⟩ 진폭
        bool is_measured;
        int measurement_result;
        
        QubitState() {
            reset();
        }
        
        void reset() {
            alpha_real = 1.0; alpha_imag = 0.0;
            beta_real = 0.0; beta_imag = 0.0;
            is_measured = false;
            measurement_result = -1;
        }
        
        void normalize() {
            float norm_sq = alpha_real*alpha_real + alpha_imag*alpha_imag +
                           beta_real*beta_real + beta_imag*beta_imag;
            float norm = sqrt(norm_sq);
            
            if (norm > 0.001) {
                alpha_real /= norm; alpha_imag /= norm;
                beta_real /= norm; beta_imag /= norm;
            }
        }
        
        float probability_zero() {
            return alpha_real*alpha_real + alpha_imag*alpha_imag;
        }
        
        float probability_one() {
            return beta_real*beta_real + beta_imag*beta_imag;
        }
        
        int measure() {
            if (is_measured) return measurement_result;
            
            float prob_zero = probability_zero();
            float random_val = random(0, 10000) / 10000.0;
            
            if (random_val < prob_zero) {
                alpha_real = 1.0; alpha_imag = 0.0;
                beta_real = 0.0; beta_imag = 0.0;
                measurement_result = 0;
            } else {
                alpha_real = 0.0; alpha_imag = 0.0;
                beta_real = 1.0; beta_imag = 0.0;
                measurement_result = 1;
            }
            
            is_measured = true;
            return measurement_result;
        }
        
        void display() {
            Serial.print("α = "); Serial.print(alpha_real, 4);
            if (alpha_imag >= 0) Serial.print(" + ");
            else Serial.print(" - ");
            Serial.print(abs(alpha_imag), 4); Serial.println("i");
            
            Serial.print("β = "); Serial.print(beta_real, 4);
            if (beta_imag >= 0) Serial.print(" + ");
            else Serial.print(" - ");
            Serial.print(abs(beta_imag), 4); Serial.println("i");
            
            Serial.print("P(|0⟩) = "); Serial.println(probability_zero(), 4);
            Serial.print("P(|1⟩) = "); Serial.println(probability_one(), 4);
        }
    };
    
    QubitState qubit;
    uint32_t last_visualization;
    
public:
    void initialize() {
        Serial.begin(115200);
        
        Serial.println("=== 큐비트 시뮬레이터 ===");
        Serial.println("명령어:");
        Serial.println("  'r' - 초기화 (|0⟩ 상태)");
        Serial.println("  'h' - 하다마드 게이트");
        Serial.println("  'x' - 파울리 X 게이트");
        Serial.println("  'y' - 파울리 Y 게이트");
        Serial.println("  'z' - 파울리 Z 게이트");
        Serial.println("  's' - S 게이트 (위상)");
        Serial.println("  't' - T 게이트 (π/8 위상)");
        Serial.println("  'm' - 측정");
        Serial.println("  'd' - 상태 표시");
        Serial.println("  'v' - 시각화");
        Serial.println();
        
        last_visualization = 0;
        displayState();
    }
    
    void update() {
        // 실시간 시각화 (5초마다)
        if (millis() - last_visualization > 5000) {
            visualizeBlochSphere();
            last_visualization = millis();
        }
        
        // 명령어 처리
        if (Serial.available()) {
            char command = Serial.read();
            processCommand(command);
        }
    }
    
private:
    void processCommand(char cmd) {
        Serial.println("\n--- 명령어: " + String(cmd) + " ---");
        
        switch (cmd) {
            case 'r':
                reset();
                break;
            case 'h':
                applyHadamard();
                break;
            case 'x':
                applyPauliX();
                break;
            case 'y':
                applyPauliY();
                break;
            case 'z':
                applyPauliZ();
                break;
            case 's':
                applySGate();
                break;
            case 't':
                applyTGate();
                break;
            case 'm':
                measure();
                break;
            case 'd':
                displayState();
                break;
            case 'v':
                visualizeBlochSphere();
                break;
            default:
                Serial.println("알 수 없는 명령어입니다.");
                return;
        }
        
        displayState();
    }
    
    void reset() {
        Serial.println("큐비트 초기화 → |0⟩");
        qubit.reset();
    }
    
    void applyHadamard() {
        Serial.println("하다마드 게이트 적용");
        Serial.println("H = 1/√2 [1  1]");
        Serial.println("        [1 -1]");
        
        float new_alpha_real = (qubit.alpha_real + qubit.beta_real) / sqrt(2);
        float new_alpha_imag = (qubit.alpha_imag + qubit.beta_imag) / sqrt(2);
        float new_beta_real = (qubit.alpha_real - qubit.beta_real) / sqrt(2);
        float new_beta_imag = (qubit.alpha_imag - qubit.beta_imag) / sqrt(2);
        
        qubit.alpha_real = new_alpha_real; qubit.alpha_imag = new_alpha_imag;
        qubit.beta_real = new_beta_real; qubit.beta_imag = new_beta_imag;
        qubit.is_measured = false;
    }
    
    void applyPauliX() {
        Serial.println("파울리 X 게이트 적용 (비트 플립)");
        Serial.println("X = [0 1]");
        Serial.println("    [1 0]");
        
        float temp_real = qubit.alpha_real;
        float temp_imag = qubit.alpha_imag;
        
        qubit.alpha_real = qubit.beta_real;
        qubit.alpha_imag = qubit.beta_imag;
        qubit.beta_real = temp_real;
        qubit.beta_imag = temp_imag;
        qubit.is_measured = false;
    }
    
    void applyPauliY() {
        Serial.println("파울리 Y 게이트 적용");
        Serial.println("Y = [0 -i]");
        Serial.println("    [i  0]");
        
        float new_alpha_real = qubit.beta_imag;
        float new_alpha_imag = -qubit.beta_real;
        float new_beta_real = -qubit.alpha_imag;
        float new_beta_imag = qubit.alpha_real;
        
        qubit.alpha_real = new_alpha_real; qubit.alpha_imag = new_alpha_imag;
        qubit.beta_real = new_beta_real; qubit.beta_imag = new_beta_imag;
        qubit.is_measured = false;
    }
    
    void applyPauliZ() {
        Serial.println("파울리 Z 게이트 적용 (위상 플립)");
        Serial.println("Z = [1  0]");
        Serial.println("    [0 -1]");
        
        qubit.beta_real = -qubit.beta_real;
        qubit.beta_imag = -qubit.beta_imag;
        qubit.is_measured = false;
    }
    
    void applySGate() {
        Serial.println("S 게이트 적용 (π/2 위상)");
        Serial.println("S = [1 0]");
        Serial.println("    [0 i]");
        
        float new_beta_real = -qubit.beta_imag;
        float new_beta_imag = qubit.beta_real;
        
        qubit.beta_real = new_beta_real;
        qubit.beta_imag = new_beta_imag;
        qubit.is_measured = false;
    }
    
    void applyTGate() {
        Serial.println("T 게이트 적용 (π/4 위상)");
        Serial.println("T = [1 0    ]");
        Serial.println("    [0 e^iπ/4]");
        
        float cos_pi4 = 1/sqrt(2);
        float sin_pi4 = 1/sqrt(2);
        
        float new_beta_real = qubit.beta_real * cos_pi4 - qubit.beta_imag * sin_pi4;
        float new_beta_imag = qubit.beta_real * sin_pi4 + qubit.beta_imag * cos_pi4;
        
        qubit.beta_real = new_beta_real;
        qubit.beta_imag = new_beta_imag;
        qubit.is_measured = false;
    }
    
    void measure() {
        Serial.println("큐비트 측정");
        int result = qubit.measure();
        Serial.print("측정 결과: |"); Serial.print(result); Serial.println("⟩");
        Serial.println("상태가 측정값으로 붕괴되었습니다.");
    }
    
    void displayState() {
        Serial.println("\n현재 큐비트 상태:");
        Serial.print("|ψ⟩ = ");
        
        if (abs(qubit.alpha_real) > 0.001 || abs(qubit.alpha_imag) > 0.001) {
            Serial.print("("); Serial.print(qubit.alpha_real, 3);
            if (qubit.alpha_imag >= 0) Serial.print("+");
            Serial.print(qubit.alpha_imag, 3); Serial.print("i)|0⟩");
            
            if (abs(qubit.beta_real) > 0.001 || abs(qubit.beta_imag) > 0.001) {
                Serial.print(" + ");
            }
        }
        
        if (abs(qubit.beta_real) > 0.001 || abs(qubit.beta_imag) > 0.001) {
            Serial.print("("); Serial.print(qubit.beta_real, 3);
            if (qubit.beta_imag >= 0) Serial.print("+");
            Serial.print(qubit.beta_imag, 3); Serial.print("i)|1⟩");
        }
        
        Serial.println();
        Serial.print("확률: P(0) = "); Serial.print(qubit.probability_zero(), 3);
        Serial.print(", P(1) = "); Serial.println(qubit.probability_one(), 3);
        
        if (qubit.is_measured) {
            Serial.print("측정됨: |"); Serial.print(qubit.measurement_result); Serial.println("⟩");
        }
        Serial.println();
    }
    
    void visualizeBlochSphere() {
        Serial.println("\n=== 블로흐 구면 시각화 ===");
        
        // 블로흐 벡터 계산
        float x = 2 * (qubit.alpha_real * qubit.beta_real + qubit.alpha_imag * qubit.beta_imag);
        float y = 2 * (qubit.alpha_imag * qubit.beta_real - qubit.alpha_real * qubit.beta_imag);
        float z = qubit.alpha_real * qubit.alpha_real + qubit.alpha_imag * qubit.alpha_imag -
                  qubit.beta_real * qubit.beta_real - qubit.beta_imag * qubit.beta_imag;
        
        Serial.print("블로흐 벡터: ("); Serial.print(x, 3); Serial.print(", ");
        Serial.print(y, 3); Serial.print(", "); Serial.print(z, 3); Serial.println(")");
        
        // ASCII 아트로 블로흐 구면 표현
        Serial.println("\n    z(|0⟩)");
        Serial.println("     |");
        Serial.println("     |");
        Serial.println("     •────── y");
        Serial.println("    /");
        Serial.println("   /");
        Serial.println("  x");
        Serial.println("  |1⟩");
        
        // 극좌표 표현
        float theta = acos(z);  // 0 ≤ θ ≤ π
        float phi = atan2(y, x); // -π ≤ φ ≤ π
        
        Serial.print("극좌표: θ = "); Serial.print(theta * 180 / PI, 1); Serial.print("°, ");
        Serial.print("φ = "); Serial.print(phi * 180 / PI, 1); Serial.println("°");
        
        // 특별한 상태들 확인
        if (abs(z - 1) < 0.01) {
            Serial.println("→ |0⟩ 상태 (북극)");
        } else if (abs(z + 1) < 0.01) {
            Serial.println("→ |1⟩ 상태 (남극)");
        } else if (abs(z) < 0.01 && abs(x - 1) < 0.01) {
            Serial.println("→ |+⟩ 상태 (적도, x축)");
        } else if (abs(z) < 0.01 && abs(x + 1) < 0.01) {
            Serial.println("→ |−⟩ 상태 (적도, -x축)");
        } else if (abs(z) < 0.01 && abs(y - 1) < 0.01) {
            Serial.println("→ |+i⟩ 상태 (적도, y축)");
        } else if (abs(z) < 0.01 && abs(y + 1) < 0.01) {
            Serial.println("→ |−i⟩ 상태 (적도, -y축)");
        }
        
        Serial.println();
    }
};

QubitSimulator qubit_sim;

void setup() {
    qubit_sim.initialize();
}

void loop() {
    qubit_sim.update();
    delay(100);
}
```

---

## 양자 게이트 구현

### 다중 큐비트 게이트 시스템
```cpp
class QuantumGateSystem {
private:
    static const int MAX_QUBITS = 4;
    
    struct MultiQubitState {
        int num_qubits;
        float state_vector_real[16];  // 2^4 = 16 상태
        float state_vector_imag[16];
        bool is_measured[MAX_QUBITS];
        int measurement_results[MAX_QUBITS];
        
        MultiQubitState(int n = 2) : num_qubits(min(n, MAX_QUBITS)) {
            reset();
        }
        
        void reset() {
            int total_states = 1 << num_qubits;
            for (int i = 0; i < total_states; i++) {
                state_vector_real[i] = (i == 0) ? 1.0 : 0.0;
                state_vector_imag[i] = 0.0;
            }
            
            for (int i = 0; i < num_qubits; i++) {
                is_measured[i] = false;
                measurement_results[i] = -1;
            }
        }
        
        float getProbability(int state_index) {
            return state_vector_real[state_index] * state_vector_real[state_index] +
                   state_vector_imag[state_index] * state_vector_imag[state_index];
        }
        
        String measureAll() {
            String result = "";
            int total_states = 1 << num_qubits;
            
            // 확률 분포에 따라 상태 선택
            float cumulative_prob = 0;
            float random_val = random(0, 10000) / 10000.0;
            int measured_state = 0;
            
            for (int i = 0; i < total_states; i++) {
                cumulative_prob += getProbability(i);
                if (random_val <= cumulative_prob) {
                    measured_state = i;
                    break;
                }
            }
            
            // 상태를 비트스트링으로 변환
            for (int i = 0; i < num_qubits; i++) {
                int bit = (measured_state >> i) & 1;
                result = String(bit) + result;
                measurement_results[i] = bit;
                is_measured[i] = true;
            }
            
            // 상태 벡터를 측정된 상태로 붕괴
            for (int i = 0; i < total_states; i++) {
                if (i == measured_state) {
                    state_vector_real[i] = 1.0;
                    state_vector_imag[i] = 0.0;
                } else {
                    state_vector_real[i] = 0.0;
                    state_vector_imag[i] = 0.0;
                }
            }
            
            return result;
        }
        
        void displayState() {
            Serial.println("현재 양자 상태:");
            int total_states = 1 << num_qubits;
            
            for (int i = 0; i < total_states; i++) {
                float prob = getProbability(i);
                if (prob > 0.001) {  // 무시할 수 있는 확률은 표시하지 않음
                    Serial.print("|");
                    for (int j = num_qubits - 1; j >= 0; j--) {
                        Serial.print((i >> j) & 1);
                    }
                    Serial.print("⟩: ");
                    
                    Serial.print("("); Serial.print(state_vector_real[i], 3);
                    if (state_vector_imag[i] >= 0) Serial.print("+");
                    Serial.print(state_vector_imag[i], 3); Serial.print("i) ");
                    
                    Serial.print("P = "); Serial.println(prob, 3);
                }
            }
            Serial.println();
        }
    };
    
    MultiQubitState quantum_state;
    
public:
    QuantumGateSystem() : quantum_state(2) {}
    
    void initialize() {
        Serial.begin(115200);
        
        Serial.println("=== 양자 게이트 시스템 ===");
        Serial.println("2큐비트 양자 회로 시뮬레이터");
        Serial.println();
        Serial.println("사용 가능한 게이트:");
        Serial.println("  단일 큐비트 게이트:");
        Serial.println("    'h0', 'h1' - 하다마드 게이트");
        Serial.println("    'x0', 'x1' - 파울리 X 게이트");
        Serial.println("    'y0', 'y1' - 파울리 Y 게이트");
        Serial.println("    'z0', 'z1' - 파울리 Z 게이트");
        Serial.println("  2큐비트 게이트:");
        Serial.println("    'cnot'     - CNOT 게이트 (제어: 0, 타겟: 1)");
        Serial.println("    'cz'       - Controlled-Z 게이트");
        Serial.println("    'swap'     - SWAP 게이트");
        Serial.println("  측정 및 제어:");
        Serial.println("    'measure'  - 모든 큐비트 측정");
        Serial.println("    'reset'    - 상태 초기화");
        Serial.println("    'state'    - 현재 상태 표시");
        Serial.println();
        
        quantum_state.displayState();
    }
    
    void update() {
        if (Serial.available()) {
            String command = Serial.readString();
            command.trim();
            command.toLowerCase();
            processCommand(command);
        }
    }
    
private:
    void processCommand(String cmd) {
        Serial.println("명령어: " + cmd);
        Serial.println("------------------------");
        
        if (cmd == "h0") {
            applyHadamard(0);
        } else if (cmd == "h1") {
            applyHadamard(1);
        } else if (cmd == "x0") {
            applyPauliX(0);
        } else if (cmd == "x1") {
            applyPauliX(1);
        } else if (cmd == "y0") {
            applyPauliY(0);
        } else if (cmd == "y1") {
            applyPauliY(1);
        } else if (cmd == "z0") {
            applyPauliZ(0);
        } else if (cmd == "z1") {
            applyPauliZ(1);
        } else if (cmd == "cnot") {
            applyCNOT(0, 1);
        } else if (cmd == "cz") {
            applyControlledZ(0, 1);
        } else if (cmd == "swap") {
            applySWAP(0, 1);
        } else if (cmd == "measure") {
            measureSystem();
        } else if (cmd == "reset") {
            resetSystem();
        } else if (cmd == "state") {
            quantum_state.displayState();
            return;
        } else {
            Serial.println("알 수 없는 명령어입니다.");
            return;
        }
        
        quantum_state.displayState();
    }
    
    void applyHadamard(int qubit_index) {
        Serial.println("큐비트 " + String(qubit_index) + "에 하다마드 게이트 적용");
        
        int total_states = 1 << quantum_state.num_qubits;
        float new_real[16], new_imag[16];
        
        for (int i = 0; i < total_states; i++) {
            new_real[i] = 0;
            new_imag[i] = 0;
        }
        
        for (int i = 0; i < total_states; i++) {
            int bit = (i >> qubit_index) & 1;
            int flipped = i ^ (1 << qubit_index);
            
            float factor = 1.0 / sqrt(2);
            
            if (bit == 0) {
                // |0⟩ → (|0⟩ + |1⟩)/√2
                new_real[i] += factor * quantum_state.state_vector_real[i];
                new_imag[i] += factor * quantum_state.state_vector_imag[i];
                new_real[flipped] += factor * quantum_state.state_vector_real[i];
                new_imag[flipped] += factor * quantum_state.state_vector_imag[i];
            } else {
                // |1⟩ → (|0⟩ - |1⟩)/√2
                new_real[flipped] += factor * quantum_state.state_vector_real[i];
                new_imag[flipped] += factor * quantum_state.state_vector_imag[i];
                new_real[i] -= factor * quantum_state.state_vector_real[i];
                new_imag[i] -= factor * quantum_state.state_vector_imag[i];
            }
        }
        
        for (int i = 0; i < total_states; i++) {
            quantum_state.state_vector_real[i] = new_real[i];
            quantum_state.state_vector_imag[i] = new_imag[i];
        }
    }
    
    void applyPauliX(int qubit_index) {
        Serial.println("큐비트 " + String(qubit_index) + "에 파울리 X 게이트 적용");
        
        int total_states = 1 << quantum_state.num_qubits;
        float new_real[16], new_imag[16];
        
        for (int i = 0; i < total_states; i++) {
            int flipped = i ^ (1 << qubit_index);
            new_real[i] = quantum_state.state_vector_real[flipped];
            new_imag[i] = quantum_state.state_vector_imag[flipped];
        }
        
        for (int i = 0; i < total_states; i++) {
            quantum_state.state_vector_real[i] = new_real[i];
            quantum_state.state_vector_imag[i] = new_imag[i];
        }
    }
    
    void applyPauliY(int qubit_index) {
        Serial.println("큐비트 " + String(qubit_index) + "에 파울리 Y 게이트 적용");
        
        int total_states = 1 << quantum_state.num_qubits;
        float new_real[16], new_imag[16];
        
        for (int i = 0; i < total_states; i++) {
            int bit = (i >> qubit_index) & 1;
            int flipped = i ^ (1 << qubit_index);
            
            if (bit == 0) {
                // |0⟩ → i|1⟩
                new_real[i] = 0;
                new_imag[i] = 0;
                new_real[flipped] = -quantum_state.state_vector_imag[i];
                new_imag[flipped] = quantum_state.state_vector_real[i];
            } else {
                // |1⟩ → -i|0⟩
                new_real[flipped] = quantum_state.state_vector_imag[i];
                new_imag[flipped] = -quantum_state.state_vector_real[i];
                new_real[i] = 0;
                new_imag[i] = 0;
            }
        }
        
        for (int i = 0; i < total_states; i++) {
            quantum_state.state_vector_real[i] = new_real[i];
            quantum_state.state_vector_imag[i] = new_imag[i];
        }
    }
    
    void applyPauliZ(int qubit_index) {
        Serial.println("큐비트 " + String(qubit_index) + "에 파울리 Z 게이트 적용");
        
        int total_states = 1 << quantum_state.num_qubits;
        
        for (int i = 0; i < total_states; i++) {
            int bit = (i >> qubit_index) & 1;
            if (bit == 1) {
                quantum_state.state_vector_real[i] = -quantum_state.state_vector_real[i];
                quantum_state.state_vector_imag[i] = -quantum_state.state_vector_imag[i];
            }
        }
    }
    
    void applyCNOT(int control, int target) {
        Serial.println("CNOT 게이트 적용 (제어: " + String(control) + 
                      ", 타겟: " + String(target) + ")");
        
        int total_states = 1 << quantum_state.num_qubits;
        float new_real[16], new_imag[16];
        
        for (int i = 0; i < total_states; i++) {
            int control_bit = (i >> control) & 1;
            
            if (control_bit == 1) {
                // 제어 큐비트가 |1⟩이면 타겟 큐비트 플립
                int flipped = i ^ (1 << target);
                new_real[i] = quantum_state.state_vector_real[flipped];
                new_imag[i] = quantum_state.state_vector_imag[flipped];
            } else {
                // 제어 큐비트가 |0⟩이면 변화 없음
                new_real[i] = quantum_state.state_vector_real[i];
                new_imag[i] = quantum_state.state_vector_imag[i];
            }
        }
        
        for (int i = 0; i < total_states; i++) {
            quantum_state.state_vector_real[i] = new_real[i];
            quantum_state.state_vector_imag[i] = new_imag[i];
        }
    }
    
    void applyControlledZ(int control, int target) {
        Serial.println("Controlled-Z 게이트 적용 (제어: " + String(control) + 
                      ", 타겟: " + String(target) + ")");
        
        int total_states = 1 << quantum_state.num_qubits;
        
        for (int i = 0; i < total_states; i++) {
            int control_bit = (i >> control) & 1;
            int target_bit = (i >> target) & 1;
            
            if (control_bit == 1 && target_bit == 1) {
                quantum_state.state_vector_real[i] = -quantum_state.state_vector_real[i];
                quantum_state.state_vector_imag[i] = -quantum_state.state_vector_imag[i];
            }
        }
    }
    
    void applySWAP(int qubit1, int qubit2) {
        Serial.println("SWAP 게이트 적용 (큐비트 " + String(qubit1) + 
                      " ↔ " + String(qubit2) + ")");
        
        int total_states = 1 << quantum_state.num_qubits;
        float new_real[16], new_imag[16];
        
        for (int i = 0; i < total_states; i++) {
            int bit1 = (i >> qubit1) & 1;
            int bit2 = (i >> qubit2) & 1;
            
            if (bit1 != bit2) {
                // 두 비트가 다르면 서로 교환
                int swapped = i ^ (1 << qubit1) ^ (1 << qubit2);
                new_real[i] = quantum_state.state_vector_real[swapped];
                new_imag[i] = quantum_state.state_vector_imag[swapped];
            } else {
                // 두 비트가 같으면 변화 없음
                new_real[i] = quantum_state.state_vector_real[i];
                new_imag[i] = quantum_state.state_vector_imag[i];
            }
        }
        
        for (int i = 0; i < total_states; i++) {
            quantum_state.state_vector_real[i] = new_real[i];
            quantum_state.state_vector_imag[i] = new_imag[i];
        }
    }
    
    void measureSystem() {
        Serial.println("양자 시스템 측정");
        String result = quantum_state.measureAll();
        Serial.println("측정 결과: |" + result + "⟩");
        Serial.println("상태가 측정값으로 붕괴되었습니다.");
    }
    
    void resetSystem() {
        Serial.println("시스템 초기화 → |00⟩");
        quantum_state.reset();
    }
};

QuantumGateSystem gate_system;

void setup() {
    gate_system.initialize();
}

void loop() {
    gate_system.update();
    delay(100);
}
```

이어서 양자 컴퓨팅 시스템의 나머지 부분들을 작성하겠습니다.