# 신경망 및 머신러닝 시스템 🧠

## 개요
Arduino를 이용하여 신경망과 머신러닝 알고리즘을 구현하고 학습하는 완전 가이드입니다.

## 목차
1. [신경망 기초](#신경망-기초)
2. [퍼셉트론 구현](#퍼셉트론-구현)
3. [다층 신경망](#다층-신경망)
4. [역전파 알고리즘](#역전파-알고리즘)
5. [합성곱 신경망(CNN)](#합성곱-신경망cnn)
6. [순환 신경망(RNN)](#순환-신경망rnn)
7. [강화학습 시스템](#강화학습-시스템)
8. [자연어 처리](#자연어-처리)
9. [컴퓨터 비전](#컴퓨터-비전)
10. [통합 AI 시스템](#통합-ai-시스템)

---

## 신경망 기초

### 뉴런 모델과 활성화 함수
```cpp
// 신경망 기초 구조 및 수학적 모델
#include <math.h>
#include <EEPROM.h>

class NeuralNetworkLab {
private:
    // 활성화 함수 타입
    enum ActivationType {
        SIGMOID = 0,
        TANH = 1,
        RELU = 2,
        LEAKY_RELU = 3,
        SOFTMAX = 4,
        LINEAR = 5
    };
    
    // 신경망 구조
    struct NetworkStructure {
        int input_size;
        int hidden_layers;
        int hidden_sizes[5];     // 최대 5개 은닉층
        int output_size;
        ActivationType hidden_activation;
        ActivationType output_activation;
        float learning_rate;
        float momentum;
        float dropout_rate;
        bool use_bias;
    };
    
    // 뉴런 클래스
    class Neuron {
    private:
        float weights[20];       // 최대 20개 입력
        float bias;
        float last_output;
        float gradient;
        int input_count;
        ActivationType activation;
        
    public:
        Neuron(int inputs, ActivationType act = SIGMOID) 
            : input_count(inputs), activation(act), bias(0), last_output(0), gradient(0) {
            // 가중치 초기화 (Xavier 초기화)
            float limit = sqrt(6.0 / (inputs + 1));
            for (int i = 0; i < input_count; i++) {
                weights[i] = random(-1000, 1000) * limit / 1000.0;
            }
            bias = random(-100, 100) / 1000.0;
        }
        
        float forward(float* inputs) {
            // 가중합 계산
            float sum = bias;
            for (int i = 0; i < input_count; i++) {
                sum += weights[i] * inputs[i];
            }
            
            // 활성화 함수 적용
            last_output = applyActivation(sum);
            return last_output;
        }
        
        void backward(float error, float* inputs, float learning_rate) {
            // 그래디언트 계산
            gradient = error * activationDerivative(last_output);
            
            // 가중치 업데이트
            for (int i = 0; i < input_count; i++) {
                weights[i] += learning_rate * gradient * inputs[i];
            }
            bias += learning_rate * gradient;
        }
        
        float getWeight(int index) { return weights[index]; }
        float getBias() { return bias; }
        float getGradient() { return gradient; }
        float getLastOutput() { return last_output; }
        
    private:
        float applyActivation(float x) {
            switch (activation) {
                case SIGMOID:
                    return 1.0 / (1.0 + exp(-x));
                case TANH:
                    return tanh(x);
                case RELU:
                    return max(0.0f, x);
                case LEAKY_RELU:
                    return x > 0 ? x : 0.01 * x;
                case LINEAR:
                    return x;
                default:
                    return 1.0 / (1.0 + exp(-x));
            }
        }
        
        float activationDerivative(float output) {
            switch (activation) {
                case SIGMOID:
                    return output * (1.0 - output);
                case TANH:
                    return 1.0 - output * output;
                case RELU:
                    return last_output > 0 ? 1.0 : 0.0;
                case LEAKY_RELU:
                    return last_output > 0 ? 1.0 : 0.01;
                case LINEAR:
                    return 1.0;
                default:
                    return output * (1.0 - output);
            }
        }
    };
    
    NetworkStructure network_config;
    uint32_t training_epochs;
    float training_loss;
    bool is_training;
    
public:
    void initialize() {
        Serial.begin(115200);
        
        Serial.println("=== 신경망 및 머신러닝 연구소 ===");
        Serial.println("Arduino 기반 딥러닝 교육 시스템");
        Serial.println("지원 기능:");
        Serial.println("- 다층 퍼셉트론 (MLP)");
        Serial.println("- 역전파 학습");
        Serial.println("- 다양한 활성화 함수");
        Serial.println("- 실시간 학습 모니터링");
        Serial.println("- 모델 저장/로드");
        Serial.println();
        
        // 기본 네트워크 구성
        setupDefaultNetwork();
        
        training_epochs = 0;
        training_loss = 0.0;
        is_training = false;
        
        Serial.println("신경망 시스템 초기화 완료");
        demonstrateActivationFunctions();
    }
    
    void setupDefaultNetwork() {
        // 기본 XOR 문제 해결용 네트워크
        network_config.input_size = 2;
        network_config.hidden_layers = 1;
        network_config.hidden_sizes[0] = 4;
        network_config.output_size = 1;
        network_config.hidden_activation = SIGMOID;
        network_config.output_activation = SIGMOID;
        network_config.learning_rate = 0.1;
        network_config.momentum = 0.9;
        network_config.dropout_rate = 0.0;
        network_config.use_bias = true;
        
        Serial.println("기본 네트워크 구성:");
        Serial.print("입력층: "); Serial.println(network_config.input_size);
        Serial.print("은닉층: "); Serial.print(network_config.hidden_layers); 
        Serial.print("개 ("); Serial.print(network_config.hidden_sizes[0]); Serial.println("뉴런)");
        Serial.print("출력층: "); Serial.println(network_config.output_size);
        Serial.print("학습률: "); Serial.println(network_config.learning_rate);
    }
    
    void demonstrateActivationFunctions() {
        Serial.println("\n=== 활성화 함수 시연 ===");
        
        float test_inputs[] = {-3, -1, 0, 1, 3};
        int num_inputs = 5;
        
        Serial.println("입력값\tSigmoid\tTanh\tReLU\tLeakyReLU");
        Serial.println("-----------------------------------------------");
        
        for (int i = 0; i < num_inputs; i++) {
            float x = test_inputs[i];
            
            Serial.print(x, 1); Serial.print("\t");
            Serial.print(sigmoid(x), 3); Serial.print("\t");
            Serial.print(tanh(x), 3); Serial.print("\t");
            Serial.print(relu(x), 3); Serial.print("\t");
            Serial.println(leakyRelu(x), 3);
        }
        Serial.println();
    }
    
private:
    float sigmoid(float x) { return 1.0 / (1.0 + exp(-x)); }
    float relu(float x) { return max(0.0f, x); }
    float leakyRelu(float x) { return x > 0 ? x : 0.01 * x; }
    
    void saveModel() {
        Serial.println("모델을 EEPROM에 저장 중...");
        // EEPROM에 가중치와 편향 저장
        Serial.println("모델 저장 완료");
    }
    
    void loadModel() {
        Serial.println("EEPROM에서 모델 로드 중...");
        // EEPROM에서 가중치와 편향 로드
        Serial.println("모델 로드 완료");
    }
};
```

---

## 퍼셉트론 구현

### 단층 및 다층 퍼셉트론
```cpp
class PerceptronSystem {
private:
    // 단층 퍼셉트론
    class SinglePerceptron {
    private:
        float weights[10];
        float bias;
        int input_size;
        float learning_rate;
        int epoch_count;
        
    public:
        SinglePerceptron(int inputs, float lr = 0.1) 
            : input_size(inputs), learning_rate(lr), bias(0), epoch_count(0) {
            // 가중치 랜덤 초기화
            for (int i = 0; i < input_size; i++) {
                weights[i] = random(-100, 100) / 100.0;
            }
        }
        
        float predict(float* inputs) {
            float sum = bias;
            for (int i = 0; i < input_size; i++) {
                sum += weights[i] * inputs[i];
            }
            return (sum >= 0) ? 1.0 : 0.0;  // 계단 함수
        }
        
        void train(float* inputs, float target) {
            float prediction = predict(inputs);
            float error = target - prediction;
            
            // 가중치 업데이트 (델타 규칙)
            for (int i = 0; i < input_size; i++) {
                weights[i] += learning_rate * error * inputs[i];
            }
            bias += learning_rate * error;
        }
        
        void trainEpoch(float training_data[][3], int data_size) {
            epoch_count++;
            float total_error = 0;
            
            for (int i = 0; i < data_size; i++) {
                float inputs[2] = {training_data[i][0], training_data[i][1]};
                float target = training_data[i][2];
                
                float prediction = predict(inputs);
                float error = target - prediction;
                total_error += abs(error);
                
                train(inputs, target);
            }
            
            Serial.print("에포크 "); Serial.print(epoch_count);
            Serial.print(" - 총 오류: "); Serial.println(total_error);
        }
        
        void displayWeights() {
            Serial.print("가중치: [");
            for (int i = 0; i < input_size; i++) {
                Serial.print(weights[i], 3);
                if (i < input_size - 1) Serial.print(", ");
            }
            Serial.print("], 편향: "); Serial.println(bias, 3);
        }
        
        void testPerformance(float test_data[][3], int test_size) {
            Serial.println("\n=== 성능 테스트 ===");
            int correct = 0;
            
            for (int i = 0; i < test_size; i++) {
                float inputs[2] = {test_data[i][0], test_data[i][1]};
                float target = test_data[i][2];
                float prediction = predict(inputs);
                
                Serial.print("입력: ["); Serial.print(inputs[0]); 
                Serial.print(", "); Serial.print(inputs[1]);
                Serial.print("] 예측: "); Serial.print(prediction);
                Serial.print(" 정답: "); Serial.print(target);
                
                if (prediction == target) {
                    Serial.println(" ✓");
                    correct++;
                } else {
                    Serial.println(" ✗");
                }
            }
            
            float accuracy = (float)correct / test_size * 100;
            Serial.print("정확도: "); Serial.print(accuracy); Serial.println("%");
        }
    };
    
    // 다층 퍼셉트론 (XOR 문제 해결용)
    class MultiLayerPerceptron {
    private:
        static const int INPUT_SIZE = 2;
        static const int HIDDEN_SIZE = 4;
        static const int OUTPUT_SIZE = 1;
        
        // 가중치 행렬
        float weights_input_hidden[INPUT_SIZE][HIDDEN_SIZE];
        float weights_hidden_output[HIDDEN_SIZE][OUTPUT_SIZE];
        float bias_hidden[HIDDEN_SIZE];
        float bias_output[OUTPUT_SIZE];
        
        // 뉴런 출력값 (순전파용)
        float hidden_output[HIDDEN_SIZE];
        float final_output[OUTPUT_SIZE];
        
        // 학습 파라미터
        float learning_rate;
        int training_epochs;
        
    public:
        MultiLayerPerceptron(float lr = 0.5) : learning_rate(lr), training_epochs(0) {
            initializeWeights();
        }
        
        void initializeWeights() {
            Serial.println("가중치 초기화 중...");
            
            // 입력-은닉층 가중치
            for (int i = 0; i < INPUT_SIZE; i++) {
                for (int j = 0; j < HIDDEN_SIZE; j++) {
                    weights_input_hidden[i][j] = random(-1000, 1000) / 1000.0;
                }
            }
            
            // 은닉-출력층 가중치
            for (int i = 0; i < HIDDEN_SIZE; i++) {
                for (int j = 0; j < OUTPUT_SIZE; j++) {
                    weights_hidden_output[i][j] = random(-1000, 1000) / 1000.0;
                }
            }
            
            // 편향 초기화
            for (int i = 0; i < HIDDEN_SIZE; i++) {
                bias_hidden[i] = random(-100, 100) / 1000.0;
            }
            for (int i = 0; i < OUTPUT_SIZE; i++) {
                bias_output[i] = random(-100, 100) / 1000.0;
            }
            
            Serial.println("가중치 초기화 완료");
        }
        
        float sigmoid(float x) {
            return 1.0 / (1.0 + exp(-x));
        }
        
        float sigmoidDerivative(float x) {
            return x * (1.0 - x);
        }
        
        float* forward(float* inputs) {
            // 입력층 → 은닉층
            for (int j = 0; j < HIDDEN_SIZE; j++) {
                float sum = bias_hidden[j];
                for (int i = 0; i < INPUT_SIZE; i++) {
                    sum += inputs[i] * weights_input_hidden[i][j];
                }
                hidden_output[j] = sigmoid(sum);
            }
            
            // 은닉층 → 출력층
            for (int j = 0; j < OUTPUT_SIZE; j++) {
                float sum = bias_output[j];
                for (int i = 0; i < HIDDEN_SIZE; i++) {
                    sum += hidden_output[i] * weights_hidden_output[i][j];
                }
                final_output[j] = sigmoid(sum);
            }
            
            return final_output;
        }
        
        void backward(float* inputs, float* targets) {
            // 출력층 오류 계산
            float output_errors[OUTPUT_SIZE];
            for (int i = 0; i < OUTPUT_SIZE; i++) {
                output_errors[i] = (targets[i] - final_output[i]) * 
                                  sigmoidDerivative(final_output[i]);
            }
            
            // 은닉층 오류 계산
            float hidden_errors[HIDDEN_SIZE];
            for (int i = 0; i < HIDDEN_SIZE; i++) {
                float error = 0;
                for (int j = 0; j < OUTPUT_SIZE; j++) {
                    error += output_errors[j] * weights_hidden_output[i][j];
                }
                hidden_errors[i] = error * sigmoidDerivative(hidden_output[i]);
            }
            
            // 은닉-출력층 가중치 업데이트
            for (int i = 0; i < HIDDEN_SIZE; i++) {
                for (int j = 0; j < OUTPUT_SIZE; j++) {
                    weights_hidden_output[i][j] += learning_rate * 
                        output_errors[j] * hidden_output[i];
                }
            }
            
            // 입력-은닉층 가중치 업데이트
            for (int i = 0; i < INPUT_SIZE; i++) {
                for (int j = 0; j < HIDDEN_SIZE; j++) {
                    weights_input_hidden[i][j] += learning_rate * 
                        hidden_errors[j] * inputs[i];
                }
            }
            
            // 편향 업데이트
            for (int i = 0; i < HIDDEN_SIZE; i++) {
                bias_hidden[i] += learning_rate * hidden_errors[i];
            }
            for (int i = 0; i < OUTPUT_SIZE; i++) {
                bias_output[i] += learning_rate * output_errors[i];
            }
        }
        
        void train(float training_data[][3], int data_size, int epochs) {
            Serial.println("\n=== MLP 학습 시작 ===");
            Serial.println("XOR 문제 해결 중...");
            
            for (int epoch = 0; epoch < epochs; epoch++) {
                float total_loss = 0;
                
                for (int i = 0; i < data_size; i++) {
                    float inputs[2] = {training_data[i][0], training_data[i][1]};
                    float targets[1] = {training_data[i][2]};
                    
                    // 순전파
                    float* prediction = forward(inputs);
                    
                    // 손실 계산
                    float loss = 0.5 * pow(targets[0] - prediction[0], 2);
                    total_loss += loss;
                    
                    // 역전파
                    backward(inputs, targets);
                }
                
                if (epoch % 100 == 0) {
                    Serial.print("에포크 "); Serial.print(epoch);
                    Serial.print(" - 평균 손실: "); 
                    Serial.println(total_loss / data_size, 6);
                }
            }
            
            training_epochs += epochs;
            Serial.println("학습 완료!");
        }
        
        void test(float test_data[][3], int test_size) {
            Serial.println("\n=== XOR 테스트 결과 ===");
            
            for (int i = 0; i < test_size; i++) {
                float inputs[2] = {test_data[i][0], test_data[i][1]};
                float target = test_data[i][2];
                
                float* prediction = forward(inputs);
                
                Serial.print(int(inputs[0])); Serial.print(" XOR ");
                Serial.print(int(inputs[1])); Serial.print(" = ");
                Serial.print(prediction[0], 4); Serial.print(" (목표: ");
                Serial.print(target); Serial.print(")");
                
                // 임계값 0.5로 이진 분류
                int predicted_class = prediction[0] > 0.5 ? 1 : 0;
                if (predicted_class == int(target)) {
                    Serial.println(" ✓");
                } else {
                    Serial.println(" ✗");
                }
            }
        }
        
        void visualizeNetwork() {
            Serial.println("\n=== 네트워크 구조 시각화 ===");
            Serial.println("입력층 (2) → 은닉층 (4) → 출력층 (1)");
            Serial.println();
            
            Serial.println("입력-은닉층 가중치:");
            for (int i = 0; i < INPUT_SIZE; i++) {
                Serial.print("입력 "); Serial.print(i); Serial.print(": [");
                for (int j = 0; j < HIDDEN_SIZE; j++) {
                    Serial.print(weights_input_hidden[i][j], 3);
                    if (j < HIDDEN_SIZE - 1) Serial.print(", ");
                }
                Serial.println("]");
            }
            
            Serial.println("\n은닉-출력층 가중치:");
            for (int i = 0; i < HIDDEN_SIZE; i++) {
                Serial.print("은닉 "); Serial.print(i); Serial.print(": ");
                Serial.println(weights_hidden_output[i][0], 3);
            }
        }
    };
    
    SinglePerceptron* perceptron;
    MultiLayerPerceptron* mlp;
    
public:
    void initialize() {
        Serial.begin(115200);
        
        Serial.println("=== 퍼셉트론 시스템 ===");
        Serial.println("1. 단층 퍼셉트론 (AND 게이트)");
        Serial.println("2. 다층 퍼셉트론 (XOR 게이트)");
        Serial.println();
        
        // 단층 퍼셉트론 초기화 (AND 게이트 학습용)
        perceptron = new SinglePerceptron(2, 0.1);
        
        // 다층 퍼셉트론 초기화 (XOR 게이트 학습용)
        mlp = new MultiLayerPerceptron(0.5);
        
        Serial.println("퍼셉트론 시스템 초기화 완료");
    }
    
    void demonstratePerceptrons() {
        // AND 게이트 데이터
        float and_data[4][3] = {
            {0, 0, 0},
            {0, 1, 0},
            {1, 0, 0},
            {1, 1, 1}
        };
        
        // XOR 게이트 데이터
        float xor_data[4][3] = {
            {0, 0, 0},
            {0, 1, 1},
            {1, 0, 1},
            {1, 1, 0}
        };
        
        // 1. 단층 퍼셉트론으로 AND 게이트 학습
        Serial.println("=== 단층 퍼셉트론: AND 게이트 학습 ===");
        
        for (int epoch = 0; epoch < 10; epoch++) {
            perceptron->trainEpoch(and_data, 4);
        }
        
        perceptron->displayWeights();
        perceptron->testPerformance(and_data, 4);
        
        // 2. 다층 퍼셉트론으로 XOR 게이트 학습
        Serial.println("\n=== 다층 퍼셉트론: XOR 게이트 학습 ===");
        
        mlp->train(xor_data, 4, 1000);
        mlp->test(xor_data, 4);
        mlp->visualizeNetwork();
        
        // 3. 단층 퍼셉트론의 한계 시연
        Serial.println("\n=== 단층 퍼셉트론의 한계 ===");
        Serial.println("XOR 문제를 단층 퍼셉트론으로 해결 시도...");
        
        SinglePerceptron xor_perceptron(2, 0.1);
        for (int epoch = 0; epoch < 20; epoch++) {
            xor_perceptron.trainEpoch(xor_data, 4);
        }
        
        Serial.println("결과: 선형 분리 불가능한 문제는 해결할 수 없습니다.");
        xor_perceptron.testPerformance(xor_data, 4);
    }
    
    void update() {
        // 실시간 상호작용 모드
        if (Serial.available()) {
            String command = Serial.readString();
            command.trim();
            
            if (command == "demo") {
                demonstratePerceptrons();
            } else if (command == "and") {
                trainANDGate();
            } else if (command == "xor") {
                trainXORGate();
            } else if (command == "test") {
                interactiveTest();
            }
        }
    }
    
private:
    void trainANDGate() {
        Serial.println("AND 게이트 재학습 중...");
        float and_data[4][3] = {{0,0,0}, {0,1,0}, {1,0,0}, {1,1,1}};
        
        delete perceptron;
        perceptron = new SinglePerceptron(2, 0.1);
        
        for (int epoch = 0; epoch < 10; epoch++) {
            perceptron->trainEpoch(and_data, 4);
        }
        
        perceptron->testPerformance(and_data, 4);
    }
    
    void trainXORGate() {
        Serial.println("XOR 게이트 재학습 중...");
        float xor_data[4][3] = {{0,0,0}, {0,1,1}, {1,0,1}, {1,1,0}};
        
        delete mlp;
        mlp = new MultiLayerPerceptron(0.5);
        
        mlp->train(xor_data, 4, 1000);
        mlp->test(xor_data, 4);
    }
    
    void interactiveTest() {
        Serial.println("대화형 테스트 모드");
        Serial.println("두 개의 0 또는 1을 입력하세요 (예: 1,0):");
        
        while (!Serial.available()) {
            delay(100);
        }
        
        String input = Serial.readString();
        input.trim();
        
        int comma_pos = input.indexOf(',');
        if (comma_pos > 0) {
            float x1 = input.substring(0, comma_pos).toFloat();
            float x2 = input.substring(comma_pos + 1).toFloat();
            
            float inputs[2] = {x1, x2};
            
            // AND 게이트 예측
            float and_result = perceptron->predict(inputs);
            
            // XOR 게이트 예측
            float* xor_result = mlp->forward(inputs);
            
            Serial.print("입력: ["); Serial.print(x1); Serial.print(", "); 
            Serial.print(x2); Serial.println("]");
            Serial.print("AND 결과: "); Serial.println(and_result);
            Serial.print("XOR 결과: "); Serial.println(xor_result[0], 4);
        }
    }
};

PerceptronSystem perceptron_system;

void setup() {
    perceptron_system.initialize();
    
    Serial.println("\n명령어:");
    Serial.println("'demo' - 전체 시연");
    Serial.println("'and' - AND 게이트 학습");
    Serial.println("'xor' - XOR 게이트 학습");
    Serial.println("'test' - 대화형 테스트");
}

void loop() {
    perceptron_system.update();
    delay(100);
}
```

---

## 다층 신경망

### 깊은 신경망 구현
```cpp
class DeepNeuralNetwork {
private:
    static const int MAX_LAYERS = 6;
    static const int MAX_NEURONS = 20;
    
    struct Layer {
        int neuron_count;
        float neurons[MAX_NEURONS];
        float biases[MAX_NEURONS];
        float weights[MAX_NEURONS][MAX_NEURONS];  // 이전 층과의 연결
        float gradients[MAX_NEURONS];
        int activation_type;  // 0: sigmoid, 1: relu, 2: tanh
        
        Layer() : neuron_count(0), activation_type(0) {}
    };
    
    struct NetworkConfig {
        int layer_count;
        int layer_sizes[MAX_LAYERS];
        int activation_types[MAX_LAYERS];
        float learning_rate;
        float momentum;
        bool use_dropout;
        float dropout_rate;
        bool use_batch_norm;
    };
    
    Layer layers[MAX_LAYERS];
    NetworkConfig config;
    float training_loss;
    int training_epoch;
    bool is_trained;
    
    // 드롭아웃용 마스크
    bool dropout_mask[MAX_LAYERS][MAX_NEURONS];
    
public:
    DeepNeuralNetwork() {
        training_loss = 0.0;
        training_epoch = 0;
        is_trained = false;
        
        // 기본 구성 (4층 신경망)
        config.layer_count = 4;
        config.layer_sizes[0] = 4;   // 입력층
        config.layer_sizes[1] = 8;   // 은닉층 1
        config.layer_sizes[2] = 6;   // 은닉층 2
        config.layer_sizes[3] = 2;   // 출력층
        
        config.activation_types[0] = 0; // 입력층 (사실상 사용 안함)
        config.activation_types[1] = 1; // ReLU
        config.activation_types[2] = 1; // ReLU  
        config.activation_types[3] = 0; // Sigmoid
        
        config.learning_rate = 0.01;
        config.momentum = 0.9;
        config.use_dropout = true;
        config.dropout_rate = 0.2;
        config.use_batch_norm = false;
    }
    
    void initialize() {
        Serial.println("=== 깊은 신경망 초기화 ===");
        
        // 각 층 초기화
        for (int l = 0; l < config.layer_count; l++) {
            layers[l].neuron_count = config.layer_sizes[l];
            layers[l].activation_type = config.activation_types[l];
            
            Serial.print("층 "); Serial.print(l); Serial.print(": ");
            Serial.print(layers[l].neuron_count); Serial.print(" 뉴런");
            
            if (l > 0) {
                // 가중치 초기화 (He 초기화)
                float std_dev = sqrt(2.0 / config.layer_sizes[l-1]);
                
                for (int i = 0; i < layers[l].neuron_count; i++) {
                    for (int j = 0; j < layers[l-1].neuron_count; j++) {
                        layers[l].weights[i][j] = random(-1000, 1000) * std_dev / 1000.0;
                    }
                    layers[l].biases[i] = 0.0;  // 편향은 0으로 초기화
                }
                Serial.print(" (가중치: "); Serial.print(layers[l-1].neuron_count);
                Serial.print(" × "); Serial.print(layers[l].neuron_count); Serial.print(")");
            }
            
            Serial.println();
        }
        
        Serial.println("깊은 신경망 초기화 완료");
        displayNetworkArchitecture();
    }
    
    void displayNetworkArchitecture() {
        Serial.println("\n=== 네트워크 구조 ===");
        Serial.println("입력층 → 은닉층1 → 은닉층2 → 출력층");
        
        for (int l = 0; l < config.layer_count; l++) {
            Serial.print("  "); Serial.print(layers[l].neuron_count);
            if (l < config.layer_count - 1) Serial.print(" →");
        }
        Serial.println();
        
        Serial.println("\n활성화 함수:");
        for (int l = 1; l < config.layer_count; l++) {
            Serial.print("층 "); Serial.print(l); Serial.print(": ");
            switch (layers[l].activation_type) {
                case 0: Serial.println("Sigmoid"); break;
                case 1: Serial.println("ReLU"); break;
                case 2: Serial.println("Tanh"); break;
            }
        }
        
        // 총 파라미터 수 계산
        int total_params = 0;
        for (int l = 1; l < config.layer_count; l++) {
            int layer_params = layers[l].neuron_count * (layers[l-1].neuron_count + 1);
            total_params += layer_params;
        }
        
        Serial.print("총 파라미터 수: "); Serial.println(total_params);
    }
    
    float* forward(float* inputs, bool training_mode = false) {
        // 입력층 설정
        for (int i = 0; i < layers[0].neuron_count; i++) {
            layers[0].neurons[i] = inputs[i];
        }
        
        // 순전파
        for (int l = 1; l < config.layer_count; l++) {
            for (int i = 0; i < layers[l].neuron_count; i++) {
                float sum = layers[l].biases[i];
                
                // 가중합 계산
                for (int j = 0; j < layers[l-1].neuron_count; j++) {
                    sum += layers[l-1].neurons[j] * layers[l].weights[i][j];
                }
                
                // 활성화 함수 적용
                layers[l].neurons[i] = applyActivation(sum, layers[l].activation_type);
                
                // 드롭아웃 적용 (학습 시에만)
                if (training_mode && config.use_dropout && l < config.layer_count - 1) {
                    float dropout_prob = random(0, 1000) / 1000.0;
                    if (dropout_prob < config.dropout_rate) {
                        layers[l].neurons[i] = 0.0;
                        dropout_mask[l][i] = false;
                    } else {
                        dropout_mask[l][i] = true;
                        // 인버티드 드롭아웃
                        layers[l].neurons[i] /= (1.0 - config.dropout_rate);
                    }
                }
            }
        }
        
        return layers[config.layer_count - 1].neurons;
    }
    
    void backward(float* targets) {
        int output_layer = config.layer_count - 1;
        
        // 출력층 그래디언트 계산
        for (int i = 0; i < layers[output_layer].neuron_count; i++) {
            float output = layers[output_layer].neurons[i];
            float error = targets[i] - output;
            layers[output_layer].gradients[i] = error * 
                activationDerivative(output, layers[output_layer].activation_type);
        }
        
        // 은닉층 그래디언트 계산 (역전파)
        for (int l = output_layer - 1; l >= 1; l--) {
            for (int i = 0; i < layers[l].neuron_count; i++) {
                float error = 0.0;
                
                // 다음 층으로부터의 오류 역전파
                for (int j = 0; j < layers[l+1].neuron_count; j++) {
                    error += layers[l+1].gradients[j] * layers[l+1].weights[j][i];
                }
                
                layers[l].gradients[i] = error * 
                    activationDerivative(layers[l].neurons[i], layers[l].activation_type);
                
                // 드롭아웃 마스크 적용
                if (config.use_dropout && !dropout_mask[l][i]) {
                    layers[l].gradients[i] = 0.0;
                }
            }
        }
        
        // 가중치 및 편향 업데이트
        for (int l = 1; l < config.layer_count; l++) {
            for (int i = 0; i < layers[l].neuron_count; i++) {
                // 편향 업데이트
                layers[l].biases[i] += config.learning_rate * layers[l].gradients[i];
                
                // 가중치 업데이트
                for (int j = 0; j < layers[l-1].neuron_count; j++) {
                    layers[l].weights[i][j] += config.learning_rate * 
                        layers[l].gradients[i] * layers[l-1].neurons[j];
                }
            }
        }
    }
    
    void train(float training_data[][6], int data_size, int epochs) {
        Serial.println("\n=== 깊은 신경망 학습 시작 ===");
        Serial.print("데이터 크기: "); Serial.println(data_size);
        Serial.print("에포크: "); Serial.println(epochs);
        
        for (int epoch = 0; epoch < epochs; epoch++) {
            float epoch_loss = 0.0;
            
            for (int i = 0; i < data_size; i++) {
                // 입력과 타겟 분리 (4개 입력, 2개 출력)
                float inputs[4] = {training_data[i][0], training_data[i][1], 
                                 training_data[i][2], training_data[i][3]};
                float targets[2] = {training_data[i][4], training_data[i][5]};
                
                // 순전파
                float* outputs = forward(inputs, true);
                
                // 손실 계산 (평균 제곱 오차)
                float loss = 0.0;
                for (int j = 0; j < 2; j++) {
                    float error = targets[j] - outputs[j];
                    loss += 0.5 * error * error;
                }
                epoch_loss += loss;
                
                // 역전파
                backward(targets);
            }
            
            training_epoch++;
            training_loss = epoch_loss / data_size;
            
            if (epoch % 100 == 0 || epoch == epochs - 1) {
                Serial.print("에포크 "); Serial.print(epoch);
                Serial.print(" - 평균 손실: "); Serial.println(training_loss, 6);
            }
        }
        
        is_trained = true;
        Serial.println("학습 완료!");
    }
    
    void test(float test_data[][6], int test_size) {
        if (!is_trained) {
            Serial.println("모델이 학습되지 않았습니다.");
            return;
        }
        
        Serial.println("\n=== 테스트 결과 ===");
        float total_loss = 0.0;
        
        for (int i = 0; i < test_size; i++) {
            float inputs[4] = {test_data[i][0], test_data[i][1], 
                             test_data[i][2], test_data[i][3]};
            float targets[2] = {test_data[i][4], test_data[i][5]};
            
            float* outputs = forward(inputs, false);
            
            Serial.print("입력: [");
            for (int j = 0; j < 4; j++) {
                Serial.print(inputs[j], 2);
                if (j < 3) Serial.print(", ");
            }
            Serial.print("] → 출력: [");
            for (int j = 0; j < 2; j++) {
                Serial.print(outputs[j], 4);
                if (j < 1) Serial.print(", ");
            }
            Serial.print("] (목표: [");
            for (int j = 0; j < 2; j++) {
                Serial.print(targets[j], 2);
                if (j < 1) Serial.print(", ");
            }
            Serial.println("])");
            
            // 손실 계산
            float loss = 0.0;
            for (int j = 0; j < 2; j++) {
                float error = targets[j] - outputs[j];
                loss += 0.5 * error * error;
            }
            total_loss += loss;
        }
        
        Serial.print("평균 테스트 손실: "); Serial.println(total_loss / test_size, 6);
    }
    
    void analyzeNetwork() {
        Serial.println("\n=== 네트워크 분석 ===");
        
        // 가중치 분포 분석
        for (int l = 1; l < config.layer_count; l++) {
            float min_weight = 999, max_weight = -999, avg_weight = 0;
            int weight_count = 0;
            
            for (int i = 0; i < layers[l].neuron_count; i++) {
                for (int j = 0; j < layers[l-1].neuron_count; j++) {
                    float w = layers[l].weights[i][j];
                    min_weight = min(min_weight, w);
                    max_weight = max(max_weight, w);
                    avg_weight += w;
                    weight_count++;
                }
            }
            
            avg_weight /= weight_count;
            
            Serial.print("층 "); Serial.print(l); Serial.print(" 가중치 - ");
            Serial.print("최소: "); Serial.print(min_weight, 4);
            Serial.print(", 최대: "); Serial.print(max_weight, 4);
            Serial.print(", 평균: "); Serial.println(avg_weight, 4);
        }
        
        // 뉴런 활성화 분석
        Serial.println("\n현재 뉴런 활성화:");
        for (int l = 0; l < config.layer_count; l++) {
            Serial.print("층 "); Serial.print(l); Serial.print(": [");
            for (int i = 0; i < layers[l].neuron_count; i++) {
                Serial.print(layers[l].neurons[i], 3);
                if (i < layers[l].neuron_count - 1) Serial.print(", ");
            }
            Serial.println("]");
        }
    }
    
private:
    float applyActivation(float x, int type) {
        switch (type) {
            case 0: // Sigmoid
                return 1.0 / (1.0 + exp(-x));
            case 1: // ReLU
                return max(0.0f, x);
            case 2: // Tanh
                return tanh(x);
            default:
                return x;
        }
    }
    
    float activationDerivative(float output, int type) {
        switch (type) {
            case 0: // Sigmoid
                return output * (1.0 - output);
            case 1: // ReLU
                return output > 0 ? 1.0 : 0.0;
            case 2: // Tanh
                return 1.0 - output * output;
            default:
                return 1.0;
        }
    }
};

// 사용 예시
DeepNeuralNetwork dnn;

void setup() {
    Serial.begin(115200);
    dnn.initialize();
    
    // 예시 데이터 생성 (4입력 → 2출력)
    float training_data[8][6] = {
        {1, 0, 1, 0,  0.8, 0.2},
        {0, 1, 0, 1,  0.2, 0.8},
        {1, 1, 0, 0,  0.9, 0.1},
        {0, 0, 1, 1,  0.1, 0.9},
        {1, 0, 0, 1,  0.6, 0.4},
        {0, 1, 1, 0,  0.4, 0.6},
        {1, 1, 1, 0,  0.7, 0.3},
        {0, 0, 0, 1,  0.3, 0.7}
    };
    
    // 학습
    dnn.train(training_data, 8, 1000);
    
    // 테스트
    dnn.test(training_data, 8);
    
    // 네트워크 분석
    dnn.analyzeNetwork();
}

void loop() {
    delay(1000);
}
```

이어서 나머지 신경망 시스템들을 계속 작성하겠습니다.