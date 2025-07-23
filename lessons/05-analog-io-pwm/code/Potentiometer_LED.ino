/*
  Potentiometer_LED.ino
  가변저항으로 LED 밝기 조절하기
  
  회로:
  - A0: 가변저항 중앙 핀
  - 9번 핀: LED (220Ω 저항 직렬 연결)
  - 가변저항 양쪽: 5V와 GND에 연결
  
  작성일: 2024
  작성자: Arduino 임베디드 강의
*/

const int POT_PIN = A0;    // 가변저항 아날로그 핀
const int LED_PIN = 9;     // PWM 핀 (3, 5, 6, 9, 10, 11 중 하나)

void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("가변저항 LED 밝기 조절 시작!");
  Serial.println("가변저항을 돌려서 LED 밝기를 조절하세요.");
}

void loop() {
  // 가변저항 값 읽기 (0~1023)
  int potValue = analogRead(POT_PIN);
  
  // PWM 값으로 변환 (0~255)
  int ledBrightness = map(potValue, 0, 1023, 0, 255);
  
  // LED 밝기 설정
  analogWrite(LED_PIN, ledBrightness);
  
  // 전압 계산
  float voltage = (potValue / 1023.0) * 5.0;
  
  // 백분율 계산
  int percentage = map(potValue, 0, 1023, 0, 100);
  
  // 시리얼 모니터에 상태 출력
  Serial.print("가변저항: ");
  Serial.print(potValue);
  Serial.print(" (");
  Serial.print(voltage, 2);
  Serial.print("V) → LED 밝기: ");
  Serial.print(ledBrightness);
  Serial.print(" (");
  Serial.print(percentage);
  Serial.println("%)");
  
  delay(100);  // 부드러운 변화를 위한 짧은 지연
}