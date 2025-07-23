/*
  Button_Basic.ino
  기본 버튼 입력으로 LED 제어하기
  
  회로:
  - 2번 핀: 푸시 버튼 (내부 풀업 저항 사용)
  - 13번 핀: LED (내장 LED 사용 가능)
  
  작성일: 2024
  작성자: Arduino 임베디드 강의
*/

const int BUTTON_PIN = 2;
const int LED_PIN = 13;

void setup() {
  // 버튼 핀을 내부 풀업 저항과 함께 입력으로 설정  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // LED 핀을 출력으로 설정
  pinMode(LED_PIN, OUTPUT);
  
  // 시리얼 통신 시작
  Serial.begin(9600);
  Serial.println("기본 버튼 제어 시작!");
  Serial.println("버튼을 눌러 LED를 제어하세요.");
}

void loop() {
  // 버튼 상태 읽기 (내부 풀업이므로 눌렀을 때 LOW)
  int buttonState = digitalRead(BUTTON_PIN);
  
  if (buttonState == LOW) {
    // 버튼이 눌렸을 때
    digitalWrite(LED_PIN, HIGH);  // LED 켜기
    Serial.println("버튼 눌림 → LED ON");
  } else {
    // 버튼이 떼졌을 때  
    digitalWrite(LED_PIN, LOW);   // LED 끄기
    Serial.println("버튼 떼임 → LED OFF");
  }
  
  // 디바운싱을 위한 짧은 지연
  delay(50);
}