/*
  Blink_Basic.ino
  기본 LED 깜빡이기 프로그램
  
  이 프로그램은 아두이노의 내장 LED를 1초 간격으로
  켜고 끄는 동작을 반복합니다.
  
  회로:
  - 내장 LED 사용 (13번 핀)
  
  작성일: 2024
  작성자: Arduino 임베디드 강의
*/

// LED가 연결된 핀 번호를 상수로 정의
const int LED_PIN = 13;  // 대부분의 아두이노 보드는 13번 핀에 내장 LED가 있음

void setup() {
  // 13번 핀을 출력 모드로 설정
  // 이 설정은 프로그램 시작시 한 번만 실행됩니다
  pinMode(LED_PIN, OUTPUT);
  
  // 시리얼 통신 시작 (디버깅용)
  Serial.begin(9600);
  Serial.println("LED Blink 프로그램 시작!");
}

void loop() {
  // LED 켜기
  digitalWrite(LED_PIN, HIGH);  // HIGH는 5V를 출력
  Serial.println("LED ON");     // 시리얼 모니터에 상태 출력
  delay(1000);                  // 1000밀리초(1초) 대기
  
  // LED 끄기
  digitalWrite(LED_PIN, LOW);   // LOW는 0V를 출력
  Serial.println("LED OFF");    // 시리얼 모니터에 상태 출력
  delay(1000);                  // 1초 대기
  
  // loop() 함수는 자동으로 반복 실행됩니다
}