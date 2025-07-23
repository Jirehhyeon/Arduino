/*
  Button_Toggle.ino
  버튼으로 LED 토글 (ON/OFF 상태 변경)
  디바운싱 기법 적용
  
  회로:
  - 2번 핀: 푸시 버튼 (내부 풀업 저항)
  - 13번 핀: LED
*/

const int BUTTON_PIN = 2;
const int LED_PIN = 13;

// 디바운싱 변수
int lastButtonState = HIGH;      // 이전 버튼 상태
int currentButtonState = HIGH;   // 현재 버튼 상태  
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;  // 50ms 디바운스

// LED 상태
bool ledState = false;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("버튼 토글 제어 시작!");
  Serial.println("버튼을 누를 때마다 LED가 켜지거나 꺼집니다.");
}

void loop() {
  // 현재 버튼 상태 읽기
  int reading = digitalRead(BUTTON_PIN);
  
  // 버튼 상태가 변했다면 디바운스 타이머 리셋
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // 디바운스 시간이 지났다면
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    
    // 버튼 상태가 실제로 변했다면
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      // 버튼이 눌렸다면 (HIGH → LOW)
      if (currentButtonState == LOW) {
        ledState = !ledState;  // LED 상태 반전
        digitalWrite(LED_PIN, ledState);
        
        Serial.print("버튼 클릭! LED 상태: ");
        Serial.println(ledState ? "ON" : "OFF");
      }
    }
  }
  
  // 이전 상태 업데이트
  lastButtonState = reading;
}