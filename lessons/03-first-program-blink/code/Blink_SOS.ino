/*
  Blink_SOS.ino
  LED로 SOS 모스 부호 신호 보내기
  
  모스 부호 규칙:
  - 점(dot): 짧은 신호
  - 대시(dash): 긴 신호 (점의 3배)
  - 문자 내 간격: 점 1개 길이
  - 문자 간 간격: 점 3개 길이
  - 단어 간 간격: 점 7개 길이
  
  SOS: ··· --- ···
  
  회로:
  - 13번 핀: LED (+) - 저항(220Ω) - GND
*/

const int LED_PIN = 13;
const int DOT_DURATION = 200;    // 점의 기본 시간 (밀리초)

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("SOS 신호 시작!");
}

void loop() {
  // S 신호 (···)
  sendS();
  delay(DOT_DURATION * 3);  // 문자 간 간격
  
  // O 신호 (---)
  sendO();
  delay(DOT_DURATION * 3);  // 문자 간 간격
  
  // S 신호 (···)
  sendS();
  delay(DOT_DURATION * 7);  // 단어 간 간격
}

// 점(dot) 신호 보내기
void dot() {
  digitalWrite(LED_PIN, HIGH);
  delay(DOT_DURATION);
  digitalWrite(LED_PIN, LOW);
  delay(DOT_DURATION);  // 신호 간 간격
}

// 대시(dash) 신호 보내기
void dash() {
  digitalWrite(LED_PIN, HIGH);
  delay(DOT_DURATION * 3);  // 대시는 점의 3배
  digitalWrite(LED_PIN, LOW);
  delay(DOT_DURATION);  // 신호 간 간격
}

// S 보내기 (···)
void sendS() {
  Serial.print("S");
  for(int i = 0; i < 3; i++) {
    dot();
  }
}

// O 보내기 (---)
void sendO() {
  Serial.print("O");
  for(int i = 0; i < 3; i++) {
    dash();
  }
}