/*
  RGB_Color_Mixer.ino
  3개의 가변저항으로 RGB LED 색상 조합 만들기
  
  회로:
  - A0: 빨강 조절 가변저항
  - A1: 초록 조절 가변저항  
  - A2: 파랑 조절 가변저항
  - 9번 핀: 빨강 LED (220Ω 저항)
  - 10번 핀: 초록 LED (220Ω 저항)
  - 11번 핀: 파랑 LED (220Ω 저항)
*/

// 핀 정의
const int RED_POT_PIN = A0;
const int GREEN_POT_PIN = A1;
const int BLUE_POT_PIN = A2;

const int RED_LED_PIN = 9;
const int GREEN_LED_PIN = 10;
const int BLUE_LED_PIN = 11;

void setup() {
  // LED 핀들을 출력으로 설정
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("RGB 색상 조합기 시작!");
  Serial.println("3개의 가변저항으로 빨강, 초록, 파랑을 조절하세요.");
  Serial.println("R값, G값, B값, 색상명");
}

void loop() {
  // 각 가변저항 값 읽기
  int redPot = analogRead(RED_POT_PIN);
  int greenPot = analogRead(GREEN_POT_PIN);
  int bluePot = analogRead(BLUE_POT_PIN);
  
  // PWM 값으로 변환 (0~255)
  int redValue = map(redPot, 0, 1023, 0, 255);
  int greenValue = map(greenPot, 0, 1023, 0, 255);
  int blueValue = map(bluePot, 0, 1023, 0, 255);
  
  // RGB LED에 색상 값 적용
  analogWrite(RED_LED_PIN, redValue);
  analogWrite(GREEN_LED_PIN, greenValue);
  analogWrite(BLUE_LED_PIN, blueValue);
  
  // 색상 정보 출력
  Serial.print(redValue);
  Serial.print(", ");
  Serial.print(greenValue);
  Serial.print(", ");
  Serial.print(blueValue);
  Serial.print(" → ");
  Serial.println(getColorName(redValue, greenValue, blueValue));
  
  delay(200);
}

// 색상명 추정 함수
String getColorName(int r, int g, int b) {
  // 주요 색상 판별 (간단한 버전)
  if (r > 200 && g < 100 && b < 100) return "빨강";
  if (r < 100 && g > 200 && b < 100) return "초록";
  if (r < 100 && g < 100 && b > 200) return "파랑";
  if (r > 200 && g > 200 && b < 100) return "노랑";
  if (r > 200 && g < 100 && b > 200) return "자홍";
  if (r < 100 && g > 200 && b > 200) return "청록";
  if (r > 200 && g > 200 && b > 200) return "흰색";
  if (r < 50 && g < 50 && b < 50) return "검정";
  if (r > 150 && g > 100 && b < 100) return "주황";
  if (r > 100 && g < 100 && b > 100) return "보라";
  if (r > 200 && g > 150 && b > 150) return "분홍";
  
  return "혼합색";
}