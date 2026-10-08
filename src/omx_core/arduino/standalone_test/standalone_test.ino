// 사용자가 만든 단독 시험용 스케치 (2026-10-07 받은 그대로 보관). 서버와 연동하는 판은 ../omx_io/omx_io.ino
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// 핀 설정
const int IR_PIN    = 2;    // IR 센서 OUT
const int SERVO_PIN = 6;    // 서보 신호선
const int MOTOR_IN1 = 9;    // L298N IN1
const int MOTOR_IN2 = 10;   // L298N IN2
const int PIXEL_PIN = 13;   // 네오픽셀 DIN
const int FLAME_PIN = A0;   // 화염 센서

// 네오픽셀 설정
const int NUM_PIXELS = 16;  // 네오픽셀 LED 개수
const int BRIGHTNESS = 80;  // 밝기 0~255

// 서보 각도
const int ANGLE_RUN  = 93;  // 평소 위치
const int ANGLE_STOP = 10;  // 감지 시 위치

// 서보 이동 시간 (93도 ↔ 10도 전체 이동에 걸리는 시간, ms)
const unsigned long SERVO_MOVE_TIME = 600;
const unsigned long SERVO_STEP_INTERVAL =
  SERVO_MOVE_TIME / abs(ANGLE_RUN - ANGLE_STOP);

// 물체가 치워진 뒤 복귀까지 대기 시간 (ms)
const unsigned long RELEASE_TIME = 500;

// 화염 설정 (풀다운 배선: 불꽃 감지 시 값 증가)
const int FLAME_THRESHOLD = 100;            // 이 값보다 크면 불꽃 감지 (테스트 후 조정)
const unsigned long FLAME_HOLD_TIME = 5000; // 마지막 감지 후 경보 유지 시간 (ms)
const unsigned long BLINK_INTERVAL  = 250;  // 빨간색 점멸 간격 (ms)

Servo servo;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// 동작 상태
enum Mode { MODE_RUN, MODE_OBJECT, MODE_FLAME };
Mode mode = MODE_RUN;

unsigned long lastDetectTime = 0;
unsigned long lastFlameTime  = 0;
unsigned long lastBlinkTime  = 0;
unsigned long lastPrintTime  = 0;
bool blinkOn = false;

int currentAngle = ANGLE_RUN;
int targetAngle  = ANGLE_RUN;
unsigned long lastServoStep = 0;

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

// 컨베이어 동작 (반대 방향)
void conveyorRun() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
}

void conveyorStop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

// 서보를 목표 각도까지 1도씩 천천히 이동
void updateServo() {
  if (currentAngle == targetAngle) return;
  if (millis() - lastServoStep < SERVO_STEP_INTERVAL) return;

  lastServoStep = millis();
  currentAngle += (targetAngle > currentAngle) ? 1 : -1;
  servo.write(currentAngle);
}

// 평소 상태: 컨베이어 동작, 녹색, 서보 93도
void enterRun() {
  conveyorRun();
  setColor(0, 255, 0);
  targetAngle = ANGLE_RUN;
  mode = MODE_RUN;
  Serial.println("정상 → 컨베이어 동작, 녹색, 서보 93도");
}

// 물체 감지: 컨베이어 정지, 하늘색, 서보 10도
void enterObject() {
  conveyorStop();
  setColor(0, 180, 255);
  targetAngle = ANGLE_STOP;
  mode = MODE_OBJECT;
  Serial.println("물체 감지 → 컨베이어 정지, 하늘색, 서보 10도");
}

// 화염 감지: 컨베이어 정지, 빨간색 점멸, 서보 10도
void enterFlame() {
  conveyorStop();
  targetAngle = ANGLE_STOP;
  blinkOn = true;
  setColor(255, 0, 0);
  lastBlinkTime = millis();
  mode = MODE_FLAME;
  Serial.println("FLAME DETECTED! → 컨베이어 정지, 빨간색 점멸, 서보 10도");
}

// 빨간색 점멸
void updateBlink() {
  if (millis() - lastBlinkTime < BLINK_INTERVAL) return;
  lastBlinkTime = millis();
  blinkOn = !blinkOn;
  if (blinkOn) setColor(255, 0, 0);
  else         setColor(0, 0, 0);
}

void setup() {
  Serial.begin(9600);

  pinMode(IR_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  conveyorStop();                 // 시작 직후 모터 정지 상태로 고정

  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);

  servo.write(ANGLE_RUN);         // attach 전에 각도 지정 (튐 방지)
  servo.attach(SERVO_PIN);

  delay(500);                     // 전원 안정화 대기
  Serial.println("시작 (IR D2 / 화염 A0 / 서보 D6 / 모터 D9,D10 / LED D13)");
  enterRun();
}

void loop() {
  bool objectDetected = (digitalRead(IR_PIN) == LOW);   // 감지 시 LOW
  int flameValue = analogRead(FLAME_PIN);
  bool flameDetected = (flameValue > FLAME_THRESHOLD);  // 불꽃 감지 시 값 증가

  if (objectDetected) lastDetectTime = millis();

  // 화염 감지가 가장 우선
  if (flameDetected) {
    lastFlameTime = millis();
    if (mode != MODE_FLAME) enterFlame();
  }

  if (mode == MODE_FLAME) {
    if (millis() - lastFlameTime >= FLAME_HOLD_TIME) {
      // 5초간 불꽃이 없으면 경보 해제 후 물체 상태에 따라 복귀
      if (objectDetected) enterObject();
      else                enterRun();
    } else {
      updateBlink();
    }
  } else if (mode == MODE_RUN) {
    if (objectDetected) enterObject();
  } else if (mode == MODE_OBJECT) {
    if (!objectDetected && millis() - lastDetectTime > RELEASE_TIME) {
      enterRun();
    }
  }

  updateServo();

  // 화염 센서 값 1초마다 출력 (기준값 조정용)
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    Serial.print("Flame Value: ");
    Serial.println(flameValue);
  }
}
