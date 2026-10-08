// OMX 세컨드 배리어 — 현장 입출력 (Arduino Nano)
//
// 배선 (2026-10-08 v4, 사용자 시험 스케치 기준):
//   투입 센서 D11 · 도착 센서 D12 (IR 모듈 SZH-SSBH-002, 감지 시 LOW, 내부 풀업 — 빠진 핀이 '감지'로 읽히지 않게)
//   화염 센서 A0 (불꽃이면 값 증가), 출입문 서보 D6 (93° 닫힘 / 10° 열림), 컨베이어 L298N IN1 D9 · IN2 D10
//   조명 네오픽셀 16개 D13 (항상 백색), 대피 유도 네오픽셀 60개 D3
//   D3: 평상시 은은한 백색, 화재 시 노란 물결이 아두이노 쪽에서 먼 끝(입구) 쪽으로 흐르며 대피 방향을 알린다
//       (불꽃 센서가 직접 감지했을 때 + 젯슨이 화재 상황(Y)을 알리는 동안 — 관리자 '상황 종료'까지)
//   상태 줄: s1 = D11 투입, s2 = D12 도착
//
// 판단은 젯슨이 한다. 아두이노는 젯슨 명령을 따르되, 젯슨 없이도 스스로 지키는 것만 직접 한다:
//   1) 불꽃 감지(0.1초 이상) → 컨베이어 정지, 출입문 열림, 노란불 (마지막 감지 후 5초 유지, 전원 직후 2초는 무시)
//   2) 젯슨 명령이 1초 끊기면 → 컨베이어 정지, 출입문 닫힘(불꽃 제외), 흰불
//   3) 도착 센서(D12) 감지 → 컨베이어 즉시 정지 (젯슨 왕복을 기다리면 물품이 끝을 지나칠 수 있다)
//   투입 센서(D11)로 출발하는 판단은 젯슨이 한다 (사람 근접·화재·작업 여부를 보고). 아두이노는 값만 보낸다.
//
// 프로토콜 (9600bps, 줄 단위 텍스트):
//   젯슨 → 아두이노: "C <conv> <door> <led>"  200ms마다 + 바뀔 때. 받는 것 자체가 하트비트.
//                    conv/door = 0|1, led = G(녹색: 대기·작업 끝) B(하늘색: 작업 중) O(주황 점멸: 사람 접근)
//                    Y(노랑: 화재 — 불꽃이 꺼진 뒤에도 관리자 '상황 종료'까지)
//   아두이노 → 젯슨: "S <s1> <s2> <fire> <door> <conv> <flame>"  바뀔 때 + 200ms마다
//   그 밖의 줄(시작 안내 등)은 젯슨이 무시한다.
// 9600bps인 이유: 네오픽셀 show() 동안 인터럽트가 꺼져(16개 약 0.5ms) 115200이면 받는 글자가 깨질 수 있다.

#include <Servo.h>
#include <Adafruit_NeoPixel.h>

const int PIN_START = 11;
const int PIN_ARRIVE = 12;
const int PIN_GUIDE = 3;
const int PIN_FLAME = A0;
const int PIN_SERVO = 6;
const int PIN_IN1 = 9, PIN_IN2 = 10;
const int PIN_PIXEL = 13;

const int NUM_PIXELS = 16;
const int BRIGHTNESS = 80;

// 대피 유도 물결 (사용자 시험 스케치 값 바탕)
const int GUIDE_PIXELS = 60;
const float GUIDE_MAX = 0.35;                 // 물결 최대 밝기 비율
const float GUIDE_WHITE = 0.20;               // 평상시 백색 밝기 비율 (은은하게)
const float SPACING = 6.0;                    // 빛 덩어리 간격 (LED 개수). 시험 스케치 20 → 6 (2026-10-08 사용자 조정)
const float TAIL = 5.0;                       // 꼬리 길이 (SPACING 보다 작게, 2026-10-08 사용자 조정)
const float HEAD_SOFT = 1.5;                  // 머리 앞쪽 번짐
const float SPEED = 6.0;                      // 초당 이동 칸 수
const float BASE_GLOW = 0.04;                 // 바닥 은은한 빛
const unsigned long FRAME_TIME = 15;          // 갱신 간격 (ms)
const unsigned long SERVO_DETACH_DELAY = 300; // 서보 도착 후 신호를 끊기까지 (LED 갱신 때 서보 떨림 방지)

const int ANGLE_CLOSED = 93;
const int ANGLE_OPEN = 10;
const unsigned long SERVO_MOVE_TIME = 600;   // 93° ↔ 10° 전체 이동 시간 (ms)
const unsigned long SERVO_STEP_INTERVAL = SERVO_MOVE_TIME / abs(ANGLE_CLOSED - ANGLE_OPEN);

const int FLAME_THRESHOLD = 100;              // 이 값보다 크면 불꽃 (시험 후 조정)
const unsigned long FLAME_HOLD_TIME = 5000;   // 마지막 감지 후 화재 유지 (ms)
const unsigned long FLAME_WARMUP = 2000;      // 전원 직후 화염 값이 0.5초쯤 300대에서 떨어진다(실측) → 그동안 무시
const unsigned long FLAME_CONFIRM = 100;      // 기준을 이만큼 계속 넘어야 불꽃 (순간 튐 거르기)
const unsigned long LINK_TIMEOUT = 1000;      // 젯슨 명령이 이만큼 끊기면 스스로 안전 상태로
const unsigned long DEBOUNCE_MS = 30;         // 근접센서 떨림 거르기
const unsigned long TX_INTERVAL = 200;

Servo servo;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_PIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel guide(GUIDE_PIXELS, PIN_GUIDE, NEO_GRB + NEO_KHZ800);

// 젯슨 명령
bool convCmd = false, doorCmd = false;
char ledCmd = 'G';
unsigned long lastCmd = 0;
bool everLinked = false;

// 센서 (떨림 거른 값)
struct Debounced {
  int pin; bool stable; bool raw; unsigned long since;
};
Debounced start = {PIN_START, false, false, 0};
Debounced arrive = {PIN_ARRIVE, false, false, 0};

unsigned long lastFlameTime = 0, flameSince = 0;
bool flameHigh = false;
bool fireOn = false;
int flameValue = 0;

int currentAngle = ANGLE_CLOSED, targetAngle = ANGLE_CLOSED;
unsigned long lastServoStep = 0;
bool servoAttached = false;
unsigned long servoArriveTime = 0;

float phase = 0;
unsigned long lastFrame = 0;
int guideMode = -1;                           // 0 = 백색, 1 = 화재 물결 (같은 백색은 다시 그리지 않는다)

uint32_t shownColor = 0xFFFFFFFF;            // 같은 색이면 show() 를 다시 부르지 않는다

char line[24];
int lineLen = 0;
unsigned long lastTx = 0;
int prevState = -1;

bool readDebounced(Debounced& d) {
  bool r = digitalRead(d.pin) == LOW;        // 감지 시 LOW
  if (r != d.raw) { d.raw = r; d.since = millis(); }
  if (d.stable != d.raw && millis() - d.since >= DEBOUNCE_MS) d.stable = d.raw;
  return d.stable;
}

void showColor(uint32_t c) {
  if (c == shownColor) return;
  shownColor = c;
  for (int i = 0; i < NUM_PIXELS; i++) pixels.setPixelColor(i, c);
  pixels.show();
}

void setConveyor(bool on) {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, on ? HIGH : LOW);    // 사용자 시험 스케치와 같은 방향
}

// 서보는 움직이는 동안만 신호를 준다. 네오픽셀 show() 가 인터럽트를 끄면 서보 펄스가 흔들려 떨기 때문.
void updateServo() {
  if (currentAngle != targetAngle) {
    if (!servoAttached) {
      servo.write(currentAngle);
      servo.attach(PIN_SERVO);
      servoAttached = true;
    }
    if (millis() - lastServoStep < SERVO_STEP_INTERVAL) return;
    lastServoStep = millis();
    currentAngle += (targetAngle > currentAngle) ? 1 : -1;
    servo.write(currentAngle);
    if (currentAngle == targetAngle) servoArriveTime = millis();
  } else if (servoAttached && millis() - servoArriveTime >= SERVO_DETACH_DELAY) {
    servo.detach();
    servoAttached = false;
  }
}

void drawWave() {
  for (int i = 0; i < GUIDE_PIXELS; i++) {     // 0번(아두이노 쪽) → 59번(입구 쪽)으로 흐른다
    float d = fmod(phase - i, SPACING);
    if (d < 0) d += SPACING;
    float b = 0;
    if (d <= TAIL) { float t = 1.0 - d / TAIL; b = t * t; }
    float ahead = SPACING - d;
    if (ahead < HEAD_SOFT) { float f = 1.0 - ahead / HEAD_SOFT; if (f * f > b) b = f * f; }
    if (b < BASE_GLOW) b = BASE_GLOW;
    float k = guide.gamma8((uint8_t)(b * 255)) / 255.0 * GUIDE_MAX;
    guide.setPixelColor(i, guide.Color(255 * k, 170 * k, 0));
  }
  guide.show();
}

// 대피 유도: 평상시 백색은 한 번만 그린다 (다시 show() 하지 않음). 화재 때만 물결을 15ms 마다 그린다.
// 60개 show() 는 인터럽트를 약 1.8ms 끈다(9600bps 에서 2글자 미만 → 수신 버퍼가 받아 둔다).
// 글자가 막 들어와 있는 순간과 서보가 움직이는 중에만 한 번 건너뛴다 (시험 스케치처럼 부드럽게).
// 화재 때 문이 열리는 0.6초 동안은 물결이 잠깐 멈춘다 (서보 떨림 방지, 시험 스케치와 같음).
void updateGuide(bool fire, unsigned long now) {
  if (servoAttached || Serial.available()) return;
  if (!fire) {
    if (guideMode != 0) {
      uint8_t w = 255 * GUIDE_WHITE;
      for (int i = 0; i < GUIDE_PIXELS; i++) guide.setPixelColor(i, guide.Color(w, w, w));
      guide.show();
      guideMode = 0;
    }
    return;
  }
  if (now - lastFrame < FRAME_TIME) return;
  float dt = min(0.1, (now - lastFrame) / 1000.0);
  lastFrame = now;
  phase += SPEED * dt;
  if (phase >= SPACING) phase -= SPACING;
  drawWave();
  guideMode = 1;
}

// "C <conv> <door> <led>"
void handleLine(const char* s) {
  if (s[0] != 'C' || strlen(s) < 7) return;
  if (s[2] != '0' && s[2] != '1') return;
  if (s[4] != '0' && s[4] != '1') return;
  char led = s[6];
  if (led != 'G' && led != 'B' && led != 'O' && led != 'Y') return;
  convCmd = s[2] == '1';
  doorCmd = s[4] == '1';
  ledCmd = led;
  lastCmd = millis();
  everLinked = true;
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      line[lineLen] = 0;
      if (lineLen) handleLine(line);
      lineLen = 0;
    } else if (lineLen < (int)sizeof(line) - 1) {
      line[lineLen++] = ch;
    } else {
      lineLen = 0;                           // 너무 긴 줄은 버린다
    }
  }
}

void setup() {
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_ARRIVE, INPUT_PULLUP);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  setConveyor(false);                        // 시작 직후 모터 정지

  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  showColor(pixels.Color(255, 255, 255));    // 조명: 항상 백색
  guide.begin();
  guide.setBrightness(255);                  // 밝기는 GUIDE_WHITE / GUIDE_MAX 로
  guide.clear();
  guide.show();

  servo.write(ANGLE_CLOSED);                 // attach 전에 각도 지정 (튐 방지)
  servo.attach(PIN_SERVO);
  servoAttached = true;
  servoArriveTime = millis();

  Serial.begin(9600);
  Serial.println("# omx_io v4 (투입 D11 / 도착 D12 / 화염 A0 / 문 서보 D6 / 모터 D9,D10 / 조명 D13 / 대피유도 D3)");
  lastFrame = millis();
}

void loop() {
  readSerial();
  unsigned long now = millis();

  bool a = readDebounced(start);             // 투입 (젯슨이 출발 판단에 쓴다)
  bool b = readDebounced(arrive);            // 도착 (여기서 즉시 정지)
  flameValue = analogRead(PIN_FLAME);
  bool high = flameValue > FLAME_THRESHOLD && now >= FLAME_WARMUP;
  if (high && !flameHigh) flameSince = now;
  flameHigh = high;
  if (high && now - flameSince >= FLAME_CONFIRM) { lastFlameTime = now; fireOn = true; }
  else if (fireOn && now - lastFlameTime >= FLAME_HOLD_TIME) fireOn = false;

  bool linked = everLinked && now - lastCmd < LINK_TIMEOUT;
  if (!linked) { convCmd = false; doorCmd = false; }   // 다시 연결되면 젯슨이 새로 명령해야 움직인다

  bool conv = convCmd && linked && !fireOn && !b;
  bool door = fireOn || (doorCmd && linked);
  setConveyor(conv);
  targetAngle = door ? ANGLE_OPEN : ANGLE_CLOSED;
  updateServo();
  // 대피 유도: 불꽃을 직접 봤을 때 + 젯슨이 화재 상황(Y)을 알리는 동안 (젯슨 연결이 끊겨도 불꽃이면 물결)
  updateGuide(fireOn || (linked && ledCmd == 'Y'), now);
  // D13 조명은 항상 백색 (setup 에서 켬). 젯슨의 상태 색 지시(G/B/O)는 받기만 하고 쓰지 않는다.

  int st = a | b << 1 | fireOn << 2 | door << 3 | conv << 4;
  if (st != prevState || now - lastTx >= TX_INTERVAL) {
    Serial.print("S "); Serial.print(a); Serial.print(' '); Serial.print(b); Serial.print(' ');
    Serial.print(fireOn); Serial.print(' '); Serial.print(door); Serial.print(' '); Serial.print(conv);
    Serial.print(' '); Serial.println(flameValue);
    prevState = st;
    lastTx = now;
  }
}
