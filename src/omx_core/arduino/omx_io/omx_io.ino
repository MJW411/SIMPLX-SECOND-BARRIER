// OMX 세컨드 배리어 — 현장 입출력 (Arduino Nano)
//
// 배선 (2026-10-07 실제 배선):
//   도착 센서 D12 (IR 모듈, 감지 시 LOW, 내부 풀업 — 아무것도 안 꽂힌 핀이 '감지'로 읽히지 않게), 화염 센서 A0 (불꽃이면 값 증가)
//   2026-10-07: 센서는 D12 하나만 쓴다 (D2 에는 아무것도 없음 — 그래서 INPUT 일 때 늘 '감지'였다). 상태 줄의 s1 은 항상 0, s2 가 D12.
//   출입문 서보 D6 (93° 닫힘 / 10° 열림), 컨베이어 L298N IN1 D9 · IN2 D10, 네오픽셀 16개 D13
//
// 판단은 젯슨이 한다. 아두이노는 젯슨 명령을 따르되, 젯슨 없이도 스스로 지키는 것만 직접 한다:
//   1) 불꽃 감지(0.1초 이상) → 컨베이어 정지, 출입문 열림, 노란불 (마지막 감지 후 5초 유지, 전원 직후 2초는 무시)
//   2) 젯슨 명령이 1초 끊기면 → 컨베이어 정지, 출입문 닫힘(불꽃 제외), 흰불
//   3) 도착 센서(D12) 감지 → 컨베이어 즉시 정지 (젯슨 왕복을 기다리면 물품이 끝을 지나칠 수 있다)
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

const int PIN_ARRIVE = 12;
const int PIN_FLAME = A0;
const int PIN_SERVO = 6;
const int PIN_IN1 = 9, PIN_IN2 = 10;
const int PIN_PIXEL = 13;

const int NUM_PIXELS = 16;
const int BRIGHTNESS = 80;

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
const unsigned long BLINK_INTERVAL = 300;     // 주황 점멸 간격
const unsigned long TX_INTERVAL = 200;

Servo servo;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_PIXEL, NEO_GRB + NEO_KHZ800);

// 젯슨 명령
bool convCmd = false, doorCmd = false;
char ledCmd = 'G';
unsigned long lastCmd = 0;
bool everLinked = false;

// 센서 (떨림 거른 값)
struct Debounced {
  int pin; bool stable; bool raw; unsigned long since;
};
Debounced arrive = {PIN_ARRIVE, false, false, 0};

unsigned long lastFlameTime = 0, flameSince = 0;
bool flameHigh = false;
bool fireOn = false;
int flameValue = 0;

int currentAngle = ANGLE_CLOSED, targetAngle = ANGLE_CLOSED;
unsigned long lastServoStep = 0;

uint32_t shownColor = 0xFFFFFFFF;            // 같은 색이면 show() 를 다시 부르지 않는다
unsigned long lastBlink = 0;
bool blinkOn = true;

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

void updateServo() {
  if (currentAngle == targetAngle) return;
  if (millis() - lastServoStep < SERVO_STEP_INTERVAL) return;
  lastServoStep = millis();
  currentAngle += (targetAngle > currentAngle) ? 1 : -1;
  servo.write(currentAngle);
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
  pinMode(PIN_ARRIVE, INPUT_PULLUP);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  setConveyor(false);                        // 시작 직후 모터 정지

  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  showColor(pixels.Color(255, 255, 255));    // 흰불: 젯슨 연결 전

  servo.write(ANGLE_CLOSED);                 // attach 전에 각도 지정 (튐 방지)
  servo.attach(PIN_SERVO);

  Serial.begin(9600);
  Serial.println("# omx_io v3 (도착 센서 D12 / 화염 A0 / 문 서보 D6 / 모터 D9,D10 / LED D13)");
}

void loop() {
  readSerial();
  unsigned long now = millis();

  const bool a = false;                      // 센서1 없음
  bool b = readDebounced(arrive);
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

  // 불빛: 화재(노랑) > 연결 끊김(흰색) > 젯슨 지시
  if (fireOn || (linked && ledCmd == 'Y')) showColor(pixels.Color(255, 200, 0));
  else if (!linked) showColor(pixels.Color(255, 255, 255));
  else if (ledCmd == 'B') showColor(pixels.Color(0, 180, 255));
  else if (ledCmd == 'O') {
    if (now - lastBlink >= BLINK_INTERVAL) { lastBlink = now; blinkOn = !blinkOn; }
    showColor(blinkOn ? pixels.Color(255, 90, 0) : 0);
  } else showColor(pixels.Color(0, 255, 0));

  int st = a | b << 1 | fireOn << 2 | door << 3 | conv << 4;
  if (st != prevState || now - lastTx >= TX_INTERVAL) {
    Serial.print("S "); Serial.print(a); Serial.print(' '); Serial.print(b); Serial.print(' ');
    Serial.print(fireOn); Serial.print(' '); Serial.print(door); Serial.print(' '); Serial.print(conv);
    Serial.print(' '); Serial.println(flameValue);
    prevState = st;
    lastTx = now;
  }
}
