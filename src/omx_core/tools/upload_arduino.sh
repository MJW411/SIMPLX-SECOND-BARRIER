#!/bin/bash
# 아두이노 나노에 스케치를 올린다.  사용: tools/upload_arduino.sh [스케치 폴더 이름, 기본 omx_io] [포트]
# 라이브러리(Adafruit NeoPixel)는 프로젝트 안 arduino/.arduino-user 에 둔다 (홈 폴더를 건드리지 않게).
# 시스템(ros2 launch)이 켜져 있으면 장치 노드가 포트를 쥐고 있으므로 먼저 끈다.
set -e
cd "$(dirname "$0")/../arduino"
export ARDUINO_DIRECTORIES_USER="$PWD/.arduino-user"
SKETCH="${1:-omx_io}"
PORT="${2:-$(ls /dev/serial/by-id/*FTDI* /dev/serial/by-id/*CH340* /dev/serial/by-id/*1a86* 2>/dev/null | head -1)}"
PORT="${PORT:-/dev/ttyUSB0}"
[ -d .arduino-user/libraries/Adafruit_NeoPixel ] || arduino-cli lib install "Adafruit NeoPixel"
LIBS=(--libraries "$HOME/Arduino/libraries" --libraries .arduino-user/libraries)
# 나노 복제품은 옛 부트로더(57600)인 경우가 많아 새 부트로더가 실패하면 옛 것으로 다시 시도한다
for CPU in atmega328 atmega328old; do
  FQBN="arduino:avr:nano:cpu=$CPU"
  arduino-cli compile --fqbn "$FQBN" "${LIBS[@]}" "$SKETCH" >/dev/null
  if arduino-cli upload --fqbn "$FQBN" -p "$PORT" "$SKETCH"; then
    echo "업로드 완료: $SKETCH → $PORT ($CPU)"
    exit 0
  fi
done
echo "업로드 실패: $PORT" >&2
exit 1
