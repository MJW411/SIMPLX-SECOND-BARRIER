#!/usr/bin/env bash
# systemd 사용자 서비스로 설치 (부팅 시 자동 시작은 마지막 줄의 enable 로).
set -e
SRC="$(cd "$(dirname "$0")/../systemd" && pwd)"
mkdir -p ~/.config/systemd/user
cp "$SRC"/omx*.target "$SRC"/omx-*.service ~/.config/systemd/user/
systemctl --user daemon-reload
echo "설치됨. 시작: systemctl --user start omx.target / 상태: systemctl --user status 'omx-*'"
echo "부팅 시 자동 시작: systemctl --user enable omx.target && sudo loginctl enable-linger $USER"
