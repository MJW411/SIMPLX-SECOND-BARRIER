#!/usr/bin/env bash
# 젯슨을 켜면 `ros2 launch omx_bringup omx.launch.py` 가 바로 실행되게 설치 (systemd 사용자 서비스).
# 사용: bash ~/ros2_ws/src/omx_bringup/scripts/install_autostart.sh
#   끄기: systemctl --user disable --now omx-launch.service
set -e
SRC="$(cd "$(dirname "$0")/../systemd" && pwd)"
mkdir -p ~/.config/systemd/user
cp "$SRC/omx-launch.service" ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable omx-launch.service
echo "설치·자동 시작 등록됨 (omx-launch.service)."
if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != "yes" ]; then
  echo "로그인하지 않아도 부팅하면 바로 시작하려면 한 번만 실행: sudo loginctl enable-linger $USER"
fi
echo "지금 시작: 터미널에서 돌고 있는 launch 를 Ctrl+C 로 끈 뒤  systemctl --user start omx-launch.service"
echo "상태: systemctl --user status omx-launch.service   기록: journalctl --user -u omx-launch.service -f"
