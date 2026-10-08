# OMX 세컨드 배리어 — ROS 2 (Humble) 작업공간

산업 로봇용 보조 안전장치 공모전 시제품입니다. (인증된 안전장치가 아닙니다.)
Jetson Orin Nano Super 한 대가 사람 인식, 로봇 링크 거리 계산, 감속·정지, 이벤트 녹화·DB, 관리 웹을 모두 처리합니다.
`~/omx-safety-demo`의 파이썬 단일 프로세스 버전을 ROS 2 노드 5개로 나눴습니다. 기능은 그대로 유지하고, 데모는 백업으로 남겨 두었습니다.

## 구조

```
[perception]  D435i + 입구 USB 카메라 · v2 통합 모델(fall-down/helmet/human) 엔진 1개를 실행 통로 2개로 · 녹화
   │ /omx/people (사람이 보이면 매 프레임, 없으면 5Hz)   /omx/helmet
   ▼
[safety]  판단은 여기 한 곳 (SafetyManager)
   │ /omx/permit 20Hz ──▶ [robot]  실행기 50Hz · OpenRB-150 · 티칭 · 작업 순서
   │ /omx/safety_state ─▶ [io]     아두이노 · 출입문·불빛 · 컨베이어
   ▼
[web]  제어판 :8000 · 이벤트 DB · 자원 표시 (명령은 각 노드의 /omx/<노드>/cmd 서비스로)
```

- **안전 원칙**: 판단은 한 곳에서만 하고, 로봇 명령은 한 통로로만 보냅니다. 로봇은 허가가 계속 들어올 때만 움직입니다.
- **장애 격리**: 노드 하나가 죽어도 나머지는 계속 동작합니다. 안전 쪽으로 멈추고, 다시 떠도 재가동 승인 전까지는 정지 상태를 유지합니다.

| 죽은 노드 | 결과 (가상 시스템 실측) |
|---|---|
| perception | 약 1.7초 안에 '인식 멈춤' 고장 정지 |
| safety | 약 0.5초 안에 로봇 속도 0. 장치 노드는 컨베이어를 정지 |
| io | 약 1.7초 안에 '장치 통신 고장' 정지 |
| web | 로봇과 안전 판단은 영향 없음 |
| robot | 약 1.7초 안에 '로봇 노드 응답 없음' 정지. 모터는 마지막 자세를 유지 |

- **GPU는 인식 노드 하나만 씁니다.** CUDA 컨텍스트를 한 번만 올리기 위해서입니다. 다른 프로세스에서 쓰면 약 250MB가 더 듭니다.
- 카메라 영상은 ROS 토픽으로 보내지 않습니다. 웹 화면 영상은 `/dev/shm`의 JPEG로 넘기고, 제어판을 보는 사람이 있을 때만 만듭니다.
- 상태 메시지는 값이 바뀔 때만 보내고, 그 밖에는 5Hz 생존 신호만 보냅니다. 파이썬 rclpy는 메시지 하나를 처리하는 비용이 커서입니다.

## 패키지 (`src/`)

| 패키지 | 내용 |
|---|---|
| `omx_interfaces` | 노드 사이의 메시지와 `Command` 서비스 |
| `omx_core` | 핵심 라이브러리 (`omx/`, ROS에 의존하지 않음). 데모 코드를 바탕으로 v2 모델 적용과 허점 수정을 반영 |
| `omx_nodes` | 노드 5개와 웹 제어판 (`webapp.py`: 데모와 같은 주소·응답) |
| `omx_bringup` | launch 파일, systemd 사용자 서비스, 데이터 이전·장애 격리 시험 스크립트 |
| (보관) `person_detector` 외 | 예전 비전 감시 패키지. `COLCON_IGNORE`로 빌드에서 제외 |
| (제외) `open_manipulator` | ROBOTIS 원본 복제본 `https://github.com/ROBOTIS-GIT/open_manipulator` @ `0a4af6a`. 저장소에는 넣지 않음 |

## 처음 한 번

```bash
# 1) 파이썬 환경: 시스템 rclpy·TensorRT + ~/venvs/rs(torch·pyrealsense2·cv2) + 웹 패키지
python3 -m venv --system-site-packages ~/ros2_ws/.venv
echo ~/venvs/rs/lib/python3.10/site-packages > ~/ros2_ws/.venv/lib/python3.10/site-packages/zz_rs_venv.pth
~/ros2_ws/.venv/bin/pip install "fastapi==0.141.1" "uvicorn[standard]==0.54.0" "python-multipart==0.0.32"
# 2) 빌드 (omx_core 빌드할 때 GPU·C++ 가속 라이브러리도 함께 만든다)
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select omx_interfaces omx_core omx_nodes omx_bringup
# 3) 데모 데이터(티칭 자세·보정·비밀번호·이벤트·영상)를 ~/omx_data 로 복사 (데모 폴더는 읽기만)
python3 -I src/omx_bringup/scripts/migrate_demo_data.py
```

## 실행

```bash
source ~/ros2_ws/src/omx_bringup/scripts/omx_env.sh
ros2 launch omx_bringup omx.launch.py                       # 실제 로봇·카메라·아두이노
ros2 launch omx_bringup omx.launch.py sim:=true             # 가상 로봇 + 가상 장치
ros2 launch omx_bringup omx.launch.py sim:=true video:=영상.mp4 data_dir:=/tmp/omx_test port:=8010   # 장치 없이 시험
```

- 제어판: `http://<젯슨 IP>:8000`
- 운영(부팅 시 자동 시작)은 systemd 사용자 서비스를 씁니다. `src/omx_bringup/scripts/install_systemd.sh`를 참고하세요.
- 노드는 `~/ros2_ws/.venv` 파이썬으로 실행합니다.
- `ROS_LOCALHOST_ONLY=1`로 두어 DDS 통신이 Wi-Fi로 나가지 않게 합니다.

## 데이터 (`~/omx_data`, `OMX_DATA_DIR`로 변경)

`events.db`, `clips/`, `settings.json`, `calib.json`, `poses.json`, `secret.key`, `logs/<노드>.log`

설정은 `share/omx_core/config/default.yaml` 위에 `settings.json`(제어판에서 바꾼 값)을 덮어씁니다.
여러 노드가 파일 잠금 아래에서 함께 저장하고, `/omx/config_changed`로 서로 알립니다.

## 시험

```bash
cd ~/ros2_ws/src/omx_core && export OMX_DATA_DIR=$(mktemp -d)
for t in test_core test_db test_io test_safety_v2; do ~/ros2_ws/.venv/bin/python tests/$t.py; done
~/ros2_ws/.venv/bin/python tests/test_v2.py <영상.mp4>          # v2 모델: GPU = CPU 경로, 엔진 공유, 모델 검증
# 가상 시스템을 띄운 뒤 (빈 데이터 폴더, 비밀번호 omx1234, 사람이 없는 영상)
~/ros2_ws/.venv/bin/python tests/smoke_api.py 8010              # 제어판 API 시나리오 전체
python3 -I ~/ros2_ws/src/omx_bringup/scripts/fault_isolation_test.py 8010   # 노드 강제 종료 시험
```

## 자원 실측 (2026-10-08, 가상 로봇 + 사람이 없는 영상, 입구 카메라 없음, 제어판 안 봄)

| | CPU (코어 1개 = 100%) | 메모리 PSS |
|---|---|---|
| 데모 (프로세스 1개) | 39% | 361 MB |
| ROS 2 노드 5개 | 61% | 501 MB |
| ├ perception | 39% | 335 MB |
| ├ robot / safety / io / web | 9% / 6% / 3% / 4% | 38 / 33 / 34 / 62 MB |

늘어난 부분은 프로세스를 나눈 비용입니다. 파이썬과 rclpy 프로세스는 하나당 약 35MB이고, 노드 사이의 메시지 처리 비용도 더해집니다.
처음 나눴을 때는 CPU가 202%였습니다. 다음을 적용해서 줄였습니다.
- 단일 스레드 실행기
- 값이 바뀔 때만 보내는 상태 메시지
- 제어판을 볼 때만 보내는 스냅샷
- 사람이 있을 때만 매 프레임 보내는 인식 결과
