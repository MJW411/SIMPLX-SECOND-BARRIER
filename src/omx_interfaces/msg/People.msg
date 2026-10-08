# 사람 인식 결과 한 프레임 (perception → safety). 시각은 CLOCK_MONOTONIC 초 (프로세스 사이에 같은 시계)
float64 t               # 프레임을 찍은 시각
int64 seq
bool model_ready        # 사람을 볼 수 있는 모델이 올라와 있는가 (아니면 안전 관리자가 고장 정지)
bool calibrated
float32 brightness
float32 min_dist        # 감시 구역 안 최소거리 (없으면 inf)
float32 infer_ms
uint16 height
uint16 width
string cam_error
Person[] persons
