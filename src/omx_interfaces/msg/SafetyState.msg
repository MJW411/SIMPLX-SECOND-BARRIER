# 안전 관리자 상태 (safety → robot, io, web)
float64 t
string state            # FIRE / STOP / SLOW / CONVEYOR / NORMAL
string zone             # CLEAR / SLOW / STOP
bool fire
bool maintenance
bool teach_mode
string stop_latched
float32 min_dist
uint16 vanish_unresolved
uint16 vanish_pending
string conveyor_alarm
bool fallen
bool human_alarm        # 사람이 정지 거리 안에 들어와 멈춤 → 재가동 전까지 유지 (벽면 빨강)
