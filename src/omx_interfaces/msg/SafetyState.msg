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
