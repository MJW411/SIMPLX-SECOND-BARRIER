# 로봇 실행기 상태 (robot → safety, perception, io)
float64 t
float64[6] q            # 관절 5 + 그리퍼 (rad)
bool ok                 # 상태를 한 번이라도 읽었는가
bool torque_on
bool freedrive
bool estop
string fault
string motion           # 진행 중 동작 이름 ("jog" = 조그)
bool cycle_running      # 작업 순서 실행 중
# 최근 동작 결과 (원격 동작 핸들이 기다릴 때 쓴다): 같은 순서의 id / 상태 / 이유
uint32[] motion_ids
string[] motion_status  # running / done / cancelled / failed
string[] motion_reason
