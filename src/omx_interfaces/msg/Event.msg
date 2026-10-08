# 이벤트 (모든 노드 → web: DB 저장·알림, perception: 녹화)
string key              # 노드에서 만든 고유 키 (DB 중복 방지, 클립 연결)
string type
string level            # info / warn / alarm
string message
bool clip               # 사고 영상 녹화
bool until_end          # 녹화를 end 이벤트가 올 때까지 (화재)
bool end_clip           # key 의 진행 중 녹화를 끝낸다 (type 은 비어 있음)
float64 wall_t          # time.time()
string source           # 보낸 노드
