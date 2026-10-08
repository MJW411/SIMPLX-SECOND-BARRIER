# 사람 한 명 (perception → safety). 좌표·거리 단위 m, 영상 좌표 px
float32[4] box          # x1 y1 x2 y2
float32 conf
float32[2] px           # 박스 중심 (u, v)
bool has_pos
float32[3] pos          # 대표 3D 위치 (보정 시 로봇 베이스 기준, 아니면 카메라 기준)
float32 dist            # 로봇(보정 전에는 카메라)까지 최소거리. inf = 깊이 없는 영상
float32 cam_dist
int32 npts
bool in_zone
float32 overlap
float32 near
string name             # human / fall-down / person
bool fallen             # 넘어진 사람 (알림·표시용, 정지 판단은 거리로만)
