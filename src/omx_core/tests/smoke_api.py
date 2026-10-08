"""제어판 API 연기 시험 (가상 로봇으로 띄운 서버 대상).

  OMX_DATA_DIR=/tmp/omx_smoke_data ./run.sh --sim --video 파일.mp4 --port 8001 &   python tests/smoke_api.py 8001
  (OMX_DATA_DIR 없이 띄우면 실제 서버의 data/ — 이벤트 DB·영상·설정 — 에 시험 기록이 섞인다)
시나리오: 로그인 → 재가동 → 토크 → 조그 → 자세 저장 → 순서 → 반복 → 컨베이어 이상 → 화재 → 상황 종료 → 이벤트·클립
"""
import http.cookiejar
import json
import sys
import time
import urllib.request

PORT = sys.argv[1] if len(sys.argv) > 1 else "8001"
BASE = f"http://127.0.0.1:{PORT}"
cj = http.cookiejar.CookieJar()
op = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(cj))
fails = []


def call(path, body=None, method=None, expect_ok=True):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BASE + path, data=data, method=method or ("POST" if body is not None else "GET"),
                                 headers={"Content-Type": "application/json"})
    try:
        with op.open(req, timeout=10) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        msg = json.loads(e.read() or b"{}")
        if expect_ok:
            fails.append(f"{path}: {e.code} {msg}")
        return {"_err": e.code, **msg}


def st():
    return call("/api/state")


def check(cond, what):
    print(("  OK  " if cond else "  FAIL") + " " + what)
    if not cond:
        fails.append(what)


def wait(pred, timeout=15, what=""):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = st()
        if pred(s):
            return s
        time.sleep(0.3)
    check(False, f"timeout: {what}")
    return st()


check(call("/api/state", expect_ok=False).get("_err") == 401, "로그인 전 401")
call("/api/login", {"user": "admin", "password": "omx1234"})
s = st()
check(s["safety"]["state"] == "STOP", "부팅 시 정지 유지(자동 재가동 없음)")
r = call("/api/cycle/start", {}, expect_ok=False)
check("_err" in r, "순서 없이 반복 시작 거부")
call("/api/robot/torque", {"on": True})
r = call("/api/safety/restart", {})
check(r["ok"], f"재가동 승인 {r}")
s = wait(lambda s: s["robot"]["permit"] == 1.0, what="허가 1.0")
# 휴식 자세에서 준비 자세로
call("/api/robot/tcp", {"x": 0.16, "y": 0.0, "z": 0.15, "pitch": 0.9})
s = wait(lambda s: not s["robot"]["motion"], 30, "TCP 이동 완료")
check(abs(s["robot"]["tcp"][0] - 0.16) < 0.01, f"TCP x 도달 {s['robot']['tcp']}")
r = call("/api/robot/tcp", {"x": 0.9, "y": 0.0, "z": 0.15, "pitch": 0.9}, expect_ok=False)
check(r.get("_err") == 400, f"도달 불가 목표 거부: {r.get('detail')}")
call("/api/poses/save", {"name": "home"})
call("/api/poses/home", {"name": "home"})
# 조그 (웹소켓 대신 실행기 경로 확인은 test_core에서) — 자세 여러 개 저장
for name, x, z in [("pick_above", 0.20, 0.10), ("pick", 0.20, 0.05), ("place_above", 0.14, 0.12), ("place", 0.14, 0.06)]:
    call("/api/robot/tcp", {"x": x, "y": 0.05 if "pick" in name else -0.06, "z": z, "pitch": 1.2})
    wait(lambda s: not s["robot"]["motion"], 30, f"{name} 이동")
    call("/api/poses/save", {"name": name})
# 그리퍼 가동 범위: 너무 가까우면 거부, 한계 밖은 한계로, 기존 범위가 잘못돼 있어도 한쪽씩 저장 가능
r = call("/api/gripper/range", {"close": 0.0, "open": 0.05}, expect_ok=False)
check(r.get("_err") == 400, f"가동 범위 너무 좁으면 거부: {r.get('detail', '')[:40]}")
call("/api/gripper/range", {"close": -0.03, "open": 0.9})
g = st()["teach"]["gripper"]
check(g["close"] == 0.0 and g["open"] == 0.9, f"가동 범위 저장 (닫힘 -0.03 → 한계 0.0): {g}")
call("/api/sequence/template", {})
s = st()
check(len(s["teach"]["sequence"]) == 12, "픽앤플레이스 템플릿 12단계")
call("/api/robot/home", {})
wait(lambda s: not s["robot"]["motion"], 30, "홈")
call("/api/cycle/start", {"max_cycles": 1})
s = wait(lambda s: "wait_sensor2" in s["cycle"]["status"], 20, "센서2 대기 단계")
check("wait_sensor2" in s["cycle"]["status"], f"센서2 대기: {s['cycle']['status']}")
s = wait(lambda s: s["devices"]["conveyor_on"], 3, "작업 중 컨베이어 가동")
check(s["devices"]["conveyor_on"] and s["site"]["led"] == "B", f"작업 중 + 센서 비어 있음 → 가동, 하늘색 ({s['site']['led']})")
call("/api/devices/virtual", {"sensor2": True})
s = wait(lambda s: not s["devices"]["conveyor_on"], 3, "도착 센서 → 정지")
check(not s["devices"]["conveyor_on"], "도착 센서 감지 → 컨베이어 정지")
time.sleep(0.5)
call("/api/devices/virtual", {"sensor2": False})
s = wait(lambda s: not s["cycle"]["running"], 90, "1회 사이클 완료")
check("완료" in s["cycle"]["status"], f"사이클 결과: {s['cycle']['status']}")
# 컨베이어 이상 (제한시간 1초로)
call("/api/settings/conveyor", {"travel_timeout": 1.0})
call("/api/cycle/start", {"max_cycles": 1})
s = wait(lambda s: s["safety"]["conveyor_alarm"], 8, "컨베이어 이상")
check(not s["devices"]["conveyor_on"], "제한시간 초과 → 컨베이어 정지")
call("/api/cycle/stop", {})
wait(lambda s: not s["cycle"]["running"], 20, "작업 정지")
check(s["safety"]["state"] == "CONVEYOR", f"상태 CONVEYOR: {s['safety']['state']}")
call("/api/safety/conveyor_clear", {})
call("/api/settings/conveyor", {"travel_timeout": 15.0})
# 티칭 모드: 작업 실행 차단, 켜고 끄기 기록
call("/api/safety/teach_mode", {"on": True})
s = wait(lambda s: any("티칭 모드" in x for x in s["safety"]["reasons"]), 3, "티칭 모드 표시")
check(s["safety"]["teach_mode"] and any("티칭 모드" in x for x in s["safety"]["reasons"]), "티칭 모드 켜짐")
r = call("/api/cycle/start", {"max_cycles": 1}, expect_ok=False)
check("_err" in r and "티칭 모드" in r.get("detail", ""), f"티칭 모드 중 작업 실행 거부: {r.get('detail', '')[:40]}")
call("/api/safety/teach_mode", {"on": False})
check(not st()["safety"]["teach_mode"], "티칭 모드 꺼짐")
# 출입문: 수동 열기 → 닫힘 시간 뒤 닫힘
call("/api/settings/door", {"close_sec": 1.0})
call("/api/devices/door_open", {})
s = wait(lambda s: s["devices"]["door_open"], 3, "수동 열기")
check(s["devices"]["door_open"], "출입문 수동 열기")
s = wait(lambda s: not s["devices"]["door_open"], 4, "닫힘 시간 뒤 닫힘")
check(not s["devices"]["door_open"], "닫힘 시간 뒤 닫힘")
call("/api/settings/door", {"close_sec": 5.0})
# 정비 모드
call("/api/safety/maintenance", {"on": True})
r = call("/api/robot/home", {}, expect_ok=False)
check(r.get("_err") == 409, "정비 모드에서 이동 거부")
call("/api/safety/maintenance", {"on": False})
# 작업 자세로 옮긴 뒤 화재
call("/api/robot/goto", {"pose": "pick"})
wait(lambda s: not s["robot"]["motion"], 30, "pick 이동")
call("/api/devices/virtual", {"fire": True})
s = wait(lambda s: s["safety"]["state"] == "FIRE", 5, "화재 상태")
check(s["devices"]["door_open"], "화재 시 문 개방")
s = wait(lambda s: not s["robot"]["motion"] and abs(s["robot"]["tcp"][0] - 0.16) < 0.01, 40, "화재 홈 복귀")
r = call("/api/safety/fire_end", {}, expect_ok=False)
check(r.get("_err") == 409, "센서 감지 중 상황 종료 거부")
call("/api/devices/virtual", {"fire": False})
r = call("/api/safety/fire_end", {})
check(r["ok"], "화재 상황 종료")
time.sleep(0.3)
s = st()
check(s["safety"]["state"] == "STOP", "화재 종료 후 정지 유지(재가동 승인 필요)")
r = call("/api/safety/restart", {})
check(r["ok"], "재가동")
# 비상정지 → 재가동 버튼 하나로 복구 (비상정지 해제·토크 확인까지 자동)
call("/api/robot/estop", {})
s = wait(lambda s: s["safety"]["state"] == "STOP", 5, "비상정지 → 정지")
check(s["robot"]["estop"], "비상정지 눌림")
r = call("/api/safety/restart", {})
check(r["ok"] and any("비상정지" in x for x in r.get("steps", [])), f"재가동 한 번으로 비상정지 해제+재가동: {r.get('steps')} {r.get('problems')}")
s = wait(lambda s: s["safety"]["state"] == "NORMAL" and not s["robot"]["estop"], 5, "재가동 후 정상")
# 토크가 꺼진 상태에서 비상정지 → 재가동 한 번으로 토크까지 켜짐
call("/api/robot/torque", {"on": False, "force": True})
call("/api/robot/estop", {})
wait(lambda s: s["safety"]["state"] == "STOP", 5, "비상정지 → 정지")
r = call("/api/safety/restart", {})
check(r["ok"] and "토크 켜기" in r.get("steps", []), f"재가동 한 번으로 토크까지: {r.get('steps')}")
# 이벤트
ev = call("/api/events?limit=50")["events"]
types = {e["type"] for e in ev}
check({"conveyor_jam", "fire", "estop"} <= types, f"이벤트 기록: {sorted(types)}")
print("클립 인코딩 대기 (후 16초 + 인코딩)…")
t0 = time.time()
while time.time() - t0 < 60:
    ev = call("/api/events?limit=50")["events"]
    cj_ev = [e for e in ev if e["type"] == "conveyor_jam"]
    if cj_ev and cj_ev[0]["clip_status"] == "ready":
        break
    time.sleep(2)
check(cj_ev and cj_ev[0]["clip_status"] == "ready", f"컨베이어 클립 완성 {cj_ev[0]['clip'] if cj_ev else None}")
fire_ev = [e for e in ev if e["type"] == "fire"]
check(fire_ev and fire_ev[0]["clip_status"] in ("ready", "recording"), f"화재 클립 {fire_ev[0]['clip_status'] if fire_ev else None}")
# 영상 메뉴: 분류별로 해당 종류만
for cat, types in (("conveyor", {"conveyor_jam"}), ("fire", {"fire"}), ("human", {"human_stop"})):
    r = call(f"/api/clips?cat={cat}")
    got = {c["type"] for c in r["clips"]}
    check(got <= types and r["counts"][cat]["n"] >= len(r["clips"]), f"영상 분류 {cat}: {len(r['clips'])}건 {sorted(got)}")
r = call("/api/clips?cat=other")
check(not {c["type"] for c in r["clips"]} & {"conveyor_jam", "fire", "human_stop"}, "기타 분류에 세 분류 영상 없음")
check(cj_ev[0]["id"] in [c["id"] for c in call("/api/clips?cat=conveyor")["clips"]], "방금 난 컨베이어 이상 영상이 '물품 미도달'에 있음")
call("/api/clips?cat=bad", expect_ok=False)
call("/api/robot/park", {})
wait(lambda s: not s["robot"]["torque"], 60, "휴식 후 토크 끔")
print("\n실패:", fails if fails else "없음")
sys.exit(1 if fails else 0)
