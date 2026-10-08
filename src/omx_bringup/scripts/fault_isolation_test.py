#!/usr/bin/env python3
"""장애 격리 시험: 노드를 하나씩 강제 종료(kill -9)하고 결과를 확인한다. launch(respawn) 로 띄운 가상 시스템 대상.

  ros2 launch omx_bringup omx.launch.py sim:=true video:=<사람 없는 영상> data_dir:=<임시 폴더> port:=8010 &
  python3 fault_isolation_test.py 8010 [비밀번호]

| 죽인 노드   | 기대 |
| perception | 1초 남짓 안에 '인식 멈춤' 고장 정지 → 허가 0. 다시 떠도 재가동 승인 전까지 정지 |
| safety     | 0.3초 안에 로봇 허가 끊김(속도 0), 장치 노드는 안전 노드 끊김 감지(컨베이어 정지) |
| io         | 안전 관리자가 '장치 통신 고장'으로 정지 |
| web        | 로봇·안전 판단 영향 없음 (정상 상태 유지) |
| robot      | 안전 관리자가 '로봇 노드 응답 없음' 고장 정지. 다시 뜨면 자리 유지 |
"""
import http.cookiejar
import json
import os
import signal
import subprocess
import sys
import time
import urllib.request

PORT = sys.argv[1] if len(sys.argv) > 1 else "8010"
PW = sys.argv[2] if len(sys.argv) > 2 else "omx1234"
BASE = f"http://127.0.0.1:{PORT}"
cj = http.cookiejar.CookieJar()
op = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(cj))
fails = []


def call(path, body=None, timeout=10):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BASE + path, data=data, method="POST" if body is not None else "GET",
                                 headers={"Content-Type": "application/json"})
    try:
        with op.open(req, timeout=timeout) as r:
            return json.loads(r.read())
    except Exception as e:
        return {"_err": str(e)}


def check(c, what):
    print(("  OK   " if c else "  FAIL ") + what, flush=True)
    if not c:
        fails.append(what)


def pid_of(node):
    r = subprocess.run(["pgrep", "-f", f"^/home/user/ros2_ws/.venv/bin/python -m omx_nodes.{node}_node"],
                       capture_output=True, text=True)
    p = r.stdout.split()
    return int(p[0]) if p else None


def wait(pred, timeout, step=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            v = pred()
            if v:
                return time.monotonic() - t0
        except Exception:
            pass
        time.sleep(step)
    return None


def state():
    return call("/api/state", timeout=3)


def login():
    return wait(lambda: call("/api/login", {"user": "admin", "password": PW}).get("ok"), 30, 0.5) is not None


def to_normal():
    call("/api/robot/torque", {"on": True})
    for _ in range(20):
        r = call("/api/safety/restart", {"clear_vanish": True})
        if r.get("ok"):
            break
        time.sleep(0.5)
    return wait(lambda: state()["robot"]["permit"] == 1.0 and state()["safety"]["state"] == "NORMAL", 10) is not None


def kill(node):
    p = pid_of(node)
    os.kill(p, signal.SIGKILL)
    return p


def respawned(node, old):
    return wait(lambda: (pid_of(node) or old) != old, 15, 0.2) is not None


check(login(), "로그인")
check(to_normal(), "시작: 정상·허가 1.0")

print("[perception 강제 종료]")
old = kill("perception")
dt = wait(lambda: state()["safety"]["stop_latched"].startswith("인식 멈춤"), 5)
check(dt is not None and dt < 2.0, f"인식 멈춤 고장 정지 ({dt and round(dt, 2)}s)")
check(state()["robot"]["permit"] == 0.0, "허가 0")
check(respawned("perception", old), "3초 뒤 다시 뜸")
time.sleep(4)
check(state()["safety"]["state"] == "STOP", "다시 떠도 재가동 승인 전까지 정지 유지")
check(to_normal(), "재가동 승인 후 정상")

print("[safety 강제 종료]")
old = kill("safety")
dt = wait(lambda: call("/api/robot/stop", {}) is not None and state()["robot"]["scale"] == 0.0, 3, 0.02)
check(dt is not None and dt < 0.8, f"로봇 속도 배율 0 (허가 끊김, {dt and round(dt, 2)}s)")
dt = wait(lambda: state()["safety"]["reasons"][0] == "안전 노드 응답 없음" or call("/api/state")["nodes"]["safety"]["alive"] is False, 5)
check(respawned("safety", old), "다시 뜸")
time.sleep(3)
s = state()
check(s["safety"]["stop_latched"] == "시스템 시작", f"다시 뜬 안전 노드는 정지로 시작 ({s['safety']['stop_latched']})")
check(to_normal(), "재가동 승인 후 정상")

print("[io 강제 종료]")
old = kill("io")
dt = wait(lambda: "장치" in state()["safety"]["stop_latched"], 5)
check(dt is not None, f"장치 통신 고장 정지 ({dt and round(dt, 2)}s)")
check(respawned("io", old), "다시 뜸")
time.sleep(3)
check(to_normal(), "재가동 승인 후 정상")

print("[web 강제 종료]")
old = kill("web")
time.sleep(1.0)
check(call("/api/state", timeout=1).get("_err") is not None, "웹은 응답 없음")
check(respawned("web", old), "다시 뜸")
check(login(), "다시 로그인")
s = state()
check(s["safety"]["state"] == "NORMAL" and s["robot"]["permit"] == 1.0, f"그동안 로봇·안전 판단은 그대로 ({s['safety']['state']})")

print("[robot 강제 종료]")
old = kill("robot")
dt = wait(lambda: "로봇" in state()["safety"]["stop_latched"], 5)
check(dt is not None, f"로봇 노드 응답 없음 → 고장 정지 ({dt and round(dt, 2)}s)")
check(respawned("robot", old), "다시 뜸")
time.sleep(3)
s = state()
check(s["safety"]["state"] == "STOP", "재가동 승인 전까지 정지 유지")

print()
print("실패 %d건" % len(fails) if fails else "모두 통과")
sys.exit(1 if fails else 0)
