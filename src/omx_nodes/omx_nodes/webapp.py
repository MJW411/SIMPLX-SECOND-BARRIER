"""웹 제어판 (FastAPI) — 기존 omx/web/app.py 와 같은 주소·같은 응답. 화면 파일(static)은 그대로 쓴다.

바뀐 점: 객체를 직접 부르던 곳을 각 노드의 관리 명령(Command 서비스)과 상태 스냅샷으로 바꿨다.
- 로봇·티칭·작업 → robot 노드, 안전 조작 → safety 노드, 모델·보정·헬멧 영상 → perception 노드, 장치 → io 노드
- 이벤트 DB 는 이 노드가 가진다 (/omx/events 저장). 영상은 인식 노드가 /dev/shm 에 쓴 JPEG 를 보낸다.
- 비상정지는 서비스 응답을 기다리지 않는 빠른 길(/omx/estop 토픽)을 먼저 쓴다: 안전 노드가 죽어 있어도 로봇이 선다.
"""
import asyncio
import base64
import hashlib
import hmac
import json
import logging
import os
import secrets
import signal
import subprocess
import threading
import time
from pathlib import Path

from fastapi import Body, Depends, FastAPI, File, HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

import omx.web as _web
from omx.events import EVENT_CATS, EVENT_SHOWN, INCIDENTS, STATS_ONLY, STORED
from omx.storage import stats as dash_stats
EXTS = (".pt", ".engine", ".onnx")          # omx.perception.detector 와 같다 (웹 노드가 cv2·numpy 를 불러오지 않게)

from .common import RemoteError, jsonable

log = logging.getLogger("web")
STATIC = Path(_web.__file__).parent / "static"
COOKIE = "omx_session"
PROFILE_NAME = {"figure": "피규어 시연", "human": "실제 사람"}
MOVE_CMDS = {"home", "goto", "joints", "align", "tcp"}      # 기존 API 는 동작 결과를 돌려주지 않았다


def create_app(ctx):
    """ctx: WebNode (cfg, db, sysmon, clip_dir, events, rpc(), snap(), io, safety, publish_estop, listeners)"""
    C = ctx.cfg
    app = FastAPI(title="OMX 세컨드 배리어", docs_url=None, redoc_url=None, openapi_url=None)   # API 문서 노출 안 함
    key_path = C.path("secret.key")
    if not key_path.exists():
        key_path.write_bytes(secrets.token_bytes(32))
        os.chmod(key_path, 0o600)
    SECRET = key_path.read_bytes()

    def users():
        return C["web"]["users"]

    def pw_ok(u, p):
        stored = str(users().get(u, ""))
        if stored.startswith("sha256:"):
            salt, h = stored[7:].split(":", 1)
            return hmac.compare_digest(h, hashlib.sha256((salt + p).encode()).hexdigest())
        return bool(stored) and hmac.compare_digest(stored, p)
    loop_holder = {}
    clients = set()

    # ------------------------------------------------------------ 노드 호출
    def R(target, cmd, timeout=10.0, **args):
        try:
            return ctx.rpc(target).call(cmd, timeout=timeout, **args)
        except RemoteError as e:
            raise HTTPException(400, str(e))

    def run(target, cmd, **args):
        r = R(target, cmd, **args)
        if cmd in MOVE_CMDS:
            r = None
        return {"ok": True, "result": r if isinstance(r, (int, float, str, list, dict, type(None))) else None}

    # ------------------------------------------------------------ 인증
    def sign(user, exp):
        return hmac.new(SECRET, f"{user}:{exp}".encode(), hashlib.sha256).hexdigest()[:32]

    def check_token(tok):
        try:
            user, exp, sig = tok.split(":")
            if int(exp) < time.time() or not hmac.compare_digest(sig, sign(user, exp)):
                return None
            return user
        except Exception:
            return None

    def require_user(request: Request):
        u = check_token(request.cookies.get(COOKIE, ""))
        if not u:
            raise HTTPException(401, "로그인 필요")
        return u

    @app.post("/api/login")
    def login(body: dict = Body(...)):
        u, p = body.get("user", ""), body.get("password", "")
        if u not in users() or not pw_ok(u, p):
            raise HTTPException(401, "아이디 또는 비밀번호가 틀림")
        exp = int(time.time()) + 7 * 86400
        r = JSONResponse({"ok": True, "user": u})
        r.set_cookie(COOKIE, f"{u}:{exp}:{sign(u, exp)}", max_age=7 * 86400, httponly=True, samesite="lax")
        return r

    @app.post("/api/logout")
    def logout():
        r = JSONResponse({"ok": True})
        r.delete_cookie(COOKIE)
        return r

    @app.get("/api/me")
    def me(user=Depends(require_user)):
        return {"user": user}

    # ------------------------------------------------------------ 상태
    def full_state(live=False):
        snap = ctx.snap_live if live else ctx.snap
        sf, rb, pc, io = snap("safety"), snap("robot"), snap("perception"), snap("io")
        per = dict(pc.get("perception") or {})
        site = dict(io.get("site") or {})
        site["helmet"] = pc.get("helmet")
        return {
            "t": time.time(),
            "safety": sf,
            "robot": rb.get("robot") or {},
            "robot_backend": rb.get("robot_backend", "응답 없음"),
            "perception": per,
            "devices": io.get("devices") or {},
            "conveyor": io.get("conveyor") or {},
            "site": site,
            "cycle": rb.get("cycle") or {},
            "recording": {**(pc.get("recording") or {}), "pre_sec": C["recording"]["pre_sec"],
                          "post_sec": C["recording"]["post_sec"], "conveyor_post_sec": C["recording"]["conveyor_post_sec"],
                          "limits": C["recording"]["limits"]},
            "db": {**ctx.db.stats(), "prune": ctx.db.last_prune},
            "teach": rb.get("teach") or {"poses": {}, "sequence": [], "gripper": {}, "grips": {}},
            "door_roi": C["safety"].get("door_roi_px"),
            "safety_profiles": C["safety"]["profiles"],
            "vanish_grace": C["safety"]["vanish_grace"],
            "slow_scale": C["safety"]["slow_scale"],
            "start_speed": C["robot"].get("start_speed_ratio") or C.limits["speed_default_ratio"],
            "fall": {"sec": C["safety"].get("fall_confirm_sec", 3.0), "conf": C["safety"].get("fall_min_conf", 0.6)},
            "map": {**(pc.get("persons") or {}), "zone": C["safety"]["zone_base"]},
            "sys": ctx.sysmon.snapshot(),
            "nodes": ctx.node_status(),
        }

    @app.get("/api/persons")
    def persons(user=Depends(require_user)):
        return ctx.snap("perception").get("persons") or {"persons": []}

    @app.get("/api/state")
    def state(user=Depends(require_user)):
        ctx.state_t = time.monotonic()
        return jsonable(full_state(live=True))           # 직접 조회는 각 노드의 지금 상태 (웹소켓은 캐시 5Hz)

    # ------------------------------------------------------------ 로봇
    def guard_motion():
        try:
            g = ctx.rpc("safety").call("guard", timeout=1.0)     # 지금 값으로 검사
        except RemoteError:
            g = {"fire": ctx.safety.fire, "maintenance": ctx.safety.maintenance}
        if g["fire"]:
            raise HTTPException(409, "화재 상황 중 — 로봇 홈 대기")
        if g["maintenance"]:
            raise HTTPException(409, "정비 모드 — 로봇 동작 차단")

    @app.post("/api/robot/torque")
    def torque(body: dict = Body(...), user=Depends(require_user)):
        if body.get("on"):
            guard_motion()
        return run("robot", "torque", on=bool(body.get("on")), force=bool(body.get("force")))

    @app.post("/api/robot/park")
    def park(user=Depends(require_user)):
        guard_motion()
        return run("robot", "park")

    @app.post("/api/robot/home")
    def home(user=Depends(require_user)):
        guard_motion()
        return run("robot", "home")

    @app.post("/api/robot/goto")
    def goto(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        return run("robot", "goto", pose=body["pose"])

    @app.post("/api/robot/joints")
    def joints(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        return run("robot", "joints", q=[float(v) for v in body["q"]])

    @app.post("/api/robot/freedrive")
    def freedrive(body: dict = Body(...), user=Depends(require_user)):
        if body.get("on"):
            guard_motion()
        return run("robot", "freedrive", on=bool(body.get("on")))

    @app.post("/api/robot/align")
    def align(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        t = body.get("target", "down")
        if t not in ("down", "horizontal"):
            raise HTTPException(400, "target은 down 또는 horizontal")
        return run("robot", "align", target=t)

    @app.post("/api/robot/tcp")
    def tcp(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        return run("robot", "tcp", x=body["x"], y=body["y"], z=body["z"], pitch=body["pitch"])

    @app.post("/api/robot/gripper")
    def gripper(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        r = run("robot", "gripper", **{k: body[k] for k in ("preset", "percent", "value", "force") if k in body})
        r["result"] = None
        return r

    @app.post("/api/robot/gripper_force")
    def gripper_force(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "gripper_force", force=body["force"])

    @app.post("/api/grips/save")
    def grips_save(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "grips_save", **body)

    @app.post("/api/grips/delete")
    def grips_delete(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "grips_delete", name=body["name"])

    @app.post("/api/grips/default_force")
    def grips_default_force(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "grips_default_force", force=body["force"])

    @app.post("/api/robot/estop")
    def estop(user=Depends(require_user)):
        # 1) 빠른 길: 로봇 실행기에 바로 (안전 노드가 죽어 있어도 선다)  2) 안전 관리자 정지 래치  3) 작업 중단
        ctx.publish_estop()
        errs = []
        for target, cmd, args in (("safety", "estop", {}), ("robot", "runner_stop", {"why": "비상정지"})):
            try:
                ctx.rpc(target).call(cmd, timeout=2.0, **args)
            except RemoteError as e:
                errs.append(str(e))
        return {"ok": True, "warn": errs}

    @app.post("/api/robot/reset")
    def reset(user=Depends(require_user)):
        r = run("robot", "reset")
        log.info("reset by %s: %s", user, r.get("result"))
        return r

    @app.post("/api/robot/stop")
    def stop(user=Depends(require_user)):
        R("robot", "stop")
        return {"ok": True}

    @app.post("/api/robot/speed")
    def speed(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "speed", ratio=float(body["ratio"]))

    @app.post("/api/robot/speed_save")
    def speed_save(body: dict = Body(...), user=Depends(require_user)):
        """지금 속도를 시작 속도로 저장 — 다시 켜도 이 속도로 시작 (2026-10-12 사용자)."""
        r = float(body["ratio"])
        if not 0.01 <= r <= 1.0:
            raise HTTPException(400, "범위: 1–100 %")
        run("robot", "speed", ratio=r)
        ctx.update_config("robot", {"start_speed_ratio": round(r, 3)})
        ctx.events.emit("settings", f"{user}: 로봇 시작 속도 {r * 100:.0f}% 로 저장", level="info")
        return {"ok": True}

    # ------------------------------------------------------------ 티칭
    @app.post("/api/poses/save")
    def pose_save(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "poses_save", name=body["name"])

    @app.post("/api/poses/delete")
    def pose_del(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "poses_delete", name=body["name"])

    @app.post("/api/poses/home")
    def pose_home(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "poses_home", name=body["name"])

    @app.post("/api/poses/gripper")
    def pose_gripper(body: dict = Body(...), user=Depends(require_user)):
        if body.get("which") not in ("open", "close"):
            raise HTTPException(400, "which 는 open 또는 close")
        return run("robot", "pose_gripper", which=body["which"])

    @app.post("/api/gripper/range")
    def gripper_range(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "gripper_range", open=body.get("open"), close=body.get("close"))

    @app.post("/api/sequence")
    def sequence(body: dict = Body(...), user=Depends(require_user)):
        return run("robot", "sequence", steps=body["steps"], before_start=body.get("before_start"))

    @app.post("/api/sequence/template")
    def seq_template(user=Depends(require_user)):
        return run("robot", "sequence_template")

    @app.post("/api/cycle/start")
    def cycle_start(body: dict = Body(default={}), user=Depends(require_user)):
        guard_motion()
        return run("robot", "cycle_start", max_cycles=int(body.get("max_cycles", 0)),
                   start_at=int(body.get("start_at", 0)), section=body.get("section", "main"), user=user)

    @app.post("/api/cycle/step")
    def cycle_step(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        return run("robot", "cycle_step", index=int(body["index"]), section=body.get("section", "main"))

    @app.post("/api/cycle/stop")
    def cycle_stop(user=Depends(require_user)):
        R("robot", "cycle_stop", user=user)
        return {"ok": True}

    @app.post("/api/cycle/grip_check")
    def grip_check(body: dict = Body(...), user=Depends(require_user)):
        R("robot", "grip_check", on=bool(body.get("on")))
        return {"ok": True}

    # ------------------------------------------------------------ 안전
    @app.post("/api/safety/restart")
    def restart(body: dict = Body(default={}), user=Depends(require_user)):
        """한 번에 재가동: 비상정지·고장 해제 → (확인 시) 사라짐 확인 → 토크 켜기 → 안전 조건 재검사 → 재가동.
        사람 근접·조명·장치 같은 안전 조건은 그대로 다시 검사하며, 하나라도 걸리면 재가동하지 않는다."""
        steps = []
        try:
            rs = ctx.rpc("robot").call("status", timeout=3.0)
            if rs["estop"] or rs["fault"]:
                try:
                    r = ctx.rpc("robot").call("reset", timeout=15.0)
                    steps.append("비상정지·고장 해제" + (f" ({r})" if r and r != "해제 완료" else ""))
                except RemoteError as e:
                    return {"ok": False, "problems": [f"고장 해제 실패: {e}"], "steps": steps}
            sf = ctx.snap("safety")
            if body.get("clear_vanish") and (sf.get("vanish_unresolved") or sf.get("vanish_pending")):
                ctx.rpc("safety").call("clear_vanish", user=user)
                steps.append("사라짐 확인")
            rs = ctx.rpc("robot").call("status", timeout=3.0)
            if not rs["torque_on"] and not rs["freedrive"]:
                try:
                    ctx.rpc("robot").call("torque", on=True)
                    steps.append("토크 켜기")
                except RemoteError as e:
                    return {"ok": False, "problems": [f"토크 켜기 실패: {e}"], "steps": steps}
            if rs["freedrive"]:
                return {"ok": False, "problems": ["직접 교시 중 — 먼저 직접 교시를 끝내세요"], "steps": steps}
            time.sleep(0.25)                      # 안전 관리자가 새 로봇 상태를 한 번 반영하도록 (50Hz → 20Hz)
            r = ctx.rpc("safety").call("restart", user=user)
            return {"ok": r["ok"], "problems": r["problems"], "steps": steps}
        except RemoteError as e:
            return {"ok": False, "problems": [str(e)], "steps": steps}

    @app.post("/api/safety/vanish_clear")
    def vanish_clear(user=Depends(require_user)):
        return {"ok": True, "cleared": R("safety", "clear_vanish", user=user)}

    @app.post("/api/safety/maintenance")
    def maintenance(body: dict = Body(...), user=Depends(require_user)):
        if body.get("on"):
            R("robot", "runner_stop", why="정비 모드")
        R("safety", "set_maintenance", on=bool(body.get("on")))
        return {"ok": True}

    @app.post("/api/safety/teach_mode")
    def teach_mode(body: dict = Body(...), user=Depends(require_user)):
        on = bool(body.get("on"))
        if on:
            R("robot", "runner_stop", why="티칭 모드")
        return run("safety", "set_teach_mode", on=on, user=user)

    @app.post("/api/safety/fire_end")
    def fire_end(user=Depends(require_user)):
        try:
            fire = ctx.rpc("io").call("status", timeout=1.0)["fire"]
        except RemoteError:
            fire = ctx.io.fire()
        if fire:
            raise HTTPException(409, "화재 센서가 아직 감지 중 — 센서가 꺼진 뒤 상황 종료")
        return {"ok": R("safety", "end_fire", user=user)}

    @app.post("/api/safety/conveyor_clear")
    def conveyor_clear(user=Depends(require_user)):
        R("safety", "conveyor_clear")
        return {"ok": True}

    @app.post("/api/safety/profile")
    def profile(body: dict = Body(...), user=Depends(require_user)):
        p = body.get("profile")
        if p not in C["safety"]["profiles"]:
            raise HTTPException(400, "없는 프로필")
        ctx.update_config("safety", {"profile": p})
        t = C["safety"]["profiles"][p]
        ctx.events.emit("settings", f"{user}: 거리 기준을 '{PROFILE_NAME.get(p, p)}'로 바꿈 — 감속 {t['slow']} / 정지 {t['stop']} / "
                        f"재가동 허용 {t['clear']} m", level="info")
        return {"ok": True}

    # ------------------------------------------------------------ 가중치
    @app.get("/api/models")
    def models(user=Depends(require_user)):
        return R("perception", "models")

    @app.post("/api/models/select")
    def model_select(body: dict = Body(...), user=Depends(require_user)):
        # 인식 노드가 불러오기에 성공한 뒤에만 설정에 저장한다
        return run("perception", "model_select", timeout=120.0, name=body["name"])

    @app.post("/api/models/upload")
    async def model_upload(file: UploadFile = File(...), user=Depends(require_user)):
        data = await file.read()
        name = Path(file.filename or "").name
        if Path(name).suffix not in EXTS:
            raise HTTPException(400, "지원 형식: .pt .engine .onnx")
        d = C.path("models")
        d.mkdir(exist_ok=True)
        tmp = d / (name + ".part")
        tmp.write_bytes(data)
        tmp.replace(d / name)
        return {"ok": True, "name": name}

    # ------------------------------------------------------------ 장치·설정
    @app.post("/api/devices/virtual")
    def dev_virtual(body: dict = Body(...), user=Depends(require_user)):
        try:
            ctx.rpc("io").call("set_virtual", **body)
        except RemoteError as e:
            raise HTTPException(409 if "가상 장치" in str(e) else 400, str(e))
        return {"ok": True}

    @app.post("/api/devices/door_open")
    def door_open(user=Depends(require_user)):
        R("io", "door_open", user=user)
        return {"ok": True}

    @app.get("/api/helmet/frame.jpg")
    def helmet_frame(user=Depends(require_user)):
        try:
            b64 = ctx.rpc("perception").call("helmet_frame", timeout=3.0)
        except RemoteError as e:
            raise HTTPException(404, str(e))
        return Response(base64.b64decode(b64), media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.post("/api/settings/door")
    def set_door(body: dict = Body(...), user=Depends(require_user)):
        v = float(body["close_sec"])
        if not 1 <= v <= 60:
            raise HTTPException(400, "범위: 1–60 s")
        ctx.update_config("door", {"close_sec": v})
        return {"ok": True}

    @app.post("/api/settings/recording")
    def set_recording(body: dict = Body(...), user=Depends(require_user)):
        lim = C["recording"]["limits"]
        vals = {}
        for k in ("pre_sec", "post_sec", "conveyor_post_sec"):
            if k in body:
                v = int(body[k])
                mx = lim["pre_max"] if k == "pre_sec" else lim["post_max"]
                if not 0 <= v <= mx:
                    raise HTTPException(400, f"{k} 범위: 0–{mx} s")
                vals[k] = v
        if "pre_sec" in vals and vals["pre_sec"] > C["recording"]["keep_sec"] - C["recording"]["segment_sec"]:
            raise HTTPException(400, "사전 녹화는 순환 버퍼(60 s)보다 짧아야 함")
        ctx.update_config("recording", vals)
        return {"ok": True}

    @app.post("/api/settings/door_roi")
    def set_door_roi(body: dict = Body(...), user=Depends(require_user)):
        roi = body.get("roi")
        if roi is not None:
            x1, y1, x2, y2 = [int(v) for v in roi]
            roi = [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]
        ctx.update_config("safety", {"door_roi_px": roi})
        return {"ok": True, "roi": roi}

    @app.post("/api/settings/password")
    def set_password(body: dict = Body(...), user=Depends(require_user)):
        if not pw_ok(user, body.get("old", "")):
            raise HTTPException(400, "현재 비밀번호가 틀림")
        new = body.get("new", "")
        if len(new) < 6:
            raise HTTPException(400, "새 비밀번호는 6자 이상 필요")
        salt = secrets.token_hex(8)
        ctx.update_config("web", {"users": {**users(), user: f"sha256:{salt}:{hashlib.sha256((salt + new).encode()).hexdigest()}"}})
        return {"ok": True}

    @app.post("/api/settings/thresholds")
    def set_thresholds(body: dict = Body(...), user=Depends(require_user)):
        prof = body.get("profile")
        if prof not in C["safety"]["profiles"]:
            raise HTTPException(400, "없는 프로필")
        slow, stop, clear = float(body["slow"]), float(body["stop"]), float(body["clear"])
        smin = float((C["safety"].get("profile_min_stop") or {}).get(prof, 0.03))
        if not (smin <= stop < slow < clear <= 5.0):
            raise HTTPException(400, f"조건: {smin:g} ≤ 정지 < 감속 < 해제 ≤ 5.0 m ('{PROFILE_NAME.get(prof, prof)}' 최소 정지 거리 {smin:g} m)")
        profiles = {k: dict(v) for k, v in C["safety"]["profiles"].items()}
        profiles[prof] = {"slow": slow, "stop": stop, "clear": clear}
        vals = {"profiles": profiles}
        if "vanish_grace" in body:
            g = float(body["vanish_grace"])
            if not 0.2 <= g <= 5.0:
                raise HTTPException(400, "사라짐 유예 범위: 0.2–5 s")
            vals["vanish_grace"] = g
        ctx.update_config("safety", vals)
        active = C["safety"]["profile"]
        ctx.events.emit("settings", f"{user}: '{PROFILE_NAME.get(prof, prof)}' 기준 저장 — 감속 {slow} / 정지 {stop} / 재가동 허용 {clear} m"
                        + ("" if prof == active else f" (지금 적용 중인 기준은 '{PROFILE_NAME.get(active, active)}')"), level="info")
        return {"ok": True}

    @app.post("/api/settings/guard")
    def set_guard(body: dict = Body(...), user=Depends(require_user)):
        m = float(body.get("meters", 0))
        if not 0 <= m <= 5:
            raise HTTPException(400, "범위: 0–5 m (0 = 끔)")
        ctx.update_config("safety", {"guard_person_m": m})
        R("safety", "set_guard", meters=m)
        return {"ok": True}

    @app.post("/api/settings/fall")
    def set_fall(body: dict = Body(...), user=Depends(require_user)):
        """낙상 판단: 넘어진 사람이 확신도 conf_pct% 이상으로 sec 초 넘게 보이면 낙상 (2026-10-12 사용자). 바로 적용."""
        sec, pct = float(body.get("sec", 3.0)), float(body.get("conf_pct", 60))
        if not 0.5 <= sec <= 30:
            raise HTTPException(400, "감지 시간 범위: 0.5–30 초")
        if not 30 <= pct <= 95:
            raise HTTPException(400, "확신도 범위: 30–95 %")
        ctx.update_config("safety", {"fall_confirm_sec": round(sec, 1), "fall_min_conf": round(pct / 100, 2)})
        ctx.events.emit("settings", f"{user}: 낙상 감지 기준 변경 — 확신도 {pct:.0f}% 이상 {sec:g}초", level="info")
        return {"ok": True}

    @app.post("/api/settings/slow_scale")
    def set_slow_scale(body: dict = Body(...), user=Depends(require_user)):
        """감속 구간에서 허용하는 로봇 속도 비율 (%). 저장하면 바로 적용 (2026-10-12 사용자)."""
        pct = float(body.get("pct", 35))
        if not 10 <= pct <= 80:
            raise HTTPException(400, "범위: 10–80 %")
        ctx.update_config("safety", {"slow_scale": round(pct / 100, 2)})
        ctx.events.emit("settings", f"{user}: 감속 속도 {pct:.0f}% 로 변경", level="info")
        return {"ok": True}

    @app.get("/api/logs")
    def logs(lines: int = 200, user=Depends(require_user)):
        """노드별 기록 파일의 끝부분을 시각 순으로 합친다."""
        d = C.path("logs")
        out = []
        for p in sorted(d.glob("*.log")) if d.exists() else []:
            with open(p, "rb") as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - 100_000))
                out += f.read().decode("utf-8", "replace").splitlines()[1:]
        out.sort(key=lambda s: s[:23])
        return {"lines": out[-min(lines, 1000):]}

    @app.post("/api/system/restart")
    def sys_restart(user=Depends(require_user)):
        return _shutdown(restart=True, user=user)

    @app.post("/api/system/shutdown")
    def sys_shutdown(user=Depends(require_user)):
        return _shutdown(restart=False, user=user)

    def _shutdown(restart, user):
        """전체 노드 재시작/종료. 로봇이 움직이는 중이면 거부 (토크는 켠 채 자리 유지된다)."""
        if ctx.snap("robot").get("busy"):
            raise HTTPException(409, "로봇 동작 중 — 먼저 정지 필요")
        ctx.events.emit("system", f"프로그램 {'재시작' if restart else '종료'} ({user})", level="info")

        def later():
            time.sleep(0.8)
            if os.environ.get("INVOCATION_ID"):          # systemd 사용자 서비스로 도는 중
                unit = os.environ.get("OMX_SYSTEMD_UNIT", "omx.target")   # omx-launch.service(부팅 자동 실행) 또는 노드별 omx.target
                subprocess.Popen(["systemctl", "--user", "restart" if restart else "stop", unit],
                                 start_new_session=True)
                return
            if restart:                                  # ros2 launch(respawn) 로 도는 중: 각 노드를 끝내면 다시 띄운다
                for t in ("perception", "robot", "io", "safety"):
                    try:
                        ctx.rpc(t).call("shutdown", timeout=2.0)
                    except RemoteError:
                        pass
                os._exit(3)
            os.kill(os.getppid(), signal.SIGINT)          # launch 를 끝낸다 → 모든 노드 종료
        threading.Thread(target=later, daemon=True).start()
        return {"ok": True}

    @app.post("/api/settings/detection")
    def set_detection(body: dict = Body(...), user=Depends(require_user)):
        v = float(body["conf"])
        if not 0.10 <= v <= 0.95:
            raise HTTPException(400, "최소 확신도는 10~95%")
        ctx.update_config("perception", {"conf": v})
        return {"ok": True}

    @app.post("/api/settings/conveyor")
    def set_conveyor(body: dict = Body(...), user=Depends(require_user)):
        v = float(body["travel_timeout"])
        if not 0.5 <= v <= 120:
            raise HTTPException(400, "범위: 0.5–120 s")
        ctx.update_config("conveyor", {"travel_timeout": v})
        return {"ok": True}

    # ------------------------------------------------------------ 이벤트·영상
    @app.get("/api/events")
    def events(limit: int = 50, before: int = None, cat: str = None, user=Depends(require_user)):
        if cat and cat not in EVENT_CATS:
            raise HTTPException(400, "분류는 " + ", ".join(EVENT_CATS))
        types = EVENT_CATS[cat] if cat else EVENT_SHOWN       # '전체' = 화면에 보이는 분류만 (작업·운영·설정 제외)
        return {"events": ctx.db.list(min(limit, 500), before, types=types)}

    @app.get("/api/events/summary")
    def events_summary(days: float = 7, user=Depends(require_user)):
        """대시보드: 기간 안 사고 종류별 수 + 이벤트 분류별 미확인 수."""
        since = time.time() - max(0.0, min(days, 3650)) * 86400 if days > 0 else None
        inc = ctx.db.type_counts(INCIDENTS, since)
        allc = ctx.db.type_counts(sorted(STORED))
        return {"days": days, "by_type": {t: v["n"] for t, v in inc.items()},
                "total": sum(v["n"] for v in inc.values()),
                "cat_unacked": {c: sum(allc.get(t, {}).get("unacked", 0) for t in ts) for c, ts in EVENT_CATS.items()},
                "shown_unacked": sum(allc.get(t, {}).get("unacked", 0) for t in EVENT_SHOWN)}

    @app.get("/api/stats")
    def stats(days: int = 7, user=Depends(require_user)):
        """통계 화면: 일별 작업량·평균 사이클 시간·시간대별 작업량·이벤트 발생률 (오늘 포함 days 일)."""
        days = max(1, min(int(days), 365))
        lt = time.localtime()
        since = time.mktime((lt.tm_year, lt.tm_mon, lt.tm_mday, 0, 0, 0, 0, 0, -1)) - 86400 * (days - 1)
        rows = ctx.db.rows_since(sorted(STORED | STATS_ONLY), since)
        return dash_stats.compute(rows, days)

    CLIP_CATS = {"human": ["human_stop"], "vanish": ["vanish"], "fall": ["person_fall"], "conveyor": ["conveyor_jam"], "fire": ["fire"]}

    @app.get("/api/clips")
    def clips(cat: str = "human", limit: int = 60, before: int = None, day: str = None, user=Depends(require_user)):
        if cat not in (*CLIP_CATS, "other"):
            raise HTTPException(400, "분류는 " + ", ".join((*CLIP_CATS, "other")))
        t_range = None
        if day:
            try:
                t0 = time.mktime(time.strptime(day, "%Y-%m-%d"))
            except ValueError:
                raise HTTPException(400, "날짜 형식은 YYYY-MM-DD")
            t_range = (t0, t0 + 86400)
        allcat = [t for v in CLIP_CATS.values() for t in v]
        rows = ctx.db.list_clips(types=CLIP_CATS.get(cat), exclude=allcat if cat == "other" else None,
                                 limit=min(limit, 300), before=before, t_range=t_range)
        by_type = ctx.db.clip_counts()
        counts = {}
        for c in (*CLIP_CATS, "other"):
            ts = CLIP_CATS[c] if c != "other" else [t for t in by_type if t not in allcat]
            counts[c] = {"n": sum(by_type.get(t, {}).get("n", 0) for t in ts),
                         "unacked": sum(by_type.get(t, {}).get("unacked", 0) or 0 for t in ts)}
        return {"clips": rows, "counts": counts}

    @app.post("/api/events/{eid}/ack")
    def ack(eid: int, user=Depends(require_user)):
        ctx.db.ack(eid, user)
        return {"ok": True}

    @app.post("/api/events/ack_all")
    def ack_all(user=Depends(require_user)):
        ctx.db.ack_all(user)
        return {"ok": True}

    @app.get("/clips/{name}")
    def clip(name: str, dl: int = 0, user=Depends(require_user)):
        p = (ctx.clip_dir / Path(name).name)
        if not p.exists():
            raise HTTPException(404)
        if dl:                                   # 영상 메뉴의 "저장" — 누가 어떤 영상을 가져갔는지 남긴다
            ctx.events.emit("clip_save", f"{user}: 영상 저장 {p.name}")
            return FileResponse(p, media_type="video/mp4", filename=p.name)
        return FileResponse(p, media_type="video/mp4")

    # ------------------------------------------------------------ 보정
    @app.post("/api/calib/sample")
    def calib_sample(body: dict = Body(default={}), user=Depends(require_user)):
        return run("perception", "calib_sample", u=body.get("u"), v=body.get("v"))


    def _work_yaws(teach):
        """프로그램(시작 전 + 반복)이 쓰는 자세들의 베이스 방향 (자동 보정이 작업 자리도 돌게)."""
        poses = (teach or {}).get("poses", {})
        names = [s.get("pose") for s in (teach or {}).get("before_start", []) + (teach or {}).get("sequence", [])
                 if s.get("type") in ("move", "linear")]
        return [float(poses[n]["q"][0]) for n in dict.fromkeys(names) if n in poses]

    @app.post("/api/calib/auto")
    def calib_auto(user=Depends(require_user)):
        guard_motion()
        return run("perception", "calib_auto", work_yaws=_work_yaws(ctx.snap("robot").get("teach")))

    @app.post("/api/calib/solve")
    def calib_solve(user=Depends(require_user)):
        return run("perception", "calib_solve")

    @app.post("/api/calib/reset")
    def calib_reset(user=Depends(require_user)):
        return run("perception", "calib_reset")

    @app.post("/api/calib/delete")
    def calib_delete(user=Depends(require_user)):
        r = run("perception", "calib_delete")
        ctx.events.emit("calib", f"{user}이(가) 보정 결과 삭제 — 거리 판정이 카메라 기준으로 바뀜", level="warn")
        return r

    # ------------------------------------------------------------ 영상 스트림
    @app.get("/video.mjpg")
    def video(user=Depends(require_user)):
        """인식 노드가 /dev/shm 에 쓴 오버레이 JPEG 를 보낸다 (새 파일일 때만). 보는 사람 수는 인식 노드에 알린다."""
        path = ctx.live_path

        def gen():
            ctx.viewer_add(1)
            last = None
            try:
                while True:
                    try:
                        st = path.stat()
                        key = (st.st_mtime_ns, st.st_ino)
                    except FileNotFoundError:
                        key = None
                    if key is None or key == last or time.time() - st.st_mtime > 1.5:
                        time.sleep(0.03)
                        continue
                    last = key
                    try:
                        buf = path.read_bytes()
                    except FileNotFoundError:
                        continue
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n"
            finally:
                ctx.viewer_add(-1)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    # ------------------------------------------------------------ 웹소켓
    def offer(q, data):
        try:
            q.put_nowait(data)
        except asyncio.QueueFull:                 # 느린 접속은 밀린 상태를 버린다 (다음 상태가 곧 온다)
            pass

    def broadcast(msg):
        loop = loop_holder.get("loop")
        if loop is None:
            return
        data = json.dumps(jsonable(msg), ensure_ascii=False)
        for q in list(clients):
            loop.call_soon_threadsafe(offer, q, data)

    ctx.listeners.append(broadcast)

    async def state_pump():
        """상태는 0.2초마다 한 번만 만들어 모든 접속에 같이 보낸다 (예전: 접속마다 따로 만들었다)."""
        while True:
            await asyncio.sleep(0.2)
            if not clients:
                continue
            try:
                st = await asyncio.get_running_loop().run_in_executor(None, full_state)
                data = json.dumps({"kind": "state", "data": jsonable(st)}, ensure_ascii=False)
                for q in list(clients):
                    offer(q, data)
            except Exception:
                log.exception("state pump")

    @app.on_event("startup")
    async def _startup():
        loop_holder["loop"] = asyncio.get_running_loop()
        asyncio.create_task(state_pump())

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        if not check_token(sock.cookies.get(COOKIE, "")):
            await sock.close(code=4401)
            return
        await sock.accept()
        q = asyncio.Queue(maxsize=50)
        clients.add(q)
        ctx.client_add(1)

        async def sender():
            while True:
                await sock.send_text(await q.get())

        task = asyncio.create_task(sender())
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                if "jog" in msg:
                    if ctx.safety.fire or ctx.safety.maintenance:      # 조그는 초당 여러 번 → 캐시(20Hz 갱신)
                        continue
                    try:
                        await asyncio.get_running_loop().run_in_executor(
                            None, lambda: ctx.rpc("robot").call("jog", timeout=1.0, jog=msg["jog"],
                                                                mode=msg.get("mode", "cart")))
                    except RemoteError as e:
                        await sock.send_text(json.dumps({"kind": "error", "message": str(e)}, ensure_ascii=False))
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            task.cancel()
            clients.discard(q)
            ctx.client_add(-1)

    # ------------------------------------------------------------ 정적 파일
    def asset_ver():
        return format(int(max((STATIC / f).stat().st_mtime for f in ("app.js", "style.css"))), "x")

    @app.get("/")
    def index():
        v = asset_ver()
        html = (STATIC / "index.html").read_text().replace('"/static/app.js"', f'"/static/app.js?v={v}"') \
            .replace('"/static/style.css"', f'"/static/style.css?v={v}"')
        return HTMLResponse(html, headers={"Cache-Control": "no-cache"})

    @app.get("/sw.js")
    def sw():
        js = (STATIC / "sw.js").read_text().replace('"omx-v2"', f'"omx-{asset_ver()}"')
        return Response(js, media_type="application/javascript", headers={"Cache-Control": "no-cache"})

    @app.middleware("http")
    async def static_revalidate(request, call_next):
        r = await call_next(request)
        if request.url.path.startswith("/static/"):
            r.headers["Cache-Control"] = "no-cache"
        return r

    @app.get("/manifest.webmanifest")
    def manifest():
        return FileResponse(STATIC / "manifest.webmanifest", media_type="application/manifest+json")

    app.mount("/static", StaticFiles(directory=STATIC), name="static")
    return app
