"""웹 제어판 (FastAPI). 같은 Wi-Fi의 PC·휴대폰 브라우저에서 접속, PWA로 설치 가능.

- 로그인(쿠키 세션, HMAC 서명) 후 모든 API 사용
- /ws 웹소켓: 상태 5Hz 방송 + 이벤트 즉시 알림 + 키보드 조그 수신
- /video.mjpg: 사람 인식 오버레이 영상 (보는 사람이 있을 때만 인코딩)
"""
import asyncio
import hashlib
import hmac
import json
import logging
import secrets
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from fastapi import Body, Depends, FastAPI, File, HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from ..robot import kinematics as K
from ..robot.teach import grip_target
from ..robot.task import pickplace_template

log = logging.getLogger("web")
STATIC = Path(__file__).parent / "static"
COOKIE = "omx_session"
PROFILE_NAME = {"figure": "피규어 시연", "human": "실제 사람"}


def create_app(ctx):
    C = ctx.cfg
    app = FastAPI(title="OMX 세컨드 배리어")
    key_path = C.path("secret.key")
    if not key_path.exists():
        key_path.write_bytes(secrets.token_bytes(32))
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
    def full_state():
        ex, sm = ctx.ex, ctx.safety
        per = ctx.per
        return {
            "t": time.time(),
            "safety": sm.snapshot(),
            "robot": ex.snapshot(),
            "robot_backend": ctx.robot_backend + (f" {ctx.ex.bus.port_name.split('/')[-1][:24]}" if hasattr(ctx.ex.bus, "port_name") else ""),
            "perception": {
                "model": ctx.det.model_name, "model_status": ctx.det.status, "convert": ctx.det.convert_status,
                "infer_ms": round(ctx.det.infer_ms, 1), "fps": round(per.fps, 1),
                "cam_fps": round(ctx.cam.fps_meas, 1), "cam_error": ctx.cam.error,
                "persons": len(per.latest[1].persons) if per.latest else 0,
                "robot_rejects": getattr(per, "robot_rejects", 0),
                "conf": ctx.det.conf,
                "calib": {**ctx.calib.snapshot(), "auto": ctx.autocalib.status, "auto_running": ctx.autocalib.running,
                          "valid": per.calib_valid, "fit": None if per.calib_fit is None else round(per.calib_fit, 2)},
            },
            "devices": ctx.io.snapshot(),
            "conveyor": ctx.conveyor.snapshot(),
            "site": ctx.site.snapshot(),
            "cycle": ctx.runner.snapshot(),
            "recording": {**ctx.rec.status(), "pre_sec": C["recording"]["pre_sec"],
                          "post_sec": C["recording"]["post_sec"], "conveyor_post_sec": C["recording"]["conveyor_post_sec"],
                          "limits": C["recording"]["limits"]},
            "db": {**ctx.db.stats(), "prune": ctx.db.last_prune},
            "teach": ctx.teach.snapshot(),
            "door_roi": C["safety"].get("door_roi_px"),
            "safety_profiles": C["safety"]["profiles"],
            "vanish_grace": C["safety"]["vanish_grace"],
            "slow_scale": C["safety"]["slow_scale"],
            "sys": ctx.sysmon.snapshot(),
        }

    @app.get("/api/persons")
    def persons(user=Depends(require_user)):
        lat = ctx.per.latest
        if lat is None:
            return {"persons": []}
        return {"t": lat[1].t, "persons": [{"box": [round(v) for v in p.box], "conf": round(p.conf, 3),
                                            "dist": None if not np.isfinite(p.dist) else round(p.dist, 3),
                                            "cam_dist": None if not np.isfinite(p.cam_dist) else round(p.cam_dist, 3),
                                            "npts": p.npts, "in_zone": p.in_zone,
                                            "overlap": round(p.overlap, 2), "near": round(p.near, 2)} for p in lat[1].persons],
                "robot_rejects": getattr(ctx.per, "robot_rejects", 0)}

    @app.get("/api/state")
    def state(user=Depends(require_user)):
        return full_state()

    # ------------------------------------------------------------ 로봇
    def guard_motion():
        if ctx.safety.fire:
            raise HTTPException(409, "화재 상황 중 — 로봇 홈 대기")
        if ctx.safety.maintenance:
            raise HTTPException(409, "정비 모드 — 로봇 동작 차단")

    def run(fn):
        try:
            r = fn()
            return {"ok": True, "result": r if isinstance(r, (int, float, str, list, dict, type(None))) else None}
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(400, str(e))

    @app.post("/api/robot/torque")
    def torque(body: dict = Body(...), user=Depends(require_user)):
        if body.get("on"):
            guard_motion()
        return run(lambda: ctx.ex.set_torque(bool(body.get("on")), force=bool(body.get("force"))))

    @app.post("/api/robot/park")
    def park(user=Depends(require_user)):
        """휴식 자세로 천천히 이동 후 토크 끄기 (떨어지지 않는지 확인)."""
        guard_motion()
        ex = ctx.ex

        def job():
            q = np.array(list(ex.cfg["rest_pose"]) + [ex.current_q()[5]])
            q[0] = ex.current_q()[0]          # 베이스는 돌리지 않는다 (접힌 채로 제자리)
            m = ex.move_joints(q, "park")
            if m.wait(120) == "done":
                time.sleep(0.8)
                before = ex.state.q.copy()
                ex.set_torque(False)
                time.sleep(2.0)
                if np.abs(ex.state.q[:5] - before[:5]).max() > 0.1:
                    ex.set_torque(True)
                    ctx.hub.emit("robot", "토크를 끄자 팔이 처짐 — 토크 다시 켬", level="warn")
                else:
                    ctx.hub.emit("robot", "휴식 자세로 이동 후 토크 끔", level="info")
        def start():
            ex._require_ready()
            threading.Thread(target=job, daemon=True).start()
        return run(start)

    @app.post("/api/robot/home")
    def home(user=Depends(require_user)):
        guard_motion()
        q = ctx.teach.home_q()
        if q is None:
            q = list(ctx.ex.cfg["rest_pose"]) + [ctx.ex.current_q()[5]]
            q[0] = ctx.ex.current_q()[0]
        q = np.array(q, float)
        q[5] = ctx.ex.current_q()[5]
        return run(lambda: ctx.ex.move_joints(q, "home") and None)

    @app.post("/api/robot/goto")
    def goto(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        q = np.array(ctx.teach.pose_q(body["pose"]), float)
        q[5] = ctx.ex.current_q()[5]                 # 자세는 팔만 — 그리퍼는 그리퍼 명령으로만
        return run(lambda: ctx.ex.move_joints(q, f"goto {body['pose']}") and None)

    @app.post("/api/robot/joints")
    def joints(body: dict = Body(...), user=Depends(require_user)):
        """관절 각도(rad) 직접 이동. q: 5개(팔) 또는 6개(그리퍼 포함). 한계·경로 검사는 실행기가 한다."""
        guard_motion()
        q = [float(v) for v in body["q"]]
        cur = ctx.ex.current_q()
        tgt = np.array(q + [cur[5]] if len(q) == 5 else q, float)
        return run(lambda: ctx.ex.move_joints(tgt, "joints") and None)

    @app.post("/api/robot/freedrive")
    def freedrive(body: dict = Body(...), user=Depends(require_user)):
        if body.get("on"):
            guard_motion()
            ctx.runner.stop("직접 교시")
        return run(lambda: ctx.ex.set_freedrive(bool(body.get("on"))))

    @app.post("/api/robot/align")
    def align(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        t = body.get("target", "down")
        if t not in ("down", "horizontal"):
            raise HTTPException(400, "target은 down 또는 horizontal")
        return run(lambda: ctx.ex.align(t) and None)

    @app.post("/api/robot/tcp")
    def tcp(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        return run(lambda: ctx.ex.move_linear(body["x"], body["y"], body["z"], body["pitch"], name="tcp") and None)

    @app.post("/api/robot/gripper")
    def gripper(body: dict = Body(...), user=Depends(require_user)):
        guard_motion()
        # value: open/close, percent: 열림 정도 %, preset: 저장한 그리퍼 동작 이름, force: 힘 %(없으면 기본/저장값)
        step = {"preset": body["preset"]} if body.get("preset") else \
            {"value": body["percent"] if "percent" in body else body.get("value", "open")}
        if body.get("force") is not None:
            step["force"] = body["force"]

        def go():
            target, force, _ = ctx.teach.grip_action(step)
            ctx.ex.set_gripper_force(force)
            ctx.ex.move_gripper(target)
        return run(go)

    @app.post("/api/robot/gripper_force")
    def gripper_force(body: dict = Body(...), user=Depends(require_user)):
        """그리퍼 힘만 바꾼다 (잡고 있는 물체를 더/덜 조임)."""
        return run(lambda: ctx.ex.set_gripper_force(body["force"]))

    @app.post("/api/grips/save")
    def grips_save(body: dict = Body(...), user=Depends(require_user)):
        """그리퍼 동작 저장: 이름 + 위치(지금 그리퍼 위치, 또는 percent 로 지정) + 힘 %."""
        g = ctx.teach.snapshot()["gripper"]
        pos = grip_target(g, body["percent"]) if body.get("percent") is not None else float(ctx.ex.state.q[5])
        return run(lambda: ctx.teach.save_grip(body["name"], pos, body.get("force", g.get("force", 50))))

    @app.post("/api/grips/delete")
    def grips_delete(body: dict = Body(...), user=Depends(require_user)):
        return run(lambda: ctx.teach.delete_grip(body["name"]))

    @app.post("/api/grips/default_force")
    def grips_default_force(body: dict = Body(...), user=Depends(require_user)):
        """열기·닫기(기본 동작)에 쓰는 힘 %."""
        def f():
            with ctx.teach.lock:
                ctx.teach.data["gripper"]["force"] = int(min(100, max(5, float(body["force"]))))
                ctx.teach._save()
        return run(f)

    @app.post("/api/robot/estop")
    def estop(user=Depends(require_user)):
        ctx.runner.stop("비상정지")
        ctx.safety.emergency_stop()
        return {"ok": True}

    @app.post("/api/robot/reset")
    def reset(user=Depends(require_user)):
        r = run(ctx.ex.reset)
        log.info("reset by %s: %s", user, r.get("result"))
        return r

    @app.post("/api/robot/stop")
    def stop(user=Depends(require_user)):
        ctx.runner.stop()
        ctx.ex.stop_motion()
        return {"ok": True}

    @app.post("/api/robot/speed")
    def speed(body: dict = Body(...), user=Depends(require_user)):
        return run(lambda: ctx.ex.set_speed(float(body["ratio"])))

    # ------------------------------------------------------------ 티칭
    @app.post("/api/poses/save")
    def pose_save(body: dict = Body(...), user=Depends(require_user)):
        return run(lambda: ctx.teach.save_pose(body["name"], ctx.ex.state.q))

    @app.post("/api/poses/delete")
    def pose_del(body: dict = Body(...), user=Depends(require_user)):
        return run(lambda: ctx.teach.delete_pose(body["name"]))

    @app.post("/api/poses/home")
    def pose_home(body: dict = Body(...), user=Depends(require_user)):
        return run(lambda: ctx.teach.set_home(body["name"]))

    GRIP_MIN_SPAN = 0.15        # 열림·닫힘 끝이 이보다 가까우면 열기·닫기가 사실상 움직이지 않는다

    def save_grip_range(open_=None, close=None):
        """그리퍼 가동 범위(열림 끝·닫힘 끝, rad) 저장. 그리퍼 기구 한계 안으로 맞추고, 둘 사이 0.15 rad 이상."""
        lo, hi = C.limits["gripper_limits"]
        with ctx.teach.lock:
            g = ctx.teach.data["gripper"]
            valid_before = abs(g["open"] - g["close"]) >= GRIP_MIN_SPAN
            # 기구 한계 밖 값은 한계로 맞춘다 (끝까지 닫으면 실제로 -0.03 처럼 0 아래로 읽힌다 — 실행기는 한계 밖 목표를 거부)
            o = g["open"] if open_ is None else round(min(hi, max(lo, float(open_))), 3)
            c = g["close"] if close is None else round(min(hi, max(lo, float(close))), 3)
            # 한쪽만 바꿀 때: 기존 범위가 이미 잘못돼 있으면(예: 둘 다 0.054) 먼저 저장하는 쪽은 검사하지 않는다
            # — 그래야 '닫힘 끝'부터 저장해도 막히지 않는다. 다른 쪽을 저장할 때 검사한다.
            one_side = open_ is None or close is None
            if abs(o - c) < GRIP_MIN_SPAN and not (one_side and not valid_before):
                raise ValueError(f"열림 끝({o:.3f})과 닫힘 끝({c:.3f})이 너무 가깝습니다 — {GRIP_MIN_SPAN} rad 이상 떨어져야 "
                                 "열기·닫기가 움직입니다. 그리퍼를 끝까지 연(닫은) 뒤 다시 저장하세요")
            g["open"], g["close"] = o, c
            ctx.teach._save()
        log.info("그리퍼 가동 범위: 닫힘 %.3f ~ 열림 %.3f rad", c, o)

    @app.post("/api/poses/gripper")
    def pose_gripper(body: dict = Body(...), user=Depends(require_user)):
        """지금 그리퍼 위치를 열림 끝(which=open) 또는 닫힘 끝(which=close)으로."""
        which = body["which"]
        if which not in ("open", "close"):
            raise HTTPException(400, "which 는 open 또는 close")
        v = float(ctx.ex.state.q[5])
        return run(lambda: save_grip_range(**({"open_": v} if which == "open" else {"close": v})))

    @app.post("/api/gripper/range")
    def gripper_range(body: dict = Body(...), user=Depends(require_user)):
        """가동 범위 직접 입력 (rad)."""
        return run(lambda: save_grip_range(body.get("open"), body.get("close")))

    @app.post("/api/sequence")
    def sequence(body: dict = Body(...), user=Depends(require_user)):
        return run(lambda: ctx.teach.set_sequence(body["steps"], body.get("before_start")))

    @app.post("/api/sequence/template")
    def seq_template(user=Depends(require_user)):
        return run(lambda: ctx.teach.set_sequence(pickplace_template(ctx.teach.snapshot()["poses"])))

    @app.post("/api/cycle/start")
    def cycle_start(body: dict = Body(default={}), user=Depends(require_user)):
        guard_motion()
        at = int(body.get("start_at", 0))
        sec = "before" if body.get("section") == "before" else "main"
        r = run(lambda: ctx.runner.start(int(body.get("max_cycles", 0)), at, sec))
        n = len(ctx.teach.snapshot()["sequence"])
        ctx.hub.emit("cycle_start", f"{user}: 작업 시작 ({n}단계" + (f", {body['max_cycles']}회" if body.get("max_cycles") else "")
                     + (f", 시작 전 {at + 1}단계부터" if sec == "before" else f", {at + 1}단계부터" if at else "") + ")")
        return r

    @app.post("/api/cycle/step")
    def cycle_step(body: dict = Body(...), user=Depends(require_user)):
        """한 단계만 실행 (단계 시험)."""
        guard_motion()
        return run(lambda: ctx.runner.run_one(int(body["index"]), "before" if body.get("section") == "before" else "main"))

    @app.post("/api/cycle/stop")
    def cycle_stop(user=Depends(require_user)):
        was = ctx.runner.running
        ctx.runner.stop()
        if was:
            ctx.hub.emit("cycle_stop", f"{user}: 작업 정지 ({ctx.runner.cycles}회 완료)")
        return {"ok": True}

    @app.post("/api/cycle/grip_check")
    def grip_check(body: dict = Body(...), user=Depends(require_user)):
        ctx.runner.grip_check = bool(body.get("on"))
        return {"ok": True}

    # ------------------------------------------------------------ 안전
    @app.post("/api/safety/restart")
    def restart(body: dict = Body(default={}), user=Depends(require_user)):
        """한 번에 재가동: 비상정지·고장 해제 → (확인 시) 사라짐 확인 → 토크 켜기 → 안전 조건 재검사 → 재가동.
        사람 근접·조명·장치 같은 안전 조건은 그대로 다시 검사하며, 하나라도 걸리면 재가동하지 않는다."""
        ex, sm = ctx.ex, ctx.safety
        steps = []
        if ex.estop or ex.fault:
            try:
                r = ex.reset()
                steps.append("비상정지·고장 해제" + (f" ({r})" if r and r != "해제 완료" else ""))
            except Exception as e:
                return {"ok": False, "problems": [f"고장 해제 실패: {e}"], "steps": steps}
        if body.get("clear_vanish") and (sm.vanish.unresolved or sm.vanish.pending):
            sm.clear_vanish(user)
            steps.append("사라짐 확인")
        if not ex.torque_on and not ex.freedrive:
            try:
                ex.set_torque(True)
                steps.append("토크 켜기")
            except Exception as e:
                return {"ok": False, "problems": [f"토크 켜기 실패: {e}"], "steps": steps}
        if ex.freedrive:
            return {"ok": False, "problems": ["직접 교시 중 — 먼저 직접 교시를 끝내세요"], "steps": steps}
        time.sleep(0.15)                      # 안전 관리자가 새 상태를 한 번 반영하도록
        ok, problems = sm.restart(user)
        return {"ok": ok, "problems": problems, "steps": steps}

    @app.post("/api/safety/vanish_clear")
    def vanish_clear(user=Depends(require_user)):
        return {"ok": True, "cleared": ctx.safety.clear_vanish(user)}

    @app.post("/api/safety/maintenance")
    def maintenance(body: dict = Body(...), user=Depends(require_user)):
        if body.get("on"):
            ctx.runner.stop("정비 모드")
        ctx.safety.set_maintenance(bool(body.get("on")))
        return {"ok": True}

    @app.post("/api/safety/teach_mode")
    def teach_mode(body: dict = Body(...), user=Depends(require_user)):
        on = bool(body.get("on"))
        if on:
            ctx.runner.stop("티칭 모드")
        return run(lambda: ctx.safety.set_teach_mode(on, user))

    @app.post("/api/safety/fire_end")
    def fire_end(user=Depends(require_user)):
        if ctx.io.fire():
            raise HTTPException(409, "화재 센서가 아직 감지 중 — 센서가 꺼진 뒤 상황 종료")
        return {"ok": ctx.safety.end_fire(user)}

    @app.post("/api/safety/conveyor_clear")
    def conveyor_clear(user=Depends(require_user)):
        ctx.safety.conveyor_alarm = ""
        return {"ok": True}

    @app.post("/api/safety/profile")
    def profile(body: dict = Body(...), user=Depends(require_user)):
        p = body.get("profile")
        if p not in C["safety"]["profiles"]:
            raise HTTPException(400, "없는 프로필")
        C.update("safety", {"profile": p})
        t = C["safety"]["profiles"][p]
        ctx.hub.emit("settings", f"{user}: 거리 기준을 '{PROFILE_NAME.get(p, p)}'로 바꿈 — 감속 {t['slow']} / 정지 {t['stop']} / "
                     f"재가동 허용 {t['clear']} m", level="info")
        return {"ok": True}

    # ------------------------------------------------------------ 가중치
    @app.get("/api/models")
    def models(user=Depends(require_user)):
        return {"models": ctx.det.list_models(), "current": ctx.det.model_name, "status": ctx.det.status,
                "convert": ctx.det.convert_status}

    @app.post("/api/models/select")
    def model_select(body: dict = Body(...), user=Depends(require_user)):
        name = body["name"]
        C.update("perception", {"model": name})
        return run(lambda: ctx.det.select(name))

    @app.post("/api/models/upload")
    async def model_upload(file: UploadFile = File(...), user=Depends(require_user)):
        data = await file.read()
        try:
            name = ctx.det.save_upload(file.filename, data)
        except ValueError as e:
            raise HTTPException(400, str(e))
        return {"ok": True, "name": name}

    # ------------------------------------------------------------ 장치·설정
    @app.post("/api/devices/virtual")
    def dev_virtual(body: dict = Body(...), user=Depends(require_user)):
        if not hasattr(ctx.io, "set_virtual"):
            raise HTTPException(409, "가상 장치 모드가 아님")
        ctx.io.set_virtual(**body)
        return {"ok": True}

    @app.post("/api/devices/door_open")
    def door_open(user=Depends(require_user)):
        ctx.site.open_door(user)
        return {"ok": True}

    @app.get("/api/helmet/frame.jpg")
    def helmet_frame(user=Depends(require_user)):
        g = ctx.site.gate
        img = g.overlay() if g is not None else None
        if img is None:
            raise HTTPException(404, "입구 카메라 영상 없음")
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return Response(buf.tobytes(), media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.post("/api/settings/door")
    def set_door(body: dict = Body(...), user=Depends(require_user)):
        v = float(body["close_sec"])
        if not 1 <= v <= 60:
            raise HTTPException(400, "범위: 1–60 s")
        C.update("door", {"close_sec": v})
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
        C.update("recording", vals)
        return {"ok": True}

    @app.post("/api/settings/door_roi")
    def set_door_roi(body: dict = Body(...), user=Depends(require_user)):
        roi = body.get("roi")
        if roi is not None:
            x1, y1, x2, y2 = [int(v) for v in roi]
            roi = [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]
        C.update("safety", {"door_roi_px": roi})
        return {"ok": True, "roi": roi}

    @app.post("/api/settings/password")
    def set_password(body: dict = Body(...), user=Depends(require_user)):
        if not pw_ok(user, body.get("old", "")):
            raise HTTPException(400, "현재 비밀번호가 틀림")
        new = body.get("new", "")
        if len(new) < 6:
            raise HTTPException(400, "새 비밀번호는 6자 이상 필요")
        salt = secrets.token_hex(8)
        C.update("web", {"users": {**users(), user: f"sha256:{salt}:{hashlib.sha256((salt + new).encode()).hexdigest()}"}})
        return {"ok": True}

    @app.post("/api/settings/thresholds")
    def set_thresholds(body: dict = Body(...), user=Depends(require_user)):
        prof = body.get("profile")
        if prof not in C["safety"]["profiles"]:
            raise HTTPException(400, "없는 프로필")
        slow, stop, clear = float(body["slow"]), float(body["stop"]), float(body["clear"])
        if not (0.03 <= stop < slow < clear <= 5.0):
            raise HTTPException(400, "조건: 0.03 ≤ 정지 < 감속 < 해제 ≤ 5.0 m")
        profiles = {k: dict(v) for k, v in C["safety"]["profiles"].items()}
        profiles[prof] = {"slow": slow, "stop": stop, "clear": clear}
        vals = {"profiles": profiles}
        if "vanish_grace" in body:
            g = float(body["vanish_grace"])
            if not 0.2 <= g <= 5.0:
                raise HTTPException(400, "사라짐 유예 범위: 0.2–5 s")
            vals["vanish_grace"] = g
        C.update("safety", vals)
        active = C["safety"]["profile"]
        ctx.hub.emit("settings", f"{user}: '{PROFILE_NAME.get(prof, prof)}' 기준 저장 — 감속 {slow} / 정지 {stop} / 재가동 허용 {clear} m"
                     + ("" if prof == active else f" (지금 적용 중인 기준은 '{PROFILE_NAME.get(active, active)}')"), level="info")
        return {"ok": True}

    @app.post("/api/settings/guard")
    def set_guard(body: dict = Body(...), user=Depends(require_user)):
        m = float(body.get("meters", 0))
        if not 0 <= m <= 5:
            raise HTTPException(400, "범위: 0–5 m (0 = 끔)")
        ctx.safety.guard_person_m = m
        C.update("safety", {"guard_person_m": m})
        return {"ok": True}

    @app.post("/api/settings/slow_scale")
    def set_slow_scale(body: dict = Body(...), user=Depends(require_user)):
        """감속 구간에서 허용하는 로봇 속도 비율 (%). 저장하면 바로 적용 (2026-10-12 사용자)."""
        pct = float(body.get("pct", 35))
        if not 10 <= pct <= 80:
            raise HTTPException(400, "범위: 10–80 %")
        C.update("safety", {"slow_scale": round(pct / 100, 2)})
        ctx.hub.emit("settings", f"{user}: 감속 속도 {pct:.0f}% 로 변경", level="info")
        return {"ok": True}

    @app.get("/api/logs")
    def logs(lines: int = 200, user=Depends(require_user)):
        p = C.path("logs", "omx.log")
        if not p.exists():
            return {"lines": []}
        with open(p, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 200_000))
            data = f.read().decode("utf-8", "replace").splitlines()
        return {"lines": data[-min(lines, 1000):]}

    @app.post("/api/system/restart")
    def sys_restart(user=Depends(require_user)):
        return _shutdown(restart=True, user=user)

    @app.post("/api/system/shutdown")
    def sys_shutdown(user=Depends(require_user)):
        return _shutdown(restart=False, user=user)

    def _shutdown(restart, user):
        """프로그램 재시작/종료. 로봇이 움직이는 중이면 거부 (토크는 켠 채 자리 유지된다)."""
        if ctx.ex.motion or ctx.runner.running:
            raise HTTPException(409, "로봇 동작 중 — 먼저 정지 필요")
        ctx.hub.emit("system", f"프로그램 {'재시작' if restart else '종료'} ({user})", level="info")

        def later():
            import os
            import shlex
            import subprocess
            import sys
            time.sleep(0.8)
            ctx.ex.shutdown()
            ctx.cam.stop()
            if restart and os.environ.get("INVOCATION_ID"):
                os._exit(3)                 # systemd 서비스로 도는 중: 비정상 종료 코드 → systemd가 다시 띄운다
            if restart:
                # 새 프로세스로 다시 띄운다 (카메라·시리얼이 확실히 풀린 뒤 시작하도록 3초 대기)
                root = Path(__file__).resolve().parents[2]
                args = " ".join(shlex.quote(a) for a in sys.argv[1:])
                log_path = C.path("logs", "restart.log")
                subprocess.Popen(["bash", "-c", f"sleep 3; exec ./run.sh {args}"], cwd=root, start_new_session=True,
                                 stdout=open(log_path, "a"), stderr=subprocess.STDOUT)
            os._exit(0)
        threading.Thread(target=later, daemon=True).start()
        return {"ok": True}

    @app.post("/api/settings/detection")
    def set_detection(body: dict = Body(...), user=Depends(require_user)):
        v = float(body["conf"])
        if not 0.10 <= v <= 0.95:
            raise HTTPException(400, "최소 확신도는 10~95%")
        C.update("perception", {"conf": v})
        ctx.det.conf = v
        return {"ok": True}

    @app.post("/api/settings/conveyor")
    def set_conveyor(body: dict = Body(...), user=Depends(require_user)):
        v = float(body["travel_timeout"])
        if not 0.5 <= v <= 120:
            raise HTTPException(400, "범위: 0.5–120 s")
        C.update("conveyor", {"travel_timeout": v})
        return {"ok": True}

    # ------------------------------------------------------------ 이벤트·영상
    @app.get("/api/events")
    def events(limit: int = 50, before: int = None, user=Depends(require_user)):
        from ..events import STORED
        return {"events": ctx.db.list(min(limit, 500), before, types=sorted(STORED))}

    # 영상 메뉴 분류: 사람 근접 정지 / 물품 미도달(컨베이어) / 화재 / 기타(사라짐·비상정지·고장)
    CLIP_CATS = {"human": ["human_stop"], "conveyor": ["conveyor_jam"], "fire": ["fire"]}

    @app.get("/api/clips")
    def clips(cat: str = "human", limit: int = 60, before: int = None, day: str = None, user=Depends(require_user)):
        if cat not in (*CLIP_CATS, "other"):
            raise HTTPException(400, "분류는 human, conveyor, fire, other")
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
            ctx.hub.emit("clip_save", f"{user}: 영상 저장 {p.name}")
            return FileResponse(p, media_type="video/mp4", filename=p.name)
        return FileResponse(p, media_type="video/mp4")

    # ------------------------------------------------------------ 보정
    @app.post("/api/calib/sample")
    def calib_sample(body: dict = Body(default={}), user=Depends(require_user)):
        lat = ctx.cam.latest()
        if lat is None:
            raise HTTPException(409, "카메라 영상 없음")
        tcp = K.fk_points(ctx.ex.state.q)[-1]
        return run(lambda: ctx.calib.add_sample(lat, ctx.cam.intr, tcp, body.get("u"), body.get("v")))


    def _work_yaws(teach):
        """프로그램(시작 전 + 반복)이 쓰는 자세들의 베이스 방향 (자동 보정이 작업 자리도 돌게)."""
        poses = (teach or {}).get("poses", {})
        names = [s.get("pose") for s in (teach or {}).get("before_start", []) + (teach or {}).get("sequence", [])
                 if s.get("type") in ("move", "linear")]
        return [float(poses[n]["q"][0]) for n in dict.fromkeys(names) if n in poses]

    @app.post("/api/calib/auto")
    def calib_auto(user=Depends(require_user)):
        guard_motion()
        return run(lambda: ctx.autocalib.start(_work_yaws(ctx.teach.snapshot())))

    @app.post("/api/calib/solve")
    def calib_solve(user=Depends(require_user)):
        def f():
            r = ctx.calib.solve()
            ctx.per._fits, ctx.per.calib_valid = [], True
            return r
        return run(f)

    @app.post("/api/calib/reset")
    def calib_reset(user=Depends(require_user)):
        """수동 보정 점만 지운다 (보정 결과는 유지)."""
        return run(lambda: ctx.calib.reset(keep_T=True))

    @app.post("/api/calib/delete")
    def calib_delete(user=Depends(require_user)):
        """보정 결과 삭제 (백업은 data/calib_history). 삭제하면 거리는 카메라 기준으로 바뀐다."""
        r = run(ctx.calib.delete_result)
        ctx.hub.emit("calib", f"{user}이(가) 보정 결과 삭제 — 거리 판정이 카메라 기준으로 바뀜", level="warn")
        return r

    # ------------------------------------------------------------ 영상 스트림
    def overlay(frame, pc):
        """작업 구역 영상 오버레이 — 노드 버전(perception_node.overlay)과 같은 모양 (overlay_style, 시안 B)."""
        from ..perception import overlay_style as OS
        img = frame.color.copy()
        sm = ctx.safety
        labels = OS.Labels(img)
        pts = ctx.ex.link_points()
        if pts is not None and ctx.calib.T_base_cam is not None and ctx.per.calib_valid and ctx.cam.intr is not None:
            sil = ctx.per._fk_silhouette(img.shape, ctx.calib.T_base_cam, pts)
            uv = ctx.calib.project(pts, ctx.cam.intr)
            img = OS.draw_robot(img, sil, None if uv is None else uv[2:-1], None if uv is None else uv[-1])
        if pc is not None:
            layer, tags = img.copy(), []
            for p in pc.persons:
                col = (0, 0, 255) if p.dist < sm.profile()["stop"] else (0, 170, 255) if p.dist < sm.profile()["slow"] else (0, 200, 0)
                x1, y1 = int(p.box[0]), int(p.box[1])
                if p.poly is not None:
                    cv2.fillPoly(layer, [p.poly], col)
                    x1, y1 = int(np.min(p.poly[..., 0])), int(np.min(p.poly[..., 1]))
                txt = f"{p.dist:.2f}m" if np.isfinite(p.dist) else "?"
                tags.append((p.poly, tuple(map(int, p.box)), col, f"person {p.conf:.2f} {txt}", x1, y1))
            for hb, hc, hp in getattr(pc, "helmets", []) or []:
                hx1, hy1 = int(hb[0]), int(hb[1])
                if hp is not None:
                    cv2.fillPoly(layer, [hp], (0, 215, 255))
                    hx1, hy1 = int(np.min(hp[..., 0])), int(np.min(hp[..., 1]))
                tags.append((hp, tuple(map(int, hb)), (0, 215, 255), f"helmet {hc:.2f}", hx1, hy1))
            img = cv2.addWeighted(layer, 0.25, img, 0.75, 0)
            for poly, box, col, _, _, _ in tags:
                if poly is not None:
                    OS.mask_outline(img, poly, col)
                else:
                    cv2.rectangle(img, box[:2], box[2:], col, 1, cv2.LINE_AA)
            for _, _, col, text, x, y in tags:
                labels.put(img, text, x, y, col)
        roi = C["safety"].get("door_roi_px")
        if roi:
            cv2.rectangle(img, (roi[0], roi[1]), (roi[2], roi[3]), (255, 200, 0), 1, cv2.LINE_AA)
            labels.put(img, "EXIT", roi[0], roi[1] + 20, (255, 200, 0))
        return img

    # 보는 화면이 여러 개(젯슨 화면 + 휴대폰 등)여도 프레임당 오버레이·JPEG 압축은 한 번만 한다
    jpeg_cache = {"key": None, "buf": None, "lock": threading.Lock()}

    def encoded(f, pc):
        key = (f.seq, pc is not None)
        with jpeg_cache["lock"]:
            if jpeg_cache["key"] != key:
                ok, buf = cv2.imencode(".jpg", overlay(f, pc), [cv2.IMWRITE_JPEG_QUALITY, 70])
                jpeg_cache["key"], jpeg_cache["buf"] = key, (buf.tobytes() if ok else None)
            return jpeg_cache["buf"]

    @app.get("/video.mjpg")
    def video(user=Depends(require_user)):
        def gen():
            last = -1
            while True:
                lat = ctx.per.latest
                fresh = lat is not None and time.monotonic() - lat[0].t < 1.0
                if fresh and lat[0].seq != last:
                    f, pc = lat
                elif not fresh:
                    f, pc = ctx.cam.latest(), None          # 인식이 멎었으면 원본이라도 보여준다
                    if f is None or f.seq == last:
                        time.sleep(0.03)
                        continue
                else:
                    time.sleep(0.02)
                    continue
                last = f.seq
                buf = encoded(f, pc)
                if buf:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n"
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    # ------------------------------------------------------------ 웹소켓
    def broadcast(msg):
        loop = loop_holder.get("loop")
        if loop is None:
            return
        data = json.dumps(msg, ensure_ascii=False)
        for q in list(clients):
            loop.call_soon_threadsafe(q.put_nowait, data)

    ctx.hub.listeners.append(broadcast)

    @app.on_event("startup")
    async def _startup():
        loop_holder["loop"] = asyncio.get_running_loop()

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        if not check_token(sock.cookies.get(COOKIE, "")):
            await sock.close(code=4401)
            return
        await sock.accept()
        q = asyncio.Queue()
        clients.add(q)

        async def sender():
            while True:
                try:
                    msg = await asyncio.wait_for(q.get(), timeout=0.2)
                    await sock.send_text(msg)
                except asyncio.TimeoutError:
                    st = await asyncio.get_running_loop().run_in_executor(None, full_state)
                    await sock.send_text(json.dumps({"kind": "state", "data": st}, ensure_ascii=False, default=float))

        task = asyncio.create_task(sender())
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                if "jog" in msg:
                    if ctx.safety.fire or ctx.safety.maintenance:
                        continue
                    try:
                        ctx.ex.jog_cmd(msg["jog"], msg.get("mode", "cart"))
                    except Exception as e:
                        await sock.send_text(json.dumps({"kind": "error", "message": str(e)}, ensure_ascii=False))
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            task.cancel()
            clients.discard(q)

    # ------------------------------------------------------------ 정적 파일
    # 화면 파일이 바뀌면 브라우저가 반드시 새로 받도록: index.html 의 app.js·style.css 주소에 파일 버전(수정 시각)을 붙인다
    # (2026-09-27: 새로고침해도 브라우저가 캐시의 옛 app.js 를 써서 새 기능이 안 보였다)
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
        # 서비스 워커도 버전을 품어서, 화면 파일이 바뀌면 새 워커가 설치되고 옛 캐시를 지운다
        js = (STATIC / "sw.js").read_text().replace('"omx-v2"', f'"omx-{asset_ver()}"')
        return Response(js, media_type="application/javascript", headers={"Cache-Control": "no-cache"})

    @app.middleware("http")
    async def static_revalidate(request, call_next):
        r = await call_next(request)
        if request.url.path.startswith("/static/"):
            r.headers["Cache-Control"] = "no-cache"      # 쓸 때마다 ETag 로 확인 (안 바뀌었으면 304, 가벼움)
        return r

    @app.get("/manifest.webmanifest")
    def manifest():
        return FileResponse(STATIC / "manifest.webmanifest", media_type="application/manifest+json")

    app.mount("/static", StaticFiles(directory=STATIC), name="static")
    return app
