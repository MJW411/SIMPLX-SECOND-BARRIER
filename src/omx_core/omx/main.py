"""전체 시작: 로봇 실행기 · 카메라 · 인식 · 안전 관리자 · 녹화 · DB · 장치 · 웹 제어판.

  python -m omx.main [--sim] [--video 파일] [--port 8000]
"""
import argparse
import logging
import logging.handlers
import os
import sys
from types import SimpleNamespace

import uvicorn

from .config import Config


def setup_logging(C):
    d = C.path("logs")
    d.mkdir(exist_ok=True)
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    h = logging.handlers.RotatingFileHandler(d / "omx.log", maxBytes=5_000_000, backupCount=3)
    h.setFormatter(fmt)
    root.addHandler(h)
    s = logging.StreamHandler(sys.stdout)
    s.setFormatter(fmt)
    root.addHandler(s)
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)


def build(C, sim=False):
    from .devices.io import make_io
    from .events import EventHub
    from .perception.calib import Calibration
    from .perception.camera import make_camera
    from .perception.detector import Detector
    from .perception.people import PeoplePerception
    from .recording.recorder import Recorder
    from .robot.dxl_bus import BusError, DxlBus, SimBus
    from .robot.executor import Executor
    from .robot.task import ConveyorMonitor, CycleRunner
    from .robot.teach import TeachStore
    from .safety.manager import SafetyManager
    from .storage.db import EventDB
    from .sysmon import SysMon

    log = logging.getLogger("main")
    r = C["robot"]
    ids = [r["ids"][k] for k in ("joint1", "joint2", "joint3", "joint4", "joint5", "gripper")]
    backend = "sim" if sim or r["backend"] == "sim" else "real"
    if backend == "real":
        try:
            bus = DxlBus(r["port"], r["baud"], ids, C.limits["gripper_current_ma"])
        except BusError as e:
            log.error("로봇 연결 실패 (%s) — 가상 로봇으로 시작", e)
            bus, backend = SimBus(), "sim (로봇 연결 실패)"
    else:
        bus = SimBus()

    clip_dir = C.path("clips")
    db = EventDB(C.path("events.db"), C.data_dir, C["storage"]["budget_ratio"],
                 C["storage"].get("budget_bytes"))
    # GPU 대기 중 CPU 가 바쁘게 돌지 않도록: 카메라(RealSense CUDA)·TensorRT 가 CUDA 를 쓰기 전에 정한다
    from .perception.gpu_post import blocking_sync
    log.info("CUDA 대기 방식: %s", "잠든 채 대기" if blocking_sync() else "기본(라이브러리 없음)")
    cam = make_camera(C).start()
    # 통합 모델 하나를 두 실행 통로로: 0 = 사람(D435i), 1 = 헬멧(입구 카메라). 가중치·CUDA 는 한 번만 올라간다.
    det = Detector(C.data_dir, C["perception"]["model"], C["perception"]["conf"], C["perception"]["imgsz"],
                   C["perception"].get("backend", "direct"), lanes=2,
                   require_any=C["perception"].get("person_classes", ["person"]))
    rec = Recorder(cam, C, clip_dir).start()
    hub = EventHub(C, db, rec)
    ex = Executor(bus, r, C.limits)
    calib = Calibration(C.path("calib.json"))
    per = PeoplePerception(cam, det, C, ex.link_points, calib).start()
    # 가상 로봇 + 영상 파일 시험은 실제 서버와 함께 띄울 수 있으므로 실제 장치(아두이노·모노 카메라)를 잡지 않는다
    test_only = sim and C["camera"]["source"] == "video"
    io = make_io(C, force_virtual=test_only)
    safety = SafetyManager(C, ex, per, io, hub)
    hub.state_fn = lambda: safety.state
    teach = TeachStore(C.path("poses.json"))
    safety.home_q_fn = teach.home_q
    runner = CycleRunner(ex, teach, io, hub, C.limits, safety)
    runner.home_after_done = bool(C["robot"].get("home_after_done", True))
    safety.on_fire_start.append(lambda: runner.stop("화재"))
    conveyor = ConveyorMonitor(io, hub, C, safety, runner)
    gate = None
    if C["helmet"]["enabled"] and not test_only:
        from .perception.camera import UsbCamSource
        from .perception.helmet import HelmetGate
        try:
            hc = C["helmet"]
            mono = UsbCamSource(hc["camera"], fps=hc["fps"]).start()
            gate = HelmetGate(mono, det, C, lane=1).start()
        except Exception:
            log.exception("헬멧 게이트 시작 실패 — 출입문은 화재·수동으로만 열림")
    from .devices.site import SiteController
    site = SiteController(io, safety, runner, C, gate, hub).start()
    def robot_event(kind, msg):
        if kind == "estop":
            return                      # 비상정지는 안전 관리자가 기록한다 (중복 방지)
        hub.emit("robot" if kind == "robot" else f"robot_{kind}", msg,
                 level="warn" if kind in ("fault", "temp", "angle") else "info")
    ex.on_event = robot_event
    ex.start()
    safety.start()
    guard = float(os.environ.get("OMX_GUARD_PERSON_M", "") or C["safety"].get("guard_person_m", 0) or 0)
    safety.guard_person_m = guard
    hub.emit("system", f"시스템 시작 (로봇: {backend}, 모델: {det.model_name})", level="info")
    ctx = SimpleNamespace(cfg=C, db=db, cam=cam, det=det, rec=rec, hub=hub, ex=ex, calib=calib, per=per, io=io,
                          safety=safety, teach=teach, runner=runner, conveyor=conveyor, site=site, clip_dir=clip_dir,
                          robot_backend=backend, sysmon=SysMon())
    from .perception.autocalib import AutoCalibrator
    ctx.autocalib = AutoCalibrator(ctx)
    return ctx


def single_instance(C):
    """로봇·카메라를 두 프로그램이 동시에 잡지 않도록 잠금 파일로 한 개만 실행한다.
    (2026-09-27: 종료 신호를 받은 이전 서버가 브라우저 연결 때문에 남아 카메라·로봇 포트를 계속 쥐고 있었다)"""
    import fcntl
    f = open(C.path("omx.lock"), "w")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("이미 다른 OMX 프로그램이 실행 중입니다 (data/omx.lock). 먼저 그 프로그램을 끄세요.", file=sys.stderr)
        sys.exit(2)
    f.write(str(os.getpid()))
    f.flush()
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="로봇 없이 가상 로봇으로")
    ap.add_argument("--video", help="카메라 대신 영상 파일")
    ap.add_argument("--port", type=int)
    a = ap.parse_args()
    C = Config()
    if a.video:
        C.cfg["camera"]["source"] = "video"
        C.cfg["camera"]["video_path"] = a.video
    setup_logging(C)
    # 실제 로봇이나 실제 카메라를 쓸 때만 잠근다 (가상 로봇 + 영상 파일 시험은 장치를 안 쓰므로 함께 띄울 수 있다)
    _lock = single_instance(C) if not (a.sim and a.video) else None  # noqa: F841
    ctx = build(C, sim=a.sim)
    from .web.app import create_app
    app = create_app(ctx)
    port = a.port or C["web"]["port"]
    logging.getLogger("main").info("제어판: http://<젯슨 IP>:%d  (로그인 %s)", port, ", ".join(C["web"]["users"]))
    # 영상 스트림 같은 열린 연결이 있어도 3초 안에 확실히 끝나도록 (남은 프로세스가 장치를 쥐지 않게)
    uvicorn.run(app, host=C["web"]["host"], port=port, log_level="warning", timeout_graceful_shutdown=3)


if __name__ == "__main__":
    main()
