"""이벤트 허브: DB 기록 + 즉시 알림 + 녹화 클립 요청을 한 번에.

알림은 클립 완성을 기다리지 않고 즉시 보낸다. 클립이 완성되면 다시 알린다.
"""
import logging
import threading
import time

log = logging.getLogger("events")

# 이벤트 로그(DB·제어판 '이벤트')에 남기는 종류 (2026-09-27 사용자 요청: 로그가 너무 많음).
# 그 밖의 종류(감속, 토크 켜짐/꺼짐, 직접 교시, 자동 보정 진행, 반복 자동 중단 등)는 기록 파일(data/logs/omx.log)에만 남긴다.
STORED = {
    # 안전 경보 (사고 영상이 붙는 것 포함)
    "human_stop", "vanish", "conveyor_jam", "fire", "fire_end", "fault", "estop",
    # 낙상 (알림·녹화만 — 정지 이유가 아님)
    "person_fall",
    # 로봇 이상: 모터 고장·과열, 집기 실패
    # (관절 각도 이상 robot_angle 은 2026-10-09 사용자 요청으로 이벤트 목록에서 뺌 — 로봇 기록 파일에만 남는다)
    "robot_fault", "robot_temp", "pick_fail",
    # 사용자 판단: 재가동, 작업 시작·정지, 시스템 시작·종료, 영상 저장, 사라짐 확인, 설정·정비·보정 삭제, 출입문 수동 열기, 티칭 모드
    "restart", "cycle_start", "cycle_stop", "system", "clip_save", "vanish_clear", "settings", "maintenance", "calib",
    "door_manual", "teach_mode",
    # 작업 완료 (정한 반복 횟수를 마침 — 제어판 팝업·벽면 초록)
    "cycle_done",
    # 로봇 작업 일시 정지·재개 (컨베이어 이상 → 멈춘 자리에서 대기, 해제되면 이어서)
    "cycle_pause", "cycle_resume",
    # 컨베이어 이상 자동 해제 (물품 도착·작업 새로 시작)
    "conveyor_clear",
}

# 제어판 이벤트 분류 (이벤트 화면 탭). 빠진 종류는 '운영'으로 본다.
EVENT_CATS = {
    "safety": ["human_stop", "vanish", "person_fall", "estop", "fault"],
    "fire": ["fire", "fire_end"],
    "robot": ["robot_fault", "robot_temp", "pick_fail"],
    "conveyor": ["conveyor_jam", "conveyor_clear"],
    "task": ["cycle_start", "cycle_stop", "cycle_done", "cycle_pause", "cycle_resume"],
}
EVENT_CATS["ops"] = sorted(STORED - {t for v in EVENT_CATS.values() for t in v})

# 사고로 세는 종류 (대시보드 '사고 현황' 도넛)
INCIDENTS = ["human_stop", "vanish", "person_fall", "conveyor_jam", "fire", "estop", "fault", "robot_fault", "pick_fail"]


class EventHub:
    def __init__(self, cfg, db, recorder=None):
        self.cfg = cfg
        self.db = db
        self.rec = recorder
        self.listeners = []          # fn(dict) — 웹소켓 방송 등
        self.state_fn = lambda: None
        self._last = {}
        self._last_clip = {}
        self.lock = threading.Lock()

    def emit(self, etype, message, level="info", clip=False, until_end=False):
        if etype not in STORED and not clip:
            log.info("[%s] %s: %s", level, etype, message)          # 기록 파일에만
            return None
        now = time.monotonic()
        with self.lock:
            if not clip and now - self._last.get((etype, message), -99) < 3.0:
                return None                       # 같은 알림 폭주 방지
            self._last[(etype, message)] = now
        if clip and not until_end:
            # 같은 종류 클립이 후 녹화 중이면 새 클립을 만들지 않는다 (진행 중 클립에 포함됨)
            post = self.cfg["recording"]["post_sec"]
            with self.lock:
                if now - self._last_clip.get(etype, -999) < post:
                    clip = False
                    message += " (진행 중인 클립에 포함)"
                else:
                    self._last_clip[etype] = now
        eid = self.db.add(etype, level, message, state=self.state_fn(),
                          clip_status="recording" if (clip and self.rec) else "none")
        log.info("[%s] %s: %s", level, etype, message)
        self._notify({"kind": "event", "id": eid, "type": etype, "level": level, "message": message,
                      "t": time.time()})
        if clip and self.rec:
            rc = self.cfg["recording"]
            post = None if until_end else (rc["conveyor_post_sec"] if etype.startswith("conveyor") else rc["post_sec"])
            self.rec.start_clip(eid, rc["pre_sec"], post, on_done=self._clip_done, name=etype)
        return eid

    def end_recording(self, eid):
        if eid is not None and self.rec:
            self.rec.end(eid)

    def _clip_done(self, eid, path, size):
        self.db.set_clip(eid, path.name if path else None, size, "ready" if path else "failed")
        self._notify({"kind": "clip", "id": eid, "ready": bool(path)})
        try:
            self.db.prune()
        except Exception:
            log.exception("prune")

    def _notify(self, msg):
        for cb in list(self.listeners):
            try:
                cb(msg)
            except Exception:
                log.exception("listener")
