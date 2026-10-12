"""티칭 저장소: 자세(이름 → 관절각·TCP), 홈 지정, 순서(반복 동작 단계) — data/poses.json.

원칙 (UR PolyScope 처럼): 자세는 팔만 뜻한다. 그리퍼는 "그리퍼" 단계·명령으로만 움직인다.
(자세에 그리퍼 값이 함께 저장돼 있어도 이동할 때는 쓰지 않는다 — 옛 파일 호환용으로만 남김)
"""
import json
import threading
import time
from pathlib import Path

import numpy as np

from . import kinematics as K

STEP_TYPES = ("move", "linear", "home", "grip", "wait_sensor2", "conveyor", "pause")


def grip_target(g, value):
    """그리퍼 단계 값 → 목표 위치(rad). "open"/"close" 또는 열림 정도 0~100(%, 0=닫힘 위치, 100=열림 위치)."""
    if isinstance(value, dict):                   # 저장한 그리퍼 동작
        return float(value["pos"])
    if value == "open":
        return g["open"]
    if value == "close":
        return g["close"]
    p = min(100.0, max(0.0, float(value))) / 100.0
    return g["close"] + (g["open"] - g["close"]) * p


class TeachStore:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()
        # gripper: 열림·닫힘 기준 위치(rad)와 기본 힘(%). grips: 저장한 그리퍼 동작 {이름: {"pos": rad, "force": %}}
        # before_start: 반복 시작 전에 한 번만 하는 단계 (UR PolyScope 'Before Start', 2026-10-12 사용자)
        self.data = {"poses": {}, "home": None, "sequence": [], "before_start": [],
                     "gripper": {"open": 0.8, "close": 0.0, "force": 50}, "grips": {}}
        if self.path.exists():
            try:
                self.data.update(json.loads(self.path.read_text()))
            except json.JSONDecodeError:
                pass
        self.data["gripper"].setdefault("force", 50)
        self.data.setdefault("grips", {})
        self.data.setdefault("before_start", [])

    def _save(self):
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.data, indent=2, ensure_ascii=False))
        tmp.replace(self.path)

    def save_pose(self, name, q):
        name = name.strip()[:32]
        if not name:
            raise ValueError("이름이 비어 있음")
        q = [round(float(v), 4) for v in q]
        with self.lock:
            self.data["poses"][name] = {"q": q, "tcp": [round(float(v), 4) for v in K.fk(np.array(q))],
                                        "t": time.strftime("%Y-%m-%d %H:%M:%S")}
            self._save()

    def delete_pose(self, name):
        with self.lock:
            self.data["poses"].pop(name, None)
            if self.data["home"] == name:
                self.data["home"] = None
            self._save()

    def set_home(self, name):
        with self.lock:
            if name not in self.data["poses"]:
                raise ValueError(f"'{name}' 자세가 없음")
            self.data["home"] = name
            self._save()

    def home_q(self):
        h = self.data.get("home")
        return self.data["poses"][h]["q"] if h in self.data["poses"] else None

    def pose_q(self, name):
        return self.data["poses"][name]["q"]

    # ---- 저장한 그리퍼 동작 (이름 → 위치·힘) ----
    def save_grip(self, name, pos, force):
        name = name.strip()[:32]
        if not name:
            raise ValueError("이름이 비어 있음")
        if name in ("열기", "닫기", "open", "close"):
            raise ValueError("열기·닫기는 기본 동작 이름입니다 — 다른 이름을 쓰세요")
        with self.lock:
            self.data["grips"][name] = {"pos": round(float(pos), 4), "force": int(min(100, max(5, float(force)))),
                                        "t": time.strftime("%Y-%m-%d %H:%M:%S")}
            self._save()

    def delete_grip(self, name):
        with self.lock:
            used = [f"{i + 1}단계" for i, s in enumerate(self.data["sequence"]) if s.get("type") == "grip" and s.get("preset") == name]
            used += [f"시작 전 {i + 1}단계" for i, s in enumerate(self.data["before_start"])
                     if s.get("type") == "grip" and s.get("preset") == name]
            if used:
                raise ValueError(f"작업 {', '.join(used)}에서 쓰는 중 — 먼저 단계를 바꾸세요")
            self.data["grips"].pop(name, None)
            self._save()

    def grip_action(self, step):
        """그리퍼 단계/명령 → (목표 위치 rad, 힘 %, 설명)."""
        g = self.data["gripper"]
        if step.get("preset"):
            p = self.data["grips"].get(step["preset"])
            if p is None:
                raise ValueError(f"저장된 그리퍼 동작 '{step['preset']}' 없음")
            return p["pos"], p["force"], step["preset"]
        v = step.get("value", "open")
        force = step.get("force", g.get("force", 50))
        return grip_target(g, v), force, {"open": "열기", "close": "닫기"}.get(v, f"{v}%")

    def set_sequence(self, steps, before_start=None):
        """steps = 반복 프로그램. before_start = 시작 전 한 번만 하는 단계 (None 이면 지금 것 유지)."""
        clean = self._clean_steps(steps, "")
        before = None if before_start is None else self._clean_steps(before_start, "시작 전 ")
        with self.lock:
            self.data["sequence"] = clean
            if before is not None:
                self.data["before_start"] = before
            self._save()

    def _clean_steps(self, steps, where):
        clean = []
        for i, s in enumerate(steps, 1):
            i = f"{where}{i}"
            t = s.get("type")
            if t not in STEP_TYPES:
                raise ValueError(f"{i}단계: 알 수 없는 종류 {t}")
            c = {"type": t}
            if t in ("move", "linear"):
                if s.get("pose") not in self.data["poses"]:
                    raise ValueError(f"{i}단계: 없는 자세 {s.get('pose')}")
                c["pose"] = s["pose"]
            if t in ("move", "linear", "home"):
                c["speed"] = int(min(100, max(5, float(s.get("speed", 100)))))    # 이 단계 속도 (현재 속도 설정의 %)
            if t == "grip":
                if s.get("preset"):                                              # 저장한 그리퍼 동작
                    if s["preset"] not in self.data["grips"]:
                        raise ValueError(f"{i}단계: 저장된 그리퍼 동작 '{s['preset']}' 없음")
                    c["preset"] = s["preset"]
                else:
                    v = s.get("value", "open")
                    if v not in ("open", "close"):
                        v = round(min(100.0, max(0.0, float(v))), 1)             # 열림 정도 %
                    c["value"] = v
                    if s.get("force") is not None:
                        c["force"] = int(min(100, max(5, float(s["force"]))))    # 힘 % (없으면 기본 힘)
            if t == "conveyor":
                c["value"] = "off" if s.get("value") == "off" else "on"
            if t == "pause":
                c["value"] = round(min(600.0, max(0.1, float(s.get("value", 1.0)))), 1)
            clean.append(c)
        return clean

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.data))
