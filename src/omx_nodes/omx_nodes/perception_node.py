"""인식 노드 — GPU 를 쓰는 일은 모두 이 한 프로세스에 (CUDA 컨텍스트·TensorRT 라이브러리를 한 번만 올린다).

- D435i → 사람 인식(v2 통합 모델 레인 0) → 사람-로봇 링크 거리 → /omx/people (프레임마다)
- 입구 USB 카메라 → 헬멧 게이트(같은 모델 레인 1, 마스크 없이) → /omx/helmet 10Hz
- 순환 녹화 + 사고 영상: /omx/events 의 clip 이벤트를 받아 만들고 결과를 /omx/clip_status 로
- 웹 영상: 보는 사람이 있을 때만(웹 노드가 /omx/web/viewers 로 알림) 오버레이 JPEG 를 /dev/shm 에 15fps 이하로
카메라 영상은 ROS 토픽으로 보내지 않는다 (복사·직렬화 비용).
"""
import argparse
import base64
import os
import sys
import threading
import time

import numpy as np
import rclpy
from std_msgs.msg import Int32

from omx.perception.gpu_post import blocking_sync

blocking_sync()          # CUDA 를 쓰기 전에: GPU 를 기다리는 동안 CPU 가 바쁘게 돌지 않고 잠든다

import cv2  # noqa: E402

from omx.perception.autocalib import AutoCalibrator  # noqa: E402
from omx.perception.calib import Calibration  # noqa: E402
from omx.perception.camera import UsbCamSource, make_camera  # noqa: E402
from omx.perception.detector import Detector  # noqa: E402
from omx.perception.helmet import HelmetGate  # noqa: E402
from omx.perception.people import PeoplePerception  # noqa: E402
from omx.recording.recorder import Recorder  # noqa: E402
from omx.robot import kinematics as K  # noqa: E402
from omx_interfaces.msg import ClipStatus, Event, HelmetState, People, Person  # noqa: E402

from .common import (QOS_DURABLE, QOS_LATEST, ChangePublisher, OmxNode, device_lock, loop_thread, now,  # noqa: E402
                     shm_path, spin)
from .proxies import ExecutorProxy, SafetyProxy  # noqa: E402

STREAM_FPS = 15.0
INF = float("inf")


class PerceptionNode(OmxNode):
    def __init__(self, video=None, no_helmet=False):
        super().__init__("perception")
        C = self.cfg
        if video:
            C.cfg["camera"]["source"] = "video"
            C.cfg["camera"]["video_path"] = video
        self.t_start_wall = time.time()
        devs = ([] if video else ["realsense"]) + (["monocam"] if C["helmet"]["enabled"] and not no_helmet else [])
        device_lock(*devs)
        self.ex = ExecutorProxy(self, C["robot"], permit_pub=False)
        self.safety = SafetyProxy(self)
        self.cam = make_camera(C).start()
        pc = C["perception"]
        # 통합 모델 하나, 실행 통로 둘: 0 = 사람(D435i), 1 = 헬멧(입구 카메라)
        self.det = Detector(C.data_dir, pc["model"], pc["conf"], pc["imgsz"], pc.get("backend", "direct"), lanes=2,
                            require_any=pc.get("person_classes", ["person"]))
        self.clip_dir = C.path("clips")
        self.rec = Recorder(self.cam, C, self.clip_dir).start()
        self.calib = Calibration(C.path("calib.json"))
        self.per = PeoplePerception(self.cam, self.det, C, self.ex.link_points, self.calib)
        # 사람이 보이는 동안은 매 프레임, 아무도 없으면 0.2초마다 (사람이 나타나고 사라지는 순간은 바로 보낸다).
        # 안전 관리자는 프레임 시각으로 신선도를 보므로 (1초) 빈 화면의 생존 신호로 충분하다.
        self.people_out = ChangePublisher(self.create_publisher(People, "/omx/people", QOS_LATEST), 0.2)
        self.per.listeners.append(self._publish_people)
        self.per.start()
        self.gate = None
        if C["helmet"]["enabled"] and not no_helmet:
            try:
                hc = C["helmet"]
                self.mono = UsbCamSource(hc["camera"], fps=hc["fps"]).start()
                self.gate = HelmetGate(self.mono, self.det, C, lane=1).start()
            except Exception:
                self.log.exception("헬멧 게이트 시작 실패 — 출입문은 화재·수동으로만 열림")
        self.helmet_out = ChangePublisher(self.create_publisher(HelmetState, "/omx/helmet", QOS_LATEST), 0.5)
        if self.gate is not None:
            loop_thread(self._publish_helmet, 10, "helmet-pub")
        # autocalib 은 기존 ctx 모양을 쓴다
        self.autocalib = AutoCalibrator(self._ctx())
        # 사고 영상
        self.clip_pub = self.create_publisher(ClipStatus, "/omx/clip_status", QOS_DURABLE)
        self.create_subscription(Event, "/omx/events", self._event, QOS_DURABLE, callback_group=self.cb)
        # 웹 영상
        self.viewers_t = 0.0
        self.create_subscription(Int32, "/omx/web/viewers", self._viewers, QOS_LATEST, callback_group=self.cb)
        self.live_path = shm_path(C, "live.jpg")
        threading.Thread(target=self._stream_loop, daemon=True, name="stream").start()
        self.on_config.append(self._config)
        self._register()
        self.start_snapshot(self.snapshot)
        self.log.info("인식 노드 시작 (모델 %s, %s)", self.det.model_name or "없음", self.det.status)

    def _ctx(self):
        from types import SimpleNamespace
        return SimpleNamespace(ex=self.ex, safety=self.safety, cam=self.cam, calib=self.calib, per=self.per,
                               cfg=self.cfg, hub=self.events)

    # ------------------------------------------------------------------ 발행
    def _publish_people(self, f, p):
        if p.persons:
            self._last_person_t = p.t
        # 사람이 사라진 뒤 1초 동안은 매 프레임 (해제 판정·사라짐 판정이 원래 속도로 진행되게)
        busy = p.persons or p.t - getattr(self, "_last_person_t", -9.0) < 1.0
        key = p.seq if busy else (p.model_ready, self.cam.error or "", int(p.brightness) // 8)
        self.people_out.offer(key, lambda: self._people_msg(p))

    def _people_msg(self, p):
        h, w = p.shape
        persons = []
        for q in p.persons:
            has = q.pos is not None and np.all(np.isfinite(q.pos))
            persons.append(Person(
                box=[float(v) for v in q.box], conf=float(q.conf), px=[float(q.px[0]), float(q.px[1])],
                has_pos=bool(has), pos=[float(v) for v in q.pos] if has else [0.0, 0.0, 0.0],
                dist=float(q.dist), cam_dist=float(q.cam_dist), npts=int(q.npts), in_zone=bool(q.in_zone),
                overlap=float(q.overlap), near=float(q.near), name=q.name, fallen=bool(q.fallen)))
        return People(
            t=float(p.t), seq=int(p.seq), model_ready=bool(p.model_ready), calibrated=bool(p.calibrated),
            brightness=float(p.brightness), min_dist=float(p.min_dist), infer_ms=float(p.infer_ms),
            height=int(h), width=int(w), cam_error=self.cam.error or "", persons=persons)

    def _publish_helmet(self):
        g = self.gate
        left = float(max(0.0, g.open_until - time.monotonic()))
        self.helmet_out.offer((left > 0, round(g.open_until, 1), g.state),
                              lambda: HelmetState(t=now(), open_left=left, state=g.state))

    # ------------------------------------------------------------------ 사고 영상
    def _event(self, e):
        if e.wall_t < self.t_start_wall - 2.0:          # 다시 켜졌을 때 다시 받는 지난 이벤트는 녹화하지 않는다
            return
        if e.end_clip:
            self.rec.end(e.key)
            return
        if not e.clip:
            return
        rc = self.cfg["recording"]
        post = None if e.until_end else (rc["conveyor_post_sec"] if e.type.startswith("conveyor") else rc["post_sec"])
        self.rec.start_clip(e.key, rc["pre_sec"], post, on_done=self._clip_done, name=e.type)

    def _clip_done(self, key, path, size):
        self.clip_pub.publish(ClipStatus(key=key, file=path.name if path else "", size=int(size or 0),
                                         status="ready" if path else "failed"))

    # ------------------------------------------------------------------ 웹 영상
    def _viewers(self, m):
        if m.data > 0:
            self.viewers_t = time.monotonic()

    def overlay(self, frame, pc):
        img = frame.color.copy()
        C = self.cfg
        prof = C["safety"]["profiles"][C["safety"]["profile"]]
        helmets = getattr(pc, "helmets", []) if pc is not None else []
        if pc is not None and (pc.persons or helmets):
            layer = img.copy() if (any(p.poly is not None for p in pc.persons) or any(h[2] is not None for h in helmets)) else None
            for p in pc.persons:
                col = (0, 0, 255) if p.dist < prof["stop"] else (0, 170, 255) if p.dist < prof["slow"] else (0, 200, 0)
                x1, y1, x2, y2 = map(int, p.box)
                # 사람 표시는 마스크(SEG)만 — 테두리 상자(BBOX)는 마스크가 없을 때만 (2026-10-12 사용자)
                if p.poly is not None:
                    cv2.fillPoly(layer, [p.poly], col)
                    cv2.polylines(img, [p.poly], True, col, 2)
                    y1 = int(np.min(p.poly[..., 1]))          # 글자는 마스크 맨 위에
                    x1 = int(np.min(p.poly[..., 0]))
                else:
                    cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
                txt = f"{p.dist:.2f}m" if np.isfinite(p.dist) else "?"
                label = "FALLEN" if p.fallen else p.name
                cv2.putText(img, f"{label} {p.conf:.2f} {txt}", (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            for hb, hc, hp in helmets:      # 헬멧: 노란 마스크 (표시만)
                hx1, hy1, hx2, hy2 = map(int, hb)
                if hp is not None:
                    cv2.fillPoly(layer, [hp], (0, 215, 255))
                    cv2.polylines(img, [hp], True, (0, 215, 255), 2)
                    hx1, hy1 = int(np.min(hp[..., 0])), int(np.min(hp[..., 1]))
                else:
                    cv2.rectangle(img, (hx1, hy1), (hx2, hy2), (0, 215, 255), 2)
                cv2.putText(img, f"helmet {hc:.2f}", (hx1, max(14, hy1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 215, 255), 2)
            if layer is not None:
                img = cv2.addWeighted(layer, 0.35, img, 0.65, 0)
        pts = self.ex.link_points()
        if pts is not None and self.calib.T_base_cam is not None and self.per.calib_valid and self.cam.intr is not None:
            # 계산한 로봇 모습 전체(거리 판단·보정 일치율에 쓰는 모양)를 반투명 하늘색 + 테두리로 (2026-10-12 사용자:
            # 관절 중심을 이은 가는 선은 ㄱ자 팔에서 몸통을 비껴 지나가 어긋나 보였다)
            sil = self.per._fk_silhouette(img.shape, self.calib.T_base_cam, pts)
            if sil.any():
                tint = img.copy()
                tint[sil] = (255, 230, 0)
                img = cv2.addWeighted(tint, 0.30, img, 0.70, 0)
                cs, _ = cv2.findContours(sil.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, cs, -1, (255, 230, 0), 2)
            uv = self.calib.project(pts[-1:], self.cam.intr)
            if uv is not None:
                cv2.circle(img, tuple(uv.astype(int)[-1]), 5, (255, 0, 255), -1)      # 그리퍼 끝 (TCP)
        roi = C["safety"].get("door_roi_px")
        if roi:
            cv2.rectangle(img, (roi[0], roi[1]), (roi[2], roi[3]), (255, 200, 0), 2)
            cv2.putText(img, "EXIT", (roi[0] + 4, roi[1] + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)
        return img

    def _stream_loop(self):
        """보는 사람이 있을 때만, 새 프레임만, 15fps 이하로 오버레이 JPEG 를 공유 메모리 파일에 쓴다 (웹 노드가 읽어 보냄)."""
        last, period = -1, 1.0 / STREAM_FPS
        tmp = self.live_path.with_suffix(".tmp")
        while rclpy.ok():
            t0 = time.monotonic()
            try:
                if t0 - self.viewers_t < 3.0:
                    lat = self.per.latest
                    fresh = lat is not None and t0 - lat[0].t < 1.0
                    f, pc = (lat if fresh else (self.cam.latest(), None))   # 인식이 멎었으면 원본이라도
                    if f is not None and f.seq != last:
                        last = f.seq
                        ok, buf = cv2.imencode(".jpg", self.overlay(f, pc), [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if ok:
                            tmp.write_bytes(buf.tobytes())
                            os.replace(tmp, self.live_path)
            except Exception:
                self.log.exception("stream")
                time.sleep(1.0)
            time.sleep(max(0.005, period - (time.monotonic() - t0)))

    # ------------------------------------------------------------------ 상태·명령
    def snapshot(self):
        per, det, cam = self.per, self.det, self.cam
        return {
            "perception": {
                "model": det.model_name, "model_status": det.status, "convert": det.convert_status,
                "model_ready": det.ready, "gpu_post_error": det.gpu_post_error(),
                "infer_ms": round(det.infer_ms, 1), "fps": round(per.fps, 1),
                "cam_fps": round(cam.fps_meas, 1), "cam_error": cam.error,
                "persons": len(per.latest[1].persons) if per.latest else 0,
                "fallen": sum(p.fallen for p in per.latest[1].persons) if per.latest else 0,
                "robot_rejects": getattr(per, "robot_rejects", 0),
                "conf": det.conf,
                "calib": {**self.calib.snapshot(), "auto": self.autocalib.status, "auto_running": self.autocalib.running,
                          "valid": per.calib_valid, "fit": None if per.calib_fit is None else round(per.calib_fit, 2)},
            },
            "recording": self.rec.status(),
            "helmet": self.gate.snapshot() if self.gate is not None else None,
            "persons": self._persons() if per.latest else {"persons": []},
        }

    def _links(self):
        """로봇 링크 관절점 (로봇 베이스 기준, 제어판 감시 구역 지도용). 보정 전이면 사람 위치가 카메라 기준이라 None."""
        pts = self.ex.link_points()
        if pts is None or self.calib.T_base_cam is None or not self.per.calib_valid:
            return None
        return [[round(float(v), 3) for v in p] for p in pts]

    def _persons(self):
        lat = self.per.latest
        if lat is None:
            return {"persons": []}
        fin = lambda v: None if not np.isfinite(v) else round(float(v), 3)  # noqa: E731
        return {"t": lat[1].t, "persons": [{"box": [round(v) for v in p.box], "conf": round(p.conf, 3),
                                            "dist": fin(p.dist), "cam_dist": fin(p.cam_dist), "npts": p.npts,
                                            "in_zone": p.in_zone, "name": p.name, "fallen": p.fallen,
                                            "overlap": round(p.overlap, 2), "near": round(p.near, 2),
                                            "pos": None if p.pos is None else [round(float(v), 3) for v in p.pos]}
                                           for p in lat[1].persons],
                "links": self._links(),
                "robot_rejects": getattr(self.per, "robot_rejects", 0)}

    def _config(self, section):
        if section == "perception":
            self.det.conf = float(self.cfg["perception"]["conf"])

    def _register(self):
        det, calib, cam = self.det, self.calib, self.cam

        def model_select(a):
            name = a["name"]
            ok = det.select(name, wait=True)
            if ok is False:
                raise RuntimeError(det.load_error or det.status)
            # 불러오기에 성공했을 때만 저장한다 (예전: 먼저 저장해 없는 모델이면 다음 부팅이 실패했다)
            self.update_config("perception", {"model": name})
            return None

        def calib_sample(a):
            lat = cam.latest()
            if lat is None:
                raise RuntimeError("카메라 영상 없음")
            st = self.ex.state
            if st is None:
                raise RuntimeError("로봇 상태 없음")
            tcp = K.fk_points(st.q)[-1]
            return calib.add_sample(lat, cam.intr, tcp, a.get("u"), a.get("v"))

        def calib_solve(a):
            r = calib.solve()
            self.per._fits, self.per.calib_valid = [], True
            return r

        def helmet_frame(a):
            img = self.gate.overlay() if self.gate is not None else None
            if img is None:
                raise PermissionError("입구 카메라 영상 없음")
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return base64.b64encode(buf.tobytes()).decode()

        self.handlers.update({
            "models": lambda a: {"models": det.list_models(), "current": det.model_name, "status": det.status,
                                 "convert": det.convert_status},
            "model_select": model_select,
            "calib_sample": calib_sample,
            "calib_auto": lambda a: self.autocalib.start(a.get("work_yaws") or ()),
            "calib_solve": calib_solve,
            "calib_reset": lambda a: calib.reset(keep_T=True),
            "calib_delete": lambda a: calib.delete_result(),
            "helmet_frame": helmet_frame,
            "persons": lambda a: self._persons(),
            "shutdown": lambda a: (cam.stop(), threading.Thread(target=lambda: (time.sleep(0.5), rclpy.shutdown()),
                                                                 daemon=True).start()) and None,
        })


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", help="카메라 대신 녹화 영상 (깊이가 없어 거리 판정은 하지 않음)")
    ap.add_argument("--no-helmet", action="store_true", help="입구 카메라를 쓰지 않음 (시험용)")
    a, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    spin(PerceptionNode(video=a.video, no_helmet=a.no_helmet))


if __name__ == "__main__":
    main()
