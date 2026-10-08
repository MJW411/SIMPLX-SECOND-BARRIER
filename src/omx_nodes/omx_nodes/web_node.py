"""웹 노드 — 관리자 제어판(FastAPI, :8000) + 이벤트 DB(SQLite) + 자원 표시.

- 다른 노드의 상태 스냅샷(/omx/<노드>/snapshot, 5Hz)을 모아 화면에 보낸다.
- /omx/events 를 받아 DB 에 저장하고 화면에 바로 알린다. /omx/clip_status 로 영상 결과를 반영한다.
- 이 프로세스가 죽어도 로봇·안전 판단에는 영향이 없다 (명령을 보내는 쪽일 뿐).
"""
import json
import os
import threading
import time

import psutil
import rclpy
import uvicorn
from std_msgs.msg import Empty, Int32, String

from omx.storage.db import EventDB
from omx_interfaces.msg import ClipStatus, Event

from .common import QOS_CMD, QOS_DURABLE, QOS_LATEST, QOS_SNAPSHOT, OmxNode, loop_thread, shm_path, spin
from .proxies import IoProxy, SafetyProxy

NODES = ("perception", "safety", "robot", "io")
SNAP_TIMEOUT = 3.0                       # 아무도 안 볼 때 스냅샷은 1Hz
GPU_LOAD = "/sys/devices/platform/bus@0/17000000.gpu/load"


def _read(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except OSError:
        return None


def pss_mb(pid):
    """프로세스 실사용 메모리(PSS, 공유 라이브러리를 나눠 센 값) — 프로세스들을 더하면 전체 사용량에 가깝다."""
    try:
        with open(f"/proc/{pid}/smaps_rollup") as f:
            for line in f:
                if line.startswith("Pss:"):
                    return int(line.split()[1]) / 1024
    except OSError:
        pass
    return 0.0


class SysMon:
    """젯슨 자원 + OMX 노드 전체 메모리·CPU (예전 단일 프로세스의 '이 프로그램' 자리에 노드 합계를 보인다)."""

    def __init__(self, active_fn=lambda: True):
        self.data = {}
        self.procs = {}
        self.active_fn = active_fn                 # 제어판을 보는 사람이 있을 때만 잰다
        psutil.cpu_percent()
        threading.Thread(target=self._loop, daemon=True, name="sysmon").start()

    def _find(self):
        out = {}
        for p in psutil.process_iter(["pid", "cmdline"]):
            cmd = " ".join(p.info["cmdline"] or [])
            for n in (*NODES, "web"):
                if f"omx_nodes.{n}_node" in cmd or f"/omx_nodes/{n}_node" in cmd or f"lib/omx_nodes/{n}" in cmd:
                    out[n] = self.procs.get(n) if n in self.procs and self.procs[n].pid == p.pid else p
        return out

    def _loop(self):
        n = 0
        while True:
            if not self.active_fn():
                time.sleep(1.0)
                continue
            if n % 5 == 0:
                self.procs = self._find()
            n += 1
            vm = psutil.virtual_memory()
            temps = {}
            for z in range(10):
                t = _read(f"/sys/class/thermal/thermal_zone{z}/type")
                v = _read(f"/sys/class/thermal/thermal_zone{z}/temp")
                if t and v and v.lstrip("-").isdigit():
                    temps[t.replace("-thermal", "")] = round(int(v) / 1000, 1)
            g = _read(GPU_LOAD)
            nodes, total, cpu = {}, 0.0, 0.0
            for name, p in self.procs.items():
                try:
                    c = p.cpu_percent()
                except psutil.Error:
                    continue
                m = pss_mb(p.pid)
                nodes[name] = {"mb": round(m), "cpu": round(c, 1)}
                total += m
                cpu += c
            self.data = {
                "cpu": psutil.cpu_percent(),
                "ram_used_gb": round((vm.total - vm.available) / 1e9, 2),
                "ram_total_gb": round(vm.total / 1e9, 2),
                "proc_rss_mb": round(total),
                "proc_cpu": round(cpu, 1),
                "nodes": nodes,
                "gpu": round(int(g) / 10, 1) if g and g.isdigit() else None,
                "temp": temps,
            }
            time.sleep(2.0)

    def snapshot(self):
        return self.data


class WebNode(OmxNode):
    def __init__(self):
        super().__init__("web")
        C = self.cfg
        self.clip_dir = C.path("clips")
        self.clip_dir.mkdir(parents=True, exist_ok=True)
        self.db = EventDB(C.path("events.db"), C.data_dir, C["storage"]["budget_ratio"],
                          C["storage"].get("budget_bytes"), clip_dir=self.clip_dir)
        stuck, orphan = self.db.cleanup()
        if stuck or orphan:
            self.log.info("시작 정리: 녹화 중에 멈춘 이벤트 %d건 → 실패, DB 에 없는 영상 %d개 삭제", stuck, orphan)
        self.state_t = 0.0                         # 마지막 /api/state 직접 조회
        self.sysmon = SysMon(lambda: self.ws_clients > 0 or time.monotonic() - self.state_t < 10.0)
        self.listeners = []
        self.io = IoProxy(self)
        self.safety = SafetyProxy(self)
        self.live_path = shm_path(C, "live.jpg")
        self._snaps = {n: ({}, 0.0) for n in NODES}
        for n in NODES:
            self.create_subscription(String, f"/omx/{n}/snapshot", lambda m, n=n: self._snap(n, m), QOS_SNAPSHOT,
                                     callback_group=self.cb)
        self.create_subscription(Event, "/omx/events", self._event, QOS_DURABLE, callback_group=self.cb)
        self.create_subscription(ClipStatus, "/omx/clip_status", self._clip, QOS_DURABLE, callback_group=self.cb)
        self.estop_pub = self.create_publisher(Empty, "/omx/estop", QOS_CMD)
        self.viewers = 0
        self.ws_clients = 0
        self.vlock = threading.Lock()
        self.viewers_pub = self.create_publisher(Int32, "/omx/web/viewers", QOS_LATEST)
        self.clients_pub = self.create_publisher(Int32, "/omx/web/clients", QOS_LATEST)
        loop_thread(self._announce, 1.0, "web-announce")

    # ---- 상태 ----
    def _snap(self, n, m):
        try:
            self._snaps[n] = (json.loads(m.data), time.monotonic())
        except ValueError:
            pass

    def snap(self, n):
        d, t = self._snaps[n]
        if n == "safety" and time.monotonic() - t > SNAP_TIMEOUT:
            # 안전 노드가 끊기면 화면에도 정지로 (로봇은 허가가 끊겨 이미 서 있다)
            d = {**d, "state": "STOP", "state_ko": "정지", "reasons": ["안전 노드 응답 없음"] + list(d.get("reasons", []))}
        return d

    def snap_live(self, n):
        try:
            d = self.client(n).call("snapshot", timeout=0.5)
            self._snaps[n] = (d, time.monotonic())
            return d
        except Exception:
            return self.snap(n)

    def node_status(self):
        t = time.monotonic()
        return {n: {"alive": t - self._snaps[n][1] < SNAP_TIMEOUT,
                    "age": None if not self._snaps[n][1] else round(t - self._snaps[n][1], 1)} for n in NODES}

    def rpc(self, target):
        return self.client(target)

    def publish_estop(self):
        self.estop_pub.publish(Empty())

    def _announce(self):
        # 영상을 보는 사람(인식 노드가 오버레이를 만들지), 제어판 접속 수(각 노드 스냅샷을 5Hz 로 보낼지)
        self.viewers_pub.publish(Int32(data=self.viewers))
        self.clients_pub.publish(Int32(data=self.ws_clients))

    def client_add(self, d):
        with self.vlock:
            self.ws_clients = max(0, self.ws_clients + d)
            n = self.ws_clients
        if d > 0:
            self.clients_pub.publish(Int32(data=n))

    def viewer_add(self, d):
        with self.vlock:
            self.viewers = max(0, self.viewers + d)
            n = self.viewers
        if d > 0:
            self.viewers_pub.publish(Int32(data=n))   # 첫 접속은 바로 알려 영상이 곧 나오게

    # ---- 이벤트 ----
    def _notify(self, msg):
        for cb in list(self.listeners):
            try:
                cb(msg)
            except Exception:
                self.log.exception("listener")

    def _event(self, e):
        if e.end_clip:
            return
        if self.db.id_for_key(e.key) is not None:      # 웹 노드가 다시 켜졌을 때 다시 받는 이벤트
            return
        state = self.snap("safety").get("state")
        eid = self.db.add(e.type, e.level, e.message, state=state, clip_status="recording" if e.clip else "none",
                          key=e.key, t=e.wall_t)
        self._notify({"kind": "event", "id": eid, "type": e.type, "level": e.level, "message": e.message,
                      "t": e.wall_t})

    def _clip(self, c):
        eid = self.db.id_for_key(c.key)
        if eid is None:
            return
        row = self.db.get(eid)
        if row and row["clip_status"] == c.status and (row["clip"] or "") == c.file:
            return                                       # 다시 받은 같은 결과
        self.db.set_clip(eid, c.file or None, c.size, c.status)
        self._notify({"kind": "clip", "id": eid, "ready": c.status == "ready"})
        threading.Thread(target=self._prune, daemon=True).start()

    def _prune(self):
        try:
            self.db.prune()
        except Exception:
            self.log.exception("prune")


def main():
    rclpy.init()
    node = WebNode()
    from .webapp import create_app
    app = create_app(node)
    C = node.cfg
    port = int(os.environ.get("OMX_WEB_PORT") or C["web"]["port"])
    node.log.info("제어판: http://<젯슨 IP>:%d  (로그인 %s)", port, ", ".join(C["web"]["users"]))

    def serve():
        # 다른 노드의 첫 상태를 받은 뒤에 연다 (막 켜진 웹이 '응답 없음'을 잘못 보여 주지 않게, 최대 5초)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and not all(node._snaps[n][1] for n in NODES):
            time.sleep(0.1)
        node.log.info("노드 상태 수신: %s", {n: bool(node._snaps[n][1]) for n in NODES})
        # 영상 스트림 같은 열린 연결이 있어도 3초 안에 확실히 끝나도록
        uvicorn.run(app, host=C["web"]["host"], port=port, log_level="warning", timeout_graceful_shutdown=3)
    spin(node, main=serve)


if __name__ == "__main__":
    main()
