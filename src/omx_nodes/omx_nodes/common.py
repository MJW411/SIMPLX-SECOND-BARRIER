"""노드 공통: 설정·로그, 관리 명령(Command 서비스) 서버/클라이언트, 이벤트 발행, 상태 스냅샷.

- 시각은 모두 time.monotonic() (CLOCK_MONOTONIC: 같은 컴퓨터의 프로세스끼리 비교할 수 있다).
- 관리 명령은 omx_interfaces/srv/Command 하나로: cmd + JSON 인자 → ok + 메시지 + JSON 결과.
  웹 제어판의 API 70여 개마다 서비스 타입을 만들지 않기 위해서다. 안전에 쓰는 경로는 타입 메시지로 따로 간다.
- 이벤트: 각 노드가 /omx/events 로 보내고 웹 노드가 DB 에 저장, 인식 노드가 녹화한다.
  기존 EventHub 와 같은 emit() 모양이라 안전 관리자·실행기 코드를 고치지 않고 쓴다.
"""
import itertools
import json
from concurrent.futures import ThreadPoolExecutor
import logging
import logging.handlers
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

from omx.config import Config
from omx.events import STORED
from omx_interfaces.msg import Event
from omx_interfaces.srv import Command

# 최신 값 하나만 의미 있는 상태 (허가·인식 결과·각 노드 상태)
QOS_LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
# 나중에 켜진 노드도 받아야 하는 것 (이벤트·클립 결과·스냅샷): 웹 노드가 재시작해도 놓치지 않게
QOS_DURABLE = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_SNAPSHOT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_CMD = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

log = logging.getLogger("node")


def now():
    return time.monotonic()


DEMO_LOCK = Path.home() / "omx-safety-demo" / "data" / "omx.lock"
_held_locks = []


def device_lock(*devices):
    """실제 장치를 두 프로그램이 동시에 잡지 않게 (예: 같은 OpenRB 포트에 두 실행기가 목표를 보내는 일).
    - 장치마다 배타 잠금: /run/user/<uid>/omx_<장치>.lock
    - 백업 데모(omx-safety-demo)의 잠금 파일에는 공유 잠금: 데모가 켜져 있으면 이 노드가 시작하지 않고,
      이 노드가 켜져 있으면 데모가 시작하지 않는다 (파일 내용은 건드리지 않는다).
    잡지 못하면 이유를 남기고 종료한다 (launch·systemd 가 잠시 뒤 다시 시도한다)."""
    import fcntl
    base = Path(os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid()}")
    if not base.is_dir():
        base = Path("/tmp")
    for dev in devices:
        f = open(base / f"omx_{dev}.lock", "a")
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            logging.getLogger("lock").error("장치 %s 를 다른 OMX 프로그램이 쓰는 중 — 종료", dev)
            sys.exit(2)
        _held_locks.append(f)
    if devices and DEMO_LOCK.exists():
        f = open(DEMO_LOCK, "a")
        try:
            fcntl.flock(f, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except OSError:
            logging.getLogger("lock").error("백업 데모(omx-safety-demo)가 실행 중 — 같은 장치를 쓰므로 종료")
            sys.exit(2)
        _held_locks.append(f)


def shm_path(cfg, name):
    """노드끼리 주고받는 공유 메모리 파일 (예: 웹 영상 JPEG). 데이터 폴더마다 따로 (시험용 인스턴스와 섞이지 않게)."""
    import hashlib
    from pathlib import Path
    tag = hashlib.md5(str(cfg.data_dir.resolve()).encode()).hexdigest()[:8]
    return Path(f"/dev/shm/omx_{tag}_{name}")


def jsonable(v):
    """numpy·inf 를 JSON 으로 (브라우저 JSON.parse 는 Infinity 를 못 읽는다)."""
    if isinstance(v, dict):
        return {str(k): jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [jsonable(x) for x in v]
    if isinstance(v, np.ndarray):
        return jsonable(v.tolist())
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.floating, float)):
        f = float(v)
        return f if math.isfinite(f) else None
    if isinstance(v, np.bool_):
        return bool(v)
    return v


def dumps(v):
    return json.dumps(jsonable(v), ensure_ascii=False)


def setup_logging(cfg, name):
    d = cfg.path("logs")
    d.mkdir(exist_ok=True)
    fmt = logging.Formatter(f"%(asctime)s %(levelname)s [{name}] %(name)s: %(message)s")
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    for h in list(root.handlers):
        root.removeHandler(h)
    h = logging.handlers.RotatingFileHandler(d / f"{name}.log", maxBytes=5_000_000, backupCount=3)
    h.setFormatter(fmt)
    root.addHandler(h)
    s = logging.StreamHandler(sys.stdout)
    s.setFormatter(fmt)
    root.addHandler(s)


class RemoteError(RuntimeError):
    """다른 노드가 명령을 거절했거나 응답이 없음 (메시지는 사용자에게 그대로 보인다)."""


class RpcClient:
    def __init__(self, node, target):
        self.node, self.target = node, target
        self.cli = node.create_client(Command, f"/omx/{target}/cmd", callback_group=node.cb)

    def call(self, cmd, timeout=5.0, **args):
        """동기 호출 (실행기 스레드가 아닌 곳에서). 거절되면 RemoteError(메시지)."""
        if not self.cli.service_is_ready() and not self.cli.wait_for_service(timeout_sec=min(1.0, timeout)):
            raise RemoteError(f"{self.target} 노드 응답 없음")
        req = Command.Request(cmd=cmd, args_json=dumps(args))
        done = threading.Event()
        fut = self.cli.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            fut.cancel()
            raise RemoteError(f"{self.target} 노드 응답 시간 초과 ({cmd})")
        r = fut.result()
        if r is None:
            raise RemoteError(f"{self.target} 노드 응답 없음 ({cmd})")
        if not r.ok:
            raise RemoteError(r.message)
        return json.loads(r.result_json) if r.result_json else None

    def send(self, cmd, **args):
        """기다리지 않는 호출 (결과를 쓰지 않는 명령). 서비스가 없으면 버린다."""
        if not self.cli.service_is_ready():
            log.warning("%s 노드 없음 — 명령 %s 버림", self.target, cmd)
            return
        self.cli.call_async(Command.Request(cmd=cmd, args_json=dumps(args)))


class EventEmitter:
    """기존 EventHub.emit() 과 같은 규칙: 저장 대상(STORED)이 아니고 녹화도 없으면 로그 파일에만.
    같은 알림 3초 안 반복 억제, 같은 종류 클립이 후 녹화 중이면 새 클립을 만들지 않는다.
    반환값은 이벤트 키(문자열) — 화재처럼 끝을 알려야 하는 녹화는 end_recording(key)."""

    def __init__(self, node, cfg, source):
        self.node, self.cfg, self.source = node, cfg, source
        self.pub = node.create_publisher(Event, "/omx/events", QOS_DURABLE)
        self._last, self._last_clip = {}, {}
        self._seq = itertools.count(1)
        self._boot = f"{source}-{int(time.time())}"
        self.lock = threading.Lock()
        self.listeners = []

    def emit(self, etype, message, level="info", clip=False, until_end=False):
        lg = logging.getLogger("events")
        if etype not in STORED and not clip:
            lg.info("[%s] %s: %s", level, etype, message)
            return None
        t = now()
        with self.lock:
            if not clip and t - self._last.get((etype, message), -99) < 3.0:
                return None
            self._last[(etype, message)] = t
        if clip and not until_end:
            post = self.cfg["recording"]["post_sec"]
            with self.lock:
                if t - self._last_clip.get(etype, -999) < post:
                    clip = False
                    message += " (진행 중인 클립에 포함)"
                else:
                    self._last_clip[etype] = t
        key = f"{self._boot}-{next(self._seq)}"
        lg.info("[%s] %s: %s", level, etype, message)
        self.pub.publish(Event(key=key, type=etype, level=level, message=message, clip=bool(clip),
                               until_end=bool(until_end), end_clip=False, wall_t=time.time(), source=self.source))
        return key

    def end_recording(self, key):
        if key:
            self.pub.publish(Event(key=key, end_clip=True, wall_t=time.time(), source=self.source))


class OmxNode(Node):
    """노드 공통 바탕. handlers: {cmd: fn(args dict) -> JSON 결과} 를 /omx/<name>/cmd 서비스로 연다."""

    def __init__(self, name):
        super().__init__(f"omx_{name}")
        self.name_ = name
        self.cfg = Config()
        setup_logging(self.cfg, name)
        self.log = logging.getLogger(name)
        self.cb = ReentrantCallbackGroup()
        # 관리 명령은 작업 스레드에서 실행한다: 실행기(단일 스레드, rclpy 에서 가장 가볍다)가 명령을 기다리느라
        # 다른 메시지(허가·로봇 상태 등) 처리를 미루지 않게
        self.pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix=f"{name}-cmd")
        self.handlers = {}
        self.events = EventEmitter(self, self.cfg, name)
        self._rpc_clients = {}
        self.cfg_pub = self.create_publisher(String, "/omx/config_changed", QOS_CMD)
        self.on_config = []                            # fn(section) — 다른 노드가 설정을 바꿨을 때
        self.create_subscription(String, "/omx/config_changed", self._config_changed, QOS_CMD, callback_group=self.cb)
        self.create_service(Command, f"/omx/{name}/cmd", self._serve, callback_group=self.cb)
        self.snap_pub = self.create_publisher(String, f"/omx/{name}/snapshot", QOS_SNAPSHOT)
        self.snap_fn = None
        self._snap_now = False                         # 명령을 처리하면 바로 한 번 보낸다 (화면이 늦지 않게)
        self.handlers["snapshot"] = lambda a: self.snap_fn() if self.snap_fn else None
        self.web_clients_t = 0.0                       # 제어판을 보는 사람이 있으면 스냅샷 5Hz, 없으면 1Hz
        self.create_subscription(Int32, "/omx/web/clients", self._web_clients, QOS_LATEST, callback_group=self.cb)

    # ---- 명령 ----
    async def _serve(self, req, res):
        fn = self.handlers.get(req.cmd)
        if fn is None:
            res.ok, res.message = False, f"모르는 명령: {req.cmd}"
            return res
        fut = Future(executor=self.executor)

        def work():
            try:
                args = json.loads(req.args_json) if req.args_json else {}
                r = fn(args)
                if req.cmd != "snapshot":
                    self._snap_now = True
                fut.set_result((True, r))
            except Exception as e:                     # 거절 사유는 그대로 사용자에게
                if not isinstance(e, (ValueError, RuntimeError, KeyError, PermissionError)):
                    self.log.exception("명령 %s", req.cmd)
                fut.set_result((False, e))
        self.pool.submit(work)
        ok, val = await fut
        if ok:
            try:
                res.ok, res.message, res.result_json = True, "", dumps(val)
            except Exception as e:
                res.ok, res.message = False, f"결과 변환 실패: {e}"
        else:
            res.ok, res.message = False, str(val) or type(val).__name__
        return res

    def client(self, target):
        if target not in self._rpc_clients:
            self._rpc_clients[target] = RpcClient(self, target)
        return self._rpc_clients[target]

    # ---- 설정 ----
    def update_config(self, section, values):
        """settings.json 에 저장하고 다른 노드에 알린다."""
        self.cfg.update(section, values)
        self.cfg_pub.publish(String(data=f"{self.name_}:{section}"))

    def _config_changed(self, msg):
        src, _, section = msg.data.partition(":")
        if src == self.name_:
            return
        self.cfg.reload()
        for fn in self.on_config:
            try:
                fn(section)
            except Exception:
                self.log.exception("config %s", section)

    # ---- 스냅샷 (웹 제어판 표시용): 보는 사람이 있을 때 5Hz, 없을 때 1Hz (웹 노드가 살아 있는지 보는 용도) ----
    def _web_clients(self, m):
        if m.data > 0:
            self.web_clients_t = now()

    def start_snapshot(self, fn):
        self.snap_fn = fn
        threading.Thread(target=self._snap_loop, daemon=True, name="snapshot").start()

    def _snap_loop(self):
        last = 0.0
        while rclpy.ok():
            t = now()
            period = 0.2 if t - self.web_clients_t < 3.0 else 1.0
            if t - last >= period or self._snap_now:
                last, self._snap_now = t, False
                try:
                    self.snap_pub.publish(String(data=dumps(self.snap_fn())))
                except Exception:
                    self.log.exception("snapshot")
            time.sleep(0.05)


class ChangePublisher:
    """값이 바뀔 때만 보내고, 안 바뀌어도 heartbeat 초마다 한 번 보낸다 (받는 쪽은 끊김을 heartbeat 로 안다).
    rclpy 는 메시지 하나를 주고받는 비용이 커서, 가만히 있는 상태를 고정 주기로 계속 보내지 않는다."""

    def __init__(self, pub, heartbeat=0.2):
        self.pub, self.heartbeat = pub, heartbeat
        self.last_key, self.last_t = None, 0.0
        self.lock = threading.Lock()                   # 주기 스레드와 명령 스레드가 같이 부를 수 있다

    def offer(self, key, make):
        with self.lock:
            t = now()
            if key != self.last_key or t - self.last_t >= self.heartbeat:
                self.last_key, self.last_t = key, t
                self.pub.publish(make())
                return True
            return False


def loop_thread(fn, hz, name):
    """실행기 타이머 대신 보통 스레드로 주기 작업 (rclpy 타이머 콜백 비용을 피한다)."""
    def run():
        period = 1.0 / hz
        nxt = time.monotonic()
        while rclpy.ok():
            try:
                fn()
            except Exception:
                logging.getLogger(name).exception(name)
            nxt += period
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()
    t = threading.Thread(target=run, daemon=True, name=name)
    t.start()
    return t


def spin(node, main=None):
    """단일 스레드 실행기를 백그라운드로 돌린다 (관리 명령은 노드의 작업 스레드에서). main 이 있으면 메인 스레드에서 (웹 서버)."""
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True, name="ros-exec")
    t.start()
    try:
        if main is not None:
            main()
        else:
            while rclpy.ok():
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown(timeout_sec=1.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
