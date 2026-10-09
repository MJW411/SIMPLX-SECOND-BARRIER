"""현장 입출력 장치 어댑터 (컨베이어, 근접센서 1·2, 화재 센서, 출입문, 상태 불빛).

같은 인터페이스로 가상 장치(제어판 버튼)와 실제 아두이노를 바꿔 끼운다.

아두이노 쪽 책임 (젯슨 없이도 스스로):
- 화재 감지 시 즉시 출입문 개방·컨베이어 정지·노란불
- 젯슨 하트비트가 끊기면 컨베이어 정지·출입문 닫힘
- 센서2 감지 시 컨베이어 즉시 정지 (set_arrive_stop(False) 면 안 멈춤 — 도착 센서 고장 시 임시)
"""
import logging
import threading
import time

log = logging.getLogger("io")


class IOBase:
    def sensor1(self): ...
    def sensor2(self): ...
    def fire(self): ...
    def door_open(self): ...
    def conveyor_on(self): ...
    def set_door(self, open_): ...
    def set_conveyor(self, on): ...
    def set_arrive_stop(self, on): ...          # 도착 센서 감지 시 장치가 스스로 멈출지
    def set_led(self, code): ...                # G 녹색(대기·작업 끝) B 하늘색(작업 중) O 주황 점멸(사람) Y 노랑(화재)
    def led(self): return "G"
    def flame(self): return None
    def ready(self): return True                # 실제 센서 값을 한 번이라도 받았는가
    def healthy(self): return True
    def health_msg(self): return "정상"
    def snapshot(self):
        return {"backend": self.name, "sensor1": self.sensor1(), "sensor2": self.sensor2(), "fire": self.fire(),
                "door_open": self.door_open(), "conveyor_on": self.conveyor_on(), "led": self.led(),
                "flame": self.flame(), "healthy": self.healthy(), "health": self.health_msg()}


class VirtualIO(IOBase):
    """제어판 버튼으로 조작하는 가상 장치. 아두이노 동작(화재 시 문 자동 개방)도 흉내낸다."""
    name = "virtual"

    def __init__(self):
        self.s = {"sensor1": False, "sensor2": False, "fire": False, "door": False, "conveyor": False}
        self.led_code = "G"
        self.arrive_stop = True
        self.lock = threading.Lock()

    def set_virtual(self, **kw):
        with self.lock:
            for k, v in kw.items():
                if k in self.s:
                    self.s[k] = bool(v)
            if self.s["fire"]:
                self.s["door"] = True          # 아두이노는 젯슨을 기다리지 않고 문을 연다

    def sensor1(self): return self.s["sensor1"]
    def sensor2(self): return self.s["sensor2"]
    def fire(self): return self.s["fire"]
    def door_open(self): return self.s["door"]
    def conveyor_on(self):                    # 아두이노처럼: 센서2·화재면 명령과 무관하게 멈춤
        return self.s["conveyor"] and not (self.s["sensor2"] and self.arrive_stop) and not self.s["fire"]

    def set_arrive_stop(self, on):
        self.arrive_stop = bool(on)

    def set_door(self, open_):
        self.set_virtual(door=open_)

    def set_conveyor(self, on):
        self.set_virtual(conveyor=on)

    def set_led(self, code):
        self.led_code = code

    def led(self): return self.led_code


class ArduinoIO(IOBase):
    """실제 아두이노 나노 (arduino/omx_io/omx_io.ino).

    프로토콜 (9600bps, 줄 단위 텍스트):
      젯슨 → 아두이노: "C <conv> <door> <led>" 200ms마다 + 바뀔 때. 받는 것 자체가 하트비트.
      아두이노 → 젯슨: "S <s1> <s2> <fire> <door> <conv> <flame>" 바뀔 때 + 200ms마다
    아두이노는 명령이 1초 끊기면 컨베이어를 멈추고 문을 닫으며, 불꽃이면 젯슨을 기다리지 않고 문을 연다.
    포트를 열면 아두이노가 재시작한다(DTR) → 첫 응답까지 약 2초. 그동안은 고장으로 보지 않는다.
    """
    name = "arduino"
    BOOT_GRACE = 5.0
    RX_TIMEOUT = 1.0
    TX_PERIOD = 0.2

    def __init__(self, port="auto", baud=9600, opener=None):
        self.port_cfg, self.baud = port, baud
        self.opener = opener or self._open_serial
        self.st = {"sensor1": False, "sensor2": False, "fire": False, "door": False, "conveyor": False, "flame": 0}
        self.cmd = {"conveyor": False, "door": False, "led": "G", "arrive_stop": True}
        self.port = ""
        self.err = "연결 전"
        self.last_rx = 0.0
        self.t_start = time.monotonic()
        self._dirty = threading.Event()
        self._run = True
        threading.Thread(target=self._loop, daemon=True, name="arduino").start()

    # ---- 포트
    @staticmethod
    def find_port(cfg_port="auto"):
        """'auto' → /dev/serial/by-id 의 FT232R·CH340 직렬 장치 (로봇 OpenRB·U2D2 는 제외), 없으면 첫 /dev/ttyUSB*."""
        if cfg_port and cfg_port != "auto":
            return cfg_port
        import glob
        for pat in ("*FT232R*", "*CH340*", "*1a86*", "*Arduino*"):
            for p in sorted(glob.glob("/dev/serial/by-id/" + pat)):
                if "ROBOTIS" not in p and "OpenRB" not in p:
                    return p
        usb = sorted(glob.glob("/dev/ttyUSB*"))
        if usb:
            return usb[0]
        raise IOError("아두이노 USB 장치 없음 (/dev/serial/by-id, /dev/ttyUSB*)")

    def _open_serial(self):
        import serial
        self.port = self.find_port(self.port_cfg)
        return serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.5)

    # ---- 통신 스레드
    def _cmd_line(self):
        c = self.cmd
        tail = "" if c["arrive_stop"] else " 0"         # 기본(도착 센서로 정지)이면 예전 형식 그대로
        return f"C {int(c['conveyor'])} {int(c['door'])} {c['led']}{tail}\n".encode()

    def _parse(self, line):
        p = line.split()
        if len(p) != 7 or p[0] != "S":
            return                                  # 시작 안내 같은 다른 줄은 무시
        try:
            v = [int(x) for x in p[1:]]
        except ValueError:
            return
        self.st = {"sensor1": v[0] == 1, "sensor2": v[1] == 1, "fire": v[2] == 1, "door": v[3] == 1,
                   "conveyor": v[4] == 1, "flame": v[5]}
        self.last_rx = time.monotonic()

    def _loop(self):
        ser, buf, last_tx = None, b"", 0.0
        while self._run:
            if ser is None:
                try:
                    ser = self.opener()
                    buf, self.err = b"", ""
                    log.info("아두이노 연결: %s", self.port)
                except Exception as e:
                    self.err = f"포트 열기 실패: {e}"
                    time.sleep(1.0)
                    continue
            try:
                now = time.monotonic()
                if self._dirty.is_set() or now - last_tx >= self.TX_PERIOD:
                    self._dirty.clear()
                    ser.write(self._cmd_line())
                    last_tx = now
                data = ser.read(256)
                if data:
                    buf += data
                    *lines, buf = buf.split(b"\n")
                    buf = buf[-128:]
                    for ln in lines:
                        self._parse(ln.decode(errors="replace").strip())
            except Exception as e:
                log.warning("아두이노 통신 오류, 다시 연결: %s", e)
                self.err = f"통신 오류: {e}"
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(1.0)
        if ser is not None:
            ser.close()

    def stop(self):
        self._run = False

    # ---- 인터페이스
    def sensor1(self): return self.st["sensor1"]
    def sensor2(self): return self.st["sensor2"]
    def fire(self): return self.st["fire"]
    def door_open(self): return self.st["door"]
    def conveyor_on(self): return self.st["conveyor"]
    def flame(self): return self.st["flame"]
    def led(self): return self.cmd["led"]

    def _set(self, k, v):
        if self.cmd[k] != v:
            self.cmd[k] = v
            self._dirty.set()

    def set_conveyor(self, on): self._set("conveyor", bool(on))
    def set_door(self, open_): self._set("door", bool(open_))
    def set_led(self, code): self._set("led", code)
    def set_arrive_stop(self, on): self._set("arrive_stop", bool(on))

    def ready(self):
        return self.last_rx > 0.0

    def healthy(self):
        now = time.monotonic()
        if now - self.last_rx < self.RX_TIMEOUT:
            return True
        return self.last_rx == 0.0 and now - self.t_start < self.BOOT_GRACE

    def health_msg(self):
        if self.healthy():
            return "정상"
        return "아두이노 응답 없음" + (f" ({self.err})" if self.err else "")


def make_io(cfg, force_virtual=False):
    d = cfg["devices"]
    if d["backend"] == "arduino" and not force_virtual:
        return ArduinoIO(d.get("arduino_port", "auto"), d.get("arduino_baud", 9600))
    return VirtualIO()
