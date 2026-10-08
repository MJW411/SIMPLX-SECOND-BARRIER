"""Dynamixel 버스 (실제 로봇 / 가상 로봇 같은 인터페이스).

관절 순서는 [joint1..joint5, gripper]. 각도는 rad, 원시값 2048 = 0 rad.
EEPROM(운전 모드, Drive Mode, 위치 한계)은 건드리지 않는다. ROBOTIS 공식 설정 그대로 쓴다.
"""
import logging
import math
import threading
import time

import numpy as np

log = logging.getLogger("dxl")

TICK_PER_RAD = 4096 / (2 * math.pi)

# Protocol 2.0 X 시리즈 공통 주소 (XL430, XL330)
ADDR_TORQUE = 64
ADDR_HW_ERR = 70
ADDR_BUS_WDT = 98
ADDR_GOAL_PWM = 100
ADDR_GOAL_CUR = 102
ADDR_PROF_ACC = 108
ADDR_PROF_VEL = 112
ADDR_GOAL_POS = 116
ADDR_POS_D, ADDR_POS_I, ADDR_POS_P = 80, 82, 84   # Position D/I/P Gain (RAM, 재부팅하면 기본값)
ADDR_STATE = 126          # present current/load(2) vel(4) pos(4) vel_traj(4) pos_traj(4) volt(2) temp(1)
LEN_STATE = 21
MODEL_XL430 = 1060
PWM_LIMIT = 885


def rad2raw(a):
    return int(round(a * TICK_PER_RAD)) + 2048


def raw2rad(v):
    return (v - 2048) / TICK_PER_RAD


def _s16(v):
    return v - 65536 if v >= 32768 else v


def _s32(v):
    return v - (1 << 32) if v >= (1 << 31) else v


class BusError(RuntimeError):
    pass


def find_port(port):
    """'auto' → /dev/serial/by-id 의 ROBOTIS 장치, 없으면 첫 /dev/ttyACM*."""
    import glob
    if port and port != "auto":
        return port
    ids = sorted(glob.glob("/dev/serial/by-id/*ROBOTIS*")) + sorted(glob.glob("/dev/serial/by-id/*OpenRB*"))
    if ids:
        return ids[0]
    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[0]
    raise BusError("로봇 USB 장치 없음 (/dev/serial/by-id, /dev/ttyACM*)")


class RobotState:
    __slots__ = ("t", "q", "dq", "load", "temp", "volt", "hw_err", "ok")

    def __init__(self, n):
        self.t = 0.0
        self.q = np.zeros(n)
        self.dq = np.zeros(n)
        self.load = np.zeros(n)     # 한계 대비 비율 (0~1)
        self.temp = np.zeros(n)
        self.volt = np.zeros(n)
        self.hw_err = np.zeros(n, dtype=int)
        self.ok = False


class DxlBus:
    def __init__(self, port, baud, ids, gripper_current_ma=250):
        self.port_cfg, self.baud = port, baud
        self.ids = list(ids)
        self.n = len(self.ids)
        self.lock = threading.Lock()
        self.gripper_current = gripper_current_ma
        self.cfg_args = None
        self.state = RobotState(self.n)
        self._open()

    def _open(self):
        from dynamixel_sdk import (COMM_SUCCESS, GroupSyncRead, GroupSyncWrite,
                                   PacketHandler, PortHandler)
        self._ok = COMM_SUCCESS
        self.port_name = find_port(self.port_cfg)
        self.port = PortHandler(self.port_name)
        self.pk = PacketHandler(2.0)
        if not self.port.openPort():
            raise BusError(f"포트 열기 실패: {self.port_name}")
        if not self.port.setBaudRate(self.baud):
            raise BusError("통신 속도 설정 실패")
        self.models = {}
        for i in self.ids:
            m, c, e = self.pk.ping(self.port, i)
            if c != self._ok:
                raise BusError(f"모터 ID {i} 응답 없음")
            self.models[i] = m
        self.sr = GroupSyncRead(self.port, self.pk, ADDR_STATE, LEN_STATE)
        for i in self.ids:
            self.sr.addParam(i)
        self.sw = GroupSyncWrite(self.port, self.pk, ADDR_GOAL_POS, 4)

    def reconnect(self):
        """USB 재연결·전원 재투입 뒤 다시 연다 (포트 이름이 바뀌어도 찾는다)."""
        with self.lock:
            try:
                self.port.closePort()
            except Exception:
                pass
            self._open()
        if self.cfg_args:
            self.configure(*self.cfg_args)

    # ---- 저수준 ----
    def _w(self, dxl_id, addr, size, val):
        f = {1: self.pk.write1ByteTxRx, 2: self.pk.write2ByteTxRx, 4: self.pk.write4ByteTxRx}[size]
        for _ in range(3):
            c, e = f(self.port, dxl_id, addr, int(val) & ((1 << (8 * size)) - 1))
            if c == self._ok:
                return
        raise BusError(f"쓰기 실패 ID{dxl_id} 주소 {addr}")

    def _r(self, dxl_id, addr, size):
        f = {1: self.pk.read1ByteTxRx, 2: self.pk.read2ByteTxRx, 4: self.pk.read4ByteTxRx}[size]
        for _ in range(3):
            v, c, e = f(self.port, dxl_id, addr)
            if c == self._ok:
                return v
        raise BusError(f"읽기 실패 ID{dxl_id} 주소 {addr}")

    # ---- 설정 ----
    def configure(self, profile_time_ms, profile_acc_ms, arm_pwm_ratio, watchdog_ms=0, pwm_per_joint=None,
                  gripper_gains=None):
        """arm_pwm_ratio: 기본 PWM 비율, pwm_per_joint: {관절 인덱스: 비율} 덮어쓰기.
        gripper_gains: {"p": .., "i": .., "d": ..} 그리퍼 위치 이득 (없거나 0이면 모터 값 그대로)."""
        self.cfg_args = (profile_time_ms, profile_acc_ms, arm_pwm_ratio, watchdog_ms, pwm_per_joint, gripper_gains)
        self.pwm_ratio = [(pwm_per_joint or {}).get(k, arm_pwm_ratio) for k in range(self.n - 1)] + [1.0]
        with self.lock:
            for i in self.ids:
                self._w(i, ADDR_PROF_ACC, 4, profile_acc_ms)
                self._w(i, ADDR_PROF_VEL, 4, profile_time_ms)
                self._w(i, ADDR_BUS_WDT, 1, 0)
            for k, i in enumerate(self.ids[:-1]):
                self._w(i, ADDR_GOAL_PWM, 2, int(PWM_LIMIT * self.pwm_ratio[k]))
            self._w(self.ids[-1], ADDR_GOAL_CUR, 2, self.gripper_current)
            g = self.ids[-1]
            for k, addr in (("p", ADDR_POS_P), ("i", ADDR_POS_I), ("d", ADDR_POS_D)):
                if (gripper_gains or {}).get(k):
                    self._w(g, addr, 2, int(gripper_gains[k]))
            self.gripper_gains_now = {k: self._r(g, a, 2) for k, a in (("p", ADDR_POS_P), ("i", ADDR_POS_I), ("d", ADDR_POS_D))}
            log.info("그리퍼 위치 이득 P %(p)d I %(i)d D %(d)d, 전류 한도 %(c)d mA",
                     {**self.gripper_gains_now, "c": self.gripper_current})
            if watchdog_ms:
                for i in self.ids:
                    self._w(i, ADDR_BUS_WDT, 1, max(1, round(watchdog_ms / 20)))

    def set_gripper_current(self, ma):
        """그리퍼 힘 = 전류기반 위치 모드의 Goal Current (mA). 물체에 닿으면 이 전류 이상으로 조이지 않는다."""
        with self.lock:
            self._w(self.ids[-1], ADDR_GOAL_CUR, 2, int(ma))
            self.gripper_current = int(ma)

    def torque(self, on, goal=None):
        """토크 켜기 전에 목표 = 현재 위치로 맞춰 튀는 것을 막는다."""
        with self.lock:
            if on:
                st = self._read_state_locked()
                self._write_goal_locked(st.q if goal is None else goal)
            for i in self.ids:
                self._w(i, ADDR_TORQUE, 1, 1 if on else 0)

    def torque_subset(self, idxs, on):
        """일부 관절만 토크 켜기/끄기 (프리드라이브). 켤 때는 그 관절 목표를 현재 위치로 맞춘다."""
        with self.lock:
            if on:
                st = self._read_state_locked()
                for k in idxs:
                    self._w(self.ids[k], ADDR_GOAL_POS, 4, rad2raw(st.q[k]) & 0xFFFFFFFF)
            for k in idxs:
                self._w(self.ids[k], ADDR_TORQUE, 1, 1 if on else 0)

    def torque_state(self):
        with self.lock:
            return [bool(self._r(i, ADDR_TORQUE, 1)) for i in self.ids]

    def read_hw_errors(self):
        with self.lock:
            for k, i in enumerate(self.ids):
                self.state.hw_err[k] = self._r(i, ADDR_HW_ERR, 1)
            return self.state.hw_err.copy()

    def reboot(self, idx):
        """하드웨어 오류 해제용 재부팅 (그 모터는 토크가 꺼진다)."""
        with self.lock:
            self.pk.reboot(self.port, self.ids[idx])
        time.sleep(0.6)
        if self.cfg_args:
            self.configure(*self.cfg_args)

    # ---- 주기 입출력 ----
    def _read_state_locked(self):
        c = self.sr.txRxPacket()
        st = self.state
        if c != self._ok:
            st.ok = False
            raise BusError("상태 동기 읽기 실패")
        for k, i in enumerate(self.ids):
            if not self.sr.isAvailable(i, ADDR_STATE, LEN_STATE):
                st.ok = False
                raise BusError(f"ID{i} 데이터 없음")
            cur = _s16(self.sr.getData(i, 126, 2))
            if self.models[i] == MODEL_XL430:
                st.load[k] = abs(cur) / 1000.0            # Present Load, 0.1% 단위
            elif k == self.n - 1:
                st.load[k] = abs(cur) / max(1, self.gripper_current)
            else:
                st.load[k] = abs(cur) / 1750.0            # XL330 전류 한계(mA)
            st.dq[k] = _s32(self.sr.getData(i, 128, 4)) * 0.229 * 2 * math.pi / 60
            st.q[k] = raw2rad(_s32(self.sr.getData(i, 132, 4)))
            st.volt[k] = self.sr.getData(i, 144, 2) / 10.0
            st.temp[k] = self.sr.getData(i, 146, 1)
        st.t = time.monotonic()
        st.ok = True
        return st

    def read_state(self):
        with self.lock:
            return self._read_state_locked()

    def _write_goal_locked(self, q):
        self.sw.clearParam()
        for i, a in zip(self.ids, q):
            v = rad2raw(a) & 0xFFFFFFFF
            self.sw.addParam(i, [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF])
        if self.sw.txPacket() != self._ok:
            raise BusError("목표 위치 쓰기 실패")

    def write_goal(self, q):
        with self.lock:
            self._write_goal_locked(q)

    def close(self):
        try:
            self.port.closePort()
        except Exception:
            pass


class SimBus:
    """로봇 없이 시험하기 위한 가상 로봇. 목표를 1차 지연으로 따라간다."""

    def __init__(self, n=6, q0=None, **_):
        self.n = n
        self.ids = list(range(n))
        self.state = RobotState(n)
        self.state.q[:] = q0 if q0 is not None else [0, -1.52, 1.57, 1.57, 0, 0]
        self.state.temp[:] = 30
        self.state.volt[:] = [12, 12, 12, 5, 5, 5]
        self.goal = self.state.q.copy()
        self.on = [False] * n
        self.last = time.monotonic()
        self.gripper_current = 250
        self.lock = threading.Lock()

    def set_gripper_current(self, ma):
        self.gripper_current = int(ma)

    pwm_ratio = [1.0] * 6
    port_name = "sim"

    def configure(self, *a, **k):
        pass

    def reconnect(self):
        pass

    def torque(self, on, goal=None):
        with self.lock:
            self.goal = self.state.q.copy() if goal is None else np.array(goal, float)
            self.on = [on] * self.n

    def torque_subset(self, idxs, on):
        with self.lock:
            for k in idxs:
                self.on[k] = on
                self.goal[k] = self.state.q[k]

    def torque_state(self):
        return list(self.on)

    def read_hw_errors(self):
        return self.state.hw_err.copy()

    def reboot(self, idx):
        pass

    def read_state(self):
        with self.lock:
            now = time.monotonic()
            dt, self.last = now - self.last, now
            if any(self.on):
                a = min(1.0, dt / 0.08) * np.array(self.on, float)
                new = self.state.q + (self.goal - self.state.q) * a
                self.state.dq = (new - self.state.q) / max(dt, 1e-3)
                self.state.q = new
            self.state.t = now
            self.state.ok = True
            return self.state

    def write_goal(self, q):
        with self.lock:
            self.goal = np.array(q, float)

    def close(self):
        pass
