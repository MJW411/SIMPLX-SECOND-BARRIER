"""젯슨 자원 사용량 (CPU·RAM·GPU·온도·이 프로세스 메모리) — 최적화 평가용 표시."""
import os
import threading
import time

import psutil

GPU_LOAD = "/sys/devices/platform/bus@0/17000000.gpu/load"


def _read(p, default=None):
    try:
        with open(p) as f:
            return f.read().strip()
    except OSError:
        return default


class SysMon:
    def __init__(self):
        self.proc = psutil.Process(os.getpid())
        self.data = {}
        psutil.cpu_percent()
        threading.Thread(target=self._loop, daemon=True, name="sysmon").start()

    def _loop(self):
        while True:
            vm = psutil.virtual_memory()
            temps = {}
            for z in range(10):
                t = _read(f"/sys/class/thermal/thermal_zone{z}/type")
                v = _read(f"/sys/class/thermal/thermal_zone{z}/temp")
                if t and v and v.lstrip("-").isdigit():
                    temps[t.replace("-thermal", "")] = round(int(v) / 1000, 1)
            g = _read(GPU_LOAD)
            try:
                rss = self.proc.memory_info().rss
                pcpu = self.proc.cpu_percent()
            except psutil.Error:
                rss, pcpu = 0, 0
            self.data = {
                "cpu": psutil.cpu_percent(),
                "ram_used_gb": round((vm.total - vm.available) / 1e9, 2),
                "ram_total_gb": round(vm.total / 1e9, 2),
                "proc_rss_mb": round(rss / 1e6),
                "proc_cpu": pcpu,
                "gpu": round(int(g) / 10, 1) if g and g.isdigit() else None,
                "temp": temps,
            }
            time.sleep(2.0)

    def snapshot(self):
        return self.data
