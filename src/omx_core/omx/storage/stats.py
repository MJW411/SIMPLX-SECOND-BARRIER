"""대시보드 통계: 일별 작업량·사이클 시간, 이벤트 발생률 (이벤트 DB 에서 계산).

작업량은 작업 1회마다 남기는 cycle_tick(걸린 초)을 센다. cycle_tick 이 없는 옛날 날짜는
작업 완료·정지 이벤트의 "N회 반복" / "(N회 완료)" 숫자로 대신한다.
"""
import re
import time

from ..events import INCIDENTS, STORED

_N_DONE = re.compile(r"(\d+)회 반복")
_N_STOP = re.compile(r"\((\d+)회 완료\)")


def _day(t):
    return time.strftime("%Y-%m-%d", time.localtime(t))


def compute(rows, days, now=None):
    """rows: [(t, type, message)] (기간 안), days: 기간(일). 결과는 제어판 '통계' 화면 그대로."""
    now = time.time() if now is None else now
    lt = time.localtime(now)
    today0 = time.mktime((lt.tm_year, lt.tm_mon, lt.tm_mday, 0, 0, 0, 0, 0, -1))
    keys = [_day(today0 - 86400 * k + 3600) for k in range(days - 1, -1, -1)]   # +1시간: 서머타임 경계 안전
    per = {k: {"date": k, "cycles": 0, "legacy": 0, "secs": [], "incidents": {}, "events": 0} for k in keys}
    by_hour = [0] * 24
    by_type = {}
    has_tick = set()
    for t, typ, msg in rows:
        k = _day(t)
        d = per.get(k)
        if d is None:
            continue
        if typ == "cycle_tick":
            d["cycles"] += 1
            has_tick.add(k)
            by_hour[time.localtime(t).tm_hour] += 1
            try:
                d["secs"].append(float(msg))
            except (TypeError, ValueError):
                pass
            continue
        if typ in ("cycle_done", "cycle_stop"):
            m = _N_DONE.search(msg or "") or _N_STOP.search(msg or "")
            if m:
                d["legacy"] += int(m.group(1))
        if typ in STORED:
            by_type[typ] = by_type.get(typ, 0) + 1
            d["events"] += 1
        if typ in INCIDENTS:
            d["incidents"][typ] = d["incidents"].get(typ, 0) + 1
    out_days, secs_all = [], []
    for k in keys:
        d = per[k]
        n = d["cycles"] if k in has_tick else d["legacy"]
        secs_all += d["secs"]
        out_days.append({"date": k, "cycles": n, "avg_cycle_s": round(sum(d["secs"]) / len(d["secs"]), 2) if d["secs"] else None,
                         "incidents": d["incidents"], "incident_total": sum(d["incidents"].values())})
    cycles = sum(x["cycles"] for x in out_days)
    inc = {t: n for t, n in by_type.items() if t in INCIDENTS}
    inc_total = sum(inc.values())
    active = [x for x in out_days if x["cycles"] > 0]
    rate = {t: {"n": n, "per_day": round(n / days, 2), "per_100": round(100 * n / cycles, 2) if cycles else None}
            for t, n in sorted(by_type.items(), key=lambda kv: -kv[1])}
    return {
        "days": days, "series": out_days, "by_hour": by_hour, "incidents": inc, "rate": rate,
        "totals": {
            "cycles": cycles,
            "avg_per_day": round(cycles / days, 1),
            "avg_per_active_day": round(cycles / len(active), 1) if active else 0,
            "active_days": len(active),
            "avg_cycle_s": round(sum(secs_all) / len(secs_all), 2) if secs_all else None,
            "incidents": inc_total,
            "incidents_per_100": round(100 * inc_total / cycles, 2) if cycles else None,
            "cycles_per_incident": round(cycles / inc_total, 1) if inc_total else None,
            "top_incident": max(inc, key=inc.get) if inc else None,
        },
    }
