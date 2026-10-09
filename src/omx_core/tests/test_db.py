"""데이터 예산 FIFO 시험: 작은 예산으로 오래된 확인 이벤트부터 지우고, 화재·미확인은 보호."""
import sys, tempfile, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from omx.storage.db import EventDB

d = Path(tempfile.mkdtemp())
(d / "clips").mkdir()
db = EventDB(d / "events.db", d, budget_bytes=800_000)
ids = []
for i, t in enumerate(["human_stop", "fire", "vanish", "human_stop", "conveyor_jam", "human_stop"]):
    eid = db.add(t, "alarm", f"e{i}")
    p = d / "clips" / f"c{i}.mp4"
    p.write_bytes(b"0" * 150_000)
    db.set_clip(eid, str(p), 150_000, "ready")
    ids.append(eid)
    time.sleep(0.01)
for eid in ids[:5]:
    if eid != ids[2]:            # vanish(ids[2])는 미확인으로 남긴다
        db.ack(eid, "t")
overhead = db.usage(fresh=True) - 6 * 150_000           # DB 파일(WAL 포함)도 예산에 들어간다
db.budget = 700_000 + overhead            # 클립 6개(900k) 중 2개를 지워야 맞는 예산
deleted, used = db.prune()
left = {e["message"] for e in db.list()}
print("deleted", deleted, "used", used, "left", sorted(left))
ok = left == {"e1", "e2", "e4", "e5"} and db.usage(fresh=True) <= db.budget
print("FIFO(오래된 확인 이벤트부터 삭제, 화재·미확인 보호):", "OK" if ok else "FAIL")
# 보호분만으로 예산 초과 → 경고
db.budget = 300_000 + overhead
db.prune()
ok2 = bool(db.last_prune["warn"]) and {"e1", "e2"} <= {e["message"] for e in db.list()}
print("보호분만으로 초과 시 경고:", "OK" if ok2 else "FAIL", db.last_prune["warn"])
ok = ok and ok2

# ---- 실제 운영처럼 클립 '이름만' 저장된 경우 (예전 버그: 실행 폴더 기준으로 지워 파일이 남았다)
import os  # noqa: E402
d2 = Path(tempfile.mkdtemp())
(d2 / "clips").mkdir()
db2 = EventDB(d2 / "events.db", d2, budget_bytes=200_000)
names = []
for i in range(3):
    eid = db2.add("human_stop", "alarm", f"n{i}")
    f = d2 / "clips" / f"n{i}.mp4"
    f.write_bytes(b"0" * 150_000)
    db2.set_clip(eid, f.name, 150_000, "ready")
    db2.ack(eid, "t")
    names.append(f)
    time.sleep(0.01)
cwd = os.getcwd()
db2.budget = 200_000 + db2.usage(fresh=True) - 3 * 150_000
os.chdir(tempfile.mkdtemp())                      # 실행 폴더가 데이터 폴더와 다를 때
db2.prune()
os.chdir(cwd)
ok3 = not names[0].exists() and not names[1].exists() and names[2].exists() and db2.usage(fresh=True) <= db2.budget
print("이름만 저장된 클립도 실제 파일 삭제:", "OK" if ok3 else "FAIL", [n.exists() for n in names])

# ---- 같은 이벤트 키는 한 번만, 시작 시 정리 (녹화 중에 멈춘 행 → 실패, DB 에 없는 오래된 파일 삭제)
k1 = db2.add("vanish", "alarm", "k", key="safety-1", clip_status="recording")
k2 = db2.add("vanish", "alarm", "k", key="safety-1", clip_status="recording")
orphan = d2 / "clips" / "orphan.mp4"
orphan.write_bytes(b"x")
os.utime(orphan, (time.time() - 3600, time.time() - 3600))
fresh = d2 / "clips" / "fresh.mp4"
fresh.write_bytes(b"x")
db2.budget = 10 ** 12
stuck, gone = db2.cleanup()
ok4 = (k1 == k2 and db2.id_for_key("safety-1") == k1 and db2.get(k1)["clip_status"] == "failed"
       and not orphan.exists() and fresh.exists() and names[2].exists())
print("이벤트 키 중복 방지·시작 시 정리:", "OK" if ok4 else "FAIL", stuck, gone)
# ---- 사고 현황 집계 (대시보드 도넛) · 이벤트 분류
import tempfile as _tf, time as _tm  # noqa: E402
from omx.events import EVENT_CATS, INCIDENTS, STORED  # noqa: E402
_d = _tf.mkdtemp()
_db = EventDB(_d + "/ev.db", _d)
_now = _tm.time()
for _t, _ago in (("human_stop", 10), ("human_stop", 20), ("conveyor_jam", 30), ("vanish", 9 * 86400), ("cycle_done", 5)):
    _db.add(_t, "alarm", "시험", t=_now - _ago)
_c = _db.type_counts(INCIDENTS, _now - 7 * 86400)
assert _c == {"human_stop": {"n": 2, "unacked": 2}, "conveyor_jam": {"n": 1, "unacked": 1}}, _c
assert _db.type_counts(INCIDENTS)["vanish"]["n"] == 1, "기간 없으면 전체"
_all = [t for v in EVENT_CATS.values() for t in v]
assert sorted(_all) == sorted(STORED) and len(_all) == len(set(_all)), "모든 저장 종류가 분류 하나에만 속함"
assert set(INCIDENTS) <= STORED
print("사고 현황 집계·이벤트 분류: OK")
ok = ok and ok3 and ok4
sys.exit(0 if ok else 1)

