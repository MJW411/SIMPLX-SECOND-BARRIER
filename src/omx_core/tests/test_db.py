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
deleted, used = db.prune()
left = {e["message"] for e in db.list()}
print("deleted", deleted, "used", used, "left", sorted(left))
ok = left == {"e1", "e2", "e4", "e5"} and db.usage() <= 800_000
print("FIFO(오래된 확인 이벤트부터 삭제, 화재·미확인 보호):", "OK" if ok else "FAIL")
# 보호분만으로 예산 초과 → 경고
db.budget = 300_000
db.prune()
ok2 = bool(db.last_prune["warn"]) and {"e1", "e2"} <= {e["message"] for e in db.list()}
print("보호분만으로 초과 시 경고:", "OK" if ok2 else "FAIL", db.last_prune["warn"])
ok = ok and ok2
sys.exit(0 if ok else 1)
