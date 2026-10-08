"""SQLite 이벤트 DB + 데이터 예산(FIFO).

- 이벤트: 사람 접근/정지, 사라짐, 컨베이어 이상, 화재, 고장 등과 클립 경로.
- 데이터 예산 = SSD 전체 용량 x budget_ratio(기본 50%). 데이터 폴더가 예산을 넘으면
  확인된(acked) 일반 이벤트부터 오래된 순으로 클립과 행을 지운다.
  화재 이벤트와 미확인 이벤트는 보호한다(지우지 않음). 보호분만으로 넘으면 경고.
"""
import os
import shutil
import sqlite3
import threading
import time
from pathlib import Path

def _stored_sql():
    """이벤트 로그 정책의 종류 목록 (SQL IN 절용). 정책 밖 옛 기록은 확인된 것처럼 다룬다(공간 정리 대상)."""
    from ..events import STORED
    return ",".join(f"'{t}'" for t in sorted(STORED))


SCHEMA = """
CREATE TABLE IF NOT EXISTS events (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  t REAL NOT NULL,
  type TEXT NOT NULL,
  level TEXT NOT NULL,
  message TEXT NOT NULL,
  state TEXT,
  clip TEXT,
  clip_status TEXT,          -- none | recording | encoding | ready | failed | deleted
  clip_bytes INTEGER DEFAULT 0,
  acked INTEGER DEFAULT 0,
  acked_by TEXT,
  acked_t REAL
);
CREATE INDEX IF NOT EXISTS idx_events_t ON events(t);
CREATE INDEX IF NOT EXISTS idx_events_type_acked ON events(type, acked);
"""


class EventDB:
    USAGE_TTL = 300.0           # 데이터 폴더 전체 크기는 5분마다만 다시 잰다 (그 사이는 클립 크기로 더하고 뺀다)
    STATS_TTL = 2.0

    def __init__(self, path, data_dir, budget_ratio=0.5, budget_bytes=None, clip_dir=None):
        self.path = Path(path)
        self.data_dir = Path(data_dir)
        # DB 에는 클립 파일 이름만 저장한다 → 지울 때는 반드시 이 폴더 기준 (예전: 실행 폴더 기준이라 안 지워졌다)
        self.clip_dir = Path(clip_dir) if clip_dir else self.data_dir / "clips"
        self.lock = threading.Lock()
        self.conn = sqlite3.connect(str(self.path), check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self.conn.executescript(SCHEMA)
        cols = {r[1] for r in self.conn.execute("PRAGMA table_info(events)")}
        if "ekey" not in cols:                   # 노드가 만든 이벤트 키 (같은 이벤트를 두 번 저장하지 않게)
            self.conn.execute("ALTER TABLE events ADD COLUMN ekey TEXT")
        self.conn.execute("CREATE UNIQUE INDEX IF NOT EXISTS idx_events_ekey ON events(ekey)")
        self.conn.execute("PRAGMA journal_mode=WAL")
        self.conn.commit()
        total = shutil.disk_usage(self.data_dir).total
        self.budget = budget_bytes if budget_bytes else int(total * budget_ratio)
        self.last_prune = {"deleted": 0, "warn": ""}
        self._usage = None                       # (bytes, 잰 시각)
        self._stats = None

    def add(self, etype, level, message, state=None, clip_status="none", key=None, t=None):
        """key 가 이미 있으면 새로 넣지 않고 그 행의 id 를 돌려준다."""
        with self.lock:
            if key is not None:
                r = self.conn.execute("SELECT id FROM events WHERE ekey=?", (key,)).fetchone()
                if r:
                    return r[0]
            cur = self.conn.execute(
                "INSERT INTO events(t,type,level,message,state,clip_status,ekey) VALUES(?,?,?,?,?,?,?)",
                (t or time.time(), etype, level, message, state, clip_status, key))
            self.conn.commit()
            self._stats = None
            return cur.lastrowid

    def id_for_key(self, key):
        with self.lock:
            r = self.conn.execute("SELECT id FROM events WHERE ekey=?", (key,)).fetchone()
            return r[0] if r else None

    def set_clip(self, eid, path, size, status):
        with self.lock:
            self.conn.execute("UPDATE events SET clip=?, clip_bytes=?, clip_status=? WHERE id=?",
                              (str(path) if path else None, size, status, eid))
            self.conn.commit()
            if self._usage is not None and size:
                self._usage = (self._usage[0] + int(size), self._usage[1])

    def _clip_path(self, clip):
        p = Path(clip)
        return p if p.is_absolute() else self.clip_dir / p.name

    def cleanup(self, orphan_age=600.0):
        """시작할 때: 프로그램이 녹화 중에 끝나 '녹화 중'에 멈춘 행을 '실패'로, DB 에 없는 오래된 클립 파일을 지운다."""
        with self.lock:
            n_stuck = self.conn.execute(
                "UPDATE events SET clip_status='failed' WHERE clip_status IN ('recording','encoding')").rowcount
            self.conn.commit()
            known = {Path(r[0]).name for r in self.conn.execute("SELECT clip FROM events WHERE clip IS NOT NULL")}
        n_orphan = 0
        if self.clip_dir.is_dir():
            now = time.time()
            for f in self.clip_dir.glob("*.mp4"):
                try:
                    if f.name not in known and now - f.stat().st_mtime > orphan_age:
                        f.unlink()
                        n_orphan += 1
                except OSError:
                    pass
        return n_stuck, n_orphan

    def set_clip_status(self, eid, status):
        with self.lock:
            self.conn.execute("UPDATE events SET clip_status=? WHERE id=?", (status, eid))
            self.conn.commit()

    def ack(self, eid, user):
        with self.lock:
            self.conn.execute("UPDATE events SET acked=1, acked_by=?, acked_t=? WHERE id=?", (user, time.time(), eid))
            self.conn.commit()
            self._stats = None

    def ack_all(self, user):
        with self.lock:
            self.conn.execute("UPDATE events SET acked=1, acked_by=?, acked_t=? WHERE acked=0", (user, time.time()))
            self.conn.commit()
            self._stats = None

    def list(self, limit=100, before=None, types=None):
        with self.lock:
            q = "SELECT * FROM events WHERE 1=1"
            args = []
            if types:
                q += f" AND type IN ({','.join('?' * len(types))})"
                args += list(types)
            if before:
                q += " AND id < ?"
                args.append(before)
            q += " ORDER BY id DESC LIMIT ?"
            args.append(limit)
            return [dict(r) for r in self.conn.execute(q, args)]

    CLIP_ANY = "clip_status IN ('recording','encoding','ready','failed')"

    def list_clips(self, types=None, exclude=None, limit=60, before=None, t_range=None):
        """영상이 있는 이벤트만 (영상 메뉴). types 에 든 종류만, 또는 exclude 에 든 종류를 뺀 나머지."""
        q, args = f"SELECT * FROM events WHERE {self.CLIP_ANY}", []
        for col, vals, op in (("type", types, "IN"), ("type", exclude, "NOT IN")):
            if vals:
                q += f" AND {col} {op} ({','.join('?' * len(vals))})"
                args += list(vals)
        if before:
            q += " AND id < ?"
            args.append(before)
        if t_range:
            q += " AND t >= ? AND t < ?"
            args += list(t_range)
        q += " ORDER BY id DESC LIMIT ?"
        args.append(limit)
        with self.lock:
            return [dict(r) for r in self.conn.execute(q, args)]

    def clip_counts(self):
        """종류별 영상 수와 미확인 수."""
        with self.lock:
            return {r[0]: {"n": r[1], "unacked": r[2]} for r in self.conn.execute(
                f"SELECT type, COUNT(*), SUM(acked = 0) FROM events WHERE {self.CLIP_ANY} GROUP BY type")}

    def get(self, eid):
        with self.lock:
            r = self.conn.execute("SELECT * FROM events WHERE id=?", (eid,)).fetchone()
            return dict(r) if r else None

    def usage(self, fresh=False):
        now = time.monotonic()
        if not fresh and self._usage is not None and now - self._usage[1] < self.USAGE_TTL:
            return self._usage[0]
        n = self._walk()
        self._usage = (n, now)
        return n

    def _walk(self):
        n = 0
        for root, _, files in os.walk(self.data_dir):
            for f in files:
                try:
                    n += os.path.getsize(os.path.join(root, f))
                except OSError:
                    pass
        return n

    def prune(self):
        """예산 초과 시 FIFO 삭제. 보호: 화재, 미확인, 녹화/인코딩 중."""
        used = self.usage()
        deleted = 0
        warn = ""
        while used > self.budget:
            with self.lock:
                r = self.conn.execute(
                    f"SELECT id, clip, clip_bytes FROM events WHERE (acked=1 OR type NOT IN ({_stored_sql()})) AND type NOT IN ('fire','fire_end') "
                    "AND clip_status NOT IN ('recording','encoding') ORDER BY t ASC LIMIT 1").fetchone()
                if r is None:
                    warn = "보호 이벤트(화재·미확인)만으로 예산 초과 — 확인 처리 또는 백업 필요"
                    break
                if r["clip"]:
                    try:
                        os.unlink(self._clip_path(r["clip"]))
                    except FileNotFoundError:
                        pass
                self.conn.execute("DELETE FROM events WHERE id=?", (r["id"],))
                self.conn.commit()
            used -= r["clip_bytes"] or 0
            used = self.usage(fresh=True) if deleted % 20 == 19 else used
            deleted += 1
        if deleted:
            self._usage = (used, time.monotonic())
            self._stats = None
        self.last_prune = {"deleted": deleted, "warn": warn, "t": time.time()}
        return deleted, used

    def stats(self):
        now = time.monotonic()
        if self._stats is not None and now - self._stats[1] < self.STATS_TTL:
            return self._stats[0]
        with self.lock:
            # 이벤트 로그 정책(events.STORED) 밖의 옛 기록(감속·토크 등)은 세지 않는다
            n = self.conn.execute(f"SELECT COUNT(*) FROM events WHERE type IN ({_stored_sql()})").fetchone()[0]
            un = self.conn.execute(f"SELECT COUNT(*) FROM events WHERE acked=0 AND type IN ({_stored_sql()})").fetchone()[0]
        st = {"events": n, "unacked": un, "budget_gb": round(self.budget / 1e9, 1)}
        self._stats = (st, now)
        return st
