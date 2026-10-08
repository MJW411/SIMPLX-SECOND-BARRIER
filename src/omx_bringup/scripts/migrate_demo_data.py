#!/usr/bin/env python3
"""omx-safety-demo 의 데이터(티칭 자세·보정·비밀번호·이벤트 기록·사고 영상)를 새 데이터 폴더로 복사한다.

  python3 -I migrate_demo_data.py [--src ~/omx-safety-demo/data] [--dst ~/omx_data] [--force]

- demo 폴더는 읽기만 한다 (백업으로 그대로 둔다).
- 이벤트 DB 는 SQLite 백업 기능으로 복사한다 (쓰기 로그 WAL 까지 일관되게).
- 설정 보정:
  * perception.model → v2 통합 모델 (demo 는 옛 COCO 모델을 저장해 두었다)
  * 실제 사람(human) 거리 기준이 피규어와 같은 5cm 로 덮어써져 있으면 YAML 기본값(1.20/0.60/1.40 m)으로
- 옛 모델 변환 캐시(engines), 기록 파일(logs), 실행 잠금(omx.lock)은 옮기지 않는다.
"""
import argparse
import json
import os
import shutil
import sqlite3
import sys
from pathlib import Path

HUMAN_DEFAULT = {"slow": 1.20, "stop": 0.60, "clear": 1.40}
MODEL = "v2_best.engine"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="~/omx-safety-demo/data")
    ap.add_argument("--dst", default="~/omx_data")
    ap.add_argument("--force", action="store_true", help="이미 데이터가 있어도 덮어쓴다")
    a = ap.parse_args()
    src, dst = Path(a.src).expanduser(), Path(a.dst).expanduser()
    if not src.is_dir():
        sys.exit(f"원본 없음: {src}")
    if (dst / "events.db").exists() and not a.force:
        sys.exit(f"이미 데이터가 있음: {dst} (덮어쓰려면 --force)")
    dst.mkdir(parents=True, exist_ok=True)
    done = []
    for name in ("poses.json", "calib.json", "autocalib_samples.npz", "secret.key"):
        if (src / name).exists():
            shutil.copy2(src / name, dst / name)
            done.append(name)
    if (dst / "secret.key").exists():
        os.chmod(dst / "secret.key", 0o600)
    for d in ("calib_history", "backups", "models"):
        if (src / d).is_dir():
            shutil.copytree(src / d, dst / d, dirs_exist_ok=True)
            done.append(d + "/")
    (dst / "clips").mkdir(exist_ok=True)
    n = 0
    for f in (src / "clips").glob("*.mp4"):
        shutil.copy2(f, dst / "clips" / f.name)
        n += 1
    done.append(f"clips/ ({n}개)")

    s = sqlite3.connect(f"file:{src / 'events.db'}?mode=ro", uri=True)
    d = sqlite3.connect(str(dst / "events.db"))
    s.backup(d)
    rows = d.execute("SELECT COUNT(*) FROM events").fetchone()[0]
    d.close()
    s.close()
    done.append(f"events.db ({rows}건)")

    st = json.loads((src / "settings.json").read_text()) if (src / "settings.json").exists() else {}
    notes = []
    if st.get("perception", {}).get("model") != MODEL:
        notes.append(f"인식 모델 {st.get('perception', {}).get('model')} → {MODEL}")
        st.setdefault("perception", {})["model"] = MODEL
    prof = st.get("safety", {}).get("profiles", {})
    h = prof.get("human")
    if h and h.get("stop", 1.0) < 0.30:
        notes.append(f"실제 사람 기준 {h} → {HUMAN_DEFAULT}")
        prof["human"] = dict(HUMAN_DEFAULT)
    (dst / "settings.json").write_text(json.dumps(st, indent=2, ensure_ascii=False))
    done.append("settings.json")
    print("복사:", ", ".join(done))
    for x in notes:
        print("설정 보정:", x)
    print(f"완료 → {dst}")


if __name__ == "__main__":
    main()
