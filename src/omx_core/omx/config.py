"""설정 로드.

config/default.yaml(시스템 설정) + config/safety_limits.yaml(무인 시험 한도) 위에
data/settings.json(제어판에서 바꾼 값)을 덮어쓴다. 안전 한도는 제어판에서 바꿀 수 없다.
"""
import copy
import json
import os
import threading
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = ROOT / "config"
MODELS_DIR = Path.home() / "models"          # 읽기 전용


def _merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


class Config:
    def __init__(self, path=CONFIG_DIR / "default.yaml"):
        self._lock = threading.Lock()
        self.base = yaml.safe_load(Path(path).read_text())
        self.limits = yaml.safe_load((CONFIG_DIR / "safety_limits.yaml").read_text())
        # OMX_DATA_DIR: 시험용 가상 서버가 실제 서버의 data/(이벤트 DB·영상·설정)를 건드리지 않게 따로 둔다
        self.data_dir = (ROOT / (os.environ.get("OMX_DATA_DIR") or self.base["storage"]["data_dir"])).resolve()
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.settings_path = self.data_dir / "settings.json"
        self.overrides = {}
        if self.settings_path.exists():
            try:
                self.overrides = json.loads(self.settings_path.read_text())
            except json.JSONDecodeError:
                self.overrides = {}
        self.cfg = _merge(copy.deepcopy(self.base), self.overrides)

    def __getitem__(self, key):
        return self.cfg[key]

    def update(self, section, values):
        """제어판에서 바꾼 값을 저장한다 (section 단위)."""
        with self._lock:
            self.overrides.setdefault(section, {}).update(values)
            _merge(self.cfg, {section: values})
            tmp = self.settings_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.overrides, indent=2, ensure_ascii=False))
            tmp.replace(self.settings_path)

    def path(self, *parts):
        p = self.data_dir.joinpath(*parts)
        return p
