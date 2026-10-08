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
# 설정 폴더: OMX_CONFIG_DIR (ROS 설치본은 launch 가 share/omx_bringup/config 로 지정), 없으면 소스 트리의 config/
def _config_dir():
    if os.environ.get("OMX_CONFIG_DIR"):
        return Path(os.environ["OMX_CONFIG_DIR"])
    if (ROOT / "config").is_dir():
        return ROOT / "config"
    try:                                     # colcon 설치본: share/omx_core/config
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("omx_core")) / "config"
    except Exception:
        return ROOT / "config"


CONFIG_DIR = _config_dir()
MODELS_DIR = Path.home() / "models"          # 읽기 전용
# 데이터 폴더(DB·영상·설정·보정): OMX_DATA_DIR. 상대 경로면 소스 트리 기준(기존 시험 도구 호환), 없으면 ~/omx_data
DEFAULT_DATA_DIR = Path.home() / "omx_data"


def _merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


class Config:
    """여러 프로세스(ROS 노드)가 같은 settings.json 을 쓴다. 저장할 때 파일 잠금 아래에서 최신 파일을 다시 읽고
    그 위에 바꾼 섹션만 덮어써서, 다른 노드가 방금 저장한 값을 지우지 않는다. 다른 노드의 변경은 reload()로 반영."""

    def __init__(self, path=None):
        self._lock = threading.Lock()
        path = Path(path) if path else CONFIG_DIR / "default.yaml"
        self.base = yaml.safe_load(path.read_text())
        self.limits = yaml.safe_load((path.parent / "safety_limits.yaml").read_text())
        env = os.environ.get("OMX_DATA_DIR")
        if env:
            self.data_dir = (ROOT / env).resolve()            # 절대 경로면 그대로
        else:
            self.data_dir = DEFAULT_DATA_DIR
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.settings_path = self.data_dir / "settings.json"
        self.overrides = self._read()
        self.cfg = _merge(copy.deepcopy(self.base), self.overrides)

    def _read(self):
        if self.settings_path.exists():
            try:
                return json.loads(self.settings_path.read_text())
            except json.JSONDecodeError:
                return {}
        return {}

    def __getitem__(self, key):
        return self.cfg[key]

    def reload(self):
        """다른 프로세스가 저장한 값을 반영한다. 최상위 dict 는 그대로 두고 내용만 바꾼다(참조 유지)."""
        with self._lock:
            self.overrides = self._read()
            merged = _merge(copy.deepcopy(self.base), self.overrides)
            for k in list(self.cfg):
                if k not in merged:
                    del self.cfg[k]
            for k, v in merged.items():
                if isinstance(v, dict) and isinstance(self.cfg.get(k), dict):
                    self.cfg[k].clear()
                    self.cfg[k].update(v)
                else:
                    self.cfg[k] = v

    def update(self, section, values):
        """제어판에서 바꾼 값을 저장한다 (section 단위)."""
        import fcntl
        with self._lock:
            with open(self.data_dir / ".settings.lock", "w") as lk:
                fcntl.flock(lk, fcntl.LOCK_EX)
                self.overrides = self._read()                  # 다른 노드가 저장한 최신 값 위에
                self.overrides.setdefault(section, {}).update(values)
                tmp = self.settings_path.with_suffix(".tmp")
                tmp.write_text(json.dumps(self.overrides, indent=2, ensure_ascii=False))
                tmp.replace(self.settings_path)
            merged = _merge(copy.deepcopy(self.base), self.overrides)
            for k, v in merged.items():
                if isinstance(v, dict) and isinstance(self.cfg.get(k), dict):
                    self.cfg[k].clear()
                    self.cfg[k].update(v)
                else:
                    self.cfg[k] = v

    def path(self, *parts):
        p = self.data_dir.joinpath(*parts)
        return p
