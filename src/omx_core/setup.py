"""omx_core: 빌드할 때 네이티브 가속 라이브러리(omx/native/*.so)를 먼저 만든다 (tools/build_native.sh).
빌드에 실패해도 같은 계산을 하는 파이썬 경로로 동작한다."""
import subprocess
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

HERE = Path(__file__).resolve().parent


class BuildWithNative(build_py):
    def run(self):
        subprocess.call(["bash", str(HERE / "tools" / "build_native.sh")])
        super().run()


setup(
    name="omx_core",
    version="0.1.0",
    packages=find_packages(include=["omx", "omx.*"]),
    package_data={"omx": ["native/*.so", "web/static/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/omx_core"]),
        ("share/omx_core", ["package.xml"]),
        ("share/omx_core/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="MJW411",
    maintainer_email="206329414+MJW411@users.noreply.github.com",
    description="OMX 세컨드 배리어 핵심 라이브러리",
    license="Proprietary",
    cmdclass={"build_py": BuildWithNative},
)
