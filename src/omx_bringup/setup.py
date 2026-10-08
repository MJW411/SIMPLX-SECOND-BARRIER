from glob import glob

from setuptools import setup

setup(
    name="omx_bringup",
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/omx_bringup"]),
        ("share/omx_bringup", ["package.xml"]),
        ("share/omx_bringup/launch", glob("launch/*.py")),
        ("share/omx_bringup/systemd", glob("systemd/*")),
        ("share/omx_bringup/scripts", glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MJW411",
    maintainer_email="206329414+MJW411@users.noreply.github.com",
    description="OMX 세컨드 배리어 실행 구성",
    license="Proprietary",
)
