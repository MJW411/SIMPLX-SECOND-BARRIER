from setuptools import find_packages, setup

setup(
    name="omx_nodes",
    version="0.1.0",
    packages=find_packages(include=["omx_nodes", "omx_nodes.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/omx_nodes"]),
        ("share/omx_nodes", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MJW411",
    maintainer_email="206329414+MJW411@users.noreply.github.com",
    description="OMX 세컨드 배리어 ROS 2 노드",
    license="Proprietary",
    entry_points={"console_scripts": [
        f"{n}_node = omx_nodes.{n}_node:main" for n in ("perception", "safety", "robot", "io", "web")
    ]},
)
