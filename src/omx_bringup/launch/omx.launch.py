"""OMX 세컨드 배리어 전체 실행 (개발·시연용). 운영은 systemd 사용자 서비스(systemd/)를 권장.

  ros2 launch omx_bringup omx.launch.py                     # 실제 로봇·카메라·아두이노
  ros2 launch omx_bringup omx.launch.py sim:=true           # 가상 로봇 + 가상 장치 (카메라는 실제)
  ros2 launch omx_bringup omx.launch.py sim:=true video:=/경로/영상.mp4   # 장치 없이 시험 (입구 카메라도 안 씀)

노드 5개는 각각 별도 프로세스다. 하나가 죽으면 3초 뒤 다시 띄운다 (respawn). 다시 떠도 로봇은 정지 상태로 시작한다.
노드는 ~/ros2_ws/.venv 파이썬으로 실행한다 (FastAPI·RealSense·TensorRT 를 함께 쓰기 위해).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

NODES = ("robot", "safety", "io", "perception", "web")


def make(context):
    py = LaunchConfiguration("python").perform(context)
    sim = LaunchConfiguration("sim").perform(context).lower() in ("1", "true", "yes")
    video = LaunchConfiguration("video").perform(context)
    data = os.path.expanduser(LaunchConfiguration("data_dir").perform(context))
    env = {"OMX_DATA_DIR": data, "ROS_LOCALHOST_ONLY": "1", "PYTHONUNBUFFERED": "1",
           "OMX_WEB_PORT": LaunchConfiguration("port").perform(context)}
    extra = {n: [] for n in NODES}
    if sim:
        extra["robot"].append("--sim")
        extra["io"].append("--virtual")
    if video:
        extra["perception"] += ["--video", video]
        if sim:
            extra["perception"].append("--no-helmet")      # 시험: 실제 장치를 잡지 않는다
    return [ExecuteProcess(cmd=[py, "-m", f"omx_nodes.{n}_node", *extra[n]], name=f"omx_{n}", output="screen",
                           respawn=True, respawn_delay=3.0, additional_env=env, sigterm_timeout="5")
            for n in NODES]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("python", default_value=os.path.expanduser("~/ros2_ws/.venv/bin/python")),
        DeclareLaunchArgument("sim", default_value="false", description="가상 로봇 + 가상 장치"),
        DeclareLaunchArgument("video", default_value="", description="카메라 대신 영상 파일"),
        DeclareLaunchArgument("data_dir", default_value="~/omx_data"),
        DeclareLaunchArgument("port", default_value="8000"),
        OpaqueFunction(function=make),
    ])
