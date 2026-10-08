# 터미널에서 OMX 노드를 보거나 명령할 때: source ~/ros2_ws/src/omx_bringup/scripts/omx_env.sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1          # 노드끼리만 (DDS 가 Wi-Fi 로 나가지 않음)
export OMX_DATA_DIR=~/omx_data
