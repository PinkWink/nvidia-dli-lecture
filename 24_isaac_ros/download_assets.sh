#!/bin/bash
# 강의 24: isaac_ros_unet 예제 자료(녹화 영상 quickstart.bag 등)를 받는다. Isaac ROS 컨테이너 안에서 실행한다.
#   bash /workspaces/isaac_ros-dev/download_assets.sh
# NGC 에서 Isaac ROS 4.0 에 맞는 가장 새 자료 버전을 찾아 ${ISAAC_ROS_WS}/isaac_ros_assets 에 푼다.
set -e
API="https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/isaac_ros_unet_assets/versions"
MAJOR=4
MINOR=0

version=$(curl -s -H "Accept: application/json" "$API" \
    | jq -r '.recipeVersions[].versionId' \
    | grep -E '^[0-9]+\.[0-9]+\.[0-9]+$' \
    | awk -F. -v M=$MAJOR -v m=$MINOR '$1 == M && $2 <= m' \
    | sort -V | tail -n 1)
if [ -z "$version" ]; then
    echo "Isaac ROS $MAJOR.$MINOR 에 맞는 자료 버전을 찾지 못했습니다."
    exit 1
fi
echo "자료 버전: $version"

mkdir -p "${ISAAC_ROS_WS}/isaac_ros_assets"
cd /tmp
curl -L -o quickstart.tar.gz "$API/$version/files/quickstart.tar.gz"
tar -xf quickstart.tar.gz -C "${ISAAC_ROS_WS}/isaac_ros_assets"
rm quickstart.tar.gz
echo "받은 것:"
ls "${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_unet"
