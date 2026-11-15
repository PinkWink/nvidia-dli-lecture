#!/usr/bin/env bash
# Isaac Sim 5.1.0 (Linux x86_64) 을 내려받아 ~/isaacsim 에 설치합니다.
#
#   bash 01_install/install_isaacsim.sh
#
# 내려받기 약 8.8 GB, 풀면 약 17 GB 입니다. 중간에 끊기면 다시 실행하세요 (이어받습니다).
# 설치 위치를 바꾸려면:  ISAACSIM_PATH=/다른/경로 bash 01_install/install_isaacsim.sh
set -euo pipefail

VERSION="5.1.0"
ZIP="isaac-sim-standalone-${VERSION}-linux-x86_64.zip"
URL="https://downloads.isaacsim.nvidia.com/${ZIP}"
ZIP_BYTES=8768419777
ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
DOWNLOAD_DIR="${DOWNLOAD_DIR:-$HOME/Downloads}"

if [ -e "$ISAACSIM_PATH/isaac-sim.sh" ]; then
  echo "이미 설치되어 있습니다: $ISAACSIM_PATH ($(cat "$ISAACSIM_PATH/VERSION"))"
  exit 0
fi

for tool in wget unzip; do
  command -v "$tool" >/dev/null 2>&1 || { echo "$tool 이 없습니다:  sudo apt install -y wget unzip"; exit 1; }
done

mkdir -p "$DOWNLOAD_DIR"
echo "[1/3] 내려받기: $DOWNLOAD_DIR/$ZIP"
wget -c -P "$DOWNLOAD_DIR" "$URL"
size=$(stat -c %s "$DOWNLOAD_DIR/$ZIP")
if [ "$size" -ne "$ZIP_BYTES" ]; then
  echo "파일 크기가 다릅니다 ($size, 기대값 $ZIP_BYTES). 다시 실행해 이어받으세요."
  exit 1
fi

echo "[2/3] 압축 풀기: $ISAACSIM_PATH (1~2분)"
mkdir -p "$ISAACSIM_PATH"
unzip -q "$DOWNLOAD_DIR/$ZIP" -d "$ISAACSIM_PATH"

echo "[3/3] 설치 마무리 (post_install.sh)"
(cd "$ISAACSIM_PATH" && ./post_install.sh)

echo
echo "설치 완료: $ISAACSIM_PATH ($(cat "$ISAACSIM_PATH/VERSION"))"
echo "다음: $ISAACSIM_PATH/isaac-sim.compatibility_check.sh  →  $ISAACSIM_PATH/isaac-sim.sh"
