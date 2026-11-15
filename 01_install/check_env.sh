#!/usr/bin/env bash
# Isaac Sim 5.1 을 설치하기 전에 (또는 설치한 뒤에) 내 PC 를 점검합니다.
#
#   bash 01_install/check_env.sh
#
# Isaac Sim 을 ~/isaacsim 이 아닌 곳에 설치했다면:
#   ISAACSIM_PATH=/다른/경로 bash 01_install/check_env.sh

ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
MIN_DRIVER=580      # 공식 요구: Linux 580.65.06 이상
MIN_VRAM_MB=15000   # 16 GB 급
MIN_RAM_GB=30       # 32 GB 급 (표시는 31 안팎)
MIN_DISK_GB=50

fail=0
ok()   { printf '  [통과] %s\n' "$1"; }
warn() { printf '  [주의] %s\n' "$1"; }
bad()  { printf '  [실패] %s\n' "$1"; fail=1; }

echo "1. 운영체제"
os="$(. /etc/os-release && echo "$NAME $VERSION_ID")"
case "$os" in
  "Ubuntu 22.04"|"Ubuntu 24.04") ok "$os" ;;
  *) warn "$os (공식 지원은 Ubuntu 22.04 / 24.04)" ;;
esac

echo "2. GPU 와 드라이버"
if ! command -v nvidia-smi >/dev/null 2>&1; then
  bad "nvidia-smi 가 없습니다. NVIDIA 드라이버를 먼저 설치하세요."
else
  IFS=',' read -r name driver vram < <(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader,nounits | head -1)
  name="$(echo "$name" | xargs)"; driver="$(echo "$driver" | xargs)"; vram="$(echo "$vram" | xargs)"
  case "$name" in
    *RTX*) ok "GPU: $name" ;;
    *) bad "GPU: $name (RTX 계열이 필요합니다)" ;;
  esac
  if [ "${driver%%.*}" -ge "$MIN_DRIVER" ]; then ok "드라이버: $driver"; else bad "드라이버: $driver ($MIN_DRIVER 이상 필요)"; fi
  if [ "$vram" -ge "$MIN_VRAM_MB" ]; then ok "VRAM: $(((vram + 512) / 1024)) GB"; else warn "VRAM: $(((vram + 512) / 1024)) GB (16 GB 미만이면 큰 장면에서 모자랄 수 있음)"; fi
fi

echo "3. 메모리와 디스크"
ram_gb=$(awk '/MemTotal/ {printf "%.0f", $2 / 1024 / 1024}' /proc/meminfo)
if [ "$ram_gb" -ge "$MIN_RAM_GB" ]; then ok "RAM: ${ram_gb} GB"; else warn "RAM: ${ram_gb} GB (32 GB 권장)"; fi
disk_gb=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
if [ "$disk_gb" -ge "$MIN_DISK_GB" ]; then ok "홈 폴더 남은 공간: ${disk_gb} GB"; else bad "홈 폴더 남은 공간: ${disk_gb} GB (${MIN_DISK_GB} GB 이상 필요)"; fi

echo "4. Isaac Sim"
if [ -x "$ISAACSIM_PATH/isaac-sim.sh" ]; then
  ok "$ISAACSIM_PATH (버전 $(cut -d- -f1 "$ISAACSIM_PATH/VERSION"))"
else
  warn "$ISAACSIM_PATH 에 아직 없습니다. 01_install/install_isaacsim.sh 로 설치하세요."
fi

echo
if [ "$fail" -eq 0 ]; then echo "결과: 진행해도 됩니다."; else echo "결과: [실패] 항목을 먼저 해결하세요."; fi
exit "$fail"
