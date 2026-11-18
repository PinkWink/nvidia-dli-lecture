#!/usr/bin/env bash
# Isaac Sim 5.1 설치가 잘 되었는지 확인합니다. 파일을 보고, 실제로 한 번 켜 봅니다.
#
#   bash 01_install/verify_install.sh
#
# Isaac Sim 을 ~/isaacsim 이 아닌 곳에 설치했다면:
#   ISAACSIM_PATH=/다른/경로 bash 01_install/verify_install.sh

ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
WANT="5.1.0"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="${TMPDIR:-/tmp}/isaacsim_verify.log"

fail=0
ok()  { printf '  [통과] %s\n' "$1"; }
bad() { printf '  [실패] %s\n' "$1"; fail=1; }

echo "1. 설치된 파일"
for f in isaac-sim.sh python.sh; do
  if [ -x "$ISAACSIM_PATH/$f" ]; then ok "$f"; else bad "$f 가 없거나 실행 권한이 없습니다 ($ISAACSIM_PATH)"; fi
done
version="$(cut -d- -f1 "$ISAACSIM_PATH/VERSION" 2>/dev/null)"
if [ "$version" = "$WANT" ]; then ok "버전 $version"; else bad "버전이 '$version' 입니다 ($WANT 이어야 함)"; fi
if [ -e "$ISAACSIM_PATH/extension_examples" ]; then ok "post_install.sh 실행됨"; else bad "post_install.sh 를 아직 실행하지 않았습니다:  cd $ISAACSIM_PATH && ./post_install.sh"; fi

if [ "$fail" -ne 0 ]; then
  echo
  echo "결과: [실패] 항목을 먼저 해결하세요. (4절 '내려받기 · 풀기' 를 다시 확인)"
  exit 1
fi

echo "2. 실제로 켜 보기 (창 없이, 처음에는 오래 걸릴 수 있습니다)"
start=$(date +%s)
"$ISAACSIM_PATH/python.sh" "$HERE/verify_install.py" > "$LOG" 2>&1
code=$?
took=$(( $(date +%s) - start ))
started="$(grep -a '^VERIFY 버전' "$LOG" | awk '{print $3}')"
z="$(grep -a '^VERIFY 높이' "$LOG" | awk '{print $3}')"
if [ -n "$started" ]; then ok "Isaac Sim $started 이 켜졌습니다"; else bad "Isaac Sim 이 켜지지 않았습니다"; fi
if [ -n "$z" ] && awk -v z="$z" 'BEGIN { exit !(z < 4.5) }'; then
  ok "물리 시뮬레이션: 큐브가 0.5초에 5.00 m → $z m 로 떨어졌습니다"
else
  bad "물리 시뮬레이션이 돌지 않았습니다 (큐브 높이: ${z:-알 수 없음})"
fi
if [ "$code" -eq 0 ]; then ok "정상 종료 (${took}초)"; else bad "비정상 종료 (종료 코드 $code)"; fi

echo
if [ "$fail" -eq 0 ]; then
  echo "결과: 설치가 정상입니다."
else
  echo "결과: [실패] 항목이 있습니다. 자세한 로그: $LOG"
fi
exit "$fail"
