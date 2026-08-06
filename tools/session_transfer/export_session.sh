#!/usr/bin/env bash
# =============================================================================
# export_session.sh — Claude Code 세션 이관 번들러 (원본 PC에서 실행)
#
#   목적: 대화 이력(transcript) + 프로젝트 코드(미커밋 포함) + 메모리를
#         tar 하나로 묶어, 다른 PC/계정에서 `--resume` 로 이어갈 수 있게 한다.
#
#   사용법:
#       ./export_session.sh [SESSION_ID] [출력파일.tgz]
#       ./export_session.sh --full [SESSION_ID] [출력파일.tgz]
#
#     - SESSION_ID 생략 시: 프로젝트의 가장 최근 세션을 자동 선택
#     - 출력파일 생략 시: ~/fo_session_<shortid>_<날짜>.tgz
#     - 기본은 빌드 산출물(build/install/log)·rosbag 데이터를 제외한 '경량' 번들.
#       세션 이어가기(a)엔 이걸로 충분. --full 이면 전부 포함(수 GB, 느림).
#
#   대상 PC에서는 tar 를 풀고 안에 들어있는 import_session.sh 만 실행하면 됨.
# =============================================================================
set -euo pipefail

# ---- --full 플래그 파싱 -----------------------------------------------------
FULL=0
if [ "${1:-}" = "--full" ]; then FULL=1; shift; fi

# ---- 고정 규약 (프로젝트 경로에 강결합; README 참고) ------------------------
PROJECT_DIR="/home/wise/fo_perception"
ENCODED="-home-wise-fo-perception"          # Claude Code 프로젝트 폴더 인코딩명
CLAUDE_PROJ="$HOME/.claude/projects/$ENCODED"

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; RST=$'\033[0m'
die()  { echo "${RED}[에러]${RST} $*" >&2; exit 1; }
info() { echo "${GRN}[+]${RST} $*"; }
warn() { echo "${YEL}[!]${RST} $*"; }

# ---- 사전 점검 --------------------------------------------------------------
[ -d "$PROJECT_DIR" ]  || die "프로젝트 경로 없음: $PROJECT_DIR"
[ -d "$CLAUDE_PROJ" ]  || die "Claude 세션 폴더 없음: $CLAUDE_PROJ"

# ---- 세션 ID 결정 (인자 없으면 최신 .jsonl 자동 선택) -----------------------
SESSION_ID="${1:-}"
if [ -z "$SESSION_ID" ]; then
  SESSION_ID="$(ls -t "$CLAUDE_PROJ"/*.jsonl 2>/dev/null \
                 | head -1 | xargs -r basename | sed 's/\.jsonl$//')"
  [ -n "$SESSION_ID" ] || die "세션 .jsonl 을 찾지 못함. SESSION_ID 를 인자로 넘겨줘."
  info "세션 자동 선택(최신): $SESSION_ID"
else
  info "세션 지정: $SESSION_ID"
fi

TRANSCRIPT="$CLAUDE_PROJ/$SESSION_ID.jsonl"
[ -f "$TRANSCRIPT" ] || die "transcript 없음: $TRANSCRIPT"

SHORT="${SESSION_ID:0:8}"
DATE="$(date +%Y%m%d)"
OUT="${2:-$HOME/fo_session_${SHORT}_${DATE}.tgz}"

# ---- 번들에 넣을 항목 목록(존재하는 것만) -----------------------------------
REL_PROJECT="$(basename "$PROJECT_DIR")"                 # fo_perception
BASE="$(dirname "$PROJECT_DIR")"                          # /home/wise
CLAUDE_REL=".claude/projects/$ENCODED"

# 경량 모드 제외 규칙(빌드 산출물·rosbag) — 세션 재개엔 불필요, 9G+ → 수백 MB.
EXCLUDES=()
if [ "$FULL" -eq 0 ]; then
  EXCLUDES=(
    --exclude="$REL_PROJECT/build"
    --exclude="$REL_PROJECT/install"
    --exclude="$REL_PROJECT/log"
    --exclude="$REL_PROJECT/7_to_hoamji"
    --exclude="$REL_PROJECT/9_large_inter"
    --exclude="*.mcap"
    --exclude="*.db3"
    --exclude="*.bag"
  )
  info "경량 모드: build/install/log·rosbag 제외 (전부 담으려면 --full)"
else
  warn "FULL 모드: 빌드 산출물·rosbag 포함 → 수 GB, 오래 걸림"
fi

ITEMS=( "$REL_PROJECT" )
ITEMS+=( "$CLAUDE_REL/$SESSION_ID.jsonl" )
[ -d "$CLAUDE_PROJ/$SESSION_ID" ] && ITEMS+=( "$CLAUDE_REL/$SESSION_ID" ) \
  || warn "세션 사이드폴더 없음(스냅샷 없이 진행)"
[ -d "$CLAUDE_PROJ/memory" ]      && ITEMS+=( "$CLAUDE_REL/memory" ) \
  || warn "memory 폴더 없음(생략)"

# ---- 매니페스트 + import 스크립트를 번들에 동봉 ------------------------------
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

cat > "$STAGE/MANIFEST.txt" <<EOF
Claude Code 세션 이관 번들
생성일   : $(date '+%Y-%m-%d %H:%M:%S')
세션 ID  : $SESSION_ID
원본 경로: $PROJECT_DIR
인코딩명 : $ENCODED
CC 버전  : $(command -v claude >/dev/null && claude --version 2>/dev/null || echo 'unknown')
포함 항목:
$(printf '  - %s\n' "${ITEMS[@]}")

복원: tar 를 \$HOME 에서 풀고  bash import_session.sh  실행.
주의: 대상 PC 홈은 반드시 /home/wise (프로젝트 경로 $PROJECT_DIR 강결합).
EOF

# import 스크립트를 이 스크립트 옆에서 복사(같은 디렉터리에 둠)
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SELF_DIR/import_session.sh" ]; then
  cp "$SELF_DIR/import_session.sh" "$STAGE/import_session.sh"
else
  warn "import_session.sh 를 옆에서 못 찾음 — 번들에서 빠짐(수동 복원 필요)"
fi

# ---- git 상태 스냅샷(참고용, 복원엔 불필요) ---------------------------------
( cd "$PROJECT_DIR" && git status --porcelain 2>/dev/null ) > "$STAGE/git_status_snapshot.txt" || true

# ---- 묶기 -------------------------------------------------------------------
info "번들 생성 중… (미커밋 변경·untracked 포함, node_modules/build 제외 안 함)"
info "출력: $OUT"

# 두 소스 루트($BASE 와 STAGE)를 한 tar 로: --transform 로 STAGE 파일을 최상위에 배치
tar czf "$OUT" \
  "${EXCLUDES[@]}" \
  -C "$BASE"  "${ITEMS[@]}" \
  -C "$STAGE" MANIFEST.txt git_status_snapshot.txt \
  $( [ -f "$STAGE/import_session.sh" ] && echo import_session.sh )

SIZE="$(du -h "$OUT" | cut -f1)"
info "완료 ✅  $OUT  ($SIZE)"
echo
echo "다음 단계 (대상 PC):"
echo "  1) 홈이 /home/wise 인 계정으로 로그인"
echo "  2) 번들을 \$HOME 으로 복사 후:"
echo "       cd ~ && tar xzf $(basename "$OUT") && bash import_session.sh"
