#!/usr/bin/env bash
# =============================================================================
# import_session.sh — Claude Code 세션 복원기 (대상 PC에서 실행)
#
#   전제: export_session.sh 로 만든 tar 를 이미 $HOME 에서 풀었다.
#         (풀면 fo_perception/ 과 .claude/projects/... 가 홈에 복원됨)
#         이 스크립트는 tar 안에 동봉되어 최상위에 위치한다.
#
#   하는 일:
#     1) 홈 경로 규약(/home/wise) 일치 여부 검증
#     2) 복원된 세션 파일 존재 확인
#     3) Claude Code 설치/버전 확인 (매니페스트 버전과 비교)
#     4) 확인 후 곧바로 `claude --resume <세션ID>` 실행 (--no-run 이면 안내만)
#
#   사용법:
#       bash import_session.sh              # 검증 후 자동 resume
#       bash import_session.sh --no-run     # 검증만, 실행은 수동
# =============================================================================
set -euo pipefail

PROJECT_DIR="/home/wise/fo_perception"
ENCODED="-home-wise-fo-perception"
CLAUDE_PROJ="$HOME/.claude/projects/$ENCODED"

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; RST=$'\033[0m'
die()  { echo "${RED}[에러]${RST} $*" >&2; exit 1; }
info() { echo "${GRN}[+]${RST} $*"; }
warn() { echo "${YEL}[!]${RST} $*"; }

NO_RUN=0
[ "${1:-}" = "--no-run" ] && NO_RUN=1

# ---- 1) 홈 경로 규약 검증 ---------------------------------------------------
if [ "$HOME" != "/home/wise" ]; then
  warn "현재 HOME=$HOME 이지만 이 세션은 /home/wise 에 강결합돼 있어."
  warn "프로젝트 경로가 $PROJECT_DIR 와 다르면 --resume 목록에 안 뜬다."
  warn "심볼릭 링크(sudo ln -s $HOME /home/wise)나 동일 홈 계정 사용을 권장."
fi

# ---- 2) 복원 파일 확인 + 세션 ID 추출 ---------------------------------------
[ -d "$PROJECT_DIR" ] || die "프로젝트 미복원: $PROJECT_DIR (tar 를 \$HOME 에서 풀었는지 확인)"
[ -d "$CLAUDE_PROJ" ] || die "세션 폴더 미복원: $CLAUDE_PROJ"

SESSION_ID=""
if [ -f "MANIFEST.txt" ]; then
  SESSION_ID="$(grep -m1 '^세션 ID' MANIFEST.txt | sed 's/.*: *//' || true)"
fi
if [ -z "$SESSION_ID" ]; then
  SESSION_ID="$(ls -t "$CLAUDE_PROJ"/*.jsonl 2>/dev/null | head -1 \
                 | xargs -r basename | sed 's/\.jsonl$//')"
fi
[ -n "$SESSION_ID" ] || die "세션 ID 를 확정하지 못함."
[ -f "$CLAUDE_PROJ/$SESSION_ID.jsonl" ] || die "transcript 없음: $SESSION_ID.jsonl"
info "복원된 세션: $SESSION_ID"

# ---- 3) Claude Code 확인 ----------------------------------------------------
if ! command -v claude >/dev/null 2>&1; then
  die "Claude Code(claude)가 설치돼 있지 않음. 먼저 설치 후 로그인해줘."
fi
CUR_VER="$(claude --version 2>/dev/null || echo unknown)"
info "설치된 Claude Code: $CUR_VER"
if [ -f "MANIFEST.txt" ]; then
  SRC_VER="$(grep -m1 '^CC 버전' MANIFEST.txt | sed 's/.*: *//' || true)"
  [ -n "$SRC_VER" ] && [ "$SRC_VER" != "unknown" ] && info "원본 버전: $SRC_VER (다르면 transcript 호환 주의)"
fi

# ---- 4) 실행 ----------------------------------------------------------------
echo
if [ "$NO_RUN" -eq 1 ]; then
  info "검증 완료. 수동 실행:"
  echo "    cd $PROJECT_DIR && claude --resume $SESSION_ID"
  exit 0
fi

info "세션 재개를 시작한다 → cd $PROJECT_DIR && claude --resume $SESSION_ID"
echo "  (로그인 안 돼 있으면 먼저 인증창이 뜰 수 있음. MCP 커넥터는 새 계정에서 재인증 필요.)"
cd "$PROJECT_DIR"
exec claude --resume "$SESSION_ID"
