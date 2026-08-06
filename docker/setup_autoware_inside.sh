#!/bin/bash
# setup_autoware_inside.sh
# 컨테이너 내부에서 실행: Autoware 소스 클론 + 의존성 설치 + 빌드
# Usage: bash /autoware/setup_autoware_inside.sh [--skip-clone] [--skip-build]

set -e

ROS_DISTRO="humble"
AUTOWARE_DIR="/autoware"
JOBS=$(nproc)

SKIP_CLONE=0
SKIP_BUILD=0
for arg in "$@"; do
    case $arg in
        --skip-clone) SKIP_CLONE=1 ;;
        --skip-build) SKIP_BUILD=1 ;;
    esac
done

# ──────────────────────────────────────────────
print_step() {
    echo ""
    echo "════════════════════════════════════════════"
    echo "  $1"
    echo "════════════════════════════════════════════"
}
# ──────────────────────────────────────────────

source "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "${AUTOWARE_DIR}"

# ── Step 1: Clone Autoware ──────────────────
if [ "${SKIP_CLONE}" -eq 0 ]; then
    print_step "[1/4] Autoware 메인 레포 클론"

    if [ -f "${AUTOWARE_DIR}/autoware.repos" ]; then
        echo "[INFO] autoware.repos 이미 존재 — clone 스킵"
    else
        # 현재 디렉토리가 비어있으면 직접 clone
        git clone https://github.com/autowarefoundation/autoware.git "${AUTOWARE_DIR}" --depth 1 2>/dev/null || \
        (git init && git remote add origin https://github.com/autowarefoundation/autoware.git && \
         git fetch --depth 1 origin main && git checkout main)
    fi

    # ── Step 2: VCS import (모든 서브 패키지 클론) ──
    print_step "[2/4] vcs import — 서브 패키지 다운로드"
    mkdir -p src
    vcs import src < autoware.repos
else
    echo "[SKIP] clone 단계 스킵"
fi

# ── Step 3: rosdep ─────────────────────────
print_step "[3/4] rosdep 의존성 설치"
rosdep install -y \
    --from-paths src \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    --skip-keys \
      "python3-osrf-pycommon \
       libopencv-dev \
       libpcl-dev"

# ── Step 4: colcon build ────────────────────
if [ "${SKIP_BUILD}" -eq 0 ]; then
    print_step "[4/4] colcon build (parallel=${JOBS})"
    echo "[INFO] Orin CUDA arch: 87 (sm_87)"
    echo "[INFO] 빌드에 수십 분 ~ 수 시간 소요될 수 있습니다."

    colcon build \
        --symlink-install \
        --parallel-workers "${JOBS}" \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CUDA_ARCHITECTURES=87 \
            -DCUDA_ARCH_BIN=8.7 \
        2>&1 | tee /autoware/build.log
else
    echo "[SKIP] build 단계 스킵"
fi

print_step "완료!"
echo ""
echo "워크스페이스 소싱:"
echo "  source /autoware/install/setup.bash"
echo ""
echo "rviz2 실행:"
echo "  rviz2"
echo ""
echo "빌드 로그:"
echo "  cat /autoware/build.log"
