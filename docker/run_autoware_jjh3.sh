#!/bin/bash
# run_autoware_jjh3.sh
# Autoware 컨테이너(autoware_jjh3) 실행 스크립트
# - X11 forwarding (rviz2, rqt 등 GUI 지원)
# - 로컬 ~/autoware ↔ 컨테이너 /autoware 마운트
# - NVIDIA GPU (runtime=nvidia) 지원

set -e

CONTAINER_NAME="autoware_jjh3"
IMAGE_NAME="autoware-orin:latest"

# 로컬 Autoware 워크스페이스 경로 (필요시 변경)
AUTOWARE_LOCAL="${HOME}/autoware"
AUTOWARE_CONTAINER="/autoware"

# ──────────────────────────────────────────────
# X11 Display 설정
# ──────────────────────────────────────────────
setup_x11() {
    if [ -z "${DISPLAY}" ]; then
        export DISPLAY=:0
    fi
    xhost +local:docker 2>/dev/null || true

    XAUTH_FILE="/tmp/.docker.xauth"
    touch "${XAUTH_FILE}"
    chmod 777 "${XAUTH_FILE}"
    if command -v xauth &>/dev/null && [ -n "${DISPLAY}" ]; then
        xauth nlist "${DISPLAY}" 2>/dev/null \
            | sed -e 's/^..../ffff/' \
            | xauth -f "${XAUTH_FILE}" nmerge - 2>/dev/null || true
    fi
    echo "${XAUTH_FILE}"
}

# ──────────────────────────────────────────────
# 로컬 디렉토리 준비
# ──────────────────────────────────────────────
mkdir -p "${AUTOWARE_LOCAL}/src"
mkdir -p "${HOME}/.ccache"   # ccache 공유 (빌드 속도 향상)

XAUTH_FILE=$(setup_x11)

# ──────────────────────────────────────────────
# 이미 존재하는 컨테이너 처리
# ──────────────────────────────────────────────
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "[INFO] 이미 실행 중인 컨테이너에 접속합니다: ${CONTAINER_NAME}"
        docker exec -it \
            -e DISPLAY="${DISPLAY}" \
            -e XAUTHORITY="${XAUTH_FILE}" \
            "${CONTAINER_NAME}" bash
    else
        echo "[INFO] 중지된 컨테이너를 재시작합니다: ${CONTAINER_NAME}"
        docker start "${CONTAINER_NAME}"
        docker exec -it \
            -e DISPLAY="${DISPLAY}" \
            -e XAUTHORITY="${XAUTH_FILE}" \
            "${CONTAINER_NAME}" bash
    fi
    exit 0
fi

# ──────────────────────────────────────────────
# 새 컨테이너 생성 및 실행
# ──────────────────────────────────────────────
echo "[INFO] 새 컨테이너를 생성합니다: ${CONTAINER_NAME}"
echo "[INFO] 로컬 마운트: ${AUTOWARE_LOCAL} -> ${AUTOWARE_CONTAINER}"

docker run -it \
    --name "${CONTAINER_NAME}" \
    --runtime nvidia \
    --privileged \
    --network host \
    --ipc host \
    \
    `# GPU / Display` \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="${XAUTH_FILE}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${XAUTH_FILE}:${XAUTH_FILE}" \
    \
    `# Autoware 소스 마운트 (로컬 ↔ 컨테이너 동기화)` \
    -v "${AUTOWARE_LOCAL}:${AUTOWARE_CONTAINER}" \
    \
    `# ccache 공유 (빌드 캐시 재사용)` \
    -v "${HOME}/.ccache:/home/aw/.ccache" \
    \
    `# 장치 / 시스템` \
    -v /dev:/dev \
    -v /etc/localtime:/etc/localtime:ro \
    \
    `# 메모리 설정 (Autoware 빌드에 필요)` \
    --shm-size=8g \
    --ulimit nofile=65536:65536 \
    \
    "${IMAGE_NAME}" \
    bash

echo "[INFO] 컨테이너가 종료되었습니다."
