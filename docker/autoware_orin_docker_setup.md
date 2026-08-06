# Autoware Docker 환경 구축 — NVIDIA Jetson Orin

> **환경**  
> Jetson Orin | JetPack 6.0 (L4T R36.3.0) | CUDA 12.2 | Ubuntu 22.04 | ROS 2 Humble  
> 컨테이너명: `autoware_jjh3`

---

## 목차

1. [전제 조건](#1-전제-조건)
2. [디렉토리 구조](#2-디렉토리-구조)
3. [Docker 이미지 빌드](#3-docker-이미지-빌드)
4. [컨테이너 실행](#4-컨테이너-실행)
5. [Autoware 소스 클론 및 빌드](#5-autoware-소스-클론-및-빌드)
6. [rviz2 / GUI 실행](#6-rviz2--gui-실행)
7. [일상적인 사용법](#7-일상적인-사용법)
8. [트러블슈팅](#8-트러블슈팅)

---

## 1. 전제 조건

### 호스트 설치 확인

```bash
# Docker 버전 확인
docker --version
# Docker version 29.x.x 이상 필요

# NVIDIA runtime 확인
docker info | grep -i runtime
# nvidia 가 목록에 있어야 함

# X11 / 디스플레이 확인
echo $DISPLAY
# :0 또는 :1 이 출력되어야 함
```

### nvidia-container-runtime 설치 (미설치 시)

```bash
# Jetson 전용 패키지 설치
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# 확인
docker run --rm --runtime nvidia nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

---

## 2. 디렉토리 구조

```
~/autoware/                     ← 호스트 (로컬) Autoware 워크스페이스
│                                  컨테이너 내부 /autoware 에 마운트됨
├── autoware.repos              ← Autoware 메타 레포
├── src/                        ← 모든 패키지 소스 (vcs import 결과)
│   ├── core/
│   ├── universe/
│   └── ...
├── build/                      ← colcon 빌드 아티팩트
├── install/                    ← 설치 경로 (source 대상)
├── log/                        ← colcon 로그
└── setup_autoware_inside.sh    ← 빌드 스크립트 (선택 복사)

~/fo_perception/docker/         ← 이 레포의 Docker 설정 디렉토리
├── Dockerfile.autoware-orin    ← Docker 이미지 정의
├── run_autoware_jjh3.sh        ← 컨테이너 실행 스크립트
├── setup_autoware_inside.sh    ← 컨테이너 내부 빌드 스크립트
└── autoware_orin_docker_setup.md  ← 이 문서
```

---

## 3. Docker 이미지 빌드

### Base 이미지 Pull

```bash
# dustynv 의 Humble + L4T R36.3.0 base 이미지 pull
# (최초 1회, 약 5~10 GB)
docker pull dustynv/ros:humble-ros-base-l4t-r36.3.0
```

> **주의**: `dustynv/ros` 이미지가 없거나 버전이 다를 경우  
> https://github.com/dusty-nv/jetson-containers 에서 현재 L4T 버전에 맞는 태그 확인  
> ```bash
> # 현재 L4T 버전 확인
> cat /etc/nv_tegra_release
> # R36 (release), REVISION: 3.0 → l4t-r36.3.0
> ```

### 이미지 빌드

```bash
cd ~/fo_perception/docker

# Stage 1 (개발용 deps 이미지) — 권장
docker build \
    --target autoware-orin-deps \
    -f Dockerfile.autoware-orin \
    -t autoware-orin:latest \
    .

# 빌드 완료 확인
docker images | grep autoware-orin
```

> **Stage 2** (`autoware-orin-built`)는 빌드 아티팩트를 이미지에 포함하는 용도입니다.  
> 개발 중에는 **Stage 1 이미지 + 호스트 마운트 빌드** 방식을 사용합니다.

---

## 4. 컨테이너 실행

### 스크립트에 실행 권한 부여 (최초 1회)

```bash
chmod +x ~/fo_perception/docker/run_autoware_jjh3.sh
```

### 컨테이너 시작

```bash
# X11 forwarding + 소스 마운트 + GPU 포함 컨테이너 시작
~/fo_perception/docker/run_autoware_jjh3.sh
```

스크립트가 처리하는 것:

| 항목 | 내용 |
|------|------|
| 컨테이너명 | `autoware_jjh3` |
| GPU | `--runtime nvidia` (Orin iGPU 포함) |
| 디스플레이 | X11 socket + XAUTHORITY 전달 |
| 소스 마운트 | `~/autoware` → `/autoware` |
| ccache 마운트 | `~/.ccache` → `/home/aw/.ccache` |
| 네트워크 | `--network host` (ROS 2 DDS 통신) |
| SHM | `--shm-size=8g` |

### 새 터미널 추가 접속

```bash
# 이미 실행 중인 컨테이너에 터미널 추가
docker exec -it autoware_jjh3 bash
```

### 컨테이너 관리

```bash
# 상태 확인
docker ps -a | grep autoware_jjh3

# 중지
docker stop autoware_jjh3

# 재시작 (데이터 보존)
docker start autoware_jjh3

# 삭제 (이미지는 유지됨, 소스는 호스트에 있으므로 안전)
docker rm autoware_jjh3
```

---

## 5. Autoware 소스 클론 및 빌드

컨테이너 내부에서 진행합니다.

### 방법 A: 자동 스크립트 사용

```bash
# 컨테이너 내부에서
bash /autoware/setup_autoware_inside.sh
```

> 호스트에서 스크립트를 컨테이너로 복사하려면:
> ```bash
> # 호스트에서
> cp ~/fo_perception/docker/setup_autoware_inside.sh ~/autoware/
> ```

**단계별 스킵 옵션**:

```bash
# 소스는 이미 있고, 빌드만 다시 하고 싶을 때
bash /autoware/setup_autoware_inside.sh --skip-clone

# 소스 클론만 하고 빌드는 나중에
bash /autoware/setup_autoware_inside.sh --skip-build
```

### 방법 B: 수동 단계별 실행

#### Step 1 — Autoware 메인 레포 클론

```bash
cd /autoware
git clone https://github.com/autowarefoundation/autoware.git . --depth 1
```

#### Step 2 — 모든 패키지 vcs import

```bash
mkdir -p src
vcs import src < autoware.repos
```

> `vcs import`는 `autoware.repos`에 정의된 수십 개의 Git 레포를 `src/` 아래에 클론합니다.  
> 완료까지 수 분 소요 (네트워크 속도에 따라 다름).

#### Step 3 — rosdep 의존성 설치

```bash
source /opt/ros/humble/setup.bash

rosdep install -y \
    --from-paths src \
    --ignore-src \
    --rosdistro humble
```

#### Step 4 — colcon build

```bash
colcon build \
    --symlink-install \
    --parallel-workers $(nproc) \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_ARCHITECTURES=87 \
        -DCUDA_ARCH_BIN=8.7 \
    2>&1 | tee /autoware/build.log
```

> **빌드 시간 안내**  
> Orin에서 첫 전체 빌드: **약 2~4시간** (ccache 없을 경우)  
> ccache 재사용 시 재빌드: **약 15~30분**  
> `nohup` 또는 `tmux`에서 실행 권장

#### Step 5 — 워크스페이스 소싱

```bash
source /autoware/install/setup.bash

# 설치 확인
ros2 pkg list | grep autoware | head -10
```

---

## 6. rviz2 / GUI 실행

### 컨테이너 내부에서 rviz2 실행

```bash
# 컨테이너 내부
source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash

rviz2
```

### X11 연결 안 될 경우 (호스트에서 실행)

```bash
# 호스트에서 X11 권한 재부여
xhost +local:docker

# 컨테이너에서 DISPLAY 확인
docker exec -it autoware_jjh3 bash -c "echo \$DISPLAY"
```

### rqt 실행

```bash
rqt
# 또는
rqt_graph        # 노드 그래프 시각화
rqt_topic        # 토픽 모니터
```

---

## 7. 일상적인 사용법

### 소스 수정 → 빌드

**호스트(VSCode 등)**에서 `~/autoware/src/` 내 파일을 수정합니다.  
컨테이너 내 `/autoware/src/`가 동기화되므로 **컨테이너에서 바로 빌드**합니다.

```bash
# 컨테이너 내부 — 특정 패키지만 빌드
colcon build \
    --packages-select <package_name> \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=87
```

### Autoware 실행 예시

```bash
# 컨테이너 내부
source /autoware/install/setup.bash

# 예: Planning Simulator 실행
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=/path/to/map \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit
```

### 컨테이너 중지 없이 호스트 재시작 후 복귀

```bash
# 호스트 재부팅 후
~/fo_perception/docker/run_autoware_jjh3.sh
# → 중지된 컨테이너를 자동 감지하고 재시작
```

---

## 8. 트러블슈팅

### `--runtime nvidia` 실패

```bash
# nvidia-container-runtime 재설정
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### rviz2 실행 시 화면이 안 뜸

```bash
# 호스트에서
xhost +local:docker
export DISPLAY=:0

# 컨테이너 재시작 시 DISPLAY 전달 확인
docker exec autoware_jjh3 env | grep DISPLAY
```

### rosdep 실패 (특정 패키지)

```bash
# 문제 패키지 제외하고 설치
rosdep install -y --from-paths src --ignore-src --rosdistro humble \
    --skip-keys "패키지_이름"
```

### colcon 빌드 중 메모리 부족

```bash
# 병렬 빌드 수 제한
colcon build --parallel-workers 2 --executor sequential ...
```

### ccache 초기화

```bash
# 컨테이너 내부
ccache -C        # 캐시 클리어
ccache -s        # 상태 확인
```

### 컨테이너 이미지 업데이트

```bash
# 호스트에서
docker pull dustynv/ros:humble-ros-base-l4t-r36.3.0
docker build --target autoware-orin-deps \
    --no-cache \
    -f ~/fo_perception/docker/Dockerfile.autoware-orin \
    -t autoware-orin:latest \
    ~/fo_perception/docker/

# 기존 컨테이너 삭제 후 재생성
docker rm autoware_jjh3
~/fo_perception/docker/run_autoware_jjh3.sh
```

---

## 참고 링크

- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers) — Jetson용 베이스 이미지
- [Autoware GitHub](https://github.com/autowarefoundation/autoware)
- [JetPack 6.0 Release Notes](https://developer.nvidia.com/embedded/jetpack-sdk-60)
