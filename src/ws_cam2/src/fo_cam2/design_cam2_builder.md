# cam2_builder_node_v2 Design

## 목적

기존 [`cam2_builder_node.cpp`](./src/cam2_builder_node.cpp)는 `Cam2LD`와 `Cam2TDTLOD` 두 입력을 받아 최종 [`Cam2Data`](/home/wise/fo_perception/src/fo_msgs/msg/Cam2Data.msg)로 합친다.

`cam2_builder_node_v2`의 목적은 다음과 같다.

- 입력 프로토콜을 `fo_msgs/Cam2LD`, `fo_msgs/Cam2TD`, `fo_msgs/Cam2TL`, `fo_msgs/Cam2OD`로 분리
- 각 필드 생성 로직과 최종 조립 로직을 완전히 분리
- 최신 샘플 기반으로 낮은 지연의 `/cam2_data` publish 유지
- upstream 노드가 바뀌어도, `Cam2XX` 메시지 계약만 유지되면 builder는 수정하지 않도록 설계

즉, `cam2_builder_node_v2`는 "후처리/인지 노드"가 아니라 "고정 프로토콜 조립기" 역할만 담당한다.

## 입력 / 출력

### Input topics

- `fo_msgs/Cam2LD`  : input_topic_ld
- `fo_msgs/Cam2TD`  : input_topic_td
- `fo_msgs/Cam2TL`  : input_topic_tl
- `fo_msgs/Cam2OD`  : input_topic_od

// 위처럼 입력 토픽 이름을 파라미터화하기

### Output topic

- `fo_msgs/Cam2Data` : output_topic

## 핵심 설계 방향

### 1. Latest-value aggregator

이 노드는 모든 입력을 완벽히 동기화해서 묶는 노드가 아니다.

- 실시간성이 더 중요함
- 일부 필드가 약간 다른 시점이어도 허용 가능
- 최신 값이 있으면 바로 조립해서 보내는 쪽이 목적에 맞음

따라서 기본 전략은 `queue accumulation`이 아니라 `latest sample overwrite`가 적합하다.

### 1-1. Multi-threaded latest-slot architecture

실시간성 우선 요구를 반영해, v2는 `MultiThreadedExecutor` 기반 구조를 기본안으로 둔다.

다만 병렬성의 핵심은 "모든 콜백이 동시에 같은 버퍼를 건드리게 하는 것"이 아니라:

- 각 입력 콜백이 자기 입력 전용 latest slot만 갱신하고
- composer timer만 최종 `Cam2Data`를 조립하고 publish하는 것

이다.

즉:

- 입력 4개는 병렬 수신 가능
- 최종 출력 조립은 단일 composer callback에서만 수행

구조로 설계한다.

### 2. Builder는 필드 생성 로직을 모름

이 노드는 다음을 하지 않는다.

- bbox에서 거리 계산
- detection을 TD/TL/OD로 분류
- lane coefficient 계산
- class mapping

이 노드는 다음만 한다.

- 각 `Cam2XX` 메시지를 최신 버퍼에 저장
- 주기적으로 최신 스냅샷을 `Cam2Data`로 조립
- publish 직전 버퍼 스왑

그래서 이후 `Cam2TD`, `Cam2TL`, `Cam2OD`를 누가 어떻게 채우든 builder는 그대로 유지된다.

## 제안 아키텍처

### Subscriber layer

노드 내부에 4개 subscriber를 둔다.

- `sub_ld`
- `sub_td`
- `sub_tl`
- `sub_od`

각 콜백은 자기 타입의 `latest buffer`만 갱신한다.

예시 개념:

- `latest_ld_`
- `latest_td_`
- `latest_tl_`
- `latest_od_`

각 latest slot은 다음 정보를 가진다.

- 마지막 메시지
- 마지막 수신 시각
- 수신 여부 플래그
- 선택적으로 sequence/debug counter
- stale 판정용 메타데이터

중요한 점은:

- 각 slot은 "입력 토픽 전용 shared member state"다
- 특정 스레드 전용 버퍼라기보다, 해당 입력 callback만 write하고 composer가 read하는 구조다
- 다른 입력 callback은 자기 slot 외의 영역에 쓰지 않는다

### Compose layer

주기 타이머 하나가 고정 주기마다 최신 스냅샷을 읽어 `write_buffer_`를 만든다.

역할:

- `latest_ld_ -> ld_snapshot`
- `latest_td_ -> td_snapshot`
- `latest_tl_ -> tl_snapshot`
- `latest_od_ -> od_snapshot`

그리고 이 snapshot들로 local `Cam2Data out`를 조립한다.

여기서 "모든 입력이 같은 시각이어야 한다"는 조건은 두지 않는다.

필요 시 다음 정책만 둔다.

- 아직 한 번도 안 온 필드는 zero/default 값 유지
- 너무 오래된 필드는 stale warning만 출력

### Partial update 동작

예를 들어 4개 입력 중:

- `Cam2LD`, `Cam2TD`는 계속 들어오고
- `Cam2TL`, `Cam2OD`는 한동안 안 들어온다면

publish되는 `Cam2Data`는 다음처럼 동작한다.

- `ld`, `td` 필드는 최신 입력에 맞춰 계속 갱신
- `tl`, `od` 필드는 마지막 값 유지 또는 zero/default 유지

즉, 이 노드는 "입력이 들어온 필드만 부분 갱신되는 latest aggregator"로 동작한다.

모든 입력이 동시에 새 값이어야 publish되는 구조는 아니다.

### Publish layer

publish도 같은 타이머 안에서 처리할 수 있지만, 역할을 분리해서 생각하면 더 명확하다.

1. compose timer
2. publish timer 또는 compose 직후 publish

v2 기본안에서는 output double-buffer를 두지 않는다.

대신 아래 절차를 사용한다.

1. timer가 각 latest slot의 값을 짧게 복사해 local snapshot 생성
2. local snapshot으로 local `Cam2Data out` 조립
3. local message를 바로 publish

즉, output 전용 `write_buffer_` / `send_buffer_`를 유지하기보다:

- input은 latest slot
- output은 timer stack/local object

구조로 단순화한다.

이 방식의 장점:

- output 상태가 줄어듦
- swap 타이밍 관리가 필요 없음
- stale 처리 분기가 더 단순함
- 현재 메시지 크기와 조립 비용 기준으로 충분히 가벼움
- publish 대상 output은 composer만 생성하므로 write 충돌이 없음

## 큐를 쓸지, queue depth 1로 갈지

### 권장 결론

기본안은 `큐를 별도로 두지 않고`, 각 subscriber를 `KeepLast(1)` 기반 latest-value buffer로 운용하는 것이다.

이유:

- `/cam2_data`는 최신 상태 표현이 목적
- 과거 샘플을 순서대로 모두 처리할 필요가 없음
- 누적 큐는 지연을 키움
- 입력 하나가 밀리면 전체 조립 결과가 늦어짐
- builder는 sensor fusion synchronizer가 아니라 protocol packer에 가까움

### queue depth 1이 적합한 이유

`Cam2LD`, `Cam2TD`, `Cam2TL`, `Cam2OD`가 고주기일 수 있으므로, builder가 과거 데이터를 다 소비하려고 하면 오히려 실시간성이 나빠진다.

따라서:

- ROS subscriber QoS: `KeepLast(1)` 권장
- 노드 내부 버퍼도 latest only 권장

즉 "손실을 감수"하는 것이 아니라, 이 노드의 목적에 맞는 "의도된 최신값 유지 정책"으로 보는 게 맞다.

### 언제 내부 큐가 필요할까

아래 조건이면 큐를 고려할 수 있다.

- 모든 메시지를 빠짐없이 기록해야 함
- 오프라인 로그 재생 목적
- strict temporal alignment가 필요함
- builder 내부에서 추가적인 state machine이 있음

현재 요구사항에서는 해당하지 않는다.

## 스레드 / 타이머 모델 제안

### 권장 기본안

- subscriber 콜백 4개
- compose+publish용 wall timer 1개

즉, 별도 write thread를 만들지 않아도 된다.

이유:

- 로직이 단순함
- 디버깅이 쉬움
- 현재 작업량이 작음
- `Cam2Data` 조립 비용이 매우 낮음

### 실행기

둘 중 하나를 선택할 수 있다.

#### Option A. SingleThreadedExecutor

장점:

- 가장 단순함
- lock 경쟁이 거의 없음
- 재현성이 좋음

단점:

- 어떤 콜백이 길어지면 다른 콜백이 밀릴 수 있음

이 노드에서는 각 콜백이 단순 복사만 하므로 동작은 충분하다.
다만 입력 4개와 composer를 병렬 가능하게 유지하려는 실시간성 목표에는 완전히 맞지 않을 수 있다.

#### Option B. MultiThreadedExecutor + callback groups

장점:

- subscriber와 timer 병렬 처리 가능
- 입력 4개가 서로 대기하지 않고 수신될 수 있음
- 실시간성 우선 설계 의도와 더 잘 맞음

단점:

- lock 설계가 더 중요해짐
- 디버깅 복잡도 증가

v2에서는 이 방식을 기본안으로 권장한다.

### 결론

v2 첫 구현은 다음이 적절하다.

- `MultiThreadedExecutor`
- 입력별 callback group 분리
- subscriber 4개
- compose/publish timer 1개
- 각 입력 latest buffer + timer local compose

## 버퍼 설계 제안

### Input snapshot

각 입력별로 별도 struct를 둔다.

예시:

```cpp
template <typename MsgT>
struct LatestSlot
{
  MsgT msg{};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool received{false};
  bool stale{true};
};
```

노드 멤버 예시:

```cpp
LatestSlot<fo_msgs::msg::Cam2LD> latest_ld_;
LatestSlot<fo_msgs::msg::Cam2TD> latest_td_;
LatestSlot<fo_msgs::msg::Cam2TL> latest_tl_;
LatestSlot<fo_msgs::msg::Cam2OD> latest_od_;
```

timer 내부 local snapshot 예시:

```cpp
fo_msgs::msg::Cam2LD ld_snapshot;
fo_msgs::msg::Cam2TD td_snapshot;
fo_msgs::msg::Cam2TL tl_snapshot;
fo_msgs::msg::Cam2OD od_snapshot;

fo_msgs::msg::Cam2Data out;
```

이 구조에서 중요한 점:

- subscriber는 shared output buffer를 직접 건드리지 않음
- composer만 local `out`를 생성하고 publish함
- 따라서 "여러 스레드가 write_buffer를 동시에 쓰는 문제"를 구조적으로 제거할 수 있음

### Lock 전략

가장 단순한 방법:

- 입력별 mutex 4개

예시:

- `mtx_ld_`
- `mtx_td_`
- `mtx_tl_`
- `mtx_od_`

subscriber는 자기 mutex만 잠그고 끝낸다.
compose timer는 각 mutex를 짧게 잡고 복사한 뒤 바로 놓는다.

이 구조면 lock contention이 매우 작다.

### memcpy 관점 정리

개념적으로는 composer가 4개 latest slot의 메시지를 local snapshot 또는 local output에 복사하는 구조다.

즉:

- 입력 callback은 자기 slot에만 write
- composer는 slot에서 읽어 local message에 copy
- publish는 composer만 수행

정리하면 "4개 입력 버퍼 -> composer local out" 흐름이며, shared write buffer를 여러 스레드가 같이 쓰지는 않는다.

## 조립 정책

### 기본 조립 규칙

publish 주기마다 다음을 수행:

1. `Cam2Data msg`
2. 최신 `LD/TD/TL/OD`를 각각 복사
3. 한 번도 수신 안 된 필드는 default 값 유지
4. 필요 시 `header` 또는 debug 상태를 별도 로그로 남김
5. local `Cam2Data out`를 바로 publish

이 규칙의 의미는 다음과 같다.

- 들어오지 않는 필드 때문에 전체 `/cam2_data` publish가 멈추지 않음
- 들어오는 필드만 계속 반영됨
- 안 들어오는 필드는 stale policy에 따라 유지 또는 초기화됨

### Staleness 정책

입력 필드마다 stale timeout을 둘 수 있다.

예시:

- `ld_stale_ms`
- `td_stale_ms`
- `tl_stale_ms`
- `od_stale_ms`

동작 예시:

- timeout 초과 시 마지막 값을 유지하되 warning만 출력
- 또는 timeout 초과 시 해당 필드만 zero/default로 reset

### "값이 안 변하면 멈춘 것" 판단

실제로는 두 종류의 stale을 구분하는 것이 좋다.

#### 1. No-message stale

일정 시간 동안 해당 토픽 메시지 자체가 안 들어오는 경우.

이건 builder가 직접 판단 가능하다.

- 마지막 수신 시각 기반
- 가장 단순하고 신뢰성이 높음

#### 2. No-change stale

메시지는 들어오지만 내용이 오랫동안 동일한 경우.

이건 builder가 판단할 수도 있지만, 의미가 더 조심스럽다.

예를 들어:

- 차선이 직선도로에서 오랫동안 거의 같은 값일 수 있음
- 신호등 상태가 실제로 한동안 안 바뀔 수 있음
- 정적 장애물은 원래 값이 안 바뀌는 것이 정상일 수 있음

따라서 "내용이 안 변했으니 멈췄다"는 해석은 field semantics를 알아야 하므로, 기본적으로는 producer 쪽 책임에 더 가깝다.

## stale -> zero 초기화 로직은 어느 레이어에 둘까

### 권장 원칙

#### Builder layer 책임

builder는 아래만 담당하는 것이 적절하다.

- 메시지 미수신 timeout 감지
- 필드별 stale 상태 기록
- 정책에 따라 마지막 값 유지 또는 zero/default reset

즉, `no-message stale`에 대한 reset 정책은 builder에 둬도 된다.

#### Producer layer 책임

producer는 아래를 담당하는 것이 적절하다.

- 해당 프로토콜 값이 실제 의미상 유효한지 판단
- "메시지는 오지만 값이 고장으로 고정된 상태" 탐지
- 필요하면 invalid/default 값으로 내려주기

즉, `no-change stale` 또는 semantic validity 판단은 producer 쪽이 더 적절하다.

## 링버퍼보다 이 구조를 우선 권장하는 이유

링버퍼는 여러 샘플을 보존하고 순차 소비하는 데 유리하다.

하지만 builder v2의 목적은:

- 과거 샘플 보존
- 순차 처리 보장

보다:

- 최신값 유지
- 낮은 지연
- 단순한 조립

에 있다.

따라서 이 노드에서는 링버퍼보다 아래 구조가 더 잘 맞는다.

- 입력별 latest slot
- 필요 시 입력별 double slot 또는 짧은 mutex 보호
- composer local output

즉, 실시간성 목적이라도 현재 역할에는 ring buffer보다 `latest-only shared slot + local compose`가 더 적합하다.

### 추천 분리안

가장 현실적인 책임 분리는 아래와 같다.

- builder:
  - 마지막 수신 시각 기준 timeout 처리
  - 일정 시간 입력이 아예 없으면 해당 필드 zero/default reset 가능
- producer:
  - 값이 계속 같아서 고장인지 정상인지 판단
  - protocol별 validity bit, default 값, failure state 생성

이렇게 두면 builder는 protocol-agnostic하게 유지되고, protocol semantics는 producer에 남는다.

초기 구현에서는 복잡도를 줄이기 위해:

- 기본값은 "마지막 값 유지"
- stale 감지만 로그로 출력

정도가 적당하다.

다만 실제 운영 목표가 "끊긴 필드는 최종 `/cam2_data`에서 0으로 떨어져야 한다"라면, 다음 단계 정책을 builder에 추가할 수 있다.

- `reset_on_stale_ld`
- `reset_on_stale_td`
- `reset_on_stale_tl`
- `reset_on_stale_od`

그리고 각 필드별로:

- timeout 전: 마지막 값 유지
- timeout 후: 해당 필드만 zero/default reset

방식을 적용한다.

## 추천 초기 정책

v2 첫 구현에서는 다음 정책을 권장한다.

### Phase 1

- 입력이 안 와도 전체 publish는 계속 수행
- stale 감지만 로그 출력
- 마지막 값 유지

### Phase 2

운영 중 stale 오동작이 실제 문제로 확인되면:

- field별 timeout 도입
- `no-message stale`만 builder에서 zero/default reset
- `no-change stale` 판단은 producer별로 개별 구현

이 순서가 가장 안전하다.

## 권장 파라미터

- `input_topic_ld`
- `input_topic_td`
- `input_topic_tl`
- `input_topic_od`
- `output_topic`
- `publish_period_ms`
- `stale_timeout_ld_ms`
- `stale_timeout_td_ms`
- `stale_timeout_tl_ms`
- `stale_timeout_od_ms`
- `reset_on_stale_ld`
- `reset_on_stale_td`
- `reset_on_stale_tl`
- `reset_on_stale_od`
- `log_stale`
- `log_rate_limit_ms`

## 기존 cam2_builder와의 차이

기존 [`cam2_builder_node.cpp`](./src/cam2_builder_node.cpp)는:

- 입력이 2개 (`Cam2LD`, `Cam2TDTLOD`)
- `TDTLOD` 내부에서 `TD/TL/OD`를 같이 묶어 받음

v2는:

- 입력이 4개 + 1개 (`Cam2LD`, `Cam2TD`, `Cam2TL`, `Cam2OD`)
- 각 producer가 독립적으로 메시지를 채워도 builder는 그대로 동작
- upstream 구조 변경의 영향이 builder에 덜 전파됨

즉, v2는 현재보다 더 모듈화된 조립기다.

## 기대 장점

- upstream 차선/객체/신호등/정적물 노드 결합도 감소
- `Cam2XX` 생성 로직 변경과 `Cam2Data` 조립 로직 분리
- 특정 필드만 새 노드로 교체 가능
- 최신값 기반이라 지연 누적이 적음
- `/cam2_data` 생성 책임이 명확해짐

## 주의할 점

### 1. 완전 동기화는 보장하지 않음

이 설계는 intentional asynchronous design이다.

- `ld`는 t0
- `td`는 t0 + 20ms
- `tl`은 t0 + 40ms

같은 식으로 섞일 수 있다.

하지만 현재 요구사항에서는 허용 범위라고 본다.

### 2. 슬롯 인덱스 정책은 upstream 책임

builder는 메시지를 "필드 단위"로만 합친다.
track slot allocation, ID association 같은 것은 builder 책임이 아니다.

### 3. Header 정책 필요

`Cam2Data` 자체에 top-level header는 없다.
따라서 디버깅용으로는 각 입력 최신 수신 시각을 로그/diagnostics로 남기는 것이 좋다.

## 구현 순서 제안

1. 새 실행 파일 `cam2_builder_v2` 추가
2. `Cam2LD`, `Cam2TD`, `Cam2TL`, `Cam2OD` subscriber 추가
3. latest-slot 구조체와 mutex 추가
4. `publish_period_ms` timer 구현
5. latest snapshot -> local compose -> publish 경로 구현
6. stale logging 추가
7. launch에 v1/v2 선택 가능하게 추가

## 결론

`cam2_builder_node_v2`는 "여러 Cam2 프로토콜 메시지를 최신값 기준으로 조립하는 독립 aggregator"로 설계하는 것이 적절하다.

현재 요구사항 기준 권장 선택은 다음과 같다.

- 내부 큐는 두지 않음
- subscriber QoS는 `KeepLast(1)`
- 노드 내부는 입력별 latest-value slot 방식
- 입력 callback은 자기 slot만 갱신
- publish는 고정 주기 timer 기반
- output은 timer local compose 후 direct publish

이 선택이 가장 단순하고, 실시간성과 유지보수성의 균형이 좋다.
