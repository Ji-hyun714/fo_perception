# 차선 인식 개선 — 진행 체크리스트

최종 갱신: 2026-07-10
관련: [[root_cause_and_improvement.md]](원인·개선안 A~D), [[roi_dev_plan.md]], [[changelog.md]], [[lane_perf_diagnosis.md]](P1~P5)

bag: `7_to_hoamji` (2237프레임 / 134.2s = **약 16.7fps**, `/camera/image_rect`)

---

## A. 측정 인프라
- [x] 결정적 headless dump 모드 `runBagDump()` 추가 (`dump_csv`/`dump_img_dir`/`dump_img_every`)
- [x] A/B 스윕 분석기 `analyze_roi_sweep.py`
- [x] 처리 성능 확인: 2237f/~90s ≈ **25fps 처리능력** (16.7fps 입력 실시간 소화)

## B. 하단 ROI (S1~S4) — **종료(무효과 확정)**
- [x] S1 `roi_top_ratio` 마스크 상단 컷 구현
- [x] S2 결정적 dump로 off/on 측정 (프레임 집합 일치)
- [x] S3 ratio 스윕 0.0~0.90 (11 pass)
- [x] S4 이분 탐색으로 경계 규명: **0.65↔0.70**
- [x] 결론: 유효 픽셀이 마스크 하단 ~35%에만 존재 → 세로 하단 ROI 개선여지 0
- [x] 시각자료 `figs/sweep_metrics.png`, `figs/onset_analysis.png`

## C. P2 degenerate 게이트 — **1차 구현 완료**
- [x] 활성 경로 규명: `use_geometry_kf=0`·`use_sample_kf=0` → `resultsFromRawFit()`
- [x] 게이트 구현: x_check(5m) 예측 횡위치 > lane_w_max*2(10m) → NODET + hold 무효화
- [x] 측정: **pos_std 1189→6.2, head_std 149.7→0.8, degen 245→63**
- [x] 부작용 확인: **both_availability 97.5→74.6%** (degenerate를 버려서 발생)
- [x] ⚠️ availability 손실 보완 → **D2 차수강등으로 해결(94.1% 회복)**

## D. 시간축 추세(trend) 기반 개선 — **진행 중**
목표: 과거 궤적의 곡률·기울기 추세를 prior로 삼아 (1) 곡선 검출 강화, (2) 직선 구간 차수 강등으로 degenerate 방지, (3) 버리는 대신 sane하게 채워 availability 회복.

- [x] D0. 기존 예측 KF baseline: `use_geometry_kf=1` → degen 38 / both_av **66.7** / std 13.6. **결론: 기존 KF는 availability 회복 못함** (오히려 감소). 차수 강등이 필수.
- [ ] D1. 최근 N프레임 곡률 κ 링버퍼 (차량 좌표계, 시간 감쇠)
- [x] D2. 차수 자동 강등 (degenerate 시 3차→1차 직선 재피팅) — **성공. std 1189→3.2, both_av 94.1(C의 74.6 대비 회복), max|pos| 29498→53. C·before 모두 능가.**
- [ ] D3. 곡선 prior 탐색밴드 확대 (`sa_anchor_*` 재활용) — 곡선 검출 강화(선택)
- [ ] D4. 추세 이탈 게이트 (`dkappa_hard=0.09` 활용) — 과거 곡률 추세로 강등 조건 정교화(선택)
- [ ] D5. 최종 비교표 문서화

## E. 보류/후속
- [ ] P1 계수 매핑 off-by-one — **외부 UDP 규약 확인 후** (SF/planning 동시 반영 필요)
- [ ] P4 원거리 커버리지 — 마스크 far-field vs BEV 호모그래피 원인 분리
- [ ] 커밋 — 사용자 지시 대기 (측정용 `runBagDump` + P2 게이트)

---

## 실측 로그
| 날짜 | 항목 | 결과 |
|---|---|---|
| 07-10 | ROI 스윕 | 0.0~0.65 무변화, 0.70+ 붕괴 |
| 07-10 | P2 게이트(C, 버림) | pos_std 1189→6.2, degen 245→63, avail 97.5→74.6 |
| 07-10 | D0 geomKF baseline | both_av 66.7 (기존 KF는 availability 회복 못함) |
| 07-10 | **D2 차수강등** | **pos_std 1189→3.2, head_std 149.7→0.4, max\|pos\| 29498→53, both_av 94.1** |
