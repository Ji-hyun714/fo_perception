# Claude Code 세션 이관 자동화

이 세션의 **대화 이력(transcript) + 프로젝트 코드(미커밋 포함) + 메모리**를 묶어
다른 PC/계정에서 `claude --resume` 로 그대로 이어가기 위한 스크립트.

## 원본 PC

```bash
cd ~
tools/session_transfer/export_session.sh
#   → ~/fo_session_<shortid>_<날짜>.tgz 생성 (경량, ~800MB)
```

- 인자 없이 실행하면 **가장 최근 세션 자동 선택**.
- 특정 세션: `export_session.sh <SESSION_ID>`
- 빌드 산출물·rosbag까지 전부: `export_session.sh --full` (수 GB)
- 기본 제외: `build/ install/ log/ 7_to_hoamji/ 9_large_inter/ *.mcap *.db3 *.bag`
  (세션 재개엔 불필요. 모델 가중치 `.onnx/.trt/.pt` 는 포함 — 추론에 필요)

## 대상 PC

```bash
cd ~                       # 홈은 반드시 /home/wise 여야 함(경로 강결합)
tar xzf fo_session_<...>.tgz
bash import_session.sh     # 검증 후 자동으로 claude --resume 실행
```

- 검증만 하고 실행은 수동: `bash import_session.sh --no-run`
- import_session.sh 는 tar 안에 **동봉**되어 최상위에 풀린다.

## 핵심 제약

| 항목 | 이유 |
|---|---|
| 대상 홈 = `/home/wise` | 프로젝트 경로 `/home/wise/fo_perception` 강결합. 다르면 `--resume` 목록에 안 뜸 (심볼릭 링크로 우회 가능) |
| Claude Code 동일 버전 권장 | transcript 스키마 호환. 버전 차이 크면 재개 실패 가능 |
| 로그인 필요 | 이력은 계정 무관(로컬 파일)이나 실행하려면 대상 계정 로그인 필요 |
| MCP 커넥터 재인증 | Gmail/Notion/Slack/Drive 등은 계정 종속 → 새 계정에서 다시 인증 |
| 미래 응답은 비트단위 동일 X | 과거 이력은 100% 복원, 이후 새 답변은 API 논결정성으로 매번 다름 |
