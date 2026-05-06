# SAC 학습 · 시뮬 적용 · 비교 · 재튜닝 가이드

`rl_pmi/train_sac.py`로 PMI 작업공간 추적 환경(`PMITrackEnv`)에서 SAC를 학습하고, MuJoCo 시뮬레이션에서 결과를 검증한 뒤, 어떤 지표를 비교하고 어떤 파라미터를 바꿔 재학습할지 정리한 문서입니다.

상세 제어·보상 수식은 [pmi_track_env.md](pmi_track_env.md)를 참고하세요.

---

## 1. 전체 흐름

1. **학습**: `train_sac.py` → 체크포인트 `*.zip` 저장 (기본 `checkpoints/sac_pmi_track.zip`).
2. **적용**: 같은 `PMITrackEnv` 설정으로 `SAC.load` 후 `predict` → `env.step` (`run_policy.py` 또는 `record_eval_video.py`).
3. **비교**: TensorBoard 스칼라, `run_policy.py` 통계, 영상, (선택) 이전 체크포인트·베이스라인.
4. **재튜닝**: 문제 징후에 맞춰 **SAC 하이퍼파라미터**와 **환경·보상·제어 인자**를 조정 후 다시 1부터 반복.

**필수 원칙**: 평가·시뮬에 쓰는 `PMITrackEnv` CLI 인자는 학습 때와 **동일**해야 합니다. 다르면 관측 분포·토크 한계·보상 정의가 달라져 정책 성능이 의미 없이 변합니다.

---

## 2. 학습 실행

가상환경은 저장소 루트 **`PMI/`** 에 두고, 패키지는 루트 `requirements.txt` 로 설치합니다. 학습 명령은 **`rl_pmi/`** 에서 실행한다고 가정합니다.

```bash
cd /path/to/PMI
.venv/bin/pip install -r requirements.txt
cd rl_pmi
../.venv/bin/python train_sac.py --timesteps 500000
```

### 2.1 주요 `train_sac.py` 인자 (알고리즘)

| 인자 | 기본값 | 역할 |
|------|--------|------|
| `--timesteps` | 500000 | 총 환경 스텝 |
| `--save` | `checkpoints/sac_pmi_track` | 저장 경로 (`.zip` 확장자는 SB3가 처리) |
| `--seed` | 0 | 재현성 |
| `--device` | `cuda` | `cpu` / `cuda` / `auto` (MLP 정책은 CPU도 무방) |
| `--learning-rate` | 3e-4 | SAC 옵티마이저 |
| `--buffer-size` | 1000000 | 리플레이 버퍼 (메모리 부족 시 축소) |
| `--batch-size` | 256 | 미니배치 |
| `--learning-starts` | 10000 | 버퍼에 이 스텝만큼 쌓인 뒤 학습 시작 |
| `--gamma` | 0.99 | 할인율 |
| `--tau` | 0.005 | 타깃 네트워크 소프트 업데이트 (알고리즘 τ; 관절 토크 한계 `--tau-limit`과 무관) |
| `--no-tensorboard` | off | 지정 시 TensorBoard 로그 비활성 |

### 2.2 환경 공통 인자 (`PMITrackEnv`)

`env_args.py`의 그룹이며 **`run_policy.py` / `record_eval_video.py`와 동일 플래그**입니다.

- **제어·안전**: `--tau-limit`, `--delta-f-scale`, `--ks`, `--kd`, `--no-gravity-ff`
- **보상**: `--w-pos`, `--w-rp`, `--w-vel`, `--w-omega`, `--action-penalty`, `--no-residual-tau-penalty`
- **에피소드**: `--t-end`, `--reset-noise`

CLI 기본은 **`--no-residual-tau-penalty`를 주지 않으면** `penalize_residual_torque=True`(잔여 관절 토크 제곱 페널티)입니다.

### 2.3 학습 중 영상 (선택)

```bash
../.venv/bin/python train_sac.py --timesteps 500000 --record-video \
  --video-folder videos/train --video-freq 50000 --video-length 400
```

`moviepy`, (권장) `ffmpeg` 필요.

---

## 3. 학습 결과를 시뮬레이션에 적용

산출물: **`checkpoints/sac_pmi_track.zip`** (또는 `--save`로 지정한 경로).

### 3.1 숫자만 빠르게 (에피소드 리턴·추적 오차)

```bash
../.venv/bin/python run_policy.py --model checkpoints/sac_pmi_track.zip --episodes 5
```

학습 시 사용한 옵션을 그대로 붙입니다. 예: 학습에 `--no-gravity-ff`를 썼다면 평가에도 동일.

### 3.2 한 에피소드 영상(mp4)

```bash
MUJOCO_GL=egl ../.venv/bin/python record_eval_video.py \
  --model checkpoints/sac_pmi_track.zip --out videos/eval.mp4
```

헤드리스/서버에서는 `MUJOCO_GL=egl` 등 표시 백엔드를 환경에 맞게 설정합니다.

### 3.3 학습 과정 모니터링 (TensorBoard)

```bash
../.venv/bin/tensorboard --logdir tensorboard_logs --port 6006
```

`--no-tensorboard`를 주지 않았을 때만 `rl_pmi/tensorboard_logs/`에 기록됩니다.

---

## 4. 무엇을 비교할 것인가

같은 환경 설정 하에서 **상대 비교**가 가장 안전합니다 (절대 “좋은 숫자” 한 줄 기준은 어렵습니다).

### 4.1 TensorBoard (학습 중)

- **`rollout/ep_rew_mean`**(또는 유사): 시간에 따라 **증가**(덜 음수)하는지.
- **`train/`** 손실: NaN·폭주 시 학습 불안정 가능.

### 4.2 `run_policy.py` 출력 (평가)

에피소드마다 출력되는 항목:

- **`return`**: 보상 합. 이 환경에서 보상은 추적 비용의 음수이므로 **클수록**(0에 가까울수록) 유리.
- **`err_pos_mean` / `err_pos_last`**: 말단 위치 오차 노름 (`info["err_pos_norm"]` 기반).
- **`track_cost_mean`**: 스텝 평균 추적 비용 (`info["tracking_cost"]`; 보상의 cost 항과 동일 정의).

여러 시드·여러 에피소드 평균·분산을 함께 보면 됩니다.

### 4.3 영상

- 목표 궤적 대비 **말단 추종**, 과도한 **진동·포화·이탈** 여부.

### 4.4 체크포인트·실험 간 비교

- 같은 조건에서 **초·중·막 저장 zip**(중간 저장을 쓰도록 코드를 바꾼 경우) 또는 **다른 하이퍼 실험**을 나란히 `run_policy.py`로 비교.
- (선택) 순수 PD만에 가까운 행동·무작위 행동 등 **베이스라인** 대비 개선 여부.

---

## 5. 재학습 시 어떤 파라미터를 조절할까

증상별로 **알고리즘(SAC)** 과 **환경·보상·제어** 중 어디를 건드릴지 나누는 것이 좋습니다.

### 5.1 SAC / 학습 스케줄 (`train_sac.py`)

| 증상 | 조절 후보 |
|------|-----------|
| 리턴·평가 지표가 **느리게만 오르거나 정체** | `--timesteps` 증가, `--learning-rate` 미세 조정(예: 1e-4 ~ 3e-4), `--batch-size` |
| 초반 탐색 부족·버퍼 미만 | `--learning-starts` 조정, `--buffer-size` 확보 |
| 학습 불안정·손실 폭주 | `--learning-rate` 감소, `--tau`(타깃 업데이트) 소폭 조정, `--batch-size` |
| 메모리 부족 | `--buffer-size` 감소 (예: 500000) |
| 속도/재현 | `--device cpu`, `--seed` 고정 |

### 5.2 추적 품질 vs 행동 크기·진동 (환경 보상·스케일)

| 증상 | 조절 후보 |
|------|-----------|
| 위치 추적은 더 중요 | `--w-pos` 증가 |
| 자세(roll/pitch) 더 중요 | `--w-rp` 증가 |
| 속도 맞추기 중요 | `--w-vel`, `--w-omega` 증가 |
| 행동이 과도·진동·잔여 토크 큼 | `--action-penalty` 증가, 또는 `--delta-f-scale` **축소**(보수적 ΔF) |
| 잔여 토크 페널티가 과하면 행동이 너무 소극적 | `--action-penalty` 감소, 또는 `--no-residual-tau-penalty`로 ‖a‖² 페널티 모드 시도 |
| 시뮬에서 토크·력이 과도 | `--tau-limit`, `--ks`/`--kd`, `--delta-f-scale` (과도한 게인·스케일 완화) |

### 5.3 분포 일치 (학습 vs 배포)

| 증상 | 조절 후보 |
|------|-----------|
| 평가에서 초기 자세 분포가 다름 | `--reset-noise`를 학습·평가 **동일**하게 맞춤 |
| 에피소드 길이·궤적 구간 문제 | `--t-end` (스텝 상한과 함께 `_max_steps`에 영향) |
| 중력 보상 유무 불일치 | `--no-gravity-ff` 학습·평가 일치 |

---

## 6. 권장 반복 절차 (요약)

1. `train_sac.py`로 학습 → `tensorboard_logs`와 (선택) 학습 영상 확인.
2. **학습과 동일 CLI**로 `run_policy.py` 실행 → `return`, `track_cost_mean`, `err_pos_*` 기록.
3. 필요 시 `record_eval_video.py`로 시각 확인.
4. 목표 미달 시 위 표에 따라 **한두 축만** 바꿔 재학습 (한 번에 모두 바꾸면 원인 분석이 어려움).
5. 개선 여부는 **동일 평가 프로토콜**로 이전 체크포인트와 숫자·영상 비교.

---

## 7. 관련 파일

| 파일 | 역할 |
|------|------|
| [train_sac.py](../train_sac.py) | SAC 학습 |
| [run_policy.py](../run_policy.py) | 체크포인트 rollout (숫자) |
| [record_eval_video.py](../record_eval_video.py) | 체크포인트 rollout (mp4) |
| [env_args.py](../env_args.py) | 환경 공통 CLI |
| [pmi_track_env.md](pmi_track_env.md) | 제어·관측·보상 명세 |
| [README.md](../README.md) | 패키지 개요·명령 예시 |
