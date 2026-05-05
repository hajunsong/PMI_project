# PMITrackEnv (`pmi_track_env.py`) 명세

이 문서는 [`envs/pmi_track_env.py`](../envs/pmi_track_env.py)의 **제어 법칙·관측·보상·하이퍼파라미터**를 정리합니다.

**유지보수:** `PMITrackEnv`의 `step` 제어, `action_space`/`observation_space` 차원, 기본 게인·스케일을 변경할 때 **반드시 본 파일도 함께 수정**해 두세요.

---

## 제어 모델 (작업공간 힘 → 관절 토크)

`analysis/python/main.py`의 **`run_vsd`** 와 같은 해석 기구학을 사용합니다 (`ControlMain`).

| 기호 | 의미 |
|------|------|
| \(e \in \mathbb{R}^5\) | 위치 오차(3) + roll/pitch 오차(2), 해석 EE(`body[3]`) 기준 |
| \(e_v \in \mathbb{R}^5\) | 목표 선속도·각속도 대비 EE 속도 오차 |
| \(F_{\mathrm{VSD}}\) | 작업공간 PD 힘: \(K_s \odot e + K_d \odot e_v\) (성분별 곱) |
| \(\Delta F\) | RL 보정력: 정규화 행동 \(a \in [-1,1]^5\)에 대한 \(\Delta F = a \odot \texttt{delta\_f\_scale}\) |
| \(J\) | 해석 자코비안 \(5 \times 4\) (`jacobian_calculation`) |
| \(\tau\) | \(\tau = J^{\top}(F_{\mathrm{VSD}} + \Delta F)\), 이후 \(\pm\texttt{tau\_limit}\)으로 클립 |

선택적으로 MuJoCo에서 **중력 feedforward** (`qvel=0` 상태의 `qfrc_bias`)를 더해 `ctrl`에 넣습니다 (`use_gravity_feedforward`).

**참고:** `run_vsd`의 MuJoCo 통합 경로에는 `mujoco_pd_scale` 등이 있으나, 본 환경에는 동일 옵션이 없습니다. 시뮬이 과도하면 `ks`/`kd`/`delta_f_scale`/`tau_limit`을 조정하세요.

---

## 스텝 시퀀스

1. MuJoCo `qpos`/`qvel` → `ControlMain.body[i].qi`/`dqi` 동기화 후 `position_calculation`, `velocity_calculation`.
2. \(e, e_v\) 계산 (`run_vsd`와 동일 수식).
3. \(J\) 계산 → \(\tau\) 계산 → 클립 → (옵션) 중력 보상 → `mj_step`.

---

## Gymnasium 스페이스

| 항목 | 형태 | 비고 |
|------|------|------|
| `observation_space` | `Box(18,)` | `q(4)`, `qdot(4)`, MuJoCo 사이트 `ee` 기준 위치·자세 오차(3+2), 속도 오차(3+2) |
| `action_space` | `Box(5,)` low=-1, high=1 | \(\Delta F\)용 정규화 행동 |

관측·보상의 EE 오차는 **MuJoCo 사이트 자코비안** 기준입니다. 제어에 쓰는 \(e, e_v, J\)는 **해석 모델** 기준이므로, 두 정의가 완전히 일치하지 않을 수 있습니다.

---

## 보상

스텝 종료 후(MuJoCo 상태 기준):

\[
\text{cost} = w_{\mathrm{pos}}\|\Delta p\|^2 + w_{\mathrm{rp}}(\Delta\mathrm{roll}^2+\Delta\mathrm{pitch}^2) + w_{\mathrm{vel}}\|\Delta v\|^2 + w_{\omega}(\omega_{\mathrm{err}}^2)
\]

\[
r = -\,\text{cost} - w_u\, u
\]

\(u\)는 설정에 따라 다음 중 하나입니다.

- **`penalize_residual_torque=False` (기본):** \(u = \|a\|^2\) — 정규화 행동 크기 페널티.
- **`penalize_residual_torque=True`:** \(u = \|\Delta\tau_{RL}\|^2\), \(\Delta\tau_{RL} = J^{\top}\Delta F\) — 잔여 관절 토크 크기 페널티 (제안식에 가깝게 진동 억제).

계수 \(w_u\)는 코드상 **`action_penalty`** 한 개로 둡니다 (두 모드 공통).

---

## 주요 생성자 인자 (기본값)

정확한 시그니처는 소스의 `PMITrackEnv.__init__`를 따릅니다. 아래는 문서화 목적의 요약입니다.

| 인자 | 기본 (요약) | 설명 |
|------|-------------|------|
| `mjcf_path` | `mujoco_pmi_viz/models/pmi_arm_primitive_actuated.xml` | MuJoCo 모델 |
| `h` | `0.001` | 시뮬 스텝·궤적 샘플 간격 |
| `t_end` | `3.0` | 에피소드 목표 시간 상한(스텝 수와 함께 `_max_steps` 결정) |
| `tau_limit` | `600.0` | 관절 토크 클립 [\,\mathrm{Nm}\,] |
| `delta_f_scale` | `[80, 80, 80, 40, 40]` | \(\Delta F\) 물리 스케일 (길이 5) |
| `ks` | `[15000, 15000, 15000, 1500, 1500]` | \(F_{\mathrm{VSD}}\) 위치·자세 게인 |
| `kd` | `[1000, 1000, 1000, 10, 10]` | \(F_{\mathrm{VSD}}\) 속도 게인 |
| `use_gravity_feedforward` | `True` | 중력 `qfrc_bias` 가산 여부 |
| `w_pos`, `w_rp`, `w_vel`, `w_omega` | `1.0`, `0.1`, `0.01`, `0.01` | 보상 가중치 |
| `action_penalty` | `1e-4` | \(w_u\): \(\|a\|^2\) 또는 \(\|\Delta\tau_{RL}\|^2\) 계수 (`penalize_residual_torque`) |
| `penalize_residual_torque` | `False` | `True`면 \(w_u\|\Delta\tau_{RL}\|^2\) (`train_sac.py` 기본은 스크립트에서 `True`로 생성) |
| `reset_noise` | `0.05` | 초기 `qpos` 균일 노이즈 반폭 |

목표 자세: `des_roll = -π/2`, `des_pitch = 0` (코드 상 상수).

---

## 궤적

`trajectory.build_run_vsd_trajectories(h)`로 생성하며, `run_vsd`와 동일한 웨이포인트·보간을 사용합니다.

---

## 학습 스크립트

공통 환경 인자는 [`env_args.py`](../env_args.py) 에 모여 있으며 `train_sac.py / train_ppo.py / run_policy.py / record_eval_video.py` 에 동일하게 노출됩니다. 변경 시 본 문서 표와 함께 갱신하는 것이 좋습니다.

- [`train_sac.py`](../train_sac.py): **SAC** (연속 행동, 오프폴리시). `--record-video` 로 `VecVideoRecorder` 기반 **학습 중 mp4** (`moviepy`, `render_mode=rgb_array`).
- [`run_policy.py`](../run_policy.py): `SAC.load` 후 **`model.predict` → `env.step`** 으로 추론만 (영상 없음).
- [`record_eval_video.py`](../record_eval_video.py): 저장된 SAC 체크포인트로 **한 에피소드 rollout 영상** 저장.
- [`train_ppo.py`](../train_ppo.py): PPO (온폴리시, 비교용).

### 학습 결과 적용 / 시뮬 검증

체크포인트 `.zip`은 SB3 `SAC.load` 로 불러오고, **`PMITrackEnv` 생성 인자를 학습 시와 동일하게** 맞춘 뒤 `deterministic=True` 로 `predict` 하는 것이 기본입니다.

시뮬에서만 결과를 볼 때는 [`run_policy.py`](../run_policy.py)(로그), [`record_eval_video.py`](../record_eval_video.py)(mp4)를 쓰면 됩니다. 절차는 [`README.md` — 시뮬레이션으로 결과 확인](../README.md) 참고.

---

## 관련 파일

- [`analysis/python/main.py`](../../analysis/python/main.py): `ControlMain`, `run_vsd`.
