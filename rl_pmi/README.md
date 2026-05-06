# rl_pmi — 작업공간 추적 강화학습

`analysis/python/main.py`의 **`run_vsd`** 와 동일한 목표 EE 궤적(5차 스플라인 웨이포인트)을 따라가며, **위치·roll/pitch·선속도·각속도 오차**를 줄이도록 학습합니다.

상세 명세(제어식, 차원, 기본 게인)는 **[docs/pmi_track_env.md](docs/pmi_track_env.md)** 를 참고하세요.

## 아이디어

- **환경** (`envs/pmi_track_env.py`): MuJoCo `pmi_arm_primitive_actuated.xml`. 제어는 해석 모델 `ControlMain`으로 \(F_{\mathrm{VSD}} = K_s e + K_d e_v\), \(\tau = J^{\top}(F_{\mathrm{VSD}}+\Delta F)\) (+ 선택 시 중력 feedforward). 관측·보상의 오차는 사이트 `ee`(MuJoCo) 기준.
- **보상**:  
  `- (w_pos·‖Δp‖² + w_rp·(Δroll²+Δpitch²) + w_vel·‖Δv‖² + w_ω·(…)) - action_penalty·‖a‖²`  
  즉 PD가 최소화하려는 성격과 비슷한 **가중 제곱 오차의 음수**.
- **행동**: 작업공간 보정력 \(\Delta F\)용 **`a ∈ [-1,1]⁵`** → `ΔF = a ⊙ delta_f_scale` (코드·기본값은 문서 참고).

## 설치

가상환경은 **저장소 루트 (`PMI/`)** 에 하나만 둡니다. 통합 의존성은 루트 `requirements.txt` 를 사용합니다.

```bash
cd /path/to/PMI
python3 -m venv .venv
.venv/bin/pip install -U pip
.venv/bin/pip install -r requirements.txt
```

이후 아래 예시는 모두 **`rl_pmi` 디렉터리**에서 실행한다고 가정하고, 인터프리터는 `../.venv/bin/python` 을 씁니다. (저장소 루트에 서 있으면 `./.venv/bin/python rl_pmi/train_sac.py` 형태도 가능합니다.)

## 학습

기본은 **SAC** (잔여 작업공간 \(\Delta F\) + 선택적 \(\|\Delta\tau_{RL}\|^2\) 페널티). 온폴리시 비교용으로 PPO 스크립트도 유지합니다.

```bash
../.venv/bin/python train_sac.py
```

- 기본 **`--timesteps 500000`**, **`--learning-starts 10000`**, 리플레이 **`--buffer-size 1000000`** (RAM 부족 시 `500000` 등으로 줄이기).
- 체크포인트: `checkpoints/sac_pmi_track.zip`
- MLP 정책은 GPU 이득이 작아 **`train_sac.py` 기본 `device=cpu`**
- 환경·보상·초기 노이즈·ΔF 스케일 등은 **`train_sac.py --help`** 의 `PMITrackEnv` 절 (`env_args.py`와 동일 플래그를 `run_policy.py` / `record_eval_video.py` 에도 사용).

### 학습 튜닝 예시 (더 긴 학습·가중·스케일·초기 분포)

```bash
# 더 길게 + 작은 초기 노이즈(평가 분포와 맞추려면 학습·run_policy 모두 같은 값)
../.venv/bin/python train_sac.py --timesteps 1000000 --reset-noise 0.03

# ΔF 스케일 축소 → 잔여력 보수적 (진동 줄이기 시도)
../.venv/bin/python train_sac.py --delta-f-scale "50,50,50,25,25"

# 위치 추적 더 중시, 행동 페널티 완화
../.venv/bin/python train_sac.py --w-pos 2.0 --action-penalty 5e-5 --tau-limit 500
```

학습에 준 **`PMITrackEnv` 관련 인자는 평가 시에도 동일하게** 넘겨야 합니다.

### 학습 과정 웹 페이지 (TensorBoard)

Stable-Baselines3가 `rl_pmi/tensorboard_logs/` 에 스칼라 로그(보상, 손실 등)를 쌓습니다. **`tensorboard` 패키지**가 설치돼 있어야 하며(`requirements.txt` 포함), 학습 시 **`--no-tensorboard` 를 주지 않은 경우**에만 기록됩니다.

별도 터미널에서:

```bash
cd rl_pmi
../.venv/bin/tensorboard --logdir tensorboard_logs --port 6006
```

브라우저에서 **`http://localhost:6006`** (또는 `http://127.0.0.1:6006`) 을 열면 됩니다.  
**WSL2** 는 Windows 10/11에서 대부분 `localhost` 가 WSL로 자동 연결되므로, TensorBoard를 **WSL 터미널**에서 띄운 뒤 **Windows Edge/Chrome** 으로 같은 주소를 열면 됩니다.

연결이 안 되면 TensorBoard를 모든 인터페이스에 바인딩합니다:

```bash
../.venv/bin/tensorboard --logdir tensorboard_logs --port 6006 --bind_all
```

그래도 안 되면 WSL의 IP를 확인합니다 (WSL 안에서):

```bash
hostname -I | awk '{print $1}'
```

Windows 브라우저에서 **`http://<위_IP>:6006`** 으로 접속합니다 (방화벽 팝업이 뜨면 허용).

원격 **진짜 리눅스 서버**면 SSH 포트 포워딩: `ssh -L 6006:localhost:6006 user@host`

> 3D 로봇 영상은 TensorBoard에 안 나오고, 스칼라 위주입니다. 영상은 위「영상 가시화」절을 쓰세요. 더 예쁜 클라우드 대시보드를 원하면 **Weights & Biases(wandb)** 같은 외부 서비스를 코드에 붙이는 방식이 일반적입니다.

옵션 예: `train_sac.py --help` 참고. 행동 페널티를 \(\|a\|^2\)로 바꾸려면 `--no-residual-tau-penalty`

PPO만 돌릴 때 (기본 50만 스텝, 환경 인자는 SAC와 동일):

```bash
../.venv/bin/python train_ppo.py
```

## 영상 가시화

- **학습 중:** Stable-Baselines3 `VecVideoRecorder`로 주기적으로 mp4 저장 (`moviepy`, 시스템에 **ffmpeg** 권장).

```bash
../.venv/bin/python train_sac.py --record-video \\
  --video-folder videos/train --video-freq 50000 --video-length 400
```

- **학습 후 (체크포인트 한 에피소드):**

```bash
../.venv/bin/python record_eval_video.py --model checkpoints/sac_pmi_track.zip --out videos/eval.mp4
```

`PMITrackEnv`는 `render_mode="rgb_array"`일 때 MuJoCo `Renderer`로 프레임을 뽑습니다. `MUJOCO_GL=egl` 등은 서버/헤드리스와 동일하게 환경 변수로 설정하면 됩니다.

## 시뮬레이션으로 결과 확인

학습된 정책은 **MuJoCo 안의 같은 환경**에서 재생하면 됩니다. 새 코드 없이 아래만 순서대로 쓰면 됩니다.

| 목적 | 명령 ( `rl_pmi` 디렉터리에서 ) |
|------|--------------------------------|
| 에피소드 리턴·추적 오차 로그만 빠르게 | `../.venv/bin/python run_policy.py --model checkpoints/sac_pmi_track.zip --episodes 5` |
| 한 번 돌린 장면을 **mp4**로 저장 | `../.venv/bin/python record_eval_video.py --model checkpoints/sac_pmi_track.zip --out videos/eval.mp4` |

**체크리스트**

1. **`checkpoints/sac_pmi_track.zip`** 이 있는지 확인 (다른 저장 경로면 `--model` 에 그 경로).
2. 학습 때 **`train_sac.py`에 준 옵션**을 추론에도 맞추기: 예를 들어 학습에 `--no-gravity-ff` 를 썼다면  
   `run_policy.py` / `record_eval_video.py` 에도 **`--no-gravity-ff`** 를 같이 준다.
3. WSL·서버 등 **디스플레이 없을 때** 영상 녹화는 보통 앞에 `MUJOCO_GL=egl` 을 붙인다.  
   예: `MUJOCO_GL=egl ../.venv/bin/python record_eval_video.py ...`
4. `record_eval_video.py` 는 **moviepy** 필요 (`pip install moviepy` 또는 `requirements.txt` 기준 설치).

숫자만 보려면 **`run_policy.py` 만** 있으면 되고, 움직임을 보려면 **`record_eval_video.py`** 로 만든 mp4를 플레이어로 연다.

### 학습이 잘 됐는지 확인하기

이 환경의 보상은 **추적 오차 비용의 음수**이므로, 에피소드 **리턴(보상 합)이 클수록**(0에 가까워질수록, 덜 음수일수록) 전반적으로 잘 따라간 것입니다.

1. **TensorBoard** (`tensorboard --logdir tensorboard_logs`)  
   - **`rollout/ep_rew_mean`**(또는 유사 이름)이 시간에 따라 **위로**(덜 나쁘게) 가는지 본다.  
   - **`train/`** 쪽 손실이 NaN·폭주하면 학습이 불안정할 수 있다.

2. **`run_policy.py`** 로 여러 에피소드 실행  
   - 출력되는 **`return`** 이 합리적인지, **`err_pos_norm`** 이 작은지 본다.  
   - 가능하면 **학습 초·중·막 저장본 zip**을 각각 불러 같은 조건으로 비교하면 개선 여부가 분명해진다.

3. **영상 (`eval.mp4` 등)**  
   - 말단이 목표 궤적을 **눈으로 따라가는지**, 과도한 진동·이탈이 없는지 확인한다.

4. **베이스라인과 비교**  
   - **순수 VSD만**(이론상 \(\Delta F=0\)에 가까운 행동)이나 **무작위 행동**보다 리턴이 **분명히 좋으면** RL 보정이 효과가 있는 편이다.

절대적인 “좋은 숫자” 한 줄로 단정하기는 어렵고, **같은 `PMITrackEnv` 설정**에서 학습 전후·다른 체크포인트와 **상대 비교**하는 방식이 가장 안전하다.

---

## 학습 결과 적용 (추론)

학습 산출물은 **`checkpoints/sac_pmi_track.zip`** 하나면 됩니다. Stable-Baselines3가 **정책·값 네트워크 가중치**를 들고 있으며, 적용 시에는 같은 **`PMITrackEnv`** 위에서 `predict` 하면 됩니다.

1. **환경을 학습 때와 맞출 것**  
   `tau_limit`, 중력 feedforward(`--no-gravity-ff`), 잔여 토크 페널티(`--no-residual-tau-penalty`) 등이 다르면 입력 분포·토크 한계가 달라져 성능이 깨질 수 있습니다.

2. **바로 실행 (점수·스텝만 확인)**  

```bash
../.venv/bin/python run_policy.py --model checkpoints/sac_pmi_track.zip --episodes 3
```

3. **영상으로 확인**  

```bash
../.venv/bin/python record_eval_video.py --model checkpoints/sac_pmi_track.zip --out videos/eval.mp4
```

4. **자신의 코드에 넣기**  

```python
from stable_baselines3 import SAC
from envs.pmi_track_env import PMITrackEnv

model = SAC.load("checkpoints/sac_pmi_track.zip", device="cpu")
env = PMITrackEnv()  # 학습 시와 동일한 생성자 인자 권장
obs, _ = env.reset(seed=0)
done = False
while not done:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    done = terminated or truncated
```

실제 로봇·하드웨어에 붙일 때는 여기서 나온 **`action`(5차원)** 이 환경 안에서 이미 \(\Delta F\)로 바뀌어 \(\tau\)까지 계산되는 구조이므로, **실기에서는 동일한 매핑(해석 기구학·\(J\), \(K_s,K_d\), `delta_f_scale`)과 통신 인터페이스**를 맞추어야 합니다 (MuJoCo 대신 실측 \(q,\dot q\) 등).

## 파일

| 파일 | 설명 |
|------|------|
| `docs/pmi_track_env.md` | `PMITrackEnv` 제어·관측·보상·파라미터 명세 (코드 변경 시 함께 갱신) |
| `trajectory.py` | `run_vsd` 와 동일 웨이포인트·`path_generation` 으로 궤적 생성 |
| `env_args.py` | `PMITrackEnv` 공통 CLI (`train_sac` / `run_policy` / `record_eval_video` / `train_ppo`) |
| `envs/pmi_track_env.py` | Gymnasium 환경 |
| `train_sac.py` | Stable-Baselines3 **SAC** 학습 (`--record-video` 로 학습 중 녹화) |
| `record_eval_video.py` | 학습된 SAC zip으로 rollout mp4 저장 |
| `run_policy.py` | 학습된 SAC zip으로 에피소드 실행 (숫자 로그만, moviepy 불필요) |
| `train_ppo.py` | Stable-Baselines3 PPO 학습 (비교용) |

## 참고

- 관측·보상 스케일은 `PMITrackEnv` 생성자 인자로 조정 가능합니다.
- 본격 학습 전에는 `tau_limit`, `ks`/`kd`/`delta_f_scale`, 가중치, episode 길이(`t_end`)를 시뮬 안정성에 맞게 튜닝하는 것이 좋습니다.
