# mujoco_pmi_viz

PMI 4축 매니퓰레이터를 **MuJoCo**로 재생·비교·(선택) 녹화하기 위한 스크립트와 MJCF 모델을 모아 둔 디렉터리입니다. 해석 코드는 상위 `analysis/python/`의 `main.py`와 연동됩니다.

## 디렉터리 구성

| 경로 | 설명 |
|------|------|
| `models/pmi_arm_primitive.xml` | 캡슐/박스 근사 기하. URDF(`ros_ws/pmi_description/urdf/pmi_description.urdf`)와 동일한 질량·관성(`inertial`, `inertiafromgeom=false`). 재생·비교용 기본 모델(STL 없을 때). |
| `models/pmi_arm_primitive_actuated.xml` | 위와 동일 기구 + 관절 **motor** 액추에이터. `main.run_vsd(integrate_with_mujoco=True)` 에서 사용. |
| `models/pmi_arm_mesh.xml` | STL 메시 시각화(`meshdir` → `ros_ws/pmi_description/meshes/`). STL이 없으면 primitive가 선택되는 편이 안전. |
| `play_trajectory.py` | `data_save` 형식 CSV의 관절각(`qi`, 열 31–34)으로 자세 재생, 선택적으로 MP4 녹화. |
| `compare_ee_trajectory.py` | CSV의 EE(`re`) vs MuJoCo `site ee`, 선택적으로 두 CSV 간 관절각 비교 플롯. |
| `requirements.txt` | `mujoco`, `numpy`, `matplotlib`, `imageio`, `imageio-ffmpeg` 등. |

## 환경 설정

```bash
cd mujoco_pmi_viz
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt
```

헤드리스/원격에서 렌더 오류가 나면 `MUJOCO_GL=egl` 또는 `osmesa` 등 환경에 맞게 설정합니다.

## 궤적 재생 (인터랙티브 뷰어)

`analysis/python`에서 생성한 예: `python_data_vsd.csv`, `python_data_path.csv`  
열 형식은 `main.py`의 `data_save()`와 동일해야 합니다.

```bash
./.venv/bin/python play_trajectory.py --csv ../analysis/python/python_data_vsd.csv
```

- `--model` 으로 MJCF 경로 지정 가능. 미지정 시 `meshes/base_link.STL` 존재 여부로 mesh vs primitive 자동 선택.
- `--use-actuator-q`: 열 19–22 (`qi_act`) 사용 시.

## MP4 녹화

뷰어를 연 뒤 창을 닫으면, 동일 궤적을 오프스크린 렌더하여 저장합니다.

```bash
./.venv/bin/python play_trajectory.py --csv ../analysis/python/python_data_vsd.csv --record traj.mp4
```

뷰만 생략:

```bash
MUJOCO_GL=egl ./.venv/bin/python play_trajectory.py \
  --csv ../analysis/python/python_data_vsd.csv --record traj.mp4 --no-viewer
```

- 해상도 기본 640×480 (더 크게 하려면 MJCF `<visual><global offwidth="..." offheight="..."/></visual>` 필요).

## EE·관절 비교 플롯

```bash
./.venv/bin/python compare_ee_trajectory.py --csv ../analysis/python/python_data_vsd.csv --save ee_compare.png
```

- 두 실험의 관절열을 비교할 때: `--csv-q-b 다른파일.csv`
- 저장물 예: `ee_compare.png`, `ee_compare_error.png`, `ee_compare_joints.png`, `ee_compare_joints_diff.png`

## `main.py` 와의 연결

- **IK 로그 재생**: CSV에 `qi` 열이 있으면 그대로 재생 가능.
- **작업공간 시뮬 (`run_vsd`)**: `integrate_with_mujoco=True` 일 때 `pmi_arm_primitive_actuated.xml` 경로(프로젝트 루트 기준)를 로드합니다. 중력 보상·PD 스케일 등은 `run_vsd` 인자를 참고하세요.

## Git에서 제외하는 파일

로컬 전용·대용량 산출물은 저장소에 넣지 않습니다. 루트 `.gitignore` 및 본 폴더 `.gitignore`에 예: `.venv/`, `*.mp4`, `ee_compare*.png`, `__pycache__/` 등이 포함되어 있습니다.
