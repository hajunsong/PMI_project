# `pmi_description`을 RViz에서 보기

`pmi_description` 패키지의 URDF(`urdf/pmi_description.urdf`)와 메시를 **RViz**에서 확인하는 절차입니다.  
Ubuntu 24.04 기준으로 **ROS 2 Jazzy** + **RViz 2**를 가정합니다.

---

## 1. 사전 준비

### ROS 2 Jazzy 설치 (미설치인 경우)

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

터미널에서 매 세션:

```bash
source /opt/ros/jazzy/setup.bash
```

(선택) 셸에 자동 적용:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### ROS 2용 패키지 형식

이 디렉터리의 `package.xml` / `CMakeLists.txt`가 **catkin(ROS 1)** 이면 `colcon`으로 빌드되지 않습니다.  
ROS 2에서 쓰려면 **ament_cmake** 패키지로 맞춰야 합니다.

**`package.xml`** (format 3 예시):

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pmi_description</name>
  <version>1.0.0</version>
  <description>URDF for pmi_description</description>
  <maintainer email="TODO@email.com">TODO</maintainer>
  <license>BSD</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**`CMakeLists.txt`** (최소 예시):

```cmake
cmake_minimum_required(VERSION 3.8)
project(pmi_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch meshes urdf DESTINATION share/${PROJECT_NAME})
ament_package()
```

GUI 슬라이더가 필요하면:

```bash
sudo apt install ros-jazzy-joint-state-publisher-gui
```

그리고 런치에서 `joint_state_publisher` 대신 `joint_state_publisher_gui` 노드를 사용합니다.

---

## 2. 워크스페이스 구성

표준은 패키지가 `src` 아래에 있는 형태입니다.

```bash
mkdir -p ~/PMI_project/ros_ws/src
# 이미 이 저장소를 쓰는 경우 예시:
cd ~/PMI_project/ros_ws
# pmi_description이 src 안에 없다면 심볼릭 링크:
# ln -sf "$(pwd)/pmi_description" src/pmi_description
```

패키지 경로가 `ros_ws/src/pmi_description` 이 되도록 한 뒤 빌드합니다.

---

## 3. 빌드 및 환경 로드

```bash
cd ~/PMI_project/ros_ws   # 또는 본인의 ros_ws 경로
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pmi_description
source install/setup.bash
```

빌드 오류가 나면 위 **ament_cmake** 설정과 `launch` 안의 Python 런치 파일이 패키지에 포함되는지 확인하세요.

---

## 4. 실행 (RViz 2)

저장소에 **`launch/display.launch.py`** 가 있으면:

```bash
source ~/PMI_project/ros_ws/install/setup.bash
ros2 launch pmi_description display.launch.py
```

- **RobotModel**이 안 보이면 RViz에서 **Add → RobotModel**, **Fixed Frame**을 URDF 루트 링크(예: `base_link`)에 맞춥니다.
- **Description Topic**은 보통 `/robot_description`(스트링 파라미터) 또는 `/robot_description` 토픽 설정을 런치에 맞춥니다. `robot_state_publisher`만 쓰는 경우 TF와 `/robot_description` 파라미터를 확인합니다.

### `joint_state_publisher_gui` 를 쓰는 경우

별도 런치 파일이 있다면 예:

```bash
ros2 launch pmi_description display_gui.launch.py
```

없다면 `display.launch.py`에서 `joint_state_publisher` 노드를 `joint_state_publisher_gui`로 바꾼 런치를 하나 두면 됩니다.

---

## 5. WSL2 / 원격 화면

- **Windows 11 WSLg**: 보통 추가 설정 없이 RViz 창이 뜹니다.
- 창이 안 뜨면 `echo $DISPLAY` 확인, 또는 X 서버 사용 시 `DISPLAY` 설정을 맞춥니다.

---

## 6. 자주 나는 오류

| 증상 | 조치 |
|------|------|
| `package 'pmi_description' not found` | `source install/setup.bash` 했는지, `colcon build` 성공 여부 확인 |
| `joint_state_publisher` 관련 패키지 없음 | `sudo apt install ros-jazzy-joint-state-publisher` |
| 메시(STL) 안 보임 | URDF의 `package://pmi_description/meshes/...` 가 설치 경로 `share/pmi_description/meshes` 와 일치하는지, 빌드 후 `install/pmi_description/share/pmi_description/` 에 `meshes` 가 복사되었는지 확인 |
| RViz에서 모델만 안 보임 | Fixed Frame을 `base_link` 등으로 설정, RobotModel 표시 추가 |

---

## 7. 파일 구조 참고

```
pmi_description/
  urdf/pmi_description.urdf   # 로봇 기술서
  meshes/*.STL               # 시각/충돌 메시
  launch/display.launch.py    # ROS 2 런치 예시(있는 경우)
  RVIZ.md                     # 본 문서
```

질문이나 패키지 이름이 `pmi_description`이 아닌 경우, 런치와 빌드 패키지 이름을 동일하게 유지하면 됩니다.
