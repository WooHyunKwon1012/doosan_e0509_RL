# Joint Pose Sequencer GUI

PyQt5 기반의 MoveIt Joint Pose 녹화 및 재생 GUI입니다.

## 파일 구조

- `gui_moveit.py` - 메인 GUI 애플리케이션
- `gui_config.py` - GUI 치수, 스타일, 설정 관리 파일
- `joint_state_bridge.py` - MoveIt과 Isaac Sim 연결 브리지

## 기능

### 📸 포즈 캡처
- 현재 `/joint_states`에서 받은 joint 값들을 실시간 표시
- 포즈 이름과 duration을 설정하고 캡처 버튼으로 저장

### 📋 포즈 관리
- **▲▼**: 포즈 순서 변경
- **✏️ Edit**: 포즈의 joint 값, 이름, duration 편집
- **🗑️ Delete**: 선택한 포즈 삭제
- **📋 Copy**: 선택한 포즈 복제
- **🔄 Clear All**: 모든 포즈 삭제

### ▶️ 재생 제어
- **▶️ Play All**: 저장된 모든 포즈를 순서대로 재생
- **⏯️ Go to Selected**: 선택한 포즈로 즉시 이동
- **⏹️ Stop**: 재생 중지
- **🔁 Loop**: 반복 재생 옵션

### 💾 파일 관리
- **Save Sequence**: 포즈 시퀀스를 JSON 파일로 저장
- **Load Sequence**: JSON 파일에서 포즈 시퀀스 불러오기

## 사용 방법

### 1. 필요한 패키지 설치
```bash
sudo apt install python3-pyqt5
```

### 2. MoveIt 실행
터미널 1:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=e0509 host:=127.0.0.1
```

### 3. (선택) Isaac Sim 연결
터미널 2 - Bridge 실행:
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/doosan-robot2/isaacsim_connect/joint_state_bridge.py
```

Isaac Sim에서 `e0509_env_ros.py` 실행

### 4. GUI 실행
터미널 3:
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/doosan-robot2/isaacsim_connect/gui_moveit.py
```

## 워크플로우

1. MoveIt에서 로봇을 원하는 포즈로 이동
2. GUI에서 포즈 이름과 duration 설정 후 "📸 Capture Pose" 클릭
3. 여러 포즈를 캡처하여 시퀀스 생성
4. ▲▼ 버튼으로 순서 조정
5. "▶️ Play All"로 전체 시퀀스 재생 또는 "⏯️ Go to Selected"로 개별 포즈 실행
6. "💾 Save Sequence"로 저장하여 나중에 재사용

## ROS2 토픽

### 구독
- `/joint_states` (sensor_msgs/JointState): MoveIt에서 받은 현재 joint 상태

### 발행
- `/dsr_moveit_controller/joint_trajectory` (trajectory_msgs/JointTrajectory): MoveIt로 보내는 trajectory 명령

## 파일 형식

저장된 JSON 파일 예시:
```json
{
  "poses": [
    {
      "name": "Home",
      "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "duration": 2.0
    },
    {
      "name": "Pick",
      "joints": [0.5, -0.5, 0.3, 0.0, 0.8, 0.0],
      "duration": 3.0
    }
  ]
}
```

## 주의사항

- GUI는 별도의 스레드에서 ROS2를 실행하므로 블로킹 없이 부드럽게 동작합니다
- 재생 중에는 각 포즈 사이에 0.5초의 여유 시간이 추가됩니다
- Loop 모드에서는 Stop 버튼을 누르기 전까지 계속 반복됩니다
- Joint 값은 라디안 단위로 표시됩니다 (-3.14 ~ 3.14)

## GUI 설정 커스터마이징

GUI의 모든 치수와 스타일은 `gui_config.py` 파일에서 관리됩니다.

### 주요 설정 항목:

**창 크기 조정:**
```python
WINDOW_WIDTH = 1200   # 창 가로 크기
WINDOW_HEIGHT = 750   # 창 세로 크기
```

**버튼 크기 조정:**
```python
BUTTON_HEIGHT = 32          # 모든 버튼 높이
BUTTON_ARROW_WIDTH = 40     # ▲▼ 버튼 폭
BUTTON_SPACING = 5          # 버튼 사이 간격
```

**리스트 설정:**
```python
LIST_MIN_HEIGHT = 250              # 리스트 최소 높이
LIST_ALTERNATING_COLORS = True     # 교대 행 색상 활성화
```

**Duration 설정:**
```python
DURATION_MIN = 0.1        # 최소값
DURATION_MAX = 10.0       # 최대값
DURATION_STEP = 0.1       # 증감 단위
DURATION_DEFAULT = 2.0    # 기본값
```

**Joint 각도 설정:**
```python
JOINT_COUNT = 6               # 조인트 개수
JOINT_ANGLE_MIN = -180.0      # 최소 각도 (도)
JOINT_ANGLE_MAX = 180.0       # 최대 각도 (도)
JOINT_ANGLE_STEP = 1.0        # 증감 단위 (도)
JOINT_ANGLE_DECIMALS = 2      # 소수점 자리수
```

**스타일 변경:**
```python
FONT_SIZE_SECTION_LABEL = '11pt'    # 섹션 제목 폰트 크기
PADDING_STATUS_LABEL = 8            # 상태 라벨 패딩
```

설정 변경 후 GUI를 재시작하면 바로 적용됩니다.

## 트러블슈팅

### GUI가 joint state를 받지 못하는 경우
```bash
# MoveIt이 실행 중인지 확인
ros2 topic echo /joint_states
```

### Trajectory가 전송되지 않는 경우
```bash
# MoveIt controller가 활성화되어 있는지 확인
ros2 topic list | grep joint_trajectory
```

### Isaac Sim 연동이 안 되는 경우
- joint_state_bridge.py가 실행 중인지 확인
- Isaac Sim에서 ROS2 bridge가 활성화되어 있는지 확인
- `/joint_input` 토픽이 발행되고 있는지 확인
  ```bash
  ros2 topic echo /joint_input
  ```
