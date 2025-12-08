# GUI Cartesian Control & Gripper Integration - 변경 사항 요약

## 📝 주요 파일

### 1. `gui_add_cartesian_control.py` ✅ 최신 버전

**완전한 기능을 갖춘 GUI 애플리케이션**

#### 핵심 기능:

##### 1) 탭 기반 실시간 제어 🎮
- **Tab 0: Joint Control** 
  - 6개 관절 슬라이더 (-180° ~ 180°)
  - 그리퍼 슬라이더 (0° ~ 63.1°)
  - Send Joint Command 버튼
  - Duration: 0.5초
  
- **Tab 1: Cartesian Control** ✅ NEW
  - Position (XYZ) SpinBox
  - Orientation (RPY) SpinBox
  - Send Cartesian Command 버튼
  - Get Current Pose 버튼
  - Duration: 2.0초

##### 2) 상호 배타적 제어
- Real-time Mode OFF: 슬라이더가 `/joint_pos` 피드백으로 자동 업데이트
- Real-time Mode ON: 슬라이더 고정, 사용자가 직접 조작
- Joint 탭 선택 → Cartesian 명령 비활성화
- Cartesian 탭 선택 → Joint 명령 비활성화

##### 3) IK/FK Service 통합 ✅ NEW
- `/compute_ik`: XYZ+RPY → Joint 변환 (비동기)
- `/compute_fk`: Joint → XYZ+RPY 변환 (동기)
- 에러 코드 해석 (NO_IK_SOLUTION 등)
- 사용자 친화적 에러 메시지

##### 4) 그리퍼 제어
- ROS Node 내부에서 점진 제어 (10Hz)
- 목표값으로 부드럽게 이동 (0.05 rad/tick)
- 실시간 상태 모니터링

---

### 2. `joint_state_bridge_with_gripper.py` ✅

#### 변경 사항:
- `/gripper_input` 구독 추가 (Float32, 라디안 단위)
- `/joint_input` 발행 시 10개 값으로 확장
  - Index 0~5: 로봇 팔 관절 (MoveIt2에서)
  - Index 6: 0.0 (사용 안 함)
  - Index 7: 그리퍼 값 (라디안) ✅
  - Index 8~9: 0.0 (사용 안 함)

#### 주요 코드:
```python
# 그리퍼 값 구독
self.gripper_subscription = self.create_subscription(
    Float32,
    '/gripper_input',
    self.gripper_callback,
    10
)

# 10개 DOF 배열 생성
full_positions = list(joint_positions)  # 6개
full_positions.append(0.0)               # index 6
full_positions.append(self.gripper_value) # index 7 (그리퍼, 라디안)
full_positions.append(0.0)               # index 8
full_positions.append(0.0)               # index 9
```

---

### 3. `e0509_jointpos_with_gripper_ros2.py` ✅

#### 변경 사항:
- `/joint_input` 구독 (10 DOF)
- Index 7을 그리퍼로 자동 처리
- `/joint_pos` 발행 (30Hz 최적화)
- `/gripper_input` 직접 구독 지원

---

## 🔄 전체 시스템 흐름

### 시작 순서 (4개 터미널)

```bash
# Terminal 1: Isaac Sim
python3 e0509_jointpos_with_gripper_ros2.py

# Terminal 2: MoveIt2
ros2 launch dsr_launcher2 dsr_moveit.launch.py

# Terminal 3: Bridge
ros2 run isaacsim_connect joint_state_bridge_with_gripper.py

# Terminal 4: GUI
python3 gui_add_cartesian_control.py
```

### 데이터 흐름

```
┌─────────────────────────┐
│ GUI (Cartesian Control) │
│ - XYZ + RPY 입력       │
│ - Get Current Pose      │
└─────────────────────────┘
         │
         ├─ /compute_fk → MoveIt2 → XYZ+RPY
         │
         ├─ /compute_ik ← XYZ+RPY → Joint
         │
         └─ /dsr_moveit_controller/joint_trajectory
                   ↓
         ┌─────────────────┐
         │ MoveIt2         │
         │ - IK/FK 계산    │
         │ - 궤적 계획     │
         └─────────────────┘
                   ↓
              /joint_states (6 joints)
                   ↓
         ┌─────────────────┐
         │ Bridge          │
         │ - 메시지 변환   │
         │ - 그리퍼 통합   │
         └─────────────────┘
                   ↓
              /joint_input (10 DOF)
                   ↓
         ┌─────────────────┐
         │ Isaac Sim       │
         │ - 물리 시뮬레이션│
         └─────────────────┘
                   ↓
              /joint_pos (30Hz)
                   ↓
         GUI 모니터링 (슬라이더는 Real-time OFF일 때만 업데이트)
```

---

## 🔄 토픽 흐름

### 새로운 토픽 구조:

```
┌──────────────────────┐
│  gui_moveit.py       │
└──────────────────────┘
    │
    ├─ 구독: /joint_states (MoveIt2, 6 joints)
    ├─ 구독: /joint_pos (Isaac Sim, 10 DOF) ← 새로 추가
    │
    ├─ 발행: /dsr_moveit_controller/joint_trajectory (로봇 팔 제어)
    └─ 발행: /gripper_input (Float32, 라디안) ← 새로 추가
         ↓
┌──────────────────────┐
│ joint_state_bridge.py│
└──────────────────────┘
    │
    ├─ 구독: /joint_states (MoveIt2)
    ├─ 구독: /gripper_input (Float32, 라디안) ← 새로 추가
    │
    └─ 발행: /joint_input (Float32MultiArray, 10 DOF) ← 확장됨
         ↓
         [0~5]: 로봇 팔 관절 (라디안)
         [6]:   0.0
         [7]:   그리퍼 (라디안) ← 그리퍼 값
         [8~9]: 0.0
         ↓
┌──────────────────────┐
│ e0509_jointpos_ros2.py│ (Isaac Sim)
└──────────────────────┘
    │
    ├─ 구독: /joint_input (10 DOF)
    │        → index[7]을 그리퍼로 처리 (자동)
    │
    └─ 발행: /joint_pos (10 DOF) ← GUI가 구독
```

---

## 🎯 사용 방법

### 1. 파일 교체
```bash
cd /home/woo/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_gripper

# 기존 파일 백업
cp gui_moveit.py gui_moveit_old_backup.py

# 새 파일로 교체
mv gui_moveit_new.py gui_moveit.py
```

### 2. 실행 순서
```bash
# 1) Isaac Sim 실행
python3 e0509_jointpos_ros2.py

# 2) MoveIt2 실행
ros2 launch dsr_moveit2 moveit.launch.py

# 3) Bridge 실행
python3 joint_state_bridge.py

# 4) GUI 실행
python3 gui_moveit.py
```

### 3. GUI 사용법

#### 실시간 제어 모드:
1. "Enable Real-time Control" 체크박스 활성화
2. 슬라이더로 관절/그리퍼 값 조정
3. "Send Joint Command" 또는 "Send Gripper Command" 클릭
4. Isaac Sim에서 로봇이 움직이는 것 확인

#### 시퀀스 모드:
1. "Enable Real-time Control" 비활성화
2. "Add Joint Pose" 또는 "Add Gripper Action" 클릭
3. 시퀀스 리스트에 블록 추가
4. "▶️ Play Sequence" 클릭하여 실행

---

## ⚙️ 주요 파라미터

### 그리퍼 제어 속도
`gui_moveit.py` 74라인:
```python
self.gripper_speed = 0.05  # 라디안/틱 (약 2.87도/틱, 10Hz)
```
- 값을 높이면 더 빠르게 이동
- 값을 낮추면 더 부드럽게 이동

### 그리퍼 제어 주파수
`gui_moveit.py` 108라인:
```python
self.gripper_timer = self.create_timer(0.1, self.gripper_control_loop)  # 10Hz
```
- `0.1` = 10Hz (초당 10번 업데이트)
- 값을 줄이면 더 자주 업데이트 (예: `0.05` = 20Hz)

---

## 🧪 테스트 명령

### 1. 토픽 확인
```bash
# 토픽 리스트
ros2 topic list

# /joint_input 확인 (10개 값이어야 함)
ros2 topic echo /joint_input

# /gripper_input 확인
ros2 topic echo /gripper_input

# /joint_pos 확인 (10개 값)
ros2 topic echo /joint_pos
```

### 2. 수동 테스트
```bash
# 그리퍼 열기 (63.1도 = 1.101 라디안)
ros2 topic pub --once /gripper_input std_msgs/Float32 "data: 1.101"

# 그리퍼 닫기
ros2 topic pub --once /gripper_input std_msgs/Float32 "data: 0.0"

# 그리퍼 중간
ros2 topic pub --once /gripper_input std_msgs/Float32 "data: 0.55"
```

---

## 🔧 주의사항

### 1. 모드 전환
- 실시간 모드와 시퀀스 모드는 상호 배타적
- 전환 시 현재 `/joint_pos` 값으로 자동 동기화
- 토픽 충돌 방지를 위해 한 번에 하나의 모드만 사용

### 2. 그리퍼 제어
- GUI에서는 **도(degree) 단위**로 입력 (0~63.1)
- 내부적으로 **라디안**으로 변환하여 전송
- Isaac Sim은 자동으로 처리 (변환 불필요)

### 3. 시퀀스 재생
- Joint 액션과 Gripper 액션은 독립적
- 각 액션마다 개별 duration 설정 가능
- 액션은 순차적으로 실행 (병렬 아님)

### 4. 파일 호환성
- 새 JSON 형식은 `type` 필드로 구분
- 기존 형식과 호환되지 않을 수 있음
- 필요시 수동으로 변환 필요

---

## 🐛 트러블슈팅

### 그리퍼가 움직이지 않을 때
```bash
# 1. /gripper_input 토픽 확인
ros2 topic hz /gripper_input

# 2. /joint_input 확인 (index 7에 값이 있는지)
ros2 topic echo /joint_input

# 3. Isaac Sim 로그 확인
# e0509_jointpos_ros2.py 터미널에서 에러 메시지 확인
```

### GUI가 /joint_pos를 받지 못할 때
```bash
# 1. Isaac Sim이 /joint_pos를 발행하는지 확인
ros2 topic hz /joint_pos

# 2. 노드 연결 확인
ros2 node info /joint_pose_gui

# 3. ROS_DOMAIN_ID 확인
echo $ROS_DOMAIN_ID
```

### 실시간 모드와 시퀀스 모드가 충돌할 때
1. GUI를 완전히 종료
2. 다시 시작
3. 한 번에 하나의 모드만 사용

---

## 📊 변경 요약

| 항목 | 변경 전 | 변경 후 |
|------|---------|---------|
| **joint_state_bridge.py** | 6개 DOF | 10개 DOF (index 7 = 그리퍼) |
| **gui_moveit.py** | Joint만 제어 | Joint + Gripper 제어 |
| **시퀀스 구조** | Joint 포즈만 | Joint + Gripper 블록 |
| **실시간 제어** | 없음 | Joint + Gripper 슬라이더 |
| **모니터링** | /joint_states만 | /joint_pos 추가 |
| **토픽** | 3개 | 5개 (+gripper_input, +joint_pos 구독) |

---

## 🎉 완료!

이제 GUI에서 로봇 팔과 그리퍼를 모두 제어할 수 있습니다!

- ✅ 실시간 슬라이더 제어
- ✅ 시퀀스 블록 구성 (Joint + Gripper)
- ✅ 라이브러리 관리
- ✅ 부드러운 그리퍼 이동
- ✅ 실시간 상태 모니터링

질문이나 문제가 있으면 언제든지 알려주세요!
