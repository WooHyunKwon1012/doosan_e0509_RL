# 🏗️ E0509 Robot with Surface Gripper - System Architecture

> **Last Updated**: 2025-12-11  
> **Isaac Sim Version**: 5.1  
> **ROS2 Distribution**: jazzy 

---

## 📋 목차
1. [전체 시스템 구조도](#전체-시스템-구조도)
2. [컴포넌트별 상세 구조](#컴포넌트별-상세-구조)
3. [데이터 흐름도](#데이터-흐름도)
4. [ROS2 토픽 맵](#ros2-토픽-맵)
5. [파일 구조](#파일-구조)

---

## 🎯 전체 시스템 구조도

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          USER INTERFACE (PyQt5)                             │
│                     gui_with_surface_gripper.py                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ 📊 Current State Monitor (/joint_pos)                              │    │
│  │   • J1~J6 Joint Angles (degrees)                                   │    │
│  │   • Gripper Angle (degrees + radians)                              │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ 🎮 Real-time Control (접기/펼치기)                                  │    │
│  │ ☐ Enable Real-time Control                                         │    │
│  │                                                                      │    │
│  │   🤏 Gripper Control (공통)                                         │    │
│  │     • Gripper Angle Slider (0~63.1°)                               │    │
│  │     • Send Gripper Angle → /gripper_input                          │    │
│  │     • [Open Gripper] → /gripper_command (0)                        │    │
│  │     • [Close Gripper] → /gripper_command (1)                       │    │
│  │                                                                      │    │
│  │   [탭] 🤖 Joint Control                                             │    │
│  │     • 6개 Joint Sliders (-180° ~ 180°)                             │    │
│  │     • Send Joint Command → /joint_trajectory                       │    │
│  │     • Get Current Joint State                                      │    │
│  │                                                                      │    │
│  │   [탭] 🗺️ Cartesian Control                                         │    │
│  │     • Position: X, Y, Z (meters)                                   │    │
│  │     • Orientation: Roll, Pitch, Yaw (degrees)                      │    │
│  │     • Send Cartesian Command (IK 사용)                             │    │
│  │     • Get Current Pose                                             │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌──────────────────────────────┬──────────────────────────────────────┐   │
│  │ 📝 Sequence Editor (60%)     │ 📦 Pose & Action Blocks (40%)       │   │
│  │                              │                                      │   │
│  │ [📸][➕][🤏][🟢][🔴][🗺️][⏱️]   │ [JOINT] zero pose                   │   │
│  │                              │ [JOINT] cup up                      │   │
│  │ 1. [JOINT] zero pose (2.0s)  │ [GRIPPER] cup grip                  │   │
│  │ 2. 🔴 [GRIPPER CLOSE]        │ [CARTESIAN] pick position           │   │
│  │ 3. [JOINT] cup up (3.0s)     │                                      │   │
│  │ 4. 🟢 [GRIPPER OPEN]         │ [💾 Save] [← Add] [🗑️]              │   │
│  │ 5. 🗺️ [CARTESIAN] pick (2s)  │                                      │   │
│  │ 6. ⏱️ [SLEEP] 1.0s            │                                      │   │
│  │ ...                          │                                      │   │
│  │                              │                                      │   │
│  │ [✏️ Edit] [🗑️ Delete] [▲][▼] │                                      │   │
│  └──────────────────────────────┴──────────────────────────────────────┘   │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ ▶️ Playback Control                                                 │    │
│  │   [▶️ Play Sequence] [⏹️ Stop]    Progress: ████████░░ 80%         │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ 📚 Sequence Libraries                                               │    │
│  │   • pick_and_place_demo.json                                       │    │
│  │   • calibration_routine.json                                       │    │
│  │   [💾 Save] [📂 Load] [🗑️ Delete]                                   │    │
│  └────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ▼
        ┌───────────────────────────────────────────────────────┐
        │            ROS2 Communication Layer                   │
        └───────────────────────────────────────────────────────┘
                                    ▼
┌──────────────────────┬──────────────────────┬──────────────────────────────┐
│   Joint State        │   Joint Trajectory   │    Gripper Control          │
│   Bridge             │   Controller         │    Topics                   │
│                      │                      │                             │
│ joint_state_bridge.py│  (MoveIt2 내부)      │  /gripper_input (Float32)  │
│                      │                      │  /gripper_command (Int32)   │
│                      │                      │    0 = Open                 │
│ /joint_states (6DOF) │ /joint_trajectory    │    1 = Close                │
│        ↓             │        ↓             │        ↓                    │
│ /joint_input (10DOF) │  Trajectory          │  Gripper Commands           │
│  [0-5]: Joint angles │  Execution           │                             │
│  [6]: 0.0 (dummy)    │                      │                             │
│  [7]: Gripper angle  │                      │                             │
│  [8-9]: 0.0 (dummy)  │                      │                             │
└──────────────────────┴──────────────────────┴──────────────────────────────┘
                                    ▼
        ┌───────────────────────────────────────────────────────┐
        │          Isaac Sim 5.1 (Physics Engine)              │
        └───────────────────────────────────────────────────────┘
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                  isaacsim_e0509_surface_gripper_in_usd_pick_all.py         │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ 🌍 World Setup (backend="numpy")                                   │    │
│  │   • Room environment (room_without_e0509.usd)                      │    │
│  │   • Robot model (e0509_model.usd)                                  │    │
│  │   • Objects (teddy_bear, cups, etc.)                               │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ 🤖 Robot Controller (SingleManipulator)                            │    │
│  │   • Articulation: /World/e0509_model/e0509                         │    │
│  │   • DOF: 10 (6 joints + 4 gripper-related)                         │    │
│  │   • Joint Names: [joint_1, ..., joint_6, rh_p12_rn, ...]          │    │
│  │                                                                      │    │
│  │   ROS2 Subscriptions:                                               │    │
│  │     • /joint_input (Float32MultiArray) - 10 DOF control            │    │
│  │     • /gripper_command (Int32) - Open/Close commands               │    │
│  │                                                                      │    │
│  │   ROS2 Publications:                                                │    │
│  │     • /joint_pos (Float32MultiArray) - Current joint positions     │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ 🤏 Surface Gripper (GripperView)                                    │    │
│  │   • USD Prim: /World/e0509_model/e0509/gripper/gripper/           │    │
│  │                 SurfaceGripper                                      │    │
│  │                                                                      │    │
│  │   Attachment Points (4 finger links):                               │    │
│  │     1. rh_p12_rn_l1                                                │    │
│  │     2. rh_p12_rn_l2                                                │    │
│  │     3. rh_p12_rn_r1                                                │    │
│  │     4. rh_p12_rn_r2                                                │    │
│  │                                                                      │    │
│  │   Properties:                                                       │    │
│  │     • max_grip_distance: 0.2m (20cm)                               │    │
│  │     • coaxial_force_limit: 10,000N                                 │    │
│  │     • shear_force_limit: 10,000N                                   │    │
│  │     • contactOffset: 0.02m                                         │    │
│  │                                                                      │    │
│  │   Control:                                                          │    │
│  │     • /gripper_command 0 → gripper.close()                         │    │
│  │     • /gripper_command 1 → gripper.open()                          │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ ⚙️ Physics Simulation Loop                                          │    │
│  │   1. Read ROS2 commands (/joint_input, /gripper_command)           │    │
│  │   2. Apply ArticulationAction to robot                             │    │
│  │   3. Step physics simulation (my_world.step())                     │    │
│  │   4. Update Surface Gripper state                                  │    │
│  │   5. Publish current state (/joint_pos)                            │    │
│  │   6. Check gripper attachment status                               │    │
│  │   7. Loop at ~60Hz                                                 │    │
│  └────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔧 컴포넌트별 상세 구조

### 1️⃣ **GUI Application** (`gui_with_surface_gripper.py`)

```python
MainWindow (QMainWindow)
├── ROSThread (별도 스레드)
│   └── JointStateMonitor (rclpy.Node)
│       ├── Subscriptions:
│       │   ├── /joint_states (MoveIt2 → 6 DOF)
│       │   └── /joint_pos (Isaac Sim → 10 DOF feedback)
│       │
│       └── Publishers:
│           ├── /joint_trajectory (Trajectory control)
│           ├── /joint_input (10 DOF direct control)
│           ├── /gripper_input (Float32 - angle in radians)
│           └── /gripper_command (Int32 - 0=open, 1=close)
│
├── UI Widgets
│   ├── Monitor Panel (상태 표시)
│   ├── Real-time Control Panel (접기 가능)
│   │   ├── Gripper Control (공통)
│   │   └── Tabs:
│   │       ├── Joint Control
│   │       └── Cartesian Control
│   │
│   ├── Sequence Editor (가로 분할)
│   │   ├── Action Buttons (7개 버튼 한 줄)
│   │   ├── Sequence List (QListWidget)
│   │   └── Edit Controls (Edit, Delete, ▲, ▼)
│   │
│   ├── Blocks Panel (오른쪽)
│   │   ├── Saved Poses List
│   │   └── Block Management
│   │
│   ├── Playback Control
│   └── Libraries Panel
│
└── Data Structures
    ├── self.sequence: List[(type, data)]
    │   └── Types: 'joint', 'gripper', 'CartesianPose',
    │               'GripperOpen', 'GripperClose', 'Sleep'
    │
    ├── self.single_poses: Dict[filename, (type, data)]
    │   └── Files: [JOINT] name.json, [CARTESIAN] name.json, etc.
    │
    └── self.library_sequences: Dict[filename, sequence]
```

### 2️⃣ **Joint State Bridge** (`joint_state_bridge.py`)

```python
JointStateBridge (rclpy.Node)
│
├── Input Topics:
│   ├── /joint_states (sensor_msgs/JointState)
│   │   └── From: MoveIt2 (6 joints)
│   │       • joint_1, joint_2, ..., joint_6
│   │
│   └── /gripper_input (std_msgs/Float32)
│       └── From: GUI
│           • Gripper angle in radians (0 ~ 1.10 rad)
│
├── Processing:
│   └── Combine 6 joints + gripper → 10 DOF array:
│       [0-5]: Joint angles (radians)
│       [6]:   0.0 (dummy)
│       [7]:   Gripper angle (radians)
│       [8-9]: 0.0 (dummy)
│
└── Output Topic:
    └── /joint_input (std_msgs/Float32MultiArray)
        └── To: Isaac Sim
            • 10 DOF full robot control
```

### 3️⃣ **Isaac Sim Controller** (`isaacsim_e0509_surface_gripper_in_usd_pick_all.py`)

```python
Main Simulation
│
├── Initialization:
│   ├── World Setup (backend="numpy")
│   ├── Load Environment USD
│   ├── Load Robot USD + Set Position
│   ├── Setup Surface Gripper (USD-based)
│   └── Configure Physics (friction, collision)
│
├── ROS2 Interface:
│   ├── rclpy.init()
│   ├── Create Node: "isaac_sim_controller"
│   │
│   ├── Subscriptions:
│   │   ├── /joint_input (Float32MultiArray)
│   │   │   └── Callback: update_joint_positions()
│   │   │       • Store target positions for 10 DOF
│   │   │
│   │   └── /gripper_command (Int32)
│   │       └── Callback: gripper_command_callback()
│   │           • 0 → gripper.open()
│   │           • 1 → gripper.close()
│   │
│   └── Publications:
│       └── /joint_pos (Float32MultiArray)
│           • Current robot state (10 DOF)
│           • Published every simulation step
│
├── Robot Control:
│   ├── SingleManipulator (articulation)
│   │   ├── DOF: 10
│   │   ├── Joint names: [joint_1, ..., joint_6, rh_p12_rn, ...]
│   │   └── apply_action(ArticulationAction)
│   │
│   └── Control Modes:
│       └── Position Control (position tracking)
│
├── Surface Gripper:
│   ├── GripperView initialization
│   │   ├── USD Prim Path: .../SurfaceGripper
│   │   ├── Attachment Points: 4 finger links
│   │   └── Properties: force limits, grip distance
│   │
│   ├── Methods:
│   │   ├── close() - Close gripper, try to attach objects
│   │   ├── open() - Open gripper, release objects
│   │   └── update() - Update attachment state
│   │
│   └── Status Tracking:
│       ├── is_closed (bool)
│       └── Attached objects list
│
└── Simulation Loop (60Hz):
    1. rclpy.spin_once() - Process ROS2 messages
    2. Apply ArticulationAction (target positions)
    3. my_world.step() - Step physics
    4. gripper_view.update() - Update gripper state
    5. Publish current state (/joint_pos)
    6. Check gripper status (attachment)
    7. Handle objects physics
```

---

## 📊 데이터 흐름도

### 실시간 제어 흐름

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Real-time Control                           │
└─────────────────────────────────────────────────────────────────────┘

[GUI] User Interaction
   │
   ├─ Joint Control:
   │  └─> /joint_trajectory (trajectory_msgs/JointTrajectory)
   │      └─> [MoveIt2]
   │          └─> /joint_states (sensor_msgs/JointState) [6 DOF]
   │              └─> [joint_state_bridge.py]
   │                  └─> /joint_input (Float32MultiArray) [10 DOF]
   │                      └─> [Isaac Sim] Robot Control
   │
   ├─ Gripper Angle:
   │  └─> /gripper_input (std_msgs/Float32) [radians]
   │      └─> [joint_state_bridge.py]
   │          └─> /joint_input (Float32MultiArray) [10 DOF]
   │              └─> [Isaac Sim] Robot Control
   │
   └─ Gripper Open/Close:
      └─> /gripper_command (std_msgs/Int32) [0 or 1]
          └─> [Isaac Sim] Surface Gripper
              └─> gripper.open() or gripper.close()

[Isaac Sim] Feedback
   └─> /joint_pos (Float32MultiArray) [10 DOF]
       └─> [GUI] Display current state
```

### 시퀀스 재생 흐름

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Sequence Playback                            │
└─────────────────────────────────────────────────────────────────────┘

[GUI] Play Sequence Button
   │
   └─> Start playback thread
       │
       └─> For each item in sequence:
           │
           ├─ [JOINT] type:
           │  └─> Send /joint_trajectory
           │      └─> Wait for completion (duration seconds)
           │
           ├─ [GRIPPER] type:
           │  └─> Send /gripper_input (angle)
           │      └─> Wait for completion (duration seconds)
           │
           ├─ [GripperOpen] type:
           │  └─> Send /gripper_command (0)
           │      └─> Wait 0.5 seconds
           │
           ├─ [GripperClose] type:
           │  └─> Send /gripper_command (1)
           │      └─> Wait 0.5 seconds
           │
           ├─ [CartesianPose] type:
           │  └─> Request IK service (MoveIt2)
           │      └─> Get joint solution
           │          └─> Send /joint_trajectory
           │              └─> Wait for completion (duration seconds)
           │
           └─ [Sleep] type:
              └─> time.sleep(duration)
```

---

## 🔌 ROS2 토픽 맵

### 📤 Published Topics

| Topic | Message Type | Publisher | Description |
|-------|-------------|-----------|-------------|
| `/joint_pos` | `Float32MultiArray` | Isaac Sim | 현재 로봇 상태 (10 DOF) |
| `/joint_input` | `Float32MultiArray` | joint_state_bridge | 로봇 제어 명령 (10 DOF) |
| `/joint_trajectory` | `JointTrajectory` | GUI | Trajectory 명령 (6 DOF) |
| `/gripper_input` | `Float32` | GUI | 그리퍼 각도 (radians) |
| `/gripper_command` | `Int32` | GUI | 그리퍼 Open/Close (0/1) |

### 📥 Subscribed Topics

| Topic | Message Type | Subscriber | Description |
|-------|-------------|------------|-------------|
| `/joint_states` | `JointState` | joint_state_bridge, GUI | MoveIt2 joint 상태 (6 DOF) |
| `/joint_pos` | `Float32MultiArray` | GUI | Isaac Sim 피드백 (10 DOF) |
| `/joint_input` | `Float32MultiArray` | Isaac Sim | 로봇 제어 명령 |
| `/gripper_input` | `Float32` | joint_state_bridge | 그리퍼 각도 제어 |
| `/gripper_command` | `Int32` | Isaac Sim | 그리퍼 Open/Close |

### 🔄 Services

| Service | Service Type | Provider | Description |
|---------|-------------|----------|-------------|
| `/compute_ik` | `GetPositionIK` | MoveIt2 | Cartesian → Joint IK 계산 |
| `/compute_fk` | `GetPositionFK` | MoveIt2 | Joint → Cartesian FK 계산 |

---

## 📁 파일 구조

```
e0509_with_env_pick_place/
│
├── 🐍 Python Scripts
│   ├── gui_with_surface_gripper.py          # 메인 GUI (1900+ lines)
│   ├── isaacsim_e0509_surface_gripper_in_usd_pick_all.py  # Isaac Sim 컨트롤러
│   ├── joint_state_bridge.py                # ROS2 브릿지 (6DOF → 10DOF)
│   └── gui_config.py                        # GUI 설정 (치수, 색상)
│
├── 🌍 USD Assets
│   ├── isaac_env/
│   │   ├── room_without_e0509.usd          # 환경 (테이블, 물체)
│   │   └── e0509_model.usd                 # 로봇 모델 (그리퍼 포함)
│   │
│   └── SurfaceGripper USD Properties:
│       └── /World/e0509_model/e0509/gripper/gripper/SurfaceGripper
│           ├── max_grip_distance: 0.2
│           ├── coaxial_force_limit: 10000
│           ├── shear_force_limit: 10000
│           └── contactOffset: 0.02
│
├── 💾 Data Storage
│   ├── sequence/
│   │   ├── single_pose/                    # 개별 포즈 블록
│   │   │   ├── [JOINT] zero_pose.json
│   │   │   ├── [JOINT] cup_up.json
│   │   │   ├── [GRIPPER] cup_grip.json
│   │   │   └── [CARTESIAN] pick_position.json
│   │   │
│   │   └── library/                        # 전체 시퀀스
│   │       ├── pick_and_place_demo.json
│   │       └── calibration_routine.json
│   │
│   └── logs/                               # 실행 로그
│       └── isaac_sim_log_YYYYMMDD_HHMMSS.txt
│
└── 📄 Documentation
    ├── SYSTEM_ARCHITECTURE.md              # 이 파일
    ├── GRIPPER_INTEGRATION_SUMMARY.md
    ├── CARTESIAN_CONTROL_GUIDE.md
    └── ROS2_TOPIC_DIAGRAM_WITH_GRIPPER.md
```

---

## 🎮 주요 기능

### 1. **실시간 제어 모드**
- ✅ Enable/Disable 토글
- ✅ Joint 개별 제어 (슬라이더)
- ✅ Cartesian 제어 (XYZ + RPY)
- ✅ Gripper 각도 제어 (슬라이더)
- ✅ Gripper Open/Close (버튼)

### 2. **시퀀스 편집**
- ✅ 7가지 액션 타입:
  - 📸 Capture Pose (현재 포즈 캡처)
  - ➕ Joint (Joint 포즈 추가)
  - 🤏 Gripper (Gripper 포즈 추가)
  - 🟢 Gripper Open
  - 🔴 Gripper Close
  - 🗺️ Cartesian Pose (IK 사용)
  - ⏱️ Sleep (대기)
- ✅ 편집 기능 (모든 타입)
- ✅ 재정렬 (▲ ▼)
- ✅ 삭제 기능

### 3. **블록 관리**
- ✅ 시퀀스 → 블록 저장 (타입별 접두사)
  - `[JOINT] name.json`
  - `[GRIPPER] name.json`
  - `[CARTESIAN] name.json`
  - `[ACTION] name.json`
- ✅ 블록 → 시퀀스 추가
- ✅ 파일명에 타입 표시

### 4. **시퀀스 재생**
- ✅ 전체 시퀀스 재생
- ✅ Progress bar 표시
- ✅ Stop 기능
- ✅ 실시간 모드와 상호 배타적

### 5. **라이브러리 시스템**
- ✅ 시퀀스 저장/로드
- ✅ JSON 파일 형식
- ✅ 라이브러리 관리 (삭제)

---

## 🔧 Surface Gripper 설정

### USD 기반 설정
```python
# Gripper USD Prim Path
/World/e0509_model/e0509/gripper/gripper/SurfaceGripper

# Attachment Points (4 finger links)
attachment_points = [
    "rh_p12_rn_l1",  # Left finger 1
    "rh_p12_rn_l2",  # Left finger 2
    "rh_p12_rn_r1",  # Right finger 1
    "rh_p12_rn_r2",  # Right finger 2
]

# Properties
max_grip_distance = 0.2         # 20cm
coaxial_force_limit = 10000     # 10,000N
shear_force_limit = 10000       # 10,000N
contactOffset = 0.02            # 2cm
```

### 제어 방식
```python
# Close gripper (try to attach)
gripper_view.close()

# Open gripper (release)
gripper_view.open()

# Update state (every simulation step)
gripper_view.update()
```

---

## 📈 성능 특성

- **시뮬레이션 주기**: ~60Hz (Isaac Sim physics step)
- **ROS2 메시지 레이트**: ~60Hz
- **GUI 업데이트**: ~10Hz (QTimer)
- **Joint DOF**: 6 (로봇 팔)
- **Total DOF**: 10 (로봇 팔 6 + 그리퍼 관련 4)
- **Gripper 범위**: 0 ~ 63.1° (0 ~ 1.10 radians)

---

## 🔍 디버깅 정보

### 로그 위치
```bash
/home/woo/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env_pick_place/logs/
└── isaac_sim_log_YYYYMMDD_HHMMSS.txt
```

### 주요 로그 메시지
- `[GRIPPER STATUS]` - 그리퍼 상태 (open/closed, attached objects)
- `[ROS2]` - ROS2 메시지 수신/발신
- `[JOINT_INPUT]` - Joint 명령 수신
- `[GRIPPER_COMMAND]` - Gripper 명령 처리
- `[IK]` - IK 계산 결과

### ROS2 토픽 확인
```bash
# 모든 토픽 확인
ros2 topic list

# 특정 토픽 모니터링
ros2 topic echo /joint_pos
ros2 topic echo /gripper_command

# 토픽 정보
ros2 topic info /joint_input
```

---

## 🚀 시작 방법

### 1. Isaac Sim 실행
```bash
cd /home/woo/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env_pick_place
python3 isaacsim_e0509_surface_gripper_in_usd_pick_all.py
```

### 2. Joint State Bridge 실행 (선택적)
```bash
# MoveIt2 사용 시
python3 joint_state_bridge.py
```

### 3. GUI 실행
```bash
python3 gui_with_surface_gripper.py
```

---

## 📝 Notes

- **Surface Gripper**는 USD 파일에 미리 설정되어 있어야 함
- **Attachment points**는 finger links를 직접 참조 (collision meshes 아님)
- **Force limits**는 충분히 높게 설정 (10,000N 권장)
- **Sequence 재생 중**에는 실시간 제어 비활성화
- **파일 저장**은 타입별 접두사 자동 추가

---

**End of System Architecture Document**
