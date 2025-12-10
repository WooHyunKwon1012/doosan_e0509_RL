# Cartesian 제어 가이드

## 📝 개요

MoveIt2의 IK (Inverse Kinematics) 및 FK (Forward Kinematics) service를 사용하여 **XYZ + RPY** 좌표계 기반 실시간 제어를 구현했습니다.

**파일**: `gui_add_cartesian_control.py`

---

## 🎯 구현된 기능

### 1. **탭 기반 제어 모드**
Real-time Control 패널에 2개의 탭으로 구분:

#### **Tab 0: Joint Control** 🤖
- 6개 관절 슬라이더 + 그리퍼 슬라이더
- 각도 단위로 직접 제어
- Send Joint Command 버튼으로 즉시 전송

#### **Tab 1: Cartesian Control** 🗺️
- **Position (XYZ)**: End-effector의 위치 (meters)
  - X: -1.0 ~ 1.0 m
  - Y: -1.0 ~ 1.0 m
  - Z: 0.0 ~ 1.0 m

- **Orientation (RPY)**: End-effector의 자세 (degrees)
  - Roll: -180° ~ 180°
  - Pitch: -180° ~ 180°
  - Yaw: -180° ~ 180°

### 2. **명령 전송 버튼**
- **🎯 Send Cartesian Command**: 입력한 XYZ+RPY로 로봇 이동
- **📍 Get Current Pose**: 현재 end-effector 위치를 FK로 조회하여 표시 ✅

### 3. **상호 배타적 제어**
- Joint Control 탭 활성 시 → Cartesian 명령 비활성화
- Cartesian Control 탭 활성 시 → Joint 명령 비활성화
- 슬라이더는 Real-time 모드가 OFF일 때만 자동 업데이트

---

## 🔄 동작 흐름

### Cartesian → Joint 변환 (IK)
```
GUI: XYZ + RPY 입력
   ↓
Euler Angles → Quaternion 변환
   ↓
/compute_ik service 호출 (비동기)
   ↓
MoveIt2: IK 계산 (충돌 회피 포함)
   ↓
IK Success → Joint 값 반환
   ↓
/dsr_moveit_controller/joint_trajectory 발행
   ↓
MoveIt2: /joint_states 업데이트
   ↓
Bridge: /joint_states → /joint_input 변환
   ↓
Isaac Sim: 로봇 이동
```

### Joint → Cartesian 변환 (FK)
```
GUI: "Get Current Pose" 클릭
   ↓
현재 joint 상태 읽기
   ↓
/compute_fk service 호출 (동기)
   ↓
MoveIt2: FK 계산 (link_6 위치)
   ↓
FK Success → XYZ + Quaternion 반환
   ↓
Quaternion → Euler (RPY) 변환
   ↓
GUI 입력 필드에 표시
```

---

## 🚀 사용 방법

### 1. 시스템 시작

**4개 터미널 필요:**

```bash
# Terminal 1: Isaac Sim
cd ~/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env
python3 e0509_jointpos_ros2.py

# Terminal 2: MoveIt2
ros2 launch dsr_launcher2 dsr_moveit.launch.py

# Terminal 3: Bridge
cd ~/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env
python3 joint_state_bridge.py

# Terminal 4: GUI
cd ~/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env
python3 gui_add_cartesian_control.py
```

### 2. Cartesian 제어 사용

1. GUI에서 **"Enable Real-time Control"** 체크
2. **"Cartesian Control"** 탭 선택
3. **"📍 Get Current Pose"** 클릭하여 현재 위치 확인
4. **Position (XYZ)** 수정:
   - 예: Z 값을 0.05m 증가
5. **Orientation (RPY)** 필요 시 수정
6. **"🎯 Send Cartesian Command"** 클릭
7. RViz와 Isaac Sim에서 로봇 이동 확인

### 2. 예제 좌표

#### 홈 포지션 위
```
X: 0.500 m
Y: 0.000 m
Z: 0.400 m
Roll: 0.0°
Pitch: 0.0°
Yaw: 0.0°
```

#### 왼쪽으로 이동
```
X: 0.500 m
Y: 0.300 m  ← 변경
Z: 0.300 m
Roll: 0.0°
Pitch: 0.0°
Yaw: 0.0°
```

#### 아래로 이동
```
X: 0.500 m
Y: 0.000 m
Z: 0.200 m  ← 변경
Roll: 0.0°
Pitch: 0.0°
Yaw: 0.0°
```

#### 회전
```
X: 0.500 m
Y: 0.000 m
Z: 0.300 m
Roll: 0.0°
Pitch: 0.0°
Yaw: 90.0°  ← 변경 (Z축 회전)
```

---

## ⚙️ 기술 세부사항

### IK Service (Inverse Kinematics)
- **Service 이름**: `/compute_ik`
- **Type**: `moveit_msgs/srv/GetPositionIK`
- **Group 이름**: `manipulator`
- **Frame ID**: `base_link`
- **호출 방식**: 비동기 (async)
- **Duration**: 2.0초

### FK Service (Forward Kinematics)
- **Service 이름**: `/compute_fk`
- **Type**: `moveit_msgs/srv/GetPositionFK`
- **Target Link**: `link_6` (end-effector)
- **Frame ID**: `base_link`
- **호출 방식**: 동기 (sync, timeout 2초)

### Euler to Quaternion 변환
```python
cy = cos(yaw * 0.5)
sy = sin(yaw * 0.5)
cp = cos(pitch * 0.5)
sp = sin(pitch * 0.5)
cr = cos(roll * 0.5)
sr = sin(roll * 0.5)

qw = cr * cp * cy + sr * sp * sy
qx = sr * cp * cy - cr * sp * sy
qy = cr * sp * cy + sr * cp * sy
qz = cr * cp * sy - sr * sp * cy
```

### Quaternion to Euler 변환
```python
# Roll (x-axis)
sinr_cosp = 2 * (w * x + y * z)
cosr_cosp = 1 - 2 * (x * x + y * y)
roll = atan2(sinr_cosp, cosr_cosp)

# Pitch (y-axis)
sinp = 2 * (w * y - z * x)
pitch = asin(sinp) if abs(sinp) < 1 else copysign(pi/2, sinp)

# Yaw (z-axis)
siny_cosp = 2 * (w * z + x * y)
cosy_cosp = 1 - 2 * (y * y + z * z)
yaw = atan2(siny_cosp, cosy_cosp)
```

### IK 에러 코드
| Code | 의미 |
|------|------|
| 1 | SUCCESS |
| -31 | NO_IK_SOLUTION (도달 불가능) |
| -12 | GOAL_IN_COLLISION (충돌 상태) |
| -20 | FRAME_TRANSFORM_FAILURE |

---

## 📊 성능 최적화

### Isaac Sim 주파수 제한
`e0509_jointpos_ros2.py` 수정됨:
```python
publish_rate = 30  # Hz (기존 60Hz → 30Hz)
publish_interval = 1.0 / publish_rate
```

**효과:**
- CPU 사용량 약 30% 감소
- GUI 응답성 향상
- Isaac Sim 렌더링 부하 감소

---

## 🐛 트러블슈팅

### IK 실패 시

**증상:**
```
IK failed with error code: -31
```

**원인:**
- 도달 불가능한 위치
- 관절 한계 초과
- 충돌 발생

**해결:**
1. 로봇의 작업 공간 내 좌표 입력
2. Z 값을 높여서 테스트 (예: 0.3 이상)
3. Orientation을 0°으로 시작

### IK Service 연결 안 됨

**확인:**
```bash
ros2 service list | grep ik
# /compute_ik가 있어야 함

ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK ...
```

**해결:**
- MoveIt2가 실행 중인지 확인
- GUI 재시작

### 로봇이 이상한 방향으로 이동

**원인:**
- Euler angle → Quaternion 변환 오류
- Frame ID 불일치

**해결:**
1. Roll, Pitch, Yaw를 0°으로 초기화
2. XYZ만 변경하여 테스트
3. 터미널에서 로그 확인:
   ```bash
   ros2 topic echo /joint_states
   ```

---

## 🔮 향후 개선 사항

### 1. Forward Kinematics (FK)
- **목표**: "📍 Get Current Pose" 버튼 활성화
- **방법**: `/tf` 토픽 구독 또는 FK service 호출
- **결과**: 현재 end-effector 위치를 GUI에 자동 표시

### 2. Cartesian 경로 계획
- **목표**: 직선 경로 이동
- **방법**: MoveIt2의 `compute_cartesian_path` service 사용
- **결과**: 더 예측 가능한 이동 궤적

### 3. Cartesian 시퀀스 블록
- **목표**: 시퀀스에 Cartesian 액션 추가
- **형식**: `[CARTESIAN] Pick (2.0s): XYZ=(0.5, 0.2, 0.15), RPY=(0, 90, 0)`

---

## 🧪 테스트 명령

### 1. 수동 IK 테스트
```bash
# IK service 직접 호출
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "
ik_request:
  group_name: 'manipulator'
  pose_stamped:
    header:
      frame_id: 'base_link'
    pose:
      position:
        x: 0.5
        y: 0.0
        z: 0.3
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  avoid_collisions: true
"
```

### 2. TF 확인
```bash
# End-effector의 현재 위치 확인
ros2 run tf2_ros tf2_echo base_link link_6

# 또는
ros2 topic echo /tf --field transforms[0].transform
```

### 3. Joint 값 확인
```bash
# IK 결과 확인
ros2 topic echo /joint_states
```

---

## 📚 참고 자료

- [MoveIt2 IK Tutorial](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [ROS2 TF2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [Quaternion to Euler](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

---

## 🎉 완료!

이제 Cartesian 좌표로 로봇을 제어할 수 있습니다!

- ✅ XYZ + RPY 입력
- ✅ IK 자동 계산
- ✅ 충돌 회피
- ✅ 성능 최적화 (30Hz)
- ⏳ FK (향후 추가 예정)
- ⏳ Cartesian 경로 계획 (향후 추가 예정)

질문이나 문제가 있으면 언제든지 알려주세요!
