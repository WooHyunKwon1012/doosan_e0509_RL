# E0509 Gripper Control with Isaac Sim

## 개요

Isaac Sim에서 Doosan e0509 로봇과 그리퍼를 ROS2로 제어하는 스크립트입니다.

## ROS2 토픽

### 구독 (Subscriptions)

1. **`/joint_input`** (std_msgs/Float32MultiArray)
   - 로봇 팔 6개 관절 위치 명령 (라디안)
   - **8번째 값(인덱스 7)**: 그리퍼 제어 (라디안, 0~1.101 rad = 0~63.1도)
   - 예: `[0.0, -0.5, 0.5, 0.0, 0.5, 0.0, 0.0, 0.5]` (마지막 0.5는 그리퍼를 약 28.6도 열기)

2. **`/gripper_input`** (std_msgs/Float32)
   - 그리퍼 위치 명령
   - 범위: **0.0 (완전 닫힘) ~ 63.1 (완전 열림)**
   - Joint path: `/World/e0509_model/e0509/gripper/gripper/joints/rh_r1_joint`

### 발행 (Publications)

1. **`/joint_pos`** (std_msgs/Float32MultiArray)
   - 현재 로봇 팔 관절 위치 (라디안)

2. **`/gripper_pos`** (std_msgs/Float32)
   - 현재 그리퍼 위치 (0.0 ~ 63.1)

## 사용 방법

### 1. Isaac Sim에서 실행
```bash
# Isaac Sim 환경에서
cd ~/ros2_ws/src/doosan-robot2/isaacsim_connect
python3 e0509_jointpos_ros2.py
```

### 2. 그리퍼 제어 예제

**완전 열기 (Open):**
```bash
ros2 topic pub --once /gripper_input std_msgs/Float32 "data: 63.1"
```

**완전 닫기 (Close):**
```bash
ros2 topic pub --once /gripper_input std_msgs/Float32 "data: 0.0"
```

**중간 위치:**
```bash
ros2 topic pub --once /gripper_input std_msgs/Float32 "data: 30.0"
```

### 3. 로봇 팔 + 그리퍼 통합 제어 예제

**8번째 값(인덱스 7)으로 그리퍼 제어:**
```bash
# 로봇 팔 이동 + 그리퍼 완전 열기 (1.101 rad = 63.1도)
ros2 topic pub --once /joint_input std_msgs/Float32MultiArray \
  "data: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0, 0.0, 1.101]"

# 로봇 팔 이동 + 그리퍼 반만 열기 (0.55 rad ≈ 31.5도)
ros2 topic pub --once /joint_input std_msgs/Float32MultiArray \
  "data: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0, 0.0, 0.55]"

# 로봇 팔 이동 + 그리퍼 닫기 (0.0 rad = 0도)
ros2 topic pub --once /joint_input std_msgs/Float32MultiArray \
  "data: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0]"
```

**로봇 팔만 제어 (기존 방식):**
```bash
ros2 topic pub --once /joint_input std_msgs/Float32MultiArray \
  "data: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]"
```

### 4. Python에서 그리퍼 제어

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

rclpy.init()
node = Node('gripper_test')
pub = node.create_publisher(Float32, '/gripper_input', 10)

# 그리퍼 열기
msg = Float32()
msg.data = 63.1
pub.publish(msg)

# 2초 대기
import time
time.sleep(2)

# 그리퍼 닫기
msg.data = 0.0
pub.publish(msg)

rclpy.shutdown()
```

### 5. GUI에서 그리퍼 통합 제어

`gui_moveit.py`에 그리퍼 제어를 추가하려면:

```python
from std_msgs.msg import Float32

class JointStateMonitor(Node):
    def __init__(self, update_signal):
        super().__init__('joint_pose_gui')
        # ... 기존 코드 ...
        
        # 그리퍼 publisher 추가
        self.gripper_pub = self.create_publisher(Float32, '/gripper_input', 10)
    
    def send_gripper_command(self, value):
        """그리퍼 명령 전송 (0.0 ~ 63.1)"""
        msg = Float32()
        msg.data = float(value)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper command: {value}')
```

## 그리퍼 상세 정보

### Joint 구조
- **Master Joint**: `/World/e0509_model/e0509/gripper/gripper/joints/rh_r1_joint`
- **Mimic Joints**: 나머지 그리퍼 조인트들은 자동으로 따라감

### 값 범위
- **최소값 (닫힘)**: 0.0
- **최대값 (열림)**: 63.1
- **단위**: 도(degrees) 또는 그리퍼 고유 단위

### 제어 방식
- USD Stage의 `UsdPhysics.RevoluteJoint`를 통해 직접 제어
- Position drive 사용 (stiffness=1e7, damping=1e7)

## 트러블슈팅

### 그리퍼가 움직이지 않는 경우

1. **Joint path 확인**
   ```bash
   # Isaac Sim에서 USD 경로 확인
   /World/e0509_model/e0509/gripper/gripper/joints/rh_r1_joint
   ```

2. **토픽 구독 확인**
   ```bash
   ros2 topic echo /gripper_pos
   ```

3. **범위 확인**
   - 0.0 ~ 63.1 범위 내 값 사용
   - 로그에서 clamping 경고 확인

### 그리퍼 위치가 튀는 경우

- Drive 파라미터 조정:
  ```python
  drive.GetDampingAttr().Set(1e6)    # 감소
  drive.GetStiffnessAttr().Set(1e6)  # 감소
  ```

## 통합 예제

로봇 팔과 그리퍼를 동시에 제어:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

rclpy.init()
node = Node('robot_gripper_test')

joint_pub = node.create_publisher(Float32MultiArray, '/joint_input', 10)
gripper_pub = node.create_publisher(Float32, '/gripper_input', 10)

# 로봇 팔 이동
joint_msg = Float32MultiArray()
joint_msg.data = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
joint_pub.publish(joint_msg)

import time
time.sleep(2)

# 그리퍼 닫기
gripper_msg = Float32()
gripper_msg.data = 0.0
gripper_pub.publish(gripper_msg)

rclpy.shutdown()
```

## 추가 정보

- Mimic joints는 자동으로 master joint를 따라가므로 별도 제어 불필요
- 그리퍼 제어는 실시간으로 반영됨
- GUI 통합 시 슬라이더나 버튼으로 쉽게 제어 가능
