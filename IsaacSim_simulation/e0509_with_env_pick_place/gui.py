#!/usr/bin/env python3

import sys
import json
import math
import threading
import time
import os
from pathlib import Path
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QLabel, QDoubleSpinBox, QLineEdit,
    QFileDialog, QMessageBox, QCheckBox, QProgressBar, QDialog,
    QDialogButtonBox, QGridLayout, QSlider, QGroupBox, QRadioButton,
    QInputDialog, QSplitter, QListWidgetItem, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState
import tf2_ros
from tf2_ros import TransformException

# GUI 설정 파일 import
from gui_config import *

# 저장 경로 설정
SCRIPT_DIR = Path(__file__).parent
LIBRARY_DIR = SCRIPT_DIR / 'sequence' / 'library'
SINGLE_POSE_DIR = SCRIPT_DIR / 'sequence' / 'single_pose'

# 디렉토리 생성
LIBRARY_DIR.mkdir(parents=True, exist_ok=True)
SINGLE_POSE_DIR.mkdir(parents=True, exist_ok=True)


class ROSThread(QObject):
    """ROS2를 별도 스레드에서 실행"""
    joint_state_updated = pyqtSignal(list)
    joint_pos_updated = pyqtSignal(list)  # /joint_pos 업데이트
    ik_error = pyqtSignal(str)  # IK 에러 메시지
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.running = False
        
    def start(self):
        self.running = True
        rclpy.init()
        self.node = JointStateMonitor(self.joint_state_updated, self.joint_pos_updated, self.ik_error)
        
        # 별도 스레드에서 spin
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        
    def _spin(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
    def stop(self):
        self.running = False
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()


class JointStateMonitor(Node):
    """ROS2 노드: /joint_states, /joint_pos 구독 및 명령 발행"""
    def __init__(self, joint_state_signal, joint_pos_signal, ik_error_signal):
        super().__init__('joint_pose_gui')
        self.joint_state_signal = joint_state_signal
        self.joint_pos_signal = joint_pos_signal
        self.ik_error_signal = ik_error_signal
        self.current_joints = [0.0] * 6
        self.current_gripper = 0.0
        self.target_gripper = 0.0
        self.gripper_control_active = False
        self.gripper_speed = 0.05  # 라디안/틱 (약 2.87도/틱)
        
        # /joint_states 구독 (MoveIt2)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # /joint_pos 구독 (Isaac Sim 피드백)
        self.joint_pos_sub = self.create_subscription(
            Float32MultiArray,
            '/joint_pos',
            self.joint_pos_callback,
            10
        )
        
        # MoveIt trajectory 발행
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/dsr_moveit_controller/joint_trajectory',
            10
        )
        
        # 그리퍼 제어 발행
        self.gripper_pub = self.create_publisher(
            Float32,
            '/gripper_input',
            10
        )
        
        # 그리퍼 제어를 위한 타이머 (10Hz)
        self.gripper_timer = self.create_timer(0.1, self.gripper_control_loop)
        
        # IK service 클라이언트
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # FK service 클라이언트
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # TF2 리스너 설정 (사용 안 함 - FK service 사용)
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Joint State Monitor started')
        self.get_logger().info('Waiting for IK service...')
        if self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('IK service available!')
        else:
            self.get_logger().warn('IK service not available')
        
        self.get_logger().info('Waiting for FK service...')
        if self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('FK service available!')
        else:
            self.get_logger().warn('FK service not available')
        
    def joint_state_callback(self, msg: JointState):
        """MoveIt2에서 발행하는 /joint_states 구독"""
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joints = []
        
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joints.append(msg.position[idx])
            else:
                joints.append(0.0)
        
        self.current_joints = joints
        self.joint_state_signal.emit(joints)
    
    def joint_pos_callback(self, msg: Float32MultiArray):
        """Isaac Sim에서 발행하는 /joint_pos 구독"""
        if len(msg.data) >= 8:
            joints = list(msg.data[:6])
            gripper_rad = msg.data[7]  # index 7이 그리퍼 (라디안)
            
            # 현재 그리퍼 값 업데이트
            self.current_gripper = gripper_rad
            
            # GUI로 전달 (joints + gripper)
            full_state = joints + [gripper_rad]
            self.joint_pos_signal.emit(full_state)
    
    def gripper_control_loop(self):
        """그리퍼를 목표값으로 점진적으로 이동"""
        if not self.gripper_control_active:
            return
        
        error = self.target_gripper - self.current_gripper
        
        if abs(error) < 0.001:  # 목표 도달
            # 목표 도달 후에도 계속 발행 (유지)
            msg = Float32()
            msg.data = self.target_gripper
            self.gripper_pub.publish(msg)
            return
        
        # 점진적으로 이동
        if error > 0:
            next_value = self.current_gripper + min(self.gripper_speed, error)
        else:
            next_value = self.current_gripper + max(-self.gripper_speed, error)
        
        msg = Float32()
        msg.data = next_value
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper: {next_value:.4f} rad (target: {self.target_gripper:.4f})')
    
    def set_gripper_target(self, value_rad, activate=True):
        """그리퍼 목표값 설정"""
        self.target_gripper = value_rad
        self.gripper_control_active = activate
        self.get_logger().info(f'Gripper target set: {value_rad:.4f} rad')
    
    def stop_gripper_control(self):
        """그리퍼 제어 중지"""
        self.gripper_control_active = False
        
    def send_trajectory(self, poses, durations):
        """여러 포즈를 순차적으로 실행"""
        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        time_offset = 0.0
        for pose, duration in zip(poses, durations):
            point = JointTrajectoryPoint()
            point.positions = pose
            time_offset += duration
            point.time_from_start = Duration(sec=int(time_offset), nanosec=int((time_offset % 1) * 1e9))
            traj.points.append(point)
        
        self.traj_pub.publish(traj)
        self.get_logger().info(f'Published trajectory with {len(poses)} poses')
    
    def send_cartesian_command(self, x, y, z, roll, pitch, yaw, duration=0.5):
        """Cartesian 좌표로 명령 (XYZ + RPY)"""
        # Euler angles to Quaternion 변환
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        # IK 요청 생성
        ik_request = PositionIKRequest()
        ik_request.group_name = 'manipulator'
        ik_request.avoid_collisions = True
        ik_request.timeout = Duration(sec=1, nanosec=0)
        
        # 목표 포즈 설정
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = Point(x=x, y=y, z=z)
        pose_stamped.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        ik_request.pose_stamped = pose_stamped
        
        # 현재 상태를 시작 상태로 설정
        robot_state = RobotState()
        robot_state.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        robot_state.joint_state.position = self.current_joints
        ik_request.robot_state = robot_state
        
        # IK service 호출 (비동기)
        request = GetPositionIK.Request()
        request.ik_request = ik_request
        
        future = self.ik_client.call_async(request)
        future.add_done_callback(lambda f: self._ik_response_callback(f, duration))
        
        self.get_logger().info(f'Sent IK request: xyz=({x:.3f}, {y:.3f}, {z:.3f}), rpy=({math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, {math.degrees(yaw):.1f})')
    
    def _ik_response_callback(self, future, duration):
        """IK 응답 처리"""
        try:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                # IK 성공 - joint 값 추출
                joint_positions = list(response.solution.joint_state.position[:6])
                self.get_logger().info(f'IK success! Sending trajectory...')
                self.send_trajectory([joint_positions], [duration])
            else:
                # IK 실패 - 에러 코드 해석
                error_codes = {
                    -1: "FAILURE",
                    -2: "PLANNING_FAILED",
                    -3: "INVALID_MOTION_PLAN",
                    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                    -5: "CONTROL_FAILED",
                    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                    -7: "TIMED_OUT",
                    -8: "PREEMPTED",
                    -10: "START_STATE_IN_COLLISION",
                    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                    -12: "GOAL_IN_COLLISION",
                    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                    -14: "GOAL_CONSTRAINTS_VIOLATED",
                    -15: "INVALID_GROUP_NAME",
                    -16: "INVALID_GOAL_CONSTRAINTS",
                    -17: "INVALID_ROBOT_STATE",
                    -18: "INVALID_LINK_NAME",
                    -19: "INVALID_OBJECT_NAME",
                    -20: "FRAME_TRANSFORM_FAILURE",
                    -21: "COLLISION_CHECKING_UNAVAILABLE",
                    -22: "ROBOT_STATE_STALE",
                    -23: "SENSOR_INFO_STALE",
                    -31: "NO_IK_SOLUTION"
                }
                error_msg = error_codes.get(response.error_code.val, f"UNKNOWN ({response.error_code.val})")
                self.get_logger().error(f'IK failed: {error_msg} (code: {response.error_code.val})')
                self.get_logger().warn('Try adjusting the target pose - it may be out of reach or in collision')
                
                # GUI로 에러 메시지 전달
                self.ik_error_signal.emit(f'IK Failed: {error_msg}\nThe target pose may be out of reach or in collision.')
        except Exception as e:
            self.get_logger().error(f'IK service call failed: {str(e)}')
            self.ik_error_signal.emit(f'IK Service Error: {str(e)}')
    
    def get_current_ee_pose(self):
        """FK 서비스를 통해 현재 end-effector 위치 가져오기"""
        try:
            # FK 요청 생성
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link'
            request.header.stamp = self.get_clock().now().to_msg()
            request.fk_link_names = ['link_6']  # end-effector link (언더스코어 사용)
            
            # 현재 joint 상태 설정
            request.robot_state.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            request.robot_state.joint_state.position = self.current_joints
            
            # FK 서비스 동기 호출
            future = self.fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.error_code.val == 1:  # SUCCESS
                    pose = response.pose_stamped[0].pose
                    
                    # Position
                    x = pose.position.x
                    y = pose.position.y
                    z = pose.position.z
                    
                    # Orientation (Quaternion -> Euler)
                    qx = pose.orientation.x
                    qy = pose.orientation.y
                    qz = pose.orientation.z
                    qw = pose.orientation.w
                    
                    # Quaternion to Euler (RPY)
                    roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
                    
                    self.get_logger().info(f'FK success: xyz=({x:.3f}, {y:.3f}, {z:.3f}), rpy=({math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, {math.degrees(yaw):.1f})')
                    return (x, y, z, roll, pitch, yaw)
                else:
                    self.get_logger().error(f'FK failed with error code: {response.error_code.val}')
                    return None
            else:
                self.get_logger().error('FK service call timed out')
                return None
                
        except Exception as ex:
            self.get_logger().error(f'FK service call failed: {ex}')
            return None
    
    def quaternion_to_euler(self, x, y, z, w):
        """Quaternion을 Euler 각도(roll, pitch, yaw)로 변환"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


class EditPoseDialog(QDialog):
    """포즈 편집 다이얼로그"""
    def __init__(self, pose, name, duration, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Edit Pose')
        self.pose = list(pose)
        
        layout = QVBoxLayout()
        
        # 이름 입력
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel('Name:'))
        self.name_edit = QLineEdit(name)
        name_layout.addWidget(self.name_edit)
        layout.addLayout(name_layout)
        
        # Duration 입력
        dur_layout = QHBoxLayout()
        dur_layout.addWidget(QLabel('Duration (s):'))
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(DURATION_MIN, DURATION_MAX)
        self.duration_spin.setSingleStep(DURATION_STEP)
        self.duration_spin.setValue(duration)
        dur_layout.addWidget(self.duration_spin)
        layout.addLayout(dur_layout)
        
        # Joint 값 입력 (각도)
        grid = QGridLayout()
        self.joint_spins = []
        for i in range(JOINT_COUNT):
            grid.addWidget(QLabel(f'Joint {i+1} (°):'), i, 0)
            spin = QDoubleSpinBox()
            spin.setRange(JOINT_ANGLE_MIN, JOINT_ANGLE_MAX)
            spin.setSingleStep(JOINT_ANGLE_STEP)
            spin.setDecimals(JOINT_ANGLE_DECIMALS)
            spin.setValue(self.pose[i] * 180.0 / math.pi)
            spin.setSuffix(' °')
            self.joint_spins.append(spin)
            grid.addWidget(spin, i, 1)
        
        layout.addLayout(grid)
        
        # 버튼
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        
        self.setLayout(layout)
        
    def get_values(self):
        """편집된 값 반환 (각도를 라디안으로 변환)"""
        pose = [spin.value() * math.pi / 180.0 for spin in self.joint_spins]
        name = self.name_edit.text()
        duration = self.duration_spin.value()
        return pose, name, duration


class EditGripperDialog(QDialog):
    """그리퍼 액션 편집 다이얼로그"""
    def __init__(self, name, gripper_deg, duration, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Edit Gripper Action')
        
        layout = QVBoxLayout()
        
        # 이름 입력
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel('Name:'))
        self.name_edit = QLineEdit(name)
        name_layout.addWidget(self.name_edit)
        layout.addLayout(name_layout)
        
        # 그리퍼 값 입력 (각도)
        gripper_layout = QHBoxLayout()
        gripper_layout.addWidget(QLabel('Gripper (°):'))
        self.gripper_spin = QDoubleSpinBox()
        self.gripper_spin.setRange(0.0, 63.1)
        self.gripper_spin.setSingleStep(1.0)
        self.gripper_spin.setDecimals(1)
        self.gripper_spin.setValue(gripper_deg)
        self.gripper_spin.setSuffix(' °')
        gripper_layout.addWidget(self.gripper_spin)
        layout.addLayout(gripper_layout)
        
        # Duration 입력
        dur_layout = QHBoxLayout()
        dur_layout.addWidget(QLabel('Duration (s):'))
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.1, 10.0)
        self.duration_spin.setSingleStep(0.1)
        self.duration_spin.setValue(duration)
        self.duration_spin.setSuffix(' s')
        dur_layout.addWidget(self.duration_spin)
        layout.addLayout(dur_layout)
        
        # 버튼
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        
        self.setLayout(layout)
        
    def get_values(self):
        """편집된 값 반환"""
        name = self.name_edit.text()
        gripper_deg = self.gripper_spin.value()
        gripper_rad = math.radians(gripper_deg)  # 도 → 라디안 변환
        duration = self.duration_spin.value()
        return name, gripper_rad, duration


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(WINDOW_TITLE)
        self.setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT)
        
        # 디렉토리 생성
        LIBRARY_DIR.mkdir(parents=True, exist_ok=True)
        SINGLE_POSE_DIR.mkdir(parents=True, exist_ok=True)
        
        # 데이터
        self.sequence = []  # [(type, data), ...] type='joint' or 'gripper'
        self.single_poses = {}  # {filename: (type, data)} 개별 블록 저장용
        self.library_sequences = {}  # {filename: [sequence]} 라이브러리 저장용
        self.current_joints = [0.0] * 6
        self.current_gripper_rad = 0.0  # 현재 그리퍼 (라디안)
        self.is_playing = False
        self.play_thread = None
        self.realtime_mode = False  # 실시간 제어 모드
        self.joint_control_mode = True  # True: Joint, False: Cartesian
        
        # ROS 스레드
        self.ros_thread = ROSThread()
        self.ros_thread.joint_state_updated.connect(self.update_joint_states)
        self.ros_thread.joint_pos_updated.connect(self.update_joint_pos)
        self.ros_thread.ik_error.connect(self.show_ik_error)
        self.ros_thread.start()
        
        self.init_ui()
        
        # 저장된 파일 자동 로드
        self.load_all_single_poses()
        self.load_all_libraries()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # === 현재 상태 모니터링 패널 (/joint_pos 구독) ===
        monitor_group = QGroupBox('📊 Current State Monitor (/joint_pos)')
        monitor_layout = QVBoxLayout()
        
        # Joint 상태
        joint_monitor_layout = QHBoxLayout()
        self.joint_monitor_labels = []
        for i in range(6):
            label = QLabel(f'J{i+1}: 0.00°')
            label.setStyleSheet('font-family: monospace;')
            self.joint_monitor_labels.append(label)
            joint_monitor_layout.addWidget(label)
        monitor_layout.addLayout(joint_monitor_layout)
        
        # 그리퍼 상태
        self.gripper_monitor_label = QLabel('Gripper: 0.00° (0.0000 rad)')
        self.gripper_monitor_label.setStyleSheet('font-family: monospace; font-weight: bold;')
        monitor_layout.addWidget(self.gripper_monitor_label)
        
        monitor_group.setLayout(monitor_layout)
        main_layout.addWidget(monitor_group)
        
        # === 상태 표시 (모니터링 패널 바로 밑) ===
        self.status_label = QLabel('Ready')
        self.status_label.setStyleSheet('padding: 8px; background: #f0f0f0; font-weight: bold;')
        main_layout.addWidget(self.status_label)
        
        # === 실시간 제어 패널 (탭으로 구성) ===
        realtime_group = QGroupBox('🎮 Real-time Control')
        realtime_layout = QVBoxLayout()
        
        # 모드 전환 버튼
        mode_layout = QHBoxLayout()
        self.realtime_toggle = QCheckBox('Enable Real-time Control')
        self.realtime_toggle.stateChanged.connect(self.toggle_realtime_mode)
        mode_layout.addWidget(self.realtime_toggle)
        mode_layout.addStretch()
        realtime_layout.addLayout(mode_layout)
        
        # 탭 위젯 생성
        self.control_tabs = QTabWidget()
        self.control_tabs.setEnabled(False)  # 초기에는 비활성화
        self.control_tabs.currentChanged.connect(self.on_control_tab_changed)
        
        # === Joint Control 탭 ===
        joint_tab = QWidget()
        joint_tab_layout = QVBoxLayout()
        
        # Joint 슬라이더
        self.joint_sliders = []
        self.joint_slider_labels = []
        for i in range(6):
            h_layout = QHBoxLayout()
            label = QLabel(f'J{i+1}:')
            label.setFixedWidth(30)
            h_layout.addWidget(label)
            
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.valueChanged.connect(lambda v, idx=i: self.update_joint_slider_label(idx, v))
            self.joint_sliders.append(slider)
            h_layout.addWidget(slider)
            
            value_label = QLabel('0°')
            value_label.setFixedWidth(50)
            self.joint_slider_labels.append(value_label)
            h_layout.addWidget(value_label)
            
            joint_tab_layout.addLayout(h_layout)
        
        # 그리퍼 슬라이더
        gripper_layout = QHBoxLayout()
        gripper_layout.addWidget(QLabel('Gripper:'))
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 631)  # 0~63.1도 * 10
        self.gripper_slider.setValue(0)
        self.gripper_slider.valueChanged.connect(self.update_gripper_slider_label)
        gripper_layout.addWidget(self.gripper_slider)
        
        self.gripper_slider_label = QLabel('0.0°')
        self.gripper_slider_label.setFixedWidth(50)
        gripper_layout.addWidget(self.gripper_slider_label)
        joint_tab_layout.addLayout(gripper_layout)
        
        # Joint Control 버튼들
        joint_btn_layout = QHBoxLayout()
        
        joint_send_btn = QPushButton('Send Joint Command')
        joint_send_btn.clicked.connect(self.send_realtime_joint)
        self.joint_send_btn = joint_send_btn
        joint_btn_layout.addWidget(joint_send_btn)
        
        get_joint_state_btn = QPushButton('📍 Get Current Joint State')
        get_joint_state_btn.clicked.connect(self.get_current_joint_state)
        joint_btn_layout.addWidget(get_joint_state_btn)
        
        joint_tab_layout.addLayout(joint_btn_layout)
        
        # Gripper Send 버튼
        gripper_send_btn = QPushButton('Send Gripper Command')
        gripper_send_btn.clicked.connect(self.send_realtime_gripper)
        self.gripper_send_btn = gripper_send_btn
        joint_tab_layout.addWidget(gripper_send_btn)
        
        joint_tab.setLayout(joint_tab_layout)
        
        # === Cartesian Control 탭 ===
        cartesian_tab = QWidget()
        cartesian_tab_layout = QVBoxLayout()
        
        # Position (XYZ)
        pos_grid = QGridLayout()
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-1.0, 1.0)
        self.x_spin.setSingleStep(0.01)
        self.x_spin.setDecimals(3)
        self.x_spin.setValue(0.500)
        self.x_spin.setSuffix(' m')
        pos_grid.addWidget(QLabel('X:'), 0, 0)
        pos_grid.addWidget(self.x_spin, 0, 1)
        
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-1.0, 1.0)
        self.y_spin.setSingleStep(0.01)
        self.y_spin.setDecimals(3)
        self.y_spin.setValue(0.000)
        self.y_spin.setSuffix(' m')
        pos_grid.addWidget(QLabel('Y:'), 0, 2)
        pos_grid.addWidget(self.y_spin, 0, 3)
        
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(0.0, 1.0)
        self.z_spin.setSingleStep(0.01)
        self.z_spin.setDecimals(3)
        self.z_spin.setValue(0.300)
        self.z_spin.setSuffix(' m')
        pos_grid.addWidget(QLabel('Z:'), 0, 4)
        pos_grid.addWidget(self.z_spin, 0, 5)
        
        cartesian_tab_layout.addLayout(pos_grid)
        
        # Orientation (RPY)
        ori_grid = QGridLayout()
        self.roll_spin = QDoubleSpinBox()
        self.roll_spin.setRange(-180.0, 180.0)
        self.roll_spin.setSingleStep(5.0)
        self.roll_spin.setDecimals(1)
        self.roll_spin.setValue(0.0)
        self.roll_spin.setSuffix(' °')
        ori_grid.addWidget(QLabel('Roll:'), 0, 0)
        ori_grid.addWidget(self.roll_spin, 0, 1)
        
        self.pitch_spin = QDoubleSpinBox()
        self.pitch_spin.setRange(-180.0, 180.0)
        self.pitch_spin.setSingleStep(5.0)
        self.pitch_spin.setDecimals(1)
        self.pitch_spin.setValue(0.0)
        self.pitch_spin.setSuffix(' °')
        ori_grid.addWidget(QLabel('Pitch:'), 0, 2)
        ori_grid.addWidget(self.pitch_spin, 0, 3)
        
        self.yaw_spin = QDoubleSpinBox()
        self.yaw_spin.setRange(-180.0, 180.0)
        self.yaw_spin.setSingleStep(5.0)
        self.yaw_spin.setDecimals(1)
        self.yaw_spin.setValue(0.0)
        self.yaw_spin.setSuffix(' °')
        ori_grid.addWidget(QLabel('Yaw:'), 0, 4)
        ori_grid.addWidget(self.yaw_spin, 0, 5)
        
        cartesian_tab_layout.addLayout(ori_grid)
        
        # Send 버튼
        cartesian_btn_layout = QHBoxLayout()
        send_cartesian_btn = QPushButton('🎯 Send Cartesian Command')
        send_cartesian_btn.clicked.connect(self.send_cartesian_command_gui)
        cartesian_btn_layout.addWidget(send_cartesian_btn)
        
        get_current_btn = QPushButton('📍 Get Current Pose')
        get_current_btn.clicked.connect(self.get_current_cartesian_pose)
        cartesian_btn_layout.addWidget(get_current_btn)
        
        cartesian_tab_layout.addLayout(cartesian_btn_layout)
        
        cartesian_tab.setLayout(cartesian_tab_layout)
        
        # 탭에 추가
        self.control_tabs.addTab(joint_tab, '🤖 Joint Control')
        self.control_tabs.addTab(cartesian_tab, '🗺️ Cartesian Control')
        
        realtime_layout.addWidget(self.control_tabs)
        realtime_group.setLayout(realtime_layout)
        main_layout.addWidget(realtime_group)
        
        # === 시퀀스 편집 + 블록 패널 (2단 구조) ===
        editor_splitter = QSplitter(Qt.Horizontal)
        
        # 왼쪽: 시퀀스 편집 패널
        sequence_group = QGroupBox('📝 Sequence Editor')
        sequence_layout = QVBoxLayout()
        
        # Capture 버튼
        capture_layout = QHBoxLayout()
        capture_btn = QPushButton('📸 Capture Current Pose')
        capture_btn.clicked.connect(self.capture_current_pose)
        capture_layout.addWidget(capture_btn)
        
        add_joint_btn = QPushButton('➕ Joint')
        add_joint_btn.clicked.connect(self.add_joint_pose)
        capture_layout.addWidget(add_joint_btn)
        
        add_gripper_btn = QPushButton('🤏 Gripper')
        add_gripper_btn.clicked.connect(self.add_gripper_action)
        capture_layout.addWidget(add_gripper_btn)
        
        sequence_layout.addLayout(capture_layout)
        
        # 시퀀스 리스트
        self.sequence_list = QListWidget()
        self.sequence_list.setAlternatingRowColors(True)
        self.sequence_list.itemDoubleClicked.connect(self.edit_sequence_item)
        sequence_layout.addWidget(self.sequence_list)
        
        # 편집 버튼
        edit_layout = QHBoxLayout()
        edit_btn = QPushButton('✏️ Edit')
        edit_btn.clicked.connect(self.edit_sequence_item)
        edit_layout.addWidget(edit_btn)
        
        delete_btn = QPushButton('🗑️ Delete')
        delete_btn.clicked.connect(self.delete_sequence_item)
        edit_layout.addWidget(delete_btn)
        
        up_btn = QPushButton('▲')
        up_btn.clicked.connect(self.move_up)
        edit_layout.addWidget(up_btn)
        
        down_btn = QPushButton('▼')
        down_btn.clicked.connect(self.move_down)
        edit_layout.addWidget(down_btn)
        
        sequence_layout.addLayout(edit_layout)
        sequence_group.setLayout(sequence_layout)
        
        # 오른쪽: 블록 패널
        blocks_group = QGroupBox('📦 Pose & Action Blocks')
        blocks_layout = QVBoxLayout()
        
        # 블록 리스트
        self.blocks_list = QListWidget()
        self.blocks_list.setAlternatingRowColors(True)
        self.blocks_list.itemDoubleClicked.connect(self.add_block_to_sequence)
        blocks_layout.addWidget(self.blocks_list)
        
        # 블록 버튼
        block_btn_layout = QHBoxLayout()
        save_block_btn = QPushButton('💾 Save Selected Block')
        save_block_btn.clicked.connect(self.save_selected_as_block)
        block_btn_layout.addWidget(save_block_btn)
        blocks_layout.addLayout(block_btn_layout)
        
        block_btn_layout2 = QHBoxLayout()
        add_block_btn = QPushButton('← Add to Sequence')
        add_block_btn.clicked.connect(self.add_block_to_sequence)
        block_btn_layout2.addWidget(add_block_btn)
        
        delete_block_btn = QPushButton('🗑️ Delete Block')
        delete_block_btn.clicked.connect(self.delete_block)
        block_btn_layout2.addWidget(delete_block_btn)
        blocks_layout.addLayout(block_btn_layout2)
        
        blocks_group.setLayout(blocks_layout)
        
        # Splitter에 추가
        editor_splitter.addWidget(sequence_group)
        editor_splitter.addWidget(blocks_group)
        editor_splitter.setStretchFactor(0, 3)  # Sequence: 60%
        editor_splitter.setStretchFactor(1, 2)  # Blocks: 40%
        
        main_layout.addWidget(editor_splitter)
        
        # === 라이브러리 패널 (전체 시퀀스) ===
        library_group = QGroupBox('📚 Sequence Libraries')
        library_layout = QVBoxLayout()
        
        # 라이브러리 리스트
        self.library_list = QListWidget()
        self.library_list.setAlternatingRowColors(True)
        self.library_list.setMaximumHeight(120)
        library_layout.addWidget(self.library_list)
        
        # 라이브러리 버튼
        lib_btn_layout = QHBoxLayout()
        save_to_lib_btn = QPushButton('💾 Save Current Sequence')
        save_to_lib_btn.clicked.connect(self.save_sequence_to_library)
        lib_btn_layout.addWidget(save_to_lib_btn)
        
        load_from_lib_btn = QPushButton('📂 Load to Sequence')
        load_from_lib_btn.clicked.connect(self.load_library_to_sequence)
        lib_btn_layout.addWidget(load_from_lib_btn)
        
        delete_lib_btn = QPushButton('🗑️ Delete')
        delete_lib_btn.clicked.connect(self.delete_library)
        lib_btn_layout.addWidget(delete_lib_btn)
        
        library_layout.addLayout(lib_btn_layout)
        library_group.setLayout(library_layout)
        main_layout.addWidget(library_group)
        
        # === Playback 패널 ===
        playback_group = QGroupBox('▶️ Playback Control')
        playback_layout = QVBoxLayout()
        
        play_btn_layout = QHBoxLayout()
        self.play_btn = QPushButton('▶️ Play Sequence')
        self.play_btn.clicked.connect(self.play_sequence)
        play_btn_layout.addWidget(self.play_btn)
        
        self.stop_btn = QPushButton('⏹️ Stop')
        self.stop_btn.clicked.connect(self.stop_sequence)
        self.stop_btn.setEnabled(False)
        play_btn_layout.addWidget(self.stop_btn)
        
        playback_layout.addLayout(play_btn_layout)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        playback_layout.addWidget(self.progress_bar)
        
        playback_group.setLayout(playback_layout)
        main_layout.addWidget(playback_group)
    
    def update_joint_states(self, joints):
        """MoveIt2 /joint_states 업데이트"""
        self.current_joints = joints
    
    def show_ik_error(self, error_msg):
        """IK 에러 메시지를 GUI에 표시"""
        QMessageBox.warning(
            self,
            '❌ IK Solver Error',
            error_msg + '\n\nSuggestions:\n'
            '• Check if the target position is within robot workspace\n'
            '• Try smaller movements from current position\n'
            '• Avoid extreme orientations (RPY values)\n'
            '• Use "Get Current Pose" to see reachable pose'
        )
        self.status_label.setText('❌ IK Failed - Target pose unreachable')
    
    def update_joint_pos(self, full_state):
        """Isaac Sim /joint_pos 업데이트 (joints + gripper)"""
        if len(full_state) >= 7:
            joints = full_state[:6]
            gripper_rad = full_state[6]
            
            # 모니터 업데이트
            for i, (label, joint_rad) in enumerate(zip(self.joint_monitor_labels, joints)):
                joint_deg = math.degrees(joint_rad)
                label.setText(f'J{i+1}: {joint_deg:6.2f}°')
            
            gripper_deg = math.degrees(gripper_rad)
            self.gripper_monitor_label.setText(f'Gripper: {gripper_deg:5.2f}° ({gripper_rad:.4f} rad)')
            self.current_gripper_rad = gripper_rad
            
            # 실시간 모드가 아닐 때만 슬라이더 업데이트 (실시간 모드에서는 사용자가 직접 조작)
            if not self.realtime_mode:
                for i, joint_rad in enumerate(joints):
                    joint_deg = math.degrees(joint_rad)
                    self.joint_sliders[i].blockSignals(True)
                    self.joint_sliders[i].setValue(int(joint_deg))
                    self.joint_sliders[i].blockSignals(False)
                
                self.gripper_slider.blockSignals(True)
                self.gripper_slider.setValue(int(gripper_deg * 10))
                self.gripper_slider.blockSignals(False)
    
    def update_joint_slider_label(self, idx, value):
        """Joint 슬라이더 라벨 업데이트"""
        self.joint_slider_labels[idx].setText(f'{value}°')
    
    def update_gripper_slider_label(self, value):
        """그리퍼 슬라이더 라벨 업데이트"""
        gripper_deg = value / 10.0
        self.gripper_slider_label.setText(f'{gripper_deg:.1f}°')
    
    def toggle_realtime_mode(self, state):
        """실시간 제어 모드 토글"""
        self.realtime_mode = (state == Qt.Checked)
        
        # 탭 위젯 활성화/비활성화
        self.control_tabs.setEnabled(self.realtime_mode)
        
        # 시퀀스 기능 비활성화/활성화
        self.play_btn.setEnabled(not self.realtime_mode)
        
        if self.realtime_mode:
            current_tab = self.control_tabs.currentIndex()
            if current_tab == 0:
                self.status_label.setText('🤖 Real-time Joint Control Mode ON')
            else:
                self.status_label.setText('🗺️ Real-time Cartesian Control Mode ON')
            # 현재 /joint_pos 값으로 슬라이더 초기화
            # (이미 update_joint_pos에서 처리됨)
        else:
            self.status_label.setText('📝 Sequence Mode ON')
            self.ros_thread.node.stop_gripper_control()
    
    def on_control_tab_changed(self, index):
        """제어 탭 전환 시 호출"""
        if not self.realtime_mode:
            return
        
        if index == 0:  # Joint Control 탭
            self.joint_control_mode = True
            self.status_label.setText('🤖 Real-time Joint Control Mode ON')
        elif index == 1:  # Cartesian Control 탭
            self.joint_control_mode = False
            self.status_label.setText('🗺️ Real-time Cartesian Control Mode ON')
    
    def send_realtime_joint(self):
        """실시간 Joint 명령 전송"""
        # Joint Control 탭이 활성화되어 있을 때만 전송
        if not self.joint_control_mode:
            return
        
        joint_degs = [slider.value() for slider in self.joint_sliders]
        joint_rads = [math.radians(deg) for deg in joint_degs]
        
        # 단일 포즈를 0.5초 duration으로 전송
        self.ros_thread.node.send_trajectory([joint_rads], [0.5])
        self.status_label.setText(f'Sent joint command: {[f"{d:.1f}°" for d in joint_degs]}')
    
    def send_realtime_gripper(self):
        """실시간 그리퍼 명령 전송"""
        # Joint Control 탭이 활성화되어 있을 때만 전송
        if not self.joint_control_mode:
            return
        
        gripper_deg = self.gripper_slider.value() / 10.0
        gripper_rad = math.radians(gripper_deg)
        
        self.ros_thread.node.set_gripper_target(gripper_rad, activate=True)
        self.status_label.setText(f'Sent gripper command: {gripper_deg:.1f}° ({gripper_rad:.4f} rad)')
    
    def send_cartesian_command_gui(self):
        """GUI에서 Cartesian 명령 전송"""
        # Cartesian Control 탭이 활성화되어 있을 때만 전송
        if self.joint_control_mode:
            return
        
        x = self.x_spin.value()
        y = self.y_spin.value()
        z = self.z_spin.value()
        roll = math.radians(self.roll_spin.value())
        pitch = math.radians(self.pitch_spin.value())
        yaw = math.radians(self.yaw_spin.value())
        
        self.ros_thread.node.send_cartesian_command(x, y, z, roll, pitch, yaw, duration=2.0)
        self.status_label.setText(f'Sent Cartesian: XYZ=({x:.3f}, {y:.3f}, {z:.3f}), RPY=({self.roll_spin.value():.1f}°, {self.pitch_spin.value():.1f}°, {self.yaw_spin.value():.1f}°)')
    
    def get_current_joint_state(self):
        """현재 joint 값을 슬라이더에 표시"""
        joints = self.current_joints
        
        if not joints or len(joints) < 6:
            QMessageBox.warning(
                self,
                'Joint State Error',
                'Could not get current joint state.\n'
                'Make sure the robot is running and publishing joint states.'
            )
            return
        
        # 슬라이더에 현재 joint 값 설정 (라디안 → 도)
        for i in range(6):
            deg_value = math.degrees(joints[i])
            self.joint_sliders[i].blockSignals(True)  # 시그널 차단하여 무한 루프 방지
            self.joint_sliders[i].setValue(int(deg_value))
            self.joint_sliders[i].blockSignals(False)
            self.update_joint_slider_label(i, int(deg_value))
        
        # 그리퍼 값도 업데이트
        gripper_deg = math.degrees(self.current_gripper_rad)
        self.gripper_slider.blockSignals(True)
        self.gripper_slider.setValue(int(gripper_deg * 10))
        self.gripper_slider.blockSignals(False)
        self.update_gripper_slider_label(int(gripper_deg * 10))
        
        joint_str = ', '.join([f'{math.degrees(j):.1f}°' for j in joints])
        self.status_label.setText(f'📍 Current Joint State: [{joint_str}], Gripper: {gripper_deg:.1f}°')
    
    def get_current_cartesian_pose(self):
        """현재 end-effector 위치를 GUI에 표시"""
        pose = self.ros_thread.node.get_current_ee_pose()
        
        if pose is None:
            QMessageBox.warning(
                self,
                'TF Error',
                'Could not get current end-effector pose.\n'
                'Make sure TF is being published and the robot is running.'
            )
            return
        
        x, y, z, roll, pitch, yaw = pose
        
        # GUI 입력 필드에 현재 값 설정
        self.x_spin.setValue(x)
        self.y_spin.setValue(y)
        self.z_spin.setValue(z)
        self.roll_spin.setValue(math.degrees(roll))
        self.pitch_spin.setValue(math.degrees(pitch))
        self.yaw_spin.setValue(math.degrees(yaw))
        
        self.status_label.setText(f'📍 Current Pose: XYZ=({x:.3f}, {y:.3f}, {z:.3f}), RPY=({math.degrees(roll):.1f}°, {math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°)')
    
    def add_joint_pose(self):
        """Joint 포즈 추가"""
        name = f'Pose {len(self.sequence)+1}'
        joints = list(self.current_joints)
        duration = 2.0
        
        dialog = EditPoseDialog(joints, name, duration, self)
        if dialog.exec_():
            joints, name, duration = dialog.get_values()
            self.sequence.append(('joint', {'name': name, 'joints': joints, 'duration': duration}))
            self.update_sequence_list()
            self.status_label.setText(f'Added joint pose: {name}')
    
    def add_gripper_action(self):
        """그리퍼 액션 추가"""
        name = f'Gripper {len(self.sequence)+1}'
        gripper_deg = math.degrees(self.current_gripper_rad)
        duration = 1.0
        
        dialog = EditGripperDialog(name, gripper_deg, duration, self)
        if dialog.exec_():
            name, gripper_rad, duration = dialog.get_values()
            self.sequence.append(('gripper', {'name': name, 'gripper': gripper_rad, 'duration': duration}))
            self.update_sequence_list()
            self.status_label.setText(f'Added gripper action: {name}')
    
    def edit_sequence_item(self):
        """시퀀스 아이템 편집"""
        current_row = self.sequence_list.currentRow()
        if current_row < 0 or current_row >= len(self.sequence):
            return
        
        item_type, data = self.sequence[current_row]
        
        if item_type == 'joint':
            dialog = EditPoseDialog(data['joints'], data['name'], data['duration'], self)
            if dialog.exec_():
                joints, name, duration = dialog.get_values()
                self.sequence[current_row] = ('joint', {'name': name, 'joints': joints, 'duration': duration})
                self.update_sequence_list()
                self.status_label.setText(f'Edited: {name}')
        
        elif item_type == 'gripper':
            gripper_deg = math.degrees(data['gripper'])
            dialog = EditGripperDialog(data['name'], gripper_deg, data['duration'], self)
            if dialog.exec_():
                name, gripper_rad, duration = dialog.get_values()
                self.sequence[current_row] = ('gripper', {'name': name, 'gripper': gripper_rad, 'duration': duration})
                self.update_sequence_list()
                self.status_label.setText(f'Edited: {name}')
    
    def delete_sequence_item(self):
        """시퀀스 아이템 삭제"""
        current_row = self.sequence_list.currentRow()
        if current_row >= 0:
            del self.sequence[current_row]
            self.update_sequence_list()
            self.status_label.setText('Deleted item')
    
    def move_up(self):
        """아이템 위로 이동"""
        current_row = self.sequence_list.currentRow()
        if current_row > 0:
            self.sequence[current_row], self.sequence[current_row-1] = \
                self.sequence[current_row-1], self.sequence[current_row]
            self.update_sequence_list()
            self.sequence_list.setCurrentRow(current_row - 1)
    
    def move_down(self):
        """아이템 아래로 이동"""
        current_row = self.sequence_list.currentRow()
        if 0 <= current_row < len(self.sequence) - 1:
            self.sequence[current_row], self.sequence[current_row+1] = \
                self.sequence[current_row+1], self.sequence[current_row]
            self.update_sequence_list()
            self.sequence_list.setCurrentRow(current_row + 1)
    
    def update_sequence_list(self):
        """시퀀스 리스트 업데이트"""
        self.sequence_list.clear()
        for i, (item_type, data) in enumerate(self.sequence):
            if item_type == 'joint':
                joints_str = ', '.join([f"{math.degrees(j):.1f}°" for j in data['joints']])
                text = f"[JOINT] {data['name']} ({data['duration']:.1f}s): [{joints_str}]"
            elif item_type == 'gripper':
                gripper_deg = math.degrees(data['gripper'])
                text = f"[GRIPPER] {data['name']} ({data['duration']:.1f}s): {gripper_deg:.1f}°"
            self.sequence_list.addItem(text)
    
    def play_sequence(self):
        """시퀀스 재생"""
        if not self.sequence:
            QMessageBox.warning(self, 'Warning', 'Sequence is empty!')
            return
        
        if self.realtime_mode:
            QMessageBox.warning(self, 'Warning', 'Disable real-time mode first!')
            return
        
        self.is_playing = True
        self.play_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.progress_bar.setValue(0)
        
        # 별도 스레드에서 재생
        self.play_thread = threading.Thread(target=self._play_sequence_thread, daemon=True)
        self.play_thread.start()
    
    def _play_sequence_thread(self):
        """시퀀스 재생 스레드"""
        total_items = len(self.sequence)
        
        for i, (item_type, data) in enumerate(self.sequence):
            if not self.is_playing:
                break
            
            if item_type == 'joint':
                # Joint 포즈 전송
                self.ros_thread.node.send_trajectory([data['joints']], [data['duration']])
                self.status_label.setText(f"Playing [{i+1}/{total_items}]: {data['name']}")
                time.sleep(data['duration'])
            
            elif item_type == 'gripper':
                # 그리퍼 액션 전송
                self.ros_thread.node.set_gripper_target(data['gripper'], activate=True)
                self.status_label.setText(f"Playing [{i+1}/{total_items}]: {data['name']}")
                time.sleep(data['duration'])
            
            # 진행률 업데이트
            progress = int((i + 1) / total_items * 100)
            self.progress_bar.setValue(progress)
        
        self.is_playing = False
        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText('✅ Sequence completed')
    
    def stop_sequence(self):
        """시퀀스 중지"""
        self.is_playing = False
        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText('⏹️ Sequence stopped')
    
    # ========== 새로운 메서드: 블록 관리 ==========
    
    def capture_current_pose(self):
        """현재 포즈를 캡처해서 시퀀스에 추가"""
        name = f'Pose {len(self.sequence)+1}'
        joints = list(self.current_joints)
        duration = 2.0
        
        dialog = EditPoseDialog(joints, name, duration, self)
        if dialog.exec_():
            joints, name, duration = dialog.get_values()
            self.sequence.append(('joint', {'name': name, 'joints': joints, 'duration': duration}))
            self.update_sequence_list()
            self.status_label.setText(f'Added joint pose: {name}')
    
    def save_selected_as_block(self):
        """시퀀스에서 선택된 항목을 블록으로 저장"""
        current_row = self.sequence_list.currentRow()
        if current_row < 0 or current_row >= len(self.sequence):
            QMessageBox.warning(self, 'Warning', 'Select an item from sequence first!')
            return
        
        item_type, data = self.sequence[current_row]
        
        # 파일 이름 입력
        name, ok = QInputDialog.getText(self, 'Save Block', 'Enter a name for this block:')
        if not ok or not name:
            return
        
        # 파일명 정리 (공백을 언더스코어로)
        filename = name.replace(' ', '_') + '.json'
        filepath = SINGLE_POSE_DIR / filename
        
        # 중복 확인
        if filepath.exists():
            reply = QMessageBox.question(
                self, 'File Exists',
                f'{filename} already exists. Overwrite?',
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.No:
                return
        
        # 저장
        try:
            with open(filepath, 'w') as f:
                json.dump({'type': item_type, 'data': data}, f, indent=2)
            
            # 메모리에 추가
            self.single_poses[filename] = (item_type, data)
            self.update_blocks_list()
            self.status_label.setText(f'Saved block: {name}')
        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Failed to save: {str(e)}')
    
    def add_block_to_sequence(self):
        """블록 리스트에서 선택된 항목을 시퀀스에 추가"""
        current_row = self.blocks_list.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, 'Warning', 'Select a block first!')
            return
        
        # Qt.UserRole에서 파일명 가져오기
        item = self.blocks_list.item(current_row)
        filename = item.data(Qt.UserRole)
        
        if filename in self.single_poses:
            item_type, data = self.single_poses[filename]
            self.sequence.append((item_type, dict(data)))  # 복사
            self.update_sequence_list()
            self.status_label.setText(f'Added block to sequence: {filename}')
    
    def delete_block(self):
        """블록 삭제"""
        current_row = self.blocks_list.currentRow()
        if current_row < 0:
            return
        
        # Qt.UserRole에서 파일명 가져오기
        item = self.blocks_list.item(current_row)
        filename = item.data(Qt.UserRole)
        
        reply = QMessageBox.question(
            self, 'Delete Block',
            f'Delete {filename}?',
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            filepath = SINGLE_POSE_DIR / filename
            try:
                if filepath.exists():
                    filepath.unlink()
                if filename in self.single_poses:
                    del self.single_poses[filename]
                self.update_blocks_list()
                self.status_label.setText(f'Deleted block: {filename}')
            except Exception as e:
                QMessageBox.critical(self, 'Error', f'Failed to delete: {str(e)}')
    
    def update_blocks_list(self):
        """블록 리스트 업데이트"""
        self.blocks_list.clear()
        for filename, (item_type, data) in sorted(self.single_poses.items()):
            if item_type == 'joint':
                joints_str = ', '.join([f"{math.degrees(j):.1f}°" for j in data['joints']])
                display_name = filename.replace('.json', '').replace('_', ' ')
                text = f"[JOINT] {display_name}"
            elif item_type == 'gripper':
                gripper_deg = math.degrees(data['gripper'])
                display_name = filename.replace('.json', '').replace('_', ' ')
                text = f"[GRIPPER] {display_name}"
            # 파일명을 data로 저장 (나중에 참조용)
            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, filename)  # 파일명을 저장
            self.blocks_list.addItem(item)
    
    # ========== 라이브러리 관리 (파일 기반) ==========
    
    def save_sequence_to_library(self):
        """현재 시퀀스를 라이브러리 파일로 저장"""
        if not self.sequence:
            QMessageBox.warning(self, 'Warning', 'Sequence is empty!')
            return
        
        name, ok = QInputDialog.getText(self, 'Save Library', 'Enter a name for this sequence:')
        if not ok or not name:
            return
        
        filename = name.replace(' ', '_') + '.json'
        filepath = LIBRARY_DIR / filename
        
        # 중복 확인
        if filepath.exists():
            reply = QMessageBox.question(
                self, 'File Exists',
                f'{filename} already exists. Overwrite?',
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.No:
                return
        
        # 저장
        try:
            data = {
                'name': name,
                'sequence': [{'type': t, 'data': d} for t, d in self.sequence]
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            
            # 메모리에 추가
            self.library_sequences[filename] = list(self.sequence)
            self.update_library_list()
            self.status_label.setText(f'Saved to library: {name}')
        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Failed to save: {str(e)}')
    
    def load_library_to_sequence(self):
        """라이브러리에서 시퀀스 로드"""
        current_row = self.library_list.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, 'Warning', 'Select a library item first!')
            return
        
        item_text = self.library_list.item(current_row).text()
        filename = item_text.split(' (')[0].replace(' ', '_') + '.json'
        
        if filename in self.library_sequences:
            self.sequence = list(self.library_sequences[filename])
            self.update_sequence_list()
            self.status_label.setText(f'Loaded from library: {filename}')
    
    def delete_library(self):
        """라이브러리 삭제"""
        current_row = self.library_list.currentRow()
        if current_row < 0:
            return
        
        item_text = self.library_list.item(current_row).text()
        filename = item_text.split(' (')[0].replace(' ', '_') + '.json'
        
        reply = QMessageBox.question(
            self, 'Delete Library',
            f'Delete {filename}?',
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            filepath = LIBRARY_DIR / filename
            try:
                if filepath.exists():
                    filepath.unlink()
                if filename in self.library_sequences:
                    del self.library_sequences[filename]
                self.update_library_list()
                self.status_label.setText(f'Deleted library: {filename}')
            except Exception as e:
                QMessageBox.critical(self, 'Error', f'Failed to delete: {str(e)}')
    
    def update_library_list(self):
        """라이브러리 리스트 업데이트"""
        self.library_list.clear()
        for filename, sequence in sorted(self.library_sequences.items()):
            display_name = filename.replace('_', ' ').replace('.json', '')
            text = f"{display_name} ({len(sequence)} actions)"
            self.library_list.addItem(text)
    
    # ========== 파일 자동 로드 ==========
    
    def load_all_single_poses(self):
        """저장된 모든 단일 블록 로드"""
        try:
            for filepath in SINGLE_POSE_DIR.glob('*.json'):
                try:
                    with open(filepath, 'r') as f:
                        data = json.load(f)
                    
                    filename = filepath.name
                    # 최신 파일만 로드 (중복 시)
                    if filename not in self.single_poses:
                        self.single_poses[filename] = (data['type'], data['data'])
                except Exception as e:
                    print(f"Failed to load {filepath}: {e}")
            
            self.update_blocks_list()
            print(f"Loaded {len(self.single_poses)} blocks from {SINGLE_POSE_DIR}")
        except Exception as e:
            print(f"Error loading single poses: {e}")
    
    def load_all_libraries(self):
        """저장된 모든 라이브러리 로드"""
        try:
            for filepath in LIBRARY_DIR.glob('*.json'):
                try:
                    with open(filepath, 'r') as f:
                        data = json.load(f)
                    
                    filename = filepath.name
                    # 최신 파일만 로드 (중복 시)
                    if filename not in self.library_sequences:
                        sequence = [(item['type'], item['data']) for item in data['sequence']]
                        self.library_sequences[filename] = sequence
                except Exception as e:
                    print(f"Failed to load {filepath}: {e}")
            
            self.update_library_list()
            print(f"Loaded {len(self.library_sequences)} libraries from {LIBRARY_DIR}")
        except Exception as e:
            print(f"Error loading libraries: {e}")
    
    def closeEvent(self, event):
        """프로그램 종료 시"""
        self.is_playing = False
        self.ros_thread.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
