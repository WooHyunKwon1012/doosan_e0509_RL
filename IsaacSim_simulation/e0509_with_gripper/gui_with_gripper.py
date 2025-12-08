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
    QInputDialog, QSplitter, QListWidgetItem
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32MultiArray, Float32

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
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.running = False
        
    def start(self):
        self.running = True
        rclpy.init()
        self.node = JointStateMonitor(self.joint_state_updated, self.joint_pos_updated)
        
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
    def __init__(self, joint_state_signal, joint_pos_signal):
        super().__init__('joint_pose_gui')
        self.joint_state_signal = joint_state_signal
        self.joint_pos_signal = joint_pos_signal
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
        
        self.get_logger().info('Joint State Monitor started')
        
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
        
        # ROS 스레드
        self.ros_thread = ROSThread()
        self.ros_thread.joint_state_updated.connect(self.update_joint_states)
        self.ros_thread.joint_pos_updated.connect(self.update_joint_pos)
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
        
        # === 실시간 제어 패널 ===
        realtime_group = QGroupBox('🎮 Real-time Control')
        realtime_layout = QVBoxLayout()
        
        # 모드 전환 버튼
        mode_layout = QHBoxLayout()
        self.realtime_toggle = QCheckBox('Enable Real-time Control')
        self.realtime_toggle.stateChanged.connect(self.toggle_realtime_mode)
        mode_layout.addWidget(self.realtime_toggle)
        mode_layout.addStretch()
        realtime_layout.addLayout(mode_layout)
        
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
            slider.setEnabled(False)
            slider.valueChanged.connect(lambda v, idx=i: self.update_joint_slider_label(idx, v))
            self.joint_sliders.append(slider)
            h_layout.addWidget(slider)
            
            value_label = QLabel('0°')
            value_label.setFixedWidth(50)
            self.joint_slider_labels.append(value_label)
            h_layout.addWidget(value_label)
            
            realtime_layout.addLayout(h_layout)
        
        # Joint Send 버튼
        joint_send_btn = QPushButton('Send Joint Command')
        joint_send_btn.clicked.connect(self.send_realtime_joint)
        joint_send_btn.setEnabled(False)
        self.joint_send_btn = joint_send_btn
        realtime_layout.addWidget(joint_send_btn)
        
        # 그리퍼 슬라이더
        gripper_layout = QHBoxLayout()
        gripper_layout.addWidget(QLabel('Gripper:'))
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 631)  # 0~63.1도 * 10
        self.gripper_slider.setValue(0)
        self.gripper_slider.setEnabled(False)
        self.gripper_slider.valueChanged.connect(self.update_gripper_slider_label)
        gripper_layout.addWidget(self.gripper_slider)
        
        self.gripper_slider_label = QLabel('0.0°')
        self.gripper_slider_label.setFixedWidth(50)
        gripper_layout.addWidget(self.gripper_slider_label)
        realtime_layout.addLayout(gripper_layout)
        
        # Gripper Send 버튼
        gripper_send_btn = QPushButton('Send Gripper Command')
        gripper_send_btn.clicked.connect(self.send_realtime_gripper)
        gripper_send_btn.setEnabled(False)
        self.gripper_send_btn = gripper_send_btn
        realtime_layout.addWidget(gripper_send_btn)
        
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
            
            # 실시간 모드가 아닐 때만 슬라이더 업데이트
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
        
        # 슬라이더 활성화/비활성화
        for slider in self.joint_sliders:
            slider.setEnabled(self.realtime_mode)
        self.gripper_slider.setEnabled(self.realtime_mode)
        self.joint_send_btn.setEnabled(self.realtime_mode)
        self.gripper_send_btn.setEnabled(self.realtime_mode)
        
        # 시퀀스 기능 비활성화/활성화
        self.play_btn.setEnabled(not self.realtime_mode)
        
        if self.realtime_mode:
            self.status_label.setText('🎮 Real-time Control Mode ON')
            # 현재 /joint_pos 값으로 슬라이더 초기화
            # (이미 update_joint_pos에서 처리됨)
        else:
            self.status_label.setText('📝 Sequence Mode ON')
            self.ros_thread.node.stop_gripper_control()
    
    def send_realtime_joint(self):
        """실시간 Joint 명령 전송"""
        joint_degs = [slider.value() for slider in self.joint_sliders]
        joint_rads = [math.radians(deg) for deg in joint_degs]
        
        # 단일 포즈를 0.5초 duration으로 전송
        self.ros_thread.node.send_trajectory([joint_rads], [0.5])
        self.status_label.setText(f'Sent joint command: {[f"{d:.1f}°" for d in joint_degs]}')
    
    def send_realtime_gripper(self):
        """실시간 그리퍼 명령 전송"""
        gripper_deg = self.gripper_slider.value() / 10.0
        gripper_rad = math.radians(gripper_deg)
        
        self.ros_thread.node.set_gripper_target(gripper_rad, activate=True)
        self.status_label.setText(f'Sent gripper command: {gripper_deg:.1f}° ({gripper_rad:.4f} rad)')
    
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
