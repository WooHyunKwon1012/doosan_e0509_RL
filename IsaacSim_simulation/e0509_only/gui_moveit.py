#!/usr/bin/env python3

import sys
import json
import math
import threading
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QLabel, QDoubleSpinBox, QLineEdit,
    QFileDialog, QMessageBox, QCheckBox, QProgressBar, QDialog,
    QDialogButtonBox, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# GUI 설정 파일 import
from gui_config import *


class ROSThread(QObject):
    """ROS2를 별도 스레드에서 실행"""
    joint_state_updated = pyqtSignal(list)
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.running = False
        
    def start(self):
        self.running = True
        rclpy.init()
        self.node = JointStateMonitor(self.joint_state_updated)
        
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
    """ROS2 노드: /joint_states 구독 및 명령 발행"""
    def __init__(self, update_signal):
        super().__init__('joint_pose_gui')
        self.update_signal = update_signal
        self.current_joints = [0.0] * 6
        
        # /joint_states 구독
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # MoveIt trajectory 발행
        self.pub = self.create_publisher(
            JointTrajectory,
            '/dsr_moveit_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Joint State Monitor started')
        
    def joint_state_callback(self, msg: JointState):
        # e0509의 joint_1 ~ joint_6만 추출
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joints = []
        
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joints.append(msg.position[idx])
            else:
                joints.append(0.0)
        
        self.current_joints = joints
        self.update_signal.emit(joints)
        
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
        
        self.pub.publish(traj)
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
            spin.setValue(self.pose[i] * 180.0 / math.pi)  # 라디안을 각도로 변환
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
        pose = [spin.value() * math.pi / 180.0 for spin in self.joint_spins]  # 각도를 라디안으로 변환
        name = self.name_edit.text()
        duration = self.duration_spin.value()
        return pose, name, duration


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(WINDOW_TITLE)
        self.setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT)
        
        # 데이터
        self.poses = []  # 재생할 시퀀스 [(name, joints, duration), ...]
        self.library_poses = []  # 포즈 라이브러리 (재료 보관)
        self.current_joints = [0.0] * 6
        self.is_playing = False
        self.play_thread = None
        
        # ROS 스레드
        self.ros_thread = ROSThread()
        self.ros_thread.joint_state_updated.connect(self.update_current_joints)
        self.ros_thread.start()
        
        self.init_ui()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # === 현재 Joint State 표시 (각도) ===
        main_layout.addWidget(QLabel('Current Joint State (degrees):'))
        self.joint_labels = []
        joint_layout = QHBoxLayout()
        for i in range(6):
            label = QLabel(f'J{i+1}: 0.00°')
            self.joint_labels.append(label)
            joint_layout.addWidget(label)
        main_layout.addLayout(joint_layout)
        
        # === Capture 버튼 ===
        capture_layout = QHBoxLayout()
        
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText('Pose name (optional)')
        capture_layout.addWidget(self.name_input)
        
        capture_layout.addWidget(QLabel('Duration:'))
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(DURATION_MIN, DURATION_MAX)
        self.duration_spin.setSingleStep(DURATION_STEP)
        self.duration_spin.setValue(DURATION_DEFAULT)
        self.duration_spin.setSuffix(' s')
        capture_layout.addWidget(self.duration_spin)
        
        capture_btn = QPushButton('📸 Capture Pose')
        capture_btn.clicked.connect(self.capture_pose)
        capture_layout.addWidget(capture_btn)
        
        manual_btn = QPushButton('✍️ Manual Input')
        manual_btn.clicked.connect(self.manual_input_pose)
        capture_layout.addWidget(manual_btn)
        
        main_layout.addLayout(capture_layout)
        
        # === 포즈 관리 영역 (두 개의 리스트) ===
        lists_container = QHBoxLayout()
        lists_container.setSpacing(LIST_CONTAINER_SPACING)
        
        # 왼쪽: 재생 시퀀스
        sequence_container = QVBoxLayout()
        sequence_label = QLabel('▶️ Playback Sequence')
        sequence_label.setStyleSheet(STYLE_SECTION_LABEL)
        sequence_container.addWidget(sequence_label)
        
        self.pose_list = QListWidget()
        self.pose_list.setSelectionMode(QListWidget.SingleSelection)
        self.pose_list.itemDoubleClicked.connect(self.edit_pose)
        self.pose_list.setMinimumHeight(LIST_MIN_HEIGHT)
        self.pose_list.setAlternatingRowColors(LIST_ALTERNATING_COLORS)
        sequence_container.addWidget(self.pose_list, 1)
        
        # 시퀀스 제어 버튼
        seq_btn_layout = QHBoxLayout()
        seq_btn_layout.setSpacing(BUTTON_SPACING)
        seq_btn_layout.setContentsMargins(0, LAYOUT_MARGIN_TOP, 0, LAYOUT_MARGIN_BOTTOM)
        
        up_btn = QPushButton('▲')
        up_btn.setFixedWidth(BUTTON_ARROW_WIDTH)
        up_btn.setMinimumHeight(BUTTON_HEIGHT)
        up_btn.clicked.connect(self.move_up)
        seq_btn_layout.addWidget(up_btn)
        
        down_btn = QPushButton('▼')
        down_btn.setFixedWidth(BUTTON_ARROW_WIDTH)
        down_btn.setMinimumHeight(BUTTON_HEIGHT)
        down_btn.clicked.connect(self.move_down)
        seq_btn_layout.addWidget(down_btn)
        
        edit_seq_btn = QPushButton('✏️ Edit')
        edit_seq_btn.setMinimumHeight(BUTTON_HEIGHT)
        edit_seq_btn.clicked.connect(self.edit_pose)
        seq_btn_layout.addWidget(edit_seq_btn)
        
        update_seq_btn = QPushButton('🔄 Update')
        update_seq_btn.setMinimumHeight(BUTTON_HEIGHT)
        update_seq_btn.clicked.connect(self.update_pose_from_current)
        seq_btn_layout.addWidget(update_seq_btn)
        
        sequence_container.addLayout(seq_btn_layout)
        
        seq_btn_layout2 = QHBoxLayout()
        seq_btn_layout2.setSpacing(BUTTON_SPACING)
        
        to_library_btn = QPushButton('⬇️ To Library')
        to_library_btn.setMinimumHeight(BUTTON_HEIGHT)
        to_library_btn.clicked.connect(self.move_to_library)
        seq_btn_layout2.addWidget(to_library_btn)
        
        remove_from_seq_btn = QPushButton('❌ Remove')
        remove_from_seq_btn.setMinimumHeight(BUTTON_HEIGHT)
        remove_from_seq_btn.clicked.connect(self.delete_pose)
        seq_btn_layout2.addWidget(remove_from_seq_btn)
        
        sequence_container.addLayout(seq_btn_layout2)
        lists_container.addLayout(sequence_container, 1)
        
        # 오른쪽: 포즈 라이브러리
        library_container = QVBoxLayout()
        library_label = QLabel('📚 Pose Library')
        library_label.setStyleSheet(STYLE_SECTION_LABEL)
        library_container.addWidget(library_label)
        
        self.library_list = QListWidget()
        self.library_list.setSelectionMode(QListWidget.SingleSelection)
        self.library_list.itemDoubleClicked.connect(self.edit_library_pose)
        self.library_list.setMinimumHeight(LIST_MIN_HEIGHT)
        self.library_list.setAlternatingRowColors(LIST_ALTERNATING_COLORS)
        library_container.addWidget(self.library_list, 1)
        
        # 라이브러리 버튼
        lib_btn_layout = QHBoxLayout()
        lib_btn_layout.setSpacing(BUTTON_SPACING)
        lib_btn_layout.setContentsMargins(0, LAYOUT_MARGIN_TOP, 0, LAYOUT_MARGIN_BOTTOM)
        
        save_to_lib_btn = QPushButton('💾 Save Here')
        save_to_lib_btn.setMinimumHeight(BUTTON_HEIGHT)
        save_to_lib_btn.clicked.connect(self.save_to_library)
        lib_btn_layout.addWidget(save_to_lib_btn)
        
        edit_lib_btn = QPushButton('✏️ Edit')
        edit_lib_btn.setMinimumHeight(BUTTON_HEIGHT)
        edit_lib_btn.clicked.connect(self.edit_library_pose)
        lib_btn_layout.addWidget(edit_lib_btn)
        
        library_container.addLayout(lib_btn_layout)
        
        lib_btn_layout2 = QHBoxLayout()
        lib_btn_layout2.setSpacing(BUTTON_SPACING)
        
        to_sequence_btn = QPushButton('⬆️ To Sequence')
        to_sequence_btn.setMinimumHeight(BUTTON_HEIGHT)
        to_sequence_btn.clicked.connect(self.move_to_sequence)
        lib_btn_layout2.addWidget(to_sequence_btn)
        
        delete_lib_btn = QPushButton('🗑️ Delete')
        delete_lib_btn.setMinimumHeight(BUTTON_HEIGHT)
        delete_lib_btn.clicked.connect(self.delete_library_pose)
        lib_btn_layout2.addWidget(delete_lib_btn)
        
        library_container.addLayout(lib_btn_layout2)
        lists_container.addLayout(library_container, 1)
        
        main_layout.addLayout(lists_container)
        
        # === 재생 제어 ===
        playback_group_label = QLabel('Playback Control')
        playback_group_label.setStyleSheet(STYLE_SECTION_LABEL)
        main_layout.addWidget(playback_group_label)
        
        play_grid = QGridLayout()
        play_grid.setSpacing(LAYOUT_SPACING)
        play_grid.setContentsMargins(0, LAYOUT_MARGIN_TOP, 0, LAYOUT_MARGIN_TOP)
        
        play_all_btn = QPushButton('▶️ Play All')
        play_all_btn.setMinimumHeight(BUTTON_HEIGHT)
        play_all_btn.clicked.connect(self.play_all)
        play_grid.addWidget(play_all_btn, 0, 0, 1, 2)
        
        go_to_btn = QPushButton('⏯️ Go to Selected')
        go_to_btn.setMinimumHeight(BUTTON_HEIGHT)
        go_to_btn.clicked.connect(self.go_to_selected)
        play_grid.addWidget(go_to_btn, 0, 2, 1, 2)
        
        stop_btn = QPushButton('⏹️ Stop')
        stop_btn.setMinimumHeight(BUTTON_HEIGHT)
        stop_btn.clicked.connect(self.stop_play)
        play_grid.addWidget(stop_btn, 0, 4)
        
        self.loop_check = QCheckBox('🔁 Loop')
        play_grid.addWidget(self.loop_check, 0, 5)
        
        copy_btn = QPushButton('📋 Copy Selected')
        copy_btn.setMinimumHeight(BUTTON_HEIGHT)
        copy_btn.clicked.connect(self.copy_pose)
        play_grid.addWidget(copy_btn, 1, 0, 1, 2)
        
        clear_btn = QPushButton('🗑️ Clear All Sequence')
        clear_btn.setMinimumHeight(BUTTON_HEIGHT)
        clear_btn.clicked.connect(self.clear_all)
        play_grid.addWidget(clear_btn, 1, 2, 1, 2)
        
        main_layout.addLayout(play_grid)
        
        # === 진행 상태 ===
        status_group_label = QLabel('Status')
        status_group_label.setStyleSheet(STYLE_SECTION_LABEL)
        main_layout.addWidget(status_group_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setMinimumHeight(PROGRESS_BAR_HEIGHT)
        main_layout.addWidget(self.progress_bar)
        
        self.status_label = QLabel('✅ Ready')
        self.status_label.setStyleSheet(STYLE_STATUS_LABEL)
        main_layout.addWidget(self.status_label)
        
        # === 파일 저장/불러오기 ===
        file_group_label = QLabel('File Management')
        file_group_label.setStyleSheet(STYLE_SECTION_LABEL)
        main_layout.addWidget(file_group_label)
        
        file_layout = QHBoxLayout()
        file_layout.setSpacing(LAYOUT_SPACING)
        file_layout.setContentsMargins(0, LAYOUT_MARGIN_TOP, 0, LAYOUT_MARGIN_TOP)
        
        save_btn = QPushButton('💾 Save (Sequence + Library)')
        save_btn.setMinimumHeight(BUTTON_HEIGHT)
        save_btn.clicked.connect(self.save_sequence)
        file_layout.addWidget(save_btn)
        
        load_btn = QPushButton('📂 Load (Sequence + Library)')
        load_btn.setMinimumHeight(BUTTON_HEIGHT)
        load_btn.clicked.connect(self.load_sequence)
        file_layout.addWidget(load_btn)
        
        main_layout.addLayout(file_layout)
        
    def update_current_joints(self, joints):
        """ROS2에서 받은 joint state 업데이트 (각도로 표시)"""
        self.current_joints = joints
        for i, label in enumerate(self.joint_labels):
            degrees = joints[i] * 180.0 / math.pi
            label.setText(f'J{i+1}: {degrees:.2f}°')
            
    def capture_pose(self):
        """현재 joint state를 목록에 추가"""
        name = self.name_input.text().strip()
        if not name:
            name = f'Pose {len(self.poses) + 1}'
        
        duration = self.duration_spin.value()
        joints = list(self.current_joints)
        
        self.poses.append((name, joints, duration))
        self.update_pose_list()
        
        self.name_input.clear()
        self.status_label.setText(f'✅ Captured: {name}')
        
    def manual_input_pose(self):
        """수동으로 조인트 값을 입력하여 포즈 추가"""
        name = self.name_input.text().strip()
        if not name:
            name = f'Pose {len(self.poses) + 1}'
        
        duration = self.duration_spin.value()
        
        # 현재 joint 값을 초기값으로 사용
        initial_joints = list(self.current_joints)
        
        dialog = EditPoseDialog(initial_joints, name, duration, self)
        
        if dialog.exec_():
            joints, new_name, new_duration = dialog.get_values()
            self.poses.append((new_name, joints, new_duration))
            self.update_pose_list()
            
            self.name_input.clear()
            self.status_label.setText(f'✍️ Manually added: {new_name}')
        
    def update_pose_list(self):
        """재생 시퀀스 리스트 UI 업데이트 (각도로 표시)"""
        self.pose_list.clear()
        for i, (name, joints, duration) in enumerate(self.poses):
            joints_deg = [j * 180.0 / math.pi for j in joints]
            joints_str = ', '.join([f'{j:.1f}°' for j in joints_deg])
            item_text = f'{i+1}. {name} ({duration}s): [{joints_str}]'
            self.pose_list.addItem(item_text)
            
    def update_library_list(self):
        """포즈 라이브러리 리스트 UI 업데이트 (각도로 표시)"""
        self.library_list.clear()
        for i, (name, joints, duration) in enumerate(self.library_poses):
            joints_deg = [j * 180.0 / math.pi for j in joints]
            joints_str = ', '.join([f'{j:.1f}°' for j in joints_deg])
            item_text = f'{name} ({duration}s): [{joints_str}]'
            self.library_list.addItem(item_text)
            
    def save_to_library(self):
        """현재 joint state를 라이브러리에 직접 저장"""
        name = self.name_input.text().strip()
        if not name:
            name = f'Lib Pose {len(self.library_poses) + 1}'
        
        duration = self.duration_spin.value()
        joints = list(self.current_joints)
        
        self.library_poses.append((name, joints, duration))
        self.update_library_list()
        
        self.name_input.clear()
        self.status_label.setText(f'💾 Saved to library: {name}')
        
    def move_to_library(self):
        """선택한 포즈를 시퀀스에서 라이브러리로 이동"""
        idx = self.pose_list.currentRow()
        if idx < 0:
            QMessageBox.warning(self, 'Warning', 'Please select a pose from sequence!')
            return
            
        pose = self.poses.pop(idx)
        self.library_poses.append(pose)
        
        self.update_pose_list()
        self.update_library_list()
        self.status_label.setText(f'⬇️ Moved to library: {pose[0]}')
        
    def move_to_sequence(self):
        """선택한 포즈를 라이브러리에서 시퀀스로 이동"""
        idx = self.library_list.currentRow()
        if idx < 0:
            QMessageBox.warning(self, 'Warning', 'Please select a pose from library!')
            return
            
        pose = self.library_poses[idx]  # 복사 (라이브러리에는 유지)
        self.poses.append(pose)
        
        self.update_pose_list()
        self.status_label.setText(f'⬆️ Added to sequence: {pose[0]}')
        
    def delete_library_pose(self):
        """라이브러리에서 포즈 삭제"""
        idx = self.library_list.currentRow()
        if idx < 0:
            QMessageBox.warning(self, 'Warning', 'Please select a pose from library!')
            return
            
        name = self.library_poses[idx][0]
        reply = QMessageBox.question(
            self, 'Delete from Library', 
            f'Delete "{name}" from library?',
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.library_poses.pop(idx)
            self.update_library_list()
            self.status_label.setText(f'🗑️ Deleted from library: {name}')
            
    def edit_library_pose(self):
        """라이브러리의 포즈 편집"""
        idx = self.library_list.currentRow()
        if idx < 0:
            QMessageBox.warning(self, 'Warning', 'Please select a pose to edit!')
            return
            
        name, joints, duration = self.library_poses[idx]
        dialog = EditPoseDialog(joints, name, duration, self)
        
        if dialog.exec_():
            new_joints, new_name, new_duration = dialog.get_values()
            self.library_poses[idx] = (new_name, new_joints, new_duration)
            self.update_library_list()
            self.library_list.setCurrentRow(idx)
            self.status_label.setText(f'✏️ Edited in library: {new_name}')
            
    def move_up(self):
        """선택한 포즈를 위로 이동"""
        idx = self.pose_list.currentRow()
        if idx > 0:
            self.poses[idx], self.poses[idx-1] = self.poses[idx-1], self.poses[idx]
            self.update_pose_list()
            self.pose_list.setCurrentRow(idx-1)
            
    def move_down(self):
        """선택한 포즈를 아래로 이동"""
        idx = self.pose_list.currentRow()
        if 0 <= idx < len(self.poses) - 1:
            self.poses[idx], self.poses[idx+1] = self.poses[idx+1], self.poses[idx]
            self.update_pose_list()
            self.pose_list.setCurrentRow(idx+1)
            
    def edit_pose(self):
        """선택한 포즈 편집"""
        idx = self.pose_list.currentRow()
        if idx < 0:
            QMessageBox.warning(self, 'Warning', 'Please select a pose to edit!')
            return
            
        name, joints, duration = self.poses[idx]
        dialog = EditPoseDialog(joints, name, duration, self)
        
        if dialog.exec_():
            new_joints, new_name, new_duration = dialog.get_values()
            self.poses[idx] = (new_name, new_joints, new_duration)
            self.update_pose_list()
            self.pose_list.setCurrentRow(idx)  # 편집 후 같은 위치 선택 유지
            self.status_label.setText(f'✏️ Edited: {new_name}')
            
    def update_pose_from_current(self):
        """선택한 포즈를 현재 joint state로 업데이트"""
        idx = self.pose_list.currentRow()
        if idx < 0:
            QMessageBox.warning(self, 'Warning', 'Please select a pose to update!')
            return
            
        name, _, duration = self.poses[idx]
        joints = list(self.current_joints)
        
        reply = QMessageBox.question(
            self, 'Update Pose', 
            f'Update "{name}" with current joint state?',
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.poses[idx] = (name, joints, duration)
            self.update_pose_list()
            self.pose_list.setCurrentRow(idx)
            self.status_label.setText(f'🔄 Updated: {name}')
            
    def delete_pose(self):
        """선택한 포즈 삭제"""
        idx = self.pose_list.currentRow()
        if idx >= 0:
            name = self.poses[idx][0]
            self.poses.pop(idx)
            self.update_pose_list()
            self.status_label.setText(f'🗑️ Deleted: {name}')
            
    def copy_pose(self):
        """선택한 포즈 복제"""
        idx = self.pose_list.currentRow()
        if idx >= 0:
            name, joints, duration = self.poses[idx]
            new_name = f'{name} (copy)'
            self.poses.insert(idx + 1, (new_name, list(joints), duration))
            self.update_pose_list()
            self.status_label.setText(f'📋 Copied: {name}')
            
    def clear_all(self):
        """모든 포즈 삭제"""
        reply = QMessageBox.question(self, 'Clear All', 
                                     'Delete all poses?',
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.poses.clear()
            self.update_pose_list()
            self.status_label.setText('🔄 Cleared all poses')
            
    def play_all(self):
        """모든 포즈 순차 실행"""
        if not self.poses:
            QMessageBox.warning(self, 'Warning', 'No poses to play!')
            return
            
        if self.is_playing:
            return
            
        self.is_playing = True
        self.play_thread = threading.Thread(target=self._play_sequence, daemon=True)
        self.play_thread.start()
        
    def _play_sequence(self):
        """포즈 시퀀스 재생 (별도 스레드)"""
        while True:
            poses_data = [p[1] for p in self.poses]  # joints만 추출
            durations = [p[2] for p in self.poses]  # durations
            
            total = len(poses_data)
            
            for i, (pose, duration) in enumerate(zip(poses_data, durations)):
                if not self.is_playing:
                    break
                    
                self.status_label.setText(f'▶️ Playing pose {i+1}/{total}...')
                self.progress_bar.setValue(int((i+1) / total * 100))
                
                # 단일 포즈 전송
                self.ros_thread.node.send_trajectory([pose], [duration])
                
                # duration + 약간의 여유 시간 대기
                time.sleep(duration + 0.5)
                
            self.progress_bar.setValue(100)
            
            # Loop 체크
            if not self.loop_check.isChecked():
                break
                
        self.is_playing = False
        self.status_label.setText('✅ Playback completed')
        self.progress_bar.setValue(0)
        
    def go_to_selected(self):
        """선택한 포즈로 즉시 이동"""
        idx = self.pose_list.currentRow()
        if idx < 0:
            QMessageBox.warning(self, 'Warning', 'No pose selected!')
            return
            
        name, joints, duration = self.poses[idx]
        self.ros_thread.node.send_trajectory([joints], [duration])
        self.status_label.setText(f'⏯️ Going to: {name}')
        
    def stop_play(self):
        """재생 중지"""
        self.is_playing = False
        self.status_label.setText('⏹️ Stopped')
        
    def save_sequence(self):
        """포즈 시퀀스와 라이브러리를 파일로 저장"""
        if not self.poses and not self.library_poses:
            QMessageBox.warning(self, 'Warning', 'No poses to save!')
            return
            
        file_path, _ = QFileDialog.getSaveFileName(
            self, 'Save Sequence', '', 'JSON Files (*.json)'
        )
        
        if file_path:
            data = {
                'sequence': [
                    {'name': name, 'joints': joints, 'duration': duration}
                    for name, joints, duration in self.poses
                ],
                'library': [
                    {'name': name, 'joints': joints, 'duration': duration}
                    for name, joints, duration in self.library_poses
                ]
            }
            
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
                
            self.status_label.setText(f'💾 Saved sequence & library to: {file_path}')
            
    def load_sequence(self):
        """파일에서 포즈 시퀀스와 라이브러리 불러오기"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, 'Load Sequence', '', 'JSON Files (*.json)'
        )
        
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)
                
                # 이전 버전 호환성 (poses만 있는 경우)
                if 'poses' in data:
                    self.poses = [
                        (p['name'], p['joints'], p['duration'])
                        for p in data['poses']
                    ]
                    self.library_poses = []
                # 새 버전 (sequence와 library 분리)
                else:
                    self.poses = [
                        (p['name'], p['joints'], p['duration'])
                        for p in data.get('sequence', [])
                    ]
                    self.library_poses = [
                        (p['name'], p['joints'], p['duration'])
                        for p in data.get('library', [])
                    ]
                
                self.update_pose_list()
                self.update_library_list()
                self.status_label.setText(f'📂 Loaded from: {file_path}')
                
            except Exception as e:
                QMessageBox.critical(self, 'Error', f'Failed to load: {str(e)}')
                
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