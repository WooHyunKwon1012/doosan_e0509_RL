import rclpy
import std_msgs.msg

import omni.graph.core
from isaacsim.core.nodes import BaseResetNode

from custom.python.ros2_node.ogn.OgnCustomPythonRos2NodePyDatabase import (
    OgnCustomPythonRos2NodePyDatabase
)


class OgnCustomPythonRos2NodePyInternalState(BaseResetNode):
    """ROS2 노드 + 구독자 + 퍼블리셔 관리"""

    def __init__(self):
        self._ros2_node = None
        self._sub = None
        self._pub = None
        self._data = None
        super().__init__(initialize=False)

    def _callback(self, msg):
        """구독 콜백: 가장 최근 값 저장"""
        self._data = msg.data

    def initialize(self):
        """ROS2 노드 생성 + 구독자(/test_msg) + 퍼블리셔(/test_msg_scaled)"""
        try:
            rclpy.init()
        except:
            pass

        if not self._ros2_node:
            self._ros2_node = rclpy.create_node("isaacsim_multiplier_node")

        if not self._sub:
            self._sub = self._ros2_node.create_subscription(
                std_msgs.msg.Int32,
                "/test_msg",
                self._callback,
                10
            )

        if not self._pub:
            self._pub = self._ros2_node.create_publisher(
                std_msgs.msg.Int32,
                "/test_msg_scaled",
                10
            )

        self.initialized = True

    def spin_once(self):
        rclpy.spin_once(self._ros2_node, timeout_sec=0.01)

    def publish(self, value):
        msg = std_msgs.msg.Int32()
        msg.data = int(value)
        self._pub.publish(msg)

    @property
    def data(self):
        tmp = self._data
        self._data = None
        return tmp

    def custom_reset(self):
        try:
            if self._ros2_node:
                if self._sub:
                    self._ros2_node.destroy_subscription(self._sub)
                if self._pub:
                    self._ros2_node.destroy_publisher(self._pub)
                self._ros2_node.destroy_node()
        except:
            pass

        self._ros2_node = None
        self._sub = None
        self._pub = None
        self._data = None
        self.initialized = False

        try:
            rclpy.try_shutdown()
        except:
            pass


class OgnCustomPythonRos2NodePy:
    """OmniGraph compute"""

    @staticmethod
    def internal_state():
        return OgnCustomPythonRos2NodePyInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state

        try:
            if not state.initialized:
                state.initialize()

            # 메시지 체크
            state.spin_once()
            number = state.data

            if number is not None:
                scaled = number * 15

                # 퍼블리시
                state.publish(scaled)

                # 그래프 트리거
                db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED

        except Exception as e:
            db.log_error(f"Error: {e}")
            return False

        return True

    @staticmethod
    def release(node):
        try:
            state = OgnCustomPythonRos2NodePyDatabase.per_instance_internal_state(node)
            state.reset()
            state.initialized = False
        except:
            pass
