# test_topic_pub_fixed.py
import time
import traceback

# --- 먼저 SimulationApp을 import & 인스턴스화 ---
from isaacsim import SimulationApp
# SimulationApp 옵션 (headless: True/False 필요에 따라 설정)
simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

# --- 이후에 다른 IsaacSim / Omniverse 관련 import ---
import omni
from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

# Ros2 imports (rclpy는 여기서 import해도 무방)
import rclpy
import std_msgs.msg

# ---- ROS2 publisher/subscriber 클래스 (이전 예제와 유사) ----
class Ros2Multiplier:
    def __init__(self, node_name="isaacsim_multiplier_node"):
        self._node = None
        self._sub = None
        self._pub = None
        self._latest = None
        self._initialized = False
        self._node_name = node_name

    def initialize(self, sub_topic="/test_msg", pub_topic="/output_msg"):
        try:
            rclpy.init()
        except Exception:
            pass

        if not self._node:
            try:
                self._node = rclpy.create_node(self._node_name)
            except Exception:
                from rclpy.node import Node
                self._node = Node(self._node_name)

        if not self._sub:
            self._sub = self._node.create_subscription(
                std_msgs.msg.Int32, sub_topic, self._callback, 10
            )
        if not self._pub:
            self._pub = self._node.create_publisher(std_msgs.msg.Int32, pub_topic, 10)

        self._initialized = True
        self._node.get_logger().info(f"Initialized: sub={sub_topic} pub={pub_topic}")

    def _callback(self, msg):
        try:
            self._latest = int(msg.data)
            self._node.get_logger().info(f"Received: {self._latest}")
        except Exception:
            self._node.get_logger().warn("Non-int data received")

    def spin_once(self, timeout_sec=0.01):
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def consume_latest(self):
        v = self._latest
        self._latest = None
        return v

    def publish_int(self, value):
        if not self._pub:
            return False
        try:
            msg = std_msgs.msg.Int32()
            msg.data = int(value)
            self._pub.publish(msg)
            self._node.get_logger().info(f"Published: {msg.data}")
            return True
        except Exception as e:
            self._node.get_logger().error(f"Publish failed: {e}")
            return False

    def shutdown(self):
        try:
            if self._node:
                try:
                    if self._sub:
                        self._node.destroy_subscription(self._sub)
                except Exception:
                    pass
                try:
                    if self._pub:
                        self._node.destroy_publisher(self._pub)
                except Exception:
                    pass
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
        finally:
            self._latest = None
            self._sub = None
            self._pub = None
            self._node = None
            self._initialized = False
            try:
                rclpy.try_shutdown()
            except Exception:
                pass


# ---- main ----
def main():
    ros = Ros2Multiplier()
    ros.initialize(sub_topic="/test_msg", pub_topic="/output_msg")

    try:
        while simulation_app.is_running():
            ros.spin_once(timeout_sec=0.01)

            number = ros.consume_latest()
            if number is not None:
                scaled = number * 15
                ros.publish_int(scaled)

            # Isaac Sim 프레임/업데이트
            simulation_app.update()
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    except Exception:
        print("Exception in main loop:")
        traceback.print_exc()
    finally:
        ros.shutdown()
        try:
            simulation_app.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
