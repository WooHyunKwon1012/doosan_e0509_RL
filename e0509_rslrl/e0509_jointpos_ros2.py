from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import os
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.extensions import enable_extension

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

print("==== ENV DIAG ====")
print("ROS_DOMAIN_ID:", os.environ.get("ROS_DOMAIN_ID"))
print("RMW_IMPLEMENTATION:", os.environ.get("RMW_IMPLEMENTATION"))

# ROS2 bridge 활성화
enable_extension("isaacsim.ros2.bridge")

# 1. World + 로봇 로드 (그리퍼 없는 e0509)
my_world = World(stage_units_in_meters=1.0)
usd_path = "/home/woo/Downloads/source/3d/e0509/e0509.usd"
robot_prim_path = "/World/e0509"
add_reference_to_stage(usd_path=usd_path, prim_path=robot_prim_path)
my_world.scene.add_default_ground_plane()

# 2. PhysicsScene 세팅
from pxr import UsdPhysics, PhysxSchema, Gf
stage = my_world.stage
physics_scene_path = "/World/physics"
if not stage.GetPrimAtPath(physics_scene_path):
    scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
else:
    scene = UsdPhysics.Scene(stage.GetPrimAtPath(physics_scene_path))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_path))

# 3. SingleManipulator 생성 (ee_link 경로는 필요시 수정)
robot = my_world.scene.add(
    SingleManipulator(
        prim_path=robot_prim_path,
        name="e0509_arm",
        end_effector_prim_path="/World/e0509/link_6",
        gripper=None,
    )
)

print("========== DIAGNOSTICS ==========")
print("SingleManipulator dir():", dir(robot))

# 4. World reset + step 후 initialize
my_world.reset()
for _ in range(5):
    my_world.step(render=True)

robot.initialize()
dof_names = robot.dof_names
print("robot.dof_names after initialize:", dof_names)

if dof_names is None:
    print("ERROR: dof_names is None. Check e0509.usd articulation/joint settings.")
    simulation_app.close()
    raise SystemExit

num_dof = len(dof_names)
print("num_dof:", num_dof)

# 5. 초기 joint 값
init_joint_array = np.zeros(num_dof, dtype=float)
robot.set_joint_positions(init_joint_array)
print("robot.get_joint_positions() after init:", robot.get_joint_positions())


# 6. ROS2 Node 정의: /joint_input sub → 로봇 joint set, /joint_pos pub
class JointBridgeNode(Node):
    def __init__(self, robot, num_dof):
        super().__init__("e0509_joint_bridge")
        self.robot = robot
        self.num_dof = num_dof
        self.current_cmd = np.zeros(num_dof, dtype=float)

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/joint_input",
            self.joint_input_callback,
            10,
        )
        self.pub = self.create_publisher(
            Float32MultiArray,
            "/joint_pos",
            10,
        )
        self.get_logger().info(
            f"JointBridgeNode started. Subscribing /joint_input, publishing /joint_pos. DOF={num_dof}"
        )

    def joint_input_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < self.num_dof:
            self.get_logger().warn(
                f"Received {len(data)} elements, expected {self.num_dof}. Padding with zeros."
            )
            data = data + [0.0] * (self.num_dof - len(data))
        elif len(data) > self.num_dof:
            self.get_logger().warn(
                f"Received {len(data)} elements, expected {self.num_dof}. Truncating."
            )
            data = data[:self.num_dof]

        self.current_cmd = np.array(data, dtype=float)
        self.get_logger().info(f"[CB] /joint_input: {self.current_cmd}")

        # 로봇 관절에 바로 적용
        self.robot.set_joint_positions(self.current_cmd)

        # 바로 적용된 현재 관절 상태를 찍고 publish
        current = self.robot.get_joint_positions()
        self.get_logger().info(f"[CB] robot joints after set: {current}")

        out = Float32MultiArray()
        out.data = current.tolist()
        self.pub.publish(out)
        self.get_logger().info(f"[CB] published /joint_pos: {out.data}")

    def publish_joint_state(self):
        positions = self.robot.get_joint_positions()
        msg = Float32MultiArray()
        msg.data = positions.tolist()
        self.pub.publish(msg)


# 7. ROS2 init 및 Node 생성
rclpy.init()
joint_node = JointBridgeNode(robot=robot, num_dof=num_dof)

print("Ready. /joint_input → set joints, /joint_pos → current joints. Press Ctrl+C to stop.")

# 8. 메인 루프: Isaac Sim step + ROS2 spin + joint publish
try:
    frame = 0
    while simulation_app.is_running():
        my_world.step(render=True)
        frame += 1

        # 현재 joint 상태를 /joint_pos로 publish
        joint_node.publish_joint_state()

        # ROS2 콜백 처리 (joint_input 수신)
        rclpy.spin_once(joint_node, timeout_sec=0.0)

      

except KeyboardInterrupt:
    print("KeyboardInterrupt, shutting down.")

finally:
    joint_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
    print("Simulation closed.")

