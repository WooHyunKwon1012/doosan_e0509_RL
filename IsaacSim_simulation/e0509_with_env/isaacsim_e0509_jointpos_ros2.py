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

# 1. World + 환경 + 로봇 로드
my_world = World(stage_units_in_meters=1.0)
stage = my_world.stage

# 환경 로드 (로봇 없는 버전)
room_usd_path = "/home/woo/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env/isaac_env/room_without_e0509.usd"
add_reference_to_stage(usd_path=room_usd_path, prim_path="/World")

# 로봇 USD 추가 (e0509_with_gripper와 동일한 파일 사용)
robot_usd_path = "/home/woo/Downloads/source/3d/Jiwoo_e0509_gripper/e0509_model.usd"
robot_prim_path = "/World/e0509_model"
robot_spawn_position = (1.9000000283122063, -4.800000071525574, 0.78)

add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)

# Gripper 체크 (상세)
print("\n========== Checking Gripper (Detailed) ==========")
from pxr import Usd
print(f"Current working directory: {os.getcwd()}")
print(f"Robot USD path: {robot_usd_path}")

gripper_prim = stage.GetPrimAtPath("/World/e0509_model/e0509/gripper")
print(f"\nGripper prim: {gripper_prim.GetPath()}")
print(f"  Valid: {gripper_prim.IsValid()}")
print(f"  Type: {gripper_prim.GetTypeName()}")
print(f"  Has references: {gripper_prim.HasAuthoredReferences()}")

if gripper_prim.HasAuthoredReferences():
    prim_stack = gripper_prim.GetPrimStack()
    print(f"  Prim stack layers: {len(prim_stack)}")
    for i, spec in enumerate(prim_stack[:3]):
        print(f"    Layer {i}: {spec.layer.identifier if spec.layer else 'None'}")

children = list(gripper_prim.GetChildren())
print(f"  Children count: {len(children)}")
if len(children) > 0:
    for child in children[:3]:
        print(f"    Child: {child.GetPath()} (Type: {child.GetTypeName()})")
        child_children = list(child.GetChildren())
        print(f"      Sub-children: {len(child_children)}")
        if len(child_children) > 0:
            print(f"      First sub-child: {child_children[0].GetPath()}")
else:
    print("  WARNING: Gripper has no children! Reference might be broken.")

# 로봇 위치 설정
from pxr import UsdGeom, Gf
robot_prim = stage.GetPrimAtPath(robot_prim_path)
xformable = UsdGeom.Xformable(robot_prim)
xformable.ClearXformOpOrder()
translate_op = xformable.AddTranslateOp()
translate_op.Set(Gf.Vec3d(robot_spawn_position[0], robot_spawn_position[1], robot_spawn_position[2]))

# 2. PhysicsScene 세팅
from pxr import UsdPhysics, PhysxSchema
physics_scene_path = "/World/physics"
if not stage.GetPrimAtPath(physics_scene_path):
    scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
else:
    scene = UsdPhysics.Scene(stage.GetPrimAtPath(physics_scene_path))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_path))

# 3. SingleManipulator 생성
robot = my_world.scene.add(
    SingleManipulator(
        prim_path=robot_prim_path,
        name="e0509_arm",
        end_effector_prim_path="/World/e0509_model/e0509/link_6",
        gripper=None,
        position=np.array(robot_spawn_position),
    )
)

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
import time

publish_rate = 30  # Hz로 제한 (기존 60Hz에서 30Hz로 감소)
publish_interval = 1.0 / publish_rate
last_publish_time = time.time()

try:
    frame = 0
    while simulation_app.is_running():
        my_world.step(render=True)
        frame += 1

        current_time = time.time()
        
        # 주기적으로만 발행 (30Hz)
        if current_time - last_publish_time >= publish_interval:
            joint_node.publish_joint_state()
            last_publish_time = current_time

        # ROS2 콜백 처리 (joint_input 수신)
        rclpy.spin_once(joint_node, timeout_sec=0.0)

      

except KeyboardInterrupt:
    print("KeyboardInterrupt, shutting down.")

finally:
    joint_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
    print("Simulation closed.")

