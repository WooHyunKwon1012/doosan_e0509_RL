from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Standard library imports
import sys
import os
from datetime import datetime

# Third-party imports
import numpy as np

# Isaac Sim imports
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.robot.surface_gripper import GripperView
from usd.schema.isaac import robot_schema
from isaacsim.core.utils.types import ArticulationAction

# USD/PhysX imports
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, PhysxSchema, Sdf

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

# ROS2 bridge 활성화
enable_extension("isaacsim.ros2.bridge")

# 1. World + 환경 + 로봇 로드 (numpy backend for rigid body)
my_world = World(
    stage_units_in_meters=1.0,
    backend="numpy"  # RigidBody works with numpy backend
)
stage = my_world.stage

# 로그 파일 설정 (Isaac Sim 초기화 후)
import sys
from datetime import datetime
# 로그 파일 설정 (Isaac Sim 초기화 후)
log_dir = "/home/woo/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env_pick_place/logs"
os.makedirs(log_dir, exist_ok=True)
log_file_path = os.path.join(log_dir, f"isaac_sim_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
log_file = open(log_file_path, 'w', buffering=1)

class Tee:
    def __init__(self, *files):
        self.files = files
    def write(self, obj):
        for f in self.files:
            f.write(obj)
            f.flush()
    def flush(self):
        for f in self.files:
            f.flush()

sys.stdout = Tee(sys.stdout, log_file)
sys.stderr = Tee(sys.stderr, log_file)

print(f"\n{'='*60}")
print(f"Logging to: {log_file_path}")
print(f"{'='*60}\n")

# 환경 로드 (로봇 없는 버전)
room_usd_path = "/home/woo/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env_pick_place/isaac_env/room_without_e0509.usd"
add_reference_to_stage(usd_path=room_usd_path, prim_path="/World")

# 로봇 USD 추가 (e0509_with_gripper와 동일한 파일 사용)
robot_usd_path = "/home/woo/ros2_ws/src/doosan-robot2/isaacsim_connect/e0509_with_env_pick_place/isaac_env/e0509_model.usd"
robot_prim_path = "/World/e0509_model"
robot_spawn_position = (1.7000000283122063, -4.750000071525574, 0.78)

add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)

# Gripper 체크 (상세)
print("\n========== Checking Gripper (Detailed) ==========")
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

# 그리퍼 Friction 설정 (런타임에 추가)
print("\n========== Setting Gripper Friction ==========")
gripper_finger_paths = [
    "/World/e0509_model/e0509/gripper/gripper/rh_l1",
    "/World/e0509_model/e0509/gripper/gripper/rh_r1",
    "/World/e0509_model/e0509/gripper/gripper/rh_l2", 
    "/World/e0509_model/e0509/gripper/gripper/rh_r2",
]

for finger_path in gripper_finger_paths:
    finger_prim = stage.GetPrimAtPath(finger_path)
    if finger_prim.IsValid():
        print(f"\nSetting friction for: {finger_path}")
        
        # Physics Material 설정 (attribute로 직접 설정)
        try:
            # Collision이 있는지 확인
            if finger_prim.HasAPI(UsdPhysics.CollisionAPI):
                # Friction을 prim attribute로 직접 설정
                if not finger_prim.GetAttribute("physics:dynamicFriction"):
                    finger_prim.CreateAttribute("physics:dynamicFriction", Sdf.ValueTypeNames.Float).Set(1.5)
                    finger_prim.CreateAttribute("physics:staticFriction", Sdf.ValueTypeNames.Float).Set(1.5)
                    finger_prim.CreateAttribute("physics:restitution", Sdf.ValueTypeNames.Float).Set(0.0)
                    print(f"  ✓ Friction set: static=1.5, dynamic=1.5")
                else:
                    print(f"  ℹ️  Friction already set (from USD)")
            else:
                print(f"  ⚠️  No CollisionAPI on finger")
        except Exception as e:
            print(f"  ⚠️  Warning: Could not set friction: {e}")
    else:
        print(f"  ✗ Finger not found: {finger_path}")

# 로봇 위치 설정
robot_prim = stage.GetPrimAtPath(robot_prim_path)
xformable = UsdGeom.Xformable(robot_prim)
xformable.ClearXformOpOrder()
translate_op = xformable.AddTranslateOp()
translate_op.Set(Gf.Vec3d(robot_spawn_position[0], robot_spawn_position[1], robot_spawn_position[2]))

# 2. Collision 활성화 체크
print("\n========== Checking Collision Settings ==========")

# 환경 내 물체들 + 로봇 확인
sample_paths = [
    "/World/e0509_model",  # 로봇 ✅ 추가
    "/World/ikea/nova_carter/move_props",  # teddy_bear 등 픽업 대상
    "/World/ikea",  # ikea 관련
    "/World/sm_warehouse_a01_h10m_cornerinmirror_01",  # warehouse
]

for path in sample_paths:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        print(f"\n{path}:")
        # 하위 prims 중 collision 있는지 확인
        collision_count = 0
        for child_prim in Usd.PrimRange(prim):
            if child_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_count += 1
                if collision_count <= 3:  # 처음 3개만 출력
                    print(f"  Collision found: {child_prim.GetPath()}")
        print(f"  Total collision prims: {collision_count}")

# Teddy bear USD 속성 상세 분석 (World.reset 전!)
print("\n========== Detailed Teddy Bear USD Analysis (Before Reset) ==========")
teddy_path = "/World/ikea/nova_carter/move_props/teddy_bear"
teddy_bear_path = f"{teddy_path}/geometry/bear"
teddy_prim = stage.GetPrimAtPath(teddy_path)
teddy_bear_prim = stage.GetPrimAtPath(teddy_bear_path)

if teddy_prim.IsValid():
    print(f"\nTeddy bear prim: {teddy_path}")
    print(f"  Valid: True")
    print(f"  Type: {teddy_prim.GetTypeName()}")
    print(f"  Has RigidBodyAPI: {teddy_prim.HasAPI(UsdPhysics.RigidBodyAPI)}")
    print(f"  Has MassAPI: {teddy_prim.HasAPI(UsdPhysics.MassAPI)}")
    print(f"  Has CollisionAPI: {teddy_prim.HasAPI(UsdPhysics.CollisionAPI)}")
    
    # RigidBody 속성 읽기
    if teddy_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_api = UsdPhysics.RigidBodyAPI(teddy_prim)
        kinematic_attr = rigid_api.GetKinematicEnabledAttr()
        enabled_attr = rigid_api.GetRigidBodyEnabledAttr()
        print(f"  RigidBody kinematic: {kinematic_attr.Get() if kinematic_attr else 'N/A'}")
        print(f"  RigidBody enabled: {enabled_attr.Get() if enabled_attr else 'N/A'}")
    
    # Mass 속성 읽기
    if teddy_prim.HasAPI(UsdPhysics.MassAPI):
        mass_api = UsdPhysics.MassAPI(teddy_prim)
        mass_attr = mass_api.GetMassAttr()
        print(f"  Mass: {mass_attr.Get() if mass_attr else 'N/A'} kg")
    
    # 모든 API 리스트 출력
    print(f"  All applied APIs: {teddy_prim.GetAppliedSchemas()}")

if teddy_bear_prim.IsValid():
    print(f"\nTeddy bear/geometry/bear prim: {teddy_bear_path}")
    print(f"  Valid: True")
    print(f"  Type: {teddy_bear_prim.GetTypeName()}")
    print(f"  Has RigidBodyAPI: {teddy_bear_prim.HasAPI(UsdPhysics.RigidBodyAPI)}")
    print(f"  Has MassAPI: {teddy_bear_prim.HasAPI(UsdPhysics.MassAPI)}")
    print(f"  Has CollisionAPI: {teddy_bear_prim.HasAPI(UsdPhysics.CollisionAPI)}")
    print(f"  Has MeshCollisionAPI: {teddy_bear_prim.HasAPI(UsdPhysics.MeshCollisionAPI)}")
    
    # Collision approximation 읽기
    if teddy_bear_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        mesh_coll_api = UsdPhysics.MeshCollisionAPI(teddy_bear_prim)
        approx_attr = mesh_coll_api.GetApproximationAttr()
        print(f"  Collision approximation: {approx_attr.Get() if approx_attr else 'N/A'}")
    
    print(f"  All applied APIs: {teddy_bear_prim.GetAppliedSchemas()}")

# 다른 정상 작동하는 물체와 비교
print("\n========== Comparing with Working Object (Cube) ==========")
cube_path = "/World/ikea/nova_carter/move_props/Cube"
cube_prim = stage.GetPrimAtPath(cube_path)
if cube_prim.IsValid():
    print(f"Cube prim: {cube_path}")
    print(f"  Type: {cube_prim.GetTypeName()}")
    print(f"  Has RigidBodyAPI: {cube_prim.HasAPI(UsdPhysics.RigidBodyAPI)}")
    print(f"  Has MassAPI: {cube_prim.HasAPI(UsdPhysics.MassAPI)}")
    print(f"  Has CollisionAPI: {cube_prim.HasAPI(UsdPhysics.CollisionAPI)}")
    print(f"  All applied APIs: {cube_prim.GetAppliedSchemas()}")

# Physics Scene 설정 (GPU Dynamics는 RigidBody에 필요 없음)
print("\n========== Physics Scene Setup ==========")
physics_scene_path = "/physicsScene"
physics_scene_prim = stage.GetPrimAtPath(physics_scene_path)

if not physics_scene_prim.IsValid():
    print(f"  Creating physics scene...")
    physics_scene_prim = UsdPhysics.Scene.Define(stage, physics_scene_path).GetPrim()

if physics_scene_prim.IsValid():
    print(f"  ✓ Physics scene ready at {physics_scene_path}")
else:
    print(f"  ERROR: Could not create/access physics scene")

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

# 5. 초기 joint 값 (numpy array)
init_joint_array = np.zeros(num_dof, dtype=np.float32)
robot.set_joint_positions(init_joint_array)
print("robot.get_joint_positions() after init:", robot.get_joint_positions())

# Teddy bear PhysxDeformableBodyAPI 활성화 + RigidBodyAPI 추가 (World reset 후!)
print("\n========== Activating PhysxDeformableBodyAPI for Teddy Bear ==========")

teddy_path = "/World/ikea/nova_carter/move_props/teddy_bear"
teddy_bear_path = f"{teddy_path}/geometry/bear"
teddy_prim = stage.GetPrimAtPath(teddy_path)
teddy_bear_prim = stage.GetPrimAtPath(teddy_bear_path)

# DeformableBodyAPI를 제거하고 RigidBodyAPI로 변경
# (DeformableBody는 Fixed Joint와 호환성 문제가 있음)
print("\n  Converting Teddy Bear from Deformable to Rigid Body...")

# 자식(bear)에서 DeformableBodyAPI 제거
if teddy_bear_prim.IsValid() and teddy_bear_prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
    print(f"  Removing PhysxDeformableBodyAPI from {teddy_bear_path}...")
    teddy_bear_prim.RemoveAPI(PhysxSchema.PhysxDeformableBodyAPI)
    teddy_bear_prim.RemoveAPI(PhysxSchema.PhysxDeformableAPI)
    print(f"  ✓ DeformableBodyAPI removed")

# 🔥 중요: RigidBodyAPI를 geometry/bear에 직접 추가!
# Fixed Joint는 collision이 있는 prim에 연결해야 제대로 작동함
if teddy_bear_prim.IsValid() and not teddy_bear_prim.HasAPI(UsdPhysics.RigidBodyAPI):
    print(f"  Adding RigidBodyAPI to geometry/bear (where collision happens)...")
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(teddy_bear_prim)
    rigid_api.CreateRigidBodyEnabledAttr().Set(True)
    print(f"  ✓ RigidBodyAPI added to {teddy_bear_path}")
    
    # Mass API도 추가 (geometry mesh에)
    if not teddy_bear_prim.HasAPI(UsdPhysics.MassAPI):
        mass_api = UsdPhysics.MassAPI.Apply(teddy_bear_prim)
        mass_api.GetMassAttr().Set(0.1)  # 100g
        print(f"  ✓ MassAPI added to geometry mesh with mass=0.1kg")
    
    # 🔥 CollisionAPI 추가 (geometry mesh에)
    if not teddy_bear_prim.HasAPI(UsdPhysics.CollisionAPI):
        collision_api = UsdPhysics.CollisionAPI.Apply(teddy_bear_prim)
        print(f"  ✓ CollisionAPI added to geometry mesh")
    
    # 🔥 MeshCollisionAPI 적용 (geometry mesh에)
    print(f"  Setting collision mesh on geometry mesh: {teddy_bear_path}")
    if not teddy_bear_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(teddy_bear_prim)
        # convexDecomposition = 가장 정확한 collision mesh
        mesh_collision_api.CreateApproximationAttr().Set("convexDecomposition")
        print(f"  ✓ MeshCollisionAPI added with convexDecomposition")
    
    # 🔥 Contact/Rest Offset 설정 (geometry mesh에) - 증가!
    if not teddy_bear_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
        physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(teddy_bear_prim)
        physx_collision_api.CreateContactOffsetAttr().Set(0.02)  # 0.005 → 0.02로 증가
        physx_collision_api.CreateRestOffsetAttr().Set(0.0)
        print(f"  ✓ Contact offset: 0.02m (increased for better grip), Rest offset: 0.0m")
    
    # 🔥 마찰 계수 설정 (geometry mesh에)
    print(f"  Setting friction properties on geometry mesh...")
    try:
        teddy_bear_prim.CreateAttribute("physics:dynamicFriction", Sdf.ValueTypeNames.Float).Set(1.5)
        teddy_bear_prim.CreateAttribute("physics:staticFriction", Sdf.ValueTypeNames.Float).Set(1.5)
        teddy_bear_prim.CreateAttribute("physics:restitution", Sdf.ValueTypeNames.Float).Set(0.0)
        print(f"  ✓ Friction: static=1.5, dynamic=1.5, restitution=0.0")
    except Exception as e:
        print(f"  ⚠️  Warning: Could not set friction: {e}")

print(f"  ✓ Teddy Bear geometry/bear converted to RigidBody (no longer deformable)")

# RigidBody 설정 확인 (geometry/bear)
if teddy_bear_prim.IsValid() and teddy_bear_prim.HasAPI(UsdPhysics.RigidBodyAPI):
    print(f"\nTeddy bear geometry/bear is now a RigidBody at: {teddy_bear_path}")
    print(f"  ✓ RigidBodyAPI present on geometry mesh")
    print(f"  ✓ CollisionAPI present on geometry mesh")
    print(f"  ✓ Can be grasped with Fixed Joint (connected to geometry mesh)")
    
    # Mass 확인
    if teddy_bear_prim.HasAPI(UsdPhysics.MassAPI):
        mass_api = UsdPhysics.MassAPI(teddy_bear_prim)
        mass_attr = mass_api.GetMassAttr()
        current_mass = mass_attr.Get() if mass_attr else None
        print(f"  ✓ Mass = {current_mass} kg")
    
    print("\n  RigidBody setup complete on geometry mesh!")
else:
    print(f"  ERROR: Teddy bear geometry/bear RigidBody setup failed!")

# 몇 스텝 실행하여 physics 안정화
print("\n  Running physics steps to stabilize...")
for i in range(10):
    my_world.step(render=True)
print("  Physics stabilization complete!")

# Surface Gripper 설정 (Isaac Sim 5.1)
print("\n========== Setting up Surface Gripper ==========")

class SurfaceGripperWrapper:
    """
    GripperView를 래핑하여 간단한 인터페이스 제공
    Isaac Sim 5.1 - USD에 이미 생성된 Surface Gripper 사용
    ⭐ USD 파일에 prim이 이미 존재함 - 코드에서 생성하지 않음
    """
    def __init__(self, end_effector_path, world, grip_threshold=0.1):
        self.end_effector_path = end_effector_path
        self.world = world
        self.is_closed = False
        self.grip_threshold = grip_threshold
        # ⭐ USD 파일에 이미 존재하는 SurfaceGripper prim 경로
        self.gripper_prim_path = "/World/e0509_model/SurfaceGripper"
        self.gripper_view = None
        
        print(f"\n  ✓ SurfaceGripperWrapper created")
        print(f"    Using existing USD prim at: {self.gripper_prim_path}")
        print(f"    End effector: {end_effector_path}")
        print(f"    ⚠️  GripperView will be initialized in initialize() (after world.reset())")
    
    def initialize(self):
        """
        World reset 후 USD에 있는 SurfaceGripper prim에 GripperView 연결
        ⭐ Prim은 USD에 이미 존재 - attachment points도 USD에서 설정됨
        """
        try:
            stage = self.world.stage
            
            print(f"\n  🔧 Initializing Surface Gripper (after world.reset())...")
            
            # ⭐ USD에 이미 존재하는 SurfaceGripper prim 확인
            print(f"    Checking existing Surface Gripper prim in USD...")
            gripper_prim = stage.GetPrimAtPath(self.gripper_prim_path)
            
            if not gripper_prim.IsValid():
                raise RuntimeError(f"SurfaceGripper prim NOT FOUND at {self.gripper_prim_path}. Check USD file!")
            
            print(f"    ✓ Found existing prim: {self.gripper_prim_path}")
            
            # ⭐ USD에 설정된 Attachment points 확인 (USD에서 finger links로 설정됨)
            attachment_points_rel = gripper_prim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name)
            if attachment_points_rel:
                targets = attachment_points_rel.GetTargets()
                print(f"    ✓ Attachment points from USD ({len(targets)} finger links):")
                for i, target in enumerate(targets, 1):
                    print(f"      {i}. {target}")
            else:
                print(f"    ⚠️  No attachment points found in USD!")
            
            # ⭐ GripperView 생성 (USD prim에 연결)
            print(f"    Creating GripperView (connecting to USD prim)...")
            self.gripper_view = GripperView(paths=self.gripper_prim_path)
            
            # ⭐ Surface Gripper 속성 설정 (force limit 크게 증가!)
            self.gripper_view.set_surface_gripper_properties(
                max_grip_distance=[self.grip_threshold],  # 잡을 수 있는 최대 거리
                coaxial_force_limit=[1e4],  # 축 방향 힘 한계 (N) - 크게 증가!
                shear_force_limit=[1e4],   # 전단 힘 한계 (N) - 크게 증가!
                retry_interval=[0.1]         # 재시도 간격 (초) - 빠르게!
            )
            
            print(f"\n  ✓ GripperView initialized")
            print(f"    Path: {self.gripper_prim_path}")
            print(f"    Max grip distance: {self.grip_threshold}m ({self.grip_threshold*100:.1f}cm)")
            print(f"    Coaxial force limit: 1e4 N (10000 N)")
            print(f"    Shear force limit: 1e4 N (10000 N)")
            print(f"    Retry interval: 0.1 s")
            
        except Exception as e:
            print(f"\n  ❌ ERROR initializing Surface Gripper: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def close(self):
        """물체를 잡음"""
        if self.is_closed:
            print("  [GRIPPER] Already closed")
            return False
        
        print(f"  [GRIPPER] Attempting to grasp...")
        
        # 🔍 디버그: Prim 존재 여부 확인
        stage = self.world.stage
        gripper_prim = stage.GetPrimAtPath(self.gripper_prim_path)
        print(f"  [DEBUG] SurfaceGripper prim valid: {gripper_prim.IsValid()}")
        if not gripper_prim.IsValid():
            print(f"  [ERROR] ❌ SurfaceGripper prim NOT FOUND at {self.gripper_prim_path}")
            print(f"  [ERROR] This is the source of 'Gripper not found' error!")
            return False
        
        # GripperView: 양수 값으로 close (0.0 ~ 1.0)
        # 0.5 이상이면 close 명령으로 인식
        try:
            self.gripper_view.apply_gripper_action([1.0])
            self.is_closed = True
            print(f"  [GRIPPER] ✓ Close command sent")
            print(f"  [GRIPPER] ℹ️  Will grasp objects within {self.grip_threshold*100:.1f}cm")
            return True
        except Exception as e:
            print(f"  [ERROR] ❌ apply_gripper_action failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def open(self):
        """물체를 놓음"""
        if not self.is_closed:
            print("  [GRIPPER] Already open")
            return False
        
        print(f"  [GRIPPER] Releasing object...")
        # GripperView: 음수 값으로 open (0.0 ~ -1.0)
        # -0.5 이하면 open 명령으로 인식
        self.gripper_view.apply_gripper_action([-1.0])
        self.is_closed = False
        print(f"  [GRIPPER] ✓ Open command sent")
        return True
    
    def update(self):
        """매 프레임 호출 - Attach 상태만 확인"""
        # 그립 상태 확인 - 매 30프레임마다 체크 (조인트 생성 여부 확인)
        if self.is_closed and self.gripper_view is not None:
            try:
                if not hasattr(self, '_grip_check_count'):
                    self._grip_check_count = 0
                    self._last_gripped_state = None
                
                self._grip_check_count += 1
                
                # 30프레임마다 상태 체크
                if self._grip_check_count % 30 == 0:
                    status = self.gripper_view.get_surface_gripper_status()[0]
                    gripped_objects = self.gripper_view.get_gripped_objects()
                    
                    # 상태 변경 시에만 출력
                    current_state = (status, len(gripped_objects[0]) if gripped_objects else 0)
                    if current_state != self._last_gripped_state:
                        if gripped_objects and len(gripped_objects[0]) > 0:
                            print(f"  [GRIPPER] 🎯 ATTACHED! Gripped objects: {gripped_objects[0]}")
                            print(f"  [GRIPPER] Status: {status} (1=closed/attached)")
                        else:
                            print(f"  [GRIPPER] ⚠️  NOT ATTACHED (status: {status}, objects: 0)")
                            print(f"  [GRIPPER] 💡 Check: force limits, contactOffset, EE position")
                        self._last_gripped_state = current_state
            except Exception as e:
                print(f"  [GRIPPER] Error checking status: {e}")

# 테디베어 구조 디버깅
print("\n========== Teddy Bear Structure Debug ==========")
teddy_parent = stage.GetPrimAtPath("/World/ikea/nova_carter/move_props/teddy_bear")
teddy_geo = stage.GetPrimAtPath("/World/ikea/nova_carter/move_props/teddy_bear/geometry")
teddy_bear = stage.GetPrimAtPath("/World/ikea/nova_carter/move_props/teddy_bear/geometry/bear")

for path, prim in [
    ("teddy_bear (parent)", teddy_parent),
    ("teddy_bear/geometry", teddy_geo),
    ("teddy_bear/geometry/bear", teddy_bear)
]:
    if prim.IsValid():
        print(f"\n{path}:")
        print(f"  Type: {prim.GetTypeName()}")
        print(f"  Has RigidBodyAPI: {prim.HasAPI(UsdPhysics.RigidBodyAPI)}")
        print(f"  Has CollisionAPI: {prim.HasAPI(UsdPhysics.CollisionAPI)}")
        print(f"  Applied APIs: {prim.GetAppliedSchemas()}")

# 🔍 그리퍼 구조 탐색 (디버그용)
print("\n========== Inspecting Gripper Structure ==========")

def inspect_prim_recursive(prim, depth=0, max_depth=5):
    """재귀적으로 prim 구조 탐색"""
    if depth > max_depth:
        return
    
    indent = "  " * depth
    prim_path = str(prim.GetPath())
    prim_type = prim.GetTypeName()
    
    # 중요한 정보 출력
    has_rigid = prim.HasAPI(UsdPhysics.RigidBodyAPI)
    has_collision = prim.HasAPI(UsdPhysics.CollisionAPI)
    
    # rh_ 또는 joint 관련 prim만 상세 출력
    if "rh_" in prim_path or "joint" in prim_path or "hand" in prim_path:
        print(f"{indent}├─ {prim.GetName()} (Type: {prim_type})")
        print(f"{indent}│  Path: {prim_path}")
        print(f"{indent}│  RigidBody: {has_rigid}, Collision: {has_collision}")
        if has_rigid or has_collision:
            print(f"{indent}│  ⭐ PHYSICS ENABLED")
    
    # 자식 prim 탐색
    for child in prim.GetChildren():
        inspect_prim_recursive(child, depth + 1, max_depth)

# 그리퍼 루트부터 탐색
gripper_root_path = "/World/e0509_model/e0509/gripper"
gripper_root = stage.GetPrimAtPath(gripper_root_path)
if gripper_root.IsValid():
    print(f"\n🔍 Exploring gripper structure from: {gripper_root_path}")
    inspect_prim_recursive(gripper_root, depth=0, max_depth=6)
else:
    print(f"⚠️  Gripper root not found: {gripper_root_path}")

# Surface Gripper 생성
print("\n========== Creating Surface Gripper ==========")

# End effector path 설정 (그리퍼 중심의 end_effector_link 사용)
# SingleManipulator는 link_6을 사용하지만, Surface Gripper는 실제 그리퍼 중심을 사용
end_effector_path = "/World/e0509_model/e0509/gripper/gripper/hand/rh_p12_rn_r2"
print(f"  Using end_effector for Surface Gripper: {end_effector_path}")
print(f"  ℹ️  This is the actual gripper center (between fingers)")

# 🔥 grip_threshold 설정 (20cm로 증가 - teddy bear가 13-14cm 거리에 있음)
grip_threshold = 0.10  # 20cm = 0.20m (현재 거리: ~0.13m)

gripper = SurfaceGripperWrapper(
    end_effector_path=end_effector_path,
    world=my_world,
    grip_threshold=grip_threshold
)
print(f"\n  ✓ Surface Gripper setup complete")
print(f"  ℹ️  Gripper will automatically detect and grasp nearby objects")
print(f"  ℹ️  Move link_6 close to the object and send gripper command")

# World reset 후 gripper 초기화
print("\n  Initializing gripper after world reset...")
gripper.initialize()
print("  ✓ Gripper initialization complete!")

# 6. ROS2 Node 정의: /joint_input sub → 로봇 joint set, /joint_pos pub
class JointBridgeNode(Node):
    def __init__(self, robot, num_dof, gripper):
        super().__init__("e0509_joint_bridge")
        self.robot = robot
        self.num_dof = num_dof
        self.gripper = gripper
        self.current_cmd = np.zeros(num_dof, dtype=np.float32)

        # Joint input subscription
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/joint_input",
            self.joint_input_callback,
            10,
        )
        
        # Joint position publisher
        self.pub = self.create_publisher(
            Float32MultiArray,
            "/joint_pos",
            10,
        )
        
        # Gripper command subscription (0 = open, 1 = close)
        self.gripper_sub = self.create_subscription(
            Int32,
            "/gripper_command",
            self.gripper_command_callback,
            10,
        )
        
        self.get_logger().info(
            f"JointBridgeNode started. Subscribing /joint_input, /gripper_command, publishing /joint_pos. DOF={num_dof}"
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

        # numpy array로 변환
        self.current_cmd = np.array(data, dtype=np.float32)
        self.get_logger().info(f"[CB] /joint_input: {data}")

        # 로봇 관절에 바로 적용
        self.robot.set_joint_positions(self.current_cmd)

        # 바로 적용된 현재 관절 상태를 찍고 publish
        current = self.robot.get_joint_positions()
        self.get_logger().info(f"[CB] robot joints after set: {current}")

        out = Float32MultiArray()
        out.data = current.tolist() if hasattr(current, 'tolist') else list(current)
        self.pub.publish(out)
        self.get_logger().info(f"[CB] published /joint_pos: {out.data}")

    def publish_joint_state(self):
        positions = self.robot.get_joint_positions()
        msg = Float32MultiArray()
        msg.data = positions.tolist() if hasattr(positions, 'tolist') else list(positions)
        self.pub.publish(msg)
    
    def gripper_command_callback(self, msg: Int32):
        """
        Gripper command callback
        0 = open (release object)
        1 = close (grasp object)
        """
        command = msg.data
        self.get_logger().info(f"[GRIPPER] Received command: {command}")
        
        if command == 1:
            # Close gripper (grasp)
            success = self.gripper.close()
            if success:
                self.get_logger().info("[GRIPPER] ✓ Closed - Object grasped!")
            else:
                self.get_logger().warn("[GRIPPER] Failed to close")
        elif command == 0:
            # Open gripper (release)
            success = self.gripper.open()
            if success:
                self.get_logger().info("[GRIPPER] ✓ Opened - Object released!")
            else:
                self.get_logger().warn("[GRIPPER] Failed to open")
        else:
            self.get_logger().warn(f"[GRIPPER] Unknown command: {command}")


# 7. ROS2 init 및 Node 생성
rclpy.init()
joint_node = JointBridgeNode(robot=robot, num_dof=num_dof, gripper=gripper)

print("Ready. /joint_input → set joints, /joint_pos → current joints, /gripper_command → grasp/release. Press Ctrl+C to stop.")
print("\nGripper Command Usage:")
print("  ros2 topic pub /gripper_command std_msgs/msg/Int32 '{data: 1}'  # Close (grasp)")
print("  ros2 topic pub /gripper_command std_msgs/msg/Int32 '{data: 0}'  # Open (release)")

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

        # ROS2 콜백 처리 (joint_input, gripper_command 수신)
        rclpy.spin_once(joint_node, timeout_sec=0.0)
        
        # Gripper update (physics 상호작용)
        gripper.update()


except KeyboardInterrupt:
    print("KeyboardInterrupt, shutting down.")

finally:
    joint_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
    print("Simulation closed.")
    log_file.close()
    print(f"\n==== Log saved to: {log_file_path} ====")

