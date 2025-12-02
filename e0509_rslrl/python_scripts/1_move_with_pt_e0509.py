import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom, UsdLux, Gf, UsdPhysics, PhysxSchema

# dynamic_control import
try:
    from omni.isaac.dynamic_control import _dynamic_control
except Exception:
    _dynamic_control = None

# ------------------------------
# 1. ActorCritic 네트워크
# ------------------------------
class RslRlActorCritic(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int):
        super().__init__()
        self.actor_0 = nn.Linear(obs_dim, 256)
        self.actor_2 = nn.Linear(256, 128)
        self.actor_4 = nn.Linear(128, 64)
        self.actor_6 = nn.Linear(64, action_dim)

        self.critic_0 = nn.Linear(obs_dim, 256)
        self.critic_2 = nn.Linear(256, 128)
        self.critic_4 = nn.Linear(128, 64)
        self.critic_6 = nn.Linear(64, 1)

        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, obs: torch.Tensor):
        x_a = F.elu(self.actor_0(obs))
        x_a = F.elu(self.actor_2(x_a))
        x_a = F.elu(self.actor_4(x_a))
        mean = self.actor_6(x_a)

        x_v = F.elu(self.critic_0(obs))
        x_v = F.elu(self.critic_2(x_v))
        x_v = F.elu(self.critic_4(x_v))
        value = self.critic_6(x_v).squeeze(-1)

        std = self.log_std.exp().expand_as(mean)
        return mean, std, value

# ------------------------------
# 2. 체크포인트 로드
# ------------------------------
ckpt_path = "/isaac-sim/e0509_pt/model_300.pt"
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
ckpt = torch.load(ckpt_path, map_location="cpu")
msd = ckpt["model_state_dict"]

obs_dim = 27
action_dim = 6
policy_model = RslRlActorCritic(obs_dim, action_dim).to(device)

# state_dict key mapping
state_dict = policy_model.state_dict()
mapping = {}
for k in msd.keys():
    if k.startswith("actor."):
        mapping[k.replace("actor.", "actor_", 1)] = k
    elif k.startswith("critic."):
        mapping[k.replace("critic.", "critic_", 1)] = k
    elif k == "std":
        mapping["log_std"] = k
for new_k, old_k in mapping.items():
    if new_k in state_dict:
        state_dict[new_k] = msd[old_k]
policy_model.load_state_dict(state_dict, strict=False)
policy_model.eval()
print("Loaded policy from:", ckpt_path)

# ------------------------------
# 3. World / Stage
# ------------------------------
world = World(stage_units_in_meters=1.0)
world.scene.clear()
world.reset()
stage = world.stage

# Physics Scene
physics_scene_path = "/World/physics"
scene = UsdPhysics.Scene.Define(stage, physics_scene_path) if not stage.GetPrimAtPath(physics_scene_path) else UsdPhysics.Scene(stage.GetPrimAtPath(physics_scene_path))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0,0.0,-1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_path))

# DomeLight
light_prim = stage.DefinePrim("/World/light", "DomeLight")
light = UsdLux.DomeLight(light_prim)
light.GetIntensityAttr().Set(2500.0)
light.GetColorAttr().Set((0.75,0.75,0.75))

# Table
assets_root = get_assets_root_path()
table_usd_path = assets_root + "/Isaac/Props/Mounts/SeattleLabTable/table_instanceable.usd"
table_prim = define_prim("/World/Table","Xform")
table_prim.GetReferences().AddReference(table_usd_path)
table_xform = UsdGeom.Xformable(table_prim)
for op in table_xform.GetOrderedXformOps():
    if op.GetOpName()=="xformOp:translate":
        op.Set((0.55,0.0,0.0))
        break

# ------------------------------
# 4. Cube 타겟
# ------------------------------
cube_root_path = "/World/Cube"
cube_root_prim = define_prim(cube_root_path, "Xform")
cube_root_xform = UsdGeom.Xformable(cube_root_prim)

# remove old xformOp
for attr in list(cube_root_prim.GetAttributes()):
    name = attr.GetName()
    if name.startswith("xformOp:") or name=="xformOpOrder":
        cube_root_prim.RemoveProperty(name)

scale_op = cube_root_xform.AddScaleOp()
translate_op = cube_root_xform.AddTranslateOp()
cube_root_xform.SetXformOpOrder([scale_op, translate_op])
scale_op.Set(Gf.Vec3f(0.05,0.05,0.05))
translate_op.Set(Gf.Vec3f(0.3,0.0,0.05))

cube_geom_prim = define_prim(cube_root_path+"/geom","Cube")
for attr in list(cube_geom_prim.GetAttributes()):
    name = attr.GetName()
    if name.startswith("xformOp:") or name=="xformOpOrder":
        cube_geom_prim.RemoveProperty(name)

# Physics on root
try: UsdPhysics.CollisionAPI.Apply(cube_root_prim)
except: pass
try: rb_api = UsdPhysics.RigidBodyAPI.Apply(cube_root_prim); rb_api.CreateRigidBodyEnabledAttr().Set(True)
except: pass
try: PhysxSchema.PhysxRigidBodyAPI.Apply(cube_root_prim)
except: pass
try: mass_api = UsdPhysics.MassAPI.Apply(cube_root_prim); mass_api.CreateMassAttr().Set(0.5)
except: pass

# step few frames to register
for _ in range(6): world.step(render=False)

# ------------------------------
# 5. Dynamic Control Handle
# ------------------------------
dc = None
rigid_handle = None
if _dynamic_control:
    dc = _dynamic_control.acquire_dynamic_control_interface()
    for _ in range(4): world.step(render=False)
    try:
        n = dc.get_rigid_body_count()
        for i in range(n):
            pth = dc.get_rigid_body_path(i)
            if pth.startswith(cube_root_path):
                rigid_handle = dc.get_rigid_body(pth)
                break
    except: pass
    print("[DEBUG] initial rigid_handle:", rigid_handle)

# ------------------------------
# 6. Robot Spawn (E0509)
# ------------------------------
e0509_usd_path = "/home/woo/ros2_ws/src/doosan-robot2/dsr_description2/usd/e0509.usd"
robot_prim_path = "/World/e0509"
add_reference_to_stage(e0509_usd_path, robot_prim_path)
ee_prim_path = "/World/e0509/e0509/tool0"

world.step(render=False)
world.reset()
robot = world.scene.add(Articulation(prim_path=robot_prim_path,name="e0509"))
world.reset()

num_joints = robot.num_dof
arm_joint_indices = np.arange(0,6,dtype=np.int64)
default_q = np.zeros(num_joints,dtype=np.float32)
default_q[2] = 1.5708
default_q[4] = 1.5708
robot.set_joint_positions(default_q)
world.step(render=False)
default_q_arm = default_q[arm_joint_indices]
action_scale = 0.5
prev_action = np.zeros(action_dim,dtype=np.float32)

# ------------------------------
# 7. Observation / Reset Utilities
# ------------------------------
def get_ee_pose_world():
    ee_prim = get_prim_at_path(ee_prim_path)
    if not ee_prim.IsValid():
        return np.zeros(3,dtype=np.float32), np.array([1.0,0.0,0.0,0.0],dtype=np.float32)
    m = UsdGeom.Xformable(ee_prim).ComputeLocalToWorldTransform(0.0)
    pos = m.ExtractTranslation()
    rot = m.ExtractRotation().GetQuat()
    pos_np = np.array([float(pos[0]),float(pos[1]),float(pos[2])],dtype=np.float32)
    quat_np = np.array([rot.GetReal(),rot.GetImaginary()[0],rot.GetImaginary()[1],rot.GetImaginary()[2]],dtype=np.float32)
    return pos_np, quat_np

def quat_to_euler_zyx(q):
    w,x,y,z = q
    sinr_cosp = 2.0*(w*x + y*z)
    cosr_cosp = 1.0 - 2.0*(x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2.0*(w*y - z*x)
    pitch = np.arcsin(np.clip(2.0*(w*y - z*x),-1,1))
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll,pitch,yaw],dtype=np.float32)

def get_cube_pos_world():
    p = translate_op.Get()
    return np.array([float(p[0]),float(p[1]),float(p[2])],dtype=np.float32)

def compute_obs(last_action):
    q = robot.get_joint_positions()
    qd = robot.get_joint_velocities()
    q_arm = q[arm_joint_indices]
    qd_arm = qd[arm_joint_indices]
    jointpos_rel = (q_arm - default_q_arm).astype(np.float32)
    jointvel = qd_arm.astype(np.float32)
    ee_pos_w, ee_quat_w = get_ee_pose_world()
    ee_euler_w = quat_to_euler_zyx(ee_quat_w)
    cube_pos_w = get_cube_pos_world()
    obs_parts = [jointpos_rel,jointvel,ee_pos_w,ee_euler_w,cube_pos_w,last_action.astype(np.float32)]
    return np.concatenate(obs_parts,axis=-1)

def teleport_rigid(dc, handle_or_path, pos, quat_wxyz):
    ok = False
    try: dc.set_rigid_body_world_pose(handle_or_path,list(pos),list(quat_wxyz)); ok=True
    except: pass
    try: dc.set_rigid_body_linear_velocity(handle_or_path,[0,0,0])
    except: pass
    try: dc.set_rigid_body_angular_velocity(handle_or_path,[0,0,0])
    except: pass
    try: dc.wake_up_rigid_body(handle_or_path)
    except: pass
    return ok

def reset_episode():
    global episode_idx, prev_action, step_in_episode
    step_in_episode = 0
    q = robot.get_joint_positions(); q[:] = default_q; robot.set_joint_positions(q)
    x = np.random.uniform(0.2,0.5); y=np.random.uniform(-0.2,0.2); z=0.05
    translate_op.Set(Gf.Vec3f(float(x),float(y),float(z)))
    # dynamic_control teleport
    target = rigid_handle if rigid_handle else cube_root_path
    if dc: teleport_rigid(dc,target,[x,y,z],[1,0,0,0])
    prev_action = np.zeros(action_dim,dtype=np.float32)
    world.step(render=False)
    print(f"=== Reset episode {episode_idx} ===")

# ------------------------------
# 8. Main loop
# ------------------------------
episode_idx = 0
step_in_episode = 0
prev_action = np.zeros(action_dim, dtype=np.float32)  # 미리 초기화

reset_episode()  # 이제 episode_idx가 정의되어 있음

episode_length_steps = 600
max_episodes = 20
first_play = True


with torch.no_grad():
    while simulation_app.is_running():
        world.step(render=True)
        if not world.is_playing(): continue
        if first_play: translate_op.Set(translate_op.Get()); first_play=False
        step_in_episode += 1
        if step_in_episode > episode_length_steps:
            episode_idx += 1
            if episode_idx >= max_episodes: break
            reset_episode(); continue

        obs = compute_obs(prev_action)
        obs_tensor = torch.from_numpy(obs).float().unsqueeze(0).to(device)
        mean,std,value = policy_model(obs_tensor)
        action = mean.cpu().numpy().squeeze().astype(np.float32)
        action_clipped = np.clip(action,-1,1)
        target_q_arm = default_q_arm + action_clipped*action_scale
        q = robot.get_joint_positions()
        q[arm_joint_indices] = target_q_arm
        robot.apply_action(ArticulationAction(joint_positions=q))
        prev_action = action_clipped.copy()

simulation_app.close()
