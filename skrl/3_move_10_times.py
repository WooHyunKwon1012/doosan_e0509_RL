import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from omni.isaac.kit import SimulationApp

# Isaac Sim 앱 실행
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdGeom, UsdLux, Gf

############################################################
# 1. skrl PPO policy 모듈 (obs_dim=32, action_dim=7)
############################################################

class SkrlPPOPolicy(nn.Module):
    def __init__(self, obs_dim, action_dim):
        super().__init__()
        self.net_container = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.ELU(),
            nn.Linear(64, 64),
            nn.ELU(),
        )
        self.policy_layer = nn.Linear(64, action_dim)
        self.value_layer = nn.Linear(64, 1)
        self.log_std_parameter = nn.Parameter(torch.zeros(action_dim))

    def forward(self, obs):
        x = self.net_container(obs)
        mean = self.policy_layer(x)
        value = self.value_layer(x)
        log_std = self.log_std_parameter
        std = log_std.exp()
        return mean, std, value

############################################################
# 2. 체크포인트 로드 및 scaler 구성
############################################################

policy_path = "/home/woo/isaac_lab_ws/IsaacLab/logs/skrl/reach_franka/2025-11-28_09-39-35_ppo_torch/checkpoints/best_agent.pt"

ckpt = torch.load(policy_path)
sp = ckpt["state_preprocessor"]
running_mean = sp["running_mean"].cpu().numpy()        # (32,)
running_var = sp["running_variance"].cpu().numpy()     # (32,)
state_std = np.sqrt(running_var + 1e-8)
obs_dim = running_mean.shape[0]                        # 32

policy_state = ckpt["policy"]
action_dim = policy_state["policy_layer.weight"].shape[0]  # 7

print(f"obs_dim={obs_dim}, action_dim={action_dim}")

policy_model = SkrlPPOPolicy(obs_dim, action_dim)
policy_model.load_state_dict(policy_state, strict=True)
policy_model.eval()
device = "cuda:0"
policy_model.to(device)

def normalize_obs(obs_np: np.ndarray) -> np.ndarray:
    return (obs_np - running_mean) / (state_std + 1e-8)

############################################################
# 3. Isaac Sim world/scene (Franka + Table + Goal)
############################################################

world = World(stage_units_in_meters=1.0)
world.scene.clear()
world.reset()

# DomeLight
dome_prim = define_prim("/World/Light/DomeLight", "DomeLight")
dome = UsdLux.DomeLight(dome_prim)
dome.GetIntensityAttr().Set(2500.0)
dome.GetColorAttr().Set((0.75, 0.75, 0.75))

assets_root = get_assets_root_path()
print("assets_root:", assets_root)

table_usd_path = assets_root + "/Isaac/Props/Mounts/SeattleLabTable/table_instanceable.usd"
franka_usd_path = assets_root + "/Isaac/IsaacLab/Robots/FrankaEmika/panda_instanceable.usd"

# 1) Table 스폰
table_prim = define_prim("/World/Table", "Xform")
table_prim.GetReferences().AddReference(table_usd_path)

table_xform = UsdGeom.Xformable(table_prim)
ops = table_xform.GetOrderedXformOps()
for op in ops:
    if op.GetOpName() == "xformOp:translate":
        op.Set((0.55, 0.0, 0.0))
        break

# 2) Robot 스폰
robot_prim = define_prim("/World/Robot", "Xform")
robot_prim.GetReferences().AddReference(franka_usd_path)

robot_xform = UsdGeom.Xformable(robot_prim)
ops = robot_xform.GetOrderedXformOps()
for op in ops:
    if op.GetOpName() == "xformOp:translate":
        op.Set((0.0, 0.0, 0.0))
        break

# 3) Goal marker (작은 Sphere)
goal_prim = define_prim("/World/Goal", "Sphere")
goal_xform = UsdGeom.Xformable(goal_prim)
goal_xform.AddTranslateOp().Set(Gf.Vec3f(0.5, 0.0, 0.3))

radius_attr = goal_prim.GetAttribute("radius")
if radius_attr.IsValid():
    radius_attr.Set(0.05)


# 4) physics 초기화
world.step(render=False)

# 5) Articulation 등록
robot = world.scene.add(
    Articulation(
        prim_path="/World/Robot",
        name="franka_robot",
    )
)

world.reset()

############################################################
# 4. joint / default pose / index 설정
############################################################

# Isaac Lab FrankaReachEnvCfg 기준: arm 7 DOF + finger 2 DOF = 9개만 사용
num_arm_joints = action_dim          # 7
num_finger_joints = 2                # 2
num_used_joints = num_arm_joints + num_finger_joints  # 9

# arm joint index: 0~6, finger: 7,8 이라고 가정
arm_joint_indices = np.arange(0, num_arm_joints, dtype=np.int64)
finger_joint_indices = np.arange(num_arm_joints, num_used_joints, dtype=np.int64)

# default_pos_full: 길이 9짜리만 관리
default_pos_full = np.zeros(num_used_joints, dtype=np.float32)
default_pos_full[1] = -0.569
default_pos_full[3] = -2.81
default_pos_full[5] = 3.037
default_pos_full[6] = 0.741
default_pos_full[finger_joint_indices] = 0.04  # finger 두 개

# 로봇에 default pos 적용 (앞 9개만)
current_full = robot.get_joint_positions()
current_full[:num_used_joints] = default_pos_full
robot.set_joint_positions(current_full)

default_pos_arm = default_pos_full[arm_joint_indices]

action_scale = 0.5
prev_action_arm = np.zeros(num_arm_joints, dtype=np.float32)

############################################################
# 5. 에피소드 및 EE pose command 설정
############################################################

episode_length_steps = 300
max_episodes = 10
episode_idx = 0
step_in_episode = 0

def sample_ee_pose():
    # reach_env_cfg.CommandsCfg.ranges와 비슷하게 샘플링
    x = np.random.uniform(0.35, 0.65)
    y = np.random.uniform(-0.2, 0.2)
    z = np.random.uniform(0.15, 0.5)
    # orientation: 간단히 identity
    return np.array([x, y, z, 1.0, 0.0, 0.0, 0.0], dtype=np.float32)

# 초기 목표
ee_pose_cmd = sample_ee_pose()
print(f"[EP {episode_idx}] new goal:", ee_pose_cmd[:3])
############################################################
# 6. 메인 루프: obs(32) → policy → action(7) → joint pos
############################################################

while simulation_app.is_running():
    world.step(render=True)
    if not world.is_playing():
        continue

    # 에피소드 관리
    step_in_episode += 1
    if step_in_episode > episode_length_steps:
        episode_idx += 1
        step_in_episode = 0
        print(f"=== New episode {episode_idx} ===")

        # 새 목표 샘플
        ee_pose_cmd = sample_ee_pose()

        # 로봇 joint 리셋
        q_full = robot.get_joint_positions()
        q_full[:num_used_joints] = default_pos_full
        robot.set_joint_positions(q_full)

        # 이전 action 리셋
        prev_action_arm = np.zeros(num_arm_joints, dtype=np.float32)

        if episode_idx >= max_episodes:
            break

    # Goal marker를 ee_pose_cmd 위치로 이동 (world 좌표)
    goal_pos = Gf.Vec3f(float(ee_pose_cmd[0]), float(ee_pose_cmd[1]), float(ee_pose_cmd[2]))
    goal_ops = goal_xform.GetOrderedXformOps()
    for op in goal_ops:
        if op.GetOpName() == "xformOp:translate":
            op.Set(goal_pos)
            break

    # 관측 계산
    q_full = robot.get_joint_positions()
    qd_full = robot.get_joint_velocities()

    q_used = q_full[:num_used_joints]
    ee_prim = get_prim_at_path("/World/Robot/panda_hand") # EE 링크 경로
    ee_xform = UsdGeom.Xformable(ee_prim)
    m = ee_xform.ComputeLocalToWorldTransform(0.0)
    pos = m.ExtractTranslation()
    print("ee pos (xyz):", float(pos[0]), float(pos[1]), float(pos[2]))
    
    qd_used = qd_full[:num_used_joints]

    joint_pos_rel_9 = (q_used - default_pos_full).astype(np.float32)
    joint_vel_rel_9 = qd_used.astype(np.float32)

    last_action_7 = prev_action_arm.astype(np.float32)  # (7,)
    print("obs pose_command (xyz):", ee_pose_cmd[:3])

    obs_parts = [
        joint_pos_rel_9,   # 9
        joint_vel_rel_9,   # 9
        ee_pose_cmd,       # 7
        last_action_7,     # 7
    ]
    obs = np.concatenate(obs_parts, axis=-1)  # (32,)

    #if obs.shape != obs_dim:
        #print(f"[WARN] obs_dim mismatch: expected {obs_dim}, got {obs.shape}")
        #continue

    # 정규화 + 정책
    obs_norm = normalize_obs(obs)
    with torch.no_grad():
        obs_tensor = torch.from_numpy(obs_norm).float().unsqueeze(0).to(device)
        mean, std, value = policy_model(obs_tensor)
        action_arm = mean.cpu().numpy().squeeze().astype(np.float32)   # (7,)

    # 액션 적용
    action_arm_clipped = np.clip(action_arm, -1.0, 1.0)
    print("action_arm:", action_arm_clipped)
    target_pos_arm = default_pos_arm + action_arm_clipped * action_scale

    # 앞 9개 중 arm 7개만 변경, finger는 default 유지
    target_q_used = default_pos_full.copy()
    target_q_used[arm_joint_indices] = target_pos_arm

    # 로봇 전체 joint 벡터에 반영
    target_joint_pos_full = q_full.copy()
    target_joint_pos_full[:num_used_joints] = target_q_used

    robot.apply_action(ArticulationAction(joint_positions=target_joint_pos_full))

    prev_action_arm = action_arm_clipped.copy()

simulation_app.close()

