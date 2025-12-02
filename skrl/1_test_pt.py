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
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdGeom

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
# 3. Isaac Sim world/scene (Franka + Table)
############################################################

world = World(stage_units_in_meters=1.0)
world.scene.clear()
world.reset()

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

# 3) physics 초기화
world.step(render=False)

# 4) Articulation 등록
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
arm_joint_indices = np.arange(0, num_arm_joints, dtype=np.int64)           # [0..6]
finger_joint_indices = np.arange(num_arm_joints, num_used_joints, dtype=np.int64)  # [7,8]

# default_pos_full: 길이 9짜리만 관리 (나머지 joint는 0으로 둠)
default_pos_full = np.zeros(num_used_joints, dtype=np.float32)
default_pos_full[1] = -0.569
default_pos_full[3] = -2.81
default_pos_full[5] = 3.037
default_pos_full[6] = 0.741
default_pos_full[finger_joint_indices] = 0.04  # 두 finger

# 로봇에 default pos 적용 (나머지 joint는 0으로 유지)
current_full = robot.get_joint_positions()
# 앞 9개만 default로 덮어쓰고, 나머지는 그대로 두기
current_full[:num_used_joints] = default_pos_full
robot.set_joint_positions(current_full)

default_pos_arm = default_pos_full[arm_joint_indices]

action_scale = 0.5
prev_action_arm = np.zeros(num_arm_joints, dtype=np.float32)

############################################################
# 5. EE pose command (고정)
############################################################

ee_pose_cmd = np.array([0.5, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0], dtype=np.float32)

############################################################
# 6. 메인 루프: obs(32) → policy → action(7) → joint pos
############################################################

while simulation_app.is_running():
    world.step(render=True)
    if not world.is_playing():
        continue

    q_full = robot.get_joint_positions()
    qd_full = robot.get_joint_velocities()

    # 앞 9개 joint만 사용 (7 DOF + 2 finger)
    q_used = q_full[:num_used_joints]
    qd_used = qd_full[:num_used_joints]

    joint_pos_rel_9 = (q_used - default_pos_full).astype(np.float32)  # (9,)
    joint_vel_rel_9 = qd_used.astype(np.float32)                      # (9,)

    last_action_7 = prev_action_arm.astype(np.float32)                # (7,)

    obs_parts = [
        joint_pos_rel_9,
        joint_vel_rel_9,
        ee_pose_cmd,
        last_action_7,
    ]
    obs = np.concatenate(obs_parts, axis=-1)  # (32,)

    if obs.shape[0] != obs_dim:
        print(f"[WARN] obs_dim mismatch: expected {obs_dim}, got {obs.shape[0]}")
        continue

    obs_norm = normalize_obs(obs)

    with torch.no_grad():
        obs_tensor = torch.from_numpy(obs_norm).float().unsqueeze(0).to(device)
        mean, std, value = policy_model(obs_tensor)
        action_arm = mean.cpu().numpy().squeeze().astype(np.float32)   # (7,)

    action_arm_clipped = np.clip(action_arm, -1.0, 1.0)
    target_pos_arm = default_pos_arm + action_arm_clipped * action_scale

    # 전체 joint target 구성: 앞 9개 중 arm 7개만 변경, finger는 default 유지
    target_q_used = default_pos_full.copy()
    target_q_used[arm_joint_indices] = target_pos_arm

    # 로봇 전체 joint 벡터에 반영
    target_joint_pos_full = q_full.copy()
    target_joint_pos_full[:num_used_joints] = target_q_used

    robot.apply_action(ArticulationAction(joint_positions=target_joint_pos_full))

    prev_action_arm = action_arm_clipped.copy()

simulation_app.close()

