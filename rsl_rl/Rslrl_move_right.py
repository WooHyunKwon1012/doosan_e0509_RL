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
from pxr import UsdGeom, UsdLux, Gf

############################################################
# 1. 유틸: RPY -> quaternion
############################################################

def rpy_to_quat(roll, pitch, yaw):
    # 표준 ZYX(RPY) 오일러 → 쿼터니언
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qw, qx, qy, qz], dtype=np.float32)

############################################################
# 2. rsl-rl ActorCritic 네트워크 정의
#    (obs_dim=32, action_dim=7, hidden 64-64, ELU)
############################################################

class RslRlActorCritic(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int):
        super().__init__()
        # actor: Linear(obs_dim,64) -> ELU -> Linear(64,64) -> ELU -> Linear(64, action_dim)
        self.actor_0 = nn.Linear(obs_dim, 64)
        self.actor_2 = nn.Linear(64, 64)
        self.actor_4 = nn.Linear(64, action_dim)
        # critic: Linear(obs_dim,64) -> ELU -> Linear(64,64) -> ELU -> Linear(64,1)
        self.critic_0 = nn.Linear(obs_dim, 64)
        self.critic_2 = nn.Linear(64, 64)
        self.critic_4 = nn.Linear(64, 1)
        # rsl-rl ckpt의 "std" 파라미터를 여기 log-std로 받는다
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, obs: torch.Tensor):
        # obs: [B, obs_dim]
        x_a = F.elu(self.actor_0(obs))
        x_a = F.elu(self.actor_2(x_a))
        mean = self.actor_4(x_a)  # [B, action_dim]

        x_v = F.elu(self.critic_0(obs))
        x_v = F.elu(self.critic_2(x_v))
        value = self.critic_4(x_v).squeeze(-1)  # [B]

        std = self.log_std.exp().expand_as(mean)
        return mean, std, value

############################################################
# 3. rsl-rl 체크포인트 로드 (model_999.pt)
############################################################

# rsl-rl 학습 로그에서의 모델 경로
ckpt_path = "/home/woo/isaac_lab_ws/IsaacLab/logs/rsl_rl/franka_reach/2025-11-28_20-03-27/model_999.pt"

device = torch.device("cuda:0")

ckpt = torch.load(ckpt_path, map_location="cpu")
msd = ckpt["model_state_dict"]  # OrderedDict: keys = ["std", "actor.*", "critic.*"]

obs_dim = 32         # FrankaReachEnv에서 사용한 obs 길이[attached_file:skrl_move_right-pt-pail-gudong.py]
action_dim = 7       # Franka arm 7 DOF[attached_file:skrl_move_right-pt-pail-gudong.py]

policy_model = RslRlActorCritic(obs_dim, action_dim)
policy_model.to(device)

# 현재 네트워크 state_dict 가져오기
state_dict = policy_model.state_dict()

# rsl-rl ckpt의 키를 이 네트워크에 맞게 매핑
mapping = {}
for k in list(msd.keys()):
    if k.startswith("actor."):
        # actor.0.weight -> actor_0.weight
        new_k = k.replace("actor.", "actor_", 1)
    elif k.startswith("critic."):
        # critic.0.weight -> critic_0.weight
        new_k = k.replace("critic.", "critic_", 1)
    elif k == "std":
        # std -> log_std
        new_k = "log_std"
    else:
        continue
    mapping[new_k] = k

# 해당되는 파라미터만 복사
for new_k, old_k in mapping.items():
    if new_k in state_dict:
        state_dict[new_k] = msd[old_k]

policy_model.load_state_dict(state_dict, strict=False)
policy_model.eval()

############################################################
# 4. Isaac Sim world/scene (Franka + Table + Goal)
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
# 5. joint / default pose / index 설정
############################################################

# FrankaReachEnv 기준: arm 7 DOF + finger 2 DOF = 9개만 사용[attached_file:skrl_move_right-pt-pail-gudong.py]
num_arm_joints = action_dim      # 7
num_finger_joints = 2            # 2
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
# 6. 에피소드 및 EE pose command 설정
############################################################

episode_length_steps = 300
max_episodes = 10
episode_idx = 0
step_in_episode = 0

def sample_ee_pose():
    # env.yaml ranges와 동일[attached_file:skrl_move_right-pt-pail-gudong.py][attached_file:env.yaml]
    x = np.random.uniform(0.35, 0.65)
    y = np.random.uniform(-0.2, 0.2)
    z = np.random.uniform(0.15, 0.5)
    roll = 0.0
    pitch = np.pi  # env.yaml에서 pi로 고정
    yaw = np.random.uniform(-3.14, 3.14)
    quat = rpy_to_quat(roll, pitch, yaw)  # [qw, qx, qy, qz]
    return np.concatenate(
        [np.array([x, y, z], dtype=np.float32), quat], axis=0
    ).astype(np.float32)

# 초기 목표
ee_pose_cmd = sample_ee_pose()
print(f"[EP {episode_idx}] new goal:", ee_pose_cmd[:3])

############################################################
# 7. 메인 루프: obs(32) → policy → action(7) → joint pos
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

    ee_prim = get_prim_at_path("/World/Robot/panda_hand")  # EE 링크 경로[attached_file:skrl_move_right-pt-pail-gudong.py]
    ee_xform = UsdGeom.Xformable(ee_prim)
    m = ee_xform.ComputeLocalToWorldTransform(0.0)
    pos = m.ExtractTranslation()
    print("ee pos (xyz):", float(pos[0]), float(pos[1]), float(pos[2]))

    qd_used = qd_full[:num_used_joints]

    joint_pos_rel_9 = (q_used - default_pos_full).astype(np.float32)
    joint_vel_rel_9 = qd_used.astype(np.float32)
    last_action_7 = prev_action_arm.astype(np.float32)  # (7,)

    print("obs pose_command (xyz):", ee_pose_cmd[:3])

    # obs 구성: joint_pos(9) + joint_vel(9) + ee_pose_cmd(7) + last_action(7) = 32[attached_file:skrl_move_right-pt-pail-gudong.py]
    obs_parts = [
        joint_pos_rel_9,  # 9
        joint_vel_rel_9,  # 9
        ee_pose_cmd,      # 7
        last_action_7,    # 7
    ]
    obs = np.concatenate(obs_parts, axis=-1)  # (32,)

    # rsl-rl 설정에서 obs 정규화는 사용하지 않음 (actor_obs_normalization=False)[attached_file:agent.yaml][attached_file:rsl_rl_ppo_cfg.py]
    obs_tensor = torch.from_numpy(obs).float().unsqueeze(0).to(device)

    # 정책 호출 (deterministic: mean 사용)
    with torch.no_grad():
        mean, std, value = policy_model(obs_tensor)
        action_arm = mean.cpu().numpy().squeeze().astype(np.float32)  # (7,)

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

