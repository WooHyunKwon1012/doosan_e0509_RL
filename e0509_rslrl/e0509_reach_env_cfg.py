# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING
import math

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import mdp

##
# Scene definition
##


@configclass
class E0509ReachSceneCfg(InteractiveSceneCfg):
    """Configuration for the E0509 reach scene with a robotic arm and cube target."""

    # world
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd",
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.55, 0.0, 0.0), rot=(0.70711, 0.0, 0.0, 0.70711)),
    )

    # target cube on table
    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True,  # Cube doesn't fall
                disable_gravity=True,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.3, 0.0, 0.02)),
    )

    # E0509 robot
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            ###디렉토리 수정 필요###
            # Doosan robotics2 에서 제공하는 기본 e0509 모델 사용
            usd_path="/home/been/isaaclab/IsaacLab/models/e0509/e0509.usd",
            activate_contact_sensors=False,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={
                "joint_1": 0.0,
                "joint_2": 0.0,
                "joint_3": 1.5708,  # 90 degrees in radians
                "joint_4": 0.0,
                "joint_5": 1.5708,  # 90 degrees in radians
                "joint_6": 0.0,
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint_[1-6]"],
                effort_limit=200.0,
                velocity_limit=3.14,
                stiffness=800.0,
                damping=40.0,
            ),
        },
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action: ActionTerm = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["joint_[1-6]"],
        scale=0.5,
        use_default_offset=True,
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Robot state
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        
        # EE position in robot root frame
        ee_position = ObsTerm(func=mdp.ee_position_in_robot_root_frame)
        
        # EE orientation (roll, pitch, yaw)
        ee_orientation = ObsTerm(func=mdp.ee_orientation_euler)
        
        # Cube position in robot root frame
        cube_position = ObsTerm(func=mdp.cube_position_in_robot_root_frame)
        
        # Actions
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (1.0, 1.0),  # Fixed at 1.0 (default pose) - no randomization
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_cube_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (0.1, 0.5),  # Table left side
                "y": (-0.2, 0.2),  # Table left area
                "z": (0.0, 0.0),
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("cube"),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # Main task reward: EE to cube distance with target distance (10cm above)
    ee_cube_distance = RewTerm(
        func=mdp.ee_cube_distance_reward,
        weight=2.0,
        params={
            "std": 0.05,  # Tolerance around target distance
            "target_distance": 0.15,  # 15cm above cube center
            "asset_cfg": SceneEntityCfg("robot", body_names=["tool0"]),
        },
    )

    # Fine-grained reward when close to target distance
    ee_cube_distance_fine = RewTerm(
        func=mdp.ee_cube_distance_reward,
        weight=1.0,
        params={
            "std": 0.02,  # Stricter tolerance
            "target_distance": 0.15,  # 15cm above cube center
            "asset_cfg": SceneEntityCfg("robot", body_names=["tool0"]),
        },
    )

    # Penalty for being too low (avoid table collision)
    ee_height_penalty = RewTerm(
        func=mdp.ee_height_penalty,
        weight=-0.5,
        params={"min_height": 0.05, "asset_cfg": SceneEntityCfg("robot", body_names=["tool0"])},
    )

    # Reward for maintaining vertical (downward) orientation of end-effector
    # INCREASED from 0.3 to 1.5 to strongly enforce Y-axis correction
    ee_vertical_orientation = RewTerm(
        func=mdp.ee_vertical_orientation_reward,
        weight=0.3,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=["tool0"])},
    )

    # Action smoothness penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.0001)

    # Joint velocity penalty (encourage smooth motion)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0001,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # Success termination: EE reaches 10cm above cube center and stays still for 1 second
    task_success = DoneTerm(
        func=mdp.task_success_termination,
        time_out=False,
        params={
            "success_threshold": 0.02,  # 2cm tolerance
            "stillness_threshold": 0.01,  # 0.01 m/s velocity threshold
            "asset_cfg": SceneEntityCfg("robot", body_names=["tool0"]),
        },
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "action_rate", "weight": -0.01, "num_steps": 5000},
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -0.001, "num_steps": 5000},
    )


##
# Environment configuration
##


@configclass
class E0509ReachEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the E0509 reach environment."""

    # Scene settings
    scene: E0509ReachSceneCfg = E0509ReachSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.sim.render_interval = self.decimation
        self.episode_length_s = 10.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0 / 60.0


@configclass
class E0509ReachEnvCfg_PLAY(E0509ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False