# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the E0509 Pick and Place environment with joint position control."""

import math

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, FrameTransformerCfg, OffsetCfg
from isaaclab.utils import configclass

# Import custom MDP functions
from . import mdp as custom_mdp

# Import centralized configuration
from .config_constants import CONFIG

##
# Pre-defined configs
##

from isaaclab.markers import CUBOID_MARKER_CFG  # isort: skip
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip

FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.10, 0.10, 0.10)

##
# Scene definition
##


@configclass
class E0509SceneCfg(InteractiveSceneCfg):
    """Configuration for the E0509 pick and place scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    # robot - E0509 with gripper
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/woo/Downloads/source/3d/e0509/e0509_with_gripper.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
            activate_contact_sensors=True,  # 🔧 Contact sensor 활성화
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "joint_1": 0.0,
                "joint_2": 0.0,   # 블록 방향으로 약간 숙임
                "joint_3": 1.57,    # 팔꿈치 구부림
                "joint_4": 0.0,
                "joint_5": 1.57,   # 손목 아래로
                "joint_6": 0.0,
                CONFIG.gripper.JOINT_NAMES[0]: 0.0,  # "rh_l1" 
                CONFIG.gripper.JOINT_NAMES[1]: 0.0,  # "rh_l2"
                CONFIG.gripper.JOINT_NAMES[2]: 0.0,  # "rh_p12_rn"
                CONFIG.gripper.JOINT_NAMES[3]: 0.0,  # "rh_r2"
            },
            pos=(0.0, 0.0, 0.0),
        ),
        actuators={
            "arm_joint_1": ImplicitActuatorCfg(
                joint_names_expr=["joint_1"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=10000.0,
                damping=100.0,
            ),
            "arm_joint_2": ImplicitActuatorCfg(
                joint_names_expr=["joint_2"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=10000.0,
                damping=100.0,
            ),
            "arm_joint_3": ImplicitActuatorCfg(
                joint_names_expr=["joint_3"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=10000.0,
                damping=100.0,
            ),
            "arm_joint_4": ImplicitActuatorCfg(
                joint_names_expr=["joint_4"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=10000.0,
                damping=100.0,
            ),
            "arm_joint_5": ImplicitActuatorCfg(
                joint_names_expr=["joint_5"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=10000.0,
                damping=100.0,
            ),
            "arm_joint_6": ImplicitActuatorCfg(
                joint_names_expr=["joint_6"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=10000.0,
                damping=100.0,
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=CONFIG.gripper.JOINT_NAMES,
                effort_limit_sim=80.0,      # 더 강한 힘 (50.0 → 80.0)
                velocity_limit_sim=200.0,   # 더 빠른 속도 (50.0 → 200.0)  
                stiffness=1000.0,           # 더 유연함 (5000.0 → 1000.0)
                damping=20.0,               # 더 부드러운 움직임 (50.0 → 20.0)
            ),
        },
    )

    # End-effector frame transformer
    ee_frame: FrameTransformerCfg = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link_6",  # E0509 end-effector link
        debug_vis=False,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/link_6",
                name="ee_tcp",
                offset=OffsetCfg(
                    pos=(0.0, 0.0, 0.0),
                    rot=(1.0, 0.0, 0.0, 0.0),
                ),
            ),
        ],
    )

    # 🟢 NEW: 새로운 간단한 블록 설정
    target_object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetObject",
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.04, 0.04, 0.04),  # 4cm 정육면체 - 더 작고 다루기 쉽게
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),  # 더 가벼운 질량
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.0, 0.0),  # 빨간색으로 변경
                metallic=0.1,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.02)),  # 더 가까운 위치에 생성
    )

    # goal box (green) - Visual indicator for target location
    box = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Box",
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.2, 0.2, 0.01),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,  # Keep disabled - this is just visual
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,  # Keep disabled - this is just visual
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.0, 1.0, 0.0),
                metallic=0.0,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.6, 0.6, 0.0)),
    )

    # 🟢 Contact sensor for gripper-object interaction
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/rh_p12_rn",  # 그리퍼 베이스
        update_period=0.0,
        history_length=3,
        debug_vis=False,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/TargetObject"],
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action: mdp.JointPositionActionCfg = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
        ],
        scale=0.1,  # Reduced from 0.5 to 0.25 (half speed)
    )
    
    gripper_action: mdp.JointPositionActionCfg = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=CONFIG.gripper.JOINT_NAMES,
        scale=0.2,  # 🚀 더 역동적인 그리퍼 움직임 (0.025 → 0.2)
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # robot joint state
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot")})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot")})
        
        # end-effector position and orientation
        ee_pos = ObsTerm(func=custom_mdp.ee_position_w, params={"asset_cfg": SceneEntityCfg("robot", body_names=["link_6"])})
        
        # object positions  
        target_pos = ObsTerm(func=mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("target_object")})
        box_pos = ObsTerm(func=mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("box")})
        
        # gripper state
        gripper_pos = ObsTerm(
            func=mdp.joint_pos_rel, 
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=CONFIG.gripper.JOINT_NAMES)}
        )

        # relative distances
        rel_ee_target_distance = ObsTerm(func=custom_mdp.rel_ee_block_distance)
        
        # action history
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # Primary reward: Distance-based approaching
    reach_target_distance = RewTerm(
        func=custom_mdp.linear_distance_reward, 
        weight=CONFIG.weights.DISTANCE_APPROACH,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["link_6"]),
            "target_cfg": SceneEntityCfg("target_object"),
            "max_dist": CONFIG.distance.MAX_APPROACH_DISTANCE,
        },
    )

    # Secondary reward: Exponential distance for precision
    reach_target_precision = RewTerm(
        func=custom_mdp.exp_distance_reward,
        weight=CONFIG.weights.PRECISION_APPROACH,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["link_6"]),
            "target_cfg": SceneEntityCfg("target_object"),
            "alpha": CONFIG.distance.EXPONENTIAL_ALPHA,
        },
    )

    # 🚫 GRIPPER ALIGNMENT DISABLED - No orientation constraints

    # 🤖 NEW: Pre-Grasp Positioning Reward  
    pre_grasp_position = RewTerm(
        func=custom_mdp.pre_grasp_reward,
        weight=CONFIG.weights.PRE_GRASP,
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.FINGER_TIPS),  # 끝단으로 정확한 거리 측정
            "target_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "approach_distance": CONFIG.distance.CLOSE_APPROACH,
            "open_threshold": CONFIG.gripper.OPEN_THRESHOLD,
        },
    )

    # ✋ NEW: Grasping Reward
    grasp_success = RewTerm(
        func=custom_mdp.grasp_reward,
        weight=CONFIG.weights.GRASP_SUCCESS,
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.ALL_FINGER_BODIES),  # 전체 집게로 접촉 감지
            "target_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "grasp_distance": CONFIG.distance.GRASP_DISTANCE,
            "close_threshold": CONFIG.gripper.CLOSE_THRESHOLD,
        },
    )

    # 🚀 NEW: Lifting Reward (Sequential after grasp)
    lift_block = RewTerm(
        func=custom_mdp.lift_reward,
        weight=CONFIG.weights.LIFT_SUCCESS,
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.FINGER_TIPS),  # 끝단으로 높이 측정
            "block_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "min_lift_height": CONFIG.height.MIN_LIFT_HEIGHT,
            "close_threshold": CONFIG.gripper.CLOSE_THRESHOLD,
            "contact_threshold": CONFIG.distance.CONTACT_THRESHOLD,
        },
    )

    # 🏆 NEW: Complete Task Reward (Final bonus)
    complete_task = RewTerm(
        func=custom_mdp.complete_task_reward,
        weight=CONFIG.weights.COMPLETE_TASK,
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.FINGER_TIPS),  # 끝단으로 완료 확인
            "block_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "min_lift_height": CONFIG.height.MIN_LIFT_HEIGHT,
            "hold_time": CONFIG.timing.GRASP_HOLD_TIME,
            "close_threshold": CONFIG.gripper.CLOSE_THRESHOLD,
            "contact_threshold": CONFIG.distance.CONTACT_THRESHOLD,
        },
    )

    # Success bonus (reduced weight as sequence rewards are primary now)
    reach_success_bonus = RewTerm(
        func=custom_mdp.success_bonus,
        weight=CONFIG.weights.SUCCESS_BONUS,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.FINGER_TIPS),  # 끝단으로 성공 판정
            "target_cfg": SceneEntityCfg("block"),
            "threshold": CONFIG.distance.SUCCESS_THRESHOLD,
        },
    )

    # Penalties
    action_penalty = RewTerm(
        func=mdp.action_l2,
        weight=-0.05,
    )
    joint_vel_penalty = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0001,
    )

    # Contact reward for successful grasping (uses the correct sensor name)
    object_contact_reward = RewTerm(
        func=custom_mdp.grasp_reward,
        weight=CONFIG.weights.GRASP_SUCCESS,
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.ALL_FINGER_BODIES),
            "target_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joints": CONFIG.gripper.JOINT_NAMES,
            "grasp_distance": CONFIG.distance.GRASP_DISTANCE,
            "close_threshold": CONFIG.gripper.CLOSE_THRESHOLD,
        },
    )

    # 🚫 TEMPORARILY DISABLED: Workspace penalty for debugging
    # workspace_penalty = RewTerm(
    #     func=custom_mdp.workspace_boundary_penalty,
    #     weight=1.0,
    #     params={
    #         "robot_cfg": SceneEntityCfg("robot"),
    #         "ee_body_name": CONFIG.gripper.LEFT_FINGER_TIP,  # 왼쪽 집게 끝단을 대표로 사용
    #         "workspace_limits": {
    #             "x": CONFIG.workspace.X_LIMITS,
    #             "y": CONFIG.workspace.Y_LIMITS,
    #             "z": CONFIG.workspace.Z_LIMITS,
    #         },
    #         "boundary_penalty": CONFIG.penalty.WORKSPACE_BOUNDARY,
    #     },
    # )

    joint_safety_penalty = RewTerm(
        func=custom_mdp.joint_limits_penalty, 
        weight=1.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            "limit_margin": 0.1,  # 관절 한계의 10% 여유
            "penalty_weight": -5.0,
        },
    )

    # 🚨 NEW: Block Collision Avoidance (Critical!)
    block_collision_penalty = RewTerm(
        func=custom_mdp.premature_block_collision_penalty,
        weight=1.0,  # Full weight - prevent premature collision!
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.ALL_FINGER_BODIES),  # 전체 집게로 충돌 감지
            "block_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joints": CONFIG.gripper.JOINT_NAMES,
            "approach_phase_distance": CONFIG.distance.FAR_APPROACH,
            "grasp_ready_distance": CONFIG.distance.CLOSE_APPROACH,
            "collision_penalty": CONFIG.penalty.BLOCK_COLLISION,
        },
    )

    # ✅ NEW: Safe Approach Reward (Encourage proper approach)
    safe_approach = RewTerm(
        func=custom_mdp.safe_approach_reward,
        weight=CONFIG.weights.SAFE_APPROACH,  # Encourage safe approach pattern
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.FINGER_TIPS),  # 끝단으로 접근 위치 측정
            "block_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "safe_approach_height": CONFIG.height.APPROACH_HEIGHT,
            "approach_corridor_radius": CONFIG.workspace.BOUNDARY_MARGIN,
            "reward_weight": 10.0,
        },
    )

    # ✅ NEW: Safe Height Maintenance (Encourage proper height)
    safe_height_reward = RewTerm(
        func=custom_mdp.safe_height_reward,
        weight=CONFIG.weights.SAFE_HEIGHT,  # Encourage maintaining safe height
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.FINGER_TIPS),  # 끝단으로 높이 측정
            "min_safe_height": CONFIG.height.MIN_SAFE_HEIGHT,
            "optimal_height": CONFIG.height.OPTIMAL_HEIGHT,
            "reward_scale": 2.0,
        },
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    
    # 🏆 NEW: Sequential Task Success (Complete pick and lift)
    task_complete = DoneTerm(
        func=custom_mdp.sequential_task_termination,
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.FINGER_TIPS),  # 끝단으로 성공 판정
            "block_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "min_lift_height": CONFIG.height.MIN_LIFT_HEIGHT,
            "hold_time": CONFIG.timing.GRASP_HOLD_TIME,
            "close_threshold": CONFIG.gripper.CLOSE_THRESHOLD,
            "contact_threshold": CONFIG.distance.CONTACT_THRESHOLD,
        },
    )

    # 🚨 NEW: Safety Terminations (Critical!)
    collision_termination = DoneTerm(
        func=custom_mdp.collision_termination,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "collision_sensor_name": "collision_sensor",
        },
    )

    workspace_termination = DoneTerm(
        func=custom_mdp.workspace_boundary_termination,
        params={
            "robot_cfg": SceneEntityCfg("robot"), 
            "ee_body_name": CONFIG.gripper.LEFT_FINGER_TIP,  # 왼쪽 집게 끝단을 대표로 사용
            "workspace_limits": {
                "x": CONFIG.workspace.X_LIMITS_CRITICAL,   # 임계 영역: ±100cm
                "y": CONFIG.workspace.Y_LIMITS_CRITICAL,   # 임계 영역: ±100cm
                "z": CONFIG.workspace.Z_LIMITS_CRITICAL,   # 그리퍼 끝이 바닥~150cm (블록 충돌 방지)
            },
        },
    )

    # 🚨 NEW: Premature Block Collision Termination
    block_collision_termination = DoneTerm(
        func=custom_mdp.premature_block_collision_termination,
        params={
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.ALL_FINGER_BODIES),  # 전체 집게로 충돌 감지
            "block_cfg": SceneEntityCfg("block"),
            "robot_cfg": SceneEntityCfg("robot"),
            "collision_force_threshold": CONFIG.distance.COLLISION_THRESHOLD,
        },
    )

    # 🚨 NEW: Ground Collision Termination (Critical Safety!)
    ground_collision_termination = DoneTerm(
        func=custom_mdp.ground_collision_termination,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_cfg": SceneEntityCfg("robot", body_names=CONFIG.gripper.ALL_FINGER_BODIES),  # 전체 집게로 충돌 감지
            "collision_force_threshold": CONFIG.distance.GROUND_COLLISION_THRESHOLD,
        },
    )


@configclass
class EventsCfg:
    """Configuration for environment reset events."""

    reset_robot = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.9, 1.1),
            "velocity_range": (0.0, 0.0),
        },
    )

    # 🟢 NEW: 새로운 블록 리셋 이벤트 - 간단하고 안정적
    reset_target = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("target_object"),
            "pose_range": {
                "x": (0.4, 0.6),   # 로봇 앞쪽 40-60cm
                "y": (-0.15, 0.15), # 좌우 15cm 범위
                "z": (0.02, 0.02),  # 바닥 위 2cm 고정
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
            },
        },
    )


##
# Environment configuration
##


@configclass
class E0509PickPlaceEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the E0509 pick and place environment."""

    # Scene settings
    scene: E0509SceneCfg = E0509SceneCfg(num_envs=4096, env_spacing=2.5)
    
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventsCfg = EventsCfg()
    
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 4.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.2
        # update viewer settings
        self.viewer.eye = (1.5, 1.5, 1.2)
        self.viewer.lookat = (0.0, 0.0, 0.3)


##
# Play (test) environment configuration
##

@configclass 
class E0509SceneCfg_PLAY(E0509SceneCfg):
    """Play scene configuration - simplified without contact sensors."""
    
    # Override contact sensor with None to disable it
    contact_forces = None
    
    def __post_init__(self):
        """Remove contact sensors for play mode."""
        # Call parent post init but skip contact sensor initialization
        super().__post_init__()


@configclass
class E0509PickPlaceEnvCfg_PLAY(E0509PickPlaceEnvCfg):
    """Play environment configuration."""
    
    scene: E0509SceneCfg_PLAY = E0509SceneCfg_PLAY(num_envs=4, env_spacing=2.5)
    
    def __post_init__(self):
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 4
        self.scene.env_spacing = 2.5
        # reduce the number of objects in the scene
        self.scene.dome_light.spawn.intensity = 4000.0
        # disable randomization for play
        self.observations.policy.enable_corruption = False