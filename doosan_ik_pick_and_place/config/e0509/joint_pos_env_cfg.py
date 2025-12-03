# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils import configclass
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from doosan_ik_pick_place import mdp

from doosan_ik_pick_place.pick_place_env_cfg import (
    FRAME_MARKER_SMALL_CFG,
    E0509PickPlaceEnvCfg,
)

import isaaclab.sim as sim_utils

##
# Pre-defined configs for E0509 robot
##

# E0509 robot configuration with standard PD gains
E0509_CFG = ArticulationCfg(
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
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 1.5708,  # 90도 (π/2 라디안)
            "joint_4": 0.0,
            "joint_5": 1.5708,  # 90도 (π/2 라디안)
            "joint_6": 0.0,
            "rh_l1": 0.0,
            "rh_l2": 0.0,
            "rh_p12_rn": 0.0,
            "rh_r2": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            effort_limit=500.0,
            velocity_limit=500.0,
            stiffness=800.0,
            damping=20.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["rh_.*"],
            effort_limit=50.0,
            velocity_limit=50.0,
            stiffness=5000.0,
            damping=50.0,
        ),
    },
)

# High PD gains for IK tracking (similar to FRANKA_PANDA_HIGH_PD_CFG)
E0509_HIGH_PD_CFG = E0509_CFG.copy()
E0509_HIGH_PD_CFG.actuators = {
    "arm": ImplicitActuatorCfg(
        joint_names_expr=["joint_[1-6]"],
        effort_limit=500.0,
        velocity_limit=500.0,
        stiffness=2000.0,  # Higher stiffness for better tracking
        damping=100.0,     # Higher damping for stability
    ),
    "gripper": ImplicitActuatorCfg(
        joint_names_expr=["rh_.*"],
        effort_limit=50.0,
        velocity_limit=50.0,
        stiffness=5000.0,
        damping=50.0,
    ),
}


@configclass
class E0509PickPlaceEnvCfg(E0509PickPlaceEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set E0509 as robot
        self.scene.robot = E0509_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set Actions for the specific robot type (E0509)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["joint_[1-6]"],
            scale=0.5,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.JointPositionActionCfg(
            asset_name="robot", 
            joint_names=["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"],
            scale=0.05,
        )

        # Listens to the required transforms
        # IMPORTANT: The order of the frames in the list is important. The first frame is the tool center point (TCP)
        # the other frames are the gripper fingers
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            debug_vis=False,
            visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/EndEffectorFrameTransformer"),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/link_6",
                    name="ee_tcp",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.15),  # E0509 end-effector offset
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/rh_l1",
                    name="gripper_l1",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.02),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/rh_r2",
                    name="gripper_r2",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.02),
                    ),
                ),
            ],
        )


@configclass
class E0509PickPlaceEnvCfg_PLAY(E0509PickPlaceEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False