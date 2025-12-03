# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import joint_pos_env_cfg


@configclass
class E0509PickPlaceEnvCfg(joint_pos_env_cfg.E0509PickPlaceEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set E0509 robot with high PD gains for better IK tracking
        self.scene.robot = joint_pos_env_cfg.E0509_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (E0509) with relative IK control
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["joint_[1-6]"],
            body_name="link_6",  # E0509 end-effector link
            controller=DifferentialIKControllerCfg(
                command_type="pose", 
                use_relative_mode=True,   # Relative pose control
                ik_method="dls"           # Damped Least Squares
            ),
            scale=0.5,  # Smaller scale for safer incremental movements
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.15]),  # E0509 TCP offset
        )
        
        # Keep gripper as joint position control
        self.actions.gripper_action = joint_pos_env_cfg.mdp.JointPositionActionCfg(
            asset_name="robot", 
            joint_names=["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"],
            scale=0.05,
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