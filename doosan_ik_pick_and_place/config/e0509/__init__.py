# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

##
# Joint Position Control
##

gym.register(
    id="Isaac-Pick-Place-E0509-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:E0509PickPlaceEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:E0509PPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Pick-Place-E0509-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:E0509PickPlaceEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:E0509PPORunnerCfg",
    },
    disable_env_checker=True,
)


##
# Inverse Kinematics - Absolute Pose Control
##

gym.register(
    id="Isaac-Pick-Place-E0509-IK-Abs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.ik_abs_env_cfg:E0509PickPlaceEnvCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Pick-Place-E0509-IK-Abs-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.ik_abs_env_cfg:E0509PickPlaceEnvCfg_PLAY",
    },
    disable_env_checker=True,
)

##
# Inverse Kinematics - Relative Pose Control
##

gym.register(
    id="Isaac-Pick-Place-E0509-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.ik_rel_env_cfg:E0509PickPlaceEnvCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Pick-Place-E0509-IK-Rel-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.ik_rel_env_cfg:E0509PickPlaceEnvCfg_PLAY",
    },
    disable_env_checker=True,
)
