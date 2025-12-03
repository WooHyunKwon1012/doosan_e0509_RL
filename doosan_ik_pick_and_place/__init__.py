# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Manipulation environments for E0509 robot pick and place tasks with IK control."""

import gymnasium as gym

from . import mdp
from .pick_place_env_cfg import E0509PickPlaceEnvCfg, E0509PickPlaceEnvCfg_PLAY

##
# Register Gym environments.
##

gym.register(
    id="E0509-PickPlace-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": E0509PickPlaceEnvCfg,
        "rsl_rl_cfg_entry_point": f"{__name__}.config.e0509.agents:rsl_rl_cfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="E0509-PickPlace-Play-v0", 
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": E0509PickPlaceEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": f"{__name__}.config.e0509.agents:rsl_rl_cfg",
    },
    disable_env_checker=True,
)

# IK environments
from .config.e0509.ik_abs_env_cfg import E0509PickPlaceEnvCfg as E0509PickPlaceEnvCfg_IK_ABS

gym.register(
    id="E0509-PickPlace-IK-Abs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv", 
    kwargs={
        "env_cfg_entry_point": E0509PickPlaceEnvCfg_IK_ABS,
        "rsl_rl_cfg_entry_point": f"{__name__}.config.e0509.agents:rsl_rl_ik_cfg",
    },
    disable_env_checker=True,
)

from .config.e0509.ik_rel_env_cfg import E0509PickPlaceEnvCfg as E0509PickPlaceEnvCfg_IK_REL

gym.register(
    id="E0509-PickPlace-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": E0509PickPlaceEnvCfg_IK_REL,
        "rsl_rl_cfg_entry_point": f"{__name__}.config.e0509.agents:rsl_rl_ik_cfg", 
    },
    disable_env_checker=True,
)