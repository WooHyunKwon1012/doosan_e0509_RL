# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom observation functions for E0509 tasks."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ee_position_w(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """End-effector position in world frame.
    
    Args:
        env: The environment instance.
        asset_cfg: The scene entity configuration with body_names specified.
        
    Returns:
        End-effector position in world frame with shape (num_envs, 3).
    """
    asset = env.scene[asset_cfg.name]
    body_names = asset_cfg.body_names
    
    if body_names:
        link_name = body_names[0]
        idx = asset.data.body_names.index(link_name)
        return asset.data.body_pos_w[:, idx, :]
    
    return asset.data.root_pos_w


def rel_ee_block_distance(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The distance between the end-effector and the block."""
    ee_tf_data = env.scene["ee_frame"].data
    block_data = env.scene["block"].data

    return block_data.root_pos_w - ee_tf_data.target_pos_w[..., 0, :]


def ee_pos(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The position of the end-effector relative to the environment origins."""
    ee_tf_data = env.scene["ee_frame"].data
    ee_pos = ee_tf_data.target_pos_w[..., 0, :] - env.scene.env_origins

    return ee_pos


def ee_quat(env: ManagerBasedRLEnv, make_quat_unique: bool = True) -> torch.Tensor:
    """The orientation of the end-effector in the environment frame.

    If :attr:`make_quat_unique` is True, the quaternion is made unique by ensuring the real part is positive.
    """
    import isaaclab.utils.math as math_utils
    ee_tf_data = env.scene["ee_frame"].data
    ee_quat = ee_tf_data.target_quat_w[..., 0, :]
    # make first element of quaternion positive
    return math_utils.quat_unique(ee_quat) if make_quat_unique else ee_quat


def gripper_fingertips_pos(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The position of the gripper fingertips relative to the environment origins."""
    ee_tf_data = env.scene["ee_frame"].data
    fingertips_pos = ee_tf_data.target_pos_w[..., 1:, :] - env.scene.env_origins.unsqueeze(1)

    return fingertips_pos.view(env.num_envs, -1)
