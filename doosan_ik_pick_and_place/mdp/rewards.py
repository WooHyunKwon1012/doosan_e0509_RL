# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom reward functions for E0509 pick and place task."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _get_entity_pos_w(env: ManagerBasedRLEnv, entity_cfg: SceneEntityCfg) -> torch.Tensor:
    """Get entity position in world frame.
    
    Args:
        env: The environment instance.
        entity_cfg: The scene entity configuration.
        
    Returns:
        Entity position in world frame with shape (num_envs, 3).
    """
    asset = env.scene[entity_cfg.name]
    body_names = getattr(entity_cfg, "body_names", None)
    
    if body_names:
        if len(body_names) == 1:
            # Single body - original behavior
            link_name = body_names[0]
            idx = asset.data.body_names.index(link_name)
            return asset.data.body_pos_w[:, idx, :]
        else:
            # Multiple bodies (e.g., gripper fingers) - return average position
            positions = []
            for link_name in body_names:
                if link_name in asset.data.body_names:
                    idx = asset.data.body_names.index(link_name)
                    positions.append(asset.data.body_pos_w[:, idx, :])
            
            if positions:
                # Return average position of all gripper fingers
                return torch.stack(positions, dim=0).mean(dim=0)
            else:
                # Fallback to root if no bodies found
                return asset.data.root_pos_w
    
    return asset.data.root_pos_w


def _get_gripper_lowest_point(env: ManagerBasedRLEnv, entity_cfg: SceneEntityCfg) -> torch.Tensor:
    """Get the lowest point of gripper fingers for ground collision detection.
    
    Args:
        env: The environment instance.
        entity_cfg: The gripper entity configuration.
        
    Returns:
        Lowest Z coordinate among gripper fingers with shape (num_envs,).
    """
    asset = env.scene[entity_cfg.name]
    body_names = getattr(entity_cfg, "body_names", None)
    
    if body_names and len(body_names) > 1:
        # Get all gripper finger positions
        finger_heights = []
        for link_name in body_names:
            if link_name in asset.data.body_names:
                idx = asset.data.body_names.index(link_name)
                finger_heights.append(asset.data.body_pos_w[:, idx, 2])  # Z coordinate
        
        if finger_heights:
            # Return the minimum height (closest to ground)
            return torch.stack(finger_heights, dim=0).min(dim=0)[0]
    
    # Fallback to single body or average
    pos = _get_entity_pos_w(env, entity_cfg)
    return pos[:, 2]


def exp_distance_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    alpha: float = 5.0,
) -> torch.Tensor:
    """Exponential distance reward between two entities.
    
    Reward = exp(-alpha * distance)
    
    Args:
        env: The environment instance.
        asset_cfg: The asset entity configuration.
        target_cfg: The target entity configuration.
        alpha: Exponential decay factor. Higher values give steeper rewards.
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    asset_pos = _get_entity_pos_w(env, asset_cfg)
    target_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(asset_pos - target_pos, dim=-1)
    return torch.exp(-alpha * distance)


def linear_distance_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    max_dist: float = 1.0,
) -> torch.Tensor:
    """Linear distance reward between two entities.
    
    Reward = max(0, 1 - distance / max_dist)
    
    Args:
        env: The environment instance.
        asset_cfg: The asset entity configuration.
        target_cfg: The target entity configuration.
        max_dist: Maximum distance for normalization.
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    asset_pos = _get_entity_pos_w(env, asset_cfg)
    target_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(asset_pos - target_pos, dim=-1)
    return torch.clamp(1.0 - distance / max_dist, min=0.0)


def success_bonus(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    threshold: float = 0.05,
) -> torch.Tensor:
    """Success bonus when asset is close enough to target.
    
    Args:
        env: The environment instance.
        asset_cfg: The asset entity configuration.
        target_cfg: The target entity configuration.
        threshold: Distance threshold for success (in meters).
        
    Returns:
        Reward tensor with shape (num_envs,). 1.0 if within threshold, 0.0 otherwise.
    """
    asset_pos = _get_entity_pos_w(env, asset_cfg)
    target_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(asset_pos - target_pos, dim=-1)
    return (distance < threshold).float()


def velocity_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    weight: float = 0.01,
) -> torch.Tensor:
    """Penalty for high velocities to encourage smooth motion.
    
    Inspired by train_pick_and_place.py's smooth action approach.
    
    Args:
        env: The environment instance.
        asset_cfg: The asset entity configuration.
        weight: Weight for velocity penalty.
        
    Returns:
        Penalty tensor with shape (num_envs,).
    """
    asset = env.scene[asset_cfg.name]
    velocity = torch.norm(asset.data.root_lin_vel_b, dim=-1)
    return -weight * velocity.pow(2)


def action_smoothness_penalty(
    env: ManagerBasedRLEnv,
    weight: float = 0.01,
) -> torch.Tensor:
    """Penalty for large action changes between timesteps.
    
    Promotes smoother control policies similar to train_pick_and_place.py.
    
    Args:
        env: The environment instance.
        weight: Weight for smoothness penalty.
        
    Returns:
        Penalty tensor with shape (num_envs,).
    """
    # Isaac Lab에서는 action_manager를 통해 액션에 접근
    if hasattr(env.action_manager, 'actions'):
        current_actions = env.action_manager.actions
    elif hasattr(env, '_actions'):
        current_actions = env._actions
    else:
        return torch.zeros(env.num_envs, device=env.device)
    
    if not hasattr(env, '_prev_actions'):
        env._prev_actions = torch.zeros_like(current_actions)
        return torch.zeros(env.num_envs, device=env.device)
    
    action_diff = current_actions - env._prev_actions
    env._prev_actions = current_actions.clone()
    
    return -weight * torch.norm(action_diff, dim=-1).pow(2)


def adaptive_distance_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    alpha_start: float = 2.0,
    alpha_end: float = 8.0,
    curriculum_steps: int = 1000000,
) -> torch.Tensor:
    """Adaptive exponential distance reward with curriculum learning.
    
    Starts with wider exploration (lower alpha) and gradually becomes more precise.
    Inspired by train_pick_and_place.py's exploration strategy.
    
    Args:
        env: The environment instance.
        asset_cfg: The asset entity configuration.
        target_cfg: The target entity configuration.
        alpha_start: Initial alpha value (wider reward).
        alpha_end: Final alpha value (more precise reward).
        curriculum_steps: Steps over which to transition.
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get current training step (if available)
    current_step = getattr(env, 'common_step_counter', 0)
    
    # Compute curriculum progress
    progress = min(current_step / curriculum_steps, 1.0)
    alpha = alpha_start + (alpha_end - alpha_start) * progress
    
    # Compute distance reward
    asset_pos = _get_entity_pos_w(env, asset_cfg)
    target_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(asset_pos - target_pos, dim=-1)
    
    return torch.exp(-alpha * distance)


def gripper_approach_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    gripper_joint_names: list[str],
    max_dist: float = 1.0,
    open_threshold: float = 0.3,
) -> torch.Tensor:
    """Reward for approaching target with open gripper.
    
    Args:
        env: The environment instance.
        asset_cfg: The robot asset configuration.
        target_cfg: The target entity configuration.
        gripper_joint_names: Names of gripper joints.
        max_dist: Maximum distance for normalization.
        open_threshold: Minimum opening for gripper to be considered "open".
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get robot asset
    robot = env.scene[asset_cfg.name]
    
    # Get gripper joint positions
    gripper_indices = []
    for joint_name in gripper_joint_names:
        if joint_name in robot.data.joint_names:
            idx = robot.data.joint_names.index(joint_name)
            gripper_indices.append(idx)
    
    if not gripper_indices:
        return torch.zeros(env.num_envs, device=env.device)
    
    gripper_pos = robot.data.joint_pos[:, gripper_indices]
    gripper_opening = torch.mean(gripper_pos, dim=-1)  # Average opening
    
    # Check if gripper is open
    is_open = gripper_opening > open_threshold
    
    # Distance reward (only when gripper is open)
    asset_pos = _get_entity_pos_w(env, asset_cfg)
    target_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(asset_pos - target_pos, dim=-1)
    distance_reward = torch.clamp(1.0 - distance / max_dist, 0.0, 1.0)
    
    # Apply only when gripper is open
    return distance_reward * is_open.float()


def gripper_grasp_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    gripper_joint_names: list[str],
    contact_threshold: float = 0.05,
    close_threshold: float = 0.1,
) -> torch.Tensor:
    """Reward for grasping target with closed gripper.
    
    Args:
        env: The environment instance.
        asset_cfg: The robot asset configuration.
        target_cfg: The target entity configuration.
        gripper_joint_names: Names of gripper joints.
        contact_threshold: Maximum distance to consider "contact".
        close_threshold: Maximum opening to consider gripper "closed".
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get robot asset
    robot = env.scene[asset_cfg.name]
    
    # Get gripper joint positions
    gripper_indices = []
    for joint_name in gripper_joint_names:
        if joint_name in robot.data.joint_names:
            idx = robot.data.joint_names.index(joint_name)
            gripper_indices.append(idx)
    
    if not gripper_indices:
        return torch.zeros(env.num_envs, device=env.device)
    
    gripper_pos = robot.data.joint_pos[:, gripper_indices]
    gripper_opening = torch.mean(gripper_pos, dim=-1)  # Average opening
    
    # Check if gripper is closed
    is_closed = gripper_opening < close_threshold
    
    # Check contact (close distance)
    asset_pos = _get_entity_pos_w(env, asset_cfg)
    target_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(asset_pos - target_pos, dim=-1)
    is_contact = distance < contact_threshold
    
    # Reward when both conditions are met
    grasp_success = is_closed.float() * is_contact.float()
    
    return grasp_success


def block_lift_reward(
    env: ManagerBasedRLEnv,
    block_cfg: SceneEntityCfg,
    gripper_joint_names: list[str],
    robot_cfg: SceneEntityCfg,
    min_height: float = 0.05,
    close_threshold: float = 0.1,
) -> torch.Tensor:
    """Reward for lifting block when grasped.
    
    Args:
        env: The environment instance.
        block_cfg: The block entity configuration.
        gripper_joint_names: Names of gripper joints.
        robot_cfg: The robot asset configuration.
        min_height: Minimum height to consider "lifted".
        close_threshold: Maximum opening to consider gripper "closed".
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get robot asset
    robot = env.scene[robot_cfg.name]
    
    # Get gripper state
    gripper_indices = []
    for joint_name in gripper_joint_names:
        if joint_name in robot.data.joint_names:
            idx = robot.data.joint_names.index(joint_name)
            gripper_indices.append(idx)
    
    if not gripper_indices:
        return torch.zeros(env.num_envs, device=env.device)
    
    gripper_pos = robot.data.joint_pos[:, gripper_indices]
    gripper_opening = torch.mean(gripper_pos, dim=-1)
    is_closed = gripper_opening < close_threshold
    
    # Get block height
    block_pos = _get_entity_pos_w(env, block_cfg)
    block_height = block_pos[:, 2]  # Z coordinate
    
    # Initial height (approximately table height)
    table_height = 0.0  # Adjust based on your scene setup
    lift_height = block_height - table_height
    
    # Reward for lifting when gripper is closed
    lift_reward = torch.clamp(lift_height / min_height, 0.0, 2.0)  # Cap at 2x
    
    return lift_reward * is_closed.float()


def block_place_reward(
    env: ManagerBasedRLEnv,
    block_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    gripper_joint_names: list[str],
    robot_cfg: SceneEntityCfg,
    place_threshold: float = 0.1,
    open_threshold: float = 0.3,
) -> torch.Tensor:
    """Reward for placing block at target when gripper opens.
    
    Args:
        env: The environment instance.
        block_cfg: The block entity configuration.
        target_cfg: The target entity configuration.
        gripper_joint_names: Names of gripper joints.
        robot_cfg: The robot asset configuration.
        place_threshold: Maximum distance to consider "placed".
        open_threshold: Minimum opening to consider gripper "open".
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get robot asset
    robot = env.scene[robot_cfg.name]
    
    # Get gripper state
    gripper_indices = []
    for joint_name in gripper_joint_names:
        if joint_name in robot.data.joint_names:
            idx = robot.data.joint_names.index(joint_name)
            gripper_indices.append(idx)
    
    if not gripper_indices:
        return torch.zeros(env.num_envs, device=env.device)
    
    gripper_pos = robot.data.joint_pos[:, gripper_indices]
    gripper_opening = torch.mean(gripper_pos, dim=-1)
    is_open = gripper_opening > open_threshold
    
    # Check if block is near target
    block_pos = _get_entity_pos_w(env, block_cfg)
    target_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(block_pos - target_pos, dim=-1)
    is_placed = distance < place_threshold
    
    # Reward when block is placed and gripper is open
    place_success = is_placed.float() * is_open.float()
    
    return place_success


def reach_success_termination(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    threshold: float = 0.05,
) -> torch.Tensor:
    """Termination condition when end-effector reaches near the target block.
    
    Args:
        env: The environment instance.
        asset_cfg: The robot asset configuration.
        target_cfg: The target asset configuration.
        threshold: Distance threshold for success (in meters).
        
    Returns:
        Boolean tensor indicating success for each environment.
    """
    # Get end-effector position
    ee_pos = _get_entity_pos_w(env, asset_cfg)
    
    # Get target position
    target_pos = _get_entity_pos_w(env, target_cfg)
    
    # Calculate distance
    distance = torch.norm(ee_pos - target_pos, dim=-1)
    
    # Success when distance is less than threshold
    success = distance < threshold
    
    return success


def gripper_alignment_reward(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    xy_threshold: float = 0.1,
    height_threshold: float = 0.2,
    orientation_weight: float = 0.5,
) -> torch.Tensor:
    """Reward for proper gripper alignment above target block.
    
    This function encourages the gripper to:
    1. Be positioned above the block (X-Y alignment)
    2. Be at appropriate height above the block 
    3. Be oriented downward for grasping
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        target_cfg: The target block configuration.
        xy_threshold: Maximum X-Y distance for full alignment reward.
        height_threshold: Optimal height above block.
        orientation_weight: Weight for orientation alignment (0-1).
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get gripper and block positions
    gripper_pos = _get_entity_pos_w(env, gripper_cfg)
    block_pos = _get_entity_pos_w(env, target_cfg)
    
    # 1. X-Y Alignment (gripper should be directly above block)
    xy_distance = torch.norm(gripper_pos[:, :2] - block_pos[:, :2], dim=-1)
    xy_alignment = torch.exp(-10.0 * xy_distance / xy_threshold)
    
    # 2. Height Alignment (gripper should be at optimal height above block)
    height_diff = torch.abs(gripper_pos[:, 2] - block_pos[:, 2] - height_threshold)
    height_alignment = torch.exp(-5.0 * height_diff / height_threshold)
    
    # 3. Orientation Alignment (gripper should point downward)
    # 🚫 ORIENTATION ALIGNMENT DISABLED - No gripper orientation constraints
    # Robot can use any orientation to approach the target
    orientation_alignment = torch.ones(env.num_envs, device=env.device)
    
    # Combine all alignment components
    total_alignment = (
        xy_alignment * 
        height_alignment * 
        (orientation_weight * orientation_alignment + (1 - orientation_weight))
    )
    
    return total_alignment


def pre_grasp_reward(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joints: list[str] = ["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"],
    approach_distance: float = 0.05,
    open_threshold: float = 0.3,
) -> torch.Tensor:
    """Reward for approaching block with open gripper in correct position.
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        target_cfg: The target block configuration.
        robot_cfg: The robot asset configuration.
        gripper_joints: Names of gripper joints.
        approach_distance: Distance to consider "ready to grasp".
        open_threshold: Minimum opening to consider gripper "open".
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get robot asset
    robot = env.scene[robot_cfg.name]
    
    # Check gripper opening
    gripper_indices = []
    for joint_name in gripper_joints:
        if joint_name in robot.data.joint_names:
            idx = robot.data.joint_names.index(joint_name)
            gripper_indices.append(idx)
    
    is_open = torch.ones(env.num_envs, device=env.device)  # Default to open
    if gripper_indices:
        gripper_pos = robot.data.joint_pos[:, gripper_indices]
        gripper_opening = torch.mean(gripper_pos, dim=-1)
        is_open = (gripper_opening > open_threshold).float()
    
    # Check if gripper is close enough to block
    gripper_pos = _get_entity_pos_w(env, gripper_cfg)
    block_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(gripper_pos - block_pos, dim=-1)
    is_close = (distance < approach_distance).float()
    
    # Get alignment reward
    alignment = gripper_alignment_reward(env, gripper_cfg, target_cfg)
    
    # Combine: reward for being close + aligned + open gripper
    return is_close * alignment * is_open


def grasp_reward(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joints: list[str] = ["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"],
    grasp_distance: float = 0.03,
    close_threshold: float = 0.1,
) -> torch.Tensor:
    """Reward for successfully grasping the block.
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        target_cfg: The target block configuration.
        robot_cfg: The robot asset configuration.
        gripper_joints: Names of gripper joints.
        grasp_distance: Maximum distance to consider "grasped".
        close_threshold: Maximum opening to consider gripper "closed".
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get robot asset
    robot = env.scene[robot_cfg.name]
    
    # Check gripper closing
    gripper_indices = []
    for joint_name in gripper_joints:
        if joint_name in robot.data.joint_names:
            idx = robot.data.joint_names.index(joint_name)
            gripper_indices.append(idx)
    
    is_closed = torch.ones(env.num_envs, device=env.device)  # Default to closed
    if gripper_indices:
        gripper_pos = robot.data.joint_pos[:, gripper_indices]
        gripper_opening = torch.mean(gripper_pos, dim=-1)
        is_closed = (gripper_opening < close_threshold).float()
    
    # Check contact with block
    gripper_pos = _get_entity_pos_w(env, gripper_cfg)
    block_pos = _get_entity_pos_w(env, target_cfg)
    distance = torch.norm(gripper_pos - block_pos, dim=-1)
    is_grasping = (distance < grasp_distance).float()
    
    # Successful grasp = close gripper + contact with block
    return is_closed * is_grasping


def lift_reward(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    block_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joints: list[str] = ["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"],
    min_lift_height: float = 0.1,
    close_threshold: float = 0.15,
    contact_threshold: float = 0.04,
) -> torch.Tensor:
    """Reward for lifting the block while grasped.
    
    This is the sequential reward after grasping:
    1. Block must be grasped (gripper closed + contact)
    2. Block must be lifted above initial height
    3. Higher lift = more reward
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        block_cfg: The block configuration.
        robot_cfg: The robot asset configuration.
        gripper_joints: Names of gripper joints.
        min_lift_height: Minimum height to consider successful lift.
        close_threshold: Maximum opening to consider gripper "closed".
        contact_threshold: Maximum distance to consider "grasped".
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get robot asset
    robot = env.scene[robot_cfg.name]
    
    # Check if gripper is closed
    gripper_indices = []
    for joint_name in gripper_joints:
        if joint_name in robot.data.joint_names:
            idx = robot.data.joint_names.index(joint_name)
            gripper_indices.append(idx)
    
    is_closed = torch.ones(env.num_envs, device=env.device)
    if gripper_indices:
        gripper_pos = robot.data.joint_pos[:, gripper_indices]
        gripper_opening = torch.mean(gripper_pos, dim=-1)
        is_closed = (gripper_opening < close_threshold).float()
    
    # Check if block is still being grasped (contact)
    gripper_pos = _get_entity_pos_w(env, gripper_cfg)
    block_pos = _get_entity_pos_w(env, block_cfg)
    contact_distance = torch.norm(gripper_pos - block_pos, dim=-1)
    is_grasped = (contact_distance < contact_threshold).float()
    
    # Get block height (Z coordinate)
    block_height = block_pos[:, 2]
    
    # Define initial/table height (adjust based on your scene)
    initial_block_height = 0.025  # Half of block size (0.05/2) above ground
    
    # Calculate lift height
    lift_height = block_height - initial_block_height
    
    # Progressive lift reward
    lift_progress = torch.clamp(lift_height / min_lift_height, 0.0, 2.0)
    
    # Only give lift reward if block is properly grasped
    grasp_condition = is_closed * is_grasped
    
    return grasp_condition * lift_progress


def complete_task_reward(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    block_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joints: list[str] = ["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"],
    min_lift_height: float = 0.1,
    hold_time: int = 20,  # steps to hold lifted position
    close_threshold: float = 0.15,
    contact_threshold: float = 0.04,
) -> torch.Tensor:
    """Reward for completing the full pick task (grasp + lift + hold).
    
    This is the final reward for task completion:
    1. Block must be grasped and lifted above min_height
    2. Must maintain grasp for required hold_time
    3. Big bonus for task completion
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        block_cfg: The block configuration.
        robot_cfg: The robot asset configuration.
        gripper_joints: Names of gripper joints.
        min_lift_height: Minimum height for successful lift.
        hold_time: Number of steps to maintain lift.
        close_threshold: Maximum opening to consider gripper "closed".
        contact_threshold: Maximum distance to consider "grasped".
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get current lift status
    lift_status = lift_reward(
        env, gripper_cfg, block_cfg, robot_cfg, 
        gripper_joints, min_lift_height, close_threshold, contact_threshold
    )
    
    # Check if block is successfully lifted (lift_status > 1.0 means lifted above min_height)
    is_lifted = (lift_status >= 1.0).float()
    
    # Initialize or update lift counter for each environment
    if not hasattr(env, '_lift_hold_counter'):
        env._lift_hold_counter = torch.zeros(env.num_envs, device=env.device, dtype=torch.int32)
    
    # Update counter: increment if lifted, reset if not
    env._lift_hold_counter = torch.where(
        is_lifted.bool(),
        env._lift_hold_counter + 1,
        torch.zeros_like(env._lift_hold_counter)
    )
    
    # Task complete if held for required time
    task_complete = (env._lift_hold_counter >= hold_time).float()
    
    return task_complete


def sequential_task_termination(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    block_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joints: list[str] = ["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"],
    min_lift_height: float = 0.1,
    hold_time: int = 20,
    close_threshold: float = 0.15,
    contact_threshold: float = 0.04,
) -> torch.Tensor:
    """Termination condition for successful task completion.
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        block_cfg: The block configuration.
        robot_cfg: The robot asset configuration.
        gripper_joints: Names of gripper joints.
        min_lift_height: Minimum height for successful lift.
        hold_time: Number of steps to maintain lift.
        close_threshold: Maximum opening to consider gripper "closed".
        contact_threshold: Maximum distance to consider "grasped".
        
    Returns:
        Boolean tensor indicating task completion for each environment.
    """
    # Use complete_task_reward to determine success
    task_complete = complete_task_reward(
        env, gripper_cfg, block_cfg, robot_cfg,
        gripper_joints, min_lift_height, hold_time, close_threshold, contact_threshold
    )
    
    return task_complete.bool()


def collision_penalty(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    collision_sensor_name: str = "collision_sensor",
    self_collision_penalty: float = -100.0,
    ground_collision_penalty: float = -50.0,
) -> torch.Tensor:
    """Penalty for robot collisions (self-collision and ground collision).
    
    This function heavily penalizes dangerous collisions that should never occur:
    1. Self-collision: Robot arm hitting itself
    2. Ground collision: Robot arm hitting the ground/table
    
    Args:
        env: The environment instance.
        robot_cfg: The robot asset configuration.
        collision_sensor_name: Name of the collision sensor.
        self_collision_penalty: Penalty for self-collision.
        ground_collision_penalty: Penalty for ground collision.
        
    Returns:
        Penalty tensor with shape (num_envs,).
    """
    # Get collision sensor data
    if collision_sensor_name not in env.scene.sensors:
        return torch.zeros(env.num_envs, device=env.device)
    
    collision_sensor = env.scene.sensors[collision_sensor_name]
    contact_forces = collision_sensor.data.net_forces_w
    
    # Check for any contact (collision detected)
    collision_detected = torch.norm(contact_forces, dim=-1) > 0.1  # Force threshold
    
    # Apply penalties
    penalty = torch.zeros(env.num_envs, device=env.device)
    
    # Self-collision penalty (more severe)
    self_collision_mask = collision_detected  # Simplified: any collision is treated as severe
    penalty += torch.where(
        self_collision_mask,
        torch.tensor(self_collision_penalty, device=env.device),
        torch.tensor(0.0, device=env.device)
    )
    
    return penalty


def workspace_boundary_penalty(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    ee_body_name: str = "ee_link",
    workspace_limits: dict = None,
    boundary_penalty: float = -10.0,
) -> torch.Tensor:
    """Penalty for robot end-effector leaving safe workspace boundaries.
    
    Args:
        env: The environment instance.
        robot_cfg: The robot asset configuration.
        ee_body_name: Name of the end-effector body.
        workspace_limits: Dictionary with x, y, z limits.
        boundary_penalty: Penalty for leaving workspace.
        
    Returns:
        Penalty tensor with shape (num_envs,).
    """
    if workspace_limits is None:
        workspace_limits = {
            "x": (-0.8, 0.8),   # ±80cm in X
            "y": (-0.8, 0.8),   # ±80cm in Y  
            "z": (0.0, 1.2),    # 0~120cm in Z (above ground)
        }
    
    # Get end-effector position
    robot = env.scene[robot_cfg.name]
    
    if hasattr(robot.data, 'body_pos_w'):
        # Find end-effector body index
        if ee_body_name in robot.data.body_names:
            ee_idx = robot.data.body_names.index(ee_body_name)
            ee_pos = robot.data.body_pos_w[:, ee_idx, :]
        else:
            return torch.zeros(env.num_envs, device=env.device)
    else:
        return torch.zeros(env.num_envs, device=env.device)
    
    # Check workspace boundaries
    penalty = torch.zeros(env.num_envs, device=env.device)
    
    # X boundary check
    x_violation = (ee_pos[:, 0] < workspace_limits["x"][0]) | (ee_pos[:, 0] > workspace_limits["x"][1])
    
    # Y boundary check  
    y_violation = (ee_pos[:, 1] < workspace_limits["y"][0]) | (ee_pos[:, 1] > workspace_limits["y"][1])
    
    # Z boundary check (especially important - don't go below ground!)
    z_violation = (ee_pos[:, 2] < workspace_limits["z"][0]) | (ee_pos[:, 2] > workspace_limits["z"][1])
    
    # Apply penalties
    boundary_violation = x_violation | y_violation | z_violation
    
    penalty += torch.where(
        boundary_violation,
        torch.tensor(boundary_penalty, device=env.device),
        torch.tensor(0.0, device=env.device)
    )
    
    return penalty


def joint_limits_penalty(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    joint_names: list = None,
    limit_margin: float = 0.1,  # 10% margin from actual limits
    penalty_weight: float = -5.0,
) -> torch.Tensor:
    """Penalty for approaching dangerous joint limits.
    
    Args:
        env: The environment instance.
        robot_cfg: The robot asset configuration. 
        joint_names: List of joint names to monitor.
        limit_margin: Safety margin from joint limits (0.0-1.0).
        penalty_weight: Penalty weight for limit violations.
        
    Returns:
        Penalty tensor with shape (num_envs,).
    """
    if joint_names is None:
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    
    robot = env.scene[robot_cfg.name]
    
    # Get current joint positions
    joint_pos = robot.data.joint_pos
    
    # Get joint limits
    joint_limits = robot.data.soft_joint_pos_limits
    
    penalty = torch.zeros(env.num_envs, device=env.device)
    
    # Check each monitored joint
    for joint_name in joint_names:
        if joint_name in robot.data.joint_names:
            joint_idx = robot.data.joint_names.index(joint_name)
            
            current_pos = joint_pos[:, joint_idx]
            lower_limit = joint_limits[:, joint_idx, 0] 
            upper_limit = joint_limits[:, joint_idx, 1]
            
            # Calculate safe range with margin
            range_size = upper_limit - lower_limit
            safe_lower = lower_limit + range_size * limit_margin
            safe_upper = upper_limit - range_size * limit_margin
            
            # Check violations
            lower_violation = current_pos < safe_lower
            upper_violation = current_pos > safe_upper
            
            # Apply penalties
            penalty += torch.where(
                lower_violation | upper_violation,
                torch.tensor(penalty_weight, device=env.device),
                torch.tensor(0.0, device=env.device)
            )
    
    return penalty


def collision_termination(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    collision_sensor_name: str = "collision_sensor",
) -> torch.Tensor:
    """Terminate episode immediately if collision is detected.
    
    Args:
        env: The environment instance.
        robot_cfg: The robot asset configuration.
        collision_sensor_name: Name of the collision sensor.
        
    Returns:
        Boolean tensor indicating termination for each environment.
    """
    # Get collision sensor data
    if collision_sensor_name not in env.scene.sensors:
        return torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    
    collision_sensor = env.scene.sensors[collision_sensor_name]
    contact_forces = collision_sensor.data.net_forces_w
    
    # Check for any significant collision
    collision_detected = torch.norm(contact_forces, dim=-1) > 0.5  # Higher threshold for termination
    
    return collision_detected


def workspace_boundary_termination(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    ee_body_name: str = "ee_link",
    workspace_limits: dict = None,
) -> torch.Tensor:
    """Terminate episode if robot leaves critical workspace boundaries.
    
    Args:
        env: The environment instance.
        robot_cfg: The robot asset configuration.
        ee_body_name: Name of the end-effector body.
        workspace_limits: Dictionary with critical limits.
        
    Returns:
        Boolean tensor indicating termination for each environment.
    """
    if workspace_limits is None:
        workspace_limits = {
            "x": (-1.0, 1.0),   # Critical: ±100cm
            "y": (-1.0, 1.0),   # Critical: ±100cm  
            "z": (-0.05, 1.5),  # Critical: 5cm below ground ~ 150cm
        }
    
    # Get end-effector position
    robot = env.scene[robot_cfg.name]
    
    if hasattr(robot.data, 'body_pos_w'):
        if ee_body_name in robot.data.body_names:
            ee_idx = robot.data.body_names.index(ee_body_name)
            ee_pos = robot.data.body_pos_w[:, ee_idx, :]
        else:
            return torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    else:
        return torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    
    # Check critical boundaries
    x_violation = (ee_pos[:, 0] < workspace_limits["x"][0]) | (ee_pos[:, 0] > workspace_limits["x"][1])
    y_violation = (ee_pos[:, 1] < workspace_limits["y"][0]) | (ee_pos[:, 1] > workspace_limits["y"][1])
    z_violation = (ee_pos[:, 2] < workspace_limits["z"][0]) | (ee_pos[:, 2] > workspace_limits["z"][1])
    
    # Any critical violation terminates episode
    critical_violation = x_violation | y_violation | z_violation
    
    return critical_violation


def premature_block_collision_penalty(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    block_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joints: list[str] = ["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"],
    approach_phase_distance: float = 0.15,  # 15cm 이내는 접근 단계
    grasp_ready_distance: float = 0.06,     # 6cm 이내는 잡기 준비 단계
    collision_penalty: float = -50.0,       # 조기 충돌 처벌
) -> torch.Tensor:
    """Penalty for colliding with block before proper grasp setup.
    
    This prevents the robot from hitting the block during approach.
    Only allows contact when gripper is properly positioned and ready to grasp.
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        block_cfg: The target block configuration.
        robot_cfg: The robot asset configuration.
        gripper_joints: Names of gripper joints.
        approach_phase_distance: Distance defining approach phase.
        grasp_ready_distance: Distance when grasp is allowed.
        collision_penalty: Penalty for premature collision.
        
    Returns:
        Penalty tensor with shape (num_envs,).
    """
    # Get gripper and block positions
    gripper_pos = _get_entity_pos_w(env, gripper_cfg)
    block_pos = _get_entity_pos_w(env, block_cfg)
    
    # Calculate distance to block
    distance_to_block = torch.norm(gripper_pos - block_pos, dim=-1)
    
    # Get gripper state (check if gripper is open)
    robot = env.scene[robot_cfg.name]
    gripper_open = torch.ones(env.num_envs, device=env.device)  # Assume open by default
    
    if hasattr(robot.data, 'joint_pos'):
        gripper_positions = []
        for joint_name in gripper_joints:
            if joint_name in robot.data.joint_names:
                joint_idx = robot.data.joint_names.index(joint_name)
                gripper_positions.append(robot.data.joint_pos[:, joint_idx])
        
        if gripper_positions:
            # Simple heuristic: gripper is "open" if joints are near zero position
            gripper_openness = torch.stack(gripper_positions, dim=-1).abs().mean(dim=-1)
            gripper_open = gripper_openness < 0.2  # Open if joints < 0.2 rad
    
    # Check for block contact/collision
    contact_detected = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    
    # 🔧 NEW: Check collision with ALL robot body parts (not just gripper)
    contact_detected = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    arm_collision = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    
    # Get contact sensor data if available
    if "contact_forces" in env.scene.sensors:
        contact_sensor = env.scene.sensors["contact_forces"]
        contact_forces = contact_sensor.data.net_forces_w
        
        # Detect ANY contact with block (전체 로봇 바디)
        total_contact_force = torch.norm(contact_forces, dim=-1).sum(dim=-1)
        contact_detected = total_contact_force > 0.1
        
        # 🚨 CRITICAL: Detect ARM collision (not gripper)
        # 실제로는 contact sensor body mapping이 필요하지만, 
        # 현재는 gripper 외 모든 충돌을 arm collision으로 간주
        if hasattr(contact_sensor.data, 'body_names') and len(contact_forces.shape) > 1:
            # 여러 바디의 contact force를 체크
            for body_idx in range(contact_forces.shape[1]):
                body_force = torch.norm(contact_forces[:, body_idx], dim=-1)
                if body_idx < 7:  # link_0 ~ link_6 (arm bodies)
                    arm_collision |= (body_force > 0.1)
        else:
            # Simplified: 강한 충돌이면 arm collision으로 추정
            arm_collision = total_contact_force > 1.0
    
    # Penalty logic: collision is bad unless we're ready to grasp
    penalty = torch.zeros(env.num_envs, device=env.device)
    
    # Phase 1: Far approach (> 15cm) - No penalty for collision (shouldn't happen anyway)
    far_approach = distance_to_block > approach_phase_distance
    
    # Phase 2: Close approach (6-15cm) - Penalty for collision (not ready to grasp yet)
    close_approach = (distance_to_block <= approach_phase_distance) & (distance_to_block > grasp_ready_distance)
    
    # Phase 3: Grasp ready (< 6cm) - Allow collision only if gripper is open and properly positioned
    grasp_ready = distance_to_block <= grasp_ready_distance
    
    # Apply penalties with ARM collision priority
    # 🚨 CRITICAL: ARM collision is ALWAYS severely penalized
    arm_collision_penalty = arm_collision & contact_detected
    penalty += torch.where(
        arm_collision_penalty,
        torch.tensor(collision_penalty * 2.0, device=env.device),  # 팔 충돌은 2배 페널티
        torch.tensor(0.0, device=env.device)
    )
    
    # Penalty in close approach phase if collision detected (gripper only)
    gripper_premature_collision = close_approach & contact_detected & (~arm_collision)
    penalty += torch.where(
        gripper_premature_collision,
        torch.tensor(collision_penalty, device=env.device),
        torch.tensor(0.0, device=env.device)
    )

    # Penalty in grasp ready phase if collision but gripper not open (gripper only)
    improper_grasp_collision = grasp_ready & contact_detected & (~gripper_open) & (~arm_collision)
    penalty += torch.where(
        improper_grasp_collision, 
        torch.tensor(collision_penalty * 0.5, device=env.device),  # Lighter penalty
        torch.tensor(0.0, device=env.device)
    )
    
    return penalty


def safe_approach_reward(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    block_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    safe_approach_height: float = 0.1,      # 10cm above block is safe
    approach_corridor_radius: float = 0.05,  # 5cm radius approach corridor
    reward_weight: float = 10.0,
) -> torch.Tensor:
    """Reward for safe approach from above without collision.
    
    Encourages robot to approach block from above in a safe corridor.
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        block_cfg: The target block configuration.
        robot_cfg: The robot asset configuration.
        safe_approach_height: Minimum safe height above block.
        approach_corridor_radius: Radius of safe approach corridor.
        reward_weight: Weight for safe approach reward.
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get positions
    gripper_pos = _get_entity_pos_w(env, gripper_cfg) 
    block_pos = _get_entity_pos_w(env, block_cfg)
    
    # Calculate relative position
    rel_pos = gripper_pos - block_pos
    
    # XY distance from block (lateral distance)
    xy_distance = torch.norm(rel_pos[:, :2], dim=-1)
    
    # Height above block
    height_above = rel_pos[:, 2]  # Z difference
    
    # Reward components
    reward = torch.zeros(env.num_envs, device=env.device)
    
    # 1. Reward for being above the block (positive Z)
    above_block = height_above > 0.0
    height_reward = torch.clamp(height_above / safe_approach_height, 0.0, 2.0)  # Max 2x reward
    
    # 2. Reward for being within approach corridor (small XY distance)
    corridor_reward = torch.exp(-5.0 * xy_distance / approach_corridor_radius)
    
    # 3. Combined safe approach reward (only when above block)
    safe_approach = above_block & (xy_distance < approach_corridor_radius * 2.0)  # Extended corridor
    
    reward += torch.where(
        safe_approach,
        height_reward * corridor_reward * reward_weight,
        torch.tensor(0.0, device=env.device)
    )
    
    return reward


def premature_block_collision_termination(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    block_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    collision_force_threshold: float = 2.0,  # Strong collision threshold
) -> torch.Tensor:
    """Terminate episode for severe premature collision with block.
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        block_cfg: The target block configuration.
        robot_cfg: The robot asset configuration.
        collision_force_threshold: Force threshold for termination.
        
    Returns:
        Boolean tensor indicating termination for each environment.
    """
    # Get gripper and block positions  
    gripper_pos = _get_entity_pos_w(env, gripper_cfg)
    block_pos = _get_entity_pos_w(env, block_cfg)
    
    # Calculate distance
    distance_to_block = torch.norm(gripper_pos - block_pos, dim=-1)
    
    # 🔧 NEW: Check for strong collision with ANY robot body part
    strong_collision = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    arm_collision = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    
    if "contact_forces" in env.scene.sensors:
        contact_sensor = env.scene.sensors["contact_forces"]
        contact_forces = contact_sensor.data.net_forces_w
        
        # Check total collision force (all robot bodies)
        total_collision_force = torch.norm(contact_forces, dim=-1).sum(dim=-1)
        strong_collision = total_collision_force > collision_force_threshold
        
        # 🚨 CRITICAL: ARM collision = immediate termination (lower threshold)
        if hasattr(contact_sensor.data, 'body_names') and len(contact_forces.shape) > 1:
            # Check individual body forces for arm collision detection
            for body_idx in range(min(7, contact_forces.shape[1])):  # link_0 ~ link_6
                body_force = torch.norm(contact_forces[:, body_idx], dim=-1)
                arm_collision |= (body_force > (collision_force_threshold * 0.5))  # 팔 충돌은 더 낮은 임계값
        else:
            # Simplified: Strong force usually indicates arm collision
            arm_collision = total_collision_force > (collision_force_threshold * 1.5)
    
    # Terminate conditions:
    # 1. ARM collision = immediate termination (any distance)
    # 2. Strong collision during approach phase (not grasping)
    approach_phase = distance_to_block > 0.04  # Not in final grasping distance
    
    terminate = (
        arm_collision |  # 🚨 팔 충돌은 즉시 종료
        (strong_collision & approach_phase)  # 접근 중 강한 충돌도 종료
    )
    
    return terminate


def ground_collision_penalty(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    gripper_cfg: SceneEntityCfg,
    collision_penalty: float = -100.0,  # 바닥 충돌은 매우 강한 페널티
    min_height_threshold: float = 0.02,  # 2cm 이하는 위험한 높이
) -> torch.Tensor:
    """🚨 CRITICAL: Penalty for robot/gripper collision with ground.
    
    Prevents robot from hitting the ground/table surface.
    
    Args:
        env: The environment instance.
        robot_cfg: The robot asset configuration.
        gripper_cfg: The gripper entity configuration.
        collision_penalty: Penalty for ground collision.
        min_height_threshold: Minimum safe height above ground.
        
    Returns:
        Penalty tensor with shape (num_envs,).
    """
    penalty = torch.zeros(env.num_envs, device=env.device)
    
    # Get gripper height (lowest point of gripper fingers for accurate ground detection)
    gripper_height = _get_gripper_lowest_point(env, gripper_cfg)
    
    # Check for ground collision using collision sensor
    ground_collision_detected = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    
    if "collision_sensor" in env.scene.sensors:
        collision_sensor = env.scene.sensors["collision_sensor"]
        collision_forces = collision_sensor.data.net_forces_w
        
        # Detect ground collision (any significant force with ground)
        total_collision_force = torch.norm(collision_forces, dim=-1).sum(dim=-1)
        ground_collision_detected = total_collision_force > 0.5  # 0.5N threshold for ground contact
    
    # Also penalize being too close to ground (preventive measure)
    too_low = gripper_height < min_height_threshold
    
    # Apply penalties
    # Severe penalty for actual collision
    penalty += torch.where(
        ground_collision_detected,
        torch.tensor(collision_penalty, device=env.device),
        torch.tensor(0.0, device=env.device)
    )
    
    # Moderate penalty for being dangerously low
    penalty += torch.where(
        too_low & (~ground_collision_detected),
        torch.tensor(collision_penalty * 0.3, device=env.device),  # 30% of collision penalty
        torch.tensor(0.0, device=env.device)
    )
    
    return penalty


def ground_collision_termination(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    gripper_cfg: SceneEntityCfg,
    collision_force_threshold: float = 1.0,  # Ground collision termination threshold
) -> torch.Tensor:
    """Terminate episode for ground collision.
    
    Args:
        env: The environment instance.
        robot_cfg: The robot asset configuration.
        gripper_cfg: The gripper entity configuration.
        collision_force_threshold: Force threshold for termination.
        
    Returns:
        Boolean tensor indicating termination for each environment.
    """
    terminate = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    
    # Check for ground collision
    if "collision_sensor" in env.scene.sensors:
        collision_sensor = env.scene.sensors["collision_sensor"]
        collision_forces = collision_sensor.data.net_forces_w
        
        # Check for strong ground collision
        total_collision_force = torch.norm(collision_forces, dim=-1).sum(dim=-1)
        strong_ground_collision = total_collision_force > collision_force_threshold
        
        # Also check if gripper is below safe minimum height (lowest finger point)
        gripper_height = _get_gripper_lowest_point(env, gripper_cfg)
        critically_low = gripper_height < -0.01  # Below ground level
        
        terminate = strong_ground_collision | critically_low
    
    return terminate


def safe_height_reward(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    min_safe_height: float = 0.03,  # 3cm minimum safe height
    optimal_height: float = 0.15,   # 15cm optimal working height
    reward_scale: float = 2.0,
) -> torch.Tensor:
    """Reward for maintaining safe height above ground.
    
    Encourages robot to operate at appropriate heights to avoid ground collision.
    
    Args:
        env: The environment instance.
        gripper_cfg: The gripper entity configuration.
        min_safe_height: Minimum safe height above ground.
        optimal_height: Optimal working height for maximum reward.
        reward_scale: Scaling factor for reward.
        
    Returns:
        Reward tensor with shape (num_envs,).
    """
    # Get gripper height (lowest point of gripper fingers for accurate safety)
    gripper_height = _get_gripper_lowest_point(env, gripper_cfg)
    
    # Calculate height-based reward
    reward = torch.zeros(env.num_envs, device=env.device)
    
    # Case 1: Below minimum safe height - strong penalty gradient
    below_safe = gripper_height < min_safe_height
    unsafe_penalty = -reward_scale * torch.exp(10.0 * (min_safe_height - gripper_height))
    reward = torch.where(below_safe, unsafe_penalty, reward)
    
    # Case 2: Above safe height - positive reward with optimal point
    above_safe = gripper_height >= min_safe_height
    
    # Gaussian reward centered at optimal height
    height_diff = torch.abs(gripper_height - optimal_height)
    safe_reward = reward_scale * torch.exp(-5.0 * height_diff / optimal_height)
    
    reward = torch.where(above_safe, safe_reward, reward)
    
    return reward
