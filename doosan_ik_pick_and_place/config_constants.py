# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Configuration constants for E0509 Pick and Place environment.
Centralized parameter management to avoid duplication.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass
class GripperConfig:
    """Gripper-related configuration constants."""
    
    # Gripper hierarchy structure
    # ee_link → rh_p12_rn_base → rh_p12_rn_l1/r1 → rh_p12_rn_l2/r2
    EE_LINK: str = "ee_link"                     # 로봇 엔드 이펙터 링크
    GRIPPER_BASE: str = "rh_p12_rn_base"         # 그리퍼 베이스
    
    # Left finger hierarchy: ee_link -> rh_p12_rn_base -> rh_p12_rn_l1 -> rh_p12_rn_l2
    LEFT_FINGER_BASE: str = "rh_p12_rn_l1"      # 왼쪽 집게 베이스
    LEFT_FINGER_TIP: str = "rh_p12_rn_l2"       # 왼쪽 집게 끝단 (실제 접촉점)
    
    # Right finger hierarchy: ee_link -> rh_p12_rn_base -> rh_p12_rn_r1 -> rh_p12_rn_r2  
    RIGHT_FINGER_BASE: str = "rh_p12_rn_r1"     # 오른쪽 집게 베이스
    RIGHT_FINGER_TIP: str = "rh_p12_rn_r2"      # 오른쪽 집게 끝단 (실제 접촉점)
    
    # Finger tips (끝단만 - 실제 접촉 및 높이 측정용)
    FINGER_TIPS: List[str] = field(default_factory=lambda: ["rh_p12_rn_l2", "rh_p12_rn_r2"])
    
    # All finger bodies (베이스 + 전체 - 완전한 충돌 감지용)
    ALL_FINGER_BODIES: List[str] = field(default_factory=lambda: [
        "rh_p12_rn_base",                 # 그리퍼 베이스
        "rh_p12_rn_l1", "rh_p12_rn_l2",  # 왼쪽 집게 전체
        "rh_p12_rn_r1", "rh_p12_rn_r2"   # 오른쪽 집게 전체
    ])
    
    # Gripper joint names (제어용)
    JOINT_NAMES: List[str] = field(default_factory=lambda: ["rh_l1", "rh_l2", "rh_p12_rn", "rh_r2"])
    
    # Gripper state thresholds
    OPEN_THRESHOLD: float = 0.3      # 그리퍼 열림 임계값
    CLOSE_THRESHOLD: float = 0.15    # 그리퍼 닫힘 임계값


@dataclass  
class DistanceConfig:
    """Distance and threshold configuration constants."""
    
    # Approach distances
    FAR_APPROACH: float = 0.15       # 원거리 접근 (15cm)
    CLOSE_APPROACH: float = 0.06     # 근거리 접근 (6cm) 
    GRASP_DISTANCE: float = 0.04     # 그립 거리 (4cm)
    SUCCESS_THRESHOLD: float = 0.03   # 성공 임계값 (3cm)
    
    # Collision thresholds
    CONTACT_THRESHOLD: float = 0.04   # 접촉 감지 임계값
    COLLISION_THRESHOLD: float = 2.0   # 충돌 힘 임계값  
    GROUND_COLLISION_THRESHOLD: float = 1.0  # 지면 충돌 힘 임계값
    
    # Maximum distances for rewards
    MAX_DISTANCE: float = 0.5        # 최대 거리 (50cm)
    MAX_APPROACH_DISTANCE: float = 0.5  # 최대 접근 거리
    
    # Exponential reward parameters
    EXPONENTIAL_ALPHA: float = 15.0     # 지수 보상 알파 값


@dataclass
class HeightConfig:
    """Height-related configuration constants."""
    
    # Safety heights
    MIN_SAFE_HEIGHT: float = 0.03    # 최소 안전 높이 (3cm)
    OPTIMAL_HEIGHT: float = 0.15     # 최적 작업 높이 (15cm)
    APPROACH_HEIGHT: float = 0.10    # 안전 접근 높이 (10cm)
    ALIGNMENT_HEIGHT: float = 0.12   # 그리퍼 정렬 높이 (12cm) - 블록 위 충분한 간격
    
    # Lift heights
    MIN_LIFT_HEIGHT: float = 0.10    # 최소 리프팅 높이 (10cm)
    
    # Danger zones
    DANGER_HEIGHT: float = 0.02      # 위험 높이 (2cm)
    CRITICAL_HEIGHT: float = -0.01   # 치명적 높이 (-1cm, 바닥 아래)


@dataclass
class PenaltyConfig:
    """Penalty and reward scaling configuration."""
    
    # Collision penalties
    GROUND_COLLISION: float = -100.0     # 바닥 충돌 페널티
    ARM_COLLISION: float = -100.0        # 팔 충돌 페널티
    BLOCK_COLLISION: float = -50.0       # 블록 충돌 페널티
    SELF_COLLISION: float = -100.0       # 자기 충돌 페널티
    
    # Safety penalties
    HEIGHT_DANGER: float = -30.0         # 위험 높이 페널티
    WORKSPACE_BOUNDARY: float = -10.0    # 작업영역 이탈 페널티
    JOINT_LIMIT: float = -5.0           # 관절 한계 페널티


@dataclass  
class WorkspaceConfig:
    """Workspace boundary configuration."""
    
    # Workspace limits (x, y, z) in meters
    X_LIMITS: Tuple[float, float] = (-0.8, 0.8)   # ±80cm
    Y_LIMITS: Tuple[float, float] = (-0.8, 0.8)   # ±80cm  
    Z_LIMITS: Tuple[float, float] = (0.06, 1.2)   # 6cm ~ 120cm
    
    # Critical limits for termination
    X_LIMITS_CRITICAL: Tuple[float, float] = (-1.0, 1.0)   # ±100cm 임계
    Y_LIMITS_CRITICAL: Tuple[float, float] = (-1.0, 1.0)   # ±100cm 임계  
    Z_LIMITS_CRITICAL: Tuple[float, float] = (0.00, 1.5)   # 0cm ~ 150cm 임계
    
    # Safety margins
    BOUNDARY_MARGIN: float = 0.05    # 5cm 경계 여유


@dataclass
class TimingConfig:
    """Timing-related configuration."""
    
    # Hold times
    GRASP_HOLD_TIME: int = 20        # 그립 유지 시간 (steps)
    LIFT_HOLD_TIME: int = 30         # 리프트 유지 시간 (steps)
    
    # Sequence timeouts
    APPROACH_TIMEOUT: int = 200      # 접근 타임아웃
    GRASP_TIMEOUT: int = 100         # 그립 타임아웃
    LIFT_TIMEOUT: int = 150          # 리프트 타임아웃


@dataclass
class RewardWeights:
    """Reward weight configuration."""
    
    # Primary rewards
    DISTANCE_APPROACH: float = 20.0      # 거리 기반 접근
    PRECISION_APPROACH: float = 15.0     # 정밀 접근
    GRIPPER_ALIGNMENT: float = 30.0      # 그리퍼 정렬
    
    # Sequential task rewards  
    PRE_GRASP: float = 25.0             # 그립 준비
    GRASP_SUCCESS: float = 100.0        # 그립 성공
    LIFT_SUCCESS: float = 200.0         # 리프트 성공
    COMPLETE_TASK: float = 500.0        # 태스크 완료
    
    # Safety rewards
    SAFE_APPROACH: float = 15.0         # 안전 접근
    SAFE_HEIGHT: float = 5.0            # 안전 높이
    
    # Success bonuses
    SUCCESS_BONUS: float = 10.0         # 성공 보너스
    
    # Orientation and movement
    ORIENTATION_WEIGHT: float = 0.7     # 방향 가중치
    
    # Penalties (negative weights handled in penalty config)
    ACTION_PENALTY: float = -0.05       # 액션 페널티
    VELOCITY_PENALTY: float = -0.0001   # 속도 페널티


# Consolidated configuration class
@dataclass
class E0509Config:
    """Master configuration class combining all constants."""
    
    gripper: GripperConfig = field(default_factory=GripperConfig)
    distance: DistanceConfig = field(default_factory=DistanceConfig)
    height: HeightConfig = field(default_factory=HeightConfig)
    penalty: PenaltyConfig = field(default_factory=PenaltyConfig)
    workspace: WorkspaceConfig = field(default_factory=WorkspaceConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    weights: RewardWeights = field(default_factory=RewardWeights)


# Global configuration instance
CONFIG = E0509Config()