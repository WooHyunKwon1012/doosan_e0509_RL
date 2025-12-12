#!/usr/bin/env python3
"""
GUI Configuration for Joint Pose Sequencer
모든 GUI 치수, 색상, 스타일 설정을 관리
"""

# ============================================================================
# Window Settings
# ============================================================================
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 750
WINDOW_TITLE = 'Joint Pose Sequencer for MoveIt'

# ============================================================================
# List Widget Settings
# ============================================================================
LIST_MIN_HEIGHT = 250
LIST_ALTERNATING_COLORS = True

# ============================================================================
# Button Settings
# ============================================================================
BUTTON_HEIGHT = 32
BUTTON_ARROW_WIDTH = 40  # ▲▼ 버튼 폭
BUTTON_SPACING = 5

# ============================================================================
# Layout Settings
# ============================================================================
LAYOUT_SPACING = 5
LAYOUT_MARGIN_TOP = 5
LAYOUT_MARGIN_BOTTOM = 0
LIST_CONTAINER_SPACING = 10

# ============================================================================
# Progress Bar Settings
# ============================================================================
PROGRESS_BAR_HEIGHT = 25

# ============================================================================
# Font Settings
# ============================================================================
FONT_SIZE_SECTION_LABEL = '11pt'  # 섹션 제목 (Playback Sequence, Status 등)

# ============================================================================
# Padding Settings
# ============================================================================
PADDING_SECTION_TOP = 10  # 섹션 사이 상단 여백
PADDING_STATUS_LABEL = 8  # 상태 라벨 내부 패딩

# ============================================================================
# Styles (CSS)
# ============================================================================
STYLE_SECTION_LABEL = f"font-size: {FONT_SIZE_SECTION_LABEL}; padding: 5px;"

STYLE_STATUS_LABEL = f"padding: {PADDING_STATUS_LABEL}px; border: 1px solid #ccc; background-color: #f0f0f0;"

# ============================================================================
# Duration Settings
# ============================================================================
DURATION_MIN = 0.1
DURATION_MAX = 100.0
DURATION_STEP = 0.1
DURATION_DEFAULT = 2.0

# ============================================================================
# Joint Settings
# ============================================================================
JOINT_COUNT = 6
JOINT_ANGLE_MIN = -180.0  # degrees
JOINT_ANGLE_MAX = 180.0   # degrees
JOINT_ANGLE_STEP = 1.0
JOINT_ANGLE_DECIMALS = 2

# ============================================================================
# Tooltip Messages
# ============================================================================
TOOLTIP_MOVE_UP = 'Move Up'
TOOLTIP_MOVE_DOWN = 'Move Down'
TOOLTIP_UPDATE = 'Update from Current State'
TOOLTIP_COPY = 'Copy selected pose in sequence'
