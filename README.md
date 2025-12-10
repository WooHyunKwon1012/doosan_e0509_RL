# doosan_e0509_RL
# E0509 Pick and Place Environment with IK Control

DOOSAN E0509 로봇을 위한 강화학습 환경으로, cabinet 프로젝트와 동일한 방식으로 IK 제어를 통합했습니다.

## 🚀 환경 구조

### 제어 방식 (3가지)

1. **관절 제어** (`joint_pos_env_cfg.py`)
   - 직접 관절 각도 제어
   - 액션 공간: 6차원 (arm) + 4차원 (gripper)

2. **절대 IK 제어** (`ik_abs_env_cfg.py`)
   - 절대 좌표계에서 엔드이펙터 위치/자세 제어
   - 액션 공간: 6차원 (pose) + 4차원 (gripper)

3. **상대 IK 제어** (`ik_rel_env_cfg.py`)
   - 증분적 엔드이펙터 움직임 제어
   - 액션 공간: 6차원 (pose delta) + 4차원 (gripper)

### 등록된 환경들

```python
# Joint Position Control
"Isaac-Pick-Place-E0509-v0"
"Isaac-Pick-Place-E0509-Play-v0"

# Absolute IK Control  
"Isaac-Pick-Place-E0509-IK-Abs-v0"
"Isaac-Pick-Place-E0509-IK-Abs-Play-v0"

# Relative IK Control
"Isaac-Pick-Place-E0509-IK-Rel-v0" 
"Isaac-Pick-Place-E0509-IK-Rel-Play-v0"
```

## 🔧 주요 특징

### IK 제어 통합
- **차분 IK (Differential IK)**: 매 timestep마다 작은 증분 움직임
- **DLS 솔버**: 안정적인 특이점 처리
- **높은 PD 게인**: IK 추적 성능 향상

### FrameTransformer 설정
```python
self.scene.ee_frame = FrameTransformerCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link",
    target_frames=[
        # TCP (Tool Center Point)
        FrameTransformerCfg.FrameCfg(
            prim_path="{ENV_REGEX_NS}/Robot/link_6",
            name="ee_tcp",
            offset=OffsetCfg(pos=(0.0, 0.0, 0.15)),
        ),
        # Gripper fingers
        ...
    ],
)
```

### 계층적 제어 구조
```
RL 에이전트 (고수준 정책)
    ↓ 
IK 솔버 (중간 제어기) ← IK 환경에서만
    ↓
PD 제어기 (저수준 제어기)
    ↓
로봇 관절
```

## 🎯 강화학습과 IK의 결합

### 왜 이 방식이 효과적인가?

1. **학습 효율성**: 작업공간 제어가 관절 제어보다 직관적
2. **안정성**: IK 솔버가 특이점과 관절 한계를 자동 처리
3. **전이성**: 다른 로봇으로 정책 전이 용이
4. **유연성**: RL이 여전히 경로와 전략을 학습

### 실제 동작 과정

```python
# 매 timestep(0.005초)마다:

# 1. RL 에이전트가 작은 움직임 결정
action = [0.01, -0.02, 0.03, 0.0, 0.1, 0.0]  # [Δx, Δy, Δz, Δroll, Δpitch, Δyaw]

# 2. IK 솔버가 관절 증분으로 변환  
joint_delta = ik_solver.solve(current_pose, action)

# 3. PD 제어기가 관절을 목표로 이동
new_joint_pos = current_joint_pos + joint_delta

# 4. 다음 스텝에서 RL이 새로운 액션 결정
```

## 🏗️ 파일 구조

```
doosan_pick_place/
├── __init__.py
├── pick_place_env_cfg.py        # 기본 환경 설정
├── mdp/
│   ├── __init__.py
│   ├── observations.py          # 관측 함수들
│   └── rewards.py              # 보상 함수들
└── config/
    └── e0509/
        ├── __init__.py         # 환경 등록
        ├── joint_pos_env_cfg.py  # 관절 제어
        ├── ik_abs_env_cfg.py     # 절대 IK 제어
        ├── ik_rel_env_cfg.py     # 상대 IK 제어
        └── agents/
            ├── __init__.py
            └── rsl_rl_ppo_cfg.py # PPO 설정
```

## 💡 사용 예시

```python
import gymnasium as gym

# 상대 IK 제어 환경 생성
env = gym.make("Isaac-Pick-Place-E0509-IK-Rel-v0", num_envs=1024)

# 학습 또는 평가 진행
obs, _ = env.reset()
for _ in range(1000):
    # RL 에이전트가 작업공간 좌표로 액션 출력
    action = policy(obs)  # 6차원: [Δx, Δy, Δz, Δroll, Δpitch, Δyaw]
    
    # IK 솔버가 자동으로 관절 각도로 변환
    obs, reward, terminated, truncated, info = env.step(action)
```

