"""
sanding_config.py
─────────────────
2D 판(panel) 샌딩 경로 생성에 필요한 모든 설정값을 모아 둔 모듈.
값을 바꾸면 다른 코드에 자동 반영됩니다.
"""

import math

# ============================================================
# 판(Panel) 물리 파라미터
# ============================================================
PANEL_WIDTH = 0.30       # 판 가로 길이 (m)
PANEL_HEIGHT = 0.20      # 판 세로 길이 (m)

# 로봇 베이스 프레임 기준, 판의 좌측 하단 코너 좌표 (x, y, z)
PANEL_ORIGIN = [0.35, -0.15, 0.15]

# 판의 법선 벡터 방향 (단위 벡터) — 기본값은 z-up 수평 판
PANEL_NORMAL = [0.0, 0.0, 1.0]

# ============================================================
# 샌딩 경로 파라미터
# ============================================================
SANDING_STEP = 0.02      # 줄 간격 (m) — 래스터 스캔 시 행 간 거리
SANDING_RESOLUTION = 0.005  # 한 줄 내 샘플 간격 (m)
SANDING_DEPTH = 0.002    # 판 표면 아래로 누르는 깊이 (m)

# 경로 패턴: 'raster', 'spiral', 'contour'
DEFAULT_PATTERN = 'raster'

# ============================================================
# 로봇 제어 파라미터
# ============================================================
CONTROL_RATE_HZ = 50     # 제어 주기 (Hz)
CONTROL_DT = 1.0 / CONTROL_RATE_HZ

# 보간 설정
INTERPOLATION_POINTS = 50   # 웨이포인트 사이 보간 점 수
MAX_JOINT_VELOCITY = 1.0    # 관절 최대 속도 (rad/s)
MAX_JOINT_ACCELERATION = 2.0  # 관절 최대 가속도 (rad/s^2)

# ============================================================
# UR10 관절 설정
# ============================================================
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

NUM_JOINTS = len(JOINT_NAMES)

# 관절 한계 (rad) — UR10 (모든 관절 ±2π)
JOINT_LIMITS_LOWER = [
    math.radians(-360),  # shoulder_pan
    math.radians(-360),  # shoulder_lift
    math.radians(-360),  # elbow
    math.radians(-360),  # wrist_1
    math.radians(-360),  # wrist_2
    math.radians(-360),  # wrist_3
]

JOINT_LIMITS_UPPER = [
    math.radians(360),   # shoulder_pan
    math.radians(360),   # shoulder_lift
    math.radians(360),   # elbow
    math.radians(360),   # wrist_1
    math.radians(360),   # wrist_2
    math.radians(360),   # wrist_3
]

# ============================================================
# 판 네 코너에 해당하는 관절 각도 (Joint Space 매핑)
# ============================================================
# 로봇을 수동으로 판 네 코너에 위치시키고 관절 각도를 기록합니다.
# 이 값을 기반으로 (u, v) → 관절각도 매핑(쌍선형 보간)을 수행합니다.
#
#  (u=0,v=1) ─────── (u=1,v=1)
#      │                 │
#      │     PANEL        │
#      │                 │
#  (u=0,v=0) ─────── (u=1,v=0)
#
CORNER_JOINTS = {
    # (u, v) : [j1, j2, j3, j4, j5, j6] (rad)
    (0, 0): [0.0,  -1.0,  1.0,  0.0,  1.57, 0.0],   # 좌측 하단
    (1, 0): [0.3,  -1.0,  1.0,  0.0,  1.57, 0.0],   # 우측 하단
    (0, 1): [0.0,  -1.1,  1.1,  0.0,  1.57, 0.0],   # 좌측 상단
    (1, 1): [0.3,  -1.1,  1.1,  0.0,  1.57, 0.0],   # 우측 상단
}

# ============================================================
# ROS 토픽 이름
# ============================================================
TOPIC_JOINT_COMMAND = '/joint_command'
TOPIC_JOINT_STATES = '/joint_states'
TOPIC_SANDING_PROGRESS = '/sanding_progress'
