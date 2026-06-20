"""
path_generator.py — 2D 판 샌딩 경로 생성 알고리즘
지원 패턴: raster(지그재그), spiral(나선형), contour(테두리축소)
"""
from __future__ import annotations
import math
from typing import List, Tuple

from simple_sanding.sanding_config import (
    CORNER_JOINTS, NUM_JOINTS,
    PANEL_HEIGHT, PANEL_WIDTH,
    SANDING_RESOLUTION, SANDING_STEP,
)


def bilinear_to_joints(u: float, v: float) -> List[float]:
    """(u,v) ∈ [0,1]² → 관절 각도 (쌍선형 보간)."""
    q00 = CORNER_JOINTS[(0, 0)]
    q10 = CORNER_JOINTS[(1, 0)]
    q01 = CORNER_JOINTS[(0, 1)]
    q11 = CORNER_JOINTS[(1, 1)]
    return [
        q00[i]*(1-u)*(1-v) + q10[i]*u*(1-v) + q01[i]*(1-u)*v + q11[i]*u*v
        for i in range(NUM_JOINTS)
    ]


def generate_raster_uv(width=PANEL_WIDTH, height=PANEL_HEIGHT,
                        step=SANDING_STEP, resolution=SANDING_RESOLUTION
                        ) -> List[Tuple[float, float]]:
    """지그재그 래스터 스캔 (u,v) 경로."""
    path = []
    num_rows = max(1, int(math.ceil(height / step)) + 1)
    num_cols = max(1, int(math.ceil(width / resolution)) + 1)
    for row in range(num_rows):
        v = min(row * step / height, 1.0) if height > 0 else 0.0
        cols = range(num_cols) if row % 2 == 0 else range(num_cols-1, -1, -1)
        for col in cols:
            u = min(col * resolution / width, 1.0) if width > 0 else 0.0
            path.append((u, v))
    return path


def generate_spiral_uv(width=PANEL_WIDTH, height=PANEL_HEIGHT,
                        step=SANDING_STEP, resolution=SANDING_RESOLUTION
                        ) -> List[Tuple[float, float]]:
    """나선형 경로 (바깥→안쪽)."""
    path = []
    margin = 0.0
    while margin < 0.5:
        u_min, u_max = margin, 1.0 - margin
        v_min, v_max = margin, 1.0 - margin
        if u_min >= u_max or v_min >= v_max:
            path.append((0.5, 0.5))
            break
        # 하단 왼→오
        u = u_min
        while u <= u_max:
            path.append((min(u, u_max), v_min)); u += resolution / width
        # 우측 아래→위
        v = v_min
        while v <= v_max:
            path.append((u_max, min(v, v_max))); v += resolution / height
        # 상단 오→왼
        u = u_max
        while u >= u_min:
            path.append((max(u, u_min), v_max)); u -= resolution / width
        # 좌측 위→아래
        v = v_max
        while v >= v_min + step / height:
            path.append((u_min, max(v, v_min))); v -= resolution / height
        margin += step / min(width, height) * 0.5
    return path


def generate_contour_uv(width=PANEL_WIDTH, height=PANEL_HEIGHT,
                         step=SANDING_STEP, resolution=SANDING_RESOLUTION
                         ) -> List[Tuple[float, float]]:
    """테두리→중심 축소 경로."""
    path = []
    layer = 0
    while True:
        ou = layer * step / width
        ov = layer * step / height
        u_min, u_max = ou, 1.0 - ou
        v_min, v_max = ov, 1.0 - ov
        if u_min >= u_max or v_min >= v_max:
            break
        # 하단
        u = u_min
        while u <= u_max:
            path.append((min(u, u_max), v_min)); u += resolution / width
        # 우측
        v = v_min + resolution / height
        while v <= v_max:
            path.append((u_max, min(v, v_max))); v += resolution / height
        # 상단 역방향
        u = u_max - resolution / width
        while u >= u_min:
            path.append((max(u, u_min), v_max)); u -= resolution / width
        # 좌측 역방향
        v = v_max - resolution / height
        while v > v_min:
            path.append((u_min, max(v, v_min))); v -= resolution / height
        layer += 1
    return path


_GENERATORS = {
    'raster': generate_raster_uv,
    'spiral': generate_spiral_uv,
    'contour': generate_contour_uv,
}


def generate_path(pattern='raster', width=PANEL_WIDTH, height=PANEL_HEIGHT,
                  step=SANDING_STEP, resolution=SANDING_RESOLUTION
                  ) -> List[List[float]]:
    """패턴 경로 → 관절 각도 리스트 반환."""
    gen = _GENERATORS.get(pattern)
    if gen is None:
        raise ValueError(f"Unknown pattern '{pattern}'. Use: {list(_GENERATORS)}")
    uv = gen(width, height, step, resolution)
    return [bilinear_to_joints(u, v) for u, v in uv]


def generate_uv_path(pattern='raster', **kw) -> List[Tuple[float, float]]:
    """(u,v) 좌표만 반환 (시각화용)."""
    gen = _GENERATORS.get(pattern)
    if gen is None:
        raise ValueError(f"Unknown pattern '{pattern}'. Use: {list(_GENERATORS)}")
    return gen(**kw)


def main():
    """CLI 실행 시 경로 생성 + matplotlib 시각화."""
    import sys
    pattern = sys.argv[1] if len(sys.argv) > 1 else 'raster'
    print(f"[path_generator] pattern={pattern}")
    uv = generate_uv_path(pattern=pattern)
    jp = generate_path(pattern=pattern)
    print(f"  waypoints: {len(uv)}")
    print(f"  first uv: {uv[0]},  last uv: {uv[-1]}")
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        us, vs = zip(*uv)
        axes[0].plot(us, vs, 'b-', lw=0.5, alpha=0.7)
        axes[0].plot(us[0], vs[0], 'go', ms=10, label='Start')
        axes[0].plot(us[-1], vs[-1], 'r^', ms=10, label='End')
        axes[0].set(xlabel='u', ylabel='v', title=f'{pattern} ({len(uv)} pts)')
        axes[0].set_xlim(-0.05, 1.05); axes[0].set_ylim(-0.05, 1.05)
        axes[0].set_aspect('equal'); axes[0].legend(); axes[0].grid(True, alpha=0.3)
        for j in range(NUM_JOINTS):
            axes[1].plot([wp[j] for wp in jp], label=f'J{j+1}')
        axes[1].set(xlabel='index', ylabel='rad', title='Joint angles')
        axes[1].legend(); axes[1].grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('/tmp/sanding_path_preview.png', dpi=150)
        print("  saved: /tmp/sanding_path_preview.png")
        plt.show()
    except ImportError:
        print("  [warn] matplotlib not found")


if __name__ == '__main__':
    main()
