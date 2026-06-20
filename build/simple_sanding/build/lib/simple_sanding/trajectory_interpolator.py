"""
trajectory_interpolator.py — 웨이포인트 간 부드러운 보간기

보간 방식:
  1. linear   — 선형 보간 (가장 단순)
  2. cubic    — 3차 스플라인 보간 (부드러운 곡선)
  3. trapezoid — 사다리꼴 속도 프로파일 (가감속 포함)
"""
from __future__ import annotations
import math
from typing import List

from simple_sanding.sanding_config import (
    NUM_JOINTS,
    INTERPOLATION_POINTS,
    MAX_JOINT_VELOCITY,
    MAX_JOINT_ACCELERATION,
    CONTROL_DT,
)


def linear_interpolation(
    start: List[float],
    end: List[float],
    num_points: int = INTERPOLATION_POINTS,
) -> List[List[float]]:
    """두 관절 위치 사이를 선형 보간."""
    points = []
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        pt = [s + (e - s) * t for s, e in zip(start, end)]
        points.append(pt)
    return points


def cubic_hermite(p0, p1, m0, m1, t):
    """1D 3차 에르미트 보간."""
    t2 = t * t
    t3 = t2 * t
    h00 = 2*t3 - 3*t2 + 1
    h10 = t3 - 2*t2 + t
    h01 = -2*t3 + 3*t2
    h11 = t3 - t2
    return h00*p0 + h10*m0 + h01*p1 + h11*m1


def cubic_spline_interpolation(
    waypoints: List[List[float]],
    num_points_per_segment: int = INTERPOLATION_POINTS,
) -> List[List[float]]:
    """웨이포인트 시퀀스를 3차 에르미트 스플라인으로 보간."""
    if len(waypoints) < 2:
        return list(waypoints)

    result = []
    n = len(waypoints)

    # 각 관절별 접선(tangent) 계산 (Catmull-Rom 방식)
    tangents = []
    for i in range(n):
        tang = [0.0] * NUM_JOINTS
        if i == 0:
            tang = [waypoints[1][j] - waypoints[0][j] for j in range(NUM_JOINTS)]
        elif i == n - 1:
            tang = [waypoints[-1][j] - waypoints[-2][j] for j in range(NUM_JOINTS)]
        else:
            tang = [
                0.5 * (waypoints[i+1][j] - waypoints[i-1][j])
                for j in range(NUM_JOINTS)
            ]
        tangents.append(tang)

    # 세그먼트별 보간
    for seg in range(n - 1):
        p0 = waypoints[seg]
        p1 = waypoints[seg + 1]
        m0 = tangents[seg]
        m1 = tangents[seg + 1]

        npts = num_points_per_segment if seg < n - 2 else num_points_per_segment
        for i in range(npts):
            t = i / max(npts - 1, 1)
            pt = [cubic_hermite(p0[j], p1[j], m0[j], m1[j], t) for j in range(NUM_JOINTS)]
            result.append(pt)

    # 마지막 점 보장
    if result[-1] != waypoints[-1]:
        result.append(list(waypoints[-1]))

    return result


def trapezoidal_velocity_profile(
    start: List[float],
    end: List[float],
    max_vel: float = MAX_JOINT_VELOCITY,
    max_acc: float = MAX_JOINT_ACCELERATION,
    dt: float = CONTROL_DT,
) -> List[List[float]]:
    """
    사다리꼴 속도 프로파일로 두 점 사이를 보간.
    가속 → 등속 → 감속 구간을 생성합니다.
    """
    # 가장 큰 관절 변위 기준으로 시간 계산
    diffs = [abs(e - s) for s, e in zip(start, end)]
    max_diff = max(diffs) if diffs else 0.0

    if max_diff < 1e-8:
        return [list(start)]

    # 사다리꼴 시간 계산
    t_acc = max_vel / max_acc
    d_acc = 0.5 * max_acc * t_acc * t_acc

    if 2 * d_acc >= max_diff:
        # 삼각형 프로파일 (등속 구간 없음)
        t_acc = math.sqrt(max_diff / max_acc)
        t_total = 2 * t_acc
        t_cruise = 0.0
    else:
        d_cruise = max_diff - 2 * d_acc
        t_cruise = d_cruise / max_vel
        t_total = 2 * t_acc + t_cruise

    points = []
    t = 0.0
    while t <= t_total + dt * 0.5:
        # 정규화 위치 s ∈ [0, 1]
        if t <= t_acc:
            s = 0.5 * max_acc * t * t / max_diff
        elif t <= t_acc + t_cruise:
            s = (d_acc + max_vel * (t - t_acc)) / max_diff
        else:
            t_dec = t - t_acc - t_cruise
            s = (max_diff - 0.5 * max_acc * (t_total - t)**2) / max_diff

        s = max(0.0, min(1.0, s))
        pt = [si + (ei - si) * s for si, ei in zip(start, end)]
        points.append(pt)
        t += dt

    # 마지막 점 보장
    points.append(list(end))
    return points


def interpolate_trajectory(
    waypoints: List[List[float]],
    method: str = 'linear',
    num_points: int = INTERPOLATION_POINTS,
) -> List[List[float]]:
    """
    웨이포인트 시퀀스 전체를 보간.

    Args:
        waypoints: 원본 웨이포인트 리스트
        method: 'linear', 'cubic', 'trapezoid'
        num_points: 세그먼트당 보간 점 수

    Returns:
        보간된 관절 각도 리스트
    """
    if len(waypoints) < 2:
        return list(waypoints)

    if method == 'cubic':
        return cubic_spline_interpolation(waypoints, num_points)

    result = []
    for i in range(len(waypoints) - 1):
        if method == 'trapezoid':
            seg = trapezoidal_velocity_profile(waypoints[i], waypoints[i + 1])
        else:  # linear
            seg = linear_interpolation(waypoints[i], waypoints[i + 1], num_points)

        # 첫 세그먼트가 아니면 시작점 중복 제거
        if i > 0 and seg:
            seg = seg[1:]
        result.extend(seg)

    return result
