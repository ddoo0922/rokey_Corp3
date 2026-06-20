"""
sanding_node.py — 2D 판 샌딩 경로 실행 ROS 2 노드

기능:
  - path_generator로 경로 자동 생성
  - trajectory_interpolator로 부드러운 보간
  - 타이머 기반 비동기 제어 (time.sleep 없음)
  - 경로 패턴/파라미터를 ROS 파라미터로 실시간 설정
  - 진행률 퍼블리시
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from simple_sanding.sanding_config import (
    CONTROL_DT,
    JOINT_NAMES,
    PANEL_HEIGHT,
    PANEL_WIDTH,
    SANDING_STEP,
    SANDING_RESOLUTION,
    DEFAULT_PATTERN,
    INTERPOLATION_POINTS,
    TOPIC_JOINT_COMMAND,
    TOPIC_JOINT_STATES,
    TOPIC_SANDING_PROGRESS,
)
from simple_sanding.path_generator import generate_path
from simple_sanding.trajectory_interpolator import interpolate_trajectory


class SandingNode(Node):
    """2D 판 샌딩 경로 추적 노드."""

    def __init__(self):
        super().__init__('sanding_node')

        # ── ROS 파라미터 선언 ──
        self.declare_parameter('pattern', DEFAULT_PATTERN)
        self.declare_parameter('panel_width', PANEL_WIDTH)
        self.declare_parameter('panel_height', PANEL_HEIGHT)
        self.declare_parameter('step_size', SANDING_STEP)
        self.declare_parameter('resolution', SANDING_RESOLUTION)
        self.declare_parameter('interpolation', 'linear')
        self.declare_parameter('interp_points', INTERPOLATION_POINTS)

        # ── 퍼블리셔 / 구독자 ──
        self.joint_pub = self.create_publisher(
            JointState, TOPIC_JOINT_COMMAND, 10)
        self.progress_pub = self.create_publisher(
            Float32, TOPIC_SANDING_PROGRESS, 10)

        # (선택) 관절 피드백 구독
        self.current_joints = None
        self.joint_sub = self.create_subscription(
            JointState, TOPIC_JOINT_STATES,
            self._joint_state_cb, 10)

        # ── 경로 생성 ──
        self._build_trajectory()

        # ── 제어 타이머 ──
        self.idx = 0
        self.timer = self.create_timer(CONTROL_DT, self._control_loop)

        self.get_logger().info('='*50)
        self.get_logger().info(' 샌딩 노드 시작')
        self.get_logger().info(f'  패턴: {self._pattern}')
        self.get_logger().info(f'  웨이포인트: {self._num_raw} → 보간 후: {len(self.trajectory)}')
        self.get_logger().info(f'  제어 주기: {CONTROL_DT*1000:.0f} ms')
        self.get_logger().info('='*50)

    # ── 경로 생성 ──────────────────────────────────────
    def _build_trajectory(self):
        self._pattern = self.get_parameter('pattern').value
        pw = self.get_parameter('panel_width').value
        ph = self.get_parameter('panel_height').value
        ss = self.get_parameter('step_size').value
        res = self.get_parameter('resolution').value
        interp = self.get_parameter('interpolation').value
        npts = self.get_parameter('interp_points').value

        # 원본 웨이포인트 생성
        raw_path = generate_path(
            pattern=self._pattern,
            width=pw, height=ph,
            step=ss, resolution=res,
        )
        self._num_raw = len(raw_path)

        # 보간 (웨이포인트가 너무 많으면 건너뛰기)
        if len(raw_path) > 500:
            # 이미 촘촘한 경로 — 보간 생략 또는 간단한 선형만
            self.trajectory = raw_path
        else:
            self.trajectory = interpolate_trajectory(
                raw_path, method=interp, num_points=npts)

    # ── 관절 피드백 ────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        if msg.position:
            self.current_joints = list(msg.position)

    # ── 메인 제어 루프 ─────────────────────────────────
    def _control_loop(self):
        if self.idx >= len(self.trajectory):
            self.get_logger().info('✅ 샌딩 경로 완료!')
            self.timer.cancel()

            # 완료 진행률
            prog = Float32()
            prog.data = 1.0
            self.progress_pub.publish(prog)
            return

        target = self.trajectory[self.idx]

        # JointState 메시지 생성
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(j) for j in target]

        self.joint_pub.publish(msg)

        # 진행률 퍼블리시
        progress = self.idx / max(len(self.trajectory) - 1, 1)
        prog_msg = Float32()
        prog_msg.data = float(progress)
        self.progress_pub.publish(prog_msg)

        # 로그 (5% 단위)
        pct = int(progress * 100)
        if self.idx == 0 or pct % 5 == 0:
            prev_pct = int((self.idx - 1) / max(len(self.trajectory) - 1, 1) * 100)
            if self.idx == 0 or pct != prev_pct:
                self.get_logger().info(
                    f'[{pct:3d}%] idx={self.idx}/{len(self.trajectory)} '
                    f'J1={target[0]:.3f} J2={target[1]:.3f} J3={target[2]:.3f}'
                )

        self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = SandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
