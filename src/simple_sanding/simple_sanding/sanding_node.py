import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleSandingPlanner(Node):
    def __init__(self):
        super().__init__('simple_sanding_planner')
        
        # [체크] Isaac Sim Action Graph의 topicName 설정과 완벽히 일치해야 합니다.
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_commands', 10)
        
        # 1. 샌딩하면서 이동할 로봇의 주요 관절 각도 목표점들 (라디안 단위)
        self.waypoints = [
            [0.0, -1.0, 1.0, 0.0, 1.5, 0.0],  # Waypoint 1 (시작점)
            [0.2, -1.1, 1.1, 0.1, 1.5, 0.0],  # Waypoint 2 (샌딩 이동 중)
            [0.4, -1.2, 1.2, 0.2, 1.5, 0.0]   # Waypoint 3 (끝점)
        ]
        self.current_waypoint_idx = 0
        
        # 2. 시간 제어 변수 (각 목표점마다 로봇이 이동할 시간을 줍니다)
        self.time_spent_on_waypoint = 0.0
        self.hold_time_per_waypoint = 3.0  # 한 waypoint당 3.0초 동안 명령 유지
        
        # 3. 50Hz (0.02초) 마다 제어 명령을 보내는 타이머 생성
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('🤖 샌딩 플래너 노드가 시작되었습니다. 명령 전송 중...')

    def timer_callback(self):
        # 모든 경로점(Waypoint)을 다 돌았다면 타이머 종료
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('🎉 모든 샌딩 경로 이동 완료! 노드를 안전하게 종료합니다.')
            self.timer.cancel()
            return

        # 현재 이동해야 할 목표 각도 데이터 가져오기
        target_joints = self.waypoints[self.current_waypoint_idx]
        
        # ROS 2 메시지 객체 생성
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # [확인 완료] Isaac Sim 'joints' 폴더 내부의 실제 이름 및 순서 반영
        msg.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        msg.position = target_joints
        
        # Isaac Sim으로 토픽 발행(전송)
        self.publisher_.publish(msg)
        
        # 0.02초 누적 (time.sleep 대신 이 방식을 써야 ROS2 시스템이 멈추지 않습니다)
        self.time_spent_on_waypoint += 0.02
        
        # 지정한 시간(3초) 동안 신호를 보냈다면 다음 목표점으로 인덱스 전환
        if self.time_spent_on_waypoint >= self.hold_time_per_waypoint:
            self.get_logger().info(f'📍 Waypoint {self.current_waypoint_idx + 1} 이동 시간 종료, 다음으로 진행!')
            self.current_waypoint_idx += 1
            self.time_spent_on_waypoint = 0.0  # 누적 시간 초기화

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSandingPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
