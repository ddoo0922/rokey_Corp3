import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # 또는 Isaac Sim ROS2 제어용 토픽 메시지
import time

class SimpleSandingPlanner(Node):
    def __init__(self):
        super().__init__('simple_sanding_planner')
        # Isaac Sim의 ROS2 Subscribe 노드가 듣고 있는 토픽 이름으로 설정
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        
        # 1. 샌딩하면서 이동할 로봇의 주요 관절 각도(또는 좌표) 목표점들 정의
        # 예: [기본 위치, 샌딩 시작점, 샌딩 끝점]
        self.waypoints = [
            [0.0, -1.0, 1.0, 0.0, 1.5, 0.0],  # Waypoint 1
            [0.2, -1.1, 1.1, 0.1, 1.5, 0.0],  # Waypoint 2 (조금 이동)
            [0.4, -1.2, 1.2, 0.2, 1.5, 0.0]   # Waypoint 3 (더 이동)
        ]
        self.current_waypoint_idx = 0
        
        # 2. 50Hz (0.02초) 마다 제어 명령을 보내는 타이머 생성 (주기가 일정해야 부드럽습니다)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('모든 샌딩 경로 이동 완료!')
            self.timer.cancel()
            return

        # 현재 이동해야 할 목표 각도
        target_joints = self.waypoints[self.current_waypoint_idx]
        
        # ROS 2 메시지 담기
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = target_joints
        
        # Isaac Sim으로 전송
        self.publisher_.publish(msg)
        
        # 가장 간단하게 구현하기 위해, 일정 시간(예: 가는데 2초) 지나면 다음 점으로 넘어가도록 설정
        # (실무에서는 로봇이 도착했는지 피드백을 받고 넘어가지만, 이게 가장 쉽습니다)
        time.sleep(0.01) 
        self.current_waypoint_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSandingPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
