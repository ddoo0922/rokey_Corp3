import cv2
import numpy as np
import rclpy
import time

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class ColorDetector(Node):

    def __init__(self):
        super().__init__('m0609_color_detector')

        self.bridge = CvBridge()
        self.last_log_time = 0.0

        self.image_subscriber = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.color_publisher = self.create_publisher(
            Int32,
            '/color_id',
            10
        )

        self.get_logger().info('색상 감지 노드 실행')
        self.get_logger().info('파랑: 1, 초록: 2, 감지 없음: 0')
        self.get_logger().info('화면 전체가 아니라 Pick 영역 ROI만 색상 감지함')

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(f'이미지 변환 실패: {error}')
            return

        height, width, _ = image.shape

        # ==================================================
        # Pick 영역만 잘라서 색상 판단
        # 화면 전체를 보면 초록 place 마커, 좌표축, 바닥 마커까지 잡힘
        # 사진 기준으로 큐브가 있는 위쪽/오른쪽 영역만 사용
        # ==================================================
        x1 = int(width * 0.40)
        x2 = int(width * 0.90)
        y1 = int(height * 0.10)
        y2 = int(height * 0.55)

        roi = image[y1:y2, x1:x2]

        hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 파란색 HSV 범위
        blue_lower = np.array([85, 40, 30])
        blue_upper = np.array([140, 255, 255])

        # 초록색 HSV 범위
        green_lower = np.array([30, 40, 30])
        green_upper = np.array([95, 255, 255])

        blue_mask = cv2.inRange(
            hsv_image,
            blue_lower,
            blue_upper
        )

        green_mask = cv2.inRange(
            hsv_image,
            green_lower,
            green_upper
        )

        # 작은 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)

        blue_mask = cv2.morphologyEx(
            blue_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        green_mask = cv2.morphologyEx(
            green_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        blue_area = cv2.countNonZero(blue_mask)
        green_area = cv2.countNonZero(green_mask)

        minimum_area = 100

        result = Int32()

        if blue_area > green_area and blue_area > minimum_area:
            result.data = 1
            detected_color = '파랑'

        elif green_area > blue_area and green_area > minimum_area:
            result.data = 2
            detected_color = '초록'

        else:
            result.data = 0
            detected_color = '감지 없음'

        self.color_publisher.publish(result)

        current_time = time.time()

        if current_time - self.last_log_time >= 1.0:
            self.get_logger().info(
                f'{detected_color} | '
                f'ROI=({x1},{y1})~({x2},{y2}) | '
                f'blue_area={blue_area}, '
                f'green_area={green_area}, '
                f'color_id={result.data}'
            )
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)

    node = ColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()