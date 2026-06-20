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

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(f'이미지 변환 실패: {error}')
            return

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        blue_lower = np.array([90, 80, 50])
        blue_upper = np.array([130, 255, 255])

        green_lower = np.array([35, 80, 50])
        green_upper = np.array([85, 255, 255])

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

        blue_area = cv2.countNonZero(blue_mask)
        green_area = cv2.countNonZero(green_mask)

        minimum_area = 500

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
