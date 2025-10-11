#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.frame = None

        # 저장 디렉토리
        self.save_dir = "captured_images"
        os.makedirs(self.save_dir, exist_ok=True)

        # 100ms마다 화면 업데이트
        self.timer = self.create_timer(0.1, self.timer_callback)

    def listener_callback(self, msg):
        # ROS Image → OpenCV 변환
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def timer_callback(self):
        if self.frame is None:
            return

        # ROI 영역 자르기
        # x, y, w, h = 555, 147, 311, 222
        # roi = self.frame[y:y+h, x:x+w]

        # 영상 출력
        # cv2.imshow("ROI", roi)
        cv2.imshow("Camera Feed", self.frame)

        # 키 입력 처리
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):  # ROI 저장
            filename = os.path.join(
                self.save_dir, f"roi_{int(time.time())}.png"
            )
            cv2.imwrite(filename, self.frame)
            self.get_logger().info(f"ROI 저장됨: {filename}")

        elif key == ord('S'):  # 전체 프레임 저장
            filename = os.path.join(
                self.save_dir, f"capture.png"
            )
            cv2.imwrite(filename, self.frame)
            self.get_logger().info(f"전체 프레임 저장됨: {filename}")

        elif key == ord('q'):  # 종료
            self.get_logger().info("사용자 종료 요청")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
