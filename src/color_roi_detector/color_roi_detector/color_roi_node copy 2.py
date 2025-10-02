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

        # 저장할 디렉토리 생성
        self.save_dir = "captured_images"
        os.makedirs(self.save_dir, exist_ok=True)

    def listener_callback(self, msg):
        # ROS Image → OpenCV 변환
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        x, y, w, h = 411, 2, 560, 315
        roi =  self.frame[y:y+h, x:x+w]

        # 3. 원본 이미지와 ROI 출력
        cv2.imshow("ROI", roi)
        # cv2.imshow("Camera Feed", self.frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and self.frame is not None:
            filename = os.path.join(
                self.save_dir, f"capture_{int(time.time())}.png"
            )
            cv2.imwrite(filename, self.frame)
            self.get_logger().info(f"이미지 저장됨: {filename}")

        elif key == ord('q'):  # q 누르면 종료
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
