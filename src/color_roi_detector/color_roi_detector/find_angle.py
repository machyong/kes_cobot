import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorBlockPublisher(Node):
    def __init__(self):
        super().__init__('color_block_publisher')

        # 색상별 퍼블리셔
        self.color_publishers = {
            "red": self.create_publisher(String, 'red_block', 10),
            "blue": self.create_publisher(String, 'blue_block', 10),
            "green": self.create_publisher(String, 'green_block', 10),
            "yellow": self.create_publisher(String, 'yellow_block', 10),
        }

        # 카메라 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("ColorBlockPublisher node started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ROI
        x, y, w, h = 531, 50, 435, 330
        roi = frame[y:y+h, x:x+w]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 색상 범위 정의 (blue는 러프하게)
        color_ranges = {
            "red1": ((0, 100, 100), (10, 255, 255)),
            "red2": ((160, 100, 100), (179, 255, 255)),
            "blue": ((90, 80, 50), (130, 255, 255)),   # 더 러프
            "yellow": ((20, 100, 100), (30, 255, 255)),
            "green": ((35, 50, 50), (85, 255, 255)),
        }

        detected_results = {"red": None, "blue": None, "green": None, "yellow": None}
        centers = {}  # 블럭별 중심 좌표 저장

        for color, (lower, upper) in color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)

            if color == "red1":
                mask1 = mask
                continue
            elif color == "red2":
                mask = cv2.add(mask1, mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 300:
                    rect = cv2.minAreaRect(cnt)
                    cx, cy = int(rect[0][0]), int(rect[0][1])
                    angle = rect[2]
                    if rect[1][0] < rect[1][1]:
                        angle = angle + 90

                    main_color = color.replace('1', '').replace('2', '')
                    # 일단 picking 상태는 나중에 결정
                    result_str = f"Center: ({cx}, {cy})/ Angle: {angle:.1f}"
                    detected_results[main_color] = result_str
                    centers[main_color] = (cx, cy)

        # ============ 픽업 가능 여부 검사 ============ #
        MIN_DIST = 50  # 픽셀 단위 예시 (depth 기반으로 교체 가능)
        final_results = {}

        for c1, center1 in centers.items():
            picking = "o"
            for c2, center2 in centers.items():
                if c1 == c2:
                    continue
                dist = np.linalg.norm(np.array(center1) - np.array(center2))
                if dist < MIN_DIST:
                    picking = "x"
                    self.get_logger().warn(
                        f"{c1} block too close to {c2} (dist={dist:.1f}) → picking:x"
                    )
                    break
            
            # 결과 문자열에 picking 여부 추가
            if detected_results[c1]:
                final_results[c1] = detected_results[c1] + f" / picking: {picking}"

        # 퍼블리시
        for c, pub in self.color_publishers.items():
            if c in final_results:
                msg_out = String()
                msg_out.data = final_results[c]
                pub.publish(msg_out)
                self.get_logger().info(f"Published on /{c}_block: {msg_out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorBlockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
