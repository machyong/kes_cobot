#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorBlockServer(Node):
    def __init__(self):
        super().__init__('color_block_server')

        # ===== 서비스 생성 =====
        self.srv_red = self.create_service(Trigger, '/red_block', self.handle_red)
        self.srv_yellow = self.create_service(Trigger, '/yellow_block', self.handle_yellow)
        self.srv_green = self.create_service(Trigger, '/green_block', self.handle_green)
        self.get_logger().info("🎨 color_block_server 준비 완료 (red/yellow/green)")

        # ===== 카메라 구독 =====
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # 최신 분석 결과 저장용
        self.detected_results = {"red": None, "blue": None, "yellow": None, "green": None}

    # ===== 이미지 콜백: 영상 분석 로직 =====
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CVBridge 변환 오류: {e}")
            return

        # ROI 설정 (원본 코드와 동일)
        x, y, w, h = 525, 270, 210, 130
        roi = frame[y:y+h, x:x+w]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        color_ranges = {
            "red1": ((0, 100, 100), (10, 255, 255)),
            "red2": ((160, 100, 100), (179, 255, 255)),
            "blue": ((90, 80, 50), (130, 255, 255)),
            "yellow": ((20, 100, 100), (30, 255, 255)),
            "green": ((35, 50, 50), (85, 255, 255)),
        }

        detected_results = {"red": None, "blue": None, "green": None, "yellow": None}
        centers = {}

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
                    result_str = f"Center:({cx},{cy})/Angle:{angle:.1f}"
                    detected_results[main_color] = result_str
                    centers[main_color] = (cx, cy)

        # ===== 픽업 가능 여부 계산 =====
        MIN_DIST = 50
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

            if detected_results[c1]:
                final_results[c1] = detected_results[c1] + f"/picking:{picking}"

        # 최신 결과 갱신
        self.detected_results.update(final_results)

        # 시각화
        cv2.imshow("ROI", roi)
        cv2.waitKey(1)

    # ===== 서비스 콜백 함수들 =====
    def handle_red(self, request, response):
        msg = self.detected_results.get("red")
        if msg:
            response.success = True
            response.message = msg
        else:
            response.success = False
            response.message = "빨간 블록이 감지되지 않았습니다."
        self.get_logger().info(f"🔴 red_block 요청 처리: {response.message}")
        return response

    def handle_yellow(self, request, response):
        msg = self.detected_results.get("yellow")
        if msg:
            response.success = True
            response.message = msg
        else:
            response.success = False
            response.message = "노란 블록이 감지되지 않았습니다."
        self.get_logger().info(f"🟡 yellow_block 요청 처리: {response.message}")
        return response

    def handle_green(self, request, response):
        msg = self.detected_results.get("green")
        if msg:
            response.success = True
            response.message = msg
        else:
            response.success = False
            response.message = "초록 블록이 감지되지 않았습니다."
        self.get_logger().info(f"🟢 green_block 요청 처리: {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ColorBlockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
