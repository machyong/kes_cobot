#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class ColorRoiNode(Node):
    def __init__(self):
        super().__init__('color_roi_node')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('roi_x', 450)
        self.declare_parameter('roi_y', 150)
        self.declare_parameter('roi_w', 20)
        self.declare_parameter('roi_h', 20)
        self.declare_parameter('min_ratio', 0.02)      # 색 점유율 2% 미만이면 none
        self.declare_parameter('blur_kernel', 5)       # 홀수 권장(예: 5)
        self.declare_parameter('colors', yaml.safe_dump({
            'red':    [[[170,100,80],[180,255,255]]],
            'green':  [[[35,60,60],[85,255,255]]],
            'blue':   [[[95,60,60],[135,255,255]]],
            'yellow': [[[20,120,80],[32,255,255]]],
            'orange': [[[10,120,80],[20,255,255]]],
        }))

        # Read params
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.roi_x = int(self.get_parameter('roi_x').value)
        self.roi_y = int(self.get_parameter('roi_y').value)
        self.roi_w = int(self.get_parameter('roi_w').value)
        self.roi_h = int(self.get_parameter('roi_h').value)
        self.min_ratio = float(self.get_parameter('min_ratio').value)
        self.blur_kernel = int(self.get_parameter('blur_kernel').value) or 5

        colors_yaml = self.get_parameter('colors').get_parameter_value().string_value
        try:
            color_dict = yaml.safe_load(colors_yaml)
        except Exception:
            color_dict = {}
        self.color_ranges = {}
        for cname, ranges in color_dict.items():
            self.color_ranges[cname] = [
                (np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in ranges
            ]

        # Publisher (문자열 색상)
        self.pub_color = self.create_publisher(String, '/color_roi/color', 10)

        # Subscriber
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(
            f"Sub: {self.image_topic} | ROI=({self.roi_x},{self.roi_y},{self.roi_w},{self.roi_h}) "
            f"| colors={list(self.color_ranges.keys())}"
        )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        h, w = frame.shape[:2]
        x = max(0, min(450, w-1))
        y = max(0, min(150, h-1))
        rw = max(1, min(20, w - x))
        rh = max(1, min(20, h - y))

        # ROI 영역 표시
        cv2.rectangle(frame, (x, y), (x + rw, y + rh), (0, 255, 255), 2)

        roi = frame[y:y+rh, x:x+rw].copy()
        if self.blur_kernel >= 3 and self.blur_kernel % 2 == 1:
            roi = cv2.GaussianBlur(roi, (self.blur_kernel, self.blur_kernel), 0)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_area = roi.shape[0] * roi.shape[1] if roi.size > 0 else 0

        best_color, best_ratio = 'none', 0.0
        COLOR_TO_BGR = {
            'red':    (0,   0, 255),
            'green':  (0, 255,   0),
            'blue':   (255, 0,   0),
            'yellow': (0, 255, 255),
            'orange': (0, 165, 255),
            'none':   (255, 255, 255)  # 검출 실패시 노란색 유지(원하면 (200,200,200) 같은 회색으로)
        }   
        if roi_area > 0:
            for cname, ranges in self.color_ranges.items():
                mask_total = np.zeros(roi.shape[:2], dtype=np.uint8)
                for lower, upper in ranges:
                    mask_total = cv2.bitwise_or(mask_total, cv2.inRange(hsv, lower, upper))
                # morphology
                k = np.ones((3,3), np.uint8)
                mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, k)
                mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, k)

                count = int(cv2.countNonZero(mask_total))
                ratio = count / float(roi_area)
                if ratio > best_ratio:
                    best_ratio, best_color = ratio, cname

            if best_ratio < self.min_ratio:
                best_color = 'none'

        # 토픽 퍼블리시
        self.pub_color.publish(String(data=best_color))

        # === ROI 박스 색상 선택 ===
        rect_color = COLOR_TO_BGR.get(best_color, (0, 255, 255))  # fallback: yellow

        # ROI 사각형(외곽선) 색상 적용
        cv2.rectangle(frame, (x, y), (x + rw, y + rh), rect_color, 2)

        # (선택) 반투명 채우기까지 원하면 아래 4줄 추가
        # overlay = frame.copy()
        # cv2.rectangle(overlay, (x, y), (x + rw, y + rh), rect_color, -1)
        # alpha = 0.2
        # frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

        # 화면에 ROI + 결과 표시 (텍스트 색도 동일하게)
        cv2.putText(frame, f"Color: {best_color} ({best_ratio:.3f})", (x, max(0, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, rect_color, 2)
        cv2.imshow("Color ROI Detector", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ColorRoiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
