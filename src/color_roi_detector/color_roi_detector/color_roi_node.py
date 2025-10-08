#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2, numpy as np, yaml, os
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class ColorRoiNode(Node):
    def __init__(self):
        super().__init__('color_roi_node')
        self.bridge = CvBridge()
        config_path = None
        try:
            share_dir = get_package_share_directory('color_roi_detector')
            cand = os.path.join(share_dir, 'config', 'default.yaml')
            if os.path.exists(cand):
                config_path = cand
        except Exception:
            pass
        if config_path is None:
            # 소스에서 바로 실행할 때 대비 (이 파일 기준 상대경로)
            here = os.path.dirname(os.path.abspath(__file__))
            cand = os.path.normpath(os.path.join(here, '..', 'config', 'default.yaml'))
            if os.path.exists(cand):
                config_path = cand
        if config_path is None:
            raise FileNotFoundError("default.yaml을 찾을 수 없습니다. config/default.yaml 경로를 확인하세요.")

        # === YAML 로드 ===
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        params = cfg['color_roi_node']['ros__parameters']

        # === YAML 값 적용 (★ get_parameter 사용 절대 금지) ===
        self.image_topic = str(params['image_topic'])
        self.roi_x  = int(params['roi_x'])
        self.roi_y  = int(params['roi_y'])
        self.roi_w  = int(params['roi_w'])
        self.roi_h  = int(params['roi_h'])
        self.min_ratio   = float(params['min_ratio'])
        self.blur_kernel = int(params['blur_kernel'])

        # colors 파싱 (lower/upper dict 또는 [[lo],[hi]] 모두 허용)
        color_dict = params['colors']
        self.color_ranges = {}
        for cname, ranges in color_dict.items():
            parsed = []
            for pair in ranges:
                if isinstance(pair, dict):
                    lower = np.array(pair['lower'], dtype=np.uint8)
                    upper = np.array(pair['upper'], dtype=np.uint8)
                else:
                    lower = np.array(pair[0], dtype=np.uint8)
                    upper = np.array(pair[1], dtype=np.uint8)
                parsed.append((lower, upper))
            self.color_ranges[cname] = parsed

        # Pub/Sub
        self.pub_color = self.create_publisher(String, '/color_roi/color', 10)
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(
            f"Loaded YAML from: {config_path}\n"
            f"Sub: {self.image_topic} | ROI=({self.roi_x},{self.roi_y},{self.roi_w},{self.roi_h}) | "
            f"colors={list(self.color_ranges.keys())}"
        )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        h, w = frame.shape[:2]
        x = max(0, min(self.roi_x, w-1))
        y = max(0, min(self.roi_y, h-1))
        rw = max(1, min(self.roi_w, w - x))
        rh = max(1, min(self.roi_h, h - y))

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
