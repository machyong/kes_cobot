#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CalibrationClick(Node):
    def __init__(self):
        super().__init__('calibration_click')
        self.bridge = CvBridge()
        
        # ROI 설정
        self.roi_x = 360
        self.roi_y = 60
        self.roi_w = 700
        self.roi_h = 470
        
        self.pixel_points = []
        self.latest_image = None
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        cv2.namedWindow('Calibration - Click 4 Points')
        cv2.setMouseCallback('Calibration - Click 4 Points', self.mouse_callback)
        
        print("=" * 60)
        print("캘리브레이션 도구")
        print("=" * 60)
        print("ROI 내에서 4개 지점을 클릭하세요 (순서대로):")
        print("  1) 좌상단")
        print("  2) 좌하단")
        print("  3) 우하단")
        print("  4) 우상단")
        print()
        print("4개 지점 선택 후:")
        print("  - 각 지점에 블럭을 놓으세요")
        print("  - mycobot을 각 위치로 이동시켜 좌표를 기록하세요")
        print("=" * 60)
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.pixel_points) < 4:
                self.pixel_points.append([float(x), float(y)])
                print(f"\n포인트 {len(self.pixel_points)}: ({x}, {y})")
                
                if len(self.pixel_points) == 4:
                    print("\n" + "=" * 60)
                    print("픽셀 좌표 완료! get_coord.py에 입력할 값:")
                    print("=" * 60)
                    print("pts_pixel = np.array([")
                    for i, pt in enumerate(self.pixel_points):
                        print(f"    [{pt[0]}, {pt[1]}],  # 포인트 {i+1}")
                    print("], dtype=np.float32)")
                    print("\n이제 각 지점에 블럭을 놓고 로봇 좌표를 기록하세요!")
                    print("로봇을 각 위치로 이동 → 좌표 확인 → 기록")
                    print("=" * 60)
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_image = cv_image.copy()
        
        # ROI 박스
        cv2.rectangle(cv_image, 
                     (self.roi_x, self.roi_y), 
                     (self.roi_x + self.roi_w, self.roi_y + self.roi_h), 
                     (0, 255, 0), 3)
        
        # 선택된 포인트 표시
        for i, pt in enumerate(self.pixel_points):
            cv2.circle(cv_image, (int(pt[0]), int(pt[1])), 8, (0, 0, 255), -1)
            cv2.putText(cv_image, str(i+1), 
                       (int(pt[0])+15, int(pt[1])+15),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
        # 선으로 연결 (4개 선택 완료 시)
        if len(self.pixel_points) == 4:
            pts = np.array(self.pixel_points, dtype=np.int32)
            cv2.polylines(cv_image, [pts], True, (255, 0, 0), 2)
        
        # 안내 텍스트
        text = f"Points: {len(self.pixel_points)}/4"
        cv2.putText(cv_image, text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
        
        cv2.imshow('Calibration - Click 4 Points', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationClick()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()