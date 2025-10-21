#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.bridge = CvBridge()
        
        # ROI 설정
        self.roi_x = 440
        self.roi_y = 50
        self.roi_w = 500
        self.roi_h = 250
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 트랙바 윈도우 생성
        cv2.namedWindow('HSV Tuner')
        cv2.namedWindow('Original + ROI')
        cv2.namedWindow('Mask Result')
        
        # HSV 트랙바
        cv2.createTrackbar('H Low', 'HSV Tuner', 0, 180, lambda x: None)
        cv2.createTrackbar('H High', 'HSV Tuner', 180, 180, lambda x: None)
        cv2.createTrackbar('S Low', 'HSV Tuner', 0, 255, lambda x: None)
        cv2.createTrackbar('S High', 'HSV Tuner', 255, 255, lambda x: None)
        cv2.createTrackbar('V Low', 'HSV Tuner', 0, 255, lambda x: None)
        cv2.createTrackbar('V High', 'HSV Tuner', 255, 255, lambda x: None)
        
        print("=" * 60)
        print("HSV 색상 범위 조정 도구")
        print("=" * 60)
        print("트랙바를 조절하여 원하는 색상만 흰색으로 나오도록 조정하세요")
        print("조정 완료 후 콘솔에서 HSV 값을 복사하세요")
        print("SPACE: 현재 HSV 값 출력")
        print("ESC: 종료")
        print("=" * 60)
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # ROI 추출
        roi = cv_image[
            self.roi_y:self.roi_y+self.roi_h,
            self.roi_x:self.roi_x+self.roi_w
        ]
        
        # BGR → HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 트랙바 값 가져오기
        h_low = cv2.getTrackbarPos('H Low', 'HSV Tuner')
        h_high = cv2.getTrackbarPos('H High', 'HSV Tuner')
        s_low = cv2.getTrackbarPos('S Low', 'HSV Tuner')
        s_high = cv2.getTrackbarPos('S High', 'HSV Tuner')
        v_low = cv2.getTrackbarPos('V Low', 'HSV Tuner')
        v_high = cv2.getTrackbarPos('V High', 'HSV Tuner')
        
        # 마스크 생성
        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])
        mask = cv2.inRange(hsv, lower, upper)
        
        # 결과 표시
        result = cv2.bitwise_and(roi, roi, mask=mask)
        
        # ROI 박스 그리기
        cv_image_copy = cv_image.copy()
        cv2.rectangle(cv_image_copy, 
                     (self.roi_x, self.roi_y), 
                     (self.roi_x + self.roi_w, self.roi_y + self.roi_h), 
                     (0, 255, 0), 3)
        
        # 윤곽선 찾기 및 표시
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # 가장 큰 윤곽선
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > 100:
                # 중심점
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # ROI 내 좌표 → 전체 이미지 좌표
                    abs_x = self.roi_x + cx
                    abs_y = self.roi_y + cy
                    
                    # 마커 그리기
                    cv2.drawMarker(cv_image_copy, (abs_x, abs_y), (0, 0, 255),
                                 cv2.MARKER_CROSS, 30, 3)
                    cv2.circle(cv_image_copy, (abs_x, abs_y), 40, (0, 255, 255), 2)
                    
                    # 정보 표시
                    info = f"({abs_x},{abs_y}) Area:{area:.0f}"
                    cv2.putText(cv_image_copy, info, (abs_x - 60, abs_y - 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.imshow('Original + ROI', cv_image_copy)
        cv2.imshow('Mask Result', mask)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' '):
            print("\n" + "=" * 60)
            print("현재 HSV 값:")
            print("=" * 60)
            print(f"H: {h_low} ~ {h_high}")
            print(f"S: {s_low} ~ {s_high}")
            print(f"V: {v_low} ~ {v_high}")
            print("=" * 60)
            print("\ndefault.yaml에 추가할 형식:")
            print(f"- lower: [{h_low}, {s_low}, {v_low}]")
            print(f"  upper: [{h_high}, {s_high}, {v_high}]")
            print("=" * 60)
            print("\nfind_angle.py에 추가할 형식:")
            print(f"mask = cv2.inRange(hsv, np.array([{h_low}, {s_low}, {v_low}]), np.array([{h_high}, {s_high}, {v_high}]))")
            print("=" * 60 + "\n")
        
        elif key == 27:  # ESC
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    
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