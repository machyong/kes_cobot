#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ROIAdjuster(Node):
    def __init__(self):
        super().__init__('roi_adjuster')
        self.bridge = CvBridge()
        
        # ROI 초기값
        self.roi_x = 440
        self.roi_y = 50
        self.roi_w = 500
        self.roi_h = 250
        
        self.image_count = 0
        
        # ✅ QoS를 아예 지정하지 않고 자동 매칭 시도
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # ✅ 타이머 추가: 주기적으로 spin 확인
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer_count = 0
        
        print("=" * 50)
        print("ROI 조정 도구")
        print("=" * 50)
        print("키보드 조작:")
        print("  W/S: Y 위치 조정 (위/아래)")
        print("  A/D: X 위치 조정 (좌/우)")
        print("  ↑/↓: 세로 크기 조정")
        print("  ←/→: 가로 크기 조정")
        print("  SPACE: 현재 설정 출력")
        print("  ESC: 종료")
        print("=" * 50)
        
        self.get_logger().info("이미지 토픽 구독 시작: /camera/camera/color/image_raw")
        self.get_logger().info("이미지 수신 대기 중...")
        
    def timer_callback(self):
        """1초마다 상태 확인"""
        self.timer_count += 1
        if self.image_count == 0 and self.timer_count % 5 == 0:
            self.get_logger().warn(f"⚠️ {self.timer_count}초 경과, 아직 이미지 미수신")
        
    def image_callback(self, msg):
        self.image_count += 1
        
        if self.image_count == 1:
            self.get_logger().info("✅ 첫 이미지 수신 성공!")
            self.get_logger().info(f"이미지 크기: {msg.width}x{msg.height}")
            self.get_logger().info(f"Encoding: {msg.encoding}")
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            if self.image_count == 1:
                self.get_logger().info("✅ OpenCV 변환 성공!")
            
            # ROI 박스 그리기
            cv2.rectangle(cv_image, 
                         (self.roi_x, self.roi_y), 
                         (self.roi_x + self.roi_w, self.roi_y + self.roi_h), 
                         (0, 255, 0), 3)
            
            # 정보 표시
            info_text = [
                f"X: {self.roi_x}, Y: {self.roi_y}",
                f"W: {self.roi_w}, H: {self.roi_h}",
                f"Frame: {self.image_count}"
            ]
            for i, text in enumerate(info_text):
                cv2.putText(cv_image, text, (10, 30 + i*30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('ROI Adjuster', cv_image)
            
            if self.image_count == 1:
                self.get_logger().info("✅ OpenCV 창 표시 완료!")
            
            key = cv2.waitKey(1) & 0xFF
            
            step = 10
            
            if key == ord('w'):
                self.roi_y = max(0, self.roi_y - step)
            elif key == ord('s'):
                self.roi_y += step
            elif key == ord('a'):
                self.roi_x = max(0, self.roi_x - step)
            elif key == ord('d'):
                self.roi_x += step
            elif key == 82:  # Up arrow
                self.roi_h += step
            elif key == 84:  # Down arrow
                self.roi_h = max(50, self.roi_h - step)
            elif key == 81:  # Left arrow
                self.roi_w = max(50, self.roi_w - step)
            elif key == 83:  # Right arrow
                self.roi_w += step
            elif key == ord(' '):
                print("\n현재 ROI 설정:")
                print(f"    roi_x: {self.roi_x}")
                print(f"    roi_y: {self.roi_y}")
                print(f"    roi_w: {self.roi_w}")
                print(f"    roi_h: {self.roi_h}")
                print()
            elif key == 27:  # ESC
                print("\n최종 ROI 설정:")
                print(f"    roi_x: {self.roi_x}")
                print(f"    roi_y: {self.roi_y}")
                print(f"    roi_w: {self.roi_w}")
                print(f"    roi_h: {self.roi_h}")
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"❌ 에러 발생: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = ROIAdjuster()
    
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