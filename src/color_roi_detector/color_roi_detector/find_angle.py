#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorBlockServer(Node):
    def __init__(self):
        super().__init__('color_block_server')
        
        self.bridge = CvBridge()
        self.latest_color_image = None
        
        # ROI 설정
        self.roi_x = 440
        self.roi_y = 280
        self.roi_w = 530
        self.roi_h = 220
        
        # 고정 Z값
        self.tcp_height = 165.0
        
        # 컬러 이미지 구독
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10
        )
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Color block detection server ready")
        self.get_logger().info(f"ROI: ({self.roi_x}, {self.roi_y}, {self.roi_w}, {self.roi_h})")
        self.get_logger().info(f"고정 Z: {self.tcp_height:.1f} mm")
        self.get_logger().info(f"✅ 각도 범위: 0~90도")
        self.get_logger().info("=" * 60)
        
        # 서비스 생성
        self.red_service = self.create_service(Trigger, 'red_service', self.red_callback)
        self.yellow_service = self.create_service(Trigger, 'yellow_service', self.yellow_callback)
        self.green_service = self.create_service(Trigger, 'green_service', self.green_callback)
        self.blue_service = self.create_service(Trigger, 'blue_service', self.blue_callback)
        self.orange_service = self.create_service(Trigger, 'orange_service', self.orange_callback)
        
        self.get_logger().info("✅ 모든 서비스 준비 완료!")
        
        # 이미지 상태 체크
        self.create_timer(5.0, self.check_image_status)
    
    def check_image_status(self):
        if self.latest_color_image is None:
            self.get_logger().warn("⚠️ 컬러 이미지 수신 안됨!")
        else:
            self.get_logger().info(f"✅ 컬러 이미지 OK: {self.latest_color_image.shape}")
    
    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def red_callback(self, request, response):
        self.get_logger().info("🔴 red_service 호출됨!")
        return self.find_block_service(request, response, "빨간색")
    
    def yellow_callback(self, request, response):
        self.get_logger().info("🟡 yellow_service 호출됨!")
        return self.find_block_service(request, response, "노란색")
    
    def green_callback(self, request, response):
        self.get_logger().info("🟢 green_service 호출됨!")
        return self.find_block_service(request, response, "초록색")
    
    def blue_callback(self, request, response):
        self.get_logger().info("🔵 blue_service 호출됨!")
        return self.find_block_service(request, response, "파란색")
    
    def orange_callback(self, request, response):
        self.get_logger().info("🟠 orange_service 호출됨!")
        return self.find_block_service(request, response, "주황색")
    
    def find_block_service(self, request, response, color_name):
        """블록 검출 메인 로직"""
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🔍 {color_name} 블록 검출 시작")
        
        try:
            if self.latest_color_image is None:
                self.get_logger().error("❌ 컬러 이미지 없음!")
                response.success = False
                response.message = "컬러 이미지 없음"
                return response
            
            color_image = self.latest_color_image.copy()
            
            # ROI 추출
            roi_color = color_image[self.roi_y:self.roi_y+self.roi_h, 
                                   self.roi_x:self.roi_x+self.roi_w]
            
            # HSV 변환
            hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)
            
            # 색상별 HSV 범위
            color_ranges = {
                "빨간색": [
                    (np.array([0, 120, 70]), np.array([10, 255, 255])),
                    (np.array([170, 120, 70]), np.array([180, 255, 255]))
                ],
                "노란색": [(np.array([20, 100, 100]), np.array([30, 255, 255]))],
                "초록색": [(np.array([35, 100, 100]), np.array([85, 255, 255]))],
                "파란색": [(np.array([100, 150, 100]), np.array([130, 255, 255]))],
                "주황색": [(np.array([10, 100, 100]), np.array([20, 255, 255]))]
            }
            
            if color_name not in color_ranges:
                response.success = False
                response.message = f"지원하지 않는 색상: {color_name}"
                return response
            
            # 마스크 생성
            mask = None
            for lower, upper in color_ranges[color_name]:
                if mask is None:
                    mask = cv2.inRange(hsv, lower, upper)
                else:
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            
            # 노이즈 제거
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # 윤곽선 검출
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
                response.success = False
                response.message = f"{color_name} 블록을 찾을 수 없습니다"
                return response
            
            # 가장 큰 윤곽선
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area < 500:
                response.success = False
                response.message = f"블록이 너무 작습니다 (면적: {area:.1f})"
                return response
            
            # 중심점 계산
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                response.success = False
                response.message = "중심점 계산 실패"
                return response
            
            cx_roi = int(M["m10"] / M["m00"])
            cy_roi = int(M["m01"] / M["m00"])
            
            # 전체 이미지 좌표로 변환
            cx = cx_roi + self.roi_x
            cy = cy_roi + self.roi_y
            
            # ===== ✅ 3) 각도 계산 (0~90도 범위) =====
            rect = cv2.minAreaRect(largest_contour)
            angle = rect[2]
            width, height = rect[1]
            
            self.get_logger().info(f"minAreaRect 원본: angle={angle:.1f}°, size=({width:.1f}, {height:.1f})")
            
            if width < height:
                angle = angle + 90
            
            # 0~90도 범위로 정규화
            while angle < 0:
                angle += 90
            while angle >= 90:
                angle -= 90
            
            self.get_logger().info(f"✅ 최종 각도(0~90°): {angle:.1f}°")
            
            # Z값 고정
            tcp_height = self.tcp_height
            
            # 결과 문자열 생성
            #result_str = f"Center:({cx},{cy})/Angle:{angle:.1f}/Depth:{tcp_height:.2f}/picking:{color_name}"
            result_json = {
                "center": [int(cx), int(cy)],
                "angle": float(f"{angle:.1f}"),
                "depth": float(f"{tcp_height:.2f}"),
                "picking": color_name
            }
            result_str = (
                f"Center:({cx},{cy})/Angle:{angle:.1f}/Depth:{tcp_height:.2f}/picking:{color_name}"
                # 만약 여러 줄 허용 안되면 아래는 주석 처리
                # + "\n" + json.dumps(result_json, ensure_ascii=False)
            )
            
            self.get_logger().info("✅ 검출 성공!")
            self.get_logger().info(f"   중심: ({cx}, {cy})")
            self.get_logger().info(f"   각도: {angle:.1f}° (0~90° 범위)")
            self.get_logger().info(f"   Z(고정): {tcp_height:.2f}mm")
            self.get_logger().info(f"📤 응답: {result_str}")
            self.get_logger().info("=" * 60)
            
            response.success = True
            response.message = result_str
            return response
            
        except Exception as e:
            self.get_logger().error("=" * 60)
            self.get_logger().error(f"❌ 블록 검출 에러: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.get_logger().error("=" * 60)
            response.success = False
            response.message = f"에러: {str(e)}"
            return response


def main(args=None):
    rclpy.init(args=args)
    node = ColorBlockServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C 감지, 종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()