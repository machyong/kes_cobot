#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from pymycobot import MyCobot320
import time

class CalibrationRobotNode(Node):
    def __init__(self):
        super().__init__('calibration_robot_node')
        
        # 로봇 연결
        try:
            self.mc = MyCobot320('/dev/ttyAMA0', 115200)
            self.mc.power_on()
            time.sleep(1)
            self.get_logger().info("✅ 로봇 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ 로봇 연결 실패: {e}")
            raise
        
        # ===== 서비스 제공 =====
        # 1) 현재 좌표 읽기
        self.get_coords_srv = self.create_service(
            Trigger,
            '/calibration/get_robot_coords',
            self.handle_get_coords
        )
        
        # 2) 특정 위치로 이동 (픽셀 좌표 근처)
        self.move_to_srv = self.create_service(
            Trigger,
            '/calibration/move_to_point',
            self.handle_move_to_point
        )
        
        # Home 위치
        self.home_angles = [90, 0, -90, 0, 90, 0]
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 캘리브레이션 로봇 노드 준비 완료")
        self.get_logger().info("서비스:")
        self.get_logger().info("  - /calibration/get_robot_coords")
        self.get_logger().info("  - /calibration/move_to_point")
        self.get_logger().info("=" * 60)
    
    def handle_get_coords(self, request, response):
        """현재 로봇 XY 좌표 반환"""
        try:
            coords = self.mc.get_coords()
            
            if coords and len(coords) >= 2:
                x, y = coords[0], coords[1]
                
                # "X,Y" 형식으로 반환
                response.success = True
                response.message = f"{x:.2f},{y:.2f}"
                
                self.get_logger().info(f"📍 현재 좌표: X={x:.2f}, Y={y:.2f}")
            else:
                response.success = False
                response.message = "좌표를 읽을 수 없습니다"
                self.get_logger().warn("⚠️ 좌표 읽기 실패")
        
        except Exception as e:
            response.success = False
            response.message = f"에러: {str(e)}"
            self.get_logger().error(f"❌ 에러: {e}")
        
        return response
    
    def handle_move_to_point(self, request, response):
        """
        ROI 코너 근처로 이동 (수동 조정 가능한 위치)
        message로 포인트 번호 받음: "0", "1", "2", "3"
        """
        try:
            # 대략적인 위치 (사용자가 수동으로 미세 조정)
            # ROI 픽셀 (440,50) ~ (940,300)
            # 대략적인 로봇 좌표 (기존 캘리브레이션 참고)
            
            approximate_positions = [
                [138.5, -203.0, 200.0, 0, 180, 0],  # 좌상단 근처
                [329.8, -162.3, 200.0, 0, 180, 0],  # 좌하단 근처
                [325.1, 163.5, 200.0, 0, 180, 0],   # 우하단 근처
                [115.5, 183.3, 200.0, 0, 180, 0]    # 우상단 근처
            ]
            
            # Home에서 출발
            self.get_logger().info("🏠 Home 위치로 이동")
            self.mc.send_angles(self.home_angles, 50)
            time.sleep(3)
            
            # 모든 코너 순회
            for i, pos in enumerate(approximate_positions):
                self.get_logger().info(f"➡️ 포인트 {i+1} 근처로 이동")
                self.mc.send_coords(pos, speed=50, mode=0)
                time.sleep(4)
                
                # 현재 좌표 출력
                coords = self.mc.get_coords()
                if coords:
                    self.get_logger().info(f"   현재 위치: X={coords[0]:.2f}, Y={coords[1]:.2f}")
                
                # 사용자가 수동 조정할 시간
                self.get_logger().info(f"   ⏸️  10초 대기 (수동 조정 가능)")
                time.sleep(10)
            
            # Home 복귀
            self.get_logger().info("🏠 Home으로 복귀")
            self.mc.send_angles(self.home_angles, 50)
            time.sleep(3)
            
            response.success = True
            response.message = "이동 완료"
        
        except Exception as e:
            response.success = False
            response.message = f"에러: {str(e)}"
            self.get_logger().error(f"❌ 이동 에러: {e}")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()