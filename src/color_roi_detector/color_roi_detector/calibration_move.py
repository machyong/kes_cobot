#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import Move
from pymycobot.mycobot import MyCobot
import time
import ast

class CalibrationMove(Node):
    def __init__(self):
        super().__init__('calibration_move')
        
        # MyCobot 연결
        import serial.tools.list_ports
        plist = list(serial.tools.list_ports.comports())
        port = None
        for p in plist:
            if 'USB' in p.device or 'ACM' in p.device:
                port = p.device
                break
        
        if port is None:
            self.get_logger().error("MyCobot 포트를 찾을 수 없습니다!")
            return
        
        self.mc = MyCobot(port, 115200)
        self.get_logger().info(f"MyCobot 연결: {port}")
        
        # 그리퍼 자세
        self.gripper_rx = 0.0
        self.gripper_ry = 180.0
        
        # Z 오프셋
        self.z_safe_offset = 150.0
        self.z_ready_offset = 100.0
        self.z_approach_offset = 50.0
        
        # 안전 Z
        self.z_min_safe = 140.0
        
        # 서비스 생성
        self.srv = self.create_service(
            Move,
            'calibration_move_service',
            self.callback
        )
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 캘리브레이션 이동 서비스 시작")
        self.get_logger().info("   목표 위치로 이동 후 정지")
        self.get_logger().info("=" * 60)
    
    def safe_z(self, z):
        """안전 Z값 보장"""
        return max(z, self.z_min_safe)
    
    def callback(self, request, response):
        try:
            coords = request.result
            self.get_logger().info(f"요청 수신: '{coords}'")
            
            coo_list = ast.literal_eval(coords)
            
            # X, Y 추출
            x, y = coo_list[0].split(',')
            x2, y2 = float(x), float(y)
            
            # 각도
            theta = float(coo_list[1])
            
            # Z값
            if len(coo_list) >= 4:
                z_pick_raw = float(coo_list[3])
            else:
                z_pick_raw = 165.0
            
            z_pick = self.safe_z(z_pick_raw)
            
            # 높이 계산
            z_safe = max(z_pick + self.z_safe_offset, 300.0)
            z_ready = max(z_pick + self.z_ready_offset, 260.0)
            z_approach = max(z_pick + self.z_approach_offset, 190.0)
            
            # 그리퍼 자세
            rx = self.gripper_rx
            ry = self.gripper_ry
            rz = -theta
            
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🎯 목표: X={x2:.2f}, Y={y2:.2f}, Z={z_pick:.2f}, θ={theta:.1f}°")
            self.get_logger().info("=" * 60)
            
            # ===== 이동 시퀀스 (빠른 버전) =====
            
            # 1) 그리퍼 열기
            self.get_logger().info(f"🖐️ 그리퍼 열기")
            self.mc.set_gripper_value(100, 50, 1)
            time.sleep(0.5)  # ✅ 단축
            
            # 2) 현재 위치에서 안전 높이로
            current = self.mc.get_coords()
            if current and len(current) >= 6:
                safe_lift = [current[0], current[1], z_safe, rx, ry, current[5]]
                self.mc.send_coords(safe_lift, speed=50, mode=1)
                self.get_logger().info(f"⬆️ 안전 높이: {z_safe:.2f}")
                time.sleep(2)  # ✅ 단축
            
            # 3) 목표 상공
            target_safe = [x2, y2, z_safe, rx, ry, rz]
            self.mc.send_coords(target_safe, speed=50, mode=1)
            self.get_logger().info(f"➡️ 목표 상공: ({x2:.2f}, {y2:.2f}, {z_safe:.2f})")
            time.sleep(3)  # ✅ 단축
            
            # 4) 준비 높이
            target_ready = [x2, y2, z_ready, rx, ry, rz]
            self.mc.send_coords(target_ready, speed=40, mode=1)
            self.get_logger().info(f"⬇️ 준비 높이: {z_ready:.2f}")
            time.sleep(1.5)  # ✅ 단축
            
            # 5) 접근 높이
            target_approach = [x2, y2, z_approach, rx, ry, rz]
            self.mc.send_coords(target_approach, speed=30, mode=1)
            self.get_logger().info(f"⬇️ 접근 높이: {z_approach:.2f}")
            time.sleep(1.5)  # ✅ 단축
            
            # 6) 픽킹 높이
            target_pick = [x2, y2, z_pick, rx, ry, rz]
            self.mc.send_coords(target_pick, speed=20, mode=1)
            self.get_logger().info(f"⬇️ 픽킹 높이: {z_pick:.2f}")
            time.sleep(1.5)  # ✅ 단축
            
            # ===== ✅ 여기서 정지! 그리퍼 안 닫음, Home 안 감 =====
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ 목표 위치 도착 - 정지")
            self.get_logger().info("   실제 위치를 마우스로 클릭하세요")
            self.get_logger().info("=" * 60)
            
            response.success = True
            response.feedback = f"목표 도착: ({x2:.2f}, {y2:.2f}, {z_pick:.2f})"
            
        except Exception as e:
            self.get_logger().error(f"❌ 에러: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.feedback = f"에러: {str(e)}"
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationMove()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()