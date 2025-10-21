#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import Move
import sys

class TestMoveNode(Node):
    def __init__(self):
        super().__init__('test_move_node')
        
        # move_service 클라이언트
        self.move_cli = self.create_client(Move, 'move_service')
        
        while not self.move_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('move_service 대기 중...')
        
        self.get_logger().info("✅ Test Move Node 준비 완료!")
    
    def send_test_coords(self, x, y, z, angle=0.0):
        """특정 XYZ 좌표로 로봇 이동"""
        xy_str = f"{x:.2f},{y:.2f}"
        angle_str = f"{angle:.1f}"
        color_abbr = "t"  # test
        z_str = f"{z:.2f}"
        
        # robot_move 형식
        final_str = f"['{xy_str}', '{angle_str}', '{color_abbr}', '{z_str}']"
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🧪 테스트 이동 명령:")
        self.get_logger().info(f"   X: {x:.2f} mm")
        self.get_logger().info(f"   Y: {y:.2f} mm")
        self.get_logger().info(f"   Z: {z:.2f} mm")
        self.get_logger().info(f"   각도: {angle:.1f}°")
        self.get_logger().info(f"   전송 데이터: {final_str}")
        self.get_logger().info("=" * 60)
        
        # 서비스 호출
        req = Move.Request()
        req.result = final_str
        
        future = self.move_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ 이동 완료: {response.feedback}")
            else:
                self.get_logger().error(f"❌ 이동 실패: {response.feedback}")
        else:
            self.get_logger().error("❌ 타임아웃")

def main(args=None):
    rclpy.init(args=args)
    node = TestMoveNode()
    
    print("\n" + "=" * 60)
    print("🧪 로봇 테스트 이동 노드")
    print("=" * 60)
    print("사용법: ros2 run color_roi_detector test_move X Y Z [각도]")
    print("예시: ros2 run color_roi_detector test_move 200 100 180 0")
    print("주의: z값 (세번재 값) 은 140 밑으로 입력하지 말것 0")
    print("=" * 60)
    
    # 명령행 인자 파싱
    if len(sys.argv) < 4:
        print("❌ 인자 부족: X Y Z 좌표를 입력하세요")
        print("예시: ros2 run color_roi_detector test_move 200 100 180")
        return
    

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        if z < 134:
            print("❌ z값이 너무 낮습니다 134 이상으로 입력하세요")
            
            return
        angle = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        
        node.send_test_coords(x, y, z, angle)
        
    except ValueError as e:
        print(f"❌ 값 오류: {e}")
    except Exception as e:
        print(f"❌ 에러: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()