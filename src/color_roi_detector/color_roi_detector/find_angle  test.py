#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ColorBlockServer(Node):
    def __init__(self):
        super().__init__('color_block_server')
        # 각각의 색상에 대한 서비스 생성
        self.srv_red = self.create_service(Trigger, '/red_block', self.handle_red)
        self.srv_yellow = self.create_service(Trigger, '/yellow_block', self.handle_yellow)
        self.srv_green = self.create_service(Trigger, '/green_block', self.handle_green)
        self.get_logger().info("🎨 color_block_server 준비 완료 (red/yellow/green)")

    # === 콜백 함수들 ===
    def handle_red(self, request, response):
        self.get_logger().info("🔴 red_block 요청 받음")
        response.success = True
        response.message = "Center:(164,182)/Angle:32.5/picking:빨간색"
        return response

    def handle_yellow(self, request, response):
        self.get_logger().info("🟡 yellow_block 요청 받음")
        response.success = True
        response.message = "Center:(164,182)/Angle:32.5/picking:노란색"
        return response

    def handle_green(self, request, response):
        self.get_logger().info("🟢 green_block 요청 받음")
        response.success = True
        response.message = "Center:(164,182)/Angle:32.5/picking:초록색"
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

if __name__ == "__main__":
    main()
