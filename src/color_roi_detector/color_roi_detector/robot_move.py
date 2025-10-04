#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import Move   # Move.srv import
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
# from pymycobot.mycobot import MyCobot
import time


class MoveServiceServer(Node):
    def __init__(self):
        super().__init__('move_service_server')
        self.srv = self.create_service(Move, 'move_service', self.callback)
        self.get_logger().info("Move Service Server Ready!")
        # self.mc = MyCobot('/dev/ttyAMA0', 115200)
        # self.mc.power_on()
    def callback(self, request, response):
        # 요청받은 문자열 출력
        coords = request.result
        # coords2 = list(coords) + [0,0,0,0]
        self.get_logger().info(f"요청 수신: '{coords}'")
        # 로봇 움직임
        # self.mc.send_coords(coords, speed = 50, mode = 0)
        # 응답 설정
        response.success = True
        response.feedback = f"좌표 '{request.result}' 잘 받았습니다."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MoveServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
