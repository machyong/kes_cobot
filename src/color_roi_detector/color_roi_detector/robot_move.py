#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import Move   # Move.srv import
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from pymycobot import MyCobot320
import time
import ast

#['351.91,13.15', '84.8', 'o']

class MoveServiceServer(Node):
    def __init__(self):
        super().__init__('move_service_server')
        self.srv = self.create_service(Move, 'move_service', self.callback)
        self.get_logger().info("Move Service Server Ready!")
        # 로봇 세팅
        self.mc = MyCobot320('/dev/ttyAMA0', 115200)
        self.mc.power_on() 
        self.mc.set_gripper_mode(0) # 그리퍼 세팅
        # home위치 이동
        self.home = [90,0,-90,0,90,0]
        self.mc.send_angles(self.home, 50)

        self.trash = [130,0,-90,0,90,0]

    def callback(self, request, response):
        # 요청받은 문자열 출력
        coords = request.result
        # coords2 = list(coords) + [0,0,0,0]
        self.get_logger().info(f"요청 수신: '{coords}'")
        
        coo_list = ast.literal_eval(coords)
        x,y = coo_list[0].split(',')
        x2 , y2 = float(x), float(y)
        theta = float(coo_list[1])
        xy_list = [x2,y2]

        ready_point = self.home.copy()
        ready_point[-1] = theta
        self.get_logger().info(f"각도변화 확인: {ready_point}, theta값: {theta}")
        self.mc.send_angles(ready_point, 50)
        time.sleep(3)
        self.get_logger().info(f"gripper open")
        self.mc.set_gripper_value(80,50,1)
        time.sleep(1)
        target_point = xy_list + [180.] + [0.,180.,0.]
        # 로봇 움직임
        self.mc.send_coords(target_point, speed = 50, mode = 0)
        self.get_logger().info(f"move to point")
        time.sleep(5)

        self.get_logger().info(f"gripper close")
        self.mc.set_gripper_value(45,50,1)
        time.sleep(1)

        # trash point
        self.mc.send_angles(self.trash, 50)
        time.sleep(3)
        self.get_logger().info(f"gripper open")
        self.mc.set_gripper_value(80,50,1)
        self.get_logger().info(f"물체 버림")
        time.sleep(1)
        

        self.mc.send_angles(self.home, 50)
        self.get_logger().info(f"대기상태")
        time.sleep(3)
        self.get_logger().info(f"gripper close")
        self.mc.set_gripper_value(45,50,1)

        # 응답 설정
        response.success = True
        response.feedback = f"좌표 {request.result} 잘 받았습니다."
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
