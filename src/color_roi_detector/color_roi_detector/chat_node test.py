#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mycobot_interfaces.srv import Move

class ChatServer(Node):
    def __init__(self):
        super().__init__('chat_server')
        # 서비스 이름: 'chat_service'
        self.srv = self.create_service(Trigger, 'chat_service', self.handle_chat)
        self.get_logger().info("chat_service 준비 완료. (서비스 호출 시 터미널에서 입력받습니다)")
        # self.block_info = ""
        self.block_info = {"red":"", "blue":"", "yellow":"", "green":""}

        # self.create_subscription(String, "/red_block", lambda msg: self.color_callback(msg, "red"), 1)
        # self.create_subscription(String, "/blue_block", lambda msg: self.color_callback(msg, "blue"), 1)
        # self.create_subscription(String, "/yellow_block", lambda msg: self.color_callback(msg, "yellow"), 1)
        # self.create_subscription(String, "/green_block", lambda msg: self.color_callback(msg, "green"), 1)

        self.color_block = ''

    # def color_callback(self, msg, color):
    #     self.block_info[color] = msg.data
    def handle_chat(self, request, response):
        user_text = input("[chat_service]  \n 1: 빨간색 \n 2: 노란색 \n 3: 초록색\n색 입력:")
        set_color = {'1': "빨간색", '2': "노란색", '3': "초록색"}
        color = set_color.get(user_text)
        topic_dict = {"빨간색":"/red", "파란색":"/blue", "노란색":"/yellow", "초록색":"/green"}
        color_key = topic_dict.get(color)
        self.color_block = color_key + '_block'
        self.color_cli = self.create_client(Trigger, self.color_block)
        while not self.color_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('color_cli')

        answer = self.get_color()
        
        if color_key is None:
            response.success = False
            response.message = "(지원하지 않는 색)"
        elif color_key[-1] == 'x':
            response.success = False
            response.message = "블록간 간격이 너무 좁습니다."
        else:
            response.success = True
            response.message = answer
        return response
    
    def get_color(self):
        req = Trigger.Request()
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = ChatServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
