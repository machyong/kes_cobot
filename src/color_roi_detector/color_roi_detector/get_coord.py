#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mycobot_interfaces.srv import Move
import numpy as np
import cv2

class ChatClient(Node):
    def __init__(self):
        super().__init__('chat_client')

        # ===== 1) 호모그래피 행렬(예시값) =====
        pts_pixel = np.array([[210.0, 0.], [210.0, 130.0], [0.0, 130.0], [0.0,0.0]], dtype=np.float32)
        pts_robot = np.array([[203, 115.5], [323.1, 115.5], [323.1, -95.9], [203.4, -95.9]], dtype=np.float32)
        self.H, _ = cv2.findHomography(pts_pixel, pts_robot)

        # 1) chat_service 클라이언트
        self.chat_cli = self.create_client(Trigger, 'chat_service')
        while not self.chat_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('chat_service 대기중...')

        # 2) move_service 클라이언트
        self.move_cli = self.create_client(Move, 'move_service')
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_service 대기중...')

        self.color_list = []

        # 첫 호출 시작
        self.call_chat_service()

    # === chat_service 호출 ===
    def call_chat_service(self):
        req = Trigger.Request()
        future = self.chat_cli.call_async(req)
        future.add_done_callback(self.handle_chat_response)

    # === chat_service 응답 처리 ===
    def handle_chat_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"[chat_service 응답] {response.message}")

                # 응답 문자열 파싱
                self.color_list = []
                k2 = response.message.split("/")
                for i in range(3):
                    a = k2[i].replace(' ', '').split(":")[-1].strip('(').strip(")")
                    self.color_list.append(a)
                m, n = self.color_list[0].split(',')

                # 픽셀 → 로봇 좌표 변환
                u, v = int(m), int(n)
                uv_h = np.array([[[u, v]]], dtype=np.float32)
                XY = cv2.perspectiveTransform(uv_h, self.H)
                X, Y = XY[0][0]
                xy_str = f"{X:.2f},{Y:.2f}"
                self.color_list[0] = xy_str
                self.get_logger().info(f"픽셀: ({u},{v}) → 로봇: {xy_str}")
                self.get_logger().info(f"{self.color_list}")

                # move_service 호출
                move_req = Move.Request()
                move_req.result = str(self.color_list)
                future2 = self.move_cli.call_async(move_req)
                future2.add_done_callback(self.handle_move_response)

            else:
                self.get_logger().warn(f"chat_service 실패: {response.message}")
                # 실패해도 다시 입력 요청
                self.call_chat_service()

        except Exception as e:
            self.get_logger().error(f"chat_service 예외: {e}")
            self.call_chat_service()

    # === move_service 응답 처리 ===
    def handle_move_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"[move_service 완료] {response.feedback}")
            else:
                self.get_logger().warn(f"move_service 실패: {response.feedback}")
        except Exception as e:
            self.get_logger().error(f"move_service 예외: {e}")

        # move 끝나면 다시 chat_service 호출
        self.call_chat_service()

    
def main(args=None):
    rclpy.init(args=args)
    node = ChatClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
