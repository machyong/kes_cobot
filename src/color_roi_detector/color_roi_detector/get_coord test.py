#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mycobot_interfaces.srv import Move
import numpy as np
import cv2
import time

class ChatClient(Node):
    def __init__(self):
        super().__init__('chat_client')

        # ===== 1) 호모그래피 행렬 =====
        pts_pixel = np.array([[210.0, 0.], [210.0, 130.0], [0.0, 130.0], [0.0,0.0]], dtype=np.float32)
        pts_robot = np.array([[203, 115.5], [323.1, 115.5], [323.1, -95.9], [203.4, -95.9]], dtype=np.float32)
        self.H, _ = cv2.findHomography(pts_pixel, pts_robot)

        # ===== 2) 서비스 클라이언트 =====
        self.chat_cli = self.create_client(Trigger, 'chat_service')
        while not self.chat_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('chat_service 대기중...')

        self.move_cli = self.create_client(Move, 'move_service')
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_service 대기중...')

        # ===== 3) 내부 상태 =====
        self.color_list = []
        self.chat_future = None
        self.retry_timer = None

        # ===== 4) 첫 호출 시작 =====
        self.get_logger().info("chat_service 요청 시작")
        self.call_chat_service()

    # === chat_service 호출 ===
    def call_chat_service(self):
        """Trigger 서비스 요청 (비동기)"""
        req = Trigger.Request()
        self.chat_future = self.chat_cli.call_async(req)
        self.chat_future.add_done_callback(self.handle_chat_response)

    # === chat_service 응답 ===
    def handle_chat_response(self, future):
        try:
            response = future.result()
            if response.success:
                # ✅ 성공 시: 색상 토픽 이름 받아서 다음 단계로
                service_name = response.message.strip()
                self.get_logger().info(f"[chat_service 응답] 선택된 서비스: {service_name}")

                # find_angle 서비스 호출
                cli = self.create_client(Trigger, service_name)
                while not cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f"{service_name} 대기중...")

                req = Trigger.Request()
                future2 = cli.call_async(req)
                future2.add_done_callback(self.handle_find_angle_response)

            else:
                # ❌ 실패: 색상 미선택 상태 → 1초 후 재시도
                self.get_logger().warn(f"chat_service 실패: {response.message}")
                if not self.retry_timer:
                    self.retry_timer = self.create_timer(1.0, self.retry_chat_service)

        except Exception as e:
            self.get_logger().error(f"chat_service 예외: {e}")
            if not self.retry_timer:
                self.retry_timer = self.create_timer(1.0, self.retry_chat_service)

    # === chat_service 재시도 ===
    def retry_chat_service(self):
        """1초마다 chat_service 재호출"""
        if self.chat_future is None or self.chat_future.done():
            self.get_logger().info("chat_service 재시도...")
            self.call_chat_service()
        # retry_timer는 계속 유지 (GUI에서 색 선택 시 자동 종료)

    # === find_angle 응답 ===
    def handle_find_angle_response(self, future):
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info(f"[find_angle 응답] {result.message}")

                # 문자열 파싱
                k2 = result.message.split("/")
                self.color_list = []
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
                future3 = self.move_cli.call_async(move_req)
                future3.add_done_callback(self.handle_move_response)
            else:
                self.get_logger().warn(f"find_angle 실패 또는 응답 없음")

        except Exception as e:
            self.get_logger().error(f"find_angle 예외: {e}")

    # === move_service 응답 ===
    def handle_move_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"[move_service 완료] {response.feedback}")
            else:
                self.get_logger().warn(f"move_service 실패: {response.feedback}")
        except Exception as e:
            self.get_logger().error(f"move_service 예외: {e}")

        # move 완료 후 다시 chat_service 요청 (반복 사이클)
        self.get_logger().info("다시 chat_service 요청 대기중...")
        self.call_chat_service()


def main(args=None):
    rclpy.init(args=args)
    node = ChatClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
