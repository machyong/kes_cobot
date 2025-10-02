#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import re

from mycobot_interfaces.srv import Move       # req: .result(str) / res: .success(bool), .feedback(str)
from std_srvs.srv import Trigger              # req: (none)      / res: .success(bool), .message(str)

class PixelToRobot(Node):
    """
    입력: input_topic (예: 'red_block')
      - 메시지 예시: 'Center: (164,182) / Angle: 32.5 / picking: o'

    동작 시퀀스:
      1) input_topic 구독 → 픽셀→로봇 좌표 변환(xy_str)
      2) chat_service(Trigger) 호출 → 응답 문자열을 topic_name으로 사용
      3) topic_name 으로 xy_str publish
      4) move_service(Move) 실행
      5) move 완료 후 다시 chat_service 호출 → 응답을 기다림용 topic_name으로 사용, 그 토픽에 'WAIT' publish
    """
    def __init__(self):
        super().__init__('pixel_to_robot')

        # ===== 1) 호모그래피 행렬(예시값) =====
        pts_pixel = np.array([[324, 200], [324, 0], [0, 200], [0, 0]], dtype=np.float32)
        pts_robot = np.array([[175.3, 74.5], [307.2, 86.0], [305.2, -122.4], [186.3, -126.6]], dtype=np.float32)
        self.H, _ = cv2.findHomography(pts_pixel, pts_robot)

        self.color_list = []

        # ===== 2) 서비스 클라이언트 =====
        self.chat_cli = self.create_client(Trigger, 'chat_service')
        while not self.chat_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('chat_service 대기 중...')

        self.move_cli = self.create_client(Move, 'move_service')
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_service 대기 중...')

        # ===== 3) 입력 토픽 구독 =====
        self.declare_parameter('input_topic', 'red_block')
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            String, self.input_topic, self.listener_callback, 10
        )
        self.get_logger().info(f"[Subscribe 시작] input_topic='{self.input_topic}'")

        # ===== 4) 동적 퍼블리셔 캐시 =====
        self.pub_cache = {}  # topic_name -> Publisher

    # =========================
    # 입력 콜백 (체인 시작)
    # =========================
    def listener_callback(self, msg: String):
        data = msg.data.strip()
        k = data
        k2 = k.split("/")
        for i in range(3):
            a = k2[i].replace(' ','').split(":")[-1].strip('(').strip(")")
            print(a)
            self.color_list.append(a)
        self.color_list
        m,n = self.color_list[0].split(',')
        if not m:
            return  # Center 좌표 포맷이 아닌 건 무시

        try:
            u, v = int(m.group(1)), int(n.group(2))
            uv_h = np.array([[[u, v]]], dtype=np.float32)
            XY = cv2.perspectiveTransform(uv_h, self.H)
            X, Y = XY[0][0]
            xy_str = f"{X:.2f},{Y:.2f}"
            self.color_list[0] = xy_str
            self.get_logger().info(f"픽셀: ({u},{v}) → 로봇: {xy_str}")
            self.get_logger().info(f"{self.color_list}")
            
            # 1단계: chat_service 호출
            chat_req = Trigger.Request()
            future = self.chat_cli.call_async(chat_req)
            future.add_done_callback(lambda f: self._chat_resp_then_publish_and_move(f, xy_str))

        except Exception as e:
            self.get_logger().error(f"좌표 변환 실패: {e}")

    # =========================
    # chat → publish(xy_str) → move
    # =========================
    def _chat_resp_then_publish_and_move(self, future, xy_str: str):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"chat_service 실패: {e}")
            return

        topic_name = (resp.message or "").strip()
        self.get_logger().info(f"[Chat 응답1] success={resp.success}, topic_name='{topic_name}'")
        self.input_topic = topic_name
        self.subscription = self.create_subscription(
            String, self.input_topic, self.listener_callback, 10
        )
        self.get_logger().info(f"[Subscribe 시작] input_topic='{self.input_topic}'")

        if not resp.success or not topic_name:
            self.get_logger().warn("chat 응답이 실패이거나 빈 topic_name 입니다. 체인을 중단합니다.")
            return

        # 퍼블리시
        # self._publish_text(topic_name, xy_str)

        # 2단계: move_service 호출
        move_req = Move.Request()
        move_req.result = xy_str
        self.get_logger().info(f"[Move 요청] /move_service ← result='{xy_str}'")
        move_future = self.move_cli.call_async(move_req)
        move_future.add_done_callback(self._move_resp_then_chat_wait)

    # =========================
    # move → chat(대기) → publish('WAIT')
    # =========================
    def _move_resp_then_chat_wait(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"move_service 실패: {e}")
            return

        self.get_logger().info(f"[Move 응답] success={resp.success}, feedback='{resp.feedback}'")

        # 다시 chat_service 호출
        chat_req = Trigger.Request()
        chat_future = self.chat_cli.call_async(chat_req)
        chat_future.add_done_callback(self._chat_wait_resp_then_publish_wait)

    def _chat_wait_resp_then_publish_wait(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"chat 대기 전환 실패: {e}")
            return

        wait_topic = (resp.message or "").strip()
        self.get_logger().info(f"[Chat 응답2 - 대기] success={resp.success}, wait_topic='{wait_topic}'")

        if not resp.success or not wait_topic:
            self.get_logger().warn("대기용 chat 응답이 실패이거나 빈 topic_name 입니다.")
            return

        # 대기 표시
        self._publish_text(wait_topic, "WAIT")

    # =========================
    # 공용 퍼블리시 함수
    # =========================
    # def _publish_text(self, topic_name: str, text: str):
    #     if topic_name not in self.pub_cache:
    #         self.pub_cache[topic_name] = self.create_publisher(String, topic_name, 10)
    #         self.get_logger().info(f"[Publisher 생성] {topic_name}")
    #     msg = String()
    #     msg.data = text
    #     self.pub_cache[topic_name].publish(msg)
    #     self.get_logger().info(f"[Publish] {topic_name} ← '{text}'")


def main(args=None):
    rclpy.init(args=args)
    node = PixelToRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
