#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import sys
import re   # 정규식 사용
from mycobot_interfaces.srv import Move
from std_srvs.srv import Trigger

class PixelToRobot(Node):
    def __init__(self, topic_name):
        super().__init__('pixel_to_robot')

        # ==============================
        # 1. 호모그래피 행렬 계산
        # ==============================
        pts_pixel = np.array([[324,200], [324,0], [0,200], [0,0]], dtype=np.float32)
        pts_robot = np.array([[175.3, 74.5], [307.2, 86.0], [305.2, -122.4], [186.3, -126.6]], dtype=np.float32)
        self.H, _ = cv2.findHomography(pts_pixel, pts_robot)

        # ==============================
        # 2. 토픽 구독 시작
        # ==============================
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

        # ==============================
        # 2. 서비스 클라이언트 생성
        # ==============================
        self.cli = self.create_client(Move, 'move_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        self.req = Move.Request()

    def listener_callback(self, msg: String):
        try:
            data = msg.data
            match = re.search(r"Center:\s*\((\d+),\s*(\d+)\)", data)
            if not match:
                self.get_logger().warn(f"좌표를 찾지 못했습니다: {data}")
                return

            u, v = int(match.group(1)), int(match.group(2))
            uv_h = np.array([[[u, v]]], dtype=np.float32)
            XY = cv2.perspectiveTransform(uv_h, self.H)
            X, Y = XY[0][0]

            # 문자열로 묶기
            xy_str = f"{X:.2f},{Y:.2f}"
            self.get_logger().info(f"픽셀: ({u},{v}) → 로봇: {xy_str}")

            # 서비스 요청 보내기
            self.req.result = xy_str
            future = self.cli.call_async(self.req)

            def response_callback(future):
                try:
                    response = future.result()
                    self.get_logger().info(f"서비스 응답: success={response.success}, feedback='{response.feedback}'")
                except Exception as e:
                    self.get_logger().error(f"서비스 요청 실패: {e}")

            future.add_done_callback(response_callback)

        except Exception as e:
            self.get_logger().error(f"좌표 변환 실패: {e}")


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("사용법: ros2 run [패키지명] pixel_to_robot.py [토픽이름]")
        return

    topic_name = sys.argv[1]
    node = PixelToRobot(topic_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
