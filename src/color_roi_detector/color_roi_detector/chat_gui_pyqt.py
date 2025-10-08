#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mycobot_interfaces.srv import Move
import numpy as np
import cv2

# PyQt5 GUI
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QTextEdit, QLineEdit, QPushButton, QLabel
from PyQt5.QtCore import QTimer


class ChatClient(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'chat_client')
        QWidget.__init__(self)

        # ====== ROS 서비스 클라이언트 ======
        # 1) chat_service
        self.chat_cli = self.create_client(Trigger, 'chat_service')
        while not self.chat_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('chat_service 대기중...')

        # 2) move_service
        self.move_cli = self.create_client(Move, 'move_service')
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_service 대기중...')

        # ====== 호모그래피 (예시값) ======
        pts_pixel = np.array([[324, 200], [324, 0], [0, 200], [0, 0]], dtype=np.float32)
        pts_robot = np.array([[175.3, 74.5], [307.2, 86.0], [305.2, -122.4], [186.3, -126.6]], dtype=np.float32)
        self.H, _ = cv2.findHomography(pts_pixel, pts_robot)

        # ====== GUI 구성 ======
        self.setWindowTitle("ROS2 Chat Client with GUI")
        self.setGeometry(300, 300, 400, 500)

        layout = QVBoxLayout()
        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.input_line = QLineEdit()
        self.send_button = QPushButton("Send")

        layout.addWidget(QLabel("ROS2 Chat"))
        layout.addWidget(self.chat_display)
        layout.addWidget(self.input_line)
        layout.addWidget(self.send_button)
        self.setLayout(layout)

        # 이벤트
        self.send_button.clicked.connect(self.send_message)
        self.input_line.returnPressed.connect(self.send_message)

    # ===== GUI 입력 → chat_service 요청 =====
    def send_message(self):
        text = self.input_line.text().strip()
        if not text:
            return
        self.chat_display.append(f"나: {text}")
        self.input_line.clear()

        req = Trigger.Request()
        future = self.chat_cli.call_async(req)
        future.add_done_callback(self.handle_chat_response)

    # ===== chat_service 응답 처리 =====
    def handle_chat_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.chat_display.append(f"봇: {response.message}")

                # === 응답 파싱 ===
                color_list = []
                k2 = response.message.split("/")
                for i in range(3):
                    a = k2[i].replace(' ', '').split(":")[-1].strip('(').strip(")")
                    color_list.append(a)
                m, n = color_list[0].split(',')

                # 픽셀 → 로봇 좌표 변환
                u, v = int(m), int(n)
                uv_h = np.array([[[u, v]]], dtype=np.float32)
                XY = cv2.perspectiveTransform(uv_h, self.H)
                X, Y = XY[0][0]
                xy_str = f"{X:.2f},{Y:.2f}"
                color_list[0] = xy_str
                self.get_logger().info(f"픽셀: ({u},{v}) → 로봇: {xy_str}")

                # === move_service 호출 ===
                move_req = Move.Request()
                move_req.result = str(color_list)
                future2 = self.move_cli.call_async(move_req)
                future2.add_done_callback(self.handle_move_response)

            else:
                self.chat_display.append(f"봇: 실패 → {response.message}")

        except Exception as e:
            self.chat_display.append(f"에러: {e}")

    # ===== move_service 응답 처리 =====
    def handle_move_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.chat_display.append(f"로봇: {response.feedback}")
            else:
                self.chat_display.append(f"로봇 실패: {response.feedback}")
        except Exception as e:
            self.chat_display.append(f"로봇 응답 에러: {e}")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = ChatClient()
    gui.show()

    # ROS2 + Qt 이벤트 루프 통합
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(gui, timeout_sec=0))
    timer.start(100)

    sys.exit(app.exec_())
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
