#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QLineEdit, QPushButton
from PyQt5.QtCore import Qt
import sys

class ChatGUINode(Node):
    def __init__(self):
        super().__init__('chat_gui_node')
        self.srv = self.create_service(Trigger, 'chat_service', self.handle_chat)
        self.get_logger().info("채팅형 chat_service 준비 완료!")
        self.last_response = None
        # 색상 매핑 딕셔너리
        self.topic_dict = {
            "빨간색": "/red_block",
            "노란색": "/yellow_block",
            "초록색": "/green_block"
        }

        self.text = {"/red_block": "빨간색입니다.",
            "/yellow_block": "노란색입니다.",
            "/green_block": "초록색입니다."}

        # 선택된 색상 토픽
        self.selected_topic = None

        # GUI 구성
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("DMILLION")
        self.window.setGeometry(100, 100, 400, 500)

        self.layout = QVBoxLayout()

        # 채팅 로그창
        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.layout.addWidget(self.chat_display)

        # 입력창 + 전송 버튼
        input_layout = QHBoxLayout()
        self.input_line = QLineEdit()
        self.input_line.setPlaceholderText("예: 빨간색 / 노란색 / 초록색 입력...")
        self.send_button = QPushButton("전송")
        self.send_button.clicked.connect(self.handle_user_input)

        # ✅ Enter 키로 전송 버튼 누르기
        self.input_line.returnPressed.connect(self.send_button.click)   

        input_layout.addWidget(self.input_line)
        input_layout.addWidget(self.send_button)
        self.layout.addLayout(input_layout)

        self.window.setLayout(self.layout)
        self.window.show()

    def handle_user_input(self):
        user_text = self.input_line.text().strip()
        if not user_text:
            return

        # 사용자 입력 표시
        self.chat_display.append(f"👤 사용자: {user_text}")
        self.input_line.clear()

        # 색상 확인
        if user_text in self.topic_dict:
            topic = self.topic_dict[user_text]
            self.selected_topic = topic
            self.chat_display.append(f"🤖 시스템: '{user_text}' 색상이 선택되었습니다 ({topic})\n")
        else:
            self.selected_topic = None
            self.chat_display.append(f"🤖 시스템: 지원하지 않는 색상입니다. (빨간색/노란색/초록색)\n")

    def handle_chat(self, request, response):
        # 이전에 보낸 응답과 동일하면 재전송 방지
        if self.last_response == self.selected_topic:
            return response

        if self.selected_topic:
            response.success = True
            response.message = self.selected_topic
            self.chat_display.append(f"✅ 서비스 응답: {self.text.get(self.selected_topic)}\n")
        else:
            response.success = False
            response.message = "색상이 선택되지 않았습니다."
            self.chat_display.append(f"⚠️ 서비스 응답: 색상이 선택되지 않았습니다.\n")

        self.last_response = self.selected_topic  # 상태 저장
        return response


    def spin_with_gui(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.app.processEvents()

def main(args=None):
    rclpy.init(args=args)
    node = ChatGUINode()
    try:
        while True:
            node.spin_with_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
