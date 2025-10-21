#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QLineEdit, QPushButton, QLabel
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont, QKeyEvent
import sys
import cv2
import numpy as np
import re

class ChatGUINode(Node):
    def __init__(self, app):
        super().__init__('chat_gui_node')
        self.bridge = CvBridge()
        self.app = app
        
        # ===== ROI 설정 =====
        self.roi_x = 440
        self.roi_y = 280
        self.roi_w = 380
        self.roi_h = 220
        
        # 최신 이미지 및 검출 결과
        self.latest_image = None
        self.detection_result = None
        self.all_detections = []
        
        # ===== 색상명 영어 매핑 =====
        self.color_to_english = {
            "빨간색": "Red",
            "노란색": "Yellow",
            "초록색": "Green",
            "파란색": "Blue",
            "주황색": "Orange"
        }
        
        # 색상 유사어 사전
        self.color_synonyms = {
            "빨간색": ["빨간색", "빨강", "레드", "red", "붉은색", "빨강색", "빨강이", "빨개", "빨간", "뻘건색", "뻘건", "적색", "Red", "RED"],
            "노란색": ["노란색", "노랑", "옐로우", "yellow", "노랑색", "노란", "노래", "누런색", "누런", "황색", "Yellow", "YELLOW"],
            "초록색": ["초록색", "초록", "그린", "green", "녹색", "초록이", "초록빛", "초딩색", "풀색", "Green", "GREEN"],
            "파란색": ["파란색", "파랑", "블루", "blue", "파랑색", "파란", "파래", "푸른색", "푸른", "청색", "Blue", "BLUE"],
            "주황색": ["주황색", "주황", "오렌지", "orange", "주황이", "귤색", "Orange", "ORANGE"]
        }
        
        # 위치 키워드 사전
        self.position_keywords = {
            "위": ["위", "상단", "맨위", "맨 위", "top", "위쪽", "위에"],
            "아래": ["아래", "하단", "맨아래", "맨 아래", "bottom", "아래쪽", "밑", "밑에"],
            "왼쪽": ["왼쪽", "왼편", "left", "좌측"],
            "오른쪽": ["오른쪽", "우측", "right", "오른편"],
            "중간": ["중간", "가운데", "center", "중앙"]
        }
        
        # 이미지 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 서비스
        self.srv = self.create_service(Trigger, 'chat_service', self.handle_chat)
        self.get_logger().info("채팅형 chat_service 준비 완료!")
        
        self.last_response = None
        
        # ===== ✅ 서비스 이름 수정 =====
        self.topic_dict = {
            "빨간색": "red_service",
            "노란색": "yellow_service",
            "초록색": "green_service",
            "파란색": "blue_service",
            "주황색": "orange_service"
        }
        
        self.text = {
            "red_service": "빨간색입니다.",
            "yellow_service": "노란색입니다.",
            "green_service": "초록색입니다.",
            "blue_service": "파란색입니다.",
            "orange_service": "주황색입니다."
        }
        
        self.selected_topic = None
        
        # ===== GUI 구성 =====
        self.window = QWidget()
        self.window.setWindowTitle("DMILLION - 객체 검출 시스템")
        self.window.setGeometry(50, 50, 1400, 800)
        
        main_layout = QHBoxLayout()
        
        # 왼쪽: 카메라 뷰
        left_layout = QVBoxLayout()
        
        self.camera_label = QLabel("카메라 대기 중...")
        self.camera_label.setMinimumSize(960, 720)
        self.camera_label.setStyleSheet("border: 2px solid #2196F3; background-color: #000;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.camera_label)
        
        self.info_label = QLabel("대기 중...")
        self.info_label.setStyleSheet("""
            background-color: #263238;
            color: #00E676;
            padding: 10px;
            border-radius: 5px;
            font-size: 14px;
            font-family: 'Courier New';
        """)
        self.info_label.setMinimumHeight(60)
        left_layout.addWidget(self.info_label)
        
        # 오른쪽: 채팅 UI
        right_layout = QVBoxLayout()
        right_layout.setSpacing(10)
        
        title_label = QLabel("🤖 색상 및 위치 선택")
        title_label.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #2196F3;
            padding: 10px;
        """)
        right_layout.addWidget(title_label)
        
        # 도움말
        help_label = QLabel("💡 예시: '빨간색', 'red', '위에 있는거', '맨 위 집어줘'")
        help_label.setStyleSheet("""
            font-size: 12px;
            color: #BDBDBD;
            padding: 5px;
        """)
        right_layout.addWidget(help_label)
        
        # 채팅 로그창
        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.chat_display.setStyleSheet("""
            background-color: #1E1E1E;
            color: #E0E0E0;
            border: 2px solid #2196F3;
            border-radius: 5px;
            padding: 10px;
            font-size: 13px;
        """)
        right_layout.addWidget(self.chat_display)
        
        # 입력창
        input_layout = QHBoxLayout()
        self.input_line = CustomLineEdit()
        self.input_line.setPlaceholderText("색상이나 위치를 입력하세요...")
        self.input_line.setStyleSheet("""
            background-color: #2C2C2C;
            color: #FFFFFF;
            border: 2px solid #2196F3;
            border-radius: 5px;
            padding: 8px;
            font-size: 14px;
        """)
        self.send_button = QPushButton("전송")
        self.send_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 8px 20px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """)
        
        # 버튼과 엔터키 연결
        self.send_button.clicked.connect(self.handle_user_input)
        self.input_line.returnPressed.connect(self.send_button.click)
        
        input_layout.addWidget(self.input_line)
        input_layout.addWidget(self.send_button)
        right_layout.addLayout(input_layout)
        
        main_layout.addLayout(left_layout, 7)
        main_layout.addLayout(right_layout, 3)
        
        self.window.setLayout(main_layout)
        self.window.show()
        
        # 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(33)
        
        self.get_logger().info("✅ GUI 초기화 완료!")
    
    def image_callback(self, msg):
        """카메라 이미지 수신"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def update_display(self):
        """카메라 이미지 업데이트"""
        if self.latest_image is None:
            return
        
        display_img = self.latest_image.copy()
        
        # ROI 박스
        cv2.rectangle(display_img,
                    (self.roi_x, self.roi_y),
                    (self.roi_x + self.roi_w, self.roi_y + self.roi_h),
                    (0, 255, 0), 3)
        '''
        cv2.circle(display_img,
                    (self.roi_x+ 190, self.roi_y + 110),
                    10,
                    (255, 0, 0), 3)
        '''     
        # 검출 결과 표시
        if self.detection_result:
            x, y, angle, color_name = self.detection_result
            
            # 한글 → 영어 변환
            color_english = self.color_to_english.get(color_name, color_name)
            
            # 십자선
            cv2.drawMarker(display_img, (x, y), (0, 0, 255), 
                        cv2.MARKER_CROSS, 30, 3)
            
            # 원
            cv2.circle(display_img, (x, y), 50, (0, 255, 255), 3)
            
            # 텍스트
            info_text = f"{color_english}: ({x}, {y}), {angle:.1f}deg"
            cv2.putText(display_img, info_text,
                    (x - 100, y - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 각도 화살표
            length = 40
            end_x = int(x + length * np.cos(np.radians(angle)))
            end_y = int(y - length * np.sin(np.radians(angle)))
            cv2.arrowedLine(display_img, (x, y), (end_x, end_y),
                        (255, 0, 255), 3, tipLength=0.3)
        
        # 모든 검출 객체 표시
        for det in self.all_detections:
            x, y, color_name = det
            color_english = self.color_to_english.get(color_name, color_name)
            cv2.circle(display_img, (x, y), 20, (128, 128, 128), 2)
            cv2.putText(display_img, color_english[:3],
                    (x - 10, y - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
        
        # 표시
        rgb_image = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_label.setPixmap(scaled_pixmap)
    
    def handle_user_input(self):
        """사용자 입력 처리"""
        user_text = self.input_line.text().strip()
        if not user_text:
            return
        
        self.chat_display.append(f"<b style='color:#64B5F6'>👤 사용자:</b> {user_text}")
        self.input_line.clear()
        
        # 자연어 처리
        detected_color = self.extract_color(user_text)
        detected_position = self.extract_position(user_text)
        
        if detected_color:
            # 색상 기반 검출
            topic = self.topic_dict[detected_color]
            self.selected_topic = topic
            self.chat_display.append(f"<b style='color:#4CAF50'>🤖 시스템:</b> '{detected_color}' 감지됨!")
            self.info_label.setText(f"색상 선택: {detected_color}")
            self.request_detection(topic, detected_color)
            
        elif detected_position:
            # 위치 기반 검출
            self.chat_display.append(f"<b style='color:#4CAF50'>🤖 시스템:</b> '{detected_position}' 위치 감지됨!")
            self.info_label.setText(f"위치 검색: {detected_position}")
            self.detect_all_and_select_by_position(detected_position)
            
        else:
            self.chat_display.append(f"<b style='color:#FF5252'>⚠️ 시스템:</b> 색상이나 위치를 인식할 수 없습니다.")
            self.info_label.setText("인식 실패")
    
    def extract_color(self, text):
        """텍스트에서 색상 추출"""
        for standard_color, synonyms in self.color_synonyms.items():
            for synonym in synonyms:
                if synonym in text:
                    self.get_logger().info(f"색상 매칭: '{synonym}' → {standard_color}")
                    return standard_color
        return None
    
    def extract_position(self, text):
        """텍스트에서 위치 추출"""
        for position, keywords in self.position_keywords.items():
            for keyword in keywords:
                if keyword in text:
                    self.get_logger().info(f"위치 매칭: '{keyword}' → {position}")
                    return position
        return None
    
    def detect_all_and_select_by_position(self, position):
        """모든 색상 검출 후 위치 기준으로 선택"""
        self.all_detections = []
        
        # 모든 색상 검출
        for color_name, topic in self.topic_dict.items():
            cli = self.create_client(Trigger, topic)
            if not cli.wait_for_service(timeout_sec=1.0):
                continue
            
            req = Trigger.Request()
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                result = future.result()
                if result.success:
                    parts = result.message.split("/")
                    center_str = parts[0].split(":")[-1].strip("()")
                    x, y = map(int, center_str.split(","))
                    self.all_detections.append((x, y, color_name))
        
        if len(self.all_detections) == 0:
            self.chat_display.append(f"<b style='color:#FF5252'>⚠️ Failed:</b> No objects found")
            return
        
        # 위치 기준 선택
        selected = None
        if position == "위":
            selected = min(self.all_detections, key=lambda d: d[1])
        elif position == "아래":
            selected = max(self.all_detections, key=lambda d: d[1])
        elif position == "왼쪽":
            selected = min(self.all_detections, key=lambda d: d[0])
        elif position == "오른쪽":
            selected = max(self.all_detections, key=lambda d: d[0])
        elif position == "중간":
            center_x = self.roi_x + self.roi_w // 2
            center_y = self.roi_y + self.roi_h // 2
            selected = min(self.all_detections, 
                        key=lambda d: ((d[0]-center_x)**2 + (d[1]-center_y)**2)**0.5)
        
        if selected:
            x, y, color_name = selected
            color_english = self.color_to_english.get(color_name, color_name)
            self.chat_display.append(f"<b style='color:#4CAF50'>✅ Selected:</b> {color_english} ({x}, {y})")
            topic = self.topic_dict[color_name]
            self.selected_topic = topic
            self.request_detection(topic, color_name)
    
    def request_detection(self, topic, color_name):
        """색상 검출 요청"""
        cli = self.create_client(Trigger, topic)
        if not cli.wait_for_service(timeout_sec=2.0):
            self.chat_display.append(f"<b style='color:#FF5252'>❌ 에러:</b> {topic} 서비스 없음")
            return
        
        req = Trigger.Request()
        future = cli.call_async(req)
        future.add_done_callback(lambda f: self.handle_detection_result(f, color_name))
    
    def handle_detection_result(self, future, color_name):
        """검출 결과 처리"""
        try:
            result = future.result()
            if result.success:
                parts = result.message.split("/")
                center_str = parts[0].split(":")[-1].strip("()")
                x, y = map(int, center_str.split(","))
                angle_str = parts[1].split(":")[-1]
                angle = float(angle_str)
                
                color_english = self.color_to_english.get(color_name, color_name)
                self.detection_result = (x, y, angle, color_name)
                
                self.chat_display.append(f"<b style='color:#4CAF50'>✅ Detection:</b> {color_english} ({x}, {y}), {angle:.1f}°")
                self.info_label.setText(f"{color_english} | ({x}, {y}) | {angle:.1f}°")
            else:
                self.detection_result = None
                self.chat_display.append(f"<b style='color:#FF9800'>⚠️ Failed:</b> {result.message}")
        except Exception as e:
            self.get_logger().error(f"검출 결과 처리 에러: {e}")
    
    def handle_chat(self, request, response):
        """chat_service 콜백"""
        if self.selected_topic:
            response.success = True
            response.message = self.selected_topic
            self.get_logger().info(f"✅ chat_service 응답: {self.selected_topic}")
        else:
            response.success = False
            response.message = "색상이 선택되지 않았습니다."
            self.get_logger().warn("⚠️ chat_service: 색상 미선택")
        
        return response
    
    def spin_with_gui(self):
        """GUI와 ROS2 통합"""
        rclpy.spin_once(self, timeout_sec=0.01)
        self.app.processEvents()


class CustomLineEdit(QLineEdit):
    def keyPressEvent(self, event: QKeyEvent):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            self.returnPressed.emit()
            event.accept()
            return
        super().keyPressEvent(event)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = ChatGUINode(app)
    
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