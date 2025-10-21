#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QPushButton, QTextEdit)
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont, QCursor
from PyQt5.QtCore import Qt, QTimer, QPoint, pyqtSignal  # ✅ pyqtSignal 추가
import sys
import cv2
import numpy as np
from pathlib import Path
import json

class InteractiveCalibration(Node):
    def __init__(self, app):
        super().__init__('interactive_calibration')
        self.bridge = CvBridge()
        self.app = app
        
        # ROI 설정
        self.roi_x = 440
        self.roi_y = 280
        self.roi_w = 530
        self.roi_h = 220
        
         # ===== ✅ ROI 중앙 계산 =====
        self.roi_center_x = self.roi_x + self.roi_w / 2  # 705
        self.roi_center_y = self.roi_y + self.roi_h / 2  # 390
        
        self.latest_image = None

        # 초기 캘리브레이션 코너
        self.initial_corners_pixel = np.array([
            [440.0, 280.0],
            [440.0, 500.0],
            [860.0, 500.0],
            [860.0, 280.0]
        ], dtype=np.float32)
        
        self.initial_corners_robot = np.array([
            [126.4, -202.9],
            [299.9, -200.6],
            [300.3, 208.0],
            [125.2, 208.3]
        ], dtype=np.float32)
        
        self.H, _ = cv2.findHomography(self.initial_corners_pixel, self.initial_corners_robot)
        
        # 선형 변환 파라미터
        pixel_x_min, pixel_y_min = 440.0, 280.0
        pixel_x_max, pixel_y_max = 860.0, 500.0
        robot_x_min, robot_y_min = 126.4, -202.9
        robot_x_max, robot_y_max = 300.3, 208.0
        
        pixel_y_range = pixel_y_max - pixel_y_min
        robot_x_range = robot_x_max - robot_x_min
        self.linear_scale_x = robot_x_range / pixel_y_range
        self.linear_offset_x = robot_x_min - pixel_y_min * self.linear_scale_x
        
        pixel_x_range = pixel_x_max - pixel_x_min
        robot_y_range = robot_y_max - robot_y_min
        self.linear_scale_y = robot_y_range / pixel_x_range
        self.linear_offset_y = robot_y_min - pixel_x_min * self.linear_scale_y
        
        self.calibration_points = []
        
        # 상태
        self.state = "IDLE"
        self.target_pixel = None
        self.target_robot_homo = None
        self.target_robot_linear = None
        self.actual_pixel = None
        
        # ===== ✅ 로봇 서비스 클라이언트 =====
        from mycobot_interfaces.srv import Move
        
        # 좌표 읽기용
        self.get_coords_cli = self.create_client(Trigger, '/calibration/get_robot_coords')
        
        # ✅ 캘리브레이션용 이동 (정지)
        self.calibration_move_cli = self.create_client(Move, 'calibration_move_service')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("서비스 연결 확인 중...")
        self.get_logger().info("  - calibration_move_service (이동 후 정지)")
        self.get_logger().info("  - calibration/get_robot_coords (좌표 읽기)")
        self.get_logger().info("=" * 60)
        
        # 상태
        self.state = "IDLE"
        self.target_pixel = None
        self.target_robot_homo = None      # 호모그래피 결과
        self.target_robot_linear = None    # 선형 변환 결과
        self.actual_pixel = None
        self.use_method = "both"  # "homo", "linear", "both"
        
        # 로봇 서비스
        self.get_coords_cli = self.create_client(Trigger, '/calibration/get_robot_coords')

         # 로봇 이동용
        from mycobot_interfaces.srv import Move
        self.move_cli = self.create_client(Move, 'move_service')
        
        # 서비스 연결 대기 (비차단)
        self.get_logger().info("서비스 연결 확인 중...")
        
        # 이미지 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # ===== GUI =====
        self.window = QWidget()
        self.window.setWindowTitle("🎯 인터랙티브 캘리브레이션")
        self.window.setGeometry(50, 50, 1400, 850)
        
        main_layout = QHBoxLayout()
        
        # 왼쪽: 카메라
        left_layout = QVBoxLayout()
        
        self.camera_label = ClickableLabel()
        self.camera_label.setMinimumSize(960, 720)
        self.camera_label.setStyleSheet("border: 3px solid #FF5722; background-color: #000;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.clicked.connect(self.handle_click)
        left_layout.addWidget(self.camera_label)
        
        # 오른쪽: 컨트롤
        right_layout = QVBoxLayout()
        
        title = QLabel("🎯 인터랙티브 캘리브레이션")
        title.setStyleSheet("font-size: 22px; font-weight: bold; color: #FF5722;")
        right_layout.addWidget(title)
        
        # 사용법
        instructions = QLabel(
            "📋 !!사용법:\n\n"
            "1️⃣ ROI 내 원하는 위치 클릭\n"
            "   → 로봇이 자동 이동\n\n"
            "2️⃣ 로봇이 실제 도착한 위치 클릭\n"
            "   → 오차 계산 및 학습\n\n"
            "3️⃣ 여러 위치 반복 테스트\n"
            "   → 자동으로 정확도 향상\n\n"
            "4️⃣ '캘리브레이션 저장' 클릭"
        )
        instructions.setStyleSheet("""
            font-size: 13px;
            padding: 15px;
            background-color: #FFF3E0;
            border-radius: 5px;
            line-height: 1.6;
        """)
        right_layout.addWidget(instructions)
        
        # 상태 표시
        self.status_label = QLabel("🟢 준비됨: ROI 내 클릭하세요")
        self.status_label.setStyleSheet("""
            font-size: 15px;
            font-weight: bold;
            padding: 12px;
            background-color: #263238;
            color: #00E676;
            border-radius: 5px;
        """)
        right_layout.addWidget(self.status_label)
        
        # 통계
        self.stats_label = QLabel("📊 통계:\n  테스트 횟수: 0\n  평균 오차: -")
        self.stats_label.setStyleSheet("""
            font-size: 13px;
            padding: 10px;
            background-color: #E3F2FD;
            border-radius: 5px;
        """)
        right_layout.addWidget(self.stats_label)
        
        # 로그
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setStyleSheet("""
            background-color: #1E1E1E;
            color: #E0E0E0;
            border: 2px solid #FF5722;
            border-radius: 5px;
            padding: 10px;
            font-size: 12px;
        """)
        right_layout.addWidget(self.log_display)
        
        # 버튼
        button_layout = QVBoxLayout()
        
        self.home_button = QPushButton("🏠 Home 복귀")
        self.home_button.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 12px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #7B1FA2; }
        """)
        self.home_button.clicked.connect(self.send_robot_home)
        button_layout.addWidget(self.home_button)
        
        self.save_button = QPushButton("💾 캘리브레이션 저장")
        self.save_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 12px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #388E3C; }
        """)
        self.save_button.clicked.connect(self.save_calibration)
        button_layout.addWidget(self.save_button)
        
        self.reset_button = QPushButton("🔄 초기화")
        self.reset_button.setStyleSheet("""
            QPushButton {
                background-color: #9E9E9E;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
                font-size: 13px;
            }
            QPushButton:hover { background-color: #757575; }
        """)
        self.reset_button.clicked.connect(self.reset_calibration)
        button_layout.addWidget(self.reset_button)
        
        right_layout.addLayout(button_layout)
        
        main_layout.addLayout(left_layout, 7)
        main_layout.addLayout(right_layout, 3)
        
        self.window.setLayout(main_layout)
        self.window.show()
        
        # 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(33)
        
        # 오차 기록
        self.errors = []
        
        self.add_log("🎯 인터랙티브 캘리브레이션 시작")
        self.add_log("ROI 내 아무 곳이나 클릭하세요!")
    
    def correct_pixel(self, x, y):
        """
        픽셀 좌표 보정
        1. X를 10~20픽셀 오른쪽으로, Y가 작을수록 더 많이
        2. X가 중앙보다 왼쪽이면 Y를 20~30 아래로, 오른쪽이면 10
        """
        
        corrected_y = y 
    
        # 로그
        #self.add_log(f"📐 픽셀 보정: ({x}, {y}) → ({corrected_x:.0f}, {corrected_y:.0f})")
        #self.add_log(f"   X offset: {x_offset:+.1f}, Y offset: {y_offset:+.1f}")
        self.add_log(f"   X offset: {x:+.1f}, Y offset: {y:+.1f}")
        
        return int(x), int(corrected_y)
        
    def linear_transform(self, pixel_x, pixel_y):
        """
        단순 선형 변환
        좌표축 매핑: 픽셀(X,Y) → 로봇(Y방향, X방향)
        이미 보정된 픽셀 좌표를 받음
        """
        robot_x = pixel_y * self.linear_scale_x + self.linear_offset_x
        robot_y = pixel_x * self.linear_scale_y + self.linear_offset_y
        return robot_x, robot_y

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
    def update_display(self):
        if self.latest_image is None:
            return
        
        display_img = self.latest_image.copy()
        
        # ROI
        cv2.rectangle(display_img,
                    (self.roi_x, self.roi_y),
                    (self.roi_x + self.roi_w, self.roi_y + self.roi_h),
                    (0, 255, 0), 3)
        cv2.putText(display_img, "ROI - Click Here",
                (self.roi_x + 10, self.roi_y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        # ===== ✅ 목표 지점 (클릭한 위치) =====
        if self.target_pixel:
            cv2.drawMarker(display_img, self.target_pixel, (0, 0, 255),
                        cv2.MARKER_CROSS, 40, 3)
            cv2.circle(display_img, self.target_pixel, 50, (0, 255, 255), 2)
            cv2.putText(display_img, "Target", 
                    (self.target_pixel[0] - 30, self.target_pixel[1] - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 실제 도착 지점
        if self.actual_pixel:
            cv2.drawMarker(display_img, self.actual_pixel, (255, 0, 0),
                        cv2.MARKER_DIAMOND, 40, 3)
            cv2.circle(display_img, self.actual_pixel, 50, (255, 0, 255), 2)
            cv2.putText(display_img, "Actual",
                    (self.actual_pixel[0] - 30, self.actual_pixel[1] + 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            # 오차 선
            cv2.arrowedLine(display_img, self.target_pixel, self.actual_pixel,
                        (255, 255, 0), 3, tipLength=0.2)
            
            # 오차 거리
            dx = self.actual_pixel[0] - self.target_pixel[0]
            dy = self.actual_pixel[1] - self.target_pixel[1]
            dist = np.sqrt(dx**2 + dy**2)
            mid_x = (self.target_pixel[0] + self.actual_pixel[0]) // 2
            mid_y = (self.target_pixel[1] + self.actual_pixel[1]) // 2
            cv2.putText(display_img, f"{dist:.1f}px",
                    (mid_x, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        # 학습된 포인트들
        for pixel, robot in self.calibration_points:
            cv2.circle(display_img, pixel, 8, (0, 255, 0), -1)
        
        # 초기 4개 코너 표시
        for i, pixel in enumerate(self.initial_corners_pixel):
            px, py = int(pixel[0]), int(pixel[1])
            cv2.circle(display_img, (px, py), 12, (255, 0, 255), 2)
            cv2.putText(display_img, str(i+1), (px-6, py+6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # 표시
        rgb_image = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.camera_label.setPixmap(pixmap)
    
    def handle_click(self, pos):
        """마우스 클릭 처리"""
        # 좌표 변환
        pixmap = self.camera_label.pixmap()
        if pixmap is None or self.latest_image is None:
            return
        
        img_h, img_w = self.latest_image.shape[:2]
        label_w = self.camera_label.width()
        label_h = self.camera_label.height()
        pixmap_w = pixmap.width()
        pixmap_h = pixmap.height()
        
        offset_x = (label_w - pixmap_w) / 2
        offset_y = (label_h - pixmap_h) / 2
        
        pixmap_x = pos.x() - offset_x
        pixmap_y = pos.y() - offset_y
        
        if pixmap_x < 0 or pixmap_x >= pixmap_w or pixmap_y < 0 or pixmap_y >= pixmap_h:
            self.add_log("⚠️ 이미지 밖 클릭!")
            return
        
        scale_w = img_w / pixmap_w
        scale_h = img_h / pixmap_h
        
        img_x = int(pixmap_x * scale_w)
        img_y = int(pixmap_y * scale_h)
        
        # ROI 체크
        if not (self.roi_x <= img_x <= self.roi_x + self.roi_w and
                self.roi_y <= img_y <= self.roi_y + self.roi_h):
            self.add_log("⚠️ ROI 밖 클릭! ROI 내부를 클릭하세요")
            return
        
        if self.state == "IDLE":
            # ===== ✅ 픽셀 보정 적용 =====
            img_x_corrected, img_y_corrected = self.correct_pixel(img_x, img_y)
            
            self.target_pixel = (img_x, img_y)  # 원본 저장 (표시용)
            self.actual_pixel = None
            
            # 방법 1: 호모그래피 (보정된 좌표 사용)
            pixel_h = np.array([[img_x_corrected, img_y_corrected]], dtype=np.float32).reshape(1, 1, 2)
            robot_coords_homo = cv2.perspectiveTransform(pixel_h, self.H)
            robot_x_homo, robot_y_homo = robot_coords_homo[0][0]
            self.target_robot_homo = (robot_x_homo, robot_y_homo)
            
            # 방법 2: 선형 변환 (보정된 좌표 사용)
            robot_x_linear, robot_y_linear = self.linear_transform(img_x_corrected, img_y_corrected)
            self.target_robot_linear = (robot_x_linear, robot_y_linear)
            
            # 로그
            self.add_log("=" * 50)
            self.add_log(f"📍 원본 픽셀: ({img_x}, {img_y})")
            self.add_log(f"📐 보정 픽셀: ({img_x_corrected}, {img_y_corrected})")
            self.add_log(f"🔷 호모그래피: ({robot_x_homo:.2f}, {robot_y_homo:.2f})")
            self.add_log(f"📐 선형변환:   ({robot_x_linear:.2f}, {robot_y_linear:.2f})")
            
            # 차이 계산
            diff_x = abs(robot_x_homo - robot_x_linear)
            diff_y = abs(robot_y_homo - robot_y_linear)
            diff_dist = np.sqrt(diff_x**2 + diff_y**2)
            self.add_log(f"⚠️ 차이: ΔX={diff_x:.2f}mm, ΔY={diff_y:.2f}mm, 거리={diff_dist:.2f}mm")
            self.add_log("=" * 50)
            
            self.status_label.setText("🤖 로봇 이동 중 (2회)...")
            
            # 먼저 호모그래피 방법으로 이동
            self.add_log("🔷 [1/2] 호모그래피 방법으로 이동 시작")
            self.move_robot_to(robot_x_homo, robot_y_homo, method="homo")
            
            self.state = "WAITING_LINEAR"
            
        elif self.state == "WAITING_LINEAR":
            # 호모그래피 완료, 선형 변환 시도
            self.add_log("📐 [2/2] 선형변환 방법으로 이동 시작")
            self.move_robot_to(self.target_robot_linear[0], self.target_robot_linear[1], method="linear")
            
            self.state = "WAITING_ACTUAL"
            self.status_label.setText("📍 로봇이 실제 도착한 위치를 클릭하세요")
            
        elif self.state == "WAITING_ACTUAL":
            # ===== ✅ 실제 위치도 보정 적용하지 않음 (순수 측정) =====
            self.actual_pixel = (img_x, img_y)  # 보정 안한 원본 사용
            self.add_log(f"✅ 실제 위치: 픽셀({img_x}, {img_y})")
            
            # 오차 계산
            self.calculate_and_learn()
            
            self.state = "IDLE"
            self.status_label.setText("🟢 준비됨: 다음 위치 클릭하세요")
    
    def move_robot_to(self, x, y, method="unknown"):
        """로봇을 특정 XY 위치로 이동 (캘리브레이션용 - 정지)"""
        try:
            # ===== ✅ calibration_move_service 사용 =====
            if not self.calibration_move_cli.wait_for_service(timeout_sec=2.0):
                self.add_log("❌ calibration_move_service 연결 안됨!")
                self.add_log("   mycobot에서 실행 필요:")
                self.add_log("   ros2 run color_roi_detector calibration_move")
                self.state = "IDLE"
                self.status_label.setText("⚠️ calibration_move 실행 필요")
                return
            
            # 요청 생성
            z = 165.0
            angle = 0.0
            xy_str = f"{x:.2f},{y:.2f}"
            final_str = f"['{xy_str}', '{angle:.1f}', 't', '{z:.2f}']"
            
            method_name = {"homo": "호모그래피", "linear": "선형변환"}.get(method, method)
            self.add_log(f"🤖 [{method_name}] 이동 명령: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            
            from mycobot_interfaces.srv import Move
            req = Move.Request()
            req.result = final_str
            
            future = self.calibration_move_cli.call_async(req)
            
            def move_callback(future):
                try:
                    response = future.result()
                    if response.success:
                        self.add_log(f"✅ [{method_name}] 도착 - 정지 상태 (픽킹 안함)")
                    else:
                        self.add_log(f"❌ [{method_name}] 실패: {response.feedback}")
                except Exception as e:
                    self.add_log(f"❌ [{method_name}] 응답 에러: {e}")
            
            future.add_done_callback(move_callback)
            
        except Exception as e:
            self.add_log(f"❌ move_robot_to 에러: {e}")
            import traceback
            self.add_log(traceback.format_exc())
            self.state = "IDLE"
            self.status_label.setText("⚠️ 에러 발생")

    def calculate_and_learn(self):
        """오차 계산 및 학습"""
        if not self.target_pixel or not self.actual_pixel:
            return
        
        # 픽셀 오차
        dx_pixel = self.actual_pixel[0] - self.target_pixel[0]
        dy_pixel = self.actual_pixel[1] - self.target_pixel[1]
        error_pixel = np.sqrt(dx_pixel**2 + dy_pixel**2)
        
        # 실제 위치의 로봇 좌표 읽기
        req = Trigger.Request()
        future = self.get_coords_cli.call_async(req)
        
        def callback(future):
            try:
                response = future.result()
                if response.success:
                    coords_str = response.message
                    actual_x, actual_y = map(float, coords_str.split(','))
                    
                    # ===== ✅ 두 방법 모두와 비교 =====
                    # 호모그래피와의 오차
                    dx_homo = actual_x - self.target_robot_homo[0]
                    dy_homo = actual_y - self.target_robot_homo[1]
                    error_homo = np.sqrt(dx_homo**2 + dy_homo**2)
                    
                    # 선형 변환과의 오차
                    dx_linear = actual_x - self.target_robot_linear[0]
                    dy_linear = actual_y - self.target_robot_linear[1]
                    error_linear = np.sqrt(dx_linear**2 + dy_linear**2)
                    
                    self.add_log("=" * 50)
                    self.add_log(f"📊 오차 분석:")
                    self.add_log(f"  픽셀 오차: {error_pixel:.1f} px")
                    self.add_log(f"")
                    self.add_log(f"  실제 위치: ({actual_x:.2f}, {actual_y:.2f})")
                    self.add_log(f"")
                    self.add_log(f"  🔷 호모그래피 목표: ({self.target_robot_homo[0]:.2f}, {self.target_robot_homo[1]:.2f})")
                    self.add_log(f"     오차: {error_homo:.2f} mm (ΔX={dx_homo:+.2f}, ΔY={dy_homo:+.2f})")
                    self.add_log(f"")
                    self.add_log(f"  📐 선형변환 목표:   ({self.target_robot_linear[0]:.2f}, {self.target_robot_linear[1]:.2f})")
                    self.add_log(f"     오차: {error_linear:.2f} mm (ΔX={dx_linear:+.2f}, ΔY={dy_linear:+.2f})")
                    self.add_log(f"")
                    
                    # 승자 판정
                    if error_homo < error_linear:
                        winner = "🔷 호모그래피 승!"
                        better_error = error_homo
                        self.add_log(f"  🏆 {winner} ({error_homo:.2f}mm < {error_linear:.2f}mm)")
                    elif error_linear < error_homo:
                        winner = "📐 선형변환 승!"
                        better_error = error_linear
                        self.add_log(f"  🏆 {winner} ({error_linear:.2f}mm < {error_homo:.2f}mm)")
                    else:
                        winner = "무승부"
                        better_error = error_homo
                        self.add_log(f"  🤝 무승부!")
                    
                    self.add_log("=" * 50)
                    
                    # ===== 학습 데이터 추가 (실제 위치 사용) =====
                    self.calibration_points.append((self.target_pixel, (actual_x, actual_y)))
                    self.errors.append(better_error)
                    
                    # 호모그래피 재계산 (5개 이상일 때)
                    if len(self.calibration_points) >= 5:
                        self.recalculate_homography()
                    
                    # 통계 업데이트
                    self.update_stats()
                    
                    # Home 복귀 제안
                    self.add_log("💡 🏠 Home 복귀 후 다음 위치 테스트하세요")
                
                else:
                    self.add_log(f"❌ 좌표 읽기 실패: {response.message}")
            
            except Exception as e:
                self.add_log(f"❌ 좌표 읽기 실패: {e}")
                import traceback
                self.add_log(traceback.format_exc())
        
        future.add_done_callback(callback)
    
    def recalculate_homography(self):
        """
        새로운 데이터로 호모그래피 재계산
        ✅ 초기 4개 코너는 항상 포함하여 안정성 확보
        """
        if len(self.calibration_points) < 1:
            return
        
        # ===== ✅ 초기 4개 코너 + 새로운 측정값 결합 =====
        all_pixels = list(self.initial_corners_pixel)
        all_robots = list(self.initial_corners_robot)
        
        for pixel, robot in self.calibration_points:
            all_pixels.append(pixel)
            all_robots.append(robot)
        
        pixels = np.array(all_pixels, dtype=np.float32)
        robots = np.array(all_robots, dtype=np.float32)
        
        # 호모그래피 재계산
        self.H, _ = cv2.findHomography(pixels, robots)
        
        self.add_log(f"🧮 호모그래피 재계산 완료!")
        self.add_log(f"   초기 4개 코너 + 추가 {len(self.calibration_points)}개 = 총 {len(all_pixels)}개 점 사용")
    
    def update_stats(self):
        """통계 업데이트"""
        if len(self.errors) == 0:
            return
        
        avg_error = np.mean(self.errors)
        max_error = np.max(self.errors)
        min_error = np.min(self.errors)
        
        stats_text = (
            f"📊 통계:\n"
            f"  테스트 횟수: {len(self.errors)}\n"
            f"  평균 오차: {avg_error:.2f} mm\n"
            f"  최대 오차: {max_error:.2f} mm\n"
            f"  최소 오차: {min_error:.2f} mm\n"
            f"\n"
            f"  데이터 포인트: {len(self.calibration_points)}개"
        )
        
        self.stats_label.setText(stats_text)
    
    def send_robot_home(self):
        """로봇 Home 이동"""
        try:
            # ===== ✅ calibration_move_service 사용 =====
            if not self.calibration_move_cli.wait_for_service(timeout_sec=2.0):
                self.add_log("❌ calibration_move_service 연결 안됨!")
                return
            
            # 안전한 Home 위치로 이동만
            safe_x, safe_y, z = 200.0, 0.0, 250.0
            xy_str = f"{safe_x:.2f},{safe_y:.2f}"
            final_str = f"['{xy_str}', '0.0', 'h', '{z:.2f}']"
            
            self.add_log(f"🏠 Home 위치로 이동 중...")
            
            from mycobot_interfaces.srv import Move
            req = Move.Request()
            req.result = final_str
            
            future = self.calibration_move_cli.call_async(req)
            
            def home_callback(future):
                try:
                    response = future.result()
                    if response.success:
                        self.add_log(f"✅ Home 도착")
                    else:
                        self.add_log(f"⚠️ {response.feedback}")
                except Exception as e:
                    self.add_log(f"❌ Home 이동 에러: {e}")
            
            future.add_done_callback(home_callback)
            
        except Exception as e:
            self.add_log(f"❌ send_robot_home 에러: {e}")
            import traceback
            self.add_log(traceback.format_exc())
    
    def save_calibration(self):
        """캘리브레이션 저장"""
        if len(self.calibration_points) < 4:
            self.add_log("⚠️ 최소 4개 이상의 테스트가 필요합니다")
            return
        
        # 저장
        save_data = {
            "calibration_points": [
                {"pixel": list(p), "robot": list(r)}
                for p, r in self.calibration_points
            ],
            "errors": self.errors,
            "homography": self.H.tolist()
        }
        
        save_path = Path.home() / "interactive_calibration.json"
        with open(save_path, 'w') as f:
            json.dump(save_data, f, indent=2)
        
        self.add_log(f"💾 저장 완료: {save_path}")
        
        # get_coord.py 코드 생성
        pixels = np.array([p[0] for p in self.calibration_points[-4:]], dtype=np.float32)
        robots = np.array([p[1] for p in self.calibration_points[-4:]], dtype=np.float32)
        
        self.add_log("\n📋 get_coord.py 업데이트 코드:")
        self.add_log("=" * 50)
        self.add_log("pts_pixel = np.array([")
        for p in pixels:
            self.add_log(f"    [{p[0]:.1f}, {p[1]:.1f}],")
        self.add_log("], dtype=np.float32)\n")
        
        self.add_log("pts_robot = np.array([")
        for r in robots:
            self.add_log(f"    [{r[0]:.1f}, {r[1]:.1f}],")
        self.add_log("], dtype=np.float32)")
        self.add_log("=" * 50)
    
    def reset_calibration(self):
        """초기화"""
        self.calibration_points = []
        self.errors = []
        self.target_pixel = None
        self.actual_pixel = None
        self.state = "IDLE"
        self.status_label.setText("🟢 준비됨: ROI 내 클릭하세요")
        self.stats_label.setText("📊 통계:\n  테스트 횟수: 0\n  평균 오차: -")
        self.add_log("🔄 초기화 완료")
    
    def add_log(self, message):
        self.log_display.append(message)
        self.log_display.verticalScrollBar().setValue(
            self.log_display.verticalScrollBar().maximum()
        )
    
    def spin_with_gui(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.app.processEvents()


class ClickableLabel(QLabel):
    """클릭 가능한 QLabel"""
    clicked = pyqtSignal(QPoint)  # ✅ 클래스 레벨에서 시그널 정의
    
    def __init__(self):
        super().__init__()
    
    def mousePressEvent(self, event):
        """마우스 클릭 이벤트"""
        self.clicked.emit(event.pos())


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = InteractiveCalibration(app)
    
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