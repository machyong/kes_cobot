#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QTextEdit, QMessageBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import sys
import cv2
import numpy as np
import json
from pathlib import Path

class CalibrationGUI(Node):
    def __init__(self, app):
        super().__init__('calibration_gui')
        self.bridge = CvBridge()
        self.app = app
        
        # ROI 설정
        self.roi_x = 440
        self.roi_y = 280
        self.roi_w = 380
        self.roi_h = 220
        
        # 캘리브레이션 포인트 (픽셀 좌표)
        self.calibration_points_pixel = [
            (self.roi_x, self.roi_y),                           # 좌상단
            (self.roi_x, self.roi_y + self.roi_h),             # 좌하단
            (self.roi_x + self.roi_w, self.roi_y + self.roi_h), # 우하단
            (self.roi_x + self.roi_w, self.roi_y)              # 우상단
        ]
        
        self.point_names = ["좌상단", "좌하단", "우하단", "우상단"]
        
        # 로봇 좌표 저장
        self.calibration_points_robot = [None, None, None, None]
        self.current_point_index = 0
        
        self.latest_image = None
        
        # ===== 로봇 서비스 클라이언트 =====
        self.get_coords_cli = self.create_client(Trigger, '/calibration/get_robot_coords')
        self.move_to_cli = self.create_client(Trigger, '/calibration/move_to_point')
        
        # 서비스 대기
        self.get_logger().info("로봇 서비스 대기 중...")
        if self.get_coords_cli.wait_for_service(timeout_sec=5.0):
            self.robot_connected = True
            self.get_logger().info("✅ 로봇 서비스 연결 성공")
        else:
            self.robot_connected = False
            self.get_logger().warn("⚠️ 로봇 서비스 연결 실패 (mycobot에서 calibration_robot.py 실행 필요)")
        
        # 이미지 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # ===== GUI 구성 (이전과 동일) =====
        self.window = QWidget()
        self.window.setWindowTitle("🎯 캘리브레이션 어시스턴트")
        self.window.setGeometry(100, 100, 1200, 800)
        
        main_layout = QHBoxLayout()
        
        # 왼쪽: 카메라 뷰
        left_layout = QVBoxLayout()
        
        self.camera_label = QLabel("카메라 대기 중...")
        self.camera_label.setMinimumSize(960, 720)
        self.camera_label.setStyleSheet("border: 2px solid #FF5722; background-color: #000;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.camera_label)
        
        # 오른쪽: 컨트롤 패널
        right_layout = QVBoxLayout()
        
        title = QLabel("🎯 캘리브레이션")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #FF5722;")
        right_layout.addWidget(title)
        
        instructions = QLabel(
            "📋 절차:\n"
            "1. '자동 이동' 또는 수동으로 각 코너 이동\n"
            "2. 위치 조정 후 '좌표 기록' 클릭\n"
            "3. 4개 점 완료 후 '계산' 클릭\n"
            "4. get_coord.py에 코드 복사"
        )
        instructions.setStyleSheet("font-size: 13px; padding: 10px; background-color: #FFF3E0; border-radius: 5px;")
        right_layout.addWidget(instructions)
        
        # 연결 상태
        self.connection_label = QLabel()
        self.update_connection_status()
        right_layout.addWidget(self.connection_label)
        
        # 진행 상태
        self.status_label = QLabel("대기 중...")
        self.status_label.setStyleSheet("""
            font-size: 16px; font-weight: bold; padding: 15px;
            background-color: #263238; color: #00E676; border-radius: 5px;
        """)
        right_layout.addWidget(self.status_label)
        
        # 로그
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setStyleSheet("""
            background-color: #1E1E1E; color: #E0E0E0;
            border: 2px solid #FF5722; border-radius: 5px;
            padding: 10px; font-size: 12px;
        """)
        right_layout.addWidget(self.log_display)
        
        # 버튼들
        button_layout = QVBoxLayout()
        
        # 자동 이동 버튼
        self.auto_move_button = QPushButton("🤖 자동 이동 (4개 코너)")
        self.auto_move_button.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0; color: white; border: none;
                border-radius: 5px; padding: 12px; font-size: 14px; font-weight: bold;
            }
            QPushButton:hover { background-color: #7B1FA2; }
            QPushButton:disabled { background-color: #757575; }
        """)
        self.auto_move_button.clicked.connect(self.auto_move_corners)
        self.auto_move_button.setEnabled(self.robot_connected)
        button_layout.addWidget(self.auto_move_button)
        
        # 좌표 기록 버튼
        self.record_button = QPushButton("📍 좌표 기록")
        self.record_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5722; color: white; border: none;
                border-radius: 5px; padding: 15px; font-size: 16px; font-weight: bold;
            }
            QPushButton:hover { background-color: #E64A19; }
            QPushButton:disabled { background-color: #757575; }
        """)
        self.record_button.clicked.connect(self.record_current_position)
        self.record_button.setEnabled(self.robot_connected)
        button_layout.addWidget(self.record_button)
        
        # 계산 버튼
        self.calculate_button = QPushButton("🧮 호모그래피 계산")
        self.calculate_button.setEnabled(False)
        self.calculate_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3; color: white; border: none;
                border-radius: 5px; padding: 15px; font-size: 16px; font-weight: bold;
            }
            QPushButton:hover { background-color: #1976D2; }
            QPushButton:disabled { background-color: #757575; }
        """)
        self.calculate_button.clicked.connect(self.calculate_homography)
        button_layout.addWidget(self.calculate_button)
        
        # 리셋 버튼
        self.reset_button = QPushButton("🔄 리셋")
        self.reset_button.setStyleSheet("""
            QPushButton {
                background-color: #9E9E9E; color: white; border: none;
                border-radius: 5px; padding: 10px; font-size: 14px;
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
        
        self.update_status()
        self.add_log("🎯 캘리브레이션 GUI 시작")
        self.add_log(f"ROI: ({self.roi_x}, {self.roi_y}) ~ ({self.roi_x+self.roi_w}, {self.roi_y+self.roi_h})")
    
    def update_connection_status(self):
        if self.robot_connected:
            self.connection_label.setText("🟢 로봇 연결됨")
            self.connection_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.connection_label.setText("🔴 로봇 연결 안됨 (mycobot에서 calibration_robot.py 실행)")
            self.connection_label.setStyleSheet("color: #F44336; font-weight: bold;")
    
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
        
        # 캘리브레이션 포인트
        for i, (px, py) in enumerate(self.calibration_points_pixel):
            if i < self.current_point_index:
                # 완료
                color = (0, 255, 0)
                cv2.circle(display_img, (px, py), 15, color, -1)
                cv2.putText(display_img, str(i+1), (px-8, py+8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            elif i == self.current_point_index:
                # 현재
                color = (0, 0, 255)
                radius = 15 + int(5 * abs(np.sin(rclpy.clock.Clock().now().nanoseconds / 1e9 * 3)))
                cv2.circle(display_img, (px, py), radius, color, 3)
                cv2.putText(display_img, f"{i+1}. {self.point_names[i]}", 
                           (px+20, py),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.drawMarker(display_img, (px, py), color, 
                             cv2.MARKER_CROSS, 30, 3)
            else:
                # 대기
                color = (128, 128, 128)
                cv2.circle(display_img, (px, py), 10, color, 2)
                cv2.putText(display_img, str(i+1), (px-6, py+6),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # 표시
        rgb_image = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        scaled = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_label.setPixmap(scaled)
    
    def auto_move_corners(self):
        """자동으로 4개 코너 순회"""
        if not self.robot_connected:
            self.add_log("❌ 로봇이 연결되지 않았습니다")
            return
        
        self.add_log("🤖 자동 이동 시작...")
        req = Trigger.Request()
        future = self.move_to_cli.call_async(req)
        future.add_done_callback(self.auto_move_callback)
    
    def auto_move_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.add_log("✅ 자동 이동 완료")
            else:
                self.add_log(f"❌ 이동 실패: {response.message}")
        except Exception as e:
            self.add_log(f"❌ 에러: {e}")
    
    def record_current_position(self):
        """현재 로봇 좌표 기록"""
        if not self.robot_connected:
            self.add_log("❌ 로봇이 연결되지 않았습니다")
            return
        
        if self.current_point_index >= 4:
            self.add_log("⚠️ 모든 점이 이미 기록되었습니다")
            return
        
        # 로봇에게 좌표 요청
        req = Trigger.Request()
        future = self.get_coords_cli.call_async(req)
        future.add_done_callback(self.record_callback)
    
    def record_callback(self, future):
        try:
            response = future.result()
            
            if response.success:
                # "X,Y" 파싱
                coords_str = response.message
                x_str, y_str = coords_str.split(',')
                x, y = float(x_str), float(y_str)
                
                self.calibration_points_robot[self.current_point_index] = (x, y)
                
                px, py = self.calibration_points_pixel[self.current_point_index]
                name = self.point_names[self.current_point_index]
                
                self.add_log(f"✅ {name} 기록: 픽셀({px}, {py}) → 로봇({x:.2f}, {y:.2f})")
                
                self.current_point_index += 1
                self.update_status()
                
                if self.current_point_index >= 4:
                    self.calculate_button.setEnabled(True)
                    self.add_log("🎉 모든 점 기록 완료! '호모그래피 계산' 클릭하세요")
            else:
                self.add_log(f"❌ 좌표 읽기 실패: {response.message}")
        
        except Exception as e:
            self.add_log(f"❌ 에러: {e}")
    
    def calculate_homography(self):
        """호모그래피 계산"""
        try:
            pts_pixel = np.array(self.calibration_points_pixel, dtype=np.float32)
            pts_robot = np.array(self.calibration_points_robot, dtype=np.float32)
            
            H, status = cv2.findHomography(pts_pixel, pts_robot)
            
            self.add_log("=" * 50)
            self.add_log("🧮 호모그래피 행렬 계산 완료!")
            self.add_log("=" * 50)
            
            self.add_log("\n📊 픽셀 좌표:")
            for i, (px, py) in enumerate(self.calibration_points_pixel):
                self.add_log(f"  {self.point_names[i]}: ({px}, {py})")
            
            self.add_log("\n🤖 로봇 좌표:")
            for i, (rx, ry) in enumerate(self.calibration_points_robot):
                self.add_log(f"  {self.point_names[i]}: ({rx:.2f}, {ry:.2f})")
            
            # 파일 저장
            calibration_data = {
                "pixels": self.calibration_points_pixel,
                "robot": [[float(x), float(y)] for x, y in self.calibration_points_robot],
                "homography": H.tolist()
            }
            
            save_path = Path.home() / "calibration_result.json"
            with open(save_path, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            self.add_log(f"\n💾 저장됨: {save_path}")
            self.add_log("\n📋 get_coord.py에 복사할 코드:")
            self.add_log("=" * 50)
            self.add_log("pts_pixel = np.array([")
            for px, py in self.calibration_points_pixel:
                self.add_log(f"    [{float(px)}, {float(py)}],")
            self.add_log("], dtype=np.float32)\n")
            
            self.add_log("pts_robot = np.array([")
            for rx, ry in self.calibration_points_robot:
                self.add_log(f"    [{rx:.1f}, {ry:.1f}],")
            self.add_log("], dtype=np.float32)")
            self.add_log("=" * 50)
            
        except Exception as e:
            self.add_log(f"❌ 계산 실패: {e}")
    
    def reset_calibration(self):
        reply = QMessageBox.question(self.window, '확인', 
                                    '캘리브레이션을 리셋하시겠습니까?',
                                    QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.current_point_index = 0
            self.calibration_points_robot = [None, None, None, None]
            self.calculate_button.setEnabled(False)
            self.update_status()
            self.add_log("🔄 리셋 완료")
    
    def update_status(self):
        if self.current_point_index < 4:
            name = self.point_names[self.current_point_index]
            self.status_label.setText(f"📍 {self.current_point_index+1}/4: {name} 위치로 이동 후 기록")
        else:
            self.status_label.setText("✅ 모든 점 기록 완료!")
    
    def add_log(self, message):
        self.log_display.append(message)
        self.log_display.verticalScrollBar().setValue(
            self.log_display.verticalScrollBar().maximum()
        )
    
    def spin_with_gui(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.app.processEvents()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = CalibrationGUI(app)
    
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