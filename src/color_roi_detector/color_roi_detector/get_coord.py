#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mycobot_interfaces.srv import Move
import numpy as np
import json
import re

class ChatClient(Node):
    def __init__(self):
        super().__init__('chat_client')

        # ===== 새로운 ROI 설정 =====
        self.roi_x = 440
        self.roi_y = 280
        self.roi_w = 380
        self.roi_h = 220
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("좌표 변환 시스템 (1차 함수 기반)")
        self.get_logger().info(f"  ROI: ({self.roi_x}, {self.roi_y}, {self.roi_w}, {self.roi_h})")
        
        # ===== 캘리브레이션 데이터 =====
        # 이미지 좌표 → 로봇 좌표
        # 좌상단: (461, 300) → (210, -120, 165)
        # 좌하단: (460, 482) → (310, -150, 175)
        # 우상단: (803, 297) → (190, 150, 165)
        # 우하단: (803, 486) → (310, 150, 175)
        # 중앙: (637, 391) → (260, 10, 170)
        
        # ===== 1) 이미지 X → 로봇 Y 변환 (1차 함수) =====
        # 상단: (461, -120), (803, 150)
        # Y = a_y * u + b_y
        u1, y1 = 461, -120
        u2, y2 = 803, 150
        self.a_y = (y2 - y1) / (u2 - u1)  # ≈ 0.789
        self.b_y = y1 - self.a_y * u1
        
        # ===== 2) 이미지 Y → 로봇 X 변환 (1차 함수) =====
        # 좌측: (300, 210), (482, 310)
        # X = a_x * v + b_x
        v1, x1 = 300, 210
        v2, x2 = 482, 310
        self.a_x = (x2 - x1) / (v2 - v1)  # ≈ 0.549
        self.b_x = x1 - self.a_x * v1
        
        self.get_logger().info(f"  변환 계수:")
        self.get_logger().info(f"    이미지 X → 로봇 Y: Y = {self.a_y:.3f} * u + {self.b_y:.1f}")
        self.get_logger().info(f"    이미지 Y → 로봇 X: X = {self.a_x:.3f} * v + {self.b_x:.1f}")
        
        # ===== 로봇 X 경계값 =====
        self.x_theta_boundary = 300.0  # 이 값 이상이면 각도 반전
        self.x_z_low = 260.0           # z=165 영역
        self.x_z_high = 290.0          # z=175 영역
        
        self.get_logger().info(f"  각도 전환 경계: X ≥ {self.x_theta_boundary}")
        self.get_logger().info(f"  Z값 범위: X<{self.x_z_low}→165mm, X≥{self.x_z_high}→175mm")
        self.get_logger().info("=" * 60)

        # 서비스 클라이언트
        self.chat_cli = self.create_client(Trigger, 'chat_service')
        while not self.chat_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('chat_service 대기중...')

        self.move_cli = self.create_client(Move, 'move_service')
        self.get_logger().info("ℹ️ move_service 연결은 호출 직전에 확인합니다.")

        self.get_logger().info("✅ 모든 서비스 연결 완료!")
        self.call_chat_service()

    def pixel_to_robot(self, u, v):
        """
        이미지 좌표 (u, v) → 로봇 좌표 (X, Y)
        u: 이미지 x 좌표 → 로봇 Y
        v: 이미지 y 좌표 → 로봇 X
        """
        robot_y = self.a_y * u + self.b_y
        robot_x = self.a_x * v + self.b_x
        return robot_x, robot_y

    def calculate_z(self, robot_x):
        """
        로봇 X 좌표에 따른 Z값 계산
        X < 260: z = 165
        260 ≤ X < 290: 선형 보간
        X ≥ 290: z = 175
        """
        if robot_x < self.x_z_low:
            return 165.0
        elif robot_x >= self.x_z_high:
            return 175.0
        else:
            # 선형 보간
            ratio = (robot_x - self.x_z_low) / (self.x_z_high - self.x_z_low)
            z = 165.0 + ratio * 10.0
            return z

    def adjust_theta(self, theta_detected, robot_x):
        """
        검출된 각도를 로봇 좌표와 그리퍼 방향에 맞게 조정
        theta_detected: find_angle에서 반환된 0~90도 각도
        robot_x: 로봇 X 좌표
        
        반환:
        - robot_x < 300: 90~180도 범위 (180도가 정면)
        - robot_x ≥ 300: -45~45도 범위 (0도가 정면)
        """
        if robot_x < self.x_theta_boundary:
            # 정면 각도 (90~180도)
            # 0도 → 90도, 90도 → 180도
            if theta_detected > 45:
                theta_robot = 180.0 - theta_detected
            else:
                theta_robot = 180.0 + theta_detected
            
            self.get_logger().info(f"  각도 변환: {theta_detected:.1f}° → {theta_robot:.1f}° (정면 모드)")
        else:
            # 뒷면 각도 (-45~45도)
            # 0도 → -45도, 90도 → 45도
            if theta_detected > 45:
                theta_robot = 0 + theta_detected
            else:
                theta_robot = 0 - theta_detected
            #theta_robot = theta_detected - 45.0
            self.get_logger().info(f"  각도 변환: {theta_detected:.1f}° → {theta_robot:.1f}° (뒷면 모드)")
        
        return theta_robot

    def parse_detection_message(self, msg: str):
        """메시지 파싱 (JSON/Regex/구 포맷)"""
        raw = msg.strip()
        self.get_logger().info(f"🧩 원문 메시지: {raw}")

        # JSON 우선
        if raw.startswith("{") and raw.endswith("}"):
            try:
                d = json.loads(raw)
                cx, cy = d.get("center", [None, None])
                angle = d.get("angle", None)
                depth = d.get("depth", None)
                pick = d.get("picking", d.get("color", None))
                if None in (cx, cy, angle, depth) or pick is None:
                    raise ValueError("JSON key 부족")
                return int(round(float(cx))), int(round(float(cy))), float(angle), float(depth), str(pick)
            except Exception as e:
                self.get_logger().warn(f"JSON 파싱 실패: {e}")

        # Regex
        try:
            m_center = re.search(r'Center\s*:\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)', raw, re.I)
            m_angle = re.search(r'Angle\s*:\s*([-\d.]+)', raw, re.I)
            m_depth = re.search(r'(?:Depth|Z|Depth_mm)\s*:\s*([-\d.]+)', raw, re.I)
            m_pick = re.search(r'(?:picking|color|색상)\s*:\s*([^\s/]+)', raw, re.I)

            if m_center and m_angle and m_depth and m_pick:
                cx = int(round(float(m_center.group(1))))
                cy = int(round(float(m_center.group(2))))
                angle = float(m_angle.group(1))
                depth = float(m_depth.group(1))
                pick = m_pick.group(1)
                return cx, cy, angle, depth, pick
        except Exception as e:
            self.get_logger().warn(f"Regex 파싱 실패: {e}")

        # 구 포맷
        try:
            parts = [p.strip() for p in raw.split("/") if p.strip()]
            if len(parts) < 4:
                raise ValueError(f"Split 파트 수 부족: {len(parts)}")

            def tail(p):
                return p.split(":", 1)[-1].strip()

            center_tail = tail(parts[0]).strip("()").replace(" ", "")
            angle_tail = tail(parts[1])
            depth_tail = tail(parts[2])
            pick_tail = tail(parts[3])

            u_str, v_str = center_tail.split(",")
            u, v = int(float(u_str)), int(float(v_str))
            angle = float(angle_tail)
            depth = float(depth_tail)
            pick = pick_tail
            return u, v, angle, depth, pick
        except Exception as e:
            self.get_logger().error(f"❌ 메시지 파싱 전부 실패: {e}")
            raise

    def call_chat_service(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("📞 chat_service 호출 중...")
        req = Trigger.Request()
        future = self.chat_cli.call_async(req)
        future.add_done_callback(self.handle_chat_response)

    def handle_chat_response(self, future):
        try:
            response = future.result()
            
            if response.success:
                service_name = response.message.strip()
                self.get_logger().info(f"✅ 선택된 서비스: '{service_name}'")
                
                cli = self.create_client(Trigger, service_name)
                
                timeout_count = 0
                while not cli.wait_for_service(timeout_sec=1.0):
                    timeout_count += 1
                    if timeout_count >= 5:
                        self.get_logger().error(f"❌ {service_name} 타임아웃!")
                        self.call_chat_service()
                        return
                
                self.get_logger().info(f"✅ {service_name} 연결 완료!")
                
                req = Trigger.Request()
                future2 = cli.call_async(req)
                future2.add_done_callback(self.handle_find_angle_response)

            else:
                self.get_logger().warn(f"⚠️ chat_service 실패: {response.message}")
                self.call_chat_service()

        except Exception as e:
            self.get_logger().error(f"❌ chat_service 예외: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.call_chat_service()

    def handle_find_angle_response(self, future):
        try:
            result = future.result()
            
            if result and result.success:
                self.get_logger().info(f"✅ find_angle 성공: {result.message}")

                # 파싱
                try:
                    u, v, theta_detected, depth_val, pick_name = self.parse_detection_message(result.message)
                except Exception:
                    self.get_logger().error("파싱 실패로 재시도 대기")
                    self.call_chat_service()
                    return

                self.get_logger().info("=" * 60)
                self.get_logger().info(f"📍 이미지 픽셀: ({u}, {v})")
                self.get_logger().info(f"🧭 검출 각도: {theta_detected:.1f}°")
                
                # ===== 1차 함수로 로봇 좌표 변환 =====
                robot_x, robot_y = self.pixel_to_robot(u, v)
                
                self.get_logger().info(f"🔄 로봇 좌표: X={robot_x:.2f}, Y={robot_y:.2f}")
                
                # ===== Z값 계산 =====
                z_pick = self.calculate_z(robot_x)
                self.get_logger().info(f"📏 Z값: {z_pick:.1f}mm (X={robot_x:.1f} 기준)")
                
                # ===== 각도 조정 =====
                theta_robot = self.adjust_theta(theta_detected, robot_x)
                
                # 색상 약자
                color_abbr_map = {
                    "빨간색": "r", "red": "r", "Red": "r",
                    "노란색": "y", "yellow": "y", "Yellow": "y",
                    "초록색": "g", "green": "g", "Green": "g",
                    "파란색": "b", "blue": "b", "Blue": "b",
                    "주황색": "o", "orange": "o", "Orange": "o"
                }
                color_abbr = color_abbr_map.get(str(pick_name), "x")
                
                # 최종 문자열
                xy_str = f"{robot_x:.2f},{robot_y:.2f}"
                final_str = f"['{xy_str}', '{theta_robot:.1f}', '{color_abbr}', '{z_pick:.2f}']"
                
                self.get_logger().info(f"🎯 robot_move로 전송: {final_str}")
                self.get_logger().info("=" * 60)

                # move_service 호출
                ready = False
                for i in range(5):
                    if self.move_cli.wait_for_service(timeout_sec=1.0):
                        ready = True
                        break

                if not ready:
                    self.get_logger().error("❌ move_service 준비 안됨!")
                    self.call_chat_service()
                    return

                move_req = Move.Request()
                move_req.result = final_str
                future3 = self.move_cli.call_async(move_req)
                future3.add_done_callback(self.handle_move_response)

            else:
                self.get_logger().warn(f"⚠️ find_angle 실패")
                self.call_chat_service()
                
        except Exception as e:
            self.get_logger().error(f"❌ find_angle 예외: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.call_chat_service()

    def handle_move_response(self, future):
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f"✅ [move_service 완료] {response.feedback}")
            else:
                self.get_logger().warn(f"⚠️ move_service 실패: {response.feedback}")
                
        except Exception as e:
            self.get_logger().error(f"❌ move_service 예외: {e}")
        
        self.call_chat_service()

    
def main(args=None):
    rclpy.init(args=args)
    node = ChatClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()