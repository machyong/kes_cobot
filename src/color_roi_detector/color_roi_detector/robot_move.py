#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import Move
from pymycobot import MyCobot320
import time, ast, math

def wrap_180(a): return (a + 180.0) % 360.0 - 180.0
def shortest_delta(a_from, a_to): return wrap_180(a_to - a_from)

class MoveServiceServer(Node):
    def __init__(self):
        super().__init__('robot_move')

        # 절대경로로 서비스 광고 (네임스페이스 영향 제거)
        self.srv = self.create_service(Move, '/move_service', self.handle_move)

        # 실행 환경 로그
        self.get_logger().info("="*60)
        self.get_logger().info(f"🆔 node name      : {self.get_name()}")
        self.get_logger().info(f"🗂️ namespace      : {self.get_namespace() or '/'}")
        self.get_logger().info(f"🛰️ advertise svc  : /move_service (type: mycobot_interfaces/srv/Move)")
        self.get_logger().info("="*60)

        # 하드웨어 비동기 초기화 (서비스 광고 후)
        self.mc = None
        self._init_once = False
        self._init_timer = self.create_timer(0.2, self._late_init)

        # 파라미터
        self.z_min_safe = 140.0
        self.z_safe_offset = 150.0
        self.z_ready_offset = 100.0
        self.z_approach_offset = 50.0

        self.gripper_rx = 0.0
        self.gripper_ry = 180.0
        self.HOME_RZ = 180.0
        self.ANGLE_OFFSET_DEG = 0.0

        self.MAX_ROT_STEP_DEG = 45.0
        self.ROT_STEP_DWELL = 0.35

        self.home_angles  = [90, 0, -90, 0, 90, 180]
        self.trash_angles = [130, 0, -90, 0, 90, 180]

        self.vertical_zone_center_x = 210.0
        self.vertical_zone_center_y = 0.0
        self.vertical_zone_radius   = 80.0

    def _late_init(self):
        if self._init_once:
            return
        self._init_once = True
        try:
            self.get_logger().info("⚙️ MyCobot init start...")
            # ⚠️ 포트는 환경에 맞게 수정
            self.mc = MyCobot320('/dev/ttyAMA0', 115200)
            self.mc.power_on()
            self.mc.set_gripper_mode(0)
            self.mc.send_angles(self.home_angles, 70); time.sleep(2.0)
            self.mc.set_gripper_value(100, 50, 1)
            self.get_logger().info("✅ MyCobot init done.")
        except Exception as e:
            self.get_logger().error(f"❌ MyCobot init failed: {e}")
        if self._init_timer:
            self._init_timer.cancel()

    def safe_z(self, z):
        if z < self.z_min_safe:
            self.get_logger().warn(f"⚠️ Z 보정: {z:.1f} → {self.z_min_safe:.1f}")
            return self.z_min_safe
        return z

    def is_vertical_reachable(self, x, y):
        d = math.hypot(x - self.vertical_zone_center_x, y - self.vertical_zone_center_y)
        ok = d <= self.vertical_zone_radius
        self.get_logger().info(f"🔍 수직 접근 가능? 거리 {d:.1f}mm → {'✅' if ok else '❌'}")
        return ok

    def rotate_in_place(self, x, y, z_safe, rx, ry, rz_from, rz_to):
        rz_from = wrap_180(rz_from); rz_to = wrap_180(rz_to)
        delta = shortest_delta(rz_from, rz_to)
        sign = 1.0 if delta >= 0 else -1.0
        remain = abs(delta)

        self.mc.send_coords([x, y, z_safe, rx, ry, rz_from], 45, 1); time.sleep(0.6)
        while remain > 1e-3:
            step = min(self.MAX_ROT_STEP_DEG, remain)
            rz_from = wrap_180(rz_from + sign * step)
            self.mc.send_coords([x, y, z_safe, rx, ry, rz_from], 35, 1); time.sleep(self.ROT_STEP_DWELL)
            remain -= step
        self.mc.send_coords([x, y, z_safe, rx, ry, rz_to], 30, 1); time.sleep(self.ROT_STEP_DWELL)

    def handle_move(self, req, resp):
        if self.mc is None:
            resp.success = False
            resp.feedback = "Robot not ready (HW init pending/failed)."
            return resp
        try:
            self.get_logger().info(f"📨 req: {req.result!r}")
            # 기대 형식: ['X,Y','theta','color','Z']
            lst = ast.literal_eval(req.result)
            x_str, y_str = lst[0].split(',')
            x_t, y_t = float(x_str), float(y_str)
            theta = wrap_180(float(lst[1]) + self.ANGLE_OFFSET_DEG)
            z_pick = self.safe_z(float(lst[3]) if len(lst) >= 4 else 165.0)

            z_safe  = max(z_pick + self.z_safe_offset, 300.0)
            z_ready = max(z_pick + self.z_ready_offset, 260.0)
            z_appr  = max(z_pick + self.z_approach_offset, 190.0)

            rx, ry = self.gripper_rx, self.gripper_ry
            cur = self.mc.get_coords() or [x_t, y_t, z_safe, rx, ry, self.HOME_RZ]
            cx, cy, cz, rz_now = float(cur[0]), float(cur[1]), float(cur[2]), wrap_180(float(cur[5]))
            rz_target = wrap_180(self.HOME_RZ + theta)

            self.get_logger().info("="*60)
            self.get_logger().info(f"🎯 target ({x_t:.1f},{y_t:.1f}) θ={theta:.1f}° → RZ={rz_target:.1f}°")
            self.get_logger().info(f"Z pick/approach/ready/safe = {z_pick:.1f}/{z_appr:.1f}/{z_ready:.1f}/{z_safe:.1f}")
            self.get_logger().info("="*60)

            # open gripper
            self.mc.set_gripper_value(100, 50, 1); time.sleep(0.3)

            # up to safe z
            self.mc.send_coords([cx, cy, z_safe, rx, ry, rz_now], 50, 1); time.sleep(1.0)

            # rotate in place
            self.rotate_in_place(cx, cy, z_safe, rx, ry, rz_now, rz_target); rz_now = rz_target

            # move above target
            self.mc.send_coords([x_t, y_t, z_safe, rx, ry, rz_now], 50, 1); time.sleep(2.0)

            # descend
            if self.is_vertical_reachable(x_t, y_t):
                for z in (z_ready, z_appr, z_pick):
                    self.mc.send_coords([x_t, y_t, z, rx, ry, rz_now], 30, 1); time.sleep(0.6)
            else:
                self.mc.send_coords([x_t, y_t, z_appr, rx, ry, rz_now], 35, 1); time.sleep(1.5)
                self.mc.send_coords([x_t, y_t, z_pick, rx, ry, rz_now], 20, 1); time.sleep(1.0)

            # pick
            self.mc.set_gripper_value(45, 50, 1); time.sleep(0.5)

            # up
            self.mc.send_coords([x_t, y_t, z_safe, rx, ry, rz_now], 30, 1); time.sleep(1.0)

            # drop & home
            self.mc.send_angles(self.trash_angles, 50); time.sleep(2.5)
            self.mc.set_gripper_value(100, 50, 1); time.sleep(0.6)
            self.mc.send_angles(self.home_angles, 50); time.sleep(2.0)
            self.mc.set_gripper_value(45, 50, 1); time.sleep(0.4)

            resp.success = True
            resp.feedback = "OK"
        except Exception as e:
            self.get_logger().error(f"❌ handle_move error: {e}")
            import traceback; self.get_logger().error(traceback.format_exc())
            resp.success = False
            resp.feedback = f"error: {e}"
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = MoveServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
