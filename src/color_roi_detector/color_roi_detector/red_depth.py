import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge

# ================================
# 설정 상수
# ================================
DEPTH_TOPIC = '/camera/camera/depth/image_rect_raw'     # Depth 이미지 토픽
CAMERA_INFO_TOPIC = '/camera/camera/depth/camera_info'  # CameraInfo 토픽
MAX_DEPTH_METERS = 5.0         # 시각화 시 최대 깊이 값 (m)
NORMALIZE_DEPTH_RANGE = 3.0    # 시각화 정규화 범위 (m)
# ================================


class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.bridge = CvBridge()
        self.K = None
        self.should_exit = False

        # 블록 좌표 구독
        self.create_subscription(String, '/red_block', self.block_callback, 10)

        # Depth 이미지 구독
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)

        # CameraInfo 구독
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)

        # 최신 블록 좌표 저장
        self.x = 0
        self.y = 0

    def block_callback(self, msg: String):
        """
        예시: "Center: (259, 161)/ Angle: 137.1"
        → (259, 161) 좌표 파싱
        """
        try:
            text = msg.data
            coord_text = text.split('/')[0]                # "Center: (259, 161)"
            xy_str = coord_text.replace("Center:", "").strip()
            xy_str = xy_str.strip("()")                    # "259, 161"
            x_str, y_str = xy_str.split(',')
            x, y = int(x_str), int(y_str)

            self.latest_block_xy = (x, y)
            self.x = x
            self.y = y
            # self.get_logger().info(f"Received block center: ({x}, {y})")
        except Exception as e:
            self.get_logger().error(f"Failed to parse block msg: {e}")

    def camera_info_callback(self, msg):
        """카메라 내부 파라미터 저장"""
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f"CameraInfo received: fx={self.K[0,0]:.2f}, "
                f"fy={self.K[1,1]:.2f}, "
                f"cx={self.K[0,2]:.2f}, "
                f"cy={self.K[1,2]:.2f}"
            )

    def depth_callback(self, msg):
        if self.should_exit:
            return

        if self.K is None:
            self.get_logger().warn('Waiting for CameraInfo...')
            return

        # depth_image: uint16 or float32 in mm
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = depth_mm.shape

        cx = self.K[0, 2]
        cy = self.K[1, 2]
        u, v = self.x, self.y
        x, y, w, h = 411, 2, 560, 315
        distance_mm = depth_mm[v+2, u+411]
        distance_m = distance_mm / 1000.0  # mm → m

        self.get_logger().info(f"Image size: {width}x{height}, Distance at (u={u}, v={v}) = {distance_m:.2f} meters")

        # 시각화용 정규화 (mm → m 고려)
        depth_vis = np.nan_to_num(depth_mm, nan=0.0)
        depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)  # mm
        depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)

        # 컬러맵 적용
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # ROI 자르기
        x, y, w, h = 411, 2, 560, 315
        # roi = depth_colored[y:y+h, x:x+w]

        # 중심점 시각화 (ROI 안에서만 보이도록 처리 가능)
        cv2.circle(depth_colored, (u + x, v + y), 5, (0, 0, 0), -1)
        cv2.line(depth_colored, (0, v - y), (w, v - y), (0, 0, 0), 1)
        cv2.line(depth_colored, (u - x, 0), (u - x, h), (0, 0, 0), 1)

        # ROI 출력
        cv2.imshow('Depth Image ROI with Center Mark', depth_colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.should_exit = True



def main():
    rclpy.init()
    node = DepthChecker()

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
