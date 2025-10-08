import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from pymycobot.mycobot import MyCobot
import time

class MyCobotMoveNode(Node):
    def __init__(self):
        super().__init__('test_move')

        # Initialize mycobot (replace port and baudrate if needed)
        self.mc = MyCobot('/dev/ttyAMA0', 115200)
        self.mc.power_on()
        # self.get_logger().info(self.mc.is_power_on())
        # # Target joint angles in degrees
        # self.target_joints = [0, 0, -90, 0, 90, 0]

        # self.get_logger().info('Sending joint angles...')
        # self.mc.send_angles(self.target_joints, 50)  # 50 is speed

        # Wait until movement completes
        # time.sleep(3)
        # coords = [315,-130, 200.8, 177.09, 0.33, -89.92]
        # time.sleep(3)
        # # Get current coordinates (pose)
        # coords = self.mc.get_coords()
        # self.get_logger().info(f"Current coordinates: {coords}")

        # self.target_joints = [0, 0, 90, 0, 90, 0]
        # self.get_logger().info('Sending joint angles2...')
        
        # self.mc.send_angles(self.target_joints, 50)
        # time.sleep(5)
        # self.mc.send_coords(coords, speed = 50, mode = 0)
        # time.sleep(4)
        coords2 = self.mc.get_coords()
        self.get_logger().info(f"Current coordinates: {coords2}")

        # time.sleep(5)       
        # coords2 = self.mc.get_coords()
        # self.get_logger().info(f"Current coordinates2: {coords2}")
        
def main(args=None):
    rclpy.init(args=args)
    node = MyCobotMoveNode()
    rclpy.spin_once(node, timeout_sec=1)  # just run once
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()