import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class AuraPlanner(Node):

    def __init__(self):
        super().__init__('aura_planner')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/trajectory',
            10
        )

        self.timer = self.create_timer(0.02, self.publish_trajectory)  # 50 Hz

        # Example trajectory (replace later with MATLAB output)
        self.t = 0.0

    def publish_trajectory(self):
        msg = Float64MultiArray()

        # Simple circular trajectory (placeholder)
        x = 0.5 * np.cos(self.t)
        y = 0.5 * np.sin(self.t)
        z = 0.3

        vx = -0.5 * np.sin(self.t)
        vy = 0.5 * np.cos(self.t)
        vz = 0.0

        msg.data = [x, y, z, vx, vy, vz]

        self.publisher_.publish(msg)

        self.t += 0.02


def main(args=None):
    rclpy.init(args=args)
    node = AuraPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()