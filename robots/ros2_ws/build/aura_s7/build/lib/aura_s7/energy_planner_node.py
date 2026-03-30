import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np


class EnergyPlannerNode(Node):

    def __init__(self):
        super().__init__('energy_planner_node')

        self.initial_phase = True

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/trajectory',
            1
        )

        # 🔥 READY flag
        self.ready = True

        self.create_subscription(
            Float64MultiArray,
            '/controller_ready',
            self.ready_callback,
            1
        )

        # Load CSV
        file_path = "/media/sf_aura_shared/data_logs/trajectory.csv"
        self.trajectory = np.loadtxt(file_path, delimiter=',')

        self.N = self.trajectory.shape[0]
        self.index = 0

        # Timer (small tick, NOT time control)
        self.timer = self.create_timer(0.001, self.timer_callback)

        self.get_logger().info("SYNCHRONIZED Trajectory Publisher Started")

    def ready_callback(self, msg):

        # If still in init → switch to trajectory mode
        if self.initial_phase:
            self.initial_phase = False
            self.index = 0   # start from beginning
            self.get_logger().info("Starting trajectory tracking!")

        self.ready = True

    def timer_callback(self):

        if self.index >= self.N:
            return

        # =========================
        # 🔥 PHASE 1: SEND ONLY FIRST POINT
        # =========================
        if self.initial_phase:
            row = self.trajectory[0]   # ONLY first point

            msg = Float64MultiArray()
            msg.data = row.tolist()
            self.publisher_.publish(msg)

            return   # keep sending same point

        # =========================
        # 🔥 PHASE 2: NORMAL TRAJECTORY
        # =========================
        if not self.ready:
            return

        row = self.trajectory[self.index]

        msg = Float64MultiArray()
        msg.data = row.tolist()

        self.publisher_.publish(msg)

        self.index += 1
        self.ready = False


def main(args=None):
    rclpy.init(args=args)

    node = EnergyPlannerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()