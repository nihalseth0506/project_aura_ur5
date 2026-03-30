import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from aura_s7.ur5_kinematics import jacobian_ur5, forward_kinematics_ur5
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class AuraController(Node):

    def __init__(self):
        super().__init__('aura_controller')

        self.error_integral = np.zeros(3)

        self.initial_reached = False

        # 🔥 Set initial joint configuration (matches MATLAB start position)
        self.q = np.array([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])
        self.dt = 0.02

        self.q_min = np.array([-2*np.pi]*6)
        self.q_max = np.array([2*np.pi]*6)

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 1)
        self.ee_pub = self.create_publisher(PoseStamped, '/end_effector', 1)
        self.path_pub = self.create_publisher(Path, '/ee_path', 1)

        # 🔥 READY SIGNAL publisher
        self.ready_pub = self.create_publisher(Float64MultiArray, '/controller_ready', 1)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        self.ee_log = []

        # Subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/trajectory',
            self.trajectory_callback,
            1
        )

        self.get_logger().info("Controller Node Started")

    def trajectory_callback(self, msg):

        data = msg.data

        # =========================
        # 🔥 FREEZE FIRST POINT
        # =========================
        if not hasattr(self, 'first_point'):
            self.first_point = np.array(data[0:3])

        # Desired
        if not self.initial_reached:
            x_desired = self.first_point
            x_dot_desired = np.zeros(3)
        else:
            x_desired = np.array(data[0:3])
            x_dot_desired = np.array(data[3:6])

        # Current state
        x_current = forward_kinematics_ur5(self.q)

        # =========================
        # PATH VISUALIZATION
        # =========================
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x_current[0])
        pose.pose.position.y = float(x_current[1])
        pose.pose.position.z = float(x_current[2])

        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        # =========================
        # CONTROL LAW
        # =========================
        Kp = 0.8

        error = x_desired - x_current
        error_norm = np.linalg.norm(error)

        # =========================
        # 🔥 USE PI ONLY FOR INITIAL ALIGNMENT
        # =========================
        if not self.initial_reached:
            Ki = 0.3
            self.error_integral += error * self.dt
            self.error_integral = np.clip(self.error_integral, -0.5, 0.5)

            v_control = Kp * error + Ki * self.error_integral

        else:
            # PURE TRACKING (NO INTEGRAL)
            v_control = x_dot_desired + Kp * error

        # =========================
        # JACOBIAN
        # =========================
        J = jacobian_ur5(self.q)
        J_pos = J[0:3, :]

        lambda_reg = 0.1
        J_pinv = J_pos.T @ np.linalg.inv(J_pos @ J_pos.T + lambda_reg * np.eye(3))

        # =========================
        # 🔥 INITIAL HOLD LOGIC
        # =========================
        if not self.initial_reached:
            # Ignore velocity, go only to first point
            v_control = Kp * error

            if error_norm < 0.005:
                self.initial_reached = True
                self.get_logger().info("Initial position reached!")

        # Normal tracking after alignment
        if error_norm < 0.001:
            q_dot = np.zeros_like(self.q)
        else:
            q_dot = J_pinv @ v_control

        # =========================
        # 🔥 VELOCITY SMOOTHING
        # =========================
        if not hasattr(self, 'q_dot_prev'):
            self.q_dot_prev = np.zeros(6)

        alpha = 0.7

        q_dot = alpha * self.q_dot_prev + (1 - alpha) * q_dot
        self.q_dot_prev = q_dot

        # =========================
        # LIMITS
        # =========================
        q_dot = np.clip(q_dot, -0.6, 0.6)

        # =========================
        # INTEGRATION
        # =========================
        self.q = self.q + q_dot * self.dt
        self.q = np.clip(self.q, self.q_min, self.q_max)

        # =========================
        # PUBLISH STATES
        # =========================
        joint_msg = JointState()
        joint_msg.position = self.q.tolist()
        joint_msg.velocity = q_dot.tolist()
        self.joint_pub.publish(joint_msg)

        ee_msg = PoseStamped()
        ee_msg.header.frame_id = "map"
        ee_msg.pose.position.x = float(x_current[0])
        ee_msg.pose.position.y = float(x_current[1])
        ee_msg.pose.position.z = float(x_current[2])
        self.ee_pub.publish(ee_msg)

        # =========================
        # 🔥 SEND READY ONLY AFTER INITIAL ALIGNMENT
        # =========================
        if self.initial_reached:
            ready_msg = Float64MultiArray()
            ready_msg.data = [1.0]
            self.ready_pub.publish(ready_msg)


        # =========================
        # 🔥 LOG ONLY AFTER START
        # =========================
        if self.initial_reached:
            self.ee_log.append(x_current.tolist())


        if len(self.ee_log) > 1000:   # or use index condition
            np.savetxt('/media/sf_aura_shared/data_logs/ros_executed.csv',
                    np.array(self.ee_log),
                    delimiter=',')
            self.get_logger().info("ROS trajectory saved!")

def main(args=None):
    rclpy.init(args=args)

    node = AuraController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()