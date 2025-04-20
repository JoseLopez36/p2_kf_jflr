import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import numpy as np

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # Initialize 6D state and covariance
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.gt_measurement = np.zeros(6)
        self.iteration = 0
        low_proc_noise_std = np.array([0.02, 0.02, 0.02, 0.02, 0.02, 0.02]) # Chosen values
        low_obs_noise_std = np.array([0.02, 0.02, 0.01, 0.02, 0.02, 0.01])  # As in the low noise measurement
        high_proc_noise_std = np.array([0.1, 0.1, 0.05, 0.1, 0.1, 0.05])      # Chosen values
        high_obs_noise_std = np.array([0.1, 0.1, 0.05, 0.1, 0.1, 0.05])     # As in the high noise measurement
        self.kf = KalmanFilter_2(initial_state, initial_covariance, low_proc_noise_std, low_obs_noise_std)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            'kf_estimated_pose',
            10
        )

    def publish_estimate(self):
        # Create a PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = self.kf.mu[0]
        msg.pose.position.y = self.kf.mu[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = self.kf.mu[2]
        msg.pose.orientation.w = 1.0

        # Publish the message
        self.publisher.publish(msg)

        # Log estimated position, velocity and covariance
        self.get_logger().info(f"Iteration {self.iteration} --------------------------------")
        self.get_logger().info("Estimation:")
        self.get_logger().info(f"   - x={self.kf.mu[0]:.3f} / x_gt={self.gt_measurement[0]:.3f} / e={self.kf.mu[0] - self.gt_measurement[0]:.3f}")
        self.get_logger().info(f"   - y={self.kf.mu[1]:.3f} / y_gt={self.gt_measurement[1]:.3f} / e={self.kf.mu[1] - self.gt_measurement[1]:.3f}")
        self.get_logger().info(f"   - theta={self.kf.mu[2]:.3f} / theta_gt={self.gt_measurement[2]:.3f} / e={self.kf.mu[2] - self.gt_measurement[2]:.3f}")
        self.get_logger().info(f"   - vx={self.kf.mu[3]:.3f} / vx_gt={self.gt_measurement[3]:.3f} / e={self.kf.mu[3] - self.gt_measurement[3]:.3f}")
        self.get_logger().info(f"   - vy={self.kf.mu[4]:.3f} / vy_gt={self.gt_measurement[4]:.3f} / e={self.kf.mu[4] - self.gt_measurement[4]:.3f}")
        self.get_logger().info(f"   - omega={self.kf.mu[5]:.3f} / omega_gt={self.gt_measurement[5]:.3f} / e={self.kf.mu[5] - self.gt_measurement[5]:.3f}")
        self.get_logger().info("Covariance:")
        self.get_logger().info(f"   - xx={self.kf.Sigma[0,0]:.6f}")
        self.get_logger().info(f"   - yy={self.kf.Sigma[1,1]:.6f}")
        self.get_logger().info(f"   - theta-theta={self.kf.Sigma[2,2]:.6f}")
        self.get_logger().info(f"   - vx-vx={self.kf.Sigma[3,3]:.6f}")
        self.get_logger().info(f"   - vy-vy={self.kf.Sigma[4,4]:.6f}")
        self.get_logger().info(f"   - omega-omega={self.kf.Sigma[5,5]:.6f}")

        self.iteration += 1

    def odom_callback(self, msg):
        # Extract timestep
        dt = (self.get_clock().now().to_msg().nanosec - msg.header.stamp.nanosec) * 1e-9

        # Extract position
        gt_pose = odom_to_pose2D(msg)
        gt_x, gt_y, gt_theta = gt_pose

        # Extract velocities
        gt_vx = msg.twist.twist.linear.x
        gt_vy = msg.twist.twist.linear.y
        gt_omega = msg.twist.twist.angular.z

        # Store ground truth measurement
        self.gt_measurement = np.array([gt_x, gt_y, gt_theta, gt_vx, gt_vy, gt_omega])

        # Add noise to the measurement
        low_noise_std = np.array([0.02, 0.02, 0.01, 0.02, 0.02, 0.01])
        high_noise_std = np.array([0.1, 0.1, 0.05, 0.1, 0.1, 0.05])
        measurement = generate_noisy_measurement_2(gt_pose, gt_vx, gt_vy, gt_omega, noise_std=low_noise_std)
        x, y, theta, vx, vy, omega = measurement
        
        # Run predict() and update() of KalmanFilter_2
        self.kf.predict(u=None, dt=dt)
        self.kf.update(z=(x, y, theta, vx, vy, omega))

        # Publish estimated state
        self.publish_estimate()

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

