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
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # Initialize 6D state and covariance
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.kf = KalmanFilter_2(initial_state, initial_covariance)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

    def publish_estimate(self):
        # Create a PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = self.kf.mu[0]
        msg.pose.pose.position.y = self.kf.mu[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = self.kf.mu[2]
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = self.kf.Sigma.flatten().tolist()

        # Publish the message
        self.publisher.publish(msg)

    def odom_callback(self, msg):
        # Extract timestep
        dt = msg.header.stamp.sec - self.last_time.sec + (msg.header.stamp.nanosec - self.last_time.nanosec) * 1e-9
        self.last_time = msg.header.stamp

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.z

        # Extract velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # Run predict() and update() of KalmanFilter_2
        self.kf.predict(u=None, dt=dt)
        self.kf.update(z=(x, y, theta, vx, vy, omega))

        # Publish estimated state
        self.publish_estimate()

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

