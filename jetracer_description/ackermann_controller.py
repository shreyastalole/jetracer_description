#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import threading
import time


class AckermannController(Node):
    """
    Advanced Ackermann steering controller with proper geometric calculations.
    Provides realistic steering behavior and odometry publishing.
    """
    
    # Default physical constants
    DEFAULT_WHEELBASE = 0.25
    DEFAULT_TRACK_WIDTH = 0.3
    DEFAULT_MAX_STEERING = 0.5236  # 30 degrees
    DEFAULT_UPDATE_RATE = 50.0
    WHEEL_RADIUS = 0.05  # 5cm
    STEERING_SMOOTHING = 0.1  # Low-pass filter coefficient
    
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Initialize state variables
        self._init_state()
        
        # Set up ROS2 interfaces
        self._setup_ros_interfaces()
        
        self.get_logger().info(
            f'Ackermann Controller started: wheelbase={self.wheelbase:.2f}m, '
            f'track_width={self.track_width:.2f}m, max_steering={math.degrees(self.max_steering_angle):.1f}°'
        )
    
    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with default values."""
        self.declare_parameter('wheelbase', self.DEFAULT_WHEELBASE)
        self.declare_parameter('track_width', self.DEFAULT_TRACK_WIDTH)
        self.declare_parameter('max_steering_angle', self.DEFAULT_MAX_STEERING)
        self.declare_parameter('update_rate', self.DEFAULT_UPDATE_RATE)
    
    def _get_parameters(self) -> None:
        """Get parameter values from ROS parameter server."""
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.update_rate = self.get_parameter('update_rate').value
    
    def _init_state(self) -> None:
        """Initialize all state variables."""
        # Command values (thread-safe)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.lock = threading.Lock()
        
        # Robot pose for odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
        
        # Joint positions for smooth animation
        self.front_left_steering = 0.0
        self.front_right_steering = 0.0
        self.wheel_positions = {joint: 0.0 for joint in [
            'front_wheel_left_joint', 'front_wheel_right_joint',
            'rear_wheel_left_joint', 'rear_wheel_right_joint'
        ]}
    
    def _setup_ros_interfaces(self) -> None:
        """Set up publishers, subscribers, and timers."""
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_joint_states)
    
    def cmd_vel_callback(self, msg: Twist) -> None:
        """Thread-safe callback for cmd_vel messages."""
        with self.lock:
            self.linear_vel = msg.linear.x
            self.angular_vel = msg.angular.z
    
    def calculate_ackermann_steering(self, linear_vel: float, angular_vel: float) -> tuple[float, float]:
        """
        Calculate Ackermann steering angles for front wheels.
        
        Args:
            linear_vel: Linear velocity in m/s
            angular_vel: Angular velocity in rad/s
            
        Returns:
            Tuple of (left_steering_angle, right_steering_angle) in radians
        """
        if abs(angular_vel) < 1e-6:
            return 0.0, 0.0
        
        if abs(linear_vel) < 1e-6:
            # Pure rotation - use maximum steering
            max_angle = self.max_steering_angle
            return (max_angle if angular_vel > 0 else -max_angle,
                   -max_angle if angular_vel > 0 else max_angle)
        
        # Calculate turning radius and steering angles
        turning_radius = linear_vel / angular_vel
        half_track = self.track_width / 2
        
        if turning_radius > 0:  # Left turn
            left_steering = math.atan(self.wheelbase / (turning_radius - half_track))
            right_steering = math.atan(self.wheelbase / (turning_radius + half_track))
        else:  # Right turn
            turning_radius = abs(turning_radius)
            left_steering = -math.atan(self.wheelbase / (turning_radius + half_track))
            right_steering = -math.atan(self.wheelbase / (turning_radius - half_track))
        
        # Apply steering limits
        left_steering = max(-self.max_steering_angle, min(self.max_steering_angle, left_steering))
        right_steering = max(-self.max_steering_angle, min(self.max_steering_angle, right_steering))
        
        return left_steering, right_steering
    
    def publish_joint_states(self) -> None:
        """Main update loop: calculate states and publish joint states and odometry."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Skip large time steps (e.g., during startup)
        if dt > 0.1:
            return
        
        # Get current commands thread-safely
        with self.lock:
            linear_vel = self.linear_vel
            angular_vel = self.angular_vel
        
        # Update robot pose (odometry integration)
        self.x += linear_vel * math.cos(self.theta) * dt
        self.y += linear_vel * math.sin(self.theta) * dt
        self.theta += angular_vel * dt
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate target steering angles
        target_left_steering, target_right_steering = self.calculate_ackermann_steering(linear_vel, angular_vel)
        
        # Apply smoothing to steering angles (low-pass filter)
        self.front_left_steering += self.STEERING_SMOOTHING * (target_left_steering - self.front_left_steering)
        self.front_right_steering += self.STEERING_SMOOTHING * (target_right_steering - self.front_right_steering)
        
        # Update wheel rotations
        wheel_angular_vel = linear_vel / self.WHEEL_RADIUS
        for wheel in self.wheel_positions:
            self.wheel_positions[wheel] += wheel_angular_vel * dt
            # Keep wheel angles bounded
            if abs(self.wheel_positions[wheel]) > 2 * math.pi:
                self.wheel_positions[wheel] %= 2 * math.pi
        
        # Publish odometry and TF
        self.publish_odometry(current_time, linear_vel, angular_vel)
        
        # Publish joint states
        self._publish_joint_state_message()
    
    def _publish_joint_state_message(self) -> None:
        """Create and publish the joint state message."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # Define joint names and positions
        joint_names = [
            'front_steering_left', 'front_steering_right',
            'front_wheel_left_joint', 'front_wheel_right_joint',
            'rear_wheel_left_joint', 'rear_wheel_right_joint'
        ]
        
        joint_positions = [
            self.front_left_steering, self.front_right_steering,
            self.wheel_positions['front_wheel_left_joint'],
            self.wheel_positions['front_wheel_right_joint'],
            self.wheel_positions['rear_wheel_left_joint'],
            self.wheel_positions['rear_wheel_right_joint']
        ]
        
        joint_state.name = joint_names
        joint_state.position = joint_positions
        joint_state.velocity = [0.0] * len(joint_names)
        joint_state.effort = [0.0] * len(joint_names)
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_odometry(self, current_time: float, linear_vel: float, angular_vel: float) -> None:
        """Publish odometry message and TF transform."""
        timestamp = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        sin_theta_half = math.sin(self.theta / 2.0)
        cos_theta_half = math.cos(self.theta / 2.0)
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sin_theta_half
        odom.pose.pose.orientation.w = cos_theta_half
        
        # Set velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)
        
        # Broadcast TF transform from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = sin_theta_half
        transform.transform.rotation.w = cos_theta_half
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    controller = AckermannController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Ackermann Controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
