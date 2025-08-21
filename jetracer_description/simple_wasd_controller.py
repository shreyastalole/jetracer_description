#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time


class SimpleWASDController(Node):
    """
    Simplified WASD controller for basic teleoperation.
    Uses simplified differential drive model with visual steering representation.
    """
    
    # Constants
    WHEEL_RADIUS = 0.05  # 5cm wheel radius
    MAX_STEERING_ANGLE = 0.5  # Maximum visual steering angle in radians
    
    def __init__(self):
        super().__init__('simple_wasd_controller')
        
        # Parameters
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('wheel_radius', self.WHEEL_RADIUS)
        self.declare_parameter('max_steering_angle', self.MAX_STEERING_ANGLE)
        
        self.update_rate = self.get_parameter('update_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Joint positions (initialize as dict for cleaner code)
        self.wheel_positions = {joint: 0.0 for joint in [
            'front_wheel_left_joint', 'front_wheel_right_joint',
            'rear_wheel_left_joint', 'rear_wheel_right_joint'
        ]}
        self.steering_angle = 0.0
        
        # Publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_robot)
        
        self.get_logger().info('Simple WASD Controller Started!')
        self.get_logger().info('Use teleop_twist_keyboard to control:')
        self.get_logger().info('  ros2 run teleop_twist_keyboard teleop_twist_keyboard')
    
    def cmd_vel_callback(self, msg: Twist) -> None:
        """Handle incoming cmd_vel messages with improved steering calculation."""
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        
        # Calculate visual steering angle based on angular velocity
        # Use a more realistic relationship for visual representation
        if abs(self.current_angular) > 0.01:
            # Simple relationship: steering angle proportional to angular velocity
            # with velocity-dependent scaling for more realistic visualization
            velocity_factor = max(0.1, abs(self.current_linear)) 
            self.steering_angle = self.current_angular / velocity_factor * 0.3
        else:
            self.steering_angle = 0.0
        
        # Clamp steering angle
        self.steering_angle = max(-self.max_steering_angle, 
                                 min(self.max_steering_angle, self.steering_angle))
    
    def update_robot(self) -> None:
        """Update robot state and publish joint states and odometry."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Skip update if dt is too large (e.g., during startup)
        if dt > 0.1:
            return
        
        # Update robot pose using differential drive model
        self.x += self.current_linear * math.cos(self.theta) * dt
        self.y += self.current_linear * math.sin(self.theta) * dt
        self.theta += self.current_angular * dt
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Update wheel positions
        wheel_angular_vel = self.current_linear / self.wheel_radius
        
        for wheel in self.wheel_positions:
            self.wheel_positions[wheel] += wheel_angular_vel * dt
            # Keep wheel angles bounded to prevent overflow
            if abs(self.wheel_positions[wheel]) > 2 * math.pi:
                self.wheel_positions[wheel] %= 2 * math.pi
        
        # Publish everything
        self.publish_joint_states()
        self.publish_odometry()
    
    def publish_joint_states(self) -> None:
        """Publish joint states for visualization."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # Define joint names and positions in order
        joint_names = [
            'front_steering_left', 'front_steering_right',
            'front_wheel_left_joint', 'front_wheel_right_joint',
            'rear_wheel_left_joint', 'rear_wheel_right_joint'
        ]
        
        joint_positions = [
            self.steering_angle, self.steering_angle,  # Both front wheels steer together
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
    
    def publish_odometry(self) -> None:
        """Publish odometry and TF transform."""
        current_time = self.get_clock().now()
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation using quaternion from yaw
        sin_theta_half = math.sin(self.theta / 2.0)
        cos_theta_half = math.cos(self.theta / 2.0)
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sin_theta_half
        odom.pose.pose.orientation.w = cos_theta_half
        
        # Set velocity
        odom.twist.twist.linear.x = self.current_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.current_angular
        
        self.odom_pub.publish(odom)
        
        # Broadcast TF transform from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
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
    controller = SimpleWASDController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Simple WASD Controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
